# efficient_data_sharing

A small Rust **library** for sharing periodically-updated data between
independently scheduled threads, plus a **binary**
(`benchmark_comunication_time`) that demonstrates and measures it: one
executor publishes a payload at a fixed rate, and several independent reader
executors each poll for the *latest* payload at their own rate, spread across
several rate tiers.

It exists to answer a concrete question: *what's the simplest correct way
to share one frequently-updated, moderately-sized value between one
producer and several independent consumers running at different rates?*
The answer implemented here is a `std::sync::RwLock`-protected slot, wrapped
in a small `Executor`/`Topic` framework general enough to grow beyond this
one demo. This README explains the framework, why a plain `RwLock` is
enough for this workload, what it costs, and when it would stop being
enough.

## The scenario

- One writer executor publishes a payload of `--topic_size` `f32` values
  (1000 by default) at `--writer_frequency` Hz (50 Hz by default).
- `--readers_num` reader executors (10 by default) each independently
  consume the latest payload at their own pace, spread across 5 fixed rate
  tiers — **30 / 60 / 120 / 210 / 300 Hz** — to represent different
  consumers (e.g. a slow logger, a mid-rate obstacle detector, a fast
  control loop). Readers are split evenly across the 5 tiers, with any
  remainder going to the lowest-frequency tiers first (e.g. 12 readers →
  3/3/2/2/2 across 30/60/120/210/300 Hz).
- Readers only ever care about *the most recently published* value. If a
  reader polls faster than the writer publishes, it will simply read the
  same value more than once — that's expected and harmless, not a bug to
  fix. There is no "have I seen this one already" tracking, by design (see
  [Design decisions](#design-decisions-and-why)).

## Architecture

The library ([src/lib.rs](src/lib.rs) and friends) is deliberately generic —
it knows nothing about LIDARs or scans. It's built from four pieces:

- **`Executor`** ([src/executor.rs](src/executor.rs)) — a trait for one
  independently scheduled participant. `init(&mut self, id: u8)` is called
  once with the executor's identity; `run(&mut self, captain: &Captain)`
  is its main loop, which should keep working until
  `captain.is_running(id)` goes false. `name()` returns a short
  human-readable label (e.g. `"Writer 0"`) used for the executor's thread
  name and in diagnostic messages.
- **`Topic`** ([src/topic.rs](src/topic.rs)) — a trait for one named, typed
  slot of shared state with exactly one authorized writer and any number of
  readers: `set_writer(executor_id)` (fails if a writer is already set),
  `write(executor_id, value)` (fails unless `executor_id` is the registered
  writer), and `read() -> Item`. `RwLockTopic<T>` is the implementation of
  it, built directly on `std::sync::RwLock`.
- **`Runner`** ([src/runner.rs](src/runner.rs)) — the single object a binary
  constructs and drives. Owns every topic (`register_topic(name, initial)`)
  and every executor (`add_executor` registers one and assigns it a unique
  id). `run_all()` spawns a thread per registered executor, each calling
  `init` then `run` against a shared `Captain`, and can be called again
  later for executors added afterward. `switch_executor(id, new_executor)`
  stops whichever executor currently owns `id`, joins its thread, and starts
  `new_executor` in its place under that same id — every other running
  executor is unaffected. `stop()` signals every executor to stop;
  `join_all()` waits for whatever is still running. `topic(name)` reads a
  topic directly, e.g. for a post-run integrity check.
- **`Captain`** ([src/captain.rs](src/captain.rs)) — owned by `Runner` and
  shared read-only (via `Arc`) with every executor as the `&Captain` passed
  into `run`. Holds every topic in the system (registered by name,
  type-erased internally so topics of different types can coexist) plus the
  run/stop signals every executor polls: a global flag and a per-id one, so
  a single executor can be stopped without affecting the rest.
  `claim_writer(topic_name, executor_id)` is the executor-facing way to claim
  a topic's writer slot: it wraps `Topic::set_writer`, but if a *different*
  executor already holds it, that's treated as a fatal misconfiguration — it
  prints one clear diagnostic naming both executors (looked up by id, from
  the names `Runner` records via `set_name` as each executor is spawned) and
  terminates the whole program immediately, instead of leaving each executor
  to detect and panic over the conflict on its own.

```
                       ┌────────────────────────┐
    WriterExecutor     │                        │ ReaderExecutor x N (30 Hz)
    ──────────────────▶│        Captain         │◀───────────────────────────
    write("shared_data")│  "shared_data" topic   │ ReaderExecutor x N (60 Hz)
    @ 50 Hz             │ (RwLockTopic<Payload>) │◀───────────────────────────
                       │                        │ ReaderExecutor x N (120/210/300 Hz)
                       └────────────────────────┘◀───────────────────────────
```

`benchmark_comunication_time`
([src/bin/benchmark_comunication_time/](src/bin/benchmark_comunication_time/))
is just one concrete use of this: `WriterExecutor` and `ReaderExecutor`
([reader_writer.rs](src/bin/benchmark_comunication_time/reader_writer.rs))
are `Executor` impls that write/read a single `RwLockTopic<Payload>`
registered under the name `"shared_data"`, where `Payload = Vec<f32>`.

`RwLockTopic<T>` ([src/topic.rs](src/topic.rs)) wraps a single
`std::sync::RwLock<T>`:

- **`write(&self, executor_id, value)`** — after checking `executor_id` is
  the registered writer, takes the lock's write guard and overwrites the
  value in place: `*self.data.write().expect(...) = value`.
- **`read(&self)`** — takes the lock's read guard and clones the value out
  before returning it, so the guard (and the lock) is released as soon as
  the clone completes rather than being held open for however long the
  caller keeps the reference.

This gives every reader a consistent, torn-free view of one full value
(never half of an old scan and half of a new one). Any number of readers
can hold the read lock concurrently; a writer takes the lock exclusively,
so it briefly excludes every reader (and vice versa) for the duration of
one `write`/`read` call.

### Why not stay lock-free?

An earlier version of this project used `crossbeam-epoch`-based epoch
reclamation instead: readers and the writer never blocked each other at
all, at the cost of one small heap allocation per write, `unsafe` code, and
a materially larger amount of code to reason about. For a 50 Hz writer and
≤300 Hz readers, a `RwLock` never shows up as a bottleneck either — this
project's actual numbers (below) leave 1000x+ headroom under both
approaches — so the simpler, dependency-free, `unsafe`-free `RwLock`
version won out. The one thing it gives up is the wait-free guarantee: a
reader here is, in principle, at the mercy of the OS scheduler preempting
a lock holder, whereas the epoch-based version's readers and writer could
never block on each other at all. If you need that guarantee (e.g. a hard
real-time control loop that cannot tolerate even a rare priority
inversion), lock-free reclamation is worth the added complexity; this
project's actual workload doesn't need it.

## Design decisions (and why)

**`Topic::read()` returns an owned clone, not a reference or a closure.**
An earlier, LIDAR-only version of this code had readers pass a
`FnOnce(&Scan) -> R` closure into `read`. Generalizing to arbitrary
executors and topics made that feel like a trap: holding a read guard open
for as long as arbitrary caller code takes to run means a slow or
long-running reader delays the writer for that whole time. So
`RwLockTopic::read` clones the value and returns it, bounding the time the
read lock is held to just the copy. For a `Payload` around a few KB at
≤300 Hz this is negligible (see [Program output](#program-output)); for
much larger payloads (a much bigger `--topic_size`) or much higher rates,
this tradeoff would be worth revisiting.

**The payload is a `Vec<f32>`, so each write allocates.**
Because `--topic_size` is a runtime value, the payload can't be a
fixed-size array baked in at compile time the way the original LIDAR-only
`Scan = [f32; 1200]` was — it has to be a `Vec<f32>`. `Topic::write` takes
its value by ownership and `RwLockTopic` has no in-place "give me my old
buffer back" swap, so the writer builds a fresh `Vec<f32>` of `topic_size`
values every tick rather than mutating a persistent buffer in place. This
gives up the earlier "no heap allocation on the hot path at all" property in
exchange for a runtime-configurable payload size — one small allocation
(4 KB at the 1000-value default) per write, which is still cheap relative
to a typical writer period (see [Program output](#program-output)).

**No "is this new?" tracking on the reader side.**
Readers deliberately do not carry a sequence number or generation counter.
Because readers run *faster* than the writer for most of the 5 rate tiers
(60/120/210/300 Hz vs. the writer's default 50 Hz), a reader will sometimes
read the same payload twice in a row. This is intentional: the requirement
for this use case is "always the latest available value," not "notify me
exactly once per new value." Adding staleness detection would be needed if
a consumer had to react only to *changes* — it isn't needed here, and adding
it anyway would be unrequested complexity.

**The writer uses sleep + busy-wait for pacing, readers use plain sleep.**
`WriterExecutor::run`
([reader_writer.rs](src/bin/benchmark_comunication_time/reader_writer.rs))
needs to hit its target rate precisely, so it sleeps for the bulk of its
interval and busy-waits (`thread::yield_now`) for the last stretch to avoid
OS scheduler granularity error. `ReaderExecutor::run` just uses
`thread::sleep(read_interval)`, which is simpler but less precise —
observed reader throughput comes in a few percent under the nominal rate at
the higher tiers. That's acceptable here because readers only need "roughly
this often," not an exact deadline; if a real consumer needed precise
timing, it would want the same hybrid technique the writer uses.

**Timing is recorded locally per executor, then merged per rate tier.**
Each writer/reader executor times its own operations into a private
`Vec<u64>` of per-op nanosecond durations — no cross-thread counters on the
hot path at all. The writer folds its samples into its own
[`Report`](src/bin/benchmark_comunication_time/report.rs) (mean, standard
deviation, max) once it stops, handed back to `main` through an
`Arc<Mutex<Option<Report>>>` set up before it's registered with the
`Runner`. Each reader instead extends a `Arc<Mutex<Vec<u64>>>` sample sink
shared with every other reader at the same target rate, once it stops -
`main` then builds one merged `Report` per non-empty rate tier from that
tier's pooled samples, so a run with e.g. 3 readers at 30 Hz prints a single
combined block rather than 3 near-identical ones. This is also why the live
progress bar can't show read/write counts: nothing is shared or aggregated
until every executor has already finished.

## When would this stop being enough?

Two independent thresholds, not one:

- **Throughput.** The write path spends a few microseconds per payload at
  the default `--topic_size 1000` — mostly generating the simulated
  waveform and allocating the `Vec<f32>`, not the lock acquisition itself
  (see [Program output](#program-output)). At the default 20 ms writer
  period that's well under 1% of budget. This scales with `--topic_size`
  and `--writer_frequency`: it would start to matter if the payload grew
  much larger, or the write rate grew into the low kHz range, where the
  period and the per-write cost become comparable.
- **Worst-case latency / determinism.** More relevant if this were ever
  driving a hard real-time loop: write latency has a long tail relative to
  its average (see [Program output](#program-output) for a representative
  average vs. max), which for an `RwLock` is the inherent risk of taking any
  lock at all — a writer can, in principle, be delayed by however long a
  reader holds the read lock, or by the OS scheduler preempting a lock
  holder. If a consumer ever needed a truly *bounded* worst case rather than
  a good average, that's the point at which the lock-free, wait-free design
  this project used previously (see
  [Why not stay lock-free?](#why-not-stay-lock-free)) would be worth the
  added complexity. At the defaults (50 Hz writer, ≤300 Hz readers, 1000
  values) this project has no such requirement - every executor reports
  comfortably `within budget` (see [Program output](#program-output)).

## Program output

A run has four parts, in order: the configuration banner, a live progress
bar, a per-executor final report, and the integrity check.

### Project layout

```
src/
  lib.rs                    crate docs, module wiring, public re-exports
  executor.rs                Executor trait
  runner.rs                  Runner struct
  topic.rs                   Topic trait, TopicError, RwLockTopic<T>
  captain.rs                 Captain struct
src/bin/benchmark_comunication_time/
  main.rs                    wires topics + executors together, prints the report
  reader_writer.rs           Payload, WriterExecutor, ReaderExecutor
  report.rs                  timing stats (possibly merged across readers) + format_block
  verifier.rs                post-run integrity check
  progress.rs                the progress bar
  cli.rs                     --topic_size/--readers_num/--writer_frequency/--duration parsing
```

### 1-2. Banner and progress bar

```
=== Reader/Writer Communication Benchmark (RwLock) ===

Configuration:
  - Topic size: 1000 f32 values (4000 bytes)
  - Writer rate: 50.0 Hz (20.0ms interval)
  - Readers: 10 threads across 30Hz=2 60Hz=2 120Hz=2 210Hz=2 300Hz=2
  - Duration: 10.0 seconds

Starting threads...

[####################--------------------]  50.1% |  5.0s / 10.0s
```

The bar (`print_progress_bar` in
[progress.rs](src/bin/benchmark_comunication_time/progress.rs)) redraws in
place with `\r` and is driven purely by wall-clock elapsed time versus the
requested duration — it doesn't read from any executor, so it stays
accurate even though (per the next section) nothing is aggregated across
executors until they've all stopped. Every line of the banner reflects the
actual `--topic_size`/`--writer_frequency`/`--readers_num`/`--duration`
values passed on the command line (see [Running it](#running-it)), not
fixed defaults.

### 3. Per-tier final report

From a representative 10-second run at every default (`--topic_size 1000
--readers_num 10 --writer_frequency 50 --duration 10`, i.e. 2 readers per
rate tier):

```
========== FINAL REPORT ==========

WRITER:
  Writer 0 - target 50 Hz (period 20.000 ms)
    writes    :   500 (  500 expected)
    avg time  :   13.34 us +-  11.78 us
    max time  :  117.40 us  (0.6% of period, within budget)

READERS:
  Readers @ 30 Hz (x2) - target 30 Hz (period 33.333 ms)
    reads     :   600 (  600 expected)
    avg time  :    3.34 us +-   5.39 us
    max time  :   98.51 us  (0.3% of period, within budget)

  Readers @ 60 Hz (x2) - target 60 Hz (period 16.667 ms)
    reads     :  1193 ( 1200 expected)
    avg time  :    2.79 us +-   4.70 us
    max time  :  124.53 us  (0.7% of period, within budget)

  Readers @ 120 Hz (x2) - target 120 Hz (period 8.333 ms)
    reads     :  2365 ( 2400 expected)
    avg time  :    2.72 us +-   3.90 us
    max time  :  118.56 us  (1.4% of period, within budget)

  Readers @ 210 Hz (x2) - target 210 Hz (period 4.762 ms)
    reads     :  4080 ( 4200 expected)
    avg time  :    3.04 us +-   3.32 us
    max time  :   30.75 us  (0.6% of period, within budget)

  Readers @ 300 Hz (x2) - target 300 Hz (period 3.333 ms)
    reads     :  5786 ( 6000 expected)
    avg time  :    3.26 us +-   3.81 us
    max time  :   96.78 us  (2.9% of period, within budget)

===================================
```

Each block ([`Report::format_block`](src/bin/benchmark_comunication_time/report.rs))
reports:

- **operations vs. expected** — actual count, and in parentheses
  `rate_hz * group_size * run_duration` (`group_size` is 1 for the writer,
  or the number of readers pooled into that tier's block), the count the
  executor(s) would hit if each ran at exactly its target rate the whole
  time. Readers consistently land a little under it (`thread::sleep`
  pacing, not a correctness issue — see
  [Design decisions](#design-decisions-and-why)); the writer's precise
  sleep+busy-wait pacing hits its target exactly.
- **avg time ± standard deviation** — computed from the pooled sample
  buffer for that block: the writer's own private samples, or every reader
  in that tier's samples merged together once they've all stopped (see the
  timing note in [Design decisions](#design-decisions-and-why)).
- **max time as % of period** — "period" is `1 / rate_hz`, the time budget
  for one operation to stay on schedule (e.g. a 300 Hz reader must finish
  each read within 3.333 ms). The max line shows the single slowest
  operation across the block as a percentage of that budget, with a
  trailing `within budget` / `OVER BUDGET` flag — the latter would mean
  that block's readers could no longer keep up even in isolation, ignoring
  contention from anything else running on the machine.

Write cost here is dominated by generating the synthetic waveform and
allocating its `Vec<f32>` (1000 values by default) rather than by
publishing it — in a real system this would be replaced by however long it
takes to read the actual producer, and the topic write itself would still
be a small, constant addition on top. The write time's tail (max ~117 µs vs.
an ~13 µs average in the run above) reflects the occasional case where the
writer's lock acquisition is delayed by a reader holding the read lock or
by OS scheduler jitter, as discussed above.

### 4. Integrity check

```
Data integrity check: PASSED ✓
```

Confirms the last published payload is non-empty and every value in it is
finite — there's no real sensor behind this benchmark, so there's no
physically-motivated range to check, just that the writer produced a sane
payload.

## Running it

```sh
cargo run --release                                    # every default
cargo run --release -- --duration 30                    # run for 30 seconds instead
cargo run --release -- --topic_size 4000 --readers_num 20
cargo run --release -- --help                           # usage
```

Prints the title and configuration, then a live progress bar for the run,
then a final report block per writer/reader-tier (operation count vs.
expected, timing mean ± standard deviation, and max time as a percentage of
that block's period budget) and the integrity check. Requires no external
services or hardware — everything is simulated in
[src/bin/benchmark_comunication_time/reader_writer.rs](src/bin/benchmark_comunication_time/reader_writer.rs).
Since the crate has exactly one binary target, `--bin
benchmark_comunication_time` isn't required, but works too: `cargo run
--release --bin benchmark_comunication_time -- --duration 30`.

All four flags are optional, named, and can be given in any order; an
unknown flag, a missing value, or a non-positive value (`--readers_num`
alone also accepts `0`, for a writer-only run) prints a usage message to
stderr and exits with status 1.

### Configuration

| Flag                  | What                                                    | Default  |
|------------------------|---------------------------------------------------------|----------|
| `--topic_size N`       | `f32` values per topic payload                          | 1000     |
| `--readers_num N`      | total reader threads, spread across 5 rate tiers (30/60/120/210/300 Hz) | 10 |
| `--writer_frequency HZ`| writer publish rate                                      | 50 Hz    |
| `--duration SECS`      | benchmark run length                                     | 10 s     |

Other constants are still edited directly in
[src/bin/benchmark_comunication_time/main.rs](src/bin/benchmark_comunication_time/main.rs)
rather than exposed as flags: `TIER_RATES_HZ` (the 5 fixed reader rates) and
`BAR_WIDTH` (the progress bar's width in characters). The title/configuration
banner at the top of `main()` is interpolated from the parsed `Config` and
the computed tier distribution, so it always reflects the flags actually
passed.

## Code documentation

Every public type and method has a rustdoc comment. Generate and browse the
HTML docs with:

```sh
cargo doc --no-deps --open
```

## Dependencies

`RwLockTopic` is built directly on `std::sync::RwLock` - no dependency there.
The one dependency in the tree is [`time`](https://docs.rs/time) (with its
`local-offset` feature), used solely by `Runner`'s verbose logging
(`activate_verbose`) to timestamp each line in local time - getting the local
UTC offset isn't something `std` can do safely on its own.
