# Core framework

How the executor/topic framework under [src/core/](../src/core/) works, why a
plain `RwLock` is enough for it, and what the two communication benchmarks
(`benchmark_comunication_time` and `benchmark_data_freshness`) measure.

Everything else in the crate - sensors, localization, planning, the driving
algorithms, the web GUI - is built from these pieces: each is an `Executor`
running on its own thread at its own rate, talking to the others only through
topics.

It exists to answer a concrete question: *what's the simplest correct way to
share one frequently-updated, moderately-sized value between one producer and
several independent consumers running at different rates?* The answer
implemented here is a `std::sync::RwLock`-protected slot, wrapped in a small
`Executor`/`RwLockTopic` framework.

## Architecture

The framework is deliberately generic - it knows nothing about LIDARs or
scans. It's built from five pieces, all re-exported from the crate root
([src/lib.rs](../src/lib.rs)):

- **`Executor`** ([src/core/executor.rs](../src/core/executor.rs)) - a trait
  for one independently scheduled participant.
  - `init(&mut self, id: u16)` is called once with the executor's identity.
  - `claim_writing_topics(&mut self, captain: &Captain)` is where an executor
    claims every topic it writes. `Runner` calls it for every executor,
    synchronously, before spawning any thread, so a writer conflict is caught
    up front rather than racing, and an executor can read another's topic at
    the top of its `run` whatever order they were added in.
  - `run(&mut self, captain: &Captain)` is the main loop, which should keep
    working until `captain.is_running(id)` goes false.
  - `name()` returns a short human-readable label (e.g. `"Writer 0"`) used
    for the thread name and in diagnostic messages.
  - `fresh()` returns a brand-new instance with the same construction
    parameters, used to restart everything from scratch.
- **`RwLockTopic<T>`** ([src/core/topic.rs](../src/core/topic.rs)) - one
  named, typed slot of shared state with exactly one authorized writer and
  any number of readers, built directly on `std::sync::RwLock`:
  `set_writer(executor_id)` (fails if a different writer is already set),
  `write(executor_id, value)` (fails unless `executor_id` is the registered
  writer), and `read() -> Stamped<T>`.
- **`Stamped<T>` / `WriteMeta`** - what `read` returns: the value together
  with the bookkeeping of the write that published it (a write counter and
  the time it was written). The topic stamps it itself, so no writer can
  forget or fake it. `Stamped` derefs to `T`; `age()` says how stale the
  value is and `is_seed()` whether anyone has written it yet.
- **`Runner`** ([src/core/runner.rs](../src/core/runner.rs)) - the single
  object a binary constructs and drives. `add_executor` registers an executor
  and assigns it a unique id. `run_all()` spawns a thread per registered
  executor. `switch_executor(id, new_executor)` stops whichever executor
  currently owns `id`, joins its thread, and starts `new_executor` in its
  place under that same id - every other running executor is unaffected.
  `stop()` signals every executor to stop; `join_all()` waits for whatever is
  still running. `run_until_stopped()` does the same while also serving
  restart requests and starting/stopping named groups of executors.
  `topic(name)` reads a topic directly, e.g. for a post-run integrity check.
- **`Captain`** ([src/core/captain.rs](../src/core/captain.rs)) - owned by
  `Runner` and shared read-only (via `Arc`) with every executor as the
  `&Captain` passed into `run`. Holds every topic in the system (registered
  by name, type-erased internally so topics of different types can coexist)
  plus the run/stop signals every executor polls: a global flag and a per-id
  one, so a single executor can be stopped without affecting the rest.
  `claim_writer(topic_name, executor_id, initial)` is the executor-facing way
  to claim a topic's writer slot, registering the topic (seeded by `initial`)
  on its first claim. If a *different* executor already holds the slot,
  that's treated as a fatal misconfiguration - it prints one clear diagnostic
  naming both executors and terminates the whole program immediately, instead
  of leaving each executor to detect and panic over the conflict on its own.

A sixth, smaller piece paces every periodic loop: **`Ticker`**
([src/core/rate.rs](../src/core/rate.rs)) - see
[Design decisions](#design-decisions-and-why).

```
                        ┌────────────────────────┐
    WriterExecutor      │                        │ ReaderExecutor x N (30 Hz)
    ───────────────────▶│        Captain         │◀───────────────────────────
    write("shared_data")│  "shared_data" topic   │ ReaderExecutor x N (60 Hz)
    @ 50 Hz             │ (RwLockTopic<Payload>) │◀───────────────────────────
                        │                        │ ReaderExecutor x N (120/210/300 Hz)
                        └────────────────────────┘◀───────────────────────────
```

`RwLockTopic<T>` wraps a single `std::sync::RwLock<Stamped<T>>`:

- **`write(&self, executor_id, value)`** - after checking `executor_id` is
  the registered writer, reads the clocks, then takes the lock's write guard
  and replaces the value and its stamp together, so a reader can never see
  one without the other.
- **`read(&self)`** - takes the lock's read guard and clones the value out
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
≤300 Hz readers, a `RwLock` never shows up as a bottleneck either - this
project's actual numbers (below) leave ample headroom under both
approaches - so the simpler, dependency-free, `unsafe`-free `RwLock`
version won out. The one thing it gives up is the wait-free guarantee: a
reader here is, in principle, at the mercy of the OS scheduler preempting
a lock holder, whereas the epoch-based version's readers and writer could
never block on each other at all. If you need that guarantee (e.g. a hard
real-time control loop that cannot tolerate even a rare priority
inversion), lock-free reclamation is worth the added complexity; this
project's actual workload doesn't need it.

## Design decisions (and why)

**`RwLockTopic::read()` returns an owned clone, not a reference or a
closure.** An earlier, LIDAR-only version of this code had readers pass a
`FnOnce(&Scan) -> R` closure into `read`. Generalizing to arbitrary
executors and topics made that feel like a trap: holding a read guard open
for as long as arbitrary caller code takes to run means a slow or
long-running reader delays the writer for that whole time. So `read` clones
the value and returns it, bounding the time the read lock is held to just
the copy. For a payload around a few KB at ≤300 Hz this is negligible (see
[Program output](#program-output)); for much larger payloads or much higher
rates, this tradeoff would be worth revisiting.

**Readers get the latest value, not a queue.** A topic has no notion of
"unread" data. A reader that polls faster than the writer publishes simply
reads the same value more than once - that's expected and harmless. The
requirement is "always the latest available value," not "notify me exactly
once per new value." A consumer that does need to react only to *changes*
compares `WriteMeta::write_count` between reads (`RwLockTopic::meta()` gives
it without cloning the value), and one that cares about staleness checks
`Stamped::age()`.

**Every periodic loop paces itself with a `Ticker`.** A loop that ends with
`thread::sleep(1.0 / rate_hz)` takes `period + work + however much the OS
overshoots` per iteration, so it always runs slower than its nominal rate,
and the shortfall grows as the period shrinks. `Ticker` instead advances a
*deadline* by exactly one period each time - sleeping for the bulk of the
interval and spinning on `thread::yield_now` for the last stretch - so an
iteration that runs long is paid back by the next one sleeping less, and the
average rate stays on target.

**In the benchmark, the payload is a `Vec<f32>`, so each write allocates.**
Because `--topic_size` is a runtime value, the payload can't be a fixed-size
array baked in at compile time. `RwLockTopic::write` takes its value by
ownership and has no in-place "give me my old buffer back" swap, so the
writer builds a fresh `Vec<f32>` of `topic_size` values every tick rather
than mutating a persistent buffer in place - one small allocation (4 KB at
the 1000-value default) per write, which is still cheap relative to a
typical writer period.

**In the benchmark, timing is recorded locally per executor, then merged per
rate tier.** Each writer/reader executor times its own operations into a
private `Vec<u64>` of per-op nanosecond durations - no cross-thread counters
on the hot path at all. The writer folds its samples into its own
[`Report`](../src/bin/benchmark_comunication_time/report.rs) (mean, standard
deviation, max) once it stops, handed back to `main` through an
`Arc<Mutex<Option<Report>>>`. Each reader instead extends an
`Arc<Mutex<Vec<u64>>>` sample sink shared with every other reader at the
same target rate, once it stops - `main` then builds one merged `Report` per
non-empty rate tier, so a run with e.g. 3 readers at 30 Hz prints a single
combined block rather than 3 near-identical ones. This is also why the live
progress bar can't show read/write counts: nothing is shared or aggregated
until every executor has already finished.

## When would this stop being enough?

Two independent thresholds, not one:

- **Throughput.** The write path spends a few microseconds per payload at
  the default `--topic_size 1000` - mostly generating the simulated
  waveform and allocating the `Vec<f32>`, not the lock acquisition itself
  (see [Program output](#program-output)). At the default 20 ms writer
  period that's well under 1% of budget. This scales with `--topic_size`
  and `--writer_frequency`: it would start to matter if the payload grew
  much larger, or the write rate grew into the low kHz range, where the
  period and the per-write cost become comparable.
- **Worst-case latency / determinism.** More relevant if this were ever
  driving a hard real-time loop: latency has a long tail relative to its
  average (see [Program output](#program-output) for a representative
  average vs. max), which for an `RwLock` is the inherent risk of taking any
  lock at all - a writer can, in principle, be delayed by however long a
  reader holds the read lock, or by the OS scheduler preempting a lock
  holder. If a consumer ever needed a truly *bounded* worst case rather than
  a good average, that's the point at which the lock-free, wait-free design
  this project used previously (see
  [Why not stay lock-free?](#why-not-stay-lock-free)) would be worth the
  added complexity.

## The communication-time benchmark

`benchmark_comunication_time`
([src/bin/benchmark_comunication_time/](../src/bin/benchmark_comunication_time/))
measures how long the `write` and `read` calls themselves take:

- One `WriterExecutor` publishes a payload of `--topic_size` `f32` values
  (1000 by default) at `--writer_frequency` Hz (50 Hz by default), on a
  single `RwLockTopic<Vec<f32>>` named `"shared_data"`.
- `--readers_num` `ReaderExecutor`s (10 by default) each independently
  read the latest payload at their own pace, spread across 5 fixed rate
  tiers - **30 / 60 / 120 / 210 / 300 Hz** - to represent different
  consumers (e.g. a slow logger, a mid-rate obstacle detector, a fast
  control loop). Readers are split evenly across the 5 tiers, with any
  remainder going to the lowest-frequency tiers first (e.g. 12 readers →
  3/3/2/2/2 across 30/60/120/210/300 Hz).

### Layout

```
src/bin/benchmark_comunication_time/
  main.rs            wires the executors together, prints the report
  reader_writer.rs   Payload, WriterExecutor, ReaderExecutor
  report.rs          timing stats (possibly merged across readers) + format_block
src/bin/bench_common/   shared with benchmark_data_freshness
  cli.rs             --topic_size/--readers_num/--writer_frequency/--duration parsing
  progress.rs        the progress bar
  verifier.rs        post-run integrity check
```

### Running it

```sh
cargo run --release --bin benchmark_comunication_time                                    # every default
cargo run --release --bin benchmark_comunication_time -- --duration 30                    # run for 30 seconds instead
cargo run --release --bin benchmark_comunication_time -- --topic_size 4000 --readers_num 20
cargo run --release --bin benchmark_comunication_time -- --help                           # usage
```

It requires no external services or hardware - everything is simulated in
[reader_writer.rs](../src/bin/benchmark_comunication_time/reader_writer.rs).

| Flag                    | What                                                                    | Default |
|-------------------------|-------------------------------------------------------------------------|---------|
| `--topic_size N`        | `f32` values per topic payload                                          | 1000    |
| `--readers_num N`       | total reader threads, spread across 5 rate tiers (30/60/120/210/300 Hz) | 10      |
| `--writer_frequency HZ` | writer publish rate                                                     | 50 Hz   |
| `--duration SECS`       | benchmark run length                                                    | 10 s    |

All four flags are optional, named, and can be given in any order; an
unknown flag, a missing value, or a non-positive value (`--readers_num`
alone also accepts `0`, for a writer-only run) prints a usage message to
stderr and exits with status 1.

Other constants are edited directly in
[main.rs](../src/bin/benchmark_comunication_time/main.rs) rather than exposed
as flags: `TIER_RATES_HZ` (the 5 fixed reader rates) and `BAR_WIDTH` (the
progress bar's width in characters).

### Program output

A run has four parts, in order: the configuration banner, a live progress
bar, a per-tier final report, and the integrity check.

#### 1-2. Banner and progress bar

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
[progress.rs](../src/bin/bench_common/progress.rs)) redraws in place with
`\r` and is driven purely by wall-clock elapsed time versus the requested
duration - it doesn't read from any executor. Every line of the banner
reflects the flags actually passed on the command line, not fixed defaults.

#### 3. Per-tier final report

From a 10-second run at every default (i.e. 2 readers per rate tier):

```
========== FINAL REPORT ==========

WRITER:
  Writer 0 - target 50 Hz (period 20.000 ms)
    writes    :   501 (  500 expected)
    avg time  :    8.65 us +-   2.49 us
    max time  :   41.72 us  (0.2% of period, within budget)

READERS:
  Readers @ 30 Hz (x2) - target 30 Hz (period 33.333 ms)
    reads     :   602 (  600 expected)
    avg time  :    2.56 us +-   3.33 us
    max time  :   22.64 us  (0.1% of period, within budget)

  Readers @ 60 Hz (x2) - target 60 Hz (period 16.667 ms)
    reads     :  1202 ( 1200 expected)
    avg time  :    1.73 us +-   2.14 us
    max time  :   31.78 us  (0.2% of period, within budget)

  Readers @ 120 Hz (x2) - target 120 Hz (period 8.333 ms)
    reads     :  2404 ( 2400 expected)
    avg time  :    2.12 us +-   2.18 us
    max time  :   22.62 us  (0.3% of period, within budget)

  Readers @ 210 Hz (x2) - target 210 Hz (period 4.762 ms)
    reads     :  4206 ( 4200 expected)
    avg time  :    2.24 us +-   2.70 us
    max time  :   35.12 us  (0.7% of period, within budget)

  Readers @ 300 Hz (x2) - target 300 Hz (period 3.333 ms)
    reads     :  6010 ( 6000 expected)
    avg time  :    2.08 us +-   2.87 us
    max time  :  124.42 us  (3.7% of period, within budget)

===================================
```

Each block
([`Report::format_block`](../src/bin/benchmark_comunication_time/report.rs))
reports:

- **operations vs. expected** - actual count, and in parentheses
  `rate_hz * group_size * run_duration` (`group_size` is 1 for the writer,
  or the number of readers pooled into that tier's block), the count the
  executor(s) would hit running at exactly the target rate the whole time.
  The actual count lands on it, give or take the last tick of each executor,
  because every loop is paced by a `Ticker` (see
  [Design decisions](#design-decisions-and-why)).
- **avg time ± standard deviation** - computed from the pooled sample
  buffer for that block: the writer's own private samples, or every reader
  in that tier's samples merged together once they've all stopped.
- **max time as % of period** - "period" is `1 / rate_hz`, the time budget
  for one operation to stay on schedule (e.g. a 300 Hz reader must finish
  each read within 3.333 ms). The max line shows the single slowest
  operation across the block as a percentage of that budget, with a
  trailing `within budget` / `OVER BUDGET` flag - the latter would mean
  that block's executors could no longer keep up even in isolation,
  ignoring contention from anything else running on the machine.

Write cost here is dominated by generating the synthetic waveform and
allocating its `Vec<f32>` (1000 values by default) rather than by
publishing it - in a real system this would be replaced by however long it
takes to read the actual producer, and the topic write itself would still
be a small, constant addition on top. The tail (a max of ~124 µs against a
~2 µs average for the 300 Hz readers in the run above) reflects the
occasional case where a lock acquisition is delayed by another holder or by
OS scheduler jitter, as discussed above.

#### 4. Integrity check

```
Data integrity check: PASSED ✓
```

Confirms the last published payload is non-empty and every value in it is
finite - there's no real sensor behind this benchmark, so there's no
physically-motivated range to check, just that the writer produced a sane
payload.

## The data-freshness benchmark

`benchmark_data_freshness`
([src/bin/benchmark_data_freshness/](../src/bin/benchmark_data_freshness/))
has the same shape and the same four flags - one writer, readers spread
across the same 5 rate tiers - but measures something different. The writer
publishes a *timestamped* payload, and each reader records how old that
payload was at the moment it read it. Where `benchmark_comunication_time`
measures the latency of the `read`/`write` calls, this measures end-to-end
staleness: how far behind the writer's clock a reader's view of the world
can lag.

```sh
cargo run --release --bin benchmark_data_freshness
```

## Code documentation

Every public type and method has a rustdoc comment. Generate and browse the
HTML docs with:

```sh
cargo doc --no-deps --open
```
