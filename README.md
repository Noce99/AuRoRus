# efficient_data_sharing

A small Rust **library** for sharing periodically-updated data between
independently scheduled threads without locking, plus a **binary**
(`lidar_benchmark`) that demonstrates it on a simulated LIDAR sensor: one
executor publishes a new 360° scan at a fixed rate, and several independent
reader executors each poll for the *latest* scan at their own rate — no
mutexes, no reader blocking the writer, no readers blocking each other.

It exists to answer a concrete question: *what's the simplest correct way
to share one frequently-updated, moderately-sized value between one
producer and several independent consumers running at different rates,
without anyone taking a lock?* The answer implemented here is
[`crossbeam-epoch`](https://docs.rs/crossbeam-epoch)-based epoch reclamation,
wrapped in a small `Executor`/`Topic` framework general enough to grow
beyond this one demo. This README explains the framework, why epoch
reclamation is the right tool underneath it, what it costs, and when it
would stop being enough.

## The scenario

- One writer executor publishes a scan of `NUM_POINTS = 1200` `f32`
  distances (one per angular sample) at **50 Hz** (a real or simulated
  LIDAR's natural output rate).
- 4 reader executors each independently consume the latest scan at their
  own pace — this demo uses **30 / 60 / 150 / 300 Hz** to represent
  different consumers (e.g. a slow logger, a mid-rate obstacle detector, a
  fast control loop).
- Readers only ever care about *the most recently published* scan. If a
  reader polls faster than 50 Hz, it will simply read the same scan more
  than once — that's expected and harmless, not a bug to fix. There is no
  "have I seen this one already" tracking, by design (see
  [Design decisions](#design-decisions-and-why)).

## Architecture

The library ([src/lib.rs](src/lib.rs) and friends) is deliberately generic —
it knows nothing about LIDARs or scans. It's built from four pieces:

- **`Executor`** ([src/executor.rs](src/executor.rs)) — a trait for one
  independently scheduled participant. `init(&mut self, id: u8)` is called
  once with the executor's identity; `run(&mut self, topics: &TopicHandler)`
  is its main loop, which should keep working until
  `topics.is_running(id)` goes false.
- **`Topic`** ([src/topic.rs](src/topic.rs)) — a trait for one named, typed
  slot of shared state with exactly one authorized writer and any number of
  readers: `set_writer(executor_id)` (fails if a writer is already set),
  `write(executor_id, value)` (fails unless `executor_id` is the registered
  writer), and `read() -> Item`. `LockFreeTopic<T>` is the lock-free
  implementation of it, built on `crossbeam-epoch`.
- **`TopicHandler`** ([src/topic_handler.rs](src/topic_handler.rs)) — owns
  every topic in the system (registered by name, type-erased internally so
  topics of different types can coexist) plus the run/stop signals every
  executor polls: a global flag (`stop()`) and a per-id one, so a single
  executor can be stopped without affecting the rest. Shared read-only (via
  `Arc`) with every executor.
- **`ExecutorHandler`** ([src/executor_handler.rs](src/executor_handler.rs))
  — owns every executor. `add_executor` registers one and assigns it a
  unique id; `run_all()` spawns a thread per registered executor, each
  calling `init` then `run` against the shared `TopicHandler`, and can be
  called again later for executors added afterward. `switch_executor(id,
  new_executor)` stops whichever executor currently owns `id`, joins its
  thread, and starts `new_executor` in its place under that same id — every
  other running executor is unaffected. `join_all()` waits for whatever is
  still running (typically after `TopicHandler::stop()`).

```
                     ┌───────────────────────┐
  LidarWriterExecutor│                       │ LidarReaderExecutor (30 Hz)
  ──────────────────▶│     TopicHandler      │◀──────────────────────────
  write("lidar_scan")│  "lidar_scan" topic   │ LidarReaderExecutor (60 Hz)
  @ 50 Hz             │  (LockFreeTopic<Scan>)│◀──────────────────────────
                     │                       │ LidarReaderExecutor (150 Hz)
                     │                       │◀──────────────────────────
                     │                       │ LidarReaderExecutor (300 Hz)
                     └───────────────────────┘◀──────────────────────────
```

`lidar_benchmark` ([src/bin/lidar_benchmark/](src/bin/lidar_benchmark/)) is
just one concrete use of this: `LidarWriterExecutor` and
`LidarReaderExecutor` ([lidar.rs](src/bin/lidar_benchmark/lidar.rs)) are
`Executor` impls that write/read a single `LockFreeTopic<Scan>` registered
under the name `"lidar_scan"`.

Underneath `LockFreeTopic<T>`, [src/lock_free_cell.rs](src/lock_free_cell.rs)
wraps a single `crossbeam_epoch::Atomic<T>` — a word-sized,
atomically-swappable pointer to a heap-allocated `T`:

- **`store(&self, value: T)`** — the writer heap-allocates a new `T`,
  atomically swaps the shared pointer to point at it, and hands the *old*
  pointer to the epoch collector via `guard.defer_destroy`. The old value's
  memory isn't freed immediately; it's freed once every thread that might
  still be reading it has since left its `load_and` call.
- **`load_and<F, R>(&self, f: F) -> R`** — a reader "pins" the current
  epoch, loads whatever pointer is currently published, and runs its
  closure against it (used internally by `LockFreeTopic::read` to clone the
  value before returning it). Pinning is what tells the writer's deferred
  destructor "don't free this yet, I might still be looking at it." Reading
  never blocks the writer, and readers never block each other.

This gives every reader a consistent, torn-free view of one full value
(never half of an old scan and half of a new one) with no locks anywhere on
the hot path.

### Why not just a `Mutex<Scan>` / `RwLock<Scan>`?

It would also be correct, and honestly for a 50 Hz writer and ≤300 Hz
readers, a `RwLock` would very likely never show up as a bottleneck either
— this project's actual numbers (below) leave 1000x+ headroom under either
approach. `crossbeam-epoch` is used here specifically because it's a good
vehicle for understanding lock-free reclamation, and because it gives
*wait-free* reads: a reader is never at the mercy of the OS scheduler
preempting a lock holder. If you're adapting this code, a plain `RwLock`
is a perfectly reasonable simpler alternative unless you specifically need
that guarantee.

## Design decisions (and why)

**`Topic::read()` returns an owned clone, not a reference or a closure.**
The earlier, LIDAR-only version of this code had readers pass a
`FnOnce(&Scan) -> R` closure into `read`, running it while pinned to the
current epoch. Generalizing to arbitrary executors and topics made that
feel like a trap: holding the epoch pin open for as long as arbitrary
caller code takes to run means a slow or long-running reader delays
reclamation of every value the writer has superseded in the meantime. So
`LockFreeTopic::read` now clones the value and returns it, bounding the
pinned section to just the copy. For a 4.8 KB `Scan` at ≤300 Hz this is
negligible (see [Program output](#program-output)); for much larger
payloads or much higher rates, this tradeoff would be worth revisiting.

**Data is heap-allocated per write, not stored inline.**
`Atomic<T>` is a *pointer*; a topic's value lives in a separate heap
allocation, one new allocation per `write()` call. This is inherent to how
`crossbeam-epoch` reclaims memory for multiple concurrent readers — and at
50 Hz, one ~4.8 KB allocation every 20 ms is nowhere close to being a
bottleneck (see [Program output](#program-output)). The
`#[repr(align(64))]` on `LockFreeCell` only guards the pointer field itself
against false sharing; it's not claiming the value's bytes are
cache-resident.

**No "is this scan new?" tracking on the reader side.**
Readers deliberately do not carry a sequence number or generation counter.
Because readers run *faster* than the writer for at least one of the four
rates (60/150/300 Hz vs. the writer's 50 Hz), a reader will sometimes read
the same scan twice in a row. This is intentional: the requirement for this
use case is "always the latest available value," not "notify me exactly
once per new value." Adding staleness detection would be needed if a
consumer had to react only to *changes* — it isn't needed here, and adding
it anyway would be unrequested complexity.

**The writer uses sleep + busy-wait for pacing, readers use plain sleep.**
`LidarWriterExecutor::run` ([lidar.rs](src/bin/lidar_benchmark/lidar.rs))
needs to hit 50 Hz precisely, so it sleeps for the bulk of its interval and
busy-waits (`thread::yield_now`) for the last stretch to avoid OS scheduler
granularity error. `LidarReaderExecutor::run` just uses
`thread::sleep(read_interval)`, which is simpler but less precise —
observed reader throughput comes in a couple of percent under the nominal
rate at 300 Hz. That's acceptable here because readers only need "roughly
this often," not an exact deadline; if a real consumer needed precise
timing, it would want the same hybrid technique the writer uses.

**Timing is recorded locally per executor, not through shared atomics.**
Each writer/reader executor times its own operations into a private
`Vec<u64>` of per-op nanosecond durations — no cross-thread counters on the
hot path at all. Only once an executor stops does it fold its samples into
a [`Report`](src/bin/lidar_benchmark/report.rs) (mean, standard deviation,
max), handed back to `main` through an `Arc<Mutex<Option<Report>>>` set up
before the executor is registered with the `ExecutorHandler`, and printed
in the final report. This is also why the live progress bar can't show
read/write counts: nothing is shared or aggregated until every executor has
already finished.

## When would this stop being enough?

Two independent thresholds, not one:

- **Throughput.** The write path currently spends ~16 µs per scan — mostly
  computing the simulated 1200-point sine wave, not the allocation itself.
  At a 20 ms writer period that's under 0.1% of budget. This would start to
  matter if the write rate grew into the low kHz range, where the period
  and the per-write cost become comparable.
- **Worst-case latency / determinism.** More relevant if this were ever
  driving a hard real-time loop: write latency has a long tail relative to
  its average (~16 µs average vs. ~88 µs max in a typical run - see
  [Program output](#program-output)), almost certainly from
  `crossbeam-epoch` batching deferred frees rather than reclaiming one at a
  time. If a consumer ever needed a *bounded* worst case rather than a good
  average, that's the point at which a hand-rolled, pre-allocated
  reference-counted buffer pool (no per-update heap allocation, no batched
  reclamation bursts) would be worth the added complexity. At 50 Hz / ≤300 Hz
  this project has no such requirement - every executor reports comfortably
  `within budget` (see [Program output](#program-output)).

## Program output

A run has four parts, in order: the configuration banner, a live progress
bar, a per-executor final report, and the integrity check.

### Project layout

```
src/
  lib.rs                    crate docs, module wiring, public re-exports
  executor.rs                Executor trait
  executor_handler.rs        ExecutorHandler struct
  topic.rs                   Topic trait, TopicError, LockFreeTopic<T>
  topic_handler.rs           TopicHandler struct
  lock_free_cell.rs          internal epoch-based single-writer/multi-reader cell
src/bin/lidar_benchmark/
  main.rs                    wires topics + executors together, prints the report
  lidar.rs                   Scan/NUM_POINTS, LidarWriterExecutor, LidarReaderExecutor
  report.rs                  per-executor timing stats + format_block
  verifier.rs                post-run integrity check
  progress.rs                the progress bar
  cli.rs                     [DURATION_SECS] argument parsing
```

### 1-2. Banner and progress bar

```
=== LIDAR Shared Scan Demo with crossbeam-epoch ===

Configuration:
  - Scan size: 1200 distances (f32, 4800 bytes)
  - Update rate: 50 Hz (20ms interval)
  - Readers: 4 threads at 30/60/150/300 Hz
  - Duration: 10.0 seconds

Starting threads...

[####################--------------------]  50.1% |  5.0s / 10.0s
```

The bar (`print_progress_bar` in
[progress.rs](src/bin/lidar_benchmark/progress.rs)) redraws in place with
`\r` and is driven purely by wall-clock elapsed time versus the requested
duration — it doesn't read from any executor, so it stays accurate even
though (per the next section) nothing is aggregated across executors until
they've all stopped. `Duration` in the banner reflects whatever
`[DURATION_SECS]` was passed on the command line (see
[Running it](#running-it)), not a fixed 10s.

### 3. Per-executor final report

From a representative 10-second run, 1 writer executor (id 0) at 50 Hz and
4 reader executors (ids 1-4) at 30/60/150/300 Hz:

```
========== FINAL REPORT ==========

WRITER:
  Writer 0 - target 50 Hz (period 20.000 ms)
    writes    :   500 (  500 expected)
    avg time  :   15.65 us +-  13.43 us
    max time  :   87.77 us  (0.4% of period, within budget)

READERS:
  Reader 1 - target 30 Hz (period 33.333 ms)
    reads     :   300 (  300 expected)
    avg time  :    4.76 us +-   4.91 us
    max time  :   39.55 us  (0.1% of period, within budget)

  Reader 2 - target 60 Hz (period 16.667 ms)
    reads     :   597 (  600 expected)
    avg time  :    4.02 us +-   6.19 us
    max time  :  130.76 us  (0.8% of period, within budget)

  Reader 3 - target 150 Hz (period 6.667 ms)
    reads     :  1473 ( 1500 expected)
    avg time  :    4.42 us +-   4.24 us
    max time  :   35.05 us  (0.5% of period, within budget)

  Reader 4 - target 300 Hz (period 3.333 ms)
    reads     :  2904 ( 3000 expected)
    avg time  :    4.79 us +-   4.67 us
    max time  :   41.65 us  (1.2% of period, within budget)

===================================
```

Each block ([`Report::format_block`](src/bin/lidar_benchmark/report.rs))
reports:

- **operations vs. expected** — actual count, and in parentheses
  `rate_hz * run_duration`, the count the executor would hit if it ran at
  exactly its target rate the whole time. Readers consistently land a
  little under it (`thread::sleep` pacing, not a correctness issue — see
  [Design decisions](#design-decisions-and-why)); the writer's precise
  sleep+busy-wait pacing hits its target exactly.
- **avg time ± standard deviation** — computed from that executor's own
  private sample buffer once it stops (see the timing note in
  [Design decisions](#design-decisions-and-why)).
- **max time as % of period** — "period" is `1 / rate_hz`, the time budget
  for one operation to stay on schedule (e.g. a 300 Hz reader must finish
  each read within 3.333 ms). The max line shows the single slowest
  operation as a percentage of that budget, with a trailing `within budget`
  / `OVER BUDGET` flag — the latter would mean that executor's own pacing
  loop could no longer keep up even in isolation, ignoring contention from
  anything else running on the machine.

Write cost here is dominated by generating the synthetic scan (1200 `sin()`
calls per update) rather than by publishing it — in a real system this
would be replaced by however long it takes to read the actual sensor, and
the topic write itself would still be a small, constant addition on top.
The write time's large standard deviation and tail (max ~88 µs vs. a
~16 µs average) is consistent with `crossbeam-epoch` occasionally batching
several deferred frees into one write, as discussed above.

### 4. Integrity check

```
Data integrity check: PASSED ✓
```

Confirms every distance in the last published scan is finite and within a
plausible range (`0.0..20.0` meters — a placeholder bound for this
simulator; swap in your real sensor's documented range).

## Running it

```sh
cargo run --release              # default: 10-second run
cargo run --release -- 30        # run for 30 seconds instead
cargo run --release -- --help    # usage
```

Prints the title and configuration, then a live progress bar for the run,
then a per-executor final report block (operation count vs. expected, timing
mean ± standard deviation, and max time as a percentage of that executor's
period budget) and the integrity check. Requires no external services or
hardware — the "LIDAR" is fully simulated in
[src/bin/lidar_benchmark/lidar.rs](src/bin/lidar_benchmark/lidar.rs). Since
the crate has exactly one binary target, `--bin lidar_benchmark` isn't
required, but works too: `cargo run --release --bin lidar_benchmark -- 30`.

The optional positional argument is the benchmark duration in seconds
(must be a positive number; defaults to 10 if omitted). An invalid value
prints a usage message to stderr and exits with status 1.

### Configuration

Everything is currently a constant in `main()`, meant to be edited directly
rather than passed as CLI flags (this is a demo, not a tool):

| What                    | Where                                                  | Current value                   |
|-------------------------|---------------------------------------------------------|----------------------------------|
| Scan size               | `NUM_POINTS` (`src/bin/lidar_benchmark/lidar.rs`)      | 1200 points                      |
| Writer rate             | `WRITER_HZ` in `main()`                                | 50 Hz                            |
| Reader count/rates      | `READER_RATES_HZ` in `main()`                          | `[30.0, 60.0, 150.0, 300.0]` Hz  |
| Benchmark duration      | `DEFAULT_DURATION_SECS`, or `[DURATION_SECS]` CLI arg  | 10 s (default) |
| Progress bar width      | `BAR_WIDTH` in `main()`                                | 40 characters                    |
| Integrity check bounds  | `verifier::verify_consistency`                          | `0.0 < d < 20.0` m               |

Note the title/configuration banner at the top of `main()` is printed as
plain hardcoded text, not interpolated from these constants — if you change
a value above, update the corresponding banner line too.

## Code documentation

Every public type and method has a rustdoc comment. Generate and browse the
HTML docs with:

```sh
cargo doc --no-deps --open
```

## Dependencies

- [`crossbeam-epoch`](https://docs.rs/crossbeam-epoch) — the epoch-based
  memory reclamation this whole design is built on.
- [`crossbeam-utils`](https://docs.rs/crossbeam-utils) — pulled in
  transitively by `crossbeam-epoch`; not used directly in this crate today.
