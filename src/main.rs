//! Single-writer, multi-reader shared LIDAR scan buffer.
//!
//! One writer thread publishes a new [`Scan`] at a fixed rate; several
//! reader threads each independently poll for the *latest* published scan
//! at their own rate. There is no locking on the hot path: publishing and
//! reading both go through [`crossbeam_epoch`], which lets readers see a
//! consistent, complete scan without ever blocking the writer (or each
//! other), at the cost of one small heap allocation per published scan.
//! See `README.md` for the full design rationale and measured numbers.

use crossbeam_epoch::{self as epoch, Atomic, Owned};
use std::io::{self, Write as _};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use std::thread;

/// Number of angular samples in one LIDAR scan.
pub const NUM_POINTS: usize = 1200;

/// One LIDAR scan: a distance in meters for each of [`NUM_POINTS`] angular
/// samples, ordered by increasing angle around the sensor.
pub type Scan = [f32; NUM_POINTS];

/// A [`Scan`] shared between one writer and many readers without locking.
///
/// # How it works
///
/// [`Atomic<Scan>`] is a tagged *pointer* (word-sized), not the scan's bytes
/// inline. [`update`](Self::update) heap-allocates a new scan and atomically
/// swaps the pointer; the previous scan is freed only once every reader that
/// might still be looking at it has moved on ("epoch-based reclamation").
/// [`read`](Self::read) never blocks the writer and never blocks other
/// readers - it just follows whatever pointer was most recently published.
///
/// The `#[repr(align(64))]` on this struct only protects the pointer field
/// itself from false sharing with unrelated data on the same cache line; it
/// does not make the scan's bytes cache-line resident, since those bytes
/// live in a separate heap allocation.
#[repr(align(64))]
pub struct SharedBuffer {
    data: Atomic<Scan>,
}

impl SharedBuffer {
    /// Creates a buffer holding an all-zero scan.
    pub fn new() -> Self {
        Self {
            data: Atomic::new([0.0f32; NUM_POINTS]),
        }
    }

    /// Publishes `new_data` as the latest scan.
    ///
    /// Meant to be called from a single writer thread. Allocates once for
    /// the new scan and defers freeing the previous one until it is safe.
    #[inline(always)]
    pub fn update(&self, new_data: &Scan) {
        let guard = epoch::pin();
        let new = Owned::new(*new_data);
        let old = self.data.swap(new, Ordering::Release, &guard);
        // Old data will be reclaimed when no readers are using it
        unsafe { guard.defer_destroy(old) };
    }

    /// Runs `f` against whatever scan is currently the latest published one.
    ///
    /// Safe to call from any number of reader threads concurrently. `f`
    /// should be quick: it runs while pinned to the current epoch, which
    /// delays reclamation of any scan a writer swaps out in the meantime.
    #[inline(always)]
    pub fn read<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&Scan) -> R,
    {
        let guard = epoch::pin();
        let shared = self.data.load(Ordering::Acquire, &guard);
        unsafe { f(&*shared.as_raw()) }
    }
}

/// Whether a [`ThreadReport`] describes the writer or one of the readers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThreadKind {
    Writer,
    Reader,
}

/// Per-thread timing summary, built once a writer or reader thread stops.
///
/// Every writer/reader thread times its own operations locally (no shared
/// atomics on the hot path) and folds the samples into this report only
/// when it exits, which is also where mean/standard-deviation/max are
/// computed.
pub struct ThreadReport {
    pub kind: ThreadKind,
    pub id: usize,
    pub rate_hz: f64,
    pub count: u64,
    pub mean_ns: f64,
    pub std_dev_ns: f64,
    pub max_ns: u64,
}

impl ThreadReport {
    /// Builds a report from the individual per-operation durations (in
    /// nanoseconds) recorded by one thread over its lifetime.
    fn from_samples(kind: ThreadKind, id: usize, rate_hz: f64, samples: &[u64]) -> Self {
        let count = samples.len() as u64;
        let mean_ns = if count > 0 {
            samples.iter().sum::<u64>() as f64 / count as f64
        } else {
            0.0
        };
        let std_dev_ns = if count > 1 {
            let variance = samples
                .iter()
                .map(|&s| {
                    let d = s as f64 - mean_ns;
                    d * d
                })
                .sum::<f64>()
                / (count - 1) as f64;
            variance.sqrt()
        } else {
            0.0
        };
        let max_ns = samples.iter().copied().max().unwrap_or(0);

        Self {
            kind,
            id,
            rate_hz,
            count,
            mean_ns,
            std_dev_ns,
            max_ns,
        }
    }

    /// How many operations this thread should have completed over `duration`
    /// if it had run at exactly its target rate the whole time.
    pub fn expected_count(&self, duration: Duration) -> u64 {
        (self.rate_hz * duration.as_secs_f64()).round() as u64
    }

    /// The time budget for one operation at this thread's target rate: e.g.
    /// a 300 Hz reader must finish each read within `1/300 s` to keep pace
    /// with its own schedule (ignoring whatever else it also has to do).
    pub fn period(&self) -> Duration {
        Duration::from_secs_f64(1.0 / self.rate_hz)
    }

    /// An indented, multi-line report block for this thread, e.g.:
    ///
    /// ```text
    ///   Reader 3 - target 300 Hz (period 3.333 ms)
    ///     reads      : 2907 (3000 expected)
    ///     avg time   :    4.24 us +-    5.29 us
    ///     max time   :  129.00 us  (3.9% of period, within budget)
    /// ```
    pub fn format_block(&self, duration: Duration) -> String {
        let label = match self.kind {
            ThreadKind::Writer => format!("Writer {}", self.id),
            ThreadKind::Reader => format!("Reader {}", self.id),
        };
        let verb = match self.kind {
            ThreadKind::Writer => "writes",
            ThreadKind::Reader => "reads",
        };
        let period_us = self.period().as_secs_f64() * 1_000_000.0;
        let max_us = self.max_ns as f64 / 1000.0;
        let pct_of_period = max_us / period_us * 100.0;
        let budget_note = if pct_of_period <= 100.0 {
            "within budget"
        } else {
            "OVER BUDGET"
        };

        let header = format!("  {label} - target {rate:.0} Hz (period {period_ms:.3} ms)", rate = self.rate_hz, period_ms = period_us / 1000.0);
        let count_line = format!("    {verb:<10}: {count:>5} ({expected:>5} expected)", count = self.count, expected = self.expected_count(duration));
        let avg_line = format!("    avg time  : {mean:>7.2} us +- {std:>6.2} us", mean = self.mean_ns / 1000.0, std = self.std_dev_ns / 1000.0);
        let max_line = format!("    max time  : {max_us:>7.2} us  ({pct_of_period:.1}% of period, {budget_note})");

        format!("{header}\n{count_line}\n{avg_line}\n{max_line}")
    }
}

/// Owns the shared buffer and the run/stop flag that coordinates shutdown
/// of every writer and reader thread spawned from it.
pub struct SharedData {
    buffer: Arc<SharedBuffer>,
    running: Arc<AtomicBool>,
}

impl SharedData {
    /// Creates an empty, running shared buffer.
    pub fn new() -> Self {
        Self {
            buffer: Arc::new(SharedBuffer::new()),
            running: Arc::new(AtomicBool::new(true)),
        }
    }

    /// Spawns the single writer thread, publishing a simulated LIDAR scan
    /// at `rate_hz` using a sleep + busy-wait hybrid for precise timing.
    /// Runs until [`stop`](Self::stop) is called, then returns a
    /// [`ThreadReport`] summarizing every write it performed.
    pub fn start_writer(&self, thread_id: usize, rate_hz: f64) -> thread::JoinHandle<ThreadReport> {
        let buffer = self.buffer.clone();
        let running = self.running.clone();

        thread::Builder::new()
            .name(format!("writer-{}", thread_id))
            .spawn(move || {
                let interval = Duration::from_secs_f64(1.0 / rate_hz);
                let mut next_update = Instant::now() + interval;
                let mut scan: Scan = [0.0f32; NUM_POINTS];
                let mut frame = 0u64;
                let mut samples: Vec<u64> = Vec::new();

                while running.load(Ordering::Relaxed) {
                    let start = Instant::now();

                    // Simulate one 360-degree LIDAR scan: a slowly wobbling
                    // wall between ~3m and ~7m, one distance per angular step.
                    let phase = frame as f32 * 0.02;
                    for (i, distance) in scan.iter_mut().enumerate() {
                        let angle = i as f32 * (std::f32::consts::TAU / NUM_POINTS as f32);
                        *distance = 5.0 + 2.0 * (angle + phase).sin();
                    }

                    // Update buffer
                    buffer.update(&scan);

                    samples.push(start.elapsed().as_nanos() as u64);
                    frame += 1;

                    // Precise timing at the target rate
                    next_update += interval;
                    let sleep_duration = next_update.saturating_duration_since(Instant::now());
                    if sleep_duration > Duration::from_micros(100) {
                        thread::sleep(sleep_duration);
                    } else if sleep_duration > Duration::ZERO {
                        // Busy wait for precise timing
                        while Instant::now() < next_update {
                            thread::yield_now();
                        }
                    }
                }

                ThreadReport::from_samples(ThreadKind::Writer, thread_id, rate_hz, &samples)
            })
            .unwrap()
    }

    /// Spawns one reader thread that repeatedly reads the latest published
    /// scan, sleeping to pace itself at `rate_hz`. Runs until
    /// [`stop`](Self::stop) is called, then returns a [`ThreadReport`]
    /// summarizing every read it performed.
    ///
    /// Readers do not track whether a scan is new since their last read -
    /// if `rate_hz` is higher than the writer's publish rate, the same scan
    /// will simply be read more than once. That's expected: each reader
    /// only needs the most recently published value.
    pub fn start_reader(&self, thread_id: usize, rate_hz: f64) -> thread::JoinHandle<ThreadReport> {
        let buffer = self.buffer.clone();
        let running = self.running.clone();

        thread::Builder::new()
            .name(format!("reader-{}", thread_id))
            .spawn(move || {
                let interval = Duration::from_secs_f64(1.0 / rate_hz);
                let mut samples: Vec<u64> = Vec::new();

                while running.load(Ordering::Relaxed) {
                    let start = Instant::now();

                    // Read the latest scan and process it - here, sum the
                    // distances, standing in for real consumer work.
                    buffer.read(|data| {
                        let sum: f32 = data.iter().sum();
                        std::hint::black_box(sum);
                    });

                    samples.push(start.elapsed().as_nanos() as u64);
                    thread::sleep(interval);
                }

                ThreadReport::from_samples(ThreadKind::Reader, thread_id, rate_hz, &samples)
            })
            .unwrap()
    }

    /// Signals every writer and reader thread spawned from this instance to
    /// stop after their current iteration.
    pub fn stop(&self) {
        self.running.store(false, Ordering::Relaxed);
    }
}

/// Sanity-checks the final state of a [`SharedBuffer`] after a run.
pub struct DataVerifier {
    buffer: Arc<SharedBuffer>,
}

impl DataVerifier {
    /// Wraps a buffer for verification.
    pub fn new(buffer: Arc<SharedBuffer>) -> Self {
        Self { buffer }
    }

    /// Sanity-checks the latest scan: every distance should be a finite,
    /// physically plausible reading. The bounds are placeholders for this
    /// simulator - replace with the real sensor's documented range.
    pub fn verify_consistency(&self) -> bool {
        self.buffer
            .read(|data| data.iter().all(|&d| d.is_finite() && d > 0.0 && d < 20.0))
    }
}

/// Prints a single-line, in-place progress bar tracking `elapsed` out of
/// `total`, tqdm-style. Call repeatedly with growing `elapsed`; the caller
/// is responsible for printing a final newline once done.
fn print_progress_bar(elapsed: Duration, total: Duration, width: usize) {
    let frac = (elapsed.as_secs_f64() / total.as_secs_f64()).clamp(0.0, 1.0);
    let filled = (frac * width as f64).round() as usize;
    print!(
        "\r[{}{}] {:5.1}% | {:4.1}s / {:.1}s",
        "#".repeat(filled),
        "-".repeat(width - filled),
        frac * 100.0,
        elapsed.as_secs_f64().min(total.as_secs_f64()),
        total.as_secs_f64(),
    );
    let _ = io::stdout().flush();
}

const DEFAULT_DURATION_SECS: f64 = 10.0;

/// Parses the optional `[DURATION_SECS]` positional argument from `argv`,
/// falling back to [`DEFAULT_DURATION_SECS`]. Prints usage and exits the
/// process on `-h`/`--help` or an invalid value.
fn parse_duration_secs(mut args: impl Iterator<Item = String>) -> f64 {
    let program = args.next().unwrap_or_else(|| env!("CARGO_PKG_NAME").to_string());
    let usage = format!(
        "Usage: {program} [DURATION_SECS]\n\n\
         Runs the benchmark for DURATION_SECS seconds (default: {DEFAULT_DURATION_SECS})."
    );

    match args.next() {
        None => DEFAULT_DURATION_SECS,
        Some(arg) if arg == "-h" || arg == "--help" => {
            println!("{usage}");
            std::process::exit(0);
        }
        Some(arg) => match arg.parse::<f64>() {
            Ok(secs) if secs > 0.0 => secs,
            _ => {
                eprintln!("Invalid DURATION_SECS '{arg}': expected a positive number of seconds\n");
                eprintln!("{usage}");
                std::process::exit(1);
            }
        },
    }
}

fn main() {
    let duration_secs = parse_duration_secs(std::env::args());
    let duration = Duration::from_secs_f64(duration_secs);

    println!("=== LIDAR Shared Scan Demo with crossbeam-epoch ===\n");
    println!("Configuration:");
    println!("  - Scan size: {} distances (f32, {} bytes)", NUM_POINTS, NUM_POINTS * 4);
    println!("  - Update rate: 50 Hz (20ms interval)");
    println!("  - Readers: 4 threads at 30/60/150/300 Hz");
    println!("  - Duration: {:.1} seconds", duration_secs);
    println!("\nStarting threads...\n");

    const WRITER_HZ: f64 = 50.0;
    const READER_RATES_HZ: [f64; 4] = [30.0, 60.0, 150.0, 300.0];
    const BAR_WIDTH: usize = 40;

    let shared_data = Arc::new(SharedData::new());

    let writer_handle = shared_data.start_writer(0, WRITER_HZ);
    let reader_handles: Vec<_> = READER_RATES_HZ
        .iter()
        .enumerate()
        .map(|(i, &rate_hz)| shared_data.start_reader(i, rate_hz))
        .collect();

    // Progress bar for the run; driven purely by wall-clock elapsed time,
    // independent of how many reads/writes actually happened.
    let start_time = Instant::now();
    while start_time.elapsed() < duration {
        print_progress_bar(start_time.elapsed(), duration, BAR_WIDTH);
        thread::sleep(Duration::from_millis(100));
    }
    print_progress_bar(duration, duration, BAR_WIDTH);
    println!();

    // Stop all threads
    println!("\nStopping all threads...");
    shared_data.stop();

    // Wait for all threads to finish and collect their timing reports
    let writer_report = writer_handle.join().unwrap();
    let reader_reports: Vec<ThreadReport> = reader_handles
        .into_iter()
        .map(|h| h.join().unwrap())
        .collect();

    println!("\n========== FINAL REPORT ==========\n");
    println!("WRITER:");
    println!("{}\n", writer_report.format_block(duration));
    println!("READERS:");
    for report in &reader_reports {
        println!("{}\n", report.format_block(duration));
    }
    println!("===================================");

    // Verify data integrity
    let verifier = DataVerifier::new(shared_data.buffer.clone());
    let consistent = verifier.verify_consistency();
    println!("\nData integrity check: {}", if consistent { "PASSED ✓" } else { "FAILED ✗" });

    println!("\nBenchmark completed!");
}
