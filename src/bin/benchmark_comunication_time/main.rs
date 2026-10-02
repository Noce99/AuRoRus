//! Reader/writer communication-time benchmark, built on the `aurorus`
//! library.
//!
//! One [`reader_writer::WriterExecutor`] publishes a payload at a fixed rate; several
//! [`reader_writer::ReaderExecutor`]s each independently poll for the latest payload,
//! spread across 5 fixed rate tiers. See `documentation/core_framework.md` for the
//! full design rationale and measured numbers.

// `cli`, `progress` and `verifier` are byte-identical between the two
// benchmarks, so they live once in `src/bin/bench_common/` and are pulled in
// here by path. A plain `mod` can't reach outside this binary's own
// directory, and these are binary-only helpers that have no business in the
// library's public API.
#[path = "../bench_common/cli.rs"]
mod cli;
#[path = "../bench_common/progress.rs"]
mod progress;
mod reader_writer;
mod report;
#[path = "../bench_common/verifier.rs"]
mod verifier;

use aurorus::{Executor, Runner};
use reader_writer::{Payload, ReaderExecutor, TOPIC_NAME, WriterExecutor};
use report::Report;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

/// The 5 fixed reader rate tiers `--readers_num` is spread across.
const TIER_RATES_HZ: [f64; 5] = [30.0, 60.0, 120.0, 210.0, 300.0];
const BAR_WIDTH: usize = 40;

/// One non-empty reader tier: its rate, how many readers share it, and the sample
/// sink they all extend once they stop.
type Tier = (f64, u64, Arc<Mutex<Vec<u64>>>);

/// Splits `readers_num` readers across [`TIER_RATES_HZ`]: each tier gets
/// `readers_num / 5`, and the first `readers_num % 5` (lowest-frequency) tiers get
/// one extra reader each.
fn tier_reader_counts(readers_num: usize) -> [usize; TIER_RATES_HZ.len()] {
    let base = readers_num / TIER_RATES_HZ.len();
    let remainder = readers_num % TIER_RATES_HZ.len();
    std::array::from_fn(|i| base + if i < remainder { 1 } else { 0 })
}

fn main() {
    let config = cli::parse_config(std::env::args());
    let duration = Duration::from_secs_f64(config.duration_secs);
    let tier_counts = tier_reader_counts(config.readers_num);

    println!("=== Reader/Writer Communication Benchmark (RwLock) ===\n");
    println!("Configuration:");
    println!(
        "  - Topic size: {} f32 values ({} bytes)",
        config.topic_size,
        config.topic_size * 4
    );
    println!(
        "  - Writer rate: {:.1} Hz ({:.1}ms interval)",
        config.writer_frequency_hz,
        1000.0 / config.writer_frequency_hz
    );
    print!("  - Readers: {} threads across", config.readers_num);
    for (hz, count) in TIER_RATES_HZ.iter().zip(tier_counts) {
        print!(" {hz:.0}Hz={count}");
    }
    println!();
    println!("  - Duration: {:.1} seconds", config.duration_secs);
    println!("\nStarting threads...\n");

    let mut runner = Runner::new();

    let writer = WriterExecutor::new(config.writer_frequency_hz, config.topic_size);
    let writer_report = writer.report_handle();
    runner.add_executor(writer.boxed());

    // One shared sample sink per non-empty tier, so every reader in that tier can
    // fold its samples into one combined report once it stops.
    let mut tiers: Vec<Tier> = Vec::new();
    for (&rate_hz, &count) in TIER_RATES_HZ.iter().zip(tier_counts.iter()) {
        if count == 0 {
            continue;
        }
        let sink: Arc<Mutex<Vec<u64>>> = Arc::new(Mutex::new(Vec::new()));
        for _ in 0..count {
            runner.add_executor(ReaderExecutor::new(rate_hz, sink.clone()).boxed());
        }
        tiers.push((rate_hz, count as u64, sink));
    }

    runner.run_all();

    // Progress bar for the run; driven purely by wall-clock elapsed time,
    // independent of how many reads/writes actually happened.
    let start_time = Instant::now();
    while start_time.elapsed() < duration {
        progress::print_progress_bar(start_time.elapsed(), duration, BAR_WIDTH);
        thread::sleep(Duration::from_millis(100));
    }
    progress::print_progress_bar(duration, duration, BAR_WIDTH);
    println!();

    // Stop all executors and wait for their threads to finish.
    println!("\nStopping all threads...");
    runner.stop();
    runner.join_all();

    println!("\n========== FINAL REPORT ==========\n");
    println!("WRITER:");
    println!(
        "{}\n",
        writer_report
            .lock()
            .unwrap()
            .as_ref()
            .unwrap()
            .format_block(duration)
    );

    println!("READERS:");
    for (rate_hz, group_size, sink) in &tiers {
        let samples = sink.lock().unwrap();
        let label = format!("Readers @ {rate_hz:.0} Hz (x{group_size})");
        let report = Report::from_samples(label, "reads", *rate_hz, *group_size, &samples);
        println!("{}\n", report.format_block(duration));
    }
    println!("===================================");

    let consistent = verifier::verify_consistency::<Payload>(&runner, TOPIC_NAME, |p| p);
    println!(
        "\nData integrity check: {}",
        if consistent {
            "PASSED ✓"
        } else {
            "FAILED ✗"
        }
    );

    println!("\nBenchmark completed!");
}
