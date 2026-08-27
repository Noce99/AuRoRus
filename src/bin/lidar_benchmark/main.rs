//! LIDAR shared-scan benchmark, built on the `efficient_data_sharing` library.
//!
//! One [`lidar::LidarWriterExecutor`] publishes a simulated scan at a fixed rate;
//! several [`lidar::LidarReaderExecutor`]s each independently poll for the latest
//! scan at their own rate. See `README.md` for the full design rationale and
//! measured numbers.

mod cli;
mod lidar;
mod progress;
mod report;
mod verifier;

use efficient_data_sharing::{ExecutorHandler, TopicHandler};
use lidar::{LidarReaderExecutor, LidarWriterExecutor, Scan, LIDAR_SCAN_TOPIC, NUM_POINTS};
use report::Report;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

const WRITER_HZ: f64 = 50.0;
const READER_RATES_HZ: [f64; 4] = [30.0, 60.0, 150.0, 300.0];
const BAR_WIDTH: usize = 40;

fn main() {
    let duration_secs = cli::parse_duration_secs(std::env::args());
    let duration = Duration::from_secs_f64(duration_secs);

    println!("=== LIDAR Shared Scan Demo with crossbeam-epoch ===\n");
    println!("Configuration:");
    println!("  - Scan size: {} distances (f32, {} bytes)", NUM_POINTS, NUM_POINTS * 4);
    println!("  - Update rate: 50 Hz (20ms interval)");
    println!("  - Readers: 4 threads at 30/60/150/300 Hz");
    println!("  - Duration: {:.1} seconds", duration_secs);
    println!("\nStarting threads...\n");

    let mut topics = TopicHandler::new();
    topics.register_topic::<Scan>(LIDAR_SCAN_TOPIC, [0.0f32; NUM_POINTS]);
    let topics = Arc::new(topics);

    let writer = LidarWriterExecutor::new(WRITER_HZ);
    let writer_report: Arc<Mutex<Option<Report>>> = writer.report_handle();

    let readers: Vec<LidarReaderExecutor> = READER_RATES_HZ.iter().map(|&hz| LidarReaderExecutor::new(hz)).collect();
    let reader_reports: Vec<Arc<Mutex<Option<Report>>>> = readers.iter().map(|r| r.report_handle()).collect();

    let mut handler = ExecutorHandler::new(topics.clone());
    handler.add_executor(Box::new(writer));
    for reader in readers {
        handler.add_executor(Box::new(reader));
    }
    handler.run_all();

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
    topics.stop();
    handler.join_all();

    println!("\n========== FINAL REPORT ==========\n");
    println!("WRITER:");
    println!("{}\n", writer_report.lock().unwrap().as_ref().unwrap().format_block(duration));
    println!("READERS:");
    for report in &reader_reports {
        println!("{}\n", report.lock().unwrap().as_ref().unwrap().format_block(duration));
    }
    println!("===================================");

    let consistent = verifier::verify_consistency(&topics);
    println!("\nData integrity check: {}", if consistent { "PASSED ✓" } else { "FAILED ✗" });

    println!("\nBenchmark completed!");
}
