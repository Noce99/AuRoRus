//! The LIDAR scan type and the two executors that publish/consume it.

use crate::report::Report;
use efficient_data_sharing::{Captain, Executor, Topic};
use std::any::Any;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

/// Number of angular samples in one LIDAR scan.
pub const NUM_POINTS: usize = 1200;

/// One LIDAR scan: a distance in meters for each of [`NUM_POINTS`] angular samples,
/// ordered by increasing angle around the sensor.
pub type Scan = [f32; NUM_POINTS];

/// Name of the topic the LIDAR scan is published on.
pub const LIDAR_SCAN_TOPIC: &str = "lidar_scan";

/// Publishes a simulated LIDAR scan on [`LIDAR_SCAN_TOPIC`] at a fixed rate.
///
/// Claims the topic's writer slot for its own id in [`Executor::run`], so it must be
/// the first (and only) executor that ever writes that topic.
pub struct LidarWriterExecutor {
    id: u8,
    rate_hz: f64,
    report: Arc<Mutex<Option<Report>>>,
}

impl LidarWriterExecutor {
    pub fn new(rate_hz: f64) -> Self {
        Self {
            id: 0,
            rate_hz,
            report: Arc::new(Mutex::new(None)),
        }
    }

    /// A handle to this executor's report, filled in once [`Executor::run`] returns.
    pub fn report_handle(&self) -> Arc<Mutex<Option<Report>>> {
        self.report.clone()
    }
}

impl Executor for LidarWriterExecutor {
    fn init(&mut self, id: u8) {
        self.id = id;
    }

    fn run(&mut self, captain: &Captain) {
        let scan_topic = captain.claim_writer::<Scan>(LIDAR_SCAN_TOPIC, self.id);

        let interval = Duration::from_secs_f64(1.0 / self.rate_hz);
        let mut next_update = Instant::now() + interval;
        let mut scan: Scan = [0.0f32; NUM_POINTS];
        let mut frame = 0u64;
        let mut samples: Vec<u64> = Vec::new();

        while captain.is_running(self.id) {
            let start = Instant::now();

            // Simulate one 360-degree LIDAR scan: a slowly wobbling wall between
            // ~3m and ~7m, one distance per angular step.
            let phase = frame as f32 * 0.02;
            for (i, distance) in scan.iter_mut().enumerate() {
                let angle = i as f32 * (std::f32::consts::TAU / NUM_POINTS as f32);
                *distance = 5.0 + 2.0 * (angle + phase).sin();
            }

            scan_topic
                .write(self.id, scan)
                .expect("lost writer authorization for lidar_scan topic");

            samples.push(start.elapsed().as_nanos() as u64);
            frame += 1;

            // Precise timing at the target rate.
            next_update += interval;
            let sleep_duration = next_update.saturating_duration_since(Instant::now());
            if sleep_duration > Duration::from_micros(100) {
                thread::sleep(sleep_duration);
            } else if sleep_duration > Duration::ZERO {
                // Busy wait for precise timing.
                while Instant::now() < next_update {
                    thread::yield_now();
                }
            }
        }

        *self.report.lock().unwrap() = Some(Report::from_samples(
            self.name(),
            "writes",
            self.rate_hz,
            &samples,
        ));
    }

    fn name(&self) -> String {
        format!("Writer {}", self.id)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

/// Repeatedly reads the latest scan from [`LIDAR_SCAN_TOPIC`], pacing itself at a
/// fixed rate.
///
/// Does not track whether a scan is new since its last read - if `rate_hz` is higher
/// than the writer's publish rate, the same scan will simply be read more than once.
/// That's expected: this executor only needs the most recently published value.
pub struct LidarReaderExecutor {
    id: u8,
    rate_hz: f64,
    report: Arc<Mutex<Option<Report>>>,
}

impl LidarReaderExecutor {
    pub fn new(rate_hz: f64) -> Self {
        Self {
            id: 0,
            rate_hz,
            report: Arc::new(Mutex::new(None)),
        }
    }

    /// A handle to this executor's report, filled in once [`Executor::run`] returns.
    pub fn report_handle(&self) -> Arc<Mutex<Option<Report>>> {
        self.report.clone()
    }
}

impl Executor for LidarReaderExecutor {
    fn init(&mut self, id: u8) {
        self.id = id;
    }

    fn run(&mut self, captain: &Captain) {
        let scan_topic = captain.topic::<Scan>(LIDAR_SCAN_TOPIC);
        let interval = Duration::from_secs_f64(1.0 / self.rate_hz);
        let mut samples: Vec<u64> = Vec::new();

        while captain.is_running(self.id) {
            let start = Instant::now();

            // Read the latest scan and process it - here, sum the distances,
            // standing in for real consumer work.
            let scan = scan_topic.read();
            let sum: f32 = scan.iter().sum();
            std::hint::black_box(sum);

            samples.push(start.elapsed().as_nanos() as u64);
            thread::sleep(interval);
        }

        *self.report.lock().unwrap() = Some(Report::from_samples(
            self.name(),
            "reads",
            self.rate_hz,
            &samples,
        ));
    }

    fn name(&self) -> String {
        format!("Reader {}", self.id)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}
