//! The shared, timestamped payload type and the two executors that publish/consume
//! it.

use aurorus::{Captain, Executor};
use std::any::Any;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant, SystemTime};

/// One published value: an array of `f32`s (sized at runtime by `--topic_size`)
/// plus the [`SystemTime`] it was published at, so a reader can compute how old
/// the value it just read is. Wall-clock (not [`Instant`]) so the type can derive
/// `serde::Serialize`/`Deserialize` - required of every topic type since
/// [`aurorus::Captain::claim_writer`]/[`aurorus::Runner::register_topic`] need it
/// for generic debug recording, and `Instant` (an opaque monotonic handle) has no
/// portable representation to serialize.
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct TimestampedPayload {
    pub timestamp: SystemTime,
    pub data: Vec<f32>,
}

/// Name of the topic the benchmark payload is published on.
pub const TOPIC_NAME: &str = "shared_data";

/// Publishes a fresh, timestamped payload of `topic_size` `f32`s on [`TOPIC_NAME`]
/// at a fixed rate.
///
/// Claims the topic's writer slot for its own id in [`Executor::claim_writing_topics`],
/// so it must be the first (and only) executor that ever writes that topic.
pub struct WriterExecutor {
    id: u8,
    rate_hz: f64,
    topic_size: usize,
    report: Arc<Mutex<Option<crate::report::WriteReport>>>,
}

impl WriterExecutor {
    pub fn new(rate_hz: f64, topic_size: usize) -> Self {
        Self {
            id: 0,
            rate_hz,
            topic_size,
            report: Arc::new(Mutex::new(None)),
        }
    }

    /// A handle to this executor's report, filled in once [`Executor::run`] returns.
    pub fn report_handle(&self) -> Arc<Mutex<Option<crate::report::WriteReport>>> {
        self.report.clone()
    }
}

impl Executor for WriterExecutor {
    fn init(&mut self, id: u8) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        let topic_size = self.topic_size;
        captain.claim_writer::<TimestampedPayload>(TOPIC_NAME, self.id, move || TimestampedPayload {
            timestamp: SystemTime::now(),
            data: vec![0.0f32; topic_size],
        });
    }

    fn run(&mut self, captain: &Captain) {
        let topic = captain.topic::<TimestampedPayload>(TOPIC_NAME);

        let interval = Duration::from_secs_f64(1.0 / self.rate_hz);
        let mut next_update = Instant::now() + interval;
        let mut frame = 0u64;
        let mut writes = 0u64;

        while captain.is_running(self.id) {
            // Simulate one producer tick: a slowly wobbling waveform across
            // `topic_size` values, standing in for real producer work. The
            // timestamp is captured right before publishing, so it reflects when
            // this particular value became current as closely as possible.
            let phase = frame as f32 * 0.02;
            let data: Vec<f32> = (0..self.topic_size)
                .map(|i| {
                    let angle = i as f32 * (std::f32::consts::TAU / self.topic_size as f32);
                    5.0 + 2.0 * (angle + phase).sin()
                })
                .collect();
            let payload = TimestampedPayload {
                timestamp: SystemTime::now(),
                data,
            };

            topic
                .write(self.id, payload)
                .expect("lost writer authorization for the shared topic");

            writes += 1;
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

        *self.report.lock().unwrap() = Some(crate::report::WriteReport::new(
            self.name(),
            self.rate_hz,
            writes,
        ));
    }

    fn name(&self) -> String {
        format!("Writer {}", self.id)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn fresh(&self) -> Box<dyn Executor> {
        Box::new(WriterExecutor::new(self.rate_hz, self.topic_size))
    }
}

/// Repeatedly reads the latest payload from [`TOPIC_NAME`], pacing itself at a
/// fixed rate, and records how old each payload was when it was read.
///
/// Rather than building its own report, each reader times its own age samples into
/// a private buffer and, once it exits, extends a `sink` shared with every other
/// reader at the same target rate - so [`crate::main`] can fold all of them into one
/// combined age report per rate tier.
pub struct ReaderExecutor {
    id: u8,
    rate_hz: f64,
    sink: Arc<Mutex<Vec<u64>>>,
}

impl ReaderExecutor {
    pub fn new(rate_hz: f64, sink: Arc<Mutex<Vec<u64>>>) -> Self {
        Self {
            id: 0,
            rate_hz,
            sink,
        }
    }
}

impl Executor for ReaderExecutor {
    fn init(&mut self, id: u8) {
        self.id = id;
    }

    fn run(&mut self, captain: &Captain) {
        let topic = captain.topic::<TimestampedPayload>(TOPIC_NAME);
        let interval = Duration::from_secs_f64(1.0 / self.rate_hz);
        let mut samples: Vec<u64> = Vec::new();

        while captain.is_running(self.id) {
            let payload = topic.read();
            let age = SystemTime::now().duration_since(payload.timestamp).unwrap_or_default();

            // Touch the payload, standing in for real consumer work.
            let sum: f32 = payload.data.iter().sum();
            std::hint::black_box(sum);

            samples.push(age.as_nanos() as u64);
            thread::sleep(interval);
        }

        self.sink.lock().unwrap().extend(samples);
    }

    fn name(&self) -> String {
        format!("Reader {}", self.id)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn fresh(&self) -> Box<dyn Executor> {
        Box::new(ReaderExecutor::new(self.rate_hz, self.sink.clone()))
    }
}
