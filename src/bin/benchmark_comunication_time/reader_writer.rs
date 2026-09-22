//! The shared payload type and the two executors that publish/consume it.

use crate::report::Report;
use aurorus::{Captain, Executor, Ticker};
use std::any::Any;
use std::sync::{Arc, Mutex};
use std::time::Instant;

/// One published value: an array of `f32`s, sized at runtime by `--topic_size`.
pub type Payload = Vec<f32>;

/// Name of the topic the benchmark payload is published on.
pub const TOPIC_NAME: &str = "shared_data";

/// Publishes a payload of `topic_size` `f32`s on [`TOPIC_NAME`] at a fixed rate.
///
/// Claims the topic's writer slot for its own id in [`Executor::claim_writing_topics`],
/// so it must be the first (and only) executor that ever writes that topic.
pub struct WriterExecutor {
    id: u8,
    rate_hz: f64,
    topic_size: usize,
    report: Arc<Mutex<Option<Report>>>,
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
    pub fn report_handle(&self) -> Arc<Mutex<Option<Report>>> {
        self.report.clone()
    }
}

impl Executor for WriterExecutor {
    fn init(&mut self, id: u8) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        let topic_size = self.topic_size;
        captain.claim_writer::<Payload>(TOPIC_NAME, self.id, move || vec![0.0f32; topic_size]);
    }

    fn run(&mut self, captain: &Captain) {
        let topic = captain.topic::<Payload>(TOPIC_NAME);

        let mut ticker = Ticker::new(self.rate_hz);
        let mut frame = 0u64;
        let mut samples: Vec<u64> = Vec::new();

        while captain.is_running(self.id) {
            let start = Instant::now();

            // Simulate one producer tick: a slowly wobbling waveform across
            // `topic_size` values, standing in for real producer work. `RwLockTopic::write`
            // takes the payload by value, so a fresh `Vec` is built each tick rather
            // than mutating a persistent buffer in place.
            let phase = frame as f32 * 0.02;
            let payload: Payload = (0..self.topic_size)
                .map(|i| {
                    let angle = i as f32 * (std::f32::consts::TAU / self.topic_size as f32);
                    5.0 + 2.0 * (angle + phase).sin()
                })
                .collect();

            topic
                .write(self.id, payload)
                .expect("lost writer authorization for the shared topic");

            samples.push(start.elapsed().as_nanos() as u64);
            frame += 1;

            ticker.wait();
        }

        *self.report.lock().unwrap() = Some(Report::from_samples(
            self.name(),
            "writes",
            self.rate_hz,
            1,
            &samples,
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
/// fixed rate.
///
/// Does not track whether the payload is new since its last read - if `rate_hz` is
/// higher than the writer's publish rate, the same payload will simply be read more
/// than once. That's expected: this executor only needs the most recently published
/// value.
///
/// Rather than building its own [`Report`], each reader times its own operations
/// into a private buffer and, once it exits, extends a `sink` shared with every
/// other reader at the same target rate - so [`crate::main`] can fold all of them
/// into one combined report per rate tier.
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
        let topic = captain.topic::<Payload>(TOPIC_NAME);
        let mut ticker = Ticker::new(self.rate_hz);
        let mut samples: Vec<u64> = Vec::new();

        while captain.is_running(self.id) {
            let start = Instant::now();

            // Read the latest payload and process it - here, sum the values,
            // standing in for real consumer work.
            let payload = topic.read();
            let sum: f32 = payload.iter().sum();
            std::hint::black_box(sum);

            samples.push(start.elapsed().as_nanos() as u64);
            ticker.wait();
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
