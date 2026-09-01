//! [`DebugExecutor`]: the [`Executor`] [`crate::Runner::debug_mode`] auto-adds to
//! record every topic somebody is writing into a `.debug` file - see
//! [`crate::debug_format`] for the file layout.

use crate::core::captain::Captain;
use crate::core::executor::Executor;
use crate::debug_format::DebugFileWriter;
use std::any::Any;
use std::collections::HashMap;
use std::path::PathBuf;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// How long a rolling window is, for measuring the achieved recording rate against
/// the requested one - see [`DebugExecutor::run`].
const RATE_WINDOW: Duration = Duration::from_secs(5);
/// The achieved rate must fall below this fraction of the requested one, over one
/// [`RATE_WINDOW`], before a warning is printed.
const RATE_WARNING_THRESHOLD: f64 = 0.8;
/// Minimum time between two consecutive rate warnings, so a sustained shortfall
/// doesn't spam stderr once per window.
const RATE_WARNING_COOLDOWN: Duration = Duration::from_secs(10);

/// Snapshots every topic somebody is currently writing, at `frequency_hz`, into a
/// `.debug` file - but only when a topic's value actually changed since the last
/// snapshot, so a steady-state topic doesn't bloat the file with identical repeats.
///
/// Added automatically by [`crate::Runner::debug_mode`], never by a binary's
/// `main` directly. Never claims any topic's writer slot, so it can never appear
/// as a topic's writer itself - it therefore never records itself, with no special
/// casing needed.
pub(crate) struct DebugExecutor {
    id: u8,
    path: PathBuf,
    frequency_hz: f64,
    /// `false` for the original instance [`crate::Runner::debug_mode`] creates
    /// (start a brand-new file); `true` for every instance [`fresh`](Executor::fresh)
    /// produces after a mid-session [`crate::Captain::request_restart`] (reopen and
    /// append to the same file, rather than truncating everything recorded so far).
    resume_existing: bool,
}

impl DebugExecutor {
    pub(crate) fn new(path: PathBuf, frequency_hz: f64) -> Self {
        Self { id: 0, path, frequency_hz, resume_existing: false }
    }
}

impl Executor for DebugExecutor {
    fn init(&mut self, id: u8) {
        self.id = id;
    }

    fn run(&mut self, captain: &Captain) {
        let mut writer = match if self.resume_existing {
            DebugFileWriter::resume(&self.path)
        } else {
            DebugFileWriter::create(&self.path, self.frequency_hz)
        } {
            Ok(writer) => writer,
            Err(err) => {
                eprintln!("DebugExecutor: failed to open {:?}: {err}", self.path);
                return;
            }
        };

        let session_start_us = writer.session_start_unix_micros();
        let mut last_bytes: HashMap<String, Vec<u8>> = HashMap::new();

        let interval = Duration::from_secs_f64(1.0 / self.frequency_hz);
        let mut next_tick = Instant::now() + interval;

        let mut window_start = Instant::now();
        let mut window_ticks: u32 = 0;
        let mut last_warned = Instant::now() - RATE_WARNING_COOLDOWN;

        while captain.is_running(self.id) {
            // Wall-clock elapsed-since-session-start, not `Instant`-based: this
            // must stay consistent with whatever a `resume()`d writer already
            // wrote before a restart, and an `Instant` from a previous process
            // can't be reconstructed from the file - only a wall-clock epoch can.
            let now_us = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default().as_micros();
            let elapsed_us = now_us.saturating_sub(session_start_us) as u64;

            for (name, topic) in captain.debug_topics_snapshot() {
                let Some(writer_id) = topic.writer() else { continue }; // only "topics somebody is writing"
                let encoded = topic.read_encoded();
                if last_bytes.get(&name) != Some(&encoded) {
                    let topic_id = writer
                        .topic_id(&name, &captain.name_of(writer_id))
                        .expect("failed to write to the debug file");
                    writer
                        .write_sample(topic_id, elapsed_us, &encoded)
                        .expect("failed to write to the debug file");
                    last_bytes.insert(name, encoded);
                }
            }
            writer.maybe_flush().expect("failed to flush the debug file");

            // Precise pacing, matching `benchmark_data_freshness`'s `WriterExecutor::run`.
            next_tick += interval;
            let sleep = next_tick.saturating_duration_since(Instant::now());
            if sleep > Duration::from_micros(100) {
                thread::sleep(sleep);
            } else {
                while Instant::now() < next_tick {
                    thread::yield_now();
                }
            }

            window_ticks += 1;
            if window_start.elapsed() >= RATE_WINDOW {
                let achieved_hz = window_ticks as f64 / window_start.elapsed().as_secs_f64();
                if achieved_hz < self.frequency_hz * RATE_WARNING_THRESHOLD
                    && last_warned.elapsed() >= RATE_WARNING_COOLDOWN
                {
                    eprintln!(
                        "DebugExecutor: falling behind - achieved ~{achieved_hz:.1} Hz of {:.1} Hz requested",
                        self.frequency_hz
                    );
                    last_warned = Instant::now();
                }
                window_start = Instant::now();
                window_ticks = 0;
            }
        }

        writer.flush().expect("failed to flush the debug file");
    }

    fn name(&self) -> String {
        "Debug".to_string()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn fresh(&self) -> Box<dyn Executor> {
        Box::new(Self { id: self.id, path: self.path.clone(), frequency_hz: self.frequency_hz, resume_existing: true })
    }
}
