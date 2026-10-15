//! Debug recording: [`DebugRecorder`], the handle that starts/stops a recording and
//! reports how it's going, and the [`DebugExecutor`] it runs to record every topic
//! somebody is writing - or only some named ones, see [`DebugRecorder::for_topics`] -
//! into a `.debug` file - see [`crate::core::debug_format`] for the file layout.
//!
//! A recording runs as its own group of executors ([`DEBUG_GROUP`] by default, see
//! [`Captain::spawn_group`]), so it can start and stop while everything else keeps
//! running - and, like any group, ends for good on a restart
//! ([`Captain::request_restart`]). Recorders with different groups (e.g. a debug
//! recording and a mapping session) can run at the same time.

use crate::core::captain::{Captain, DebugTopic};
use crate::core::debug_format::DebugFileWriter;
use crate::core::executor::Executor;
use crate::core::rate::Ticker;
use std::any::Any;
use std::collections::HashMap;
use std::io;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// Name of the group a recording runs as - see [`Captain::spawn_group`].
pub const DEBUG_GROUP: &str = "debug";
/// Recording rate, in Hz, when nobody asks for another one.
pub const DEFAULT_DEBUG_FREQUENCY_HZ: f64 = 100.0;

/// How long a rolling window is, for measuring the achieved recording rate against
/// the requested one - see [`DebugExecutor::run`].
const RATE_WINDOW: Duration = Duration::from_secs(5);
/// The achieved rate must fall below this fraction of the requested one, over one
/// [`RATE_WINDOW`], before a warning is printed.
const RATE_WARNING_THRESHOLD: f64 = 0.8;
/// Minimum time between two consecutive rate warnings, so a sustained shortfall
/// doesn't spam stderr once per window.
const RATE_WARNING_COOLDOWN: Duration = Duration::from_secs(10);
/// How often a recording of named topics looks again for the ones not registered
/// yet - see [`DebugExecutor::record`].
const TOPIC_LOOKUP_PERIOD: Duration = Duration::from_secs(1);

/// Where a [`DebugRecorder`] is at.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum DebugState {
    /// Nothing recorded yet.
    Idle,
    /// Recording into [`DebugStatus::path`].
    Recording,
    /// Asked to stop: the final flush hasn't finished yet.
    Saving,
    /// The last recording is complete, at [`DebugStatus::path`].
    Saved,
    /// The last recording couldn't be opened or written - see [`DebugStatus::error`].
    Failed,
}

/// A snapshot of a [`DebugRecorder`] - see [`DebugRecorder::status`].
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct DebugStatus {
    pub state: DebugState,
    /// The file of the current or last recording, `None` while [`DebugState::Idle`].
    pub path: Option<PathBuf>,
    /// The requested recording rate, in Hz ([`DEFAULT_DEBUG_FREQUENCY_HZ`] while idle).
    pub frequency_hz: f64,
    /// How long the current or last recording has been/was running, in seconds.
    pub duration_s: f64,
    /// Samples written so far.
    pub samples: u64,
    /// Writes never recorded, because the topic was written again before the
    /// recorder polled it: the jumps in [`crate::WriteMeta::write_count`] between two
    /// samples of the same topic, summed. Expected for a topic written faster than
    /// the recording rate; writes from before the recording started aren't counted.
    pub missed_writes: u64,
    /// The rate actually achieved over the last full window, in Hz, if one has passed yet.
    pub achieved_hz: Option<f64>,
    /// Whether [`achieved_hz`](Self::achieved_hz) is noticeably short of `frequency_hz`.
    pub falling_behind: bool,
    /// Whether the last recording ended without [`DebugRecorder::stop`] asking -
    /// a restart or a shutdown. The file is complete all the same.
    pub interrupted: bool,
    /// Why the last recording failed, while [`DebugState::Failed`].
    pub error: Option<String>,
}

/// A record written once, at the very start of a recording, under its own topic
/// name - e.g. what a session was recorded with. Its `payload` is opaque to the
/// file format, like any sample's.
#[derive(Debug, Clone, PartialEq)]
pub struct DebugPreamble {
    pub topic: String,
    /// Recorded as the topic's writer.
    pub writer: String,
    pub payload: Vec<u8>,
}

/// What a [`DebugRecorder`] and the [`DebugExecutor`] it started share.
#[derive(Debug)]
struct Shared {
    /// The group recordings run as - see [`Captain::spawn_group`].
    group: String,
    /// The only topics recorded, or `None` for every topic somebody writes.
    topics: Option<Vec<String>>,
    status: DebugStatus,
    started_at: Option<Instant>,
    ended_at: Option<Instant>,
}

/// Starts and stops debug recordings, and reports how the current or last one is
/// going. Cheap to clone - every clone is the same recorder - so it can outlive
/// any one [`Captain`], e.g. carried across a restart by whoever offers the
/// controls.
#[derive(Debug, Clone)]
pub struct DebugRecorder(Arc<Mutex<Shared>>);

impl Default for DebugRecorder {
    fn default() -> Self {
        Self::new()
    }
}

impl DebugRecorder {
    /// A recorder of every topic somebody writes, as the group [`DEBUG_GROUP`].
    pub fn new() -> Self {
        Self::with(DEBUG_GROUP.to_string(), None)
    }

    /// A recorder of only `topics` (those of them somebody writes), as the group
    /// `group` - which must differ from every other recorder's that may run at the
    /// same time.
    pub fn for_topics(group: impl Into<String>, topics: Vec<String>) -> Self {
        Self::with(group.into(), Some(topics))
    }

    fn with(group: String, topics: Option<Vec<String>>) -> Self {
        Self(Arc::new(Mutex::new(Shared {
            group,
            topics,
            status: DebugStatus {
                state: DebugState::Idle,
                path: None,
                frequency_hz: DEFAULT_DEBUG_FREQUENCY_HZ,
                duration_s: 0.0,
                samples: 0,
                missed_writes: 0,
                achieved_hz: None,
                falling_behind: false,
                interrupted: false,
                error: None,
            },
            started_at: None,
            ended_at: None,
        })))
    }

    /// Starts recording into `path` (created, or overwritten if it already exists)
    /// at `frequency_hz`, as this recorder's group. Fails if a recording is
    /// already running or still saving.
    pub fn start(&self, captain: &Captain, path: PathBuf, frequency_hz: f64) -> Result<(), String> {
        self.start_with(captain, path, frequency_hz, Vec::new())
    }

    /// Like [`start`](Self::start), writing `preamble` first.
    pub fn start_with(
        &self,
        captain: &Captain,
        path: PathBuf,
        frequency_hz: f64,
        preamble: Vec<DebugPreamble>,
    ) -> Result<(), String> {
        let executor = self.begin_with(path, frequency_hz, preamble)?;
        let group = self.0.lock().unwrap().group.clone();
        captain.spawn_group(group, vec![executor]);
        Ok(())
    }

    /// Like [`start`](Self::start), but hands back the executor to run instead of
    /// asking a [`Captain`] to - for starting a recording before anything runs, via
    /// [`crate::Runner::add_group`].
    pub fn begin(&self, path: PathBuf, frequency_hz: f64) -> Result<Box<dyn Executor>, String> {
        self.begin_with(path, frequency_hz, Vec::new())
    }

    fn begin_with(
        &self,
        path: PathBuf,
        frequency_hz: f64,
        preamble: Vec<DebugPreamble>,
    ) -> Result<Box<dyn Executor>, String> {
        if !(frequency_hz.is_finite() && frequency_hz > 0.0) {
            return Err(format!("invalid recording rate {frequency_hz} Hz"));
        }
        let mut shared = self.0.lock().unwrap();
        if matches!(
            shared.status.state,
            DebugState::Recording | DebugState::Saving
        ) {
            return Err("a recording is already running".to_string());
        }
        shared.status = DebugStatus {
            state: DebugState::Recording,
            path: Some(path.clone()),
            frequency_hz,
            duration_s: 0.0,
            samples: 0,
            missed_writes: 0,
            achieved_hz: None,
            falling_behind: false,
            interrupted: false,
            error: None,
        };
        shared.started_at = Some(Instant::now());
        shared.ended_at = None;
        let topics = shared.topics.clone();
        Ok(DebugExecutor {
            id: 0,
            path,
            frequency_hz,
            topics,
            preamble,
            recorder: self.clone(),
        }
        .boxed())
    }

    /// Stops the running recording: its file is complete once
    /// [`status`](Self::status) says [`DebugState::Saved`]. Fails if nothing is recording.
    pub fn stop(&self, captain: &Captain) -> Result<(), String> {
        let mut shared = self.0.lock().unwrap();
        if shared.status.state != DebugState::Recording {
            return Err("nothing is recording".to_string());
        }
        shared.status.state = DebugState::Saving;
        captain.stop_group(shared.group.clone());
        Ok(())
    }

    /// Where the current or last recording is at.
    pub fn status(&self) -> DebugStatus {
        let shared = self.0.lock().unwrap();
        let mut status = shared.status.clone();
        if let Some(started_at) = shared.started_at {
            let end = shared.ended_at.unwrap_or_else(Instant::now);
            status.duration_s = end.duration_since(started_at).as_secs_f64();
        }
        status
    }

    fn update(&self, f: impl FnOnce(&mut DebugStatus)) {
        f(&mut self.0.lock().unwrap().status);
    }

    /// Marks the recording over: saved, or failed with `error`.
    fn finish(&self, error: Option<String>) {
        let mut shared = self.0.lock().unwrap();
        shared.ended_at = Some(Instant::now());
        let status = &mut shared.status;
        // Still `Recording` means nobody called `stop` - a restart or a shutdown did.
        status.interrupted = status.state == DebugState::Recording;
        status.state = if error.is_some() {
            DebugState::Failed
        } else {
            DebugState::Saved
        };
        status.error = error;
    }
}

/// Snapshots every topic somebody is currently writing, at `frequency_hz`, into a
/// `.debug` file - but only when a topic was written again since the last snapshot
/// (its [`crate::WriteMeta::write_count`] moved), so a topic nobody is updating
/// doesn't bloat the file with repeats. Each sample is timestamped with when it was
/// *written* (its [`crate::WriteMeta::written_at_unix_us`]), not when this recorder
/// happened to poll it - or at 0, for a value written before the recording
/// started. A writer re-publishing an identical value is recorded each time it
/// does - that is itself information - but only once per tick at most, since
/// writes landing between two ticks collapse into the latest one.
///
/// Only ever started by a [`DebugRecorder`], which it reports its progress to.
/// Never claims any topic's writer slot, so it can never appear as a topic's
/// writer itself - it therefore never records itself, with no special casing
/// needed.
pub(crate) struct DebugExecutor {
    id: u16,
    path: PathBuf,
    frequency_hz: f64,
    /// The only topics recorded, or `None` for every topic.
    topics: Option<Vec<String>>,
    /// Written once, before anything else.
    preamble: Vec<DebugPreamble>,
    recorder: DebugRecorder,
}

impl DebugExecutor {
    /// The topics to poll: every registered one, or those of [`Self::topics`]
    /// registered so far.
    fn polled_topics(&self, captain: &Captain) -> Vec<(String, Arc<dyn DebugTopic>)> {
        let snapshot = captain.debug_topics_snapshot();
        match &self.topics {
            None => snapshot,
            Some(names) => snapshot
                .into_iter()
                .filter(|(name, _)| names.contains(name))
                .collect(),
        }
    }

    /// Records until told to stop, then flushes - see [`DebugExecutor`].
    fn record(&self, captain: &Captain) -> io::Result<()> {
        let mut writer = DebugFileWriter::create(&self.path, self.frequency_hz)?;
        println!("recording debug session to {:?}", self.path);

        for preamble in &self.preamble {
            let topic_id = writer.topic_id(&preamble.topic, &preamble.writer)?;
            writer.write_sample(topic_id, 0, &preamble.payload)?;
        }

        let session_start_us = writer.session_start_unix_micros();
        let mut last_write_count: HashMap<String, u64> = HashMap::new();
        let mut samples: u64 = 0;
        let mut missed_writes: u64 = 0;
        // Every topic is looked up again on every tick, as topics can be registered
        // at any time; named ones only until all are found, then once in a while.
        let mut polled = self.polled_topics(captain);
        let mut last_lookup = Instant::now();

        let mut ticker = Ticker::new(self.frequency_hz);

        let mut window_start = Instant::now();
        let mut window_ticks: u32 = 0;
        let mut last_warned = Instant::now() - RATE_WARNING_COOLDOWN;

        while captain.is_running(self.id) {
            let all_found = self
                .topics
                .as_ref()
                .is_some_and(|names| polled.len() == names.len());
            if !all_found && (self.topics.is_none() || last_lookup.elapsed() >= TOPIC_LOOKUP_PERIOD)
            {
                polled = self.polled_topics(captain);
                last_lookup = Instant::now();
            }
            for (name, topic) in &polled {
                let Some(writer_id) = topic.writer() else {
                    continue;
                }; // only "topics somebody is writing"
                let write_count = topic.meta().write_count;
                // Still the seed, or nothing new since the last snapshot - skip without encoding.
                if write_count == 0 || last_write_count.get(name) == Some(&write_count) {
                    continue;
                }
                let (encoded, meta) = topic.read_encoded();
                let elapsed_us =
                    (meta.written_at_unix_us as u128).saturating_sub(session_start_us) as u64;
                let topic_id = writer.topic_id(name, &captain.name_of(writer_id))?;
                writer.write_sample(topic_id, elapsed_us, &encoded)?;
                samples += 1;
                if let Some(&last) = last_write_count.get(name) {
                    missed_writes += meta.write_count.saturating_sub(last + 1);
                }
                last_write_count.insert(name.clone(), meta.write_count);
            }
            writer.maybe_flush()?;
            self.recorder.update(|status| {
                status.samples = samples;
                status.missed_writes = missed_writes;
            });

            ticker.wait();

            window_ticks += 1;
            if window_start.elapsed() >= RATE_WINDOW {
                let achieved_hz = window_ticks as f64 / window_start.elapsed().as_secs_f64();
                let falling_behind = achieved_hz < self.frequency_hz * RATE_WARNING_THRESHOLD;
                self.recorder.update(|status| {
                    status.achieved_hz = Some(achieved_hz);
                    status.falling_behind = falling_behind;
                });
                if falling_behind && last_warned.elapsed() >= RATE_WARNING_COOLDOWN {
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

        writer.flush()
    }
}

impl Executor for DebugExecutor {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn run(&mut self, captain: &Captain) {
        match self.record(captain) {
            Ok(()) => {
                println!("debug session saved to {:?}", self.path);
                self.recorder.finish(None);
            }
            Err(err) => {
                eprintln!("DebugExecutor: recording to {:?} failed: {err}", self.path);
                self.recorder.finish(Some(err.to_string()));
            }
        }
    }

    fn name(&self) -> String {
        "Debug".to_string()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    /// Never called in practice: a recording always runs as a group, and a restart
    /// doesn't bring groups back.
    fn fresh(&self) -> Box<dyn Executor> {
        Box::new(Self {
            id: 0,
            path: self.path.clone(),
            frequency_hz: self.frequency_hz,
            topics: self.topics.clone(),
            preamble: self.preamble.clone(),
            recorder: self.recorder.clone(),
        })
    }
}
