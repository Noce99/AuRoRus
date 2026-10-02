//! [`Captain`] is what each running [`crate::Executor`] answers to: it owns every
//! topic plus the run/stop signals an executor polls to know when to exit.

use crate::core::log::LogColor;
use crate::core::topic::{RwLockTopic, WriteMeta};
use serde::Serialize;
use serde::de::DeserializeOwned;
use std::any::Any;
use std::collections::HashMap;
use std::fmt;
use std::io::{self, Write};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

/// Type-erased access to one topic's writer/value, for the debug-recording
/// [`crate::core::debug_executor::DebugExecutor`] - implemented generically below for every
/// [`RwLockTopic<T>`] whose `T` satisfies the bound every topic type must now meet (see
/// [`Captain::claim_writer`]/[`Captain::register_topic`]). Lives here, not in [`crate::core::topic`],
/// so `topic.rs` itself stays free of any bincode/serde dependency.
pub(crate) trait DebugTopic: Send + Sync {
    /// See [`RwLockTopic::writer`]. Named identically on purpose - there's no ambiguity in practice,
    /// since this is only ever called through `Arc<dyn DebugTopic>`, never on a concrete `RwLockTopic<T>`.
    fn writer(&self) -> Option<u16>;
    /// See [`RwLockTopic::meta`] - lets [`crate::core::debug_executor::DebugExecutor`] tell whether
    /// anything new was written without encoding the value.
    fn meta(&self) -> WriteMeta;
    /// The topic's current value (without its [`WriteMeta`]), bincode-encoded, together with the
    /// meta of the write that published it - what [`crate::core::debug_executor::DebugExecutor`]
    /// writes to the `.debug` file. Read under one lock, so the two always match.
    fn read_encoded(&self) -> (Vec<u8>, WriteMeta);
    /// The topic's current value as JSON, together with the meta of the write that published
    /// it - or `None` for the value if its JSON would exceed `max_bytes` (e.g. a whole map
    /// raster), so a viewer inspecting it can't make the process serialize megabytes per poll:
    /// serialization is cut off as soon as it crosses the limit.
    fn read_json(&self, max_bytes: usize) -> (Option<serde_json::Value>, WriteMeta);
}

/// An [`io::Write`] sink that fails once more than `limit` bytes are written to it - what lets
/// [`DebugTopic::read_json`] abandon an oversized value partway through serializing it.
struct LimitedWriter {
    buf: Vec<u8>,
    limit: usize,
}

impl Write for LimitedWriter {
    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        if self.buf.len() + data.len() > self.limit {
            return Err(io::Error::other("value exceeds the JSON size limit"));
        }
        self.buf.extend_from_slice(data);
        Ok(data.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl<T: Clone + Send + Sync + Serialize + 'static> DebugTopic for RwLockTopic<T> {
    fn writer(&self) -> Option<u16> {
        RwLockTopic::writer(self)
    }

    fn meta(&self) -> WriteMeta {
        RwLockTopic::meta(self)
    }

    fn read_encoded(&self) -> (Vec<u8>, WriteMeta) {
        let stamped = self.read();
        let encoded = bincode::serde::encode_to_vec(&stamped.value, bincode::config::standard())
            .expect("encoding a topic's current value for the debug recorder should never fail");
        (encoded, stamped.meta)
    }

    fn read_json(&self, max_bytes: usize) -> (Option<serde_json::Value>, WriteMeta) {
        let stamped = self.read();
        let mut writer = LimitedWriter {
            buf: Vec::new(),
            limit: max_bytes,
        };
        let json = serde_json::to_writer(&mut writer, &stamped.value)
            .ok()
            .and_then(|()| serde_json::from_slice(&writer.buf).ok());
        (json, stamped.meta)
    }
}

/// What an executor can ask [`crate::Runner`] to do while everything runs - see
/// [`Captain::spawn_group`] and [`Captain::stop_group`].
pub(crate) enum RunnerRequest {
    /// Start `executors`, as one group called `group`.
    Spawn {
        group: String,
        executors: Vec<Box<dyn crate::Executor>>,
    },
    /// Stop every executor of the group called `group`.
    Stop { group: String },
}

/// Source of [`Captain::epoch`]: the last epoch handed out, so two captains built within the
/// same microsecond still get distinct, increasing epochs.
static LAST_EPOCH: AtomicU64 = AtomicU64::new(0);

/// A value never handed out before in this process, and - being microseconds since the Unix
/// epoch - almost surely never handed out by an earlier run of it either.
fn next_epoch() -> u64 {
    let now_us = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_micros() as u64;
    let previous = LAST_EPOCH
        .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |last| {
            Some(now_us.max(last + 1))
        })
        .expect("the update closure always returns Some");
    now_us.max(previous + 1)
}

/// The object [`crate::Runner`] hands to every executor's [`crate::Executor::run`]:
/// a registry of named, typed topics plus the run/stop signals every executor
/// polls.
///
/// Built and owned by [`crate::Runner`] - there's no public constructor, since a
/// `Captain` only ever exists as part of one. Because different topics can hold
/// different types, each is stored type-erased and recovered via
/// [`topic`](Self::topic).
///
/// This is also where the run/stop signals live: since [`crate::Executor::run`]
/// only receives a `&Captain`, an executor's run loop polls
/// [`is_running`](Self::is_running) with its own id to know when to exit - either
/// because every executor was told to stop, or because just that one was (e.g. by
/// [`crate::Runner::switch_executor`]).
pub struct Captain {
    topics: RwLock<HashMap<String, Arc<dyn Any + Send + Sync>>>,
    /// Every topic in `topics`, again, but behind the type-erased [`DebugTopic`] handle instead of
    /// `dyn Any` - populated at the same time as `topics` (see [`topic_or_register`](Self::topic_or_register)),
    /// so [`crate::core::debug_executor::DebugExecutor`] can enumerate every topic's name/writer/value
    /// without knowing each one's concrete type.
    debug_topics: RwLock<HashMap<String, Arc<dyn DebugTopic>>>,
    running: AtomicBool,
    /// One flag per possible executor id - indexed directly, so the check every
    /// executor makes each tick in [`is_running`](Self::is_running) stays lock-free.
    executor_running: Box<[AtomicBool]>,
    names: Mutex<HashMap<u16, String>>,
    verbose: AtomicBool,
    restart_requested: AtomicBool,
    /// See [`epoch`](Self::epoch).
    epoch: u64,
    /// Where [`spawn_group`](Self::spawn_group)/[`stop_group`](Self::stop_group) send their
    /// requests, and where [`crate::Runner`] picks them up (see
    /// [`next_runner_request`](Self::next_runner_request)). A fresh captain after a restart
    /// comes with a fresh channel, so a request sent just before one is dropped with it.
    runner_requests: Sender<RunnerRequest>,
    runner_requests_rx: Mutex<Receiver<RunnerRequest>>,
}

impl Captain {
    /// Creates an empty, running captain.
    pub(crate) fn new() -> Self {
        let (runner_requests, runner_requests_rx) = mpsc::channel();
        Self {
            topics: RwLock::new(HashMap::new()),
            debug_topics: RwLock::new(HashMap::new()),
            running: AtomicBool::new(true),
            executor_running: (0..=u16::MAX).map(|_| AtomicBool::new(true)).collect(),
            names: Mutex::new(HashMap::new()),
            verbose: AtomicBool::new(false),
            restart_requested: AtomicBool::new(false),
            epoch: next_epoch(),
            runner_requests,
            runner_requests_rx: Mutex::new(runner_requests_rx),
        }
    }

    /// Identifies this captain - and so this generation of topics - among every captain this
    /// process (or, practically, any earlier run of it) has built. A restart
    /// ([`request_restart`](Self::request_restart)) builds a fresh captain whose topics'
    /// [`WriteMeta::write_count`]s start over from `0`, so a `write_count` on its own can't tell
    /// "unchanged" apart from "rewritten the same number of times since a restart"; the pair
    /// `(epoch, write_count)` can, e.g. for a web client asking whether its cached copy of a
    /// topic is still current.
    pub fn epoch(&self) -> u64 {
        self.epoch
    }

    /// Turns verbose logging on or off, mirroring [`crate::Runner`]'s own flag, so
    /// [`claim_writer`](Self::claim_writer)'s auto-registration can print the same
    /// `"registered topic ..."` line [`crate::Runner::register_topic`] always has.
    pub(crate) fn set_verbose(&self, verbose: bool) {
        self.verbose.store(verbose, Ordering::Relaxed);
    }

    /// Prints `message` with the `[Runner hh:mm:ss] - ` prefix, in bold `color`, if
    /// verbose logging is on.
    fn log_if_verbose(&self, color: LogColor, message: impl fmt::Display) {
        if self.verbose.load(Ordering::Relaxed) {
            println!("{}", crate::core::log::format_line(color, message));
        }
    }

    /// Prints `message` in bold red and terminates the whole program immediately.
    /// Used for fatal misconfigurations (a writer conflict, a missing/mistyped
    /// topic) that a single executor's thread can't safely recover from - exiting
    /// the process outright, rather than just panicking that one thread, avoids
    /// [`crate::Runner`] then panicking a second time when it joins that thread
    /// (see [`crate::Runner::join_all`]/[`crate::Runner::switch_executor`]).
    fn fatal(message: impl fmt::Display) -> ! {
        eprintln!("{}", crate::core::log::format_line(LogColor::Red, message));
        std::process::exit(1);
    }

    /// The name of the executor running on the current thread, i.e. whichever
    /// executor's [`crate::Executor::run`] is currently on the call stack -
    /// [`crate::Runner::spawn`] always names an executor's thread after
    /// [`crate::Executor::name`]. Used only for diagnostics where no executor id is
    /// otherwise available (unlike [`claim_writer`](Self::claim_writer), which
    /// already has one and uses [`name_of`](Self::name_of) instead).
    fn current_executor_name() -> String {
        std::thread::current()
            .name()
            .unwrap_or("<unknown thread>")
            .to_string()
    }

    /// Records `id`'s executor name, so [`claim_writer`](Self::claim_writer) can
    /// name it in a diagnostic without the caller having to pass it every time.
    /// Called by [`crate::Runner`] once per (re)spawned executor, before its
    /// thread starts.
    pub(crate) fn set_name(&self, id: u16, name: String) {
        self.names.lock().unwrap().insert(id, name);
    }

    /// The name previously recorded for `id` via [`set_name`](Self::set_name), or
    /// a placeholder if none was (which shouldn't happen for any id an executor
    /// is actually running under).
    pub(crate) fn name_of(&self, id: u16) -> String {
        self.names
            .lock()
            .unwrap()
            .get(&id)
            .cloned()
            .unwrap_or_else(|| format!("executor {id}"))
    }

    /// A snapshot of every currently registered topic's name and debug-recording handle - a clone of
    /// every `Arc` taken under the lock and then released immediately, so a caller's subsequent
    /// read/serialize work (potentially slow: one `read_encoded()` per topic) never holds up any other
    /// executor's `register_topic`/`claim_writer`/`topic` call. Used by
    /// [`crate::core::debug_executor::DebugExecutor`] to enumerate every topic without knowing any of
    /// their concrete types.
    pub(crate) fn debug_topics_snapshot(&self) -> Vec<(String, Arc<dyn DebugTopic>)> {
        self.debug_topics
            .read()
            .unwrap()
            .iter()
            .map(|(name, topic)| (name.clone(), topic.clone()))
            .collect()
    }

    /// Registers a new topic under `name`, seeded with `initial`.
    pub(crate) fn register_topic<
        T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
    >(
        &self,
        name: impl Into<String>,
        initial: T,
    ) {
        let name = name.into();
        let topic: Arc<RwLockTopic<T>> = Arc::new(RwLockTopic::new(initial));
        self.topics
            .write()
            .unwrap()
            .insert(name.clone(), topic.clone());
        self.debug_topics.write().unwrap().insert(name, topic);
    }

    /// Looks up a previously registered topic.
    ///
    /// Terminates the whole program (see [`fatal`](Self::fatal)) if no topic is
    /// registered under `name`, or if it was registered with a different item type
    /// than `T`. Both are programmer errors (a mismatch between how a topic was
    /// registered and how it's used, e.g. a typo'd name, or reading before any
    /// writer has claimed it), not runtime data conditions a caller could
    /// meaningfully recover from.
    pub fn topic<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
        &self,
        name: &str,
    ) -> Arc<RwLockTopic<T>> {
        let erased = self
            .topics
            .read()
            .unwrap()
            .get(name)
            .cloned()
            .unwrap_or_else(|| {
                Self::fatal(format!(
                    "fatal: executor {:?} tried to read topic {name:?}, but no topic was ever \
                 registered under that name - check for a typo, or make sure its writer's \
                 claim_writing_topics (which auto-registers it) runs before this executor's \
                 thread starts. Stopping.",
                    Self::current_executor_name(),
                ));
            });
        erased.downcast::<RwLockTopic<T>>().unwrap_or_else(|_| {
            Self::fatal(format!(
                "fatal: executor {:?} tried to read topic {name:?}, but it was registered with \
                 a different item type - two executors must agree on a topic's type. Stopping.",
                Self::current_executor_name(),
            ));
        })
    }

    /// Like [`topic`](Self::topic), but returns `None` instead of terminating the program when no
    /// topic is registered under `name`, or when it was registered with a type other than `T` -
    /// for a caller discovering topics by name (e.g. every topic sharing a prefix) that
    /// can't be sure a topic with a matching name also has the expected type, and would rather
    /// skip it than take the whole process down.
    pub fn try_topic<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
        &self,
        name: &str,
    ) -> Option<Arc<RwLockTopic<T>>> {
        let erased = self.topics.read().unwrap().get(name).cloned()?;
        erased.downcast::<RwLockTopic<T>>().ok()
    }

    /// Looks up `name`, or registers it - seeded by calling `initial` - the first
    /// time anyone asks for it, printing the same `"registered topic ..."` verbose
    /// line [`crate::Runner::register_topic`] always has. `initial` is only ever
    /// called when actually registering, so re-claiming an already-registered topic
    /// (e.g. after [`crate::Runner::switch_executor`] restarts an executor) never
    /// re-runs it.
    fn topic_or_register<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
        &self,
        name: &str,
        executor_id: u16,
        initial: impl FnOnce() -> T,
    ) -> Arc<RwLockTopic<T>> {
        if let Some(existing) = self.topics.read().unwrap().get(name) {
            return existing
                .clone()
                .downcast::<RwLockTopic<T>>()
                .unwrap_or_else(|_| {
                    Self::fatal(format!(
                        "fatal: executor {:?} tried to claim topic {name:?}, but it was registered \
                     with a different item type - two executors must agree on a topic's type. \
                     Stopping.",
                        Self::current_executor_name(),
                    ));
                });
        }

        let mut topics = self.topics.write().unwrap();
        if let Some(existing) = topics.get(name) {
            return existing
                .clone()
                .downcast::<RwLockTopic<T>>()
                .unwrap_or_else(|_| {
                    Self::fatal(format!(
                        "fatal: executor {:?} tried to claim topic {name:?}, but it was registered \
                     with a different item type - two executors must agree on a topic's type. \
                     Stopping.",
                        Self::current_executor_name(),
                    ));
                });
        }

        self.log_if_verbose(
            LogColor::Pink,
            format!(
                "{} registered topic {name:?} [{}]",
                self.name_of(executor_id),
                crate::core::log::short_type_name::<T>(),
            ),
        );
        let topic: Arc<RwLockTopic<T>> = Arc::new(RwLockTopic::new(initial()));
        topics.insert(name.to_string(), topic.clone());
        self.debug_topics
            .write()
            .unwrap()
            .insert(name.to_string(), topic.clone());
        topic
    }

    /// Claims `topic_name`'s writer slot for `executor_id` (see
    /// [`RwLockTopic::set_writer`]), auto-registering the topic - seeded by calling
    /// `initial` - if this is the first claim anyone has made on it. If a
    /// *different* executor already holds the writer slot, this is a fatal
    /// misconfiguration - two executors must never both write the same topic - so
    /// this prints a clear explanation (naming both executors, looked up via
    /// [`set_name`](Self::set_name)) and terminates the whole program immediately,
    /// rather than letting just this executor's thread panic (which
    /// [`crate::Runner`] would otherwise have to detect and re-panic on when it
    /// joins the thread).
    pub fn claim_writer<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
        &self,
        topic_name: &str,
        executor_id: u16,
        initial: impl FnOnce() -> T,
    ) -> Arc<RwLockTopic<T>> {
        let topic = self.topic_or_register::<T>(topic_name, executor_id, initial);
        if topic.set_writer(executor_id).is_err() {
            let holder_id = topic
                .writer()
                .expect("a writer must be set if set_writer failed");
            Self::fatal(format!(
                "fatal: executor {:?} (id {executor_id}) tried to become the writer of \
                 topic {topic_name:?}, but executor {:?} (id {holder_id}) already holds \
                 it - two different executors must not both write the same topic. \
                 Stopping.",
                self.name_of(executor_id),
                self.name_of(holder_id),
            ));
        }
        topic
    }

    /// Removes every topic whose writer is one of `ids` - e.g. those of a group of executors
    /// that were just stopped for good (see [`stop_group`](Self::stop_group)), so they no longer
    /// show up in the Topics panel or the debug recording. Anyone still holding one of them keeps
    /// a working, but now orphaned, handle.
    pub(crate) fn unregister_topics_written_by(&self, ids: &[u16]) {
        let mut topics = self.topics.write().unwrap();
        let mut debug_topics = self.debug_topics.write().unwrap();
        debug_topics.retain(|name, topic| {
            let keep = topic.writer().is_none_or(|writer| !ids.contains(&writer));
            if !keep {
                topics.remove(name);
                self.log_if_verbose(LogColor::Pink, format!("unregistered topic {name:?}"));
            }
            keep
        });
    }

    /// Asks [`crate::Runner`] to start `executors` while everything else keeps running, as one
    /// group called `group` - stopped together later by [`stop_group`](Self::stop_group). Every
    /// one of them claims its topics before any of them starts, like at startup (see
    /// [`crate::Runner::run_all`]), so they may read each other's topics right away.
    ///
    /// Only acted on by [`crate::Runner::run_until_stopped`], within a few tens of
    /// milliseconds - not by a bare [`crate::Runner::run_all`]. A group is never brought back by
    /// a restart ([`request_restart`](Self::request_restart)): only the executors added before
    /// running are. A second group under a name already running is refused (logged, dropped).
    /// Each executor gets a new id: ids are never reused.
    pub fn spawn_group(&self, group: impl Into<String>, executors: Vec<Box<dyn crate::Executor>>) {
        // The receiver lives as long as `self`, so sending can't fail.
        let _ = self.runner_requests.send(RunnerRequest::Spawn {
            group: group.into(),
            executors,
        });
    }

    /// Asks [`crate::Runner`] to stop every executor of the group `group` started by
    /// [`spawn_group`](Self::spawn_group), wait for them, and unregister every topic they wrote
    /// (see [`unregister_topics_written_by`](Self::unregister_topics_written_by)). Every other
    /// executor is unaffected. Asking for a group that isn't running does nothing.
    pub fn stop_group(&self, group: impl Into<String>) {
        let _ = self.runner_requests.send(RunnerRequest::Stop {
            group: group.into(),
        });
    }

    /// The next [`RunnerRequest`] sent via [`spawn_group`](Self::spawn_group) or
    /// [`stop_group`](Self::stop_group), waiting up to `timeout` for one. Only
    /// [`crate::Runner`] calls it.
    pub(crate) fn next_runner_request(&self, timeout: Duration) -> Option<RunnerRequest> {
        self.runner_requests_rx
            .lock()
            .unwrap()
            .recv_timeout(timeout)
            .ok()
    }

    /// Whether every executor hasn't been told to stop yet via [`stop`](Self::stop) (or
    /// [`request_restart`](Self::request_restart)).
    pub(crate) fn is_running_at_all(&self) -> bool {
        self.running.load(Ordering::Relaxed)
    }

    /// Whether the executor with this `id` should keep running: true only if
    /// executors haven't all been told to stop via [`stop`](Self::stop), and this
    /// particular id wasn't individually stopped (e.g. for a
    /// [`switch_executor`]-driven swap).
    ///
    /// [`switch_executor`]: crate::Runner::switch_executor
    pub fn is_running(&self, id: u16) -> bool {
        self.running.load(Ordering::Relaxed)
            && self.executor_running[id as usize].load(Ordering::Relaxed)
    }

    /// Signals every executor polling [`is_running`](Self::is_running) to stop.
    pub(crate) fn stop(&self) {
        self.running.store(false, Ordering::Relaxed);
    }

    /// Signals every executor to stop, same as [`stop`](Self::stop), but
    /// additionally marks that [`crate::Runner::run_until_stopped`] should
    /// rebuild everything from scratch - a fresh instance of every executor,
    /// against a fresh [`Captain`] - once they've all stopped, rather than
    /// treating this as a final shutdown. Callable from inside any executor's
    /// own [`crate::Executor::run`] loop.
    pub fn request_restart(&self) {
        self.restart_requested.store(true, Ordering::Relaxed);
        self.stop();
    }

    /// Atomically reads and clears the restart flag set by
    /// [`request_restart`](Self::request_restart). Used by
    /// [`crate::Runner::run_until_stopped`] to detect a restart without
    /// racing a second request that arrives while it's already rebuilding.
    pub(crate) fn take_restart_requested(&self) -> bool {
        self.restart_requested.swap(false, Ordering::Relaxed)
    }

    /// Signals just the executor running under `id` to stop, leaving every other
    /// executor unaffected.
    pub(crate) fn stop_executor(&self, id: u16) {
        self.executor_running[id as usize].store(false, Ordering::Relaxed);
    }

    /// Clears a prior [`stop_executor`](Self::stop_executor) signal for `id`, so a
    /// newly (re)started executor with that id sees [`is_running`](Self::is_running)
    /// as true again.
    pub(crate) fn resume_executor(&self, id: u16) {
        self.executor_running[id as usize].store(true, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn try_topic_is_none_for_a_missing_or_differently_typed_topic() {
        let captain = Captain::new();
        captain.register_topic("count", 7u32);

        assert!(captain.try_topic::<u32>("missing").is_none());
        assert!(captain.try_topic::<String>("count").is_none());
        assert_eq!(captain.try_topic::<u32>("count").unwrap().read().value, 7);
    }

    #[test]
    fn unregistering_removes_only_the_given_writers_topics() {
        let captain = Captain::new();
        captain.set_name(1, "Gone".to_string());
        captain.set_name(2, "Kept".to_string());
        captain.claim_writer::<u32>("gone", 1, || 0);
        captain.claim_writer::<String>("also_gone", 1, String::new);
        captain.claim_writer::<u32>("kept", 2, || 0);

        captain.unregister_topics_written_by(&[1]);

        assert!(captain.try_topic::<u32>("gone").is_none());
        assert!(captain.try_topic::<String>("also_gone").is_none());
        assert!(captain.try_topic::<u32>("kept").is_some());
        let names: Vec<String> = captain
            .debug_topics_snapshot()
            .into_iter()
            .map(|(name, _)| name)
            .collect();
        assert_eq!(names, vec!["kept".to_string()]);
    }

    #[test]
    fn every_captain_gets_a_later_epoch_than_the_one_before() {
        let first = Captain::new().epoch();
        let second = Captain::new().epoch();
        assert!(second > first);
    }

    #[test]
    fn read_json_gives_up_on_values_over_the_size_limit() {
        let captain = Captain::new();
        captain.register_topic("small", vec![1u8, 2, 3]);
        captain.register_topic("large", vec![0u8; 10_000]);
        let topics: HashMap<String, Arc<dyn DebugTopic>> =
            captain.debug_topics_snapshot().into_iter().collect();

        let (small, _) = topics["small"].read_json(1024);
        let (large, _) = topics["large"].read_json(1024);

        assert_eq!(small, Some(serde_json::json!([1, 2, 3])));
        assert_eq!(large, None);
    }
}
