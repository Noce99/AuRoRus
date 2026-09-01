//! [`Captain`] is what each running [`crate::Executor`] answers to: it owns every
//! topic plus the run/stop signals an executor polls to know when to exit.

use crate::core::log::LogColor;
use crate::core::topic::RwLockTopic;
use serde::Serialize;
use serde::de::DeserializeOwned;
use std::any::Any;
use std::collections::HashMap;
use std::fmt;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, RwLock};

/// Type-erased access to one topic's writer/value, for the debug-recording
/// [`crate::core::debug_executor::DebugExecutor`] - implemented generically below for every
/// [`RwLockTopic<T>`] whose `T` satisfies the bound every topic type must now meet (see
/// [`Captain::claim_writer`]/[`Captain::register_topic`]). Lives here, not in [`crate::core::topic`],
/// so `topic.rs` itself stays free of any bincode/serde dependency.
pub(crate) trait DebugTopic: Send + Sync {
    /// See [`RwLockTopic::writer`]. Named identically on purpose - there's no ambiguity in practice,
    /// since this is only ever called through `Arc<dyn DebugTopic>`, never on a concrete `RwLockTopic<T>`.
    fn writer(&self) -> Option<u8>;
    /// The topic's current value, bincode-encoded - what [`crate::core::debug_executor::DebugExecutor`]
    /// writes to the `.debug` file (after checking it differs from what it last recorded).
    fn read_encoded(&self) -> Vec<u8>;
}

impl<T: Clone + Send + Sync + Serialize + 'static> DebugTopic for RwLockTopic<T> {
    fn writer(&self) -> Option<u8> {
        RwLockTopic::writer(self)
    }

    fn read_encoded(&self) -> Vec<u8> {
        bincode::serde::encode_to_vec(self.read(), bincode::config::standard())
            .expect("encoding a topic's current value for the debug recorder should never fail")
    }
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
    executor_running: [AtomicBool; 256],
    names: Mutex<HashMap<u8, String>>,
    verbose: AtomicBool,
    restart_requested: AtomicBool,
}

impl Captain {
    /// Creates an empty, running captain.
    pub(crate) fn new() -> Self {
        Self {
            topics: RwLock::new(HashMap::new()),
            debug_topics: RwLock::new(HashMap::new()),
            running: AtomicBool::new(true),
            executor_running: std::array::from_fn(|_| AtomicBool::new(true)),
            names: Mutex::new(HashMap::new()),
            verbose: AtomicBool::new(false),
            restart_requested: AtomicBool::new(false),
        }
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
    pub(crate) fn set_name(&self, id: u8, name: String) {
        self.names.lock().unwrap().insert(id, name);
    }

    /// The name previously recorded for `id` via [`set_name`](Self::set_name), or
    /// a placeholder if none was (which shouldn't happen for any id an executor
    /// is actually running under).
    pub(crate) fn name_of(&self, id: u8) -> String {
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
        self.debug_topics.read().unwrap().iter().map(|(name, topic)| (name.clone(), topic.clone())).collect()
    }

    /// Registers a new topic under `name`, seeded with `initial`.
    pub(crate) fn register_topic<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
        &self,
        name: impl Into<String>,
        initial: T,
    ) {
        let name = name.into();
        let topic: Arc<RwLockTopic<T>> = Arc::new(RwLockTopic::new(initial));
        self.topics.write().unwrap().insert(name.clone(), topic.clone());
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
    pub fn topic<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(&self, name: &str) -> Arc<RwLockTopic<T>> {
        let erased = self.topics.read().unwrap().get(name).cloned().unwrap_or_else(|| {
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

    /// Looks up `name`, or registers it - seeded by calling `initial` - the first
    /// time anyone asks for it, printing the same `"registered topic ..."` verbose
    /// line [`crate::Runner::register_topic`] always has. `initial` is only ever
    /// called when actually registering, so re-claiming an already-registered topic
    /// (e.g. after [`crate::Runner::switch_executor`] restarts an executor) never
    /// re-runs it.
    fn topic_or_register<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
        &self,
        name: &str,
        initial: impl FnOnce() -> T,
    ) -> Arc<RwLockTopic<T>> {
        if let Some(existing) = self.topics.read().unwrap().get(name) {
            return existing.clone().downcast::<RwLockTopic<T>>().unwrap_or_else(|_| {
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
            return existing.clone().downcast::<RwLockTopic<T>>().unwrap_or_else(|_| {
                Self::fatal(format!(
                    "fatal: executor {:?} tried to claim topic {name:?}, but it was registered \
                     with a different item type - two executors must agree on a topic's type. \
                     Stopping.",
                    Self::current_executor_name(),
                ));
            });
        }

        self.log_if_verbose(LogColor::Pink, format!("registered topic {name:?}"));
        let topic: Arc<RwLockTopic<T>> = Arc::new(RwLockTopic::new(initial()));
        topics.insert(name.to_string(), topic.clone());
        self.debug_topics.write().unwrap().insert(name.to_string(), topic.clone());
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
        executor_id: u8,
        initial: impl FnOnce() -> T,
    ) -> Arc<RwLockTopic<T>> {
        let topic = self.topic_or_register::<T>(topic_name, initial);
        if topic.set_writer(executor_id).is_err() {
            let holder_id = topic.writer().expect("a writer must be set if set_writer failed");
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

    /// Whether the executor with this `id` should keep running: true only if every
    /// executor was told to stop via [`stop`](Self::stop), and this particular id
    /// wasn't individually stopped (e.g. for a [`switch_executor`]-driven swap).
    ///
    /// [`switch_executor`]: crate::Runner::switch_executor
    pub fn is_running(&self, id: u8) -> bool {
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
    pub(crate) fn stop_executor(&self, id: u8) {
        self.executor_running[id as usize].store(false, Ordering::Relaxed);
    }

    /// Clears a prior [`stop_executor`](Self::stop_executor) signal for `id`, so a
    /// newly (re)started executor with that id sees [`is_running`](Self::is_running)
    /// as true again.
    pub(crate) fn resume_executor(&self, id: u8) {
        self.executor_running[id as usize].store(true, Ordering::Relaxed);
    }
}
