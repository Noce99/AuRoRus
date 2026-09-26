//! [`Runner`] owns every [`Executor`] and every topic, and runs the executors in
//! parallel against a shared [`Captain`].

use crate::core::captain::{Captain, RunnerRequest};
use crate::core::debug_executor::DebugExecutor;
use crate::core::executor::Executor;
use crate::core::log::{self, LogColor};
use crate::core::topic::RwLockTopic;
use std::collections::{HashMap, HashSet};
use std::fmt;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

/// How long [`Runner::run_until_stopped`] waits for a [`RunnerRequest`] before checking again
/// whether everything was told to stop - bounds how late it notices a stop.
const REQUEST_POLL_INTERVAL: Duration = Duration::from_millis(50);

/// A cheap, `Clone`-able handle that can signal every executor to stop from any
/// thread - e.g. a Ctrl+C handler running on its own OS thread while
/// [`Runner::run_until_stopped`] blocks the caller's thread. Deliberately exposes
/// only [`stop`](Self::stop), not the rest of [`Captain`]'s surface.
///
/// Keeps working across restarts ([`Captain::request_restart`]): it stops
/// whichever [`Captain`] is current, and a stop that lands while
/// [`Runner::run_until_stopped`] is rebuilding everything still ends it.
#[derive(Clone)]
pub struct StopHandle(Arc<SharedStop>);

/// What a [`Runner`] and its [`StopHandle`]s share.
struct SharedStop {
    /// The [`Captain`] currently running - replaced on every restart.
    current: Mutex<Arc<Captain>>,
    /// Set by [`StopHandle::stop`]: a final stop, never a restart.
    stopped: AtomicBool,
}

impl StopHandle {
    /// Same effect as [`Runner::stop`], callable from any thread without a `&Runner` -
    /// except that it's always final: a restart already requested is dropped.
    pub fn stop(&self) {
        // Set before stopping the current captain, so a rebuild swapping in a
        // new one right now sees it (see `Runner::run_until_stopped`).
        self.0.stopped.store(true, Ordering::SeqCst);
        self.0.current.lock().unwrap().stop();
    }
}

/// Error returned by [`Runner::switch_executor`].
#[derive(Debug)]
pub enum SwitchExecutorError {
    /// No executor is currently running under this id - either it was never
    /// registered, or [`Runner::run_all`] hasn't started it yet.
    NotRunning(u16),
}

impl fmt::Display for SwitchExecutorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotRunning(id) => write!(f, "no executor is currently running with id {id}"),
        }
    }
}

impl std::error::Error for SwitchExecutorError {}

/// Replays one [`Runner::register_topic`] call against a rebuilt [`Captain`],
/// so a restart in [`Runner::run_until_stopped`] can bring pre-seeded topics
/// back too.
type TopicRegistration = Box<dyn Fn(&Captain) + Send + Sync>;

/// Owns every topic and every registered [`Executor`], and drives them all.
///
/// Register topics with [`register_topic`](Self::register_topic) and executors
/// with [`add_executor`](Self::add_executor), then call [`run_all`](Self::run_all)
/// once everything is ready. Each executor runs on its own thread until
/// [`stop`](Self::stop) (or [`switch_executor`](Self::switch_executor), for just
/// one executor) tells it to. Call [`join_all`](Self::join_all) to wait for
/// whatever is still running.
pub struct Runner {
    captain: Arc<Captain>,
    next_id: u16,
    pending: Vec<(u16, Box<dyn Executor>)>,
    running: HashMap<u16, thread::JoinHandle<Box<dyn Executor>>>,
    registered_topics: Vec<TopicRegistration>,
    verbose: bool,
    /// Set by [`debug_mode`](Self::debug_mode); `None` means debug recording is off.
    /// Passed to every executor's [`Executor::set_debug_mode`] as it's spawned.
    debug_frequency_hz: Option<f64>,
    /// The ids of every group of executors started by [`Captain::spawn_group`], by group name.
    groups: HashMap<String, Vec<u16>>,
    /// Shared with every [`StopHandle`] - see [`stop_handle`](Self::stop_handle).
    shared_stop: Arc<SharedStop>,
}

impl Runner {
    /// Creates an empty runner with no topics or executors registered yet.
    /// Verbose logging is off by default - see [`activate_verbose`](Self::activate_verbose).
    pub fn new() -> Self {
        let captain = Arc::new(Captain::new());
        Self {
            shared_stop: Arc::new(SharedStop {
                current: Mutex::new(captain.clone()),
                stopped: AtomicBool::new(false),
            }),
            captain,
            next_id: 0,
            pending: Vec::new(),
            running: HashMap::new(),
            registered_topics: Vec::new(),
            verbose: false,
            debug_frequency_hz: None,
            groups: HashMap::new(),
        }
    }

    /// Turns on debug recording: every topic currently being written is
    /// snapshotted at `frequency_hz` (only when its value actually changed) into
    /// a new/resumed session file at `output_path`, via a [`crate::core::debug_executor::DebugExecutor`]
    /// this adds automatically, right alongside every other executor.
    ///
    /// Consuming (`self -> Self`) rather than `&mut self`, unlike the rest of
    /// `Runner`'s builder-ish methods, to match the call shape
    /// `Runner::new().debug_mode(hz, path)` - it's still fine to chain further
    /// `&mut self` calls (`activate_verbose`, `add_executor`, ...) on the result
    /// afterward, since `Runner::new()` still returns `Self` by value.
    ///
    /// Takes both `frequency_hz` and `output_path` together, rather than a second
    /// call/method for the path, because they're always known at the same call
    /// site once a binary's CLI has resolved its `--debug`/`--debug_frequency`
    /// flags - splitting them would only invite an inconsistent state (frequency
    /// set, no path, or vice versa) with no benefit.
    pub fn debug_mode(mut self, frequency_hz: f64, output_path: impl Into<PathBuf>) -> Self {
        self.debug_frequency_hz = Some(frequency_hz);
        self.add_executor(DebugExecutor::new(output_path.into(), frequency_hz).boxed());
        self
    }

    /// A [`StopHandle`] for this runner, e.g. to wire up a Ctrl+C handler before
    /// calling [`run_until_stopped`](Self::run_until_stopped), which otherwise
    /// blocks the calling thread.
    pub fn stop_handle(&self) -> StopHandle {
        StopHandle(self.shared_stop.clone())
    }

    /// Turns on verbose logging: every subsequent call to a method below prints
    /// one `[Runner hh:mm:ss] - ...` line (local time) to stdout describing what
    /// it did. Off by default; there's no way to turn it back off.
    pub fn activate_verbose(&mut self) {
        self.verbose = true;
        self.captain.set_verbose(true);
    }

    /// Prints `message` with the `[Runner hh:mm:ss] - ` prefix, in bold `color`,
    /// if verbose logging is on.
    fn log(&self, color: LogColor, message: impl fmt::Display) {
        if self.verbose {
            println!("{}", log::format_line(color, message));
        }
    }

    /// Registers a new topic under `name`, seeded by calling `initial`. Usually
    /// unnecessary now that [`crate::Executor::claim_writing_topics`] auto-registers
    /// a topic the first time its writer claims it - use this only to pre-seed a
    /// topic that has no writer.
    ///
    /// `initial` is a factory rather than a bare value so
    /// [`run_until_stopped`](Self::run_until_stopped) can replay this
    /// registration against the fresh [`Captain`] a restart builds.
    pub fn register_topic<
        T: Clone + Send + Sync + serde::Serialize + serde::de::DeserializeOwned + 'static,
    >(
        &mut self,
        name: impl Into<String>,
        initial: impl Fn() -> T + Send + Sync + 'static,
    ) {
        let name = name.into();
        self.log(
            LogColor::Pink,
            format!(
                "Runner registered topic {name:?} [{}]",
                log::short_type_name::<T>()
            ),
        );
        self.captain.register_topic(name.clone(), initial());
        self.registered_topics
            .push(Box::new(move |captain: &Captain| {
                captain.register_topic(name.clone(), initial());
            }));
    }

    /// Looks up a previously registered topic, e.g. to read it after every
    /// executor has stopped. See [`Captain::topic`] for panic conditions.
    pub fn topic<
        T: Clone + Send + Sync + serde::Serialize + serde::de::DeserializeOwned + 'static,
    >(
        &self,
        name: &str,
    ) -> Arc<RwLockTopic<T>> {
        self.captain.topic(name)
    }

    /// Signals every executor to stop.
    pub fn stop(&self) {
        self.log(LogColor::Yellow, "stop requested");
        self.captain.stop();
    }

    /// Registers `executor`, assigning it a unique id (in registration order,
    /// starting at 0). The id is passed to the executor's [`Executor::init`]
    /// once it starts running, and returned here in case the caller needs it.
    pub fn add_executor(&mut self, executor: Box<dyn Executor>) -> u16 {
        let id = self.next_id;
        self.next_id = self
            .next_id
            .checked_add(1)
            .expect("Runner: exceeded u16::MAX executors (id space exhausted)");
        self.log(
            LogColor::Orange,
            format!("added executor {:?} (id {id})", executor.name()),
        );
        self.pending.push((id, executor));
        id
    }

    /// Spawns one thread per executor added since the last call to `run_all`: each
    /// calls [`Executor::init`] then [`Executor::run`]. Safe to call again later to
    /// start executors added afterward.
    ///
    /// Every executor claims its writing topics before *any* of them starts running,
    /// so an executor that reads another's topic at the top of its `run` always finds
    /// it registered, whatever order they were added in - which matters after a
    /// restart, when [`run_until_stopped`](Self::run_until_stopped) re-adds them in
    /// no particular order.
    pub fn run_all(&mut self) {
        let prepared: Vec<_> = self
            .pending
            .drain(..)
            .map(|(id, executor)| {
                (
                    id,
                    Self::prepare(&self.captain, id, executor, self.debug_frequency_hz),
                )
            })
            .collect();
        for (id, executor) in prepared {
            self.running
                .insert(id, Self::start(&self.captain, executor));
        }
        self.log(LogColor::Green, "all executors started");
    }

    /// Stops the executor currently running under `id`, waits for its thread to
    /// finish, and starts `new_executor` in its place under that same id. Every
    /// other running executor is unaffected.
    ///
    /// Returns the outgoing executor, e.g. to downcast via [`Executor::as_any`] and
    /// inspect its final state. Fails if no executor is currently running under `id`.
    pub fn switch_executor(
        &mut self,
        id: u16,
        new_executor: Box<dyn Executor>,
    ) -> Result<Box<dyn Executor>, SwitchExecutorError> {
        let old_handle = self
            .running
            .remove(&id)
            .ok_or(SwitchExecutorError::NotRunning(id))?;

        self.log(
            LogColor::Purple,
            format!(
                "switching executor [id={id}] [{}->{}]...",
                self.captain.name_of(id),
                new_executor.name()
            ),
        );

        self.captain.stop_executor(id);
        let old_executor = old_handle.join().expect("executor thread panicked");
        self.captain.resume_executor(id);

        let handle = Self::spawn(&self.captain, id, new_executor, self.debug_frequency_hz);
        self.running.insert(id, handle);

        Ok(old_executor)
    }

    /// Waits for every currently running executor to finish, returning each one.
    /// Typically called after [`stop`](Self::stop) has signaled them all to exit.
    pub fn join_all(&mut self) -> Vec<Box<dyn Executor>> {
        self.join_all_with_ids()
            .into_iter()
            .map(|(_, executor)| executor)
            .collect()
    }

    /// Runs every registered executor until a final [`stop`](Self::stop) -
    /// handling any number of [`Captain::request_restart`] calls along the
    /// way by rebuilding a fresh [`Captain`] (fresh topics, replaying every
    /// [`register_topic`](Self::register_topic) call) and a fresh instance of
    /// every executor (via [`Executor::fresh`], same ids), then starting
    /// over. Returns the finished executors once a stop was *not*
    /// accompanied by a restart request - same contract as
    /// [`join_all`](Self::join_all) today.
    ///
    /// Meanwhile, starts and stops groups of executors as
    /// [`Captain::spawn_group`]/[`Captain::stop_group`] ask. A restart doesn't
    /// bring any group back - only the executors added via
    /// [`add_executor`](Self::add_executor).
    pub fn run_until_stopped(&mut self) -> Vec<Box<dyn Executor>> {
        loop {
            self.run_all();
            while self.captain.is_running_at_all() {
                if let Some(request) = self.captain.next_runner_request(REQUEST_POLL_INTERVAL) {
                    self.handle(request);
                }
            }
            let finished = self.join_all_with_ids();
            if !self.captain.take_restart_requested() {
                return finished.into_iter().map(|(_, executor)| executor).collect();
            }
            self.log(
                LogColor::Green,
                "restart requested - rebuilding all executors",
            );

            self.captain = Arc::new(Captain::new());
            *self.shared_stop.current.lock().unwrap() = self.captain.clone();
            // A `StopHandle::stop` that landed during this restart stopped
            // the old captain: end here rather than starting over. One from
            // now on stops the new one.
            if self.shared_stop.stopped.load(Ordering::SeqCst) {
                return finished.into_iter().map(|(_, executor)| executor).collect();
            }
            self.captain.set_verbose(self.verbose);
            for register in &self.registered_topics {
                register(&self.captain);
            }
            let grouped: HashSet<u16> = self.groups.drain().flat_map(|(_, ids)| ids).collect();
            for (id, executor) in finished {
                if !grouped.contains(&id) {
                    self.pending.push((id, executor.fresh()));
                }
            }
        }
    }

    /// Acts on one [`RunnerRequest`] - see [`Captain::spawn_group`] and [`Captain::stop_group`].
    fn handle(&mut self, request: RunnerRequest) {
        match request {
            RunnerRequest::Spawn { group, executors } => {
                if self.groups.contains_key(&group) {
                    self.log(
                        LogColor::Red,
                        format!("group {group:?} is already running - not starting it again"),
                    );
                    return;
                }
                let ids = executors
                    .into_iter()
                    .map(|executor| self.add_executor(executor))
                    .collect();
                self.run_all();
                self.log(LogColor::Green, format!("started group {group:?}"));
                self.groups.insert(group, ids);
            }
            RunnerRequest::Stop { group } => {
                let Some(ids) = self.groups.remove(&group) else {
                    self.log(
                        LogColor::Red,
                        format!("group {group:?} isn't running - nothing to stop"),
                    );
                    return;
                };
                // Signal them all first, so they wind down in parallel.
                for &id in &ids {
                    self.captain.stop_executor(id);
                }
                for id in &ids {
                    if let Some(handle) = self.running.remove(id) {
                        handle.join().expect("executor thread panicked");
                    }
                }
                self.captain.unregister_topics_written_by(&ids);
                self.log(LogColor::Green, format!("stopped group {group:?}"));
            }
        }
    }

    /// Like [`join_all`](Self::join_all), but keeps each executor's id
    /// alongside it - needed by [`run_until_stopped`](Self::run_until_stopped)
    /// to respawn restarted executors under their original ids.
    fn join_all_with_ids(&mut self) -> Vec<(u16, Box<dyn Executor>)> {
        let executors: Vec<_> = self
            .running
            .drain()
            .map(|(id, handle)| (id, handle.join().expect("executor thread panicked")))
            .collect();
        self.log(LogColor::Green, "all executors stopped successfully");
        executors
    }

    fn spawn(
        captain: &Arc<Captain>,
        id: u16,
        executor: Box<dyn Executor>,
        debug_frequency_hz: Option<f64>,
    ) -> thread::JoinHandle<Box<dyn Executor>> {
        let executor = Self::prepare(captain, id, executor, debug_frequency_hz);
        Self::start(captain, executor)
    }

    /// Everything [`spawn`](Self::spawn) does before starting the executor's thread:
    /// assigns its id and name, and has it claim its writing topics.
    fn prepare(
        captain: &Captain,
        id: u16,
        mut executor: Box<dyn Executor>,
        debug_frequency_hz: Option<f64>,
    ) -> Box<dyn Executor> {
        executor.init(id);
        captain.set_name(id, executor.name());
        if let Some(hz) = debug_frequency_hz {
            executor.set_debug_mode(hz);
        }
        executor.claim_writing_topics(captain);
        executor
    }

    /// Starts a [`prepare`](Self::prepare)d executor running on its own thread.
    fn start(
        captain: &Arc<Captain>,
        mut executor: Box<dyn Executor>,
    ) -> thread::JoinHandle<Box<dyn Executor>> {
        let name = executor.name();
        let captain = captain.clone();
        thread::Builder::new()
            .name(name)
            .spawn(move || {
                executor.run(&captain);
                executor
            })
            .expect("failed to spawn executor thread")
    }
}

impl Default for Runner {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::any::Any;
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    use std::time::Duration;

    /// A minimal executor that records its assigned id, counts loop iterations,
    /// flags when it's finished, and (once, if `restart_after`/`stop_after` is
    /// set) asks the captain to restart everything or do a final stop after
    /// that many iterations - just enough to observe `switch_executor` and
    /// `run_until_stopped`'s effects.
    struct CountingExecutor {
        id: u16,
        iterations: Arc<AtomicUsize>,
        finished: Arc<AtomicBool>,
        restart_after: Option<usize>,
        stop_after: Option<usize>,
    }

    impl Executor for CountingExecutor {
        fn init(&mut self, id: u16) {
            self.id = id;
        }

        fn run(&mut self, captain: &Captain) {
            while captain.is_running(self.id) {
                let count = self.iterations.fetch_add(1, Ordering::Relaxed) + 1;
                if self.restart_after == Some(count) {
                    captain.request_restart();
                } else if self.stop_after == Some(count) {
                    captain.stop();
                }
                thread::sleep(Duration::from_millis(1));
            }
            self.finished.store(true, Ordering::Relaxed);
        }

        fn name(&self) -> String {
            format!("Counting {}", self.id)
        }

        fn as_any(&self) -> &dyn Any {
            self
        }

        fn fresh(&self) -> Box<dyn Executor> {
            Box::new(CountingExecutor {
                id: self.id,
                iterations: Arc::new(AtomicUsize::new(0)),
                finished: Arc::new(AtomicBool::new(false)),
                restart_after: None,
                stop_after: self.stop_after,
            })
        }
    }

    /// Claims `topic` - taking its time about it, so an executor already running
    /// by then would get to read it first - and does nothing else.
    struct Writer {
        id: u16,
    }

    impl Executor for Writer {
        fn init(&mut self, id: u16) {
            self.id = id;
        }

        fn claim_writing_topics(&mut self, captain: &Captain) {
            thread::sleep(Duration::from_millis(50));
            captain.claim_writer::<u32>("topic", self.id, || 0);
        }

        fn run(&mut self, _captain: &Captain) {}

        fn name(&self) -> String {
            "Writer".to_string()
        }

        fn as_any(&self) -> &dyn Any {
            self
        }

        fn fresh(&self) -> Box<dyn Executor> {
            Box::new(Writer { id: self.id })
        }
    }

    /// Reads `topic` - which it doesn't write - as soon as it starts.
    struct Reader;

    impl Executor for Reader {
        fn init(&mut self, _id: u16) {}

        fn run(&mut self, captain: &Captain) {
            captain.topic::<u32>("topic").read();
        }

        fn name(&self) -> String {
            "Reader".to_string()
        }

        fn as_any(&self) -> &dyn Any {
            self
        }

        fn fresh(&self) -> Box<dyn Executor> {
            Box::new(Reader)
        }
    }

    /// Claims `topic` (a `u32`) and idles until stopped, flagging when it's finished. Never
    /// meant to be restarted: its `fresh` panics, so a test fails if a restart brings it back.
    struct GroupMember {
        id: u16,
        topic: String,
        finished: Arc<AtomicBool>,
    }

    impl Executor for GroupMember {
        fn init(&mut self, id: u16) {
            self.id = id;
        }

        fn claim_writing_topics(&mut self, captain: &Captain) {
            captain.claim_writer::<u32>(&self.topic, self.id, || 0);
        }

        fn run(&mut self, captain: &Captain) {
            while captain.is_running(self.id) {
                thread::sleep(Duration::from_millis(1));
            }
            self.finished.store(true, Ordering::Relaxed);
        }

        fn name(&self) -> String {
            format!("Member {}", self.topic)
        }

        fn as_any(&self) -> &dyn Any {
            self
        }

        fn fresh(&self) -> Box<dyn Executor> {
            panic!("a group's executors must not be brought back by a restart");
        }
    }

    /// Polls `condition` every millisecond for up to a second; whether it became true.
    fn eventually(condition: impl Fn() -> bool) -> bool {
        (0..1000).any(|_| {
            thread::sleep(Duration::from_millis(1));
            condition()
        })
    }

    /// What [`Spawner`] saw, for the test to check once everything stopped.
    #[derive(Default)]
    struct SpawnerLog {
        topics_appeared: AtomicBool,
        topics_disappeared: AtomicBool,
        members_finished: AtomicBool,
    }

    /// Starts a group of two [`GroupMember`]s from inside its own `run`, waits for their topics,
    /// then - if `restart` - requests a restart with the group still running, or else stops the
    /// group, waits for its topics to go away, and stops everything. A fresh one (after a
    /// restart) stops everything right away.
    struct Spawner {
        id: u16,
        restart: bool,
        log: Arc<SpawnerLog>,
        members_finished: [Arc<AtomicBool>; 2],
    }

    impl Executor for Spawner {
        fn init(&mut self, id: u16) {
            self.id = id;
        }

        fn run(&mut self, captain: &Captain) {
            let members = self
                .members_finished
                .iter()
                .enumerate()
                .map(|(i, finished)| {
                    Box::new(GroupMember {
                        id: 0,
                        topic: format!("group/{i}"),
                        finished: finished.clone(),
                    }) as Box<dyn Executor>
                })
                .collect();
            captain.spawn_group("group", members);
            let registered = |name: &str| captain.try_topic::<u32>(name).is_some();
            let appeared = eventually(|| registered("group/0") && registered("group/1"));
            self.log.topics_appeared.store(appeared, Ordering::Relaxed);

            if self.restart {
                captain.request_restart();
                return;
            }

            captain.stop_group("group");
            let gone = eventually(|| !registered("group/0") && !registered("group/1"));
            self.log.topics_disappeared.store(gone, Ordering::Relaxed);
            let finished = self
                .members_finished
                .iter()
                .all(|finished| finished.load(Ordering::Relaxed));
            self.log.members_finished.store(finished, Ordering::Relaxed);
            captain.stop();
        }

        fn name(&self) -> String {
            "Spawner".to_string()
        }

        fn as_any(&self) -> &dyn Any {
            self
        }

        fn fresh(&self) -> Box<dyn Executor> {
            Box::new(Stopper)
        }
    }

    /// Stops everything as soon as it starts.
    struct Stopper;

    impl Executor for Stopper {
        fn init(&mut self, _id: u16) {}

        fn run(&mut self, captain: &Captain) {
            captain.stop();
        }

        fn name(&self) -> String {
            "Stopper".to_string()
        }

        fn as_any(&self) -> &dyn Any {
            self
        }

        fn fresh(&self) -> Box<dyn Executor> {
            Box::new(Stopper)
        }
    }

    fn spawner(restart: bool) -> (Spawner, Arc<SpawnerLog>, [Arc<AtomicBool>; 2]) {
        let log = Arc::new(SpawnerLog::default());
        let members_finished = [
            Arc::new(AtomicBool::new(false)),
            Arc::new(AtomicBool::new(false)),
        ];
        let spawner = Spawner {
            id: 0,
            restart,
            log: log.clone(),
            members_finished: members_finished.clone(),
        };
        (spawner, log, members_finished)
    }

    #[test]
    fn a_group_can_be_started_and_stopped_while_everything_runs() {
        let mut runner = Runner::new();
        let (spawner, log, _) = spawner(false);
        runner.add_executor(Box::new(spawner));
        let other_iterations = Arc::new(AtomicUsize::new(0));
        runner.add_executor(Box::new(CountingExecutor {
            id: 0,
            iterations: other_iterations.clone(),
            finished: Arc::new(AtomicBool::new(false)),
            restart_after: None,
            stop_after: None,
        }));

        let finished = runner.run_until_stopped();

        assert!(log.topics_appeared.load(Ordering::Relaxed));
        assert!(log.topics_disappeared.load(Ordering::Relaxed));
        assert!(log.members_finished.load(Ordering::Relaxed));
        // Stopping the group left everyone else running.
        assert!(other_iterations.load(Ordering::Relaxed) > 0);
        assert_eq!(finished.len(), 2);
        assert!(runner.groups.is_empty());
    }

    #[test]
    fn a_restart_does_not_bring_a_group_back() {
        let mut runner = Runner::new();
        let (spawner, log, members_finished) = spawner(true);
        runner.add_executor(Box::new(spawner));

        // A group member's `fresh` panics, which would fail the test here.
        let finished = runner.run_until_stopped();

        assert!(log.topics_appeared.load(Ordering::Relaxed));
        assert!(members_finished.iter().all(|f| f.load(Ordering::Relaxed)));
        // Only the spawner came back (as a `Stopper`), without its group.
        assert_eq!(finished.len(), 1);
        assert!(finished[0].as_any().is::<Stopper>());
        assert!(runner.groups.is_empty());
    }

    #[test]
    fn stopping_a_group_that_is_not_running_is_harmless() {
        let mut runner = Runner::new();
        runner.captain.stop_group("nothing");
        runner.add_executor(Box::new(Stopper));
        assert_eq!(runner.run_until_stopped().len(), 1);
    }

    /// A reader added before the writer of the topic it reads must still find that
    /// topic registered - reading an unregistered topic terminates the process, which
    /// would fail this whole test binary.
    #[test]
    fn run_all_claims_every_topic_before_starting_any_executor() {
        let mut runner = Runner::new();
        runner.add_executor(Box::new(Reader));
        runner.add_executor(Box::new(Writer { id: 0 }));

        runner.run_all();
        runner.join_all();
    }

    #[test]
    fn switch_executor_replaces_only_the_targeted_id() {
        let mut runner = Runner::new();

        let old_iterations = Arc::new(AtomicUsize::new(0));
        let old_finished = Arc::new(AtomicBool::new(false));
        let other_iterations = Arc::new(AtomicUsize::new(0));

        let target_id = runner.add_executor(Box::new(CountingExecutor {
            id: 0,
            iterations: old_iterations.clone(),
            finished: old_finished.clone(),
            restart_after: None,
            stop_after: None,
        }));
        let other_id = runner.add_executor(Box::new(CountingExecutor {
            id: 0,
            iterations: other_iterations.clone(),
            finished: Arc::new(AtomicBool::new(false)),
            restart_after: None,
            stop_after: None,
        }));

        runner.run_all();
        thread::sleep(Duration::from_millis(20));

        let new_iterations = Arc::new(AtomicUsize::new(0));
        let new_finished = Arc::new(AtomicBool::new(false));
        runner
            .switch_executor(
                target_id,
                Box::new(CountingExecutor {
                    id: 0,
                    iterations: new_iterations.clone(),
                    finished: new_finished.clone(),
                    restart_after: None,
                    stop_after: None,
                }),
            )
            .expect("target executor should still be running");

        // The old instance under target_id must have stopped and finished...
        assert!(old_finished.load(Ordering::Relaxed));
        // ...while the untouched executor kept running the whole time.
        assert!(other_iterations.load(Ordering::Relaxed) > 0);

        thread::sleep(Duration::from_millis(20));
        runner.stop();
        runner.join_all();

        // The new instance under target_id actually ran, under the same id.
        assert!(new_iterations.load(Ordering::Relaxed) > 0);
        assert!(new_finished.load(Ordering::Relaxed));
        assert_eq!(target_id, other_id - 1);
    }

    #[test]
    fn switch_executor_fails_for_an_id_that_is_not_running() {
        let mut runner = Runner::new();

        let result = runner.switch_executor(
            0,
            Box::new(CountingExecutor {
                id: 0,
                iterations: Arc::new(AtomicUsize::new(0)),
                finished: Arc::new(AtomicBool::new(false)),
                restart_after: None,
                stop_after: None,
            }),
        );

        assert!(matches!(result, Err(SwitchExecutorError::NotRunning(0))));
    }

    #[test]
    fn a_stop_handle_still_stops_everything_after_a_restart() {
        let mut runner = Runner::new();
        // Restarts once, then - as a fresh instance, with no `stop_after` -
        // would run forever.
        runner.add_executor(Box::new(CountingExecutor {
            id: 0,
            iterations: Arc::new(AtomicUsize::new(0)),
            finished: Arc::new(AtomicBool::new(false)),
            restart_after: Some(3),
            stop_after: None,
        }));
        let stop_handle = runner.stop_handle();
        let (done_tx, done_rx) = std::sync::mpsc::channel();
        thread::spawn(move || {
            runner.run_until_stopped();
            done_tx.send(()).unwrap();
        });

        // Well after the restart.
        thread::sleep(Duration::from_millis(200));
        stop_handle.stop();

        assert!(
            done_rx.recv_timeout(Duration::from_secs(2)).is_ok(),
            "run_until_stopped should return once the handle stops it"
        );
    }

    #[test]
    fn a_stop_handle_stop_is_final_even_with_a_restart_requested() {
        let mut runner = Runner::new();
        runner.add_executor(Box::new(CountingExecutor {
            id: 0,
            iterations: Arc::new(AtomicUsize::new(0)),
            finished: Arc::new(AtomicBool::new(false)),
            restart_after: None,
            stop_after: None,
        }));
        // Both land before anything runs: the stop wins, no restart.
        runner.captain.request_restart();
        runner.stop_handle().stop();
        assert_eq!(runner.run_until_stopped().len(), 1);
    }

    #[test]
    fn run_until_stopped_gives_every_executor_a_fresh_start_on_restart() {
        let mut runner = Runner::new();

        let old_finished = Arc::new(AtomicBool::new(false));
        let old_iterations = Arc::new(AtomicUsize::new(0));
        let trigger_id = runner.add_executor(Box::new(CountingExecutor {
            id: 0,
            iterations: old_iterations.clone(),
            finished: old_finished.clone(),
            restart_after: Some(3),
            stop_after: Some(20),
        }));
        let other_id = runner.add_executor(Box::new(CountingExecutor {
            id: 0,
            iterations: Arc::new(AtomicUsize::new(0)),
            finished: Arc::new(AtomicBool::new(false)),
            restart_after: None,
            stop_after: None,
        }));

        let finished = runner.run_until_stopped();

        // The pre-restart instance actually stopped...
        assert!(old_finished.load(Ordering::Relaxed));
        assert_eq!(finished.len(), 2);

        let trigger = finished
            .iter()
            .map(|executor| {
                executor
                    .as_any()
                    .downcast_ref::<CountingExecutor>()
                    .unwrap()
            })
            .find(|executor| executor.id == trigger_id)
            .expect("trigger executor should be among the finished ones");

        // ...and the executor returned after the final stop is a genuinely
        // fresh instance (its own counter), not the same one continuing on.
        assert!(!Arc::ptr_eq(&old_iterations, &trigger.iterations));
        assert!(trigger.finished.load(Ordering::Relaxed));

        assert!(finished.iter().any(|executor| {
            executor
                .as_any()
                .downcast_ref::<CountingExecutor>()
                .unwrap()
                .id
                == other_id
        }));
    }
}
