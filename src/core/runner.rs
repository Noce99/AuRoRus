//! [`Runner`] owns every [`Executor`] and every topic, and runs the executors in
//! parallel against a shared [`Captain`].

use crate::core::captain::Captain;
use crate::core::executor::Executor;
use crate::core::log::{self, LogColor};
use crate::core::topic::RwLockTopic;
use std::collections::HashMap;
use std::fmt;
use std::sync::Arc;
use std::thread;

/// Error returned by [`Runner::switch_executor`].
#[derive(Debug)]
pub enum SwitchExecutorError {
    /// No executor is currently running under this id - either it was never
    /// registered, or [`Runner::run_all`] hasn't started it yet.
    NotRunning(u8),
}

impl fmt::Display for SwitchExecutorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotRunning(id) => write!(f, "no executor is currently running with id {id}"),
        }
    }
}

impl std::error::Error for SwitchExecutorError {}

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
    next_id: u8,
    pending: Vec<(u8, Box<dyn Executor>)>,
    running: HashMap<u8, thread::JoinHandle<Box<dyn Executor>>>,
    verbose: bool,
}

impl Runner {
    /// Creates an empty runner with no topics or executors registered yet.
    /// Verbose logging is off by default - see [`activate_verbose`](Self::activate_verbose).
    pub fn new() -> Self {
        Self {
            captain: Arc::new(Captain::new()),
            next_id: 0,
            pending: Vec::new(),
            running: HashMap::new(),
            verbose: false,
        }
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

    /// Registers a new topic under `name`, seeded with `initial`. Usually
    /// unnecessary now that [`crate::Executor::claim_writing_topics`] auto-registers
    /// a topic the first time its writer claims it - use this only to pre-seed a
    /// topic that has no writer.
    pub fn register_topic<T: Send + Sync + 'static>(
        &mut self,
        name: impl Into<String>,
        initial: T,
    ) {
        let name = name.into();
        self.log(LogColor::Pink, format!("registered topic {name:?}"));
        self.captain.register_topic(name, initial);
    }

    /// Looks up a previously registered topic, e.g. to read it after every
    /// executor has stopped. See [`Captain::topic`] for panic conditions.
    pub fn topic<T: Send + Sync + 'static>(&self, name: &str) -> Arc<RwLockTopic<T>> {
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
    pub fn add_executor(&mut self, executor: Box<dyn Executor>) -> u8 {
        let id = self.next_id;
        self.next_id = self
            .next_id
            .checked_add(1)
            .expect("Runner: exceeded u8::MAX executors (id space exhausted)");
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
    pub fn run_all(&mut self) {
        for (id, executor) in self.pending.drain(..) {
            let handle = Self::spawn(&self.captain, id, executor);
            self.running.insert(id, handle);
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
        id: u8,
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

        let handle = Self::spawn(&self.captain, id, new_executor);
        self.running.insert(id, handle);

        Ok(old_executor)
    }

    /// Waits for every currently running executor to finish, returning each one.
    /// Typically called after [`stop`](Self::stop) has signaled them all to exit.
    pub fn join_all(&mut self) -> Vec<Box<dyn Executor>> {
        let executors = self
            .running
            .drain()
            .map(|(_, handle)| handle.join().expect("executor thread panicked"))
            .collect();
        self.log(LogColor::Green, "all executors stopped successfully");
        executors
    }

    fn spawn(
        captain: &Arc<Captain>,
        id: u8,
        mut executor: Box<dyn Executor>,
    ) -> thread::JoinHandle<Box<dyn Executor>> {
        executor.init(id);
        let name = executor.name();
        captain.set_name(id, name.clone());
        executor.claim_writing_topics(captain);
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

    /// A minimal executor that records its assigned id, counts loop iterations, and
    /// flags when it's finished - just enough to observe `switch_executor`'s effects.
    struct CountingExecutor {
        id: u8,
        iterations: Arc<AtomicUsize>,
        finished: Arc<AtomicBool>,
    }

    impl Executor for CountingExecutor {
        fn init(&mut self, id: u8) {
            self.id = id;
        }

        fn run(&mut self, captain: &Captain) {
            while captain.is_running(self.id) {
                self.iterations.fetch_add(1, Ordering::Relaxed);
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
        }));
        let other_id = runner.add_executor(Box::new(CountingExecutor {
            id: 0,
            iterations: other_iterations.clone(),
            finished: Arc::new(AtomicBool::new(false)),
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
            }),
        );

        assert!(matches!(result, Err(SwitchExecutorError::NotRunning(0))));
    }
}
