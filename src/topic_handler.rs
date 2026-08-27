//! Owns every [`Topic`] shared between executors, plus the run/stop signals they poll.

use crate::topic::LockFreeTopic;
use std::any::Any;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};

/// A registry of named, typed topics, shared (read-only, via `Arc`) with every
/// running executor.
///
/// Build one, [`register_topic`](Self::register_topic) every topic executors will
/// use, then hand it to [`crate::ExecutorHandler`]. Because different topics can hold
/// different types, each is stored type-erased and recovered via
/// [`topic`](Self::topic).
///
/// This is also where the run/stop signals live: since [`crate::Executor::run`] only
/// receives a `&TopicHandler`, an executor's run loop polls
/// [`is_running`](Self::is_running) with its own id to know when to exit - either
/// because every executor was told to stop, or because just that one was (e.g. by
/// [`crate::ExecutorHandler::switch_executor`]).
pub struct TopicHandler {
    topics: HashMap<String, Box<dyn Any + Send + Sync>>,
    running: AtomicBool,
    executor_running: [AtomicBool; 256],
}

impl TopicHandler {
    /// Creates an empty, running handler.
    pub fn new() -> Self {
        Self {
            topics: HashMap::new(),
            running: AtomicBool::new(true),
            executor_running: std::array::from_fn(|_| AtomicBool::new(true)),
        }
    }

    /// Registers a new topic under `name`, seeded with `initial`. Call during setup,
    /// before executors are started - topics can't be added once executors are
    /// running against a shared (`Arc`'d) handler.
    pub fn register_topic<T: Send + Sync + 'static>(&mut self, name: impl Into<String>, initial: T) {
        self.topics
            .insert(name.into(), Box::new(LockFreeTopic::new(initial)));
    }

    /// Looks up a previously registered topic.
    ///
    /// # Panics
    ///
    /// Panics if no topic is registered under `name`, or if it was registered with a
    /// different item type than `T`. Both are programmer errors (a mismatch between
    /// how a topic was registered and how it's used), not runtime data conditions.
    pub fn topic<T: Send + Sync + 'static>(&self, name: &str) -> &LockFreeTopic<T> {
        self.topics
            .get(name)
            .unwrap_or_else(|| panic!("no topic registered named {name:?}"))
            .downcast_ref::<LockFreeTopic<T>>()
            .unwrap_or_else(|| panic!("topic {name:?} was not registered with this item type"))
    }

    /// Whether the executor with this `id` should keep running: true only if every
    /// executor was told to stop via [`stop`](Self::stop), and this particular id
    /// wasn't individually stopped (e.g. for a [`switch_executor`]-driven swap).
    ///
    /// [`switch_executor`]: crate::ExecutorHandler::switch_executor
    pub fn is_running(&self, id: u8) -> bool {
        self.running.load(Ordering::Relaxed) && self.executor_running[id as usize].load(Ordering::Relaxed)
    }

    /// Signals every executor polling [`is_running`](Self::is_running) to stop.
    pub fn stop(&self) {
        self.running.store(false, Ordering::Relaxed);
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

impl Default for TopicHandler {
    fn default() -> Self {
        Self::new()
    }
}
