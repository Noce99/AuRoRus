//! Owns every [`Topic`] shared between executors, plus the run/stop signal they poll.

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
/// This is also where the shared stop signal lives: since [`crate::Executor::run`]
/// only receives a `&TopicHandler`, an executor's run loop polls
/// [`is_running`](Self::is_running) to know when to exit.
pub struct TopicHandler {
    topics: HashMap<String, Box<dyn Any + Send + Sync>>,
    running: AtomicBool,
}

impl TopicHandler {
    /// Creates an empty, running handler.
    pub fn new() -> Self {
        Self {
            topics: HashMap::new(),
            running: AtomicBool::new(true),
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

    /// Whether executors should keep running.
    pub fn is_running(&self) -> bool {
        self.running.load(Ordering::Relaxed)
    }

    /// Signals every executor polling [`is_running`](Self::is_running) to stop.
    pub fn stop(&self) {
        self.running.store(false, Ordering::Relaxed);
    }
}

impl Default for TopicHandler {
    fn default() -> Self {
        Self::new()
    }
}
