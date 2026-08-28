//! [`Captain`] is what each running [`crate::Executor`] answers to: it owns every
//! topic plus the run/stop signals an executor polls to know when to exit.

use crate::topic::RwLockTopic;
use std::any::Any;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};

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
    topics: HashMap<String, Box<dyn Any + Send + Sync>>,
    running: AtomicBool,
    executor_running: [AtomicBool; 256],
}

impl Captain {
    /// Creates an empty, running captain.
    pub(crate) fn new() -> Self {
        Self {
            topics: HashMap::new(),
            running: AtomicBool::new(true),
            executor_running: std::array::from_fn(|_| AtomicBool::new(true)),
        }
    }

    /// Registers a new topic under `name`, seeded with `initial`. Call during setup,
    /// before executors are started - topics can't be added once executors are
    /// running against a shared (`Arc`'d) captain.
    pub(crate) fn register_topic<T: Send + Sync + 'static>(
        &mut self,
        name: impl Into<String>,
        initial: T,
    ) {
        self.topics
            .insert(name.into(), Box::new(RwLockTopic::new(initial)));
    }

    /// Looks up a previously registered topic.
    ///
    /// # Panics
    ///
    /// Panics if no topic is registered under `name`, or if it was registered with a
    /// different item type than `T`. Both are programmer errors (a mismatch between
    /// how a topic was registered and how it's used), not runtime data conditions.
    pub fn topic<T: Send + Sync + 'static>(&self, name: &str) -> &RwLockTopic<T> {
        self.topics
            .get(name)
            .unwrap_or_else(|| panic!("no topic registered named {name:?}"))
            .downcast_ref::<RwLockTopic<T>>()
            .unwrap_or_else(|| panic!("topic {name:?} was not registered with this item type"))
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
