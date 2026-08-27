//! An [`Executor`] is one independently scheduled unit of work that reads and/or
//! writes [`crate::Topic`]s through a shared [`crate::TopicHandler`].

use crate::topic_handler::TopicHandler;
use std::any::Any;

/// One participant in the system: given an identity and a [`TopicHandler`], it reads
/// and/or writes whatever topics it needs until told to stop.
///
/// [`crate::ExecutorHandler`] owns a heterogeneous collection of executors and runs
/// each one on its own thread, calling [`init`](Self::init) once and then
/// [`run`](Self::run) for the executor's whole lifetime.
pub trait Executor: Send {
    /// Called once, before [`run`](Self::run), with this executor's identity. Used
    /// e.g. to claim a topic's writer slot or to tag published values.
    fn init(&mut self, id: u8);

    /// The executor's main loop. Should keep working until
    /// `topics.is_running()` returns `false`, then return.
    fn run(&mut self, topics: &TopicHandler);

    /// Enables downcasting a finished `Box<dyn Executor>` back to its concrete type,
    /// e.g. to read out implementation-specific results after
    /// [`crate::ExecutorHandler::run_all`] joins it.
    fn as_any(&self) -> &dyn Any;
}
