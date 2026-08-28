//! An [`Executor`] is one independently scheduled unit of work that reads and/or
//! writes [`crate::Topic`]s through a shared [`crate::Captain`].

use crate::captain::Captain;
use std::any::Any;

/// One participant in the system: given an identity and a [`Captain`], it reads
/// and/or writes whatever topics it needs until told to stop.
///
/// [`crate::Runner`] owns a heterogeneous collection of executors and runs
/// each one on its own thread, calling [`init`](Self::init) once and then
/// [`run`](Self::run) for the executor's whole lifetime.
pub trait Executor: Send {
    /// Called once, before [`run`](Self::run), with this executor's identity. Used
    /// e.g. to claim a topic's writer slot or to tag published values.
    fn init(&mut self, id: u8);

    /// The executor's main loop. Should keep working until
    /// `captain.is_running(id)` (with the id given to [`init`](Self::init)) returns
    /// `false`, then return.
    fn run(&mut self, captain: &Captain);

    /// Enables downcasting a finished `Box<dyn Executor>` back to its concrete type,
    /// e.g. to read out implementation-specific results after
    /// [`crate::Runner::run_all`] joins it.
    fn as_any(&self) -> &dyn Any;

    /// Boxes this executor as a `Box<dyn Executor>`, e.g. for
    /// [`crate::Runner::add_executor`] or
    /// [`crate::Runner::switch_executor`].
    fn boxed(self) -> Box<dyn Executor>
    where
        Self: Sized + 'static,
    {
        Box::new(self)
    }
}
