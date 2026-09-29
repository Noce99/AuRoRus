//! An [`Executor`] is one independently scheduled unit of work that reads and/or
//! writes [`crate::RwLockTopic`]s through a shared [`crate::Captain`].

use crate::core::captain::Captain;
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
    fn init(&mut self, id: u16);

    /// Called once, after [`init`](Self::init) and before this executor's thread is
    /// spawned. Executors that write to a topic must claim it here, via
    /// `captain.claim_writer::<T>(...)`, rather than from inside [`run`](Self::run):
    /// [`crate::Runner`] calls this synchronously, on its own thread, for every
    /// executor before spawning any of their threads, so a writer-slot conflict
    /// between two executors is caught deterministically up front instead of
    /// racing on whichever of their threads happens to run first.
    ///
    /// Default: does nothing, for executors that only read topics.
    fn claim_writing_topics(&mut self, _captain: &Captain) {}

    /// The executor's main loop. Should keep working until
    /// `captain.is_running(id)` (with the id given to [`init`](Self::init)) returns
    /// `false`, then return.
    fn run(&mut self, captain: &Captain);

    /// A short, human-readable name for this executor (e.g. `"Writer 0"`,
    /// `"Reader 2"`), used for thread naming and diagnostic messages - never
    /// parsed, just displayed.
    fn name(&self) -> String;

    /// Enables downcasting a finished `Box<dyn Executor>` back to its concrete type,
    /// e.g. to read out implementation-specific results after
    /// [`crate::Runner::run_all`] joins it.
    fn as_any(&self) -> &dyn Any;

    /// A brand-new instance of this executor: same construction parameters
    /// (name, config, ...) as when it was first created, but none of the
    /// state accumulated while running. Used by
    /// [`crate::Runner::run_until_stopped`] to give every executor a
    /// completely fresh start after a [`crate::Captain::request_restart`].
    fn fresh(&self) -> Box<dyn Executor>;

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
