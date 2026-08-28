//! Framework internals: the captain/executor/topic/runner/log pieces described
//! in the crate-level docs. Kept separate from the domain content in
//! [`crate::sensors`], [`crate::algorithms`], and [`crate::topics`].

mod captain;
mod executor;
mod log;
mod runner;
mod topic;

pub use captain::Captain;
pub use executor::Executor;
pub use runner::{Runner, SwitchExecutorError};
pub use topic::{RwLockTopic, TopicError};
