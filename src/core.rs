//! Framework internals: the captain/executor/topic/runner/log pieces described
//! in the crate-level docs. Kept separate from the domain content in
//! [`crate::sensors`], [`crate::algorithms`], and [`crate::topics`].

mod captain;
mod debug_executor;
mod executor;
mod log;
mod rate;
mod runner;
mod topic;

pub use captain::Captain;
pub(crate) use captain::DebugTopic;
pub use debug_executor::{
    DEBUG_GROUP, DEFAULT_DEBUG_FREQUENCY_HZ, DebugRecorder, DebugState, DebugStatus,
};
pub use executor::Executor;
pub use rate::Ticker;
pub use runner::{Runner, StopHandle, SwitchExecutorError};
pub use topic::{RwLockTopic, Stamped, TopicError, WriteMeta};
