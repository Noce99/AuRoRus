//! A tiny framework for sharing periodically-updated data between independently
//! scheduled threads, one writer and many readers per topic, behind a small
//! `RwLock`-based abstraction.
//!
//! The model has two kinds of pieces:
//!
//! - An [`Executor`] is one participant - e.g. a sensor driver or a consumer - that
//!   runs on its own thread at its own pace.
//! - A [`Topic`] is a single named, typed slot of shared state with exactly one
//!   authorized writer and any number of readers. Readers always see the most
//!   recently published value; there is no notion of "unread" data, so reading faster
//!   than the writer publishes simply re-observes the same value.
//!
//! Executors don't talk to topics directly - they go through a [`Captain`], which
//! owns every topic in the system and is shared (read-only) with every executor. A
//! [`Runner`] owns every topic and every executor, and starts them all running in
//! parallel once everything is registered.
//!
//! See the `benchmark_comunication_time` binary for a complete example: one executor
//! publishes a payload at a fixed rate, and several more each independently read the
//! latest payload at their own rate.

mod captain;
mod executor;
mod log;
mod runner;
mod topic;

pub use captain::Captain;
pub use executor::Executor;
pub use runner::{Runner, SwitchExecutorError};
pub use topic::{RwLockTopic, Topic, TopicError};
