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
//! Executors don't talk to topics directly - they go through a [`TopicHandler`],
//! which owns every topic in the system and is shared (read-only) with every
//! executor. An [`ExecutorHandler`] owns every executor and starts them all running
//! in parallel once everything is registered.
//!
//! See the `lidar_benchmark` binary for a complete example: one executor publishes a
//! simulated LIDAR scan at a fixed rate, and several more each independently read the
//! latest scan at their own rate.

mod executor;
mod executor_handler;
mod topic;
mod topic_handler;

pub use executor::Executor;
pub use executor_handler::{ExecutorHandler, SwitchExecutorError};
pub use topic::{RwLockTopic, Topic, TopicError};
pub use topic_handler::TopicHandler;
