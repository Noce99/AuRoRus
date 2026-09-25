//! Estimating where the vehicle is: [`crate::Executor`]s that turn raw
//! sensor readings into a pose. Starts with [`DeadReckoning`], which
//! integrates [`crate::topics::ImuReading`]s into a drifting
//! [`crate::topics::Odometry`] - the motion input the localization and
//! mapping algorithms expected to join it here (a particle filter, SLAM)
//! build on.

mod dead_reckoning;

pub use dead_reckoning::{DeadReckoning, DeadReckoningConfig, Integration};
