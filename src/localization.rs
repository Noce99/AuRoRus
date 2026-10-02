//! Estimating where the vehicle is: [`crate::Executor`]s that turn raw
//! sensor readings into a pose. [`DeadReckoning`] integrates
//! [`crate::topics::ImuReading`]s into a drifting
//! [`crate::topics::Odometry`] - the motion input the localization and
//! mapping algorithms here build on, starting with [`Slam`], which corrects
//! it against lidar scans while building a map. [`VehiclePose`] then puts
//! together the best pose available, for drawing.

mod dead_reckoning;
pub(crate) mod pose_source;
mod slam;
mod vehicle_pose;

pub use dead_reckoning::{DeadReckoning, DeadReckoningConfig, Integration};
pub use slam::{Slam, SlamConfig};
pub use vehicle_pose::{VehiclePose, WorldPose};
