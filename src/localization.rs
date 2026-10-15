//! Estimating where the vehicle is: [`crate::Executor`]s that turn raw
//! sensor readings into a pose. [`DeadReckoning`] integrates
//! [`crate::topics::ImuReading`]s into a drifting
//! [`crate::topics::Odometry`] - the motion input the localization and
//! mapping algorithms here build on, starting with [`Slam`], which corrects
//! it against lidar scans while building a map. [`VehiclePose`] then puts
//! together the best pose available, for drawing. [`mapping_session`]
//! records what SLAM maps from, to build the map again offline.

mod dead_reckoning;
pub mod mapping_session;
pub(crate) mod pose_source;
mod slam;
mod vehicle_pose;

pub use dead_reckoning::{DeadReckoning, DeadReckoningConfig, Integration};
pub use slam::{
    Slam, SlamConfig, save_parameters as save_slam_parameters, saved_values as saved_slam_values,
};
pub use vehicle_pose::{VehiclePose, WorldPose};
