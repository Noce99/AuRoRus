//! Estimating where the vehicle is: [`crate::Executor`]s that turn raw
//! sensor readings into a pose. [`DeadReckoning`] integrates
//! [`crate::topics::ImuReading`]s into a drifting
//! [`crate::topics::Odometry`] - the motion input the localization and
//! mapping algorithms here build on, starting with [`Slam`], which corrects
//! it against lidar scans while building a map. [`VehiclePose`] then puts
//! together the best pose available, for drawing. [`session`] records what
//! SLAM maps from and what localization runs from, to run them again
//! offline - [`evaluation`] localizes a session again with any algorithm and
//! scores it.

mod dead_reckoning;
pub mod evaluation;
mod particle_filter;
pub(crate) mod pose_source;
pub mod session;
mod slam;
mod vehicle_pose;

pub use dead_reckoning::{DeadReckoning, DeadReckoningConfig, Integration};
pub use evaluation::{Evaluation, OdometrySource, Pipeline, evaluate};
pub use particle_filter::{
    ParticleFilterConfig, save_parameters as save_particle_filter_parameters,
    saved_values as saved_particle_filter_values,
    tunable_parameters as particle_filter_tunable_parameters,
};
pub use slam::{
    KeptScan, ProcessedScan, RebuildOutcome, Scores, Slam, SlamConfig, TrajectoryError, WallError,
    map_walls, rebuild, save_parameters as save_slam_parameters, saved_values as saved_slam_values,
    score, tunable_parameters as slam_tunable_parameters,
};
pub use vehicle_pose::{VehiclePose, WorldPose};
