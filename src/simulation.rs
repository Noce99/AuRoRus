//! Everything that stands in for the real car and track when there is none:
//! the vehicle physics models ([`vehicle_models`]) and the [`SimulatedVehicle`]
//! that runs one, the sensors reading the simulated world ([`SimulatedLidar`],
//! [`SimulatedImu`], and the map-less [`RandomLidar`]), and the opponents
//! sharing the track ([`OpponentsManager`]).
//!
//! Their real counterparts are in [`crate::actuators`] and [`crate::sensors`];
//! a binary runs one side or the other, publishing on the same topics.

mod imu;
mod lidar;
pub mod opponents;
mod random_lidar;
pub mod vehicle;
pub mod vehicle_models;

pub use imu::{SimulatedImu, SimulatedImuConfig};
pub use lidar::{SimulatedLidar, SimulatedLidarConfig};
pub use opponents::OpponentsManager;
pub use random_lidar::{RandomLidar, RandomLidarConfig};
pub use vehicle::{
    ActuatorLimits, OpponentVehicle, SimulatedVehicle, SimulatedVehicleConfig, VehicleModel,
    VehicleState, default_model, opponent_model, save_limits as save_vehicle_limits,
    save_parameters as save_vehicle_model_parameters, saved_limits as saved_vehicle_limits,
    saved_values as saved_vehicle_model_values,
};
