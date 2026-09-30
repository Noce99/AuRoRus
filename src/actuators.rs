//! Actuator drivers: [`crate::Executor`]s that turn a desired setpoint
//! topic into motion, whether simulated ([`SimulatedVehicle`]) or real (the
//! car's VESC motor controller - [`Vesc`], see [`vesc`]).

mod simulated_vehicle;
pub mod vesc;

pub use simulated_vehicle::{
    ActuatorLimits, OpponentVehicle, SimulatedVehicle, SimulatedVehicleConfig, VehicleModel,
    VehicleState, default_model, opponent_model, save_limits as save_vehicle_limits,
    save_parameters as save_vehicle_model_parameters, saved_limits as saved_vehicle_limits,
    saved_values as saved_vehicle_model_values,
};
#[cfg(unix)]
pub use vesc::Vesc;
pub use vesc::VescConfig;
