//! Actuator drivers: [`crate::Executor`]s that turn a desired setpoint
//! topic into motion, whether simulated ([`SimulatedVehicle`]) or - one
//! day - a real motor/servo driver talking to actual hardware.

mod simulated_vehicle;

pub use simulated_vehicle::{
    ActuatorLimits, SimulatedVehicle, SimulatedVehicleConfig, VehicleModel, VehicleState, default_model,
};
