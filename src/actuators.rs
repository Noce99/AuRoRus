//! Actuator drivers: [`crate::Executor`]s that turn a desired setpoint
//! topic into motion on the real car - its VESC motor controller ([`Vesc`],
//! see [`vesc`]). The simulated counterpart is
//! [`crate::simulation::SimulatedVehicle`].

pub(crate) mod command;
pub mod vesc;

#[cfg(unix)]
pub use vesc::Vesc;
pub use vesc::VescConfig;
