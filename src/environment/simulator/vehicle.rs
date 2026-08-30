//! Physical vehicle models for the [`crate::environment::simulator`]
//! simulator: each model is a plain state struct plus one or more pure step
//! functions that integrate it forward by one control input and one time
//! step, for a simulation environment (not implemented here) to call once
//! per tick. Starts with [`bicycle`], a kinematic bicycle model with slip
//! angle, [`dynamic_bicycle`], a dynamic model with a linear tire model, and
//! [`nonlinear_bicycle`], which adds tire saturation, load transfer, and
//! combined slip on top of that; more models are expected to join them over
//! time. See `src/environment/simulator/vehicle/README.md` for the modeling
//! background and equations.
//!
//! Not a trait: dispatch across the (small, closed, compile-time-known) set
//! of models lives one level up, in
//! [`crate::actuators::simulated_vehicle::VehicleModel`], as a plain `enum`
//! matched against a companion `VehicleState` enum - idiomatic for a fixed
//! set of variants known at compile time, and avoids the boxing/downcasting
//! a `dyn Trait` would need to hold heterogeneous per-model state. A new
//! model here adds one variant to each of those two enums, plus one match
//! arm in `VehicleModel`'s dispatch.

mod bicycle;
mod dynamic_bicycle;
mod nonlinear_bicycle;

pub use bicycle::{BicycleParams, BicycleState, step};
pub use dynamic_bicycle::{DynamicParams, DynamicState, step as dynamic_step};
pub use nonlinear_bicycle::{NonlinearBicycleState, NonlinearTireParams, step as nonlinear_step};
