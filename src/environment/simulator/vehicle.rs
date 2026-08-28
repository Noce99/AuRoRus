//! Physical vehicle models for the [`crate::environment::simulator`]
//! simulator: each model is a plain state struct plus one or more pure step
//! functions that integrate it forward by one control input and one time
//! step, for a simulation environment (not implemented here) to call once
//! per tick. Starts with [`bicycle`], a kinematic bicycle model with slip
//! angle; more models are expected to join it over time. See
//! `src/environment/simulator/vehicle/README.md` for the modeling
//! background and equations.
//!
//! Deliberately not a trait: with a single model implemented so far, there
//! is no shared interface to abstract yet. A shared step-like trait can be
//! introduced once a second model actually needs to be called
//! polymorphically.

mod bicycle;

pub use bicycle::{BicycleParams, BicycleState, step};
