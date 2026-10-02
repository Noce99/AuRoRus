//! Physical vehicle models for the [`crate::environment::generator`]
//! simulator: each model is a plain state struct plus one or more pure step
//! functions that integrate it forward by one control input and one time
//! step, for a simulation environment (not implemented here) to call once
//! per tick. Starts with [`bicycle`], a kinematic bicycle model with slip
//! angle, [`dynamic_bicycle`], a dynamic model with a linear tire model,
//! [`nonlinear_bicycle`], which adds tire saturation, load transfer, and
//! combined slip on top of that via a simplified Pacejka-style curve,
//! [`pacejka_bicycle`], which upgrades that curve to the full Pacejka Magic
//! Formula with independently-tuned front/rear axles, and [`two_track`],
//! which treats all four wheels individually instead of collapsing each
//! axle into one, adding lateral load transfer and per-wheel asymmetry;
//! more models are expected to join them over time. See
//! `src/simulation/vehicle_models/README.md` for the modeling
//! background and equations.
//!
//! Not a trait: dispatch across the (small, closed, compile-time-known) set
//! of models lives one level up, in
//! [`crate::simulation::vehicle::VehicleModel`], as a plain `enum`
//! matched against a companion `VehicleState` enum - idiomatic for a fixed
//! set of variants known at compile time, and avoids the boxing/downcasting
//! a `dyn Trait` would need to hold heterogeneous per-model state. A new
//! model here adds one variant to each of those two enums, plus one match
//! arm in `VehicleModel`'s dispatch.

mod bicycle;
mod dynamic_bicycle;
mod nonlinear_bicycle;
mod pacejka_bicycle;
mod two_track;

pub use bicycle::{BicycleParams, BicycleState, step};
pub use dynamic_bicycle::{DynamicParams, DynamicState, step as dynamic_step};
pub use nonlinear_bicycle::{NonlinearBicycleState, NonlinearTireParams, step as nonlinear_step};
pub use pacejka_bicycle::{PacejkaBicycleState, PacejkaTireParams, step as pacejka_step};
pub use two_track::{TwoTrackParams, TwoTrackState, step as two_track_step};

/// Live-tuning declarations for the fields the models' `Params` structs
/// share, so every model offers each one with the same range - see each
/// struct's `tunable_parameters`. The car's own (`mass_kg`, `lf_m`, `lr_m`,
/// `track_width_m`, marked `#[serde(default)]`) aren't tuned: they come from
/// its calibration - see `crate::simulation::SimulatedVehicleConfig::for_car`.
mod tunable {
    use crate::topics::AlgorithmParameter;

    pub fn yaw_inertia_kgm2() -> AlgorithmParameter {
        AlgorithmParameter::float("yaw_inertia_kgm2", 0.01, 0.5, 0.01)
            .unit("kg·m²")
            .description("Yaw moment of inertia about the vertical axis through the CG.")
    }

    pub fn cg_height_m() -> AlgorithmParameter {
        AlgorithmParameter::float("cg_height_m", 0.01, 0.3, 0.01)
            .unit("m")
            .description("Height of the CG above the ground - drives load transfer.")
    }

    pub fn front_drive_fraction() -> AlgorithmParameter {
        AlgorithmParameter::float("front_drive_fraction", 0.0, 1.0, 0.05)
            .description("Share of the drive force through the front axle: 0 = RWD, 1 = FWD.")
    }

    /// The Magic Formula `B`, `C`, `D` (as a friction coefficient) and `E`
    /// of one axle's tires, `axle` being `"front"` or `"rear"`.
    pub fn magic_formula(axle: &str) -> [AlgorithmParameter; 4] {
        let label = if axle == "front" { "Front" } else { "Rear" };
        [
            AlgorithmParameter::float(format!("{axle}_b"), 0.5, 15.0, 0.1)
                .description(format!("{label} tire Magic Formula stiffness factor (B).")),
            AlgorithmParameter::float(format!("{axle}_c"), 0.5, 3.0, 0.05)
                .description(format!("{label} tire Magic Formula shape factor (C).")),
            AlgorithmParameter::float(format!("{axle}_d_mu"), 0.1, 2.0, 0.05).description(format!(
                "{label} tire peak friction coefficient (D = mu * Fz)."
            )),
            AlgorithmParameter::float(format!("{axle}_e"), -3.0, 1.0, 0.1)
                .description(format!("{label} tire Magic Formula curvature factor (E).")),
        ]
    }

    pub fn combined_slip() -> [AlgorithmParameter; 2] {
        [
            AlgorithmParameter::float("combined_slip_b", 0.1, 5.0, 0.1)
                .description("Stiffness factor of the combined-slip weighting curve."),
            AlgorithmParameter::float("combined_slip_c", 0.1, 5.0, 0.1)
                .description("Shape factor of the combined-slip weighting curve."),
        ]
    }
}
