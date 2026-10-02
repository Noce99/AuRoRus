//! [`VehicleModel`] and [`VehicleState`]: the closed set of physics models
//! [`super::SimulatedVehicle`] can run, and how one is stepped forward,
//! started, and swapped for another mid-run.

use crate::simulation::vehicle_models::{
    BicycleParams, BicycleState, DynamicParams, DynamicState, NonlinearBicycleState,
    NonlinearTireParams, PacejkaBicycleState, PacejkaTireParams, TwoTrackParams, TwoTrackState,
    dynamic_step, nonlinear_step, pacejka_step, step as bicycle_step, two_track_step,
};
use crate::topics::{ActuatorLimits, StartState, VehicleModelKind};

/// Which vehicle model [`SimulatedVehicle`](super::SimulatedVehicle) should run, and the geometry and
/// actuator limits it needs to do so. A plain `enum` (rather than a trait)
/// because the set of models is small and known at compile time - see
/// `src/simulation/vehicle_models.rs`'s module doc comment for why the
/// same choice was made there. Its companion state type is [`VehicleState`];
/// a new model adds one variant here, one to `VehicleState`, and one match
/// arm in [`advance`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VehicleModel {
    /// A CG-referenced kinematic bicycle model - see
    /// [`crate::simulation::vehicle_models::bicycle`].
    Bicycle {
        params: BicycleParams,
        limits: ActuatorLimits,
    },
    /// A dynamic bicycle model with lateral tire forces - see
    /// [`crate::simulation::vehicle_models::dynamic_bicycle`].
    DynamicBicycle {
        params: DynamicParams,
        limits: ActuatorLimits,
    },
    /// A dynamic bicycle model with tire saturation, load transfer, and
    /// combined slip - see
    /// [`crate::simulation::vehicle_models::nonlinear_bicycle`].
    NonlinearBicycle {
        params: NonlinearTireParams,
        limits: ActuatorLimits,
    },
    /// A dynamic bicycle model using the full Pacejka Magic Formula for
    /// lateral tire force - see
    /// [`crate::simulation::vehicle_models::pacejka_bicycle`].
    PacejkaBicycle {
        params: PacejkaTireParams,
        limits: ActuatorLimits,
    },
    /// A two-track (four-wheel) model with lateral load transfer and
    /// per-wheel asymmetry - see
    /// [`crate::simulation::vehicle_models::two_track`].
    TwoTrack {
        params: TwoTrackParams,
        limits: ActuatorLimits,
    },
}

/// The state a [`VehicleModel`] is advancing, in the same variant as the
/// model currently running it - [`SimulatedVehicle::run`](crate::Executor::run) always keeps the
/// two in sync (see [`advance`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VehicleState {
    Bicycle(BicycleState),
    DynamicBicycle(DynamicState),
    NonlinearBicycle(NonlinearBicycleState),
    PacejkaBicycle(PacejkaBicycleState),
    TwoTrack(TwoTrackState),
}

impl VehicleState {
    /// X coordinate of the CG in world coordinates, in meters - common to
    /// every model.
    pub(super) fn x_m(&self) -> f64 {
        match self {
            Self::Bicycle(s) => s.x_m,
            Self::DynamicBicycle(s) => s.x_m,
            Self::NonlinearBicycle(s) => s.x_m,
            Self::PacejkaBicycle(s) => s.x_m,
            Self::TwoTrack(s) => s.x_m,
        }
    }

    /// Y coordinate of the CG in world coordinates, in meters - common to
    /// every model.
    pub(super) fn y_m(&self) -> f64 {
        match self {
            Self::Bicycle(s) => s.y_m,
            Self::DynamicBicycle(s) => s.y_m,
            Self::NonlinearBicycle(s) => s.y_m,
            Self::PacejkaBicycle(s) => s.y_m,
            Self::TwoTrack(s) => s.y_m,
        }
    }

    /// Heading of the vehicle body, in radians - common to every model.
    pub(super) fn heading_rad(&self) -> f64 {
        match self {
            Self::Bicycle(s) => s.heading_rad,
            Self::DynamicBicycle(s) => s.heading_rad,
            Self::NonlinearBicycle(s) => s.heading_rad,
            Self::PacejkaBicycle(s) => s.heading_rad,
            Self::TwoTrack(s) => s.heading_rad,
        }
    }

    /// The vehicle's ground speed, in meters/second - a single number for
    /// the kinematic model, or the magnitude of the body-frame velocity for
    /// a model (like [`DynamicState`]/[`NonlinearBicycleState`]/
    /// [`PacejkaBicycleState`]/[`TwoTrackState`]) that tracks longitudinal
    /// and lateral velocity separately. For reporting only - see
    /// [`Self::longitudinal_speed_mps`] for the signed quantity [`advance`]'s
    /// speed controller tracks.
    pub(super) fn speed_mps(&self) -> f64 {
        match self {
            Self::Bicycle(s) => s.speed_mps,
            Self::DynamicBicycle(s) => s.vx_mps.hypot(s.vy_mps),
            Self::NonlinearBicycle(s) => s.vx_mps.hypot(s.vy_mps),
            Self::PacejkaBicycle(s) => s.vx_mps.hypot(s.vy_mps),
            Self::TwoTrack(s) => s.vx_mps.hypot(s.vy_mps),
        }
    }

    /// The signed longitudinal speed [`advance`]'s controller tracks
    /// `target_speed_mps` against, in meters/second - the kinematic model's
    /// single signed `speed_mps`, or `vx_mps` for a model that tracks
    /// longitudinal and lateral velocity separately. Unlike [`Self::speed_mps`],
    /// this is signed: using the unsigned ground-speed magnitude here would
    /// let a braking overshoot past zero (a negative `vx_mps` with `vy_mps`
    /// near zero) read as still needing to *decelerate*, since the magnitude
    /// keeps growing as the vehicle picks up speed backward - driving
    /// `accel_mps2` to stay pinned at `-max_decel_mps2` every tick instead of
    /// flipping sign, a runaway with no bound from `max_speed_mps` (only the
    /// target is clamped to it, not the state).
    pub(super) fn longitudinal_speed_mps(&self) -> f64 {
        match self {
            Self::Bicycle(s) => s.speed_mps,
            Self::DynamicBicycle(s) => s.vx_mps,
            Self::NonlinearBicycle(s) => s.vx_mps,
            Self::PacejkaBicycle(s) => s.vx_mps,
            Self::TwoTrack(s) => s.vx_mps,
        }
    }
}

/// This model's [`ActuatorLimits`], regardless of which variant it is -
/// used by [`SimulatedVehicle::run`](crate::Executor::run) without a `match` at every call site.
pub(super) fn limits_of(model: &VehicleModel) -> ActuatorLimits {
    match model {
        VehicleModel::Bicycle { limits, .. } => *limits,
        VehicleModel::DynamicBicycle { limits, .. } => *limits,
        VehicleModel::NonlinearBicycle { limits, .. } => *limits,
        VehicleModel::PacejkaBicycle { limits, .. } => *limits,
        VehicleModel::TwoTrack { limits, .. } => *limits,
    }
}

/// Which [`VehicleModelKind`] `model` is an instance of.
pub(super) fn kind_of(model: &VehicleModel) -> VehicleModelKind {
    match model {
        VehicleModel::Bicycle { .. } => VehicleModelKind::Bicycle,
        VehicleModel::DynamicBicycle { .. } => VehicleModelKind::DynamicBicycle,
        VehicleModel::NonlinearBicycle { .. } => VehicleModelKind::NonlinearBicycle,
        VehicleModel::PacejkaBicycle { .. } => VehicleModelKind::PacejkaBicycle,
        VehicleModel::TwoTrack { .. } => VehicleModelKind::TwoTrack,
    }
}

/// Builds a [`VehicleState`] of `kind` from `start`'s shared fields (position,
/// heading, speed) - used for the very first state [`SimulatedVehicle::run`](crate::Executor::run)
/// advances, so the vehicle starts wherever [`START_STATE_TOPIC_NAME`] says
/// to, zeroed on every other field. Implemented via [`carry_over_state`],
/// which does exactly this projection when switching models at runtime.
pub(super) fn state_from_start(start: StartState, kind: VehicleModelKind) -> VehicleState {
    carry_over_state(
        VehicleState::Bicycle(BicycleState {
            x_m: start.x_m,
            y_m: start.y_m,
            heading_rad: start.heading_rad,
            speed_mps: start.speed_mps,
        }),
        kind,
    )
}

/// Projects `old`'s shared fields (position, heading, speed) into a fresh
/// [`VehicleState`] of `new_kind`, zeroing any fields `new_kind` has that
/// `old` didn't - used when [`SimulatedVehicle::run`](crate::Executor::run) switches models at
/// runtime, so the vehicle doesn't visibly jump when the model underneath it
/// changes mid-drive.
pub(super) fn carry_over_state(old: VehicleState, new_kind: VehicleModelKind) -> VehicleState {
    let (x_m, y_m, heading_rad, speed_mps) =
        (old.x_m(), old.y_m(), old.heading_rad(), old.speed_mps());
    match new_kind {
        VehicleModelKind::Bicycle => VehicleState::Bicycle(BicycleState {
            x_m,
            y_m,
            heading_rad,
            speed_mps,
        }),
        VehicleModelKind::DynamicBicycle => VehicleState::DynamicBicycle(DynamicState {
            x_m,
            y_m,
            heading_rad,
            vx_mps: speed_mps,
            vy_mps: 0.0,
            yaw_rate_rad_s: 0.0,
        }),
        VehicleModelKind::NonlinearBicycle => {
            VehicleState::NonlinearBicycle(NonlinearBicycleState {
                x_m,
                y_m,
                heading_rad,
                vx_mps: speed_mps,
                vy_mps: 0.0,
                yaw_rate_rad_s: 0.0,
            })
        }
        VehicleModelKind::PacejkaBicycle => VehicleState::PacejkaBicycle(PacejkaBicycleState {
            x_m,
            y_m,
            heading_rad,
            vx_mps: speed_mps,
            vy_mps: 0.0,
            yaw_rate_rad_s: 0.0,
        }),
        VehicleModelKind::TwoTrack => VehicleState::TwoTrack(TwoTrackState {
            x_m,
            y_m,
            heading_rad,
            vx_mps: speed_mps,
            vy_mps: 0.0,
            yaw_rate_rad_s: 0.0,
        }),
    }
}

/// Advances `state`/`steering_angle_rad` by one `dt_s` tick under `model`,
/// steering and accelerating toward `target_steering_rad`/`target_speed_mps`
/// as fast as the model's [`ActuatorLimits`] allow (never instantaneously,
/// and never past the limits) - not just integrating the raw setpoint, which
/// would let the simulated vehicle do things no real actuator could.
///
/// `model` and `state` must be the same [`VehicleModelKind`] - true of every
/// call from [`SimulatedVehicle::run`](crate::Executor::run), which always switches both together.
pub(super) fn advance(
    model: &VehicleModel,
    state: VehicleState,
    steering_angle_rad: f64,
    target_steering_rad: f64,
    target_speed_mps: f64,
    dt_s: f64,
) -> (VehicleState, f64) {
    let limits = limits_of(model);

    let target_steering_rad = target_steering_rad.clamp(
        -limits.max_steering_angle_rad,
        limits.max_steering_angle_rad,
    );
    let max_steering_delta = limits.max_steering_rate_rad_s * dt_s;
    let next_steering_rad = (steering_angle_rad
        + (target_steering_rad - steering_angle_rad)
            .clamp(-max_steering_delta, max_steering_delta))
    .clamp(
        -limits.max_steering_angle_rad,
        limits.max_steering_angle_rad,
    );

    let target_speed_mps = target_speed_mps.clamp(-limits.max_speed_mps, limits.max_speed_mps);
    let speed_error_mps = target_speed_mps - state.longitudinal_speed_mps();
    let accel_mps2 = if speed_error_mps >= 0.0 {
        (speed_error_mps / dt_s).min(limits.max_accel_mps2)
    } else {
        (speed_error_mps / dt_s).max(-limits.max_decel_mps2)
    };

    let next_state = match (model, state) {
        (VehicleModel::Bicycle { params, .. }, VehicleState::Bicycle(s)) => VehicleState::Bicycle(
            bicycle_step(s, *params, next_steering_rad, accel_mps2, dt_s),
        ),
        (VehicleModel::DynamicBicycle { params, .. }, VehicleState::DynamicBicycle(s)) => {
            VehicleState::DynamicBicycle(dynamic_step(
                s,
                *params,
                next_steering_rad,
                accel_mps2,
                dt_s,
            ))
        }
        (VehicleModel::NonlinearBicycle { params, .. }, VehicleState::NonlinearBicycle(s)) => {
            VehicleState::NonlinearBicycle(nonlinear_step(
                s,
                *params,
                next_steering_rad,
                accel_mps2,
                dt_s,
            ))
        }
        (VehicleModel::PacejkaBicycle { params, .. }, VehicleState::PacejkaBicycle(s)) => {
            VehicleState::PacejkaBicycle(pacejka_step(
                s,
                *params,
                next_steering_rad,
                accel_mps2,
                dt_s,
            ))
        }
        (VehicleModel::TwoTrack { params, .. }, VehicleState::TwoTrack(s)) => {
            VehicleState::TwoTrack(two_track_step(
                s,
                *params,
                next_steering_rad,
                accel_mps2,
                dt_s,
            ))
        }
        _ => unreachable!("SimulatedVehicle::run always keeps model/state kinds in sync"),
    };
    (next_state, next_steering_rad)
}

/// The vehicle's velocity in the body frame at `state`: `(vx_mps, vy_mps,
/// yaw_rate_rad_s)`. Read straight off the state for every model that tracks
/// them; for the kinematic [`VehicleModel::Bicycle`], which only tracks a
/// ground speed, derived from its slip angle under `steering_angle_rad` with
/// the same equations its step integrates (see
/// [`crate::simulation::vehicle_models::bicycle`]).
///
/// `model` and `state` must be the same [`VehicleModelKind`], as for
/// [`advance`].
pub(super) fn body_velocity(
    model: &VehicleModel,
    state: &VehicleState,
    steering_angle_rad: f64,
) -> (f64, f64, f64) {
    match (model, state) {
        (VehicleModel::Bicycle { params, .. }, VehicleState::Bicycle(s)) => {
            let beta =
                ((params.lr_m / (params.lf_m + params.lr_m)) * steering_angle_rad.tan()).atan();
            (
                s.speed_mps * beta.cos(),
                s.speed_mps * beta.sin(),
                (s.speed_mps / params.lr_m) * beta.sin(),
            )
        }
        (_, VehicleState::DynamicBicycle(s)) => (s.vx_mps, s.vy_mps, s.yaw_rate_rad_s),
        (_, VehicleState::NonlinearBicycle(s)) => (s.vx_mps, s.vy_mps, s.yaw_rate_rad_s),
        (_, VehicleState::PacejkaBicycle(s)) => (s.vx_mps, s.vy_mps, s.yaw_rate_rad_s),
        (_, VehicleState::TwoTrack(s)) => (s.vx_mps, s.vy_mps, s.yaw_rate_rad_s),
        _ => unreachable!("SimulatedVehicle::run always keeps model/state kinds in sync"),
    }
}

/// The CG's acceleration in the body frame, `(ax_mps2, ay_mps2)`, between
/// two consecutive ticks `dt_s` apart whose body-frame velocities were
/// `previous` and `current` (each `(vx_mps, vy_mps)`), while yawing at
/// `yaw_rate_rad_s`. The body frame rotates, so this is the rate of change
/// of the body-frame velocity plus the rotation term - `ax = dvx/dt - vy *
/// yaw_rate`, `ay = dvy/dt + vx * yaw_rate` - which is what an accelerometer
/// bolted to the chassis reads.
pub(super) fn body_acceleration(
    previous: (f64, f64),
    current: (f64, f64),
    yaw_rate_rad_s: f64,
    dt_s: f64,
) -> (f64, f64) {
    let (vx_mps, vy_mps) = current;
    (
        (vx_mps - previous.0) / dt_s - vy_mps * yaw_rate_rad_s,
        (vy_mps - previous.1) / dt_s + vx_mps * yaw_rate_rad_s,
    )
}

/// The distances from the model's reference point to its front and rear
/// axles - what [`drawing`](super::drawing) turns the front wheels about.
pub(super) fn axles_of(model: &VehicleModel) -> (f64, f64) {
    match model {
        VehicleModel::Bicycle { params, .. } => (params.lf_m, params.lr_m),
        VehicleModel::DynamicBicycle { params, .. } => (params.lf_m, params.lr_m),
        VehicleModel::NonlinearBicycle { params, .. } => (params.lf_m, params.lr_m),
        VehicleModel::PacejkaBicycle { params, .. } => (params.lf_m, params.lr_m),
        VehicleModel::TwoTrack { params, .. } => (params.lf_m, params.lr_m),
    }
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;

    pub(in crate::simulation::vehicle) fn test_model() -> VehicleModel {
        VehicleModel::Bicycle {
            params: BicycleParams {
                lf_m: 0.16,
                lr_m: 0.16,
            },
            limits: ActuatorLimits {
                max_steering_angle_rad: 0.4,
                max_steering_rate_rad_s: 4.0,
                max_speed_mps: 8.0,
                max_accel_mps2: 4.0,
                max_decel_mps2: 8.0,
            },
        }
    }

    pub(in crate::simulation::vehicle) fn test_dynamic_model() -> VehicleModel {
        VehicleModel::DynamicBicycle {
            params: DynamicParams {
                mass_kg: 3.5,
                yaw_inertia_kgm2: 0.06,
                lf_m: 0.16,
                lr_m: 0.16,
                cf_n_per_rad: 60.0,
                cr_n_per_rad: 60.0,
            },
            limits: ActuatorLimits {
                max_steering_angle_rad: 0.4,
                max_steering_rate_rad_s: 4.0,
                max_speed_mps: 8.0,
                max_accel_mps2: 4.0,
                max_decel_mps2: 8.0,
            },
        }
    }

    pub(in crate::simulation::vehicle) fn test_nonlinear_model() -> VehicleModel {
        VehicleModel::NonlinearBicycle {
            params: NonlinearTireParams {
                mass_kg: 3.5,
                yaw_inertia_kgm2: 0.06,
                lf_m: 0.16,
                lr_m: 0.16,
                cg_height_m: 0.05,
                tire_mu: 1.1,
                pacejka_b: 2.5,
                pacejka_c: 1.3,
                front_drive_fraction: 0.5,
            },
            limits: ActuatorLimits {
                max_steering_angle_rad: 0.4,
                max_steering_rate_rad_s: 4.0,
                max_speed_mps: 8.0,
                max_accel_mps2: 4.0,
                max_decel_mps2: 8.0,
            },
        }
    }

    pub(in crate::simulation::vehicle) fn test_pacejka_model() -> VehicleModel {
        VehicleModel::PacejkaBicycle {
            params: PacejkaTireParams {
                mass_kg: 3.5,
                yaw_inertia_kgm2: 0.06,
                lf_m: 0.16,
                lr_m: 0.16,
                cg_height_m: 0.05,
                front_b: 2.5,
                front_c: 1.3,
                front_d_mu: 1.1,
                front_e: -0.5,
                rear_b: 2.5,
                rear_c: 1.3,
                rear_d_mu: 1.1,
                rear_e: -0.5,
                combined_slip_b: 1.0,
                combined_slip_c: 1.0,
                front_drive_fraction: 0.5,
            },
            limits: ActuatorLimits {
                max_steering_angle_rad: 0.4,
                max_steering_rate_rad_s: 4.0,
                max_speed_mps: 8.0,
                max_accel_mps2: 4.0,
                max_decel_mps2: 8.0,
            },
        }
    }

    pub(in crate::simulation::vehicle) fn test_two_track_model() -> VehicleModel {
        VehicleModel::TwoTrack {
            params: TwoTrackParams {
                mass_kg: 3.5,
                yaw_inertia_kgm2: 0.06,
                lf_m: 0.16,
                lr_m: 0.16,
                cg_height_m: 0.05,
                track_width_m: 0.2,
                front_b: 2.5,
                front_c: 1.3,
                front_d_mu: 1.1,
                front_e: -0.5,
                rear_b: 2.5,
                rear_c: 1.3,
                rear_d_mu: 1.1,
                rear_e: -0.5,
                combined_slip_b: 1.0,
                combined_slip_c: 1.0,
                front_drive_fraction: 0.5,
            },
            limits: ActuatorLimits {
                max_steering_angle_rad: 0.4,
                max_steering_rate_rad_s: 4.0,
                max_speed_mps: 8.0,
                max_accel_mps2: 4.0,
                max_decel_mps2: 8.0,
            },
        }
    }

    #[test]
    fn kind_of_matches_the_variant() {
        assert_eq!(kind_of(&test_model()), VehicleModelKind::Bicycle);
        assert_eq!(
            kind_of(&test_dynamic_model()),
            VehicleModelKind::DynamicBicycle
        );
        assert_eq!(
            kind_of(&test_nonlinear_model()),
            VehicleModelKind::NonlinearBicycle
        );
        assert_eq!(
            kind_of(&test_pacejka_model()),
            VehicleModelKind::PacejkaBicycle
        );
        assert_eq!(kind_of(&test_two_track_model()), VehicleModelKind::TwoTrack);
    }

    #[test]
    fn carry_over_state_maps_speed_into_the_new_models_shared_fields() {
        let old = VehicleState::Bicycle(BicycleState {
            x_m: 1.0,
            y_m: 2.0,
            heading_rad: 0.3,
            speed_mps: 4.0,
        });
        let next = carry_over_state(old, VehicleModelKind::DynamicBicycle);
        match next {
            VehicleState::DynamicBicycle(s) => {
                assert_eq!(s.x_m, 1.0);
                assert_eq!(s.y_m, 2.0);
                assert_eq!(s.heading_rad, 0.3);
                assert_eq!(s.vx_mps, 4.0);
                assert_eq!(s.vy_mps, 0.0);
                assert_eq!(s.yaw_rate_rad_s, 0.0);
            }
            _ => panic!("expected DynamicBicycle state"),
        }
    }

    #[test]
    fn carry_over_state_projects_ground_speed_back_to_the_kinematic_model() {
        let old = VehicleState::DynamicBicycle(DynamicState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            vx_mps: 3.0,
            vy_mps: 4.0,
            yaw_rate_rad_s: 1.0,
        });
        let next = carry_over_state(old, VehicleModelKind::Bicycle);
        match next {
            VehicleState::Bicycle(s) => assert!((s.speed_mps - 5.0).abs() < 1e-12),
            _ => panic!("expected Bicycle state"),
        }
    }

    #[test]
    fn state_from_start_carries_the_start_state_into_every_model_kind() {
        let start = StartState {
            x_m: 3.0,
            y_m: -2.0,
            heading_rad: 0.5,
            speed_mps: 0.0,
        };
        match state_from_start(start, VehicleModelKind::DynamicBicycle) {
            VehicleState::DynamicBicycle(s) => {
                assert_eq!(s.x_m, 3.0);
                assert_eq!(s.y_m, -2.0);
                assert_eq!(s.heading_rad, 0.5);
                assert_eq!(s.vx_mps, 0.0);
                assert_eq!(s.vy_mps, 0.0);
                assert_eq!(s.yaw_rate_rad_s, 0.0);
            }
            _ => panic!("expected DynamicBicycle state"),
        }
    }

    #[test]
    fn advance_never_exceeds_the_steering_rate_limit_in_one_tick() {
        let model = test_model();
        let state = VehicleState::Bicycle(BicycleState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            speed_mps: 0.0,
        });
        let dt_s = 0.01;
        let (_, next_steering) = advance(&model, state, 0.0, 10.0, 0.0, dt_s);
        assert!(next_steering <= 4.0 * dt_s + 1e-12);
    }

    #[test]
    fn advance_never_exceeds_the_max_steering_angle() {
        let model = test_model();
        let state = VehicleState::Bicycle(BicycleState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            speed_mps: 0.0,
        });
        let mut steering = 0.0;
        for _ in 0..1000 {
            let (_, next_steering) = advance(&model, state, steering, 10.0, 0.0, 0.01);
            steering = next_steering;
        }
        assert!(steering <= 0.4 + 1e-9);
    }

    #[test]
    fn advance_never_exceeds_the_accel_limit_in_one_tick() {
        let model = test_model();
        let state = VehicleState::Bicycle(BicycleState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            speed_mps: 0.0,
        });
        let dt_s = 0.01;
        let (next_state, _) = advance(&model, state, 0.0, 0.0, 100.0, dt_s);
        assert!(next_state.speed_mps() <= 4.0 * dt_s + 1e-12);
    }

    #[test]
    fn advance_never_exceeds_max_speed_even_at_a_large_dt() {
        let model = test_model();
        let state = VehicleState::Bicycle(BicycleState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            speed_mps: 0.0,
        });
        let (next_state, _) = advance(&model, state, 0.0, 0.0, 100.0, 10.0);
        assert!(next_state.speed_mps() <= 8.0 + 1e-9);
    }

    #[test]
    fn advance_dispatches_the_dynamic_bicycle_model_too() {
        let model = test_dynamic_model();
        let state = VehicleState::DynamicBicycle(DynamicState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            vx_mps: 0.0,
            vy_mps: 0.0,
            yaw_rate_rad_s: 0.0,
        });
        let (next_state, _) = advance(&model, state, 0.0, 0.0, 5.0, 0.1);
        match next_state {
            VehicleState::DynamicBicycle(s) => assert!(s.vx_mps > 0.0),
            _ => panic!("expected DynamicBicycle state"),
        }
    }

    #[test]
    fn advance_dispatches_the_nonlinear_bicycle_model_too() {
        let model = test_nonlinear_model();
        let state = VehicleState::NonlinearBicycle(NonlinearBicycleState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            vx_mps: 0.0,
            vy_mps: 0.0,
            yaw_rate_rad_s: 0.0,
        });
        let (next_state, _) = advance(&model, state, 0.0, 0.0, 5.0, 0.1);
        match next_state {
            VehicleState::NonlinearBicycle(s) => assert!(s.vx_mps > 0.0),
            _ => panic!("expected NonlinearBicycle state"),
        }
    }

    #[test]
    fn carry_over_state_maps_speed_into_the_nonlinear_models_shared_fields() {
        let old = VehicleState::Bicycle(BicycleState {
            x_m: 1.0,
            y_m: 2.0,
            heading_rad: 0.3,
            speed_mps: 4.0,
        });
        let next = carry_over_state(old, VehicleModelKind::NonlinearBicycle);
        match next {
            VehicleState::NonlinearBicycle(s) => {
                assert_eq!(s.x_m, 1.0);
                assert_eq!(s.y_m, 2.0);
                assert_eq!(s.heading_rad, 0.3);
                assert_eq!(s.vx_mps, 4.0);
                assert_eq!(s.vy_mps, 0.0);
                assert_eq!(s.yaw_rate_rad_s, 0.0);
            }
            _ => panic!("expected NonlinearBicycle state"),
        }
    }

    #[test]
    fn advance_dispatches_the_pacejka_bicycle_model_too() {
        let model = test_pacejka_model();
        let state = VehicleState::PacejkaBicycle(PacejkaBicycleState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            vx_mps: 0.0,
            vy_mps: 0.0,
            yaw_rate_rad_s: 0.0,
        });
        let (next_state, _) = advance(&model, state, 0.0, 0.0, 5.0, 0.1);
        match next_state {
            VehicleState::PacejkaBicycle(s) => assert!(s.vx_mps > 0.0),
            _ => panic!("expected PacejkaBicycle state"),
        }
    }

    #[test]
    fn carry_over_state_maps_speed_into_the_pacejka_models_shared_fields() {
        let old = VehicleState::Bicycle(BicycleState {
            x_m: 1.0,
            y_m: 2.0,
            heading_rad: 0.3,
            speed_mps: 4.0,
        });
        let next = carry_over_state(old, VehicleModelKind::PacejkaBicycle);
        match next {
            VehicleState::PacejkaBicycle(s) => {
                assert_eq!(s.x_m, 1.0);
                assert_eq!(s.y_m, 2.0);
                assert_eq!(s.heading_rad, 0.3);
                assert_eq!(s.vx_mps, 4.0);
                assert_eq!(s.vy_mps, 0.0);
                assert_eq!(s.yaw_rate_rad_s, 0.0);
            }
            _ => panic!("expected PacejkaBicycle state"),
        }
    }

    #[test]
    fn advance_dispatches_the_two_track_model_too() {
        let model = test_two_track_model();
        let state = VehicleState::TwoTrack(TwoTrackState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            vx_mps: 0.0,
            vy_mps: 0.0,
            yaw_rate_rad_s: 0.0,
        });
        let (next_state, _) = advance(&model, state, 0.0, 0.0, 5.0, 0.1);
        match next_state {
            VehicleState::TwoTrack(s) => assert!(s.vx_mps > 0.0),
            _ => panic!("expected TwoTrack state"),
        }
    }

    #[test]
    fn carry_over_state_maps_speed_into_the_two_track_models_shared_fields() {
        let old = VehicleState::Bicycle(BicycleState {
            x_m: 1.0,
            y_m: 2.0,
            heading_rad: 0.3,
            speed_mps: 4.0,
        });
        let next = carry_over_state(old, VehicleModelKind::TwoTrack);
        match next {
            VehicleState::TwoTrack(s) => {
                assert_eq!(s.x_m, 1.0);
                assert_eq!(s.y_m, 2.0);
                assert_eq!(s.heading_rad, 0.3);
                assert_eq!(s.vx_mps, 4.0);
                assert_eq!(s.vy_mps, 0.0);
                assert_eq!(s.yaw_rate_rad_s, 0.0);
            }
            _ => panic!("expected TwoTrack state"),
        }
    }

    #[test]
    fn body_velocity_of_the_kinematic_model_matches_its_heading_rate() {
        let model = test_model();
        let state = VehicleState::Bicycle(BicycleState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            speed_mps: 3.0,
        });
        let steering_rad = 0.2;
        let dt_s = 0.001;

        let (vx_mps, vy_mps, yaw_rate_rad_s) = body_velocity(&model, &state, steering_rad);
        // Hold the same steering so `advance` integrates exactly what
        // `body_velocity` describes.
        let (next, _) = advance(&model, state, steering_rad, steering_rad, 3.0, dt_s);

        assert!((vx_mps.hypot(vy_mps) - 3.0).abs() < 1e-12);
        assert!(vy_mps > 0.0, "steering left must slip the CG velocity left");
        let heading_rate = next.heading_rad() / dt_s;
        assert!(
            (yaw_rate_rad_s - heading_rate).abs() < 1e-3,
            "yaw rate {yaw_rate_rad_s} vs integrated heading rate {heading_rate}"
        );
    }

    #[test]
    fn body_velocity_of_a_dynamic_model_is_read_off_its_state() {
        let state = VehicleState::DynamicBicycle(DynamicState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            vx_mps: 2.0,
            vy_mps: -0.1,
            yaw_rate_rad_s: 0.7,
        });
        assert_eq!(
            body_velocity(&test_dynamic_model(), &state, 0.3),
            (2.0, -0.1, 0.7)
        );
    }

    #[test]
    fn body_acceleration_in_a_steady_turn_is_purely_centripetal() {
        // Constant body-frame velocity while yawing: the only acceleration is
        // v * yaw_rate = v^2 / R, pointing left for a left turn.
        let (ax_mps2, ay_mps2) = body_acceleration((3.0, 0.0), (3.0, 0.0), 1.5, 0.01);
        assert_eq!(ax_mps2, 0.0);
        assert!((ay_mps2 - 4.5).abs() < 1e-12);
    }

    #[test]
    fn body_acceleration_on_a_straight_line_is_the_speed_change_rate() {
        let (ax_mps2, ay_mps2) = body_acceleration((1.0, 0.0), (1.02, 0.0), 0.0, 0.01);
        assert!((ax_mps2 - 2.0).abs() < 1e-9);
        assert_eq!(ay_mps2, 0.0);
    }
}
