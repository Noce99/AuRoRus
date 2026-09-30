//! [`SimulatedVehicle`]: runs a vehicle model forward in time under
//! [`AUTONOMOUS_VESC_COMMAND_TOPIC_NAME`], overridden by
//! [`HUMAN_VESC_COMMAND_TOPIC_NAME`] whenever a human is driving (see
//! [`select_command`]), publishing the result on
//! its [`VehicleTopics::vehicle_status`] and its actuator limits on its
//! [`VehicleTopics::vehicle_limits`]. Also watches
//! [`VEHICLE_MODEL_SELECTION_TOPIC_NAME`] for a live model switch (e.g. from
//! `web_gui`), publishing the currently running model on
//! [`VEHICLE_MODEL_STATUS_TOPIC_NAME`] (along with its live-tunable
//! parameters, applied from [`VEHICLE_MODEL_PARAMETERS_TOPIC_NAME`]), and
//! draws the vehicle on its own drawing topic (see [`crate::topics::Drawing`]).

use crate::environment::simulator::vehicle::{
    BicycleParams, BicycleState, DynamicParams, DynamicState, NonlinearBicycleState,
    NonlinearTireParams, PacejkaBicycleState, PacejkaTireParams, TwoTrackParams, TwoTrackState,
    dynamic_step, nonlinear_step, pacejka_step, step as bicycle_step, two_track_step,
};
pub use crate::topics::ActuatorLimits;
use crate::topics::{
    AUTONOMOUS_VESC_COMMAND_TOPIC_NAME, AlgorithmParameter, Color, Drawing,
    HUMAN_VESC_COMMAND_TOPIC_NAME, Placement, PlacementTopics, Shape, StartState,
    VEHICLE_BODY_LENGTH_M, VEHICLE_BODY_WIDTH_M, VEHICLE_MODEL_PARAMETERS_TOPIC_NAME,
    VEHICLE_MODEL_SELECTION_TOPIC_NAME, VEHICLE_MODEL_STATUS_TOPIC_NAME, VESC_COMMAND_TIMEOUT,
    VehicleModelKind, VehicleModelParameters, VehicleModelSelection, VehicleModelStatus,
    VehicleStatus, VehicleTopics, VescCommand, now_ms,
};
use crate::{Captain, Executor, RwLockTopic, Stamped, Ticker};
use std::any::Any;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

/// Which vehicle model [`SimulatedVehicle`] should run, and the geometry and
/// actuator limits it needs to do so. A plain `enum` (rather than a trait)
/// because the set of models is small and known at compile time - see
/// `src/environment/simulator/vehicle.rs`'s module doc comment for why the
/// same choice was made there. Its companion state type is [`VehicleState`];
/// a new model adds one variant here, one to `VehicleState`, and one match
/// arm in [`advance`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VehicleModel {
    /// A CG-referenced kinematic bicycle model - see
    /// [`crate::environment::simulator::vehicle::bicycle`].
    Bicycle {
        params: BicycleParams,
        limits: ActuatorLimits,
    },
    /// A dynamic bicycle model with lateral tire forces - see
    /// [`crate::environment::simulator::vehicle::dynamic_bicycle`].
    DynamicBicycle {
        params: DynamicParams,
        limits: ActuatorLimits,
    },
    /// A dynamic bicycle model with tire saturation, load transfer, and
    /// combined slip - see
    /// [`crate::environment::simulator::vehicle::nonlinear_bicycle`].
    NonlinearBicycle {
        params: NonlinearTireParams,
        limits: ActuatorLimits,
    },
    /// A dynamic bicycle model using the full Pacejka Magic Formula for
    /// lateral tire force - see
    /// [`crate::environment::simulator::vehicle::pacejka_bicycle`].
    PacejkaBicycle {
        params: PacejkaTireParams,
        limits: ActuatorLimits,
    },
    /// A two-track (four-wheel) model with lateral load transfer and
    /// per-wheel asymmetry - see
    /// [`crate::environment::simulator::vehicle::two_track`].
    TwoTrack {
        params: TwoTrackParams,
        limits: ActuatorLimits,
    },
}

/// The state a [`VehicleModel`] is advancing, in the same variant as the
/// model currently running it - [`SimulatedVehicle::run`] always keeps the
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
    fn x_m(&self) -> f64 {
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
    fn y_m(&self) -> f64 {
        match self {
            Self::Bicycle(s) => s.y_m,
            Self::DynamicBicycle(s) => s.y_m,
            Self::NonlinearBicycle(s) => s.y_m,
            Self::PacejkaBicycle(s) => s.y_m,
            Self::TwoTrack(s) => s.y_m,
        }
    }

    /// Heading of the vehicle body, in radians - common to every model.
    fn heading_rad(&self) -> f64 {
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
    fn speed_mps(&self) -> f64 {
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
    fn longitudinal_speed_mps(&self) -> f64 {
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
/// used by [`SimulatedVehicle::run`] without a `match` at every call site.
fn limits_of(model: &VehicleModel) -> ActuatorLimits {
    match model {
        VehicleModel::Bicycle { limits, .. } => *limits,
        VehicleModel::DynamicBicycle { limits, .. } => *limits,
        VehicleModel::NonlinearBicycle { limits, .. } => *limits,
        VehicleModel::PacejkaBicycle { limits, .. } => *limits,
        VehicleModel::TwoTrack { limits, .. } => *limits,
    }
}

/// Which [`VehicleModelKind`] `model` is an instance of.
fn kind_of(model: &VehicleModel) -> VehicleModelKind {
    match model {
        VehicleModel::Bicycle { .. } => VehicleModelKind::Bicycle,
        VehicleModel::DynamicBicycle { .. } => VehicleModelKind::DynamicBicycle,
        VehicleModel::NonlinearBicycle { .. } => VehicleModelKind::NonlinearBicycle,
        VehicleModel::PacejkaBicycle { .. } => VehicleModelKind::PacejkaBicycle,
        VehicleModel::TwoTrack { .. } => VehicleModelKind::TwoTrack,
    }
}

/// Every tunable parameter [`SimulatedVehicle`] needs: how often it ticks,
/// the [`ActuatorLimits`] shared by every model kind, and each kind's own
/// physical parameters - loaded from `config/actuators/simulated_vehicle.toml`
/// (see [`Default`]) or from an arbitrary path via [`crate::config::load`].
#[derive(Debug, Clone, PartialEq, serde::Deserialize)]
pub struct SimulatedVehicleConfig {
    /// How often [`SimulatedVehicle`] advances the model and republishes
    /// [`VehicleStatus`], and how often it checks
    /// [`VEHICLE_MODEL_SELECTION_TOPIC_NAME`] for a wanted model switch, in
    /// Hz.
    pub tick_rate_hz: f64,
    /// Shared by every [`VehicleModelKind`] - see [`ActuatorLimits`].
    pub limits: ActuatorLimits,
    pub bicycle: BicycleParams,
    pub dynamic_bicycle: DynamicParams,
    pub nonlinear_bicycle: NonlinearTireParams,
    pub pacejka_bicycle: PacejkaTireParams,
    pub two_track: TwoTrackParams,
}

impl Default for SimulatedVehicleConfig {
    /// RC-car-scale geometry and actuator limits for a small (roughly
    /// 1/10-scale) RC racecar, matching the kind of track `generate_map`
    /// produces, from the checked-in `config/actuators/simulated_vehicle.toml`.
    /// `dynamic_bicycle`'s mass/inertia/cornering-stiffness values are
    /// placeholder estimates for that same scale of vehicle, not measured -
    /// tune them once a real (or more carefully modeled) vehicle is
    /// available. This is a deliberate exception to
    /// [`DynamicParams`]/[`BicycleParams`] having no [`Default`]: a live,
    /// web-selectable model needs *some* starting parameters for a kind the
    /// caller only names, not configures.
    fn default() -> Self {
        toml::from_str(include_str!(
            "../../config/actuators/simulated_vehicle.toml"
        ))
        .expect(
            "config/actuators/simulated_vehicle.toml must deserialize into SimulatedVehicleConfig",
        )
    }
}

/// The default [`VehicleModel`] for `kind`, built from `config`.
pub fn default_model(kind: VehicleModelKind, config: &SimulatedVehicleConfig) -> VehicleModel {
    match kind {
        VehicleModelKind::Bicycle => VehicleModel::Bicycle {
            params: config.bicycle,
            limits: config.limits,
        },
        VehicleModelKind::DynamicBicycle => VehicleModel::DynamicBicycle {
            params: config.dynamic_bicycle,
            limits: config.limits,
        },
        VehicleModelKind::NonlinearBicycle => VehicleModel::NonlinearBicycle {
            params: config.nonlinear_bicycle,
            limits: config.limits,
        },
        VehicleModelKind::PacejkaBicycle => VehicleModel::PacejkaBicycle {
            params: config.pacejka_bicycle,
            limits: config.limits,
        },
        VehicleModelKind::TwoTrack => VehicleModel::TwoTrack {
            params: config.two_track,
            limits: config.limits,
        },
    }
}

/// Where [`SimulatedVehicle`]'s config lives:
/// `config/actuators/simulated_vehicle.toml`, relative to the working
/// directory - rewritten by [`save_parameters`], and reread on a restart.
pub fn config_path() -> PathBuf {
    Path::new(crate::config::DEFAULT_CONFIG_ROOT)
        .join("actuators")
        .join("simulated_vehicle.toml")
}

/// `kind`'s live-tunable parameters, with the values `config` holds for it.
fn tunable_parameters(
    kind: VehicleModelKind,
    config: &SimulatedVehicleConfig,
) -> Vec<AlgorithmParameter> {
    fn with_values(
        mut parameters: Vec<AlgorithmParameter>,
        params: &impl serde::Serialize,
    ) -> Vec<AlgorithmParameter> {
        crate::config::refresh_parameter_values(&mut parameters, params);
        parameters
    }
    match kind {
        VehicleModelKind::Bicycle => {
            with_values(BicycleParams::tunable_parameters(), &config.bicycle)
        }
        VehicleModelKind::DynamicBicycle => {
            with_values(DynamicParams::tunable_parameters(), &config.dynamic_bicycle)
        }
        VehicleModelKind::NonlinearBicycle => with_values(
            NonlinearTireParams::tunable_parameters(),
            &config.nonlinear_bicycle,
        ),
        VehicleModelKind::PacejkaBicycle => with_values(
            PacejkaTireParams::tunable_parameters(),
            &config.pacejka_bicycle,
        ),
        VehicleModelKind::TwoTrack => {
            with_values(TwoTrackParams::tunable_parameters(), &config.two_track)
        }
    }
}

/// Applies `wanted` to `kind`'s parameters in `config`, each sanitized (see
/// [`crate::config::apply_parameters`]). Returns whether anything changed.
fn apply_wanted(
    kind: VehicleModelKind,
    config: &mut SimulatedVehicleConfig,
    wanted: &BTreeMap<String, f64>,
) -> bool {
    use crate::config::apply_parameters as apply;
    let parameters = tunable_parameters(kind, config);
    match kind {
        VehicleModelKind::Bicycle => apply(&mut config.bicycle, &parameters, wanted),
        VehicleModelKind::DynamicBicycle => apply(&mut config.dynamic_bicycle, &parameters, wanted),
        VehicleModelKind::NonlinearBicycle => {
            apply(&mut config.nonlinear_bicycle, &parameters, wanted)
        }
        VehicleModelKind::PacejkaBicycle => apply(&mut config.pacejka_bicycle, &parameters, wanted),
        VehicleModelKind::TwoTrack => apply(&mut config.two_track, &parameters, wanted),
    }
}

/// What [`SimulatedVehicle`] publishes on [`VEHICLE_MODEL_STATUS_TOPIC_NAME`]
/// while running `kind` with `config`.
fn model_status(kind: VehicleModelKind, config: &SimulatedVehicleConfig) -> VehicleModelStatus {
    let mut limits = ActuatorLimits::tunable_parameters();
    crate::config::refresh_parameter_values(&mut limits, &config.limits);
    VehicleModelStatus {
        kind,
        parameters: tunable_parameters(kind, config),
        limits,
    }
}

/// Writes `parameters`' values into `kind`'s `[<kind>]` table of
/// [`config_path`], leaving everything else in the file - comments, other
/// tables, layout - untouched. Returns the file's path.
pub fn save_parameters(
    kind: VehicleModelKind,
    parameters: &[AlgorithmParameter],
) -> Result<PathBuf, String> {
    save_table(kind.api_str(), parameters)
}

/// Like [`save_parameters`], for the actuator limits' `[limits]` table.
pub fn save_limits(parameters: &[AlgorithmParameter]) -> Result<PathBuf, String> {
    save_table("limits", parameters)
}

/// The values `kind`'s `[<kind>]` table of [`config_path`] holds for
/// `parameters`, by name, and the file's path - e.g. to go back to what was
/// last saved.
pub fn saved_values(
    kind: VehicleModelKind,
    parameters: &[AlgorithmParameter],
) -> Result<(BTreeMap<String, f64>, PathBuf), String> {
    saved_table_values(kind.api_str(), parameters)
}

/// Like [`saved_values`], for the actuator limits' `[limits]` table.
pub fn saved_limits(
    parameters: &[AlgorithmParameter],
) -> Result<(BTreeMap<String, f64>, PathBuf), String> {
    saved_table_values("limits", parameters)
}

fn saved_table_values(
    table: &str,
    parameters: &[AlgorithmParameter],
) -> Result<(BTreeMap<String, f64>, PathBuf), String> {
    let path = config_path();
    let names: Vec<&str> = parameters
        .iter()
        .map(|parameter| parameter.name.as_str())
        .collect();
    Ok((
        crate::config::load_toml_values(&path, Some(table), &names)?,
        path,
    ))
}

fn save_table(table: &str, parameters: &[AlgorithmParameter]) -> Result<PathBuf, String> {
    let path = config_path();
    let values: Vec<(&str, String)> = parameters
        .iter()
        .map(|parameter| {
            (
                parameter.name.as_str(),
                crate::config::parameter_toml_value(parameter),
            )
        })
        .collect();
    crate::config::save_toml_values(&path, Some(table), &values)?;
    Ok(path)
}

/// Builds a [`VehicleState`] of `kind` from `start`'s shared fields (position,
/// heading, speed) - used for the very first state [`SimulatedVehicle::run`]
/// advances, so the vehicle starts wherever [`START_STATE_TOPIC_NAME`] says
/// to, zeroed on every other field. Implemented via [`carry_over_state`],
/// which does exactly this projection when switching models at runtime.
fn state_from_start(start: StartState, kind: VehicleModelKind) -> VehicleState {
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
/// `old` didn't - used when [`SimulatedVehicle::run`] switches models at
/// runtime, so the vehicle doesn't visibly jump when the model underneath it
/// changes mid-drive.
fn carry_over_state(old: VehicleState, new_kind: VehicleModelKind) -> VehicleState {
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
/// call from [`SimulatedVehicle::run`], which always switches both together.
fn advance(
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
/// [`crate::environment::simulator::vehicle::bicycle`]).
///
/// `model` and `state` must be the same [`VehicleModelKind`], as for
/// [`advance`].
fn body_velocity(
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
fn body_acceleration(
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

/// [`Drawing::z_index`] of the ego vehicle's drawing, and of an opponent's
/// (see [`SimulatedVehicle::opponent`]) - underneath it.
const VEHICLE_Z_INDEX: i32 = 10;
const OPPONENT_Z_INDEX: i32 = 9;

/// The distances from the model's reference point to its front and rear
/// axles - what [`drawing`] turns the front wheels about.
fn axles_of(model: &VehicleModel) -> (f64, f64) {
    match model {
        VehicleModel::Bicycle { params, .. } => (params.lf_m, params.lr_m),
        VehicleModel::DynamicBicycle { params, .. } => (params.lf_m, params.lr_m),
        VehicleModel::NonlinearBicycle { params, .. } => (params.lf_m, params.lr_m),
        VehicleModel::PacejkaBicycle { params, .. } => (params.lf_m, params.lr_m),
        VehicleModel::TwoTrack { params, .. } => (params.lf_m, params.lr_m),
    }
}

/// What [`SimulatedVehicle`] publishes on its drawing topic every tick: the
/// vehicle at `state`, front wheels turned by `steering_angle_rad`, in `color`.
fn drawing(
    model: &VehicleModel,
    state: &VehicleState,
    steering_angle_rad: f64,
    color: Color,
) -> Drawing {
    let (front_axle_m, rear_axle_m) = axles_of(model);
    Drawing::default()
        .element(
            "Vehicle",
            [Shape::Vehicle {
                x_m: state.x_m(),
                y_m: state.y_m(),
                heading_rad: state.heading_rad(),
                // Signed, unlike `VehicleStatus::speed_mps`, so a viewer dead-reckoning
                // the vehicle between samples moves it backward while reversing.
                speed_mps: state.longitudinal_speed_mps(),
                steering_rad: steering_angle_rad,
                length_m: VEHICLE_BODY_LENGTH_M,
                width_m: VEHICLE_BODY_WIDTH_M,
                front_axle_m,
                rear_axle_m,
                color,
            }],
            true,
        )
        .stale_after(Drawing::DEFAULT_STALE_AFTER)
        .z_index(VEHICLE_Z_INDEX)
}

/// Whether `command` was written, and recently enough to act on - see
/// [`VESC_COMMAND_TIMEOUT`].
fn is_fresh(command: &Stamped<VescCommand>) -> bool {
    is_fresh_within(command, VESC_COMMAND_TIMEOUT)
}

/// Whether `command` was written at most `timeout` ago.
fn is_fresh_within(command: &Stamped<VescCommand>, timeout: Duration) -> bool {
    command.age().is_some_and(|age| age <= timeout)
}

/// Picks the command to act on this tick. The human one always overrides:
/// it wins whenever it's fresh and asks for anything at all - `web_gui`
/// re-sends a stationary, centered command every few hundred milliseconds
/// while no control is held, so "fresh" alone can't mean "the human is
/// driving". Otherwise the autonomous one is used if fresh, and a stationary,
/// centered command if neither is - so a writer that stopped publishing
/// never leaves its last setpoint latched.
fn select_command(autonomous: Stamped<VescCommand>, human: Stamped<VescCommand>) -> VescCommand {
    select_command_within(autonomous, human, VESC_COMMAND_TIMEOUT)
}

/// [`select_command`], with commands older than `timeout` counting as stale
/// - the real car's (see [`super::Vesc`]) is shorter than the simulator's.
pub(super) fn select_command_within(
    autonomous: Stamped<VescCommand>,
    human: Stamped<VescCommand>,
    timeout: Duration,
) -> VescCommand {
    if is_fresh_within(&human, timeout) && human.value != VescCommand::default() {
        human.value
    } else if is_fresh_within(&autonomous, timeout) {
        autonomous.value
    } else {
        VescCommand::default()
    }
}

/// The command an opponent acts on this tick: its algorithm's `command`, with
/// the speed scaled by `speed_scale`, if fresh - else a stationary, centered
/// one, like [`select_command`].
fn opponent_command(command: Stamped<VescCommand>, speed_scale: f64) -> VescCommand {
    if is_fresh(&command) {
        VescCommand {
            speed_mps: command.value.speed_mps * speed_scale,
            ..command.value
        }
    } else {
        VescCommand::default()
    }
}

/// What makes a [`SimulatedVehicle`] an opponent rather than the ego vehicle
/// - see [`SimulatedVehicle::opponent`].
#[derive(Debug, Clone, PartialEq)]
pub struct OpponentVehicle {
    /// The only command it acts on: its autonomous algorithm's own.
    pub command_topic: String,
    /// Multiplies every commanded speed, in `0..=1`.
    pub speed_scale: f64,
    /// What it's drawn in.
    pub color: Color,
}

/// The ego vehicle's topics that an opponent doesn't have: the human's
/// commands and the live model switching and tuning.
struct EgoTopics {
    human: Arc<RwLockTopic<VescCommand>>,
    model_selection: Arc<RwLockTopic<VehicleModelSelection>>,
    model_status: Arc<RwLockTopic<VehicleModelStatus>>,
    /// Nothing may publish parameter requests at all (e.g. a binary without
    /// `web_gui`) - then the config stays as loaded.
    parameters: Option<Arc<RwLockTopic<VehicleModelParameters>>>,
}

/// Runs `model` forward in time at [`SimulatedVehicleConfig::tick_rate_hz`], reading
/// [`AUTONOMOUS_VESC_COMMAND_TOPIC_NAME`]/[`HUMAN_VESC_COMMAND_TOPIC_NAME`] each tick
/// (see [`select_command`]) and publishing the resulting [`VehicleStatus`]. Starts at whatever
/// [`START_STATE_TOPIC_NAME`] holds at that moment (the world origin,
/// stationary, if [`crate::sensors::MapServer`] hasn't published one yet),
/// with the steering centered, and places the vehicle there again - steering
/// re-centered - every time [`START_STATE_TOPIC_NAME`] changes (e.g. a map
/// change) or [`PLACE_AT_START_TOPIC_NAME`] is bumped (e.g. `web_gui`'s "P"
/// key) - at the pose that request carries, if any - or a race starts
/// (see [`Placement`]), holding it still on its grid slot until the go (see
/// [`crate::topics::RaceStart`]). Also watches [`VEHICLE_MODEL_SELECTION_TOPIC_NAME`] each tick and
/// switches to [`default_model`] of the wanted kind - carrying over shared
/// state (see [`carry_over_state`]) - whenever it no longer matches the
/// model currently running.
///
/// An opponent (see [`SimulatedVehicle::opponent`]) instead acts only on its
/// algorithm's command, and keeps the model it was built with.
pub struct SimulatedVehicle {
    id: u16,
    name: String,
    model: VehicleModel,
    config: SimulatedVehicleConfig,
    /// Where its own topics (`vehicle_status`, `vehicle_limits`) live.
    vehicle: VehicleTopics,
    /// `None` for the ego vehicle.
    opponent: Option<OpponentVehicle>,
}

impl SimulatedVehicle {
    /// Creates a `SimulatedVehicle` that will run `model` once started,
    /// ticking and switching models per `config`.
    pub fn new(
        name: impl Into<String>,
        model: VehicleModel,
        config: SimulatedVehicleConfig,
    ) -> Self {
        Self {
            id: 0,
            name: name.into(),
            model,
            config,
            vehicle: VehicleTopics::ego(),
            opponent: None,
        }
    }

    /// Creates an opponent: a vehicle running `model` - never switched nor
    /// tuned live - with `config.limits`, publishing on `vehicle`'s topics
    /// and acting only on `opponent.command_topic`. Like the ego vehicle, it
    /// starts at, and is placed back at, the `start_state`.
    pub fn opponent(
        name: impl Into<String>,
        model: VehicleModel,
        config: SimulatedVehicleConfig,
        vehicle: VehicleTopics,
        opponent: OpponentVehicle,
    ) -> Self {
        Self {
            id: 0,
            name: name.into(),
            model,
            config,
            vehicle,
            opponent: Some(opponent),
        }
    }
}

/// `file_config` with the model the ego vehicle currently runs (`ego`, as it
/// publishes it on [`VEHICLE_MODEL_STATUS_TOPIC_NAME`]) and `limits` - and
/// that model - for an opponent (see [`SimulatedVehicle::opponent`]).
pub fn opponent_model(
    mut file_config: SimulatedVehicleConfig,
    ego: &VehicleModelStatus,
    limits: ActuatorLimits,
) -> (VehicleModel, SimulatedVehicleConfig) {
    let wanted: BTreeMap<String, f64> = ego
        .parameters
        .iter()
        .map(|parameter| (parameter.name.clone(), parameter.value))
        .collect();
    apply_wanted(ego.kind, &mut file_config, &wanted);
    file_config.limits = limits;
    (default_model(ego.kind, &file_config), file_config)
}

impl Executor for SimulatedVehicle {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_writer::<VehicleStatus>(
            &self.vehicle.vehicle_status(),
            self.id,
            VehicleStatus::default,
        );
        if self.opponent.is_none() {
            captain.claim_writer::<VehicleModelStatus>(
                VEHICLE_MODEL_STATUS_TOPIC_NAME,
                self.id,
                VehicleModelStatus::default,
            );
        }
        // Every model kind shares `config.limits`, so a model switch never
        // changes it - only live tuning does, which rewrites it.
        let limits = self.config.limits;
        captain.claim_writer::<ActuatorLimits>(
            &self.vehicle.vehicle_limits(),
            self.id,
            move || limits,
        );
        captain.claim_drawing(self.id);
    }

    fn run(&mut self, captain: &Captain) {
        let status_topic = captain.topic::<VehicleStatus>(&self.vehicle.vehicle_status());
        let autonomous_topic = captain.topic::<VescCommand>(
            self.opponent
                .as_ref()
                .map_or(AUTONOMOUS_VESC_COMMAND_TOPIC_NAME, |opponent| {
                    &opponent.command_topic
                }),
        );
        let ego = self.opponent.is_none().then(|| EgoTopics {
            human: captain.topic::<VescCommand>(HUMAN_VESC_COMMAND_TOPIC_NAME),
            model_selection: captain
                .topic::<VehicleModelSelection>(VEHICLE_MODEL_SELECTION_TOPIC_NAME),
            model_status: captain.topic::<VehicleModelStatus>(VEHICLE_MODEL_STATUS_TOPIC_NAME),
            parameters: captain
                .try_topic::<VehicleModelParameters>(VEHICLE_MODEL_PARAMETERS_TOPIC_NAME),
        });
        let (color, speed_scale) = self
            .opponent
            .as_ref()
            .map_or((Color::AMBER, 1.0), |opponent| {
                (opponent.color, opponent.speed_scale)
            });
        // Opponents are painted just underneath the ego vehicle, which a
        // viewer takes as the vehicle - to follow, and to show the speed of.
        let z_index = if self.opponent.is_some() {
            OPPONENT_Z_INDEX
        } else {
            VEHICLE_Z_INDEX
        };
        let placement_topics = PlacementTopics::new(captain);
        let limits_topic = captain.topic::<ActuatorLimits>(&self.vehicle.vehicle_limits());
        let drawing_topic = captain.drawing(self.id);

        // Tuned live, so kept apart from `self.config`: a restart rereads the
        // file (see `fresh`) rather than keeping unsaved values.
        let mut config = self.config.clone();
        // `write_count` of the last `VehicleModelParameters` looked at, so an
        // unchanged request costs one counter read per tick.
        let mut seen_parameters_write_count = 0;

        let mut applied_kind = kind_of(&self.model);
        if let Some(ego) = &ego {
            ego.model_status
                .write(self.id, model_status(applied_kind, &config))
                .expect("lost writer authorization for the vehicle_model_status topic");
        }

        let mut placement = Placement::new(&placement_topics.read());
        let mut state = state_from_start(placement.anchor(), applied_kind);
        let mut steering_angle_rad = 0.0;
        // Last tick's body-frame velocity, to differentiate into
        // `VehicleStatus`'s accelerations. `None` whenever the state was just
        // replaced wholesale (a placement or a model switch) rather than
        // integrated, so that jump never reads as an acceleration spike.
        let mut previous_body_velocity: Option<(f64, f64)> = None;
        // The model integrates a fixed `dt_s` per tick, so the loop has to
        // actually run at `tick_rate_hz` for simulated time to track real
        // time - which is what `Ticker` (unlike a fixed sleep) guarantees.
        // Keeping `dt_s` nominal rather than measuring each period keeps the
        // physics deterministic and reproducible.
        let dt_s = 1.0 / self.config.tick_rate_hz;
        let mut ticker = Ticker::new(self.config.tick_rate_hz);

        while captain.is_running(self.id) {
            if let Some(ego) = &ego {
                let wanted_kind = ego.model_selection.read().kind;
                if wanted_kind != applied_kind {
                    self.model = default_model(wanted_kind, &config);
                    state = carry_over_state(state, wanted_kind);
                    applied_kind = wanted_kind;
                    previous_body_velocity = None;
                    ego.model_status
                        .write(self.id, model_status(applied_kind, &config))
                        .expect("lost writer authorization for the vehicle_model_status topic");
                }
            }

            if let Some(ego) = &ego
                && let Some(topic) = &ego.parameters
                && topic.meta().write_count != seen_parameters_write_count
            {
                let requests = topic.read();
                seen_parameters_write_count = requests.meta.write_count;
                let model_changed = requests
                    .value
                    .values
                    .get(applied_kind.api_str())
                    .is_some_and(|wanted| apply_wanted(applied_kind, &mut config, wanted));
                let limits_changed = crate::config::apply_parameters(
                    &mut config.limits,
                    &ActuatorLimits::tunable_parameters(),
                    &requests.value.limits,
                );
                if limits_changed {
                    limits_topic
                        .write(self.id, config.limits)
                        .expect("lost writer authorization for the vehicle_limits topic");
                }
                if model_changed || limits_changed {
                    // Same kind, so the state carries over untouched.
                    self.model = default_model(applied_kind, &config);
                    ego.model_status
                        .write(self.id, model_status(applied_kind, &config))
                        .expect("lost writer authorization for the vehicle_model_status topic");
                }
            }

            // Only the ego vehicle follows a placement at a pose of its own.
            let placement_inputs = placement_topics.read();
            if let Some(anchor) = placement.update(&placement_inputs, &self.vehicle) {
                state = state_from_start(anchor, applied_kind);
                steering_angle_rad = 0.0;
                previous_body_velocity = None;
            }

            // On a race's grid, every command - even a human's - waits for
            // the go.
            let command = if placement_inputs.race.holds(&self.vehicle, now_ms()) {
                VescCommand::default()
            } else {
                match &ego {
                    Some(ego) => select_command(autonomous_topic.read(), ego.human.read()),
                    None => opponent_command(autonomous_topic.read(), speed_scale),
                }
            };

            let (next_state, next_steering_rad) = advance(
                &self.model,
                state,
                steering_angle_rad,
                command.servo_position_rad,
                command.speed_mps,
                dt_s,
            );
            state = next_state;
            steering_angle_rad = next_steering_rad;

            let (vx_mps, vy_mps, yaw_rate_rad_s) =
                body_velocity(&self.model, &state, steering_angle_rad);
            let (ax_mps2, ay_mps2) = match previous_body_velocity {
                Some(previous) => {
                    body_acceleration(previous, (vx_mps, vy_mps), yaw_rate_rad_s, dt_s)
                }
                None => (0.0, 0.0),
            };
            previous_body_velocity = Some((vx_mps, vy_mps));

            status_topic
                .write(
                    self.id,
                    VehicleStatus {
                        x_m: state.x_m(),
                        y_m: state.y_m(),
                        heading_rad: state.heading_rad(),
                        speed_mps: state.speed_mps(),
                        vx_mps,
                        vy_mps,
                        yaw_rate_rad_s,
                        ax_mps2,
                        ay_mps2,
                    },
                )
                .expect("lost writer authorization for the vehicle_status topic");
            drawing_topic
                .write(
                    self.id,
                    drawing(&self.model, &state, steering_angle_rad, color).z_index(z_index),
                )
                .expect("lost writer authorization for the vehicle's drawing topic");

            ticker.wait();
        }
    }

    fn name(&self) -> String {
        self.name.clone()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn fresh(&self) -> Box<dyn Executor> {
        // An opponent is never restarted (see `crate::Captain::spawn_group`),
        // but a fresh one would start over with the same model and config.
        if let Some(opponent) = &self.opponent {
            return Box::new(SimulatedVehicle::opponent(
                self.name.clone(),
                default_model(kind_of(&self.model), &self.config),
                self.config.clone(),
                self.vehicle.clone(),
                opponent.clone(),
            ));
        }
        // Rebuilds `model` via `default_model`, rather than cloning `self.model`,
        // so a restart resets the vehicle's simulated state (position, velocity,
        // ...) even if the model kind was switched mid-run via
        // `VEHICLE_MODEL_SELECTION_TOPIC_NAME` - only the *kind* carries over.
        // The config is reread, so parameters saved from the UI apply now.
        let kind = kind_of(&self.model);
        let config = crate::config::load(&config_path()).unwrap_or_else(|err| {
            eprintln!("{}: {err} - keeping the config it started with", self.name);
            self.config.clone()
        });
        Box::new(SimulatedVehicle::new(
            self.name.clone(),
            default_model(kind, &config),
            config,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::WriteMeta;

    fn test_model() -> VehicleModel {
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

    fn test_dynamic_model() -> VehicleModel {
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

    fn test_nonlinear_model() -> VehicleModel {
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

    fn test_pacejka_model() -> VehicleModel {
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

    fn test_two_track_model() -> VehicleModel {
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
    fn default_limits_validate() {
        assert!(
            matches!(test_model(), VehicleModel::Bicycle { limits, .. } if limits.validate().is_ok())
        );
    }

    #[test]
    fn default_model_validates_for_every_kind() {
        let config = SimulatedVehicleConfig::default();
        for kind in [
            VehicleModelKind::Bicycle,
            VehicleModelKind::DynamicBicycle,
            VehicleModelKind::NonlinearBicycle,
            VehicleModelKind::PacejkaBicycle,
            VehicleModelKind::TwoTrack,
        ] {
            match default_model(kind, &config) {
                VehicleModel::Bicycle { params, limits } => {
                    assert!(params.validate().is_ok());
                    assert!(limits.validate().is_ok());
                }
                VehicleModel::DynamicBicycle { params, limits } => {
                    assert!(params.validate().is_ok());
                    assert!(limits.validate().is_ok());
                }
                VehicleModel::NonlinearBicycle { params, limits } => {
                    assert!(params.validate().is_ok());
                    assert!(limits.validate().is_ok());
                }
                VehicleModel::PacejkaBicycle { params, limits } => {
                    assert!(params.validate().is_ok());
                    assert!(limits.validate().is_ok());
                }
                VehicleModel::TwoTrack { params, limits } => {
                    assert!(params.validate().is_ok());
                    assert!(limits.validate().is_ok());
                }
            }
        }
    }

    /// Every kind declares one tunable parameter per field of its params
    /// (a mismatched name panics in `tunable_parameters`), each starting
    /// from the checked-in config's value and within its own range - so
    /// saving untouched values never changes the file.
    #[test]
    fn every_kind_declares_a_parameter_per_field_within_range() {
        let config = SimulatedVehicleConfig::default();
        for (kind, ..) in VehicleModelKind::ALL {
            let parameters = tunable_parameters(*kind, &config);
            let json = match kind {
                VehicleModelKind::Bicycle => serde_json::to_value(config.bicycle),
                VehicleModelKind::DynamicBicycle => serde_json::to_value(config.dynamic_bicycle),
                VehicleModelKind::NonlinearBicycle => {
                    serde_json::to_value(config.nonlinear_bicycle)
                }
                VehicleModelKind::PacejkaBicycle => serde_json::to_value(config.pacejka_bicycle),
                VehicleModelKind::TwoTrack => serde_json::to_value(config.two_track),
            }
            .unwrap();
            assert_eq!(
                parameters.len(),
                json.as_object().unwrap().len(),
                "{kind:?} doesn't declare every field"
            );
            for parameter in &parameters {
                assert_eq!(
                    parameter.kind.sanitize(parameter.value),
                    Some(parameter.value),
                    "{kind:?}'s {} default is outside its range",
                    parameter.name
                );
            }
        }
    }

    #[test]
    fn limits_declare_a_parameter_per_field_within_range() {
        let config = SimulatedVehicleConfig::default();
        let limits = model_status(VehicleModelKind::Bicycle, &config).limits;
        let json = serde_json::to_value(config.limits).unwrap();
        assert_eq!(limits.len(), json.as_object().unwrap().len());
        for parameter in &limits {
            assert_eq!(
                parameter.kind.sanitize(parameter.value),
                Some(parameter.value),
                "{}'s default is outside its range",
                parameter.name
            );
        }
    }

    #[test]
    fn wanted_values_apply_to_the_running_kind_only() {
        let mut config = SimulatedVehicleConfig::default();
        let wanted = BTreeMap::from([("lf_m".to_string(), 0.3), ("mass_kg".to_string(), 99.0)]);
        assert!(apply_wanted(
            VehicleModelKind::DynamicBicycle,
            &mut config,
            &wanted
        ));
        assert_eq!(config.dynamic_bicycle.lf_m, 0.3);
        // Clamped to its range.
        assert_eq!(config.dynamic_bicycle.mass_kg, 15.0);
        assert_eq!(config.bicycle, SimulatedVehicleConfig::default().bicycle);
        assert!(!apply_wanted(
            VehicleModelKind::DynamicBicycle,
            &mut config,
            &wanted
        ));

        let status = model_status(VehicleModelKind::DynamicBicycle, &config);
        let lf = status.parameters.iter().find(|p| p.name == "lf_m").unwrap();
        assert_eq!(lf.value, 0.3);
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

    fn written(
        servo_position_rad: f64,
        speed_mps: f64,
        written_at: std::time::Instant,
    ) -> Stamped<VescCommand> {
        Stamped {
            value: VescCommand::new(servo_position_rad, speed_mps),
            meta: WriteMeta {
                write_count: 1,
                written_at: Some(written_at),
                written_at_unix_us: 1,
            },
        }
    }

    #[test]
    fn select_command_lets_an_active_human_override_the_autonomous_command() {
        let now = std::time::Instant::now();
        let human = written(0.0, 1.0, now);
        let autonomous = written(-0.4, 4.0, now + std::time::Duration::from_millis(1));
        assert_eq!(
            select_command(autonomous, human),
            VescCommand::new(0.0, 1.0)
        );
    }

    #[test]
    fn select_command_ignores_an_idle_human_heartbeat() {
        let now = std::time::Instant::now();
        let human = written(0.0, 0.0, now + std::time::Duration::from_millis(1));
        let autonomous = written(-0.4, 4.0, now);
        assert_eq!(
            select_command(autonomous, human),
            VescCommand::new(-0.4, 4.0)
        );
    }

    #[test]
    fn select_command_stops_on_stale_or_unwritten_commands() {
        let stale_at =
            std::time::Instant::now() - VESC_COMMAND_TIMEOUT - std::time::Duration::from_millis(10);
        let seed = Stamped {
            value: VescCommand::new(0.3, 5.0),
            meta: WriteMeta::default(),
        };
        assert_eq!(
            select_command(written(-0.4, 4.0, stale_at), written(0.1, 1.0, stale_at)),
            VescCommand::default()
        );
        assert_eq!(select_command(seed.clone(), seed), VescCommand::default());
    }

    #[test]
    fn opponent_command_scales_only_the_speed_of_a_fresh_command() {
        let now = std::time::Instant::now();
        assert_eq!(
            opponent_command(written(-0.4, 4.0, now), 0.5),
            VescCommand::new(-0.4, 2.0)
        );
        let stale_at = now - VESC_COMMAND_TIMEOUT - std::time::Duration::from_millis(10);
        assert_eq!(
            opponent_command(written(-0.4, 4.0, stale_at), 0.5),
            VescCommand::default()
        );
    }

    #[test]
    fn an_opponent_copies_the_ego_model_with_its_own_limits() {
        let mut ego_config = SimulatedVehicleConfig::default();
        ego_config.dynamic_bicycle.mass_kg += 1.0;
        let ego = model_status(VehicleModelKind::DynamicBicycle, &ego_config);
        let limits = ActuatorLimits {
            max_speed_mps: 1.5,
            ..ego_config.limits
        };

        let (model, config) = opponent_model(SimulatedVehicleConfig::default(), &ego, limits);

        assert_eq!(kind_of(&model), VehicleModelKind::DynamicBicycle);
        assert_eq!(config.dynamic_bicycle, ego_config.dynamic_bicycle);
        assert_eq!(config.limits, limits);
        assert_eq!(limits_of(&model), limits);
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
