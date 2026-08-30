//! [`SimulatedVehicle`]: runs a vehicle model forward in time under
//! whichever of [`VESC_COMMAND_TOPIC_NAME`]/[`HUMAN_VESC_COMMAND_TOPIC_NAME`]
//! was published most recently, publishing the result on
//! [`VEHICLE_STATUS_TOPIC_NAME`]. Also watches
//! [`VEHICLE_MODEL_SELECTION_TOPIC_NAME`] for a live model switch (e.g. from
//! `web_gui`), publishing the currently running model on
//! [`VEHICLE_MODEL_STATUS_TOPIC_NAME`].

use crate::environment::simulator::vehicle::{
    BicycleParams, BicycleState, DynamicParams, DynamicState, NonlinearBicycleState, NonlinearTireParams,
    PacejkaBicycleState, PacejkaTireParams, TwoTrackParams, TwoTrackState, dynamic_step, nonlinear_step,
    pacejka_step, step as bicycle_step, two_track_step,
};
use crate::topics::{
    HUMAN_VESC_COMMAND_TOPIC_NAME, VEHICLE_MODEL_SELECTION_TOPIC_NAME, VEHICLE_MODEL_STATUS_TOPIC_NAME,
    VESC_COMMAND_TOPIC_NAME, VEHICLE_STATUS_TOPIC_NAME, VehicleModelKind, VehicleModelSelection, VehicleModelStatus,
    VehicleStatus, VescCommand,
};
use crate::{Captain, Executor};
use std::any::Any;
use std::thread;
use std::time::Duration;

/// How often [`SimulatedVehicle`] advances the model and republishes
/// [`VehicleStatus`], and how often it checks
/// [`VEHICLE_MODEL_SELECTION_TOPIC_NAME`] for a wanted model switch.
const TICK_RATE_HZ: f64 = 100.0;

/// The physical limits a [`VehicleModel`]'s simulated actuators can't
/// exceed, no matter how far the current state is from the desired
/// setpoint - [`SimulatedVehicle`] approaches the setpoint as fast as these
/// allow, every tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ActuatorLimits {
    /// Largest front-wheel steering angle the servo can hold, in either
    /// direction, in radians.
    pub max_steering_angle_rad: f64,
    /// Fastest the steering angle can change, in radians/second.
    pub max_steering_rate_rad_s: f64,
    /// Largest speed the vehicle can be commanded to, in either direction
    /// (i.e. this also bounds reverse), in meters/second.
    pub max_speed_mps: f64,
    /// Largest forward acceleration the motor can produce, in
    /// meters/second^2.
    pub max_accel_mps2: f64,
    /// Largest deceleration (braking) the motor can produce, in
    /// meters/second^2.
    pub max_decel_mps2: f64,
}

impl ActuatorLimits {
    /// Basic sanity checks on the limit values.
    pub fn validate(&self) -> Result<(), String> {
        if self.max_steering_angle_rad <= 0.0 {
            return Err("max_steering_angle_rad must be positive".to_string());
        }
        if self.max_steering_rate_rad_s <= 0.0 {
            return Err("max_steering_rate_rad_s must be positive".to_string());
        }
        if self.max_speed_mps <= 0.0 {
            return Err("max_speed_mps must be positive".to_string());
        }
        if self.max_accel_mps2 <= 0.0 {
            return Err("max_accel_mps2 must be positive".to_string());
        }
        if self.max_decel_mps2 <= 0.0 {
            return Err("max_decel_mps2 must be positive".to_string());
        }
        Ok(())
    }
}

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
    /// and lateral velocity separately.
    fn speed_mps(&self) -> f64 {
        match self {
            Self::Bicycle(s) => s.speed_mps,
            Self::DynamicBicycle(s) => s.vx_mps.hypot(s.vy_mps),
            Self::NonlinearBicycle(s) => s.vx_mps.hypot(s.vy_mps),
            Self::PacejkaBicycle(s) => s.vx_mps.hypot(s.vy_mps),
            Self::TwoTrack(s) => s.vx_mps.hypot(s.vy_mps),
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

/// The default [`VehicleModel`] for `kind`: RC-car-scale geometry and
/// actuator limits for a small (roughly 1/10-scale) RC racecar, matching the
/// kind of track `generate_map` produces. `DynamicBicycle`'s mass/inertia/
/// cornering-stiffness values are placeholder estimates for that same scale
/// of vehicle, not measured - tune them once a real (or more carefully
/// modeled) vehicle is available. This is a deliberate exception to
/// [`DynamicParams`]/[`BicycleParams`] having no [`Default`]: a live,
/// web-selectable model needs *some* starting parameters for a kind the
/// caller only names, not configures.
pub fn default_model(kind: VehicleModelKind) -> VehicleModel {
    let limits = ActuatorLimits {
        max_steering_angle_rad: 0.4189, // 24 degrees
        max_steering_rate_rad_s: 4.0,
        max_speed_mps: 8.0,
        max_accel_mps2: 4.0,
        max_decel_mps2: 8.0,
    };
    match kind {
        VehicleModelKind::Bicycle => {
            VehicleModel::Bicycle { params: BicycleParams { lf_m: 0.16, lr_m: 0.16 }, limits }
        }
        VehicleModelKind::DynamicBicycle => VehicleModel::DynamicBicycle {
            params: DynamicParams {
                mass_kg: 3.5,
                yaw_inertia_kgm2: 0.06,
                lf_m: 0.16,
                lr_m: 0.16,
                cf_n_per_rad: 60.0,
                cr_n_per_rad: 60.0,
            },
            limits,
        },
        VehicleModelKind::NonlinearBicycle => VehicleModel::NonlinearBicycle {
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
            limits,
        },
        VehicleModelKind::PacejkaBicycle => VehicleModel::PacejkaBicycle {
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
            limits,
        },
        VehicleModelKind::TwoTrack => VehicleModel::TwoTrack {
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
            limits,
        },
    }
}

/// The origin, stationary [`VehicleState`] for `kind` - used for the very
/// first state [`SimulatedVehicle::run`] advances.
fn default_state(kind: VehicleModelKind) -> VehicleState {
    match kind {
        VehicleModelKind::Bicycle => {
            VehicleState::Bicycle(BicycleState { x_m: 0.0, y_m: 0.0, heading_rad: 0.0, speed_mps: 0.0 })
        }
        VehicleModelKind::DynamicBicycle => VehicleState::DynamicBicycle(DynamicState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            vx_mps: 0.0,
            vy_mps: 0.0,
            yaw_rate_rad_s: 0.0,
        }),
        VehicleModelKind::NonlinearBicycle => VehicleState::NonlinearBicycle(NonlinearBicycleState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            vx_mps: 0.0,
            vy_mps: 0.0,
            yaw_rate_rad_s: 0.0,
        }),
        VehicleModelKind::PacejkaBicycle => VehicleState::PacejkaBicycle(PacejkaBicycleState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            vx_mps: 0.0,
            vy_mps: 0.0,
            yaw_rate_rad_s: 0.0,
        }),
        VehicleModelKind::TwoTrack => VehicleState::TwoTrack(TwoTrackState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            vx_mps: 0.0,
            vy_mps: 0.0,
            yaw_rate_rad_s: 0.0,
        }),
    }
}

/// Projects `old`'s shared fields (position, heading, speed) into a fresh
/// [`VehicleState`] of `new_kind`, zeroing any fields `new_kind` has that
/// `old` didn't - used when [`SimulatedVehicle::run`] switches models at
/// runtime, so the vehicle doesn't visibly jump when the model underneath it
/// changes mid-drive.
fn carry_over_state(old: VehicleState, new_kind: VehicleModelKind) -> VehicleState {
    let (x_m, y_m, heading_rad, speed_mps) = (old.x_m(), old.y_m(), old.heading_rad(), old.speed_mps());
    match new_kind {
        VehicleModelKind::Bicycle => VehicleState::Bicycle(BicycleState { x_m, y_m, heading_rad, speed_mps }),
        VehicleModelKind::DynamicBicycle => VehicleState::DynamicBicycle(DynamicState {
            x_m,
            y_m,
            heading_rad,
            vx_mps: speed_mps,
            vy_mps: 0.0,
            yaw_rate_rad_s: 0.0,
        }),
        VehicleModelKind::NonlinearBicycle => VehicleState::NonlinearBicycle(NonlinearBicycleState {
            x_m,
            y_m,
            heading_rad,
            vx_mps: speed_mps,
            vy_mps: 0.0,
            yaw_rate_rad_s: 0.0,
        }),
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

    let target_steering_rad = target_steering_rad.clamp(-limits.max_steering_angle_rad, limits.max_steering_angle_rad);
    let max_steering_delta = limits.max_steering_rate_rad_s * dt_s;
    let next_steering_rad = (steering_angle_rad + (target_steering_rad - steering_angle_rad).clamp(-max_steering_delta, max_steering_delta))
        .clamp(-limits.max_steering_angle_rad, limits.max_steering_angle_rad);

    let target_speed_mps = target_speed_mps.clamp(-limits.max_speed_mps, limits.max_speed_mps);
    let speed_error_mps = target_speed_mps - state.speed_mps();
    let accel_mps2 = if speed_error_mps >= 0.0 {
        (speed_error_mps / dt_s).min(limits.max_accel_mps2)
    } else {
        (speed_error_mps / dt_s).max(-limits.max_decel_mps2)
    };

    let next_state = match (model, state) {
        (VehicleModel::Bicycle { params, .. }, VehicleState::Bicycle(s)) => {
            VehicleState::Bicycle(bicycle_step(s, *params, next_steering_rad, accel_mps2, dt_s))
        }
        (VehicleModel::DynamicBicycle { params, .. }, VehicleState::DynamicBicycle(s)) => {
            VehicleState::DynamicBicycle(dynamic_step(s, *params, next_steering_rad, accel_mps2, dt_s))
        }
        (VehicleModel::NonlinearBicycle { params, .. }, VehicleState::NonlinearBicycle(s)) => {
            VehicleState::NonlinearBicycle(nonlinear_step(s, *params, next_steering_rad, accel_mps2, dt_s))
        }
        (VehicleModel::PacejkaBicycle { params, .. }, VehicleState::PacejkaBicycle(s)) => {
            VehicleState::PacejkaBicycle(pacejka_step(s, *params, next_steering_rad, accel_mps2, dt_s))
        }
        (VehicleModel::TwoTrack { params, .. }, VehicleState::TwoTrack(s)) => {
            VehicleState::TwoTrack(two_track_step(s, *params, next_steering_rad, accel_mps2, dt_s))
        }
        _ => unreachable!("SimulatedVehicle::run always keeps model/state kinds in sync"),
    };
    (next_state, next_steering_rad)
}

/// Picks whichever of an autonomous and a human command was published more
/// recently - see [`VescCommand::time_stamp_us`]. With no autonomous
/// controller implemented yet, `autonomous` is whatever
/// [`VESC_COMMAND_TOPIC_NAME`] was pre-seeded with, so this always resolves
/// to `human` in practice today.
fn select_command(autonomous: VescCommand, human: VescCommand) -> VescCommand {
    if human.time_stamp_us >= autonomous.time_stamp_us { human } else { autonomous }
}

/// Runs `model` forward in time at [`TICK_RATE_HZ`], reading the freshest of
/// [`VESC_COMMAND_TOPIC_NAME`]/[`HUMAN_VESC_COMMAND_TOPIC_NAME`] each tick
/// and publishing the resulting [`VehicleStatus`]. Starts at the world
/// origin, stationary, with the steering centered. Also watches
/// [`VEHICLE_MODEL_SELECTION_TOPIC_NAME`] each tick and switches to
/// [`default_model`] of the wanted kind - carrying over shared state (see
/// [`carry_over_state`]) - whenever it no longer matches the model currently
/// running.
pub struct SimulatedVehicle {
    id: u8,
    name: String,
    model: VehicleModel,
}

impl SimulatedVehicle {
    /// Creates a `SimulatedVehicle` that will run `model` once started.
    pub fn new(name: impl Into<String>, model: VehicleModel) -> Self {
        Self { id: 0, name: name.into(), model }
    }
}

impl Executor for SimulatedVehicle {
    fn init(&mut self, id: u8) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_writer::<VehicleStatus>(VEHICLE_STATUS_TOPIC_NAME, self.id, VehicleStatus::default);
        captain.claim_writer::<VehicleModelStatus>(VEHICLE_MODEL_STATUS_TOPIC_NAME, self.id, VehicleModelStatus::default);
    }

    fn run(&mut self, captain: &Captain) {
        let status_topic = captain.topic::<VehicleStatus>(VEHICLE_STATUS_TOPIC_NAME);
        let vesc_topic = captain.topic::<VescCommand>(VESC_COMMAND_TOPIC_NAME);
        let human_topic = captain.topic::<VescCommand>(HUMAN_VESC_COMMAND_TOPIC_NAME);
        let model_selection_topic = captain.topic::<VehicleModelSelection>(VEHICLE_MODEL_SELECTION_TOPIC_NAME);
        let model_status_topic = captain.topic::<VehicleModelStatus>(VEHICLE_MODEL_STATUS_TOPIC_NAME);

        let mut applied_kind = kind_of(&self.model);
        model_status_topic
            .write(self.id, VehicleModelStatus { kind: applied_kind })
            .expect("lost writer authorization for the vehicle_model_status topic");

        let mut state = default_state(applied_kind);
        let mut steering_angle_rad = 0.0;
        let dt_s = 1.0 / TICK_RATE_HZ;
        let interval = Duration::from_secs_f64(dt_s);

        while captain.is_running(self.id) {
            let wanted_kind = model_selection_topic.read().kind;
            if wanted_kind != applied_kind {
                self.model = default_model(wanted_kind);
                state = carry_over_state(state, wanted_kind);
                applied_kind = wanted_kind;
                model_status_topic
                    .write(self.id, VehicleModelStatus { kind: applied_kind })
                    .expect("lost writer authorization for the vehicle_model_status topic");
            }

            let command = select_command(vesc_topic.read(), human_topic.read());

            let (next_state, next_steering_rad) =
                advance(&self.model, state, steering_angle_rad, command.servo_position_rad, command.speed_mps, dt_s);
            state = next_state;
            steering_angle_rad = next_steering_rad;

            status_topic
                .write(
                    self.id,
                    VehicleStatus {
                        x_m: state.x_m(),
                        y_m: state.y_m(),
                        heading_rad: state.heading_rad(),
                        speed_mps: state.speed_mps(),
                    },
                )
                .expect("lost writer authorization for the vehicle_status topic");

            thread::sleep(interval);
        }
    }

    fn name(&self) -> String {
        self.name.clone()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_model() -> VehicleModel {
        VehicleModel::Bicycle {
            params: BicycleParams { lf_m: 0.16, lr_m: 0.16 },
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
        assert!(matches!(test_model(), VehicleModel::Bicycle { limits, .. } if limits.validate().is_ok()));
    }

    #[test]
    fn default_model_validates_for_every_kind() {
        for kind in [
            VehicleModelKind::Bicycle,
            VehicleModelKind::DynamicBicycle,
            VehicleModelKind::NonlinearBicycle,
            VehicleModelKind::PacejkaBicycle,
            VehicleModelKind::TwoTrack,
        ] {
            match default_model(kind) {
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

    #[test]
    fn kind_of_matches_the_variant() {
        assert_eq!(kind_of(&test_model()), VehicleModelKind::Bicycle);
        assert_eq!(kind_of(&test_dynamic_model()), VehicleModelKind::DynamicBicycle);
        assert_eq!(kind_of(&test_nonlinear_model()), VehicleModelKind::NonlinearBicycle);
        assert_eq!(kind_of(&test_pacejka_model()), VehicleModelKind::PacejkaBicycle);
        assert_eq!(kind_of(&test_two_track_model()), VehicleModelKind::TwoTrack);
    }

    #[test]
    fn carry_over_state_maps_speed_into_the_new_models_shared_fields() {
        let old = VehicleState::Bicycle(BicycleState { x_m: 1.0, y_m: 2.0, heading_rad: 0.3, speed_mps: 4.0 });
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
    fn select_command_prefers_the_fresher_timestamp() {
        let older = VescCommand { time_stamp_us: 1, servo_position_rad: 0.0, speed_mps: 1.0 };
        let newer = VescCommand { time_stamp_us: 2, servo_position_rad: 0.0, speed_mps: 2.0 };
        assert_eq!(select_command(older, newer).speed_mps, 2.0);
        assert_eq!(select_command(newer, older).speed_mps, 2.0);
    }

    #[test]
    fn advance_never_exceeds_the_steering_rate_limit_in_one_tick() {
        let model = test_model();
        let state = VehicleState::Bicycle(BicycleState { x_m: 0.0, y_m: 0.0, heading_rad: 0.0, speed_mps: 0.0 });
        let dt_s = 0.01;
        let (_, next_steering) = advance(&model, state, 0.0, 10.0, 0.0, dt_s);
        assert!(next_steering <= 4.0 * dt_s + 1e-12);
    }

    #[test]
    fn advance_never_exceeds_the_max_steering_angle() {
        let model = test_model();
        let state = VehicleState::Bicycle(BicycleState { x_m: 0.0, y_m: 0.0, heading_rad: 0.0, speed_mps: 0.0 });
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
        let state = VehicleState::Bicycle(BicycleState { x_m: 0.0, y_m: 0.0, heading_rad: 0.0, speed_mps: 0.0 });
        let dt_s = 0.01;
        let (next_state, _) = advance(&model, state, 0.0, 0.0, 100.0, dt_s);
        assert!(next_state.speed_mps() <= 4.0 * dt_s + 1e-12);
    }

    #[test]
    fn advance_never_exceeds_max_speed_even_at_a_large_dt() {
        let model = test_model();
        let state = VehicleState::Bicycle(BicycleState { x_m: 0.0, y_m: 0.0, heading_rad: 0.0, speed_mps: 0.0 });
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
        let old = VehicleState::Bicycle(BicycleState { x_m: 1.0, y_m: 2.0, heading_rad: 0.3, speed_mps: 4.0 });
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
        let old = VehicleState::Bicycle(BicycleState { x_m: 1.0, y_m: 2.0, heading_rad: 0.3, speed_mps: 4.0 });
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
        let old = VehicleState::Bicycle(BicycleState { x_m: 1.0, y_m: 2.0, heading_rad: 0.3, speed_mps: 4.0 });
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
}
