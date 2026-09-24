//! [`SimulatedVehicle`]: runs a vehicle model forward in time under
//! [`AUTONOMOUS_VESC_COMMAND_TOPIC_NAME`], overridden by
//! [`HUMAN_VESC_COMMAND_TOPIC_NAME`] whenever a human is driving (see
//! [`select_command`]), publishing the result on
//! [`VEHICLE_STATUS_TOPIC_NAME`] and its actuator limits on
//! [`VEHICLE_LIMITS_TOPIC_NAME`]. Also watches
//! [`VEHICLE_MODEL_SELECTION_TOPIC_NAME`] for a live model switch (e.g. from
//! `web_gui`), publishing the currently running model on
//! [`VEHICLE_MODEL_STATUS_TOPIC_NAME`], and draws the vehicle on its own
//! drawing topic (see [`crate::topics::Drawing`]).

use crate::environment::simulator::vehicle::{
    BicycleParams, BicycleState, DynamicParams, DynamicState, NonlinearBicycleState, NonlinearTireParams,
    PacejkaBicycleState, PacejkaTireParams, TwoTrackParams, TwoTrackState, dynamic_step, nonlinear_step,
    pacejka_step, step as bicycle_step, two_track_step,
};
pub use crate::topics::ActuatorLimits;
use crate::topics::{
    AUTONOMOUS_VESC_COMMAND_TOPIC_NAME, Color, Drawing, HUMAN_VESC_COMMAND_TOPIC_NAME, Shape, PLACE_AT_START_TOPIC_NAME,
    PlaceAtStart, START_STATE_TOPIC_NAME, StartState, VEHICLE_LIMITS_TOPIC_NAME, VEHICLE_MODEL_SELECTION_TOPIC_NAME,
    VEHICLE_MODEL_STATUS_TOPIC_NAME, VEHICLE_STATUS_TOPIC_NAME, VESC_COMMAND_TIMEOUT, VehicleModelKind,
    VehicleModelSelection, VehicleModelStatus, VehicleStatus, VescCommand,
};
use crate::{Captain, Executor, Stamped, Ticker};
use std::any::Any;

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
        toml::from_str(include_str!("../../config/actuators/simulated_vehicle.toml"))
            .expect("config/actuators/simulated_vehicle.toml must deserialize into SimulatedVehicleConfig")
    }
}

/// The default [`VehicleModel`] for `kind`, built from `config`.
pub fn default_model(kind: VehicleModelKind, config: &SimulatedVehicleConfig) -> VehicleModel {
    match kind {
        VehicleModelKind::Bicycle => VehicleModel::Bicycle { params: config.bicycle, limits: config.limits },
        VehicleModelKind::DynamicBicycle => {
            VehicleModel::DynamicBicycle { params: config.dynamic_bicycle, limits: config.limits }
        }
        VehicleModelKind::NonlinearBicycle => {
            VehicleModel::NonlinearBicycle { params: config.nonlinear_bicycle, limits: config.limits }
        }
        VehicleModelKind::PacejkaBicycle => {
            VehicleModel::PacejkaBicycle { params: config.pacejka_bicycle, limits: config.limits }
        }
        VehicleModelKind::TwoTrack => VehicleModel::TwoTrack { params: config.two_track, limits: config.limits },
    }
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
    let speed_error_mps = target_speed_mps - state.longitudinal_speed_mps();
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

/// Body size [`SimulatedVehicle`] draws the vehicle at - roughly a 1/10-scale
/// RC car, matching the models' default geometry.
const DRAWN_BODY_LENGTH_M: f64 = 0.45;
const DRAWN_BODY_WIDTH_M: f64 = 0.25;

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
/// vehicle at `state`, front wheels turned by `steering_angle_rad`.
fn drawing(model: &VehicleModel, state: &VehicleState, steering_angle_rad: f64) -> Drawing {
    let (front_axle_m, rear_axle_m) = axles_of(model);
    Drawing::new(vec![Shape::Vehicle {
        x_m: state.x_m(),
        y_m: state.y_m(),
        heading_rad: state.heading_rad(),
        // Signed, unlike `VehicleStatus::speed_mps`, so a viewer dead-reckoning
        // the vehicle between samples moves it backward while reversing.
        speed_mps: state.longitudinal_speed_mps(),
        steering_rad: steering_angle_rad,
        length_m: DRAWN_BODY_LENGTH_M,
        width_m: DRAWN_BODY_WIDTH_M,
        front_axle_m,
        rear_axle_m,
        color: Color::AMBER,
    }])
    .stale_after(Drawing::DEFAULT_STALE_AFTER)
    .z_index(10)
}

/// Whether `command` was written, and recently enough to act on - see
/// [`VESC_COMMAND_TIMEOUT`].
fn is_fresh(command: &Stamped<VescCommand>) -> bool {
    command.age().is_some_and(|age| age <= VESC_COMMAND_TIMEOUT)
}

/// Picks the command to act on this tick. The human one always overrides:
/// it wins whenever it's fresh and asks for anything at all - `web_gui`
/// re-sends a stationary, centered command every few hundred milliseconds
/// while no control is held, so "fresh" alone can't mean "the human is
/// driving". Otherwise the autonomous one is used if fresh, and a stationary,
/// centered command if neither is - so a writer that stopped publishing
/// never leaves its last setpoint latched.
fn select_command(autonomous: Stamped<VescCommand>, human: Stamped<VescCommand>) -> VescCommand {
    if is_fresh(&human) && human.value != VescCommand::default() {
        human.value
    } else if is_fresh(&autonomous) {
        autonomous.value
    } else {
        VescCommand::default()
    }
}

/// Runs `model` forward in time at [`SimulatedVehicleConfig::tick_rate_hz`], reading
/// [`AUTONOMOUS_VESC_COMMAND_TOPIC_NAME`]/[`HUMAN_VESC_COMMAND_TOPIC_NAME`] each tick
/// (see [`select_command`]) and publishing the resulting [`VehicleStatus`]. Starts at whatever
/// [`START_STATE_TOPIC_NAME`] holds at that moment (the world origin,
/// stationary, if [`crate::sensors::MapServer`] hasn't published one yet),
/// with the steering centered, and places the vehicle there again - steering
/// re-centered - every time [`START_STATE_TOPIC_NAME`] changes (e.g. a map
/// change) or [`PLACE_AT_START_TOPIC_NAME`] is bumped (e.g. `web_gui`'s "P"
/// key). Also watches [`VEHICLE_MODEL_SELECTION_TOPIC_NAME`] each tick and
/// switches to [`default_model`] of the wanted kind - carrying over shared
/// state (see [`carry_over_state`]) - whenever it no longer matches the
/// model currently running.
pub struct SimulatedVehicle {
    id: u8,
    name: String,
    model: VehicleModel,
    config: SimulatedVehicleConfig,
}

impl SimulatedVehicle {
    /// Creates a `SimulatedVehicle` that will run `model` once started,
    /// ticking and switching models per `config`.
    pub fn new(name: impl Into<String>, model: VehicleModel, config: SimulatedVehicleConfig) -> Self {
        Self { id: 0, name: name.into(), model, config }
    }
}

impl Executor for SimulatedVehicle {
    fn init(&mut self, id: u8) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_writer::<VehicleStatus>(VEHICLE_STATUS_TOPIC_NAME, self.id, VehicleStatus::default);
        captain.claim_writer::<VehicleModelStatus>(VEHICLE_MODEL_STATUS_TOPIC_NAME, self.id, VehicleModelStatus::default);
        // Every model kind shares `config.limits`, so this never changes
        // after being seeded - nothing needs to write it again.
        let limits = self.config.limits;
        captain.claim_writer::<ActuatorLimits>(VEHICLE_LIMITS_TOPIC_NAME, self.id, move || limits);
        captain.claim_drawing(self.id);
    }

    fn run(&mut self, captain: &Captain) {
        let status_topic = captain.topic::<VehicleStatus>(VEHICLE_STATUS_TOPIC_NAME);
        let autonomous_topic = captain.topic::<VescCommand>(AUTONOMOUS_VESC_COMMAND_TOPIC_NAME);
        let human_topic = captain.topic::<VescCommand>(HUMAN_VESC_COMMAND_TOPIC_NAME);
        let model_selection_topic = captain.topic::<VehicleModelSelection>(VEHICLE_MODEL_SELECTION_TOPIC_NAME);
        let model_status_topic = captain.topic::<VehicleModelStatus>(VEHICLE_MODEL_STATUS_TOPIC_NAME);
        let start_state_topic = captain.topic::<StartState>(START_STATE_TOPIC_NAME);
        let place_at_start_topic = captain.topic::<PlaceAtStart>(PLACE_AT_START_TOPIC_NAME);
        let drawing_topic = captain.drawing(self.id);

        let mut applied_kind = kind_of(&self.model);
        model_status_topic
            .write(self.id, VehicleModelStatus { kind: applied_kind })
            .expect("lost writer authorization for the vehicle_model_status topic");

        let mut applied_start = start_state_topic.read().into_value();
        let mut applied_place_request = place_at_start_topic.read().requested;
        let mut state = state_from_start(applied_start, applied_kind);
        let mut steering_angle_rad = 0.0;
        // The model integrates a fixed `dt_s` per tick, so the loop has to
        // actually run at `tick_rate_hz` for simulated time to track real
        // time - which is what `Ticker` (unlike a fixed sleep) guarantees.
        // Keeping `dt_s` nominal rather than measuring each period keeps the
        // physics deterministic and reproducible.
        let dt_s = 1.0 / self.config.tick_rate_hz;
        let mut ticker = Ticker::new(self.config.tick_rate_hz);

        while captain.is_running(self.id) {
            let wanted_kind = model_selection_topic.read().kind;
            if wanted_kind != applied_kind {
                self.model = default_model(wanted_kind, &self.config);
                state = carry_over_state(state, wanted_kind);
                applied_kind = wanted_kind;
                model_status_topic
                    .write(self.id, VehicleModelStatus { kind: applied_kind })
                    .expect("lost writer authorization for the vehicle_model_status topic");
            }

            let wanted_start = start_state_topic.read().into_value();
            let wanted_place_request = place_at_start_topic.read().requested;
            if wanted_start != applied_start || wanted_place_request != applied_place_request {
                state = state_from_start(wanted_start, applied_kind);
                steering_angle_rad = 0.0;
                applied_start = wanted_start;
                applied_place_request = wanted_place_request;
            }

            let command = select_command(autonomous_topic.read(), human_topic.read());

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
            drawing_topic
                .write(self.id, drawing(&self.model, &state, steering_angle_rad))
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
        // Rebuilds `model` via `default_model`, rather than cloning `self.model`,
        // so a restart resets the vehicle's simulated state (position, velocity,
        // ...) even if the model kind was switched mid-run via
        // `VEHICLE_MODEL_SELECTION_TOPIC_NAME` - only the *kind* carries over.
        let kind = kind_of(&self.model);
        Box::new(SimulatedVehicle::new(self.name.clone(), default_model(kind, &self.config), self.config.clone()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::WriteMeta;

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
    fn state_from_start_carries_the_start_state_into_every_model_kind() {
        let start = StartState { x_m: 3.0, y_m: -2.0, heading_rad: 0.5, speed_mps: 0.0 };
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

    fn written(servo_position_rad: f64, speed_mps: f64, written_at: std::time::Instant) -> Stamped<VescCommand> {
        Stamped {
            value: VescCommand::new(servo_position_rad, speed_mps),
            meta: WriteMeta { write_count: 1, written_at: Some(written_at), written_at_unix_us: 1 },
        }
    }

    #[test]
    fn select_command_lets_an_active_human_override_the_autonomous_command() {
        let now = std::time::Instant::now();
        let human = written(0.0, 1.0, now);
        let autonomous = written(-0.4, 4.0, now + std::time::Duration::from_millis(1));
        assert_eq!(select_command(autonomous, human), VescCommand::new(0.0, 1.0));
    }

    #[test]
    fn select_command_ignores_an_idle_human_heartbeat() {
        let now = std::time::Instant::now();
        let human = written(0.0, 0.0, now + std::time::Duration::from_millis(1));
        let autonomous = written(-0.4, 4.0, now);
        assert_eq!(select_command(autonomous, human), VescCommand::new(-0.4, 4.0));
    }

    #[test]
    fn select_command_stops_on_stale_or_unwritten_commands() {
        let stale_at = std::time::Instant::now() - VESC_COMMAND_TIMEOUT - std::time::Duration::from_millis(10);
        let seed = Stamped { value: VescCommand::new(0.3, 5.0), meta: WriteMeta::default() };
        assert_eq!(select_command(written(-0.4, 4.0, stale_at), written(0.1, 1.0, stale_at)), VescCommand::default());
        assert_eq!(select_command(seed.clone(), seed), VescCommand::default());
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
