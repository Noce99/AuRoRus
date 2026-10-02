//! [`SimulatedVehicle`]: runs a vehicle model forward in time under
//! [`AUTONOMOUS_VESC_COMMAND_TOPIC_NAME`], overridden by
//! [`JOYSTICK_VESC_COMMAND_TOPIC_NAME`] or [`HUMAN_VESC_COMMAND_TOPIC_NAME`]
//! whenever a human is driving (see [`select_command`]), publishing the result on
//! its [`VehicleTopics::vehicle_status`] and its actuator limits on its
//! [`VehicleTopics::vehicle_limits`]. Also watches
//! [`VEHICLE_MODEL_SELECTION_TOPIC_NAME`] for a live model switch (e.g. from
//! `web_gui`), publishing the currently running model on
//! [`VEHICLE_MODEL_STATUS_TOPIC_NAME`] (along with its live-tunable
//! parameters, applied from [`VEHICLE_MODEL_PARAMETERS_TOPIC_NAME`]), and
//! draws the vehicle on its own drawing topic (see [`crate::topics::Drawing`]).

mod model;
mod parameters;

use crate::actuators::command::{is_fresh, select_command};
use crate::topics::{
    AUTONOMOUS_VESC_COMMAND_TOPIC_NAME, ActuatorStatus, Color, Drawing, DrawingExt,
    HUMAN_VESC_COMMAND_TOPIC_NAME, JOYSTICK_VESC_COMMAND_TOPIC_NAME, Placement, PlacementTopics,
    Shape, VEHICLE_MODEL_PARAMETERS_TOPIC_NAME, VEHICLE_MODEL_SELECTION_TOPIC_NAME,
    VEHICLE_MODEL_STATUS_TOPIC_NAME, VehicleGeometry, VehicleModelParameters,
    VehicleModelSelection, VehicleModelStatus, VehicleStatus, VehicleTopics, VescCommand, now_ms,
};
use crate::{Captain, Executor, RwLockTopic, Stamped, Ticker};
use std::any::Any;
use std::collections::BTreeMap;
use std::sync::Arc;

pub use crate::topics::ActuatorLimits;
use model::*;
pub use model::{VehicleModel, VehicleState};
use parameters::*;
pub use parameters::{
    SimulatedVehicleConfig, config_path, default_model, save_limits, save_parameters, saved_limits,
    saved_values,
};

/// [`Drawing::z_index`] of the ego vehicle's drawing, and of an opponent's
/// (see [`SimulatedVehicle::opponent`]) - underneath it.
const VEHICLE_Z_INDEX: i32 = 10;
const OPPONENT_Z_INDEX: i32 = 9;

/// What [`SimulatedVehicle`] publishes on its drawing topic every tick: the
/// vehicle at `state`, front wheels turned by `steering_angle_rad`, in `color`.
fn drawing(
    model: &VehicleModel,
    geometry: &VehicleGeometry,
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
                length_m: geometry.body_length_m,
                width_m: geometry.body_width_m,
                front_axle_m,
                rear_axle_m,
                color,
            }],
            true,
        )
        .stale_after(Drawing::DEFAULT_STALE_AFTER)
        .z_index(VEHICLE_Z_INDEX)
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
    /// `None` when no [`crate::sensors::Joystick`] runs.
    joystick: Option<Arc<RwLockTopic<VescCommand>>>,
    model_selection: Arc<RwLockTopic<VehicleModelSelection>>,
    model_status: Arc<RwLockTopic<VehicleModelStatus>>,
    /// Nothing may publish parameter requests at all (e.g. a binary without
    /// `web_gui`) - then the config stays as loaded.
    parameters: Option<Arc<RwLockTopic<VehicleModelParameters>>>,
}

impl EgoTopics {
    /// The human commands, the joystick's first - see [`select_command`].
    fn human_commands(&self) -> impl Iterator<Item = Stamped<VescCommand>> {
        let joystick = self.joystick.as_ref().map(|topic| topic.read());
        joystick.into_iter().chain([self.human.read()])
    }
}

/// Runs `model` forward in time at [`SimulatedVehicleConfig::tick_rate_hz`], reading
/// [`AUTONOMOUS_VESC_COMMAND_TOPIC_NAME`] and the human commands each tick
/// (see [`select_command`]) and publishing the resulting [`VehicleStatus`]. Starts at whatever
/// [`START_STATE_TOPIC_NAME`] holds at that moment (the world origin,
/// stationary, if [`crate::environment::MapServer`] hasn't published one yet),
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
/// that model - for an opponent (see [`SimulatedVehicle::opponent`]). But the
/// steering angle, which stays `file_config`'s: it's the car's, not a limit an
/// opponent picks.
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
    file_config.limits = ActuatorLimits {
        max_steering_angle_rad: file_config.limits.max_steering_angle_rad,
        ..limits
    };
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
        captain.claim_writer::<ActuatorStatus>(
            &self.vehicle.actuator_status(),
            self.id,
            ActuatorStatus::default,
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
        let geometry = self.config.geometry;
        captain.claim_writer::<VehicleGeometry>(
            &self.vehicle.vehicle_geometry(),
            self.id,
            move || geometry,
        );
        captain.claim_drawing(self.id);
    }

    fn run(&mut self, captain: &Captain) {
        let status_topic = captain.topic::<VehicleStatus>(&self.vehicle.vehicle_status());
        let actuator_topic = captain.topic::<ActuatorStatus>(&self.vehicle.actuator_status());
        let autonomous_topic = captain.topic::<VescCommand>(
            self.opponent
                .as_ref()
                .map_or(AUTONOMOUS_VESC_COMMAND_TOPIC_NAME, |opponent| {
                    &opponent.command_topic
                }),
        );
        let ego = self.opponent.is_none().then(|| EgoTopics {
            human: captain.topic::<VescCommand>(HUMAN_VESC_COMMAND_TOPIC_NAME),
            joystick: captain.try_topic::<VescCommand>(JOYSTICK_VESC_COMMAND_TOPIC_NAME),
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
                    &ActuatorLimits::tunable_parameters_but_steering_angle(),
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
                    Some(ego) => select_command(autonomous_topic.read(), ego.human_commands()),
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
            actuator_topic
                .write(
                    self.id,
                    ActuatorStatus {
                        steering_rad: steering_angle_rad,
                        speed_mps: state.speed_mps(),
                    },
                )
                .expect("lost writer authorization for the actuator_status topic");
            drawing_topic
                .write(
                    self.id,
                    drawing(
                        &self.model,
                        &config.geometry,
                        &state,
                        steering_angle_rad,
                        color,
                    )
                    .z_index(z_index),
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
    use crate::topics::{VESC_COMMAND_TIMEOUT, VehicleModelKind};

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
        ego_config.dynamic_bicycle.yaw_inertia_kgm2 += 0.01;
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
    fn an_opponent_steers_as_far_as_the_car_whatever_its_limits_say() {
        let ego_config = SimulatedVehicleConfig::default();
        let ego = model_status(VehicleModelKind::Bicycle, &ego_config);
        // What the web GUI's form sends: every limit but the steering angle.
        let limits = ActuatorLimits {
            max_steering_angle_rad: 0.0,
            max_speed_mps: 1.5,
            ..ego_config.limits
        };

        let (model, config) = opponent_model(ego_config.clone(), &ego, limits);

        let expected = ActuatorLimits {
            max_speed_mps: 1.5,
            ..ego_config.limits
        };
        assert_eq!(config.limits, expected);
        assert_eq!(limits_of(&model), expected);
    }
}
