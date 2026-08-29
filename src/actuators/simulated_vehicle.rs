//! [`SimulatedVehicle`]: runs a vehicle model forward in time under
//! whichever of [`VESC_COMMAND_TOPIC_NAME`]/[`HUMAN_VESC_COMMAND_TOPIC_NAME`]
//! was published most recently, publishing the result on
//! [`VEHICLE_STATUS_TOPIC_NAME`].

use crate::environment::simulator::vehicle::{BicycleParams, BicycleState, step as bicycle_step};
use crate::topics::{
    HUMAN_VESC_COMMAND_TOPIC_NAME, VESC_COMMAND_TOPIC_NAME, VEHICLE_STATUS_TOPIC_NAME, VehicleStatus, VescCommand,
};
use crate::{Captain, Executor};
use std::any::Any;
use std::thread;
use std::time::Duration;

/// How often [`SimulatedVehicle`] advances the model and republishes
/// [`VehicleStatus`].
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
/// actuator limits it needs to do so. An enum (rather than a trait) because
/// exactly one model - a kinematic bicycle - exists today; see
/// `src/environment/simulator/vehicle/README.md` for why the same choice was
/// made there. A second model would most naturally add a second variant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VehicleModel {
    /// A CG-referenced kinematic bicycle model - see
    /// [`crate::environment::simulator::vehicle::bicycle`].
    Bicycle {
        params: BicycleParams,
        limits: ActuatorLimits,
    },
}

/// This model's [`ActuatorLimits`], regardless of which variant it is -
/// used by [`SimulatedVehicle::run`] without a `match` at every call site.
fn limits_of(model: &VehicleModel) -> ActuatorLimits {
    match model {
        VehicleModel::Bicycle { limits, .. } => *limits,
    }
}

/// Advances `state`/`steering_angle_rad` by one `dt_s` tick under `model`,
/// steering and accelerating toward `target_steering_rad`/`target_speed_mps`
/// as fast as the model's [`ActuatorLimits`] allow (never instantaneously,
/// and never past the limits) - not just integrating the raw setpoint, which
/// would let the simulated vehicle do things no real actuator could.
fn advance(
    model: &VehicleModel,
    state: BicycleState,
    steering_angle_rad: f64,
    target_steering_rad: f64,
    target_speed_mps: f64,
    dt_s: f64,
) -> (BicycleState, f64) {
    let limits = limits_of(model);

    let target_steering_rad = target_steering_rad.clamp(-limits.max_steering_angle_rad, limits.max_steering_angle_rad);
    let max_steering_delta = limits.max_steering_rate_rad_s * dt_s;
    let next_steering_rad = (steering_angle_rad + (target_steering_rad - steering_angle_rad).clamp(-max_steering_delta, max_steering_delta))
        .clamp(-limits.max_steering_angle_rad, limits.max_steering_angle_rad);

    let target_speed_mps = target_speed_mps.clamp(-limits.max_speed_mps, limits.max_speed_mps);
    let speed_error_mps = target_speed_mps - state.speed_mps;
    let accel_mps2 = if speed_error_mps >= 0.0 {
        (speed_error_mps / dt_s).min(limits.max_accel_mps2)
    } else {
        (speed_error_mps / dt_s).max(-limits.max_decel_mps2)
    };

    let next_state = match model {
        VehicleModel::Bicycle { params, .. } => bicycle_step(state, *params, next_steering_rad, accel_mps2, dt_s),
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
/// origin, stationary, with the steering centered.
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
    }

    fn run(&mut self, captain: &Captain) {
        let status_topic = captain.topic::<VehicleStatus>(VEHICLE_STATUS_TOPIC_NAME);
        let vesc_topic = captain.topic::<VescCommand>(VESC_COMMAND_TOPIC_NAME);
        let human_topic = captain.topic::<VescCommand>(HUMAN_VESC_COMMAND_TOPIC_NAME);

        let mut state = BicycleState { x_m: 0.0, y_m: 0.0, heading_rad: 0.0, speed_mps: 0.0 };
        let mut steering_angle_rad = 0.0;
        let dt_s = 1.0 / TICK_RATE_HZ;
        let interval = Duration::from_secs_f64(dt_s);

        while captain.is_running(self.id) {
            let command = select_command(vesc_topic.read(), human_topic.read());

            let (next_state, next_steering_rad) =
                advance(&self.model, state, steering_angle_rad, command.servo_position_rad, command.speed_mps, dt_s);
            state = next_state;
            steering_angle_rad = next_steering_rad;

            status_topic
                .write(
                    self.id,
                    VehicleStatus {
                        x_m: state.x_m,
                        y_m: state.y_m,
                        heading_rad: state.heading_rad,
                        speed_mps: state.speed_mps,
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

    #[test]
    fn default_limits_validate() {
        assert!(matches!(test_model(), VehicleModel::Bicycle { limits, .. } if limits.validate().is_ok()));
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
        let state = BicycleState { x_m: 0.0, y_m: 0.0, heading_rad: 0.0, speed_mps: 0.0 };
        let dt_s = 0.01;
        let (_, next_steering) = advance(&model, state, 0.0, 10.0, 0.0, dt_s);
        assert!(next_steering <= 4.0 * dt_s + 1e-12);
    }

    #[test]
    fn advance_never_exceeds_the_max_steering_angle() {
        let model = test_model();
        let state = BicycleState { x_m: 0.0, y_m: 0.0, heading_rad: 0.0, speed_mps: 0.0 };
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
        let state = BicycleState { x_m: 0.0, y_m: 0.0, heading_rad: 0.0, speed_mps: 0.0 };
        let dt_s = 0.01;
        let (next_state, _) = advance(&model, state, 0.0, 0.0, 100.0, dt_s);
        assert!(next_state.speed_mps <= 4.0 * dt_s + 1e-12);
    }

    #[test]
    fn advance_never_exceeds_max_speed_even_at_a_large_dt() {
        let model = test_model();
        let state = BicycleState { x_m: 0.0, y_m: 0.0, heading_rad: 0.0, speed_mps: 0.0 };
        let (next_state, _) = advance(&model, state, 0.0, 0.0, 100.0, 10.0);
        assert!(next_state.speed_mps <= 8.0 + 1e-9);
    }
}
