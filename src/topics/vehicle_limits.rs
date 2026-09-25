//! The [`ActuatorLimits`] topic: the physical limits of the vehicle's
//! actuators, published by whatever drives them (e.g.
//! [`crate::actuators::SimulatedVehicle`]) so an autonomous algorithm can
//! command, say, full steering lock without hardcoding what that is. They
//! can be tuned live, so a reader should reread it rather than cache it.

use super::AlgorithmParameter;

/// Name of the topic [`ActuatorLimits`] is published on.
pub const VEHICLE_LIMITS_TOPIC_NAME: &str = "vehicle_limits";

/// The physical limits the vehicle's actuators can't exceed, no matter how
/// far the current state is from the desired setpoint -
/// [`crate::actuators::SimulatedVehicle`] approaches the setpoint as fast as
/// these allow, every tick, and publishes the ones it runs with on
/// [`VEHICLE_LIMITS_TOPIC_NAME`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
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
    /// The live-tunable limits, one per field - see
    /// [`crate::actuators::SimulatedVehicle`].
    pub fn tunable_parameters() -> Vec<AlgorithmParameter> {
        vec![
            AlgorithmParameter::float("max_steering_angle_rad", 0.05, 1.0, 0.01)
                .unit("rad")
                .description("Largest steering angle the servo can hold, either way."),
            AlgorithmParameter::float("max_steering_rate_rad_s", 0.5, 20.0, 0.1)
                .unit("rad/s")
                .description("Fastest the steering angle can change."),
            AlgorithmParameter::float("max_speed_mps", 0.5, 20.0, 0.1)
                .unit("m/s")
                .description("Largest speed the vehicle can be commanded to, reverse included."),
            AlgorithmParameter::float("max_accel_mps2", 0.5, 30.0, 0.1)
                .unit("m/s²")
                .description("Largest forward acceleration the motor can produce."),
            AlgorithmParameter::float("max_decel_mps2", 0.5, 30.0, 0.1)
                .unit("m/s²")
                .description("Largest deceleration (braking) the motor can produce."),
        ]
    }

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
