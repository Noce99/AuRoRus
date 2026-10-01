//! The [`ActuatorStatus`] topic: what the vehicle's actuators are doing
//! right now - how far its front wheels are steered and how fast it's
//! going - published by whatever drives them
//! ([`crate::actuators::SimulatedVehicle`] in simulation,
//! [`crate::actuators::Vesc`] on the real car), so a reader (e.g.
//! `web_gui`'s readout) shows the same thing either way.

/// Name of the topic [`ActuatorStatus`] is published on.
pub const ACTUATOR_STATUS_TOPIC_NAME: &str = "actuator_status";

/// Where the vehicle's steering is and how fast it's going - as opposed to
/// [`super::VescCommand`], which is only where they're asked to be.
#[derive(Debug, Clone, Copy, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct ActuatorStatus {
    /// The front wheels' steering angle, in radians - positive right, like
    /// [`super::VescCommand::servo_position_rad`]. On the real car, the
    /// angle the servo position last sent steers, not a measured one.
    pub steering_rad: f64,
    /// Forward speed, in meters/second - negative reversing. On the real
    /// car, the wheels' (see [`super::VescStatus::wheel_speed_mps`]).
    pub speed_mps: f64,
}
