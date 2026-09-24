//! The [`VescCommand`] topic: a desired steering/speed setpoint for the
//! vehicle's actuators, published on two separate topics -
//! [`AUTONOMOUS_VESC_COMMAND_TOPIC_NAME`] (whichever autonomous algorithm is
//! currently selected, forwarded by
//! [`crate::autonomous_control::AutonomousControlsHandler`]) and
//! [`HUMAN_VESC_COMMAND_TOPIC_NAME`] (a human driver, e.g. `web_gui`'s WASD
//! control) - both sharing this same shape so any consumer reads them
//! identically. Every autonomous algorithm's own output (see
//! [`crate::topics::AUTONOMOUS_CONTROL_TOPIC_PREFIX`]) has this shape too.

use std::time::Duration;

/// Name of the topic [`crate::autonomous_control::AutonomousControlsHandler`]
/// publishes the selected autonomous algorithm's setpoint on - the only
/// autonomous command [`crate::actuators::SimulatedVehicle`] ever acts on. It
/// reads it alongside [`HUMAN_VESC_COMMAND_TOPIC_NAME`], and the human one
/// always overrides it while any control is held.
pub const AUTONOMOUS_VESC_COMMAND_TOPIC_NAME: &str = "autonomous_vesc_command";
/// Name of the topic a human driver's desired steering/speed setpoint is
/// published on, e.g. by `web_gui`'s WASD control.
pub const HUMAN_VESC_COMMAND_TOPIC_NAME: &str = "human_vesc_command";

/// How old a [`VescCommand`] may get before its consumer stops trusting it
/// and falls back to a stationary, centered command - so a writer that
/// crashed, stalled, or simply stopped publishing never leaves its last
/// setpoint (e.g. full throttle) latched.
pub const VESC_COMMAND_TIMEOUT: Duration = Duration::from_secs(1);

/// A desired steering/speed setpoint for the vehicle's actuators: how far
/// over the front wheel should point, and how fast the vehicle should be
/// going. Consumers (e.g. [`crate::actuators::SimulatedVehicle`]) are
/// responsible for approaching this setpoint within whatever limits the
/// real (or simulated) actuators have - this struct carries only the
/// desire, not a plan for getting there.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct VescCommand {
    /// Desired front-wheel steering angle, in radians, matching
    /// [`crate::environment::simulator::vehicle::bicycle`]'s
    /// `steering_angle_rad` convention: a **positive** angle steers toward
    /// increasing `heading_rad`.
    ///
    /// The world frame has x rightward and y *downward* (see
    /// `crate::environment::simulator::raster::ImageTransform`), so
    /// increasing heading rotates clockwise as the map is drawn - i.e.
    /// positive is a **right** turn, negative a left one. That's the
    /// convention `web_gui`'s WASD control follows, with D positive.
    pub servo_position_rad: f64,
    /// Desired forward speed, in meters/second.
    pub speed_mps: f64,
}

impl VescCommand {
    /// Builds a command. When it was written is tracked by the topic itself -
    /// see [`crate::WriteMeta`].
    pub fn new(servo_position_rad: f64, speed_mps: f64) -> Self {
        Self { servo_position_rad, speed_mps }
    }
}

impl Default for VescCommand {
    /// A stationary, centered command - used to pre-seed a
    /// `autonomous_vesc_command`/`human_vesc_command` topic before its writer (if any)
    /// has published its first real value.
    fn default() -> Self {
        Self::new(0.0, 0.0)
    }
}
