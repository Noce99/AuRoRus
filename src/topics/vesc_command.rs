//! The [`VescCommand`] topic: a desired steering/speed setpoint for the
//! vehicle's actuators, published on two separate topics -
//! [`VESC_COMMAND_TOPIC_NAME`] (an autonomous controller, not implemented
//! yet) and [`HUMAN_VESC_COMMAND_TOPIC_NAME`] (a human driver, e.g.
//! `web_gui`'s WASD control) - both sharing this same shape so any consumer
//! reads them identically.

use std::time::{SystemTime, UNIX_EPOCH};

/// Name of the topic an autonomous controller publishes its desired
/// steering/speed setpoint on. Nothing writes this yet - see
/// [`crate::actuators::SimulatedVehicle`], which reads it alongside
/// [`HUMAN_VESC_COMMAND_TOPIC_NAME`] and acts on whichever was published
/// more recently.
pub const VESC_COMMAND_TOPIC_NAME: &str = "vesc_command";
/// Name of the topic a human driver's desired steering/speed setpoint is
/// published on, e.g. by `web_gui`'s WASD control.
pub const HUMAN_VESC_COMMAND_TOPIC_NAME: &str = "human_vesc_command";

/// A desired steering/speed setpoint for the vehicle's actuators: how far
/// over the front wheel should point, and how fast the vehicle should be
/// going. Consumers (e.g. [`crate::actuators::SimulatedVehicle`]) are
/// responsible for approaching this setpoint within whatever limits the
/// real (or simulated) actuators have - this struct carries only the
/// desire, not a plan for getting there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VescCommand {
    /// When this command was published, as microseconds since the Unix
    /// epoch - lets a consumer reading both [`VESC_COMMAND_TOPIC_NAME`] and
    /// [`HUMAN_VESC_COMMAND_TOPIC_NAME`] prefer whichever is fresher.
    pub time_stamp_us: u128,
    /// Desired front-wheel steering angle, in radians (positive = left,
    /// matching [`crate::environment::simulator::vehicle::bicycle`]'s
    /// `steering_angle_rad` convention).
    pub servo_position_rad: f64,
    /// Desired forward speed, in meters/second.
    pub speed_mps: f64,
}

impl VescCommand {
    /// Builds a command stamped with the current time.
    pub fn new(servo_position_rad: f64, speed_mps: f64) -> Self {
        let time_stamp_us = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_micros();
        Self { time_stamp_us, servo_position_rad, speed_mps }
    }
}

impl Default for VescCommand {
    /// A stationary, centered command, stamped with the current time - used
    /// to pre-seed a `vesc_command`/`human_vesc_command` topic before its
    /// writer (if any) has published its first real value.
    fn default() -> Self {
        Self::new(0.0, 0.0)
    }
}
