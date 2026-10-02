//! The [`ImuReading`] topic: one raw sample from the vehicle's inertial
//! sensors and wheel-speed feedback - what dead reckoning (see
//! [`crate::localization::DeadReckoning`]) integrates into
//! [`crate::topics::Odometry`]. Published by [`crate::simulation::SimulatedImu`]
//! in simulation, and meant to be published in the very same shape by the
//! real vehicle's VESC driver - already converted to SI units and to this
//! body frame, so nothing downstream can tell the two apart.

/// Name of the topic an [`ImuReading`] is published on.
pub const IMU_TOPIC_NAME: &str = "imu";

/// One raw sample, in the body frame (x forward, y left, z up) and SI units.
/// Readings are *measurements*: noisy, possibly biased, never corrected here.
///
/// Carries no sequence number or timestamp of its own - the topic's
/// [`crate::WriteMeta`] already stamps every write with a `write_count` (to
/// tell a new sample from a re-read one) and a `written_at` instant (to time
/// the interval between two samples).
#[derive(Debug, Clone, Copy, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct ImuReading {
    /// Longitudinal speed as reported by the drive motor's feedback (e.g.
    /// the VESC's ERPM, divided by its speed-to-ERPM gain), in
    /// meters/second - signed, negative while reversing.
    pub wheel_speed_mps: f64,
    /// Angular velocity about the vertical axis (counterclockwise positive),
    /// from the gyroscope, in radians/second.
    pub yaw_rate_rad_s: f64,
    /// Longitudinal acceleration from the accelerometer, in meters/second^2.
    pub ax_mps2: f64,
    /// Lateral acceleration from the accelerometer (left positive), in
    /// meters/second^2.
    pub ay_mps2: f64,
}
