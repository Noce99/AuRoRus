//! The [`Odometry`] topic: the vehicle's pose as dead-reckoned from
//! [`crate::topics::ImuReading`]s alone, in its own drifting `odom` frame -
//! published by [`crate::localization::DeadReckoning`]. The equivalent of the
//! `nav_msgs/Odometry` message a ROS 2 localization or mapping stack (e.g. a
//! particle filter or slam_toolbox) consumes.

/// Name of the topic an [`Odometry`] is published on.
pub const ODOMETRY_TOPIC_NAME: &str = "odometry";

/// The dead-reckoned pose and current motion of the vehicle.
///
/// The pose lives in the `odom` frame: it starts at the origin, heading
/// along +x, every time dead reckoning is reset (e.g. the vehicle is placed
/// at the start line), and drifts away from the truth from there - it is
/// *not* the world frame [`crate::topics::VehicleStatus`] uses. Relating the
/// two (map -> odom) is localization's job; consumers that only need
/// relative motion (e.g. a particle filter's motion model) should use the
/// difference between consecutive poses.
#[derive(Debug, Clone, Copy, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct Odometry {
    /// X coordinate in the `odom` frame, in meters.
    pub x_m: f64,
    /// Y coordinate in the `odom` frame, in meters.
    pub y_m: f64,
    /// Heading relative to the `odom` frame's x axis, in radians, wrapped to
    /// `(-pi, pi]`.
    pub heading_rad: f64,
    /// Longitudinal speed the pose was integrated with, in meters/second.
    pub speed_mps: f64,
    /// Yaw rate the pose was integrated with, in radians/second.
    pub yaw_rate_rad_s: f64,
    /// Latest longitudinal acceleration measured, in meters/second^2 -
    /// passed through untouched for filters that fuse it (e.g. an EKF).
    pub ax_mps2: f64,
    /// Latest lateral acceleration measured, in meters/second^2 - passed
    /// through untouched, like `ax_mps2`.
    pub ay_mps2: f64,
    /// Covariance of the pose `(x_m, y_m, heading_rad)`, row-major, in the
    /// matching squared units. Zero right after a reset, growing as the
    /// noise assumed in [`crate::localization::DeadReckoningConfig`]
    /// accumulates.
    pub covariance: [[f64; 3]; 3],
    /// How many times dead reckoning has been reset to the `odom` origin
    /// since it started - bumped on every reset, so a consumer buffering
    /// poses (e.g. [`crate::localization::Slam`]) can tell samples from
    /// before a reset, in a frame that no longer exists, from those after
    /// it.
    pub reset_count: u64,
}
