//! The [`VehicleStatus`] topic: the vehicle's current position, heading,
//! speed, and body-frame motion (velocity, yaw rate, acceleration), as
//! published by [`crate::actuators::SimulatedVehicle`] (or,
//! eventually, a real-vehicle localization stack producing the same shape).

/// Name of the topic a [`VehicleStatus`] is published on.
pub const VEHICLE_STATUS_TOPIC_NAME: &str = "vehicle_status";

/// Body size of every simulated vehicle - roughly a 1/10-scale RC car,
/// centered on its [`VehicleStatus`] position and aligned with its heading.
/// What [`crate::actuators::SimulatedVehicle`] draws, and what every
/// [`crate::sensors::SimulatedLidar`] sees of the other vehicles.
pub const VEHICLE_BODY_LENGTH_M: f64 = 0.45;
pub const VEHICLE_BODY_WIDTH_M: f64 = 0.25;

/// The vehicle's position, heading, and speed at one instant, in the same
/// world frame (meters) as [`crate::environment::MapInfo`], plus its motion
/// in the body frame (x forward, y left) - the ground truth a simulated
/// inertial sensor (see [`crate::sensors::SimulatedImu`]) measures from.
#[derive(Debug, Clone, Copy, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct VehicleStatus {
    /// X coordinate of the vehicle in world coordinates, in meters.
    pub x_m: f64,
    /// Y coordinate of the vehicle in world coordinates, in meters.
    pub y_m: f64,
    /// Heading of the vehicle body relative to the world X axis, in
    /// radians, wrapped to `(-pi, pi]`.
    pub heading_rad: f64,
    /// Forward speed along the body's heading, in meters/second.
    pub speed_mps: f64,
    /// Longitudinal velocity in the body frame (along the heading), in
    /// meters/second - signed, negative while reversing.
    pub vx_mps: f64,
    /// Lateral velocity in the body frame (to the left of the heading), in
    /// meters/second.
    pub vy_mps: f64,
    /// Yaw rate (rate of change of `heading_rad`, counterclockwise
    /// positive), in radians/second.
    pub yaw_rate_rad_s: f64,
    /// Longitudinal acceleration of the CG in the body frame, in
    /// meters/second^2 - what a body-mounted accelerometer's x axis reads
    /// (gravity aside), including the centripetal term, not just the rate of
    /// change of `vx_mps`.
    pub ax_mps2: f64,
    /// Lateral acceleration of the CG in the body frame, in
    /// meters/second^2 - what a body-mounted accelerometer's y axis reads,
    /// e.g. `speed^2 / radius` toward the inside of a steady turn.
    pub ay_mps2: f64,
}
