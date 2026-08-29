//! The [`VehicleStatus`] topic: the vehicle's current position, heading,
//! and speed, as published by [`crate::actuators::SimulatedVehicle`] (or,
//! eventually, a real-vehicle localization stack producing the same shape).

/// Name of the topic a [`VehicleStatus`] is published on.
pub const VEHICLE_STATUS_TOPIC_NAME: &str = "vehicle_status";

/// The vehicle's position, heading, and speed at one instant, in the same
/// world frame (meters) as [`crate::environment::MapInfo`].
#[derive(Debug, Clone, Copy, PartialEq, Default, serde::Serialize)]
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
}
