//! The [`StartState`] topic: where the vehicle should be initialized for the
//! currently loaded map - the middle of the start/finish line, heading along
//! the track's direction of travel there, at rest. Published by
//! [`crate::environment::MapServer`] alongside [`crate::topics::SelectedMap`]
//! whenever the map changes; watched continuously by
//! [`crate::simulation::SimulatedVehicle`], which places the vehicle here
//! again every time this changes - see also [`crate::topics::PlaceAtStart`]
//! for placing it here on demand even when it hasn't.

/// Name of the topic a [`StartState`] is published on.
pub const START_STATE_TOPIC_NAME: &str = "start_state";

/// The vehicle's initial position, heading, and speed for the currently
/// loaded map, in the same world frame (meters) as
/// [`crate::environment::MapInfo`] - shares [`crate::topics::VehicleStatus`]'s
/// shape since it's the state [`crate::simulation::SimulatedVehicle`] starts
/// advancing from. `speed_mps` is always `0.0`: the vehicle always starts at
/// rest.
#[derive(Debug, Clone, Copy, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct StartState {
    /// X coordinate of the start position in world coordinates, in meters -
    /// the middle of the start/finish line.
    pub x_m: f64,
    /// Y coordinate of the start position in world coordinates, in meters -
    /// the middle of the start/finish line.
    pub y_m: f64,
    /// Heading to start facing, in radians - tangent to the track's
    /// direction of travel at the start/finish line.
    pub heading_rad: f64,
    /// Always `0.0` - the vehicle always starts at rest.
    pub speed_mps: f64,
}
