//! The [`VehicleModelStatus`]/[`VehicleModelSelection`] topic pair: which
//! vehicle physics model is currently wanted, and which one is currently
//! running - together, how [`crate::actuators::SimulatedVehicle`] and a
//! driver of the wanted selection (e.g. `web_gui`) agree on the live model
//! without either one needing to know how the other decides its defaults.
//! Mirrors [`crate::topics::MapSelection`]/[`crate::topics::SelectedMap`].

/// Name of the topic [`VehicleModelSelection`] is published on.
pub const VEHICLE_MODEL_SELECTION_TOPIC_NAME: &str = "vehicle_model_selection";
/// Name of the topic [`VehicleModelStatus`] is published on.
pub const VEHICLE_MODEL_STATUS_TOPIC_NAME: &str = "vehicle_model_status";

/// Which vehicle physics model is wanted/running. The concrete model
/// parameters and actuator limits for each kind live in
/// [`crate::actuators::simulated_vehicle`], not here - this enum is just the
/// stable, serializable identity that flows over the selection/status
/// topics and the web API.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum VehicleModelKind {
    /// A CG-referenced kinematic bicycle model - see
    /// [`crate::environment::simulator::vehicle::bicycle`].
    #[default]
    Bicycle,
    /// A dynamic bicycle model with lateral tire forces - see
    /// [`crate::environment::simulator::vehicle::dynamic_bicycle`].
    DynamicBicycle,
}

/// The vehicle model kind a driver of the selection (e.g. `web_gui`)
/// currently wants running. [`crate::actuators::SimulatedVehicle`] polls
/// this and switches models whenever it no longer matches the currently
/// running one, publishing the change on [`VehicleModelStatus`].
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct VehicleModelSelection {
    pub kind: VehicleModelKind,
}

/// The vehicle model kind currently running, published by
/// [`crate::actuators::SimulatedVehicle`] so a driver of the selection can
/// tell what's actually active - e.g. to reflect it in a dropdown on
/// startup, or after another client changes it.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct VehicleModelStatus {
    pub kind: VehicleModelKind,
}
