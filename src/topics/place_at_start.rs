//! The [`PlaceAtStart`] topic: an explicit request that
//! [`crate::actuators::SimulatedVehicle`] place the vehicle at whatever
//! [`crate::topics::StartState`] currently holds - even when that value
//! hasn't changed, e.g. the "P" key in `web_gui`'s UI wanting to snap the
//! vehicle back to the start line after it's driven away on the same map.
//! [`crate::topics::StartState`] changing on its own already triggers a
//! placement (see its doc comment), so this only needs to cover the case a
//! plain value comparison can't.

/// Name of the topic a [`PlaceAtStart`] is published on.
pub const PLACE_AT_START_TOPIC_NAME: &str = "place_at_start";

/// A monotonically increasing counter - `web_gui` bumps it on every
/// placement request; [`crate::actuators::SimulatedVehicle`] places the
/// vehicle at [`crate::topics::StartState`] whenever this no longer matches
/// what it last applied. A counter rather than a bool so two rapid requests
/// can't cancel each other out via toggle parity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub struct PlaceAtStart {
    pub requested: u64,
}
