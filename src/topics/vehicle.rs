//! One vehicle's state and description: where it is, its shape and limits, which
//! physics model simulates it, and the names of its own topics.

mod actuator_status;
mod geometry;
mod limits;
mod model;
mod place_at_start;
mod start_state;
mod status;
mod topics;

pub use actuator_status::{ACTUATOR_STATUS_TOPIC_NAME, ActuatorStatus};
pub use geometry::{VEHICLE_GEOMETRY_TOPIC_NAME, VehicleGeometry};
pub use limits::{ActuatorLimits, VEHICLE_LIMITS_TOPIC_NAME};
pub use model::{
    VEHICLE_MODEL_PARAMETERS_TOPIC_NAME, VEHICLE_MODEL_SELECTION_TOPIC_NAME,
    VEHICLE_MODEL_STATUS_TOPIC_NAME, VehicleModelKind, VehicleModelParameters,
    VehicleModelSelection, VehicleModelStatus,
};
pub use place_at_start::{
    PLACE_AT_START_TOPIC_NAME, PlaceAtStart, Placement, PlacementInputs, PlacementTopics,
};
pub use start_state::{START_STATE_TOPIC_NAME, StartState};
pub use status::{VEHICLE_STATUS_TOPIC_NAME, VehicleStatus};
pub use topics::{AlgorithmTopics, OPPONENT_TOPIC_PREFIX, VehicleTopics};
