//! Shared topic definitions: the concrete types published/read on named
//! [`crate::RwLockTopic`]s, so multiple binaries can agree on the same shape
//! without redefining it.

mod autonomous_control;
mod drawing;
mod imu;
mod lidar_scan;
mod map;
mod odometry;
mod place_at_start;
mod planning;
mod race_line;
mod slam;
mod start_state;
mod vehicle_limits;
mod vehicle_model;
mod vehicle_status;
mod vesc_command;

pub use autonomous_control::{
    AUTONOMOUS_ALGORITHM_SELECTION_TOPIC_NAME, AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME,
    AUTONOMOUS_CONTROL_INFO_TOPIC_PREFIX, AUTONOMOUS_CONTROL_TOPIC_PREFIX,
    AUTONOMOUS_PARAMETERS_TOPIC_NAME, AlgorithmParameter, AutonomousAlgorithmInfo,
    AutonomousAlgorithmSelection, AutonomousAlgorithmStatus, AutonomousParameters,
    AvailableAlgorithm, ParameterKind,
};
pub use drawing::{Color, DRAW_TOPIC_PREFIX, Drawing, DrawingElement, Shape};
pub use imu::{IMU_TOPIC_NAME, ImuReading};
pub use lidar_scan::{LIDAR_SCAN_TOPIC_NAME, LidarScan};
pub use map::{MAP_SELECTION_TOPIC_NAME, MAP_TOPIC_NAME, MapSelection, SelectedMap};
pub use odometry::{ODOMETRY_TOPIC_NAME, Odometry};
pub use place_at_start::{PLACE_AT_START_TOPIC_NAME, PlaceAtStart};
pub use planning::{
    PLANNING_PARAMETERS_TOPIC_NAME, PLANNING_REQUEST_TOPIC_NAME, PLANNING_STATUS_TOPIC_NAME,
    PlanningObjective, PlanningOutcome, PlanningParameters, PlanningRequest, PlanningState,
    PlanningStatus,
};
pub use race_line::{RACE_LINE_TOPIC_NAME, RaceLineKind, SelectedRaceLine};
pub use slam::{
    SLAM_COMMAND_TOPIC_NAME, SLAM_MAP_TOPIC_NAME, SLAM_SAVE_TOPIC_NAME, SLAM_STATUS_TOPIC_NAME,
    SlamCommand, SlamMap, SlamSaveOutcome, SlamSaveRequest, SlamState, SlamStatus,
};
pub use start_state::{START_STATE_TOPIC_NAME, StartState};
pub use vehicle_limits::{ActuatorLimits, VEHICLE_LIMITS_TOPIC_NAME};
pub use vehicle_model::{
    VEHICLE_MODEL_PARAMETERS_TOPIC_NAME, VEHICLE_MODEL_SELECTION_TOPIC_NAME,
    VEHICLE_MODEL_STATUS_TOPIC_NAME, VehicleModelKind, VehicleModelParameters,
    VehicleModelSelection, VehicleModelStatus,
};
pub use vehicle_status::{VEHICLE_STATUS_TOPIC_NAME, VehicleStatus};
pub use vesc_command::{
    AUTONOMOUS_VESC_COMMAND_TOPIC_NAME, HUMAN_VESC_COMMAND_TOPIC_NAME, VESC_COMMAND_TIMEOUT,
    VescCommand,
};
