//! Shared topic definitions: the concrete types published/read on named
//! [`crate::RwLockTopic`]s, so multiple binaries can agree on the same shape
//! without redefining it.

mod autonomous_control;
mod detection;
mod drawing;
mod imu;
mod lap_telemetry;
mod lidar_scan;
mod map;
mod odometry;
mod opponents;
mod place_at_start;
mod planning;
mod race_line;
mod race_start;
mod slam;
mod start_state;
mod vehicle_limits;
mod vehicle_model;
mod vehicle_status;
mod vehicle_topics;
mod vesc_command;
mod vesc_parameters;
mod vesc_status;

pub use autonomous_control::{
    AUTONOMOUS_ALGORITHM_SELECTION_TOPIC_NAME, AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME,
    AUTONOMOUS_CONTROL_INFO_TOPIC_PREFIX, AUTONOMOUS_CONTROL_TOPIC_PREFIX,
    AUTONOMOUS_PARAMETERS_TOPIC_NAME, AlgorithmParameter, AlgorithmRequirements,
    AutonomousAlgorithmInfo, AutonomousAlgorithmSelection, AutonomousAlgorithmStatus,
    AutonomousParameters, AvailableAlgorithm, ParameterKind,
};
pub use detection::{
    BoundingBox, DETECTED_OPPONENT_TOPIC_NAME, DETECTOR_PARAMETERS_TOPIC_NAME,
    DETECTOR_STATUS_TOPIC_NAME, DetectedOpponent, DetectorParameters, DetectorStatus,
};
pub use drawing::{Color, DRAW_TOPIC_PREFIX, Drawing, DrawingElement, Shape};
pub use imu::{IMU_TOPIC_NAME, ImuReading};
pub use lap_telemetry::{LAP_TELEMETRY_TOPIC_NAME, LapRecord, LapTelemetry, LapTrace};
pub use lidar_scan::{LIDAR_SCAN_TOPIC_NAME, LidarScan};
pub use map::{MAP_SELECTION_TOPIC_NAME, MAP_TOPIC_NAME, MapSelection, SelectedMap};
pub use odometry::{ODOMETRY_TOPIC_NAME, Odometry};
pub use opponents::{
    NumberedRequest, OPPONENT_REQUESTS_TOPIC_NAME, OPPONENTS_TOPIC_NAME, Opponent, OpponentColor,
    OpponentOutcome, OpponentRequest, OpponentRequests, OpponentSpec, Opponents,
};
pub use place_at_start::{
    PLACE_AT_START_TOPIC_NAME, PlaceAtStart, Placement, PlacementInputs, PlacementTopics,
};
pub use planning::{
    PLANNING_PARAMETERS_TOPIC_NAME, PLANNING_REQUEST_TOPIC_NAME, PLANNING_STATUS_TOPIC_NAME,
    PlanningObjective, PlanningOutcome, PlanningParameters, PlanningRequest, PlanningState,
    PlanningStatus,
};
pub use race_line::{
    RACE_LINE_SELECTION_TOPIC_NAME, RACE_LINE_TOPIC_NAME, RaceLineSelection, SelectedRaceLine,
};
pub use race_start::{GridSlot, RACE_START_TOPIC_NAME, RaceStart, Racer, now_ms};
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
pub use vehicle_status::{
    VEHICLE_BODY_LENGTH_M, VEHICLE_BODY_WIDTH_M, VEHICLE_STATUS_TOPIC_NAME, VehicleStatus,
};
pub use vehicle_topics::{AlgorithmTopics, OPPONENT_TOPIC_PREFIX, VehicleTopics};
pub use vesc_command::{
    AUTONOMOUS_VESC_COMMAND_TOPIC_NAME, HUMAN_VESC_COMMAND_TOPIC_NAME, VESC_COMMAND_TIMEOUT,
    VescCommand,
};
pub use vesc_parameters::{
    VESC_PARAMETERS_STATUS_TOPIC_NAME, VESC_PARAMETERS_TOPIC_NAME, VescParameters,
    VescParametersStatus,
};
pub use vesc_status::{VESC_STATUS_TOPIC_NAME, VescStatus};
