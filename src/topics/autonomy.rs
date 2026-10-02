//! The autonomous stack: the algorithms, the one selected, their tunable
//! parameters, and the opponent the detector finds.

mod autonomous_control;
mod detection;

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
