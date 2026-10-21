//! The map being driven and what is built from it: the selected map and race
//! line, SLAM's map and commands, which localization runs, and race line
//! planning.

mod localization;
mod planning;
mod race_line;
mod selected_map;
mod slam;

pub use localization::{
    LOCALIZATION_COMMAND_TOPIC_NAME, LOCALIZATION_STATUS_TOPIC_NAME, LocalizationAlgorithm,
    LocalizationCommand, LocalizationStatus, PARTICLE_FILTER_PARAMETERS_TOPIC_NAME,
    ParticleFilterParameters,
};
pub use planning::{
    PLANNING_PARAMETERS_TOPIC_NAME, PLANNING_REQUEST_TOPIC_NAME, PLANNING_STATUS_TOPIC_NAME,
    PlanningObjective, PlanningOutcome, PlanningParameters, PlanningRequest, PlanningState,
    PlanningStatus,
};
pub use race_line::{
    RACE_LINE_SELECTION_TOPIC_NAME, RACE_LINE_TOPIC_NAME, RaceLineSelection, SelectedRaceLine,
};
pub use selected_map::{MAP_SELECTION_TOPIC_NAME, MAP_TOPIC_NAME, MapSelection, SelectedMap};
pub use slam::{
    SLAM_COMMAND_TOPIC_NAME, SLAM_MAP_TOPIC_NAME, SLAM_PARAMETERS_TOPIC_NAME, SLAM_SAVE_TOPIC_NAME,
    SLAM_STATUS_TOPIC_NAME, SlamCommand, SlamMap, SlamParameters, SlamSaveOutcome, SlamSaveRequest,
    SlamState, SlamStatus,
};
