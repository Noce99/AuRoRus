//! The map being driven and what is built from it: the selected map and race
//! line, SLAM's map and commands, and race line planning.

mod planning;
mod race_line;
mod selected_map;
mod slam;

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
    SLAM_COMMAND_TOPIC_NAME, SLAM_MAP_TOPIC_NAME, SLAM_SAVE_TOPIC_NAME, SLAM_STATUS_TOPIC_NAME,
    SlamCommand, SlamMap, SlamSaveOutcome, SlamSaveRequest, SlamState, SlamStatus,
};
