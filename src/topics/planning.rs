//! The topics tying [`crate::planning::Planner`] to whoever drives it (e.g.
//! `web_gui`'s Planning panel): [`PlanningParameters`] tunes it live,
//! [`PlanningRequest`] asks it to plan a race line for the selected map,
//! and [`PlanningStatus`] reports what it's doing, the parameter values it
//! runs with, and how the latest request went - mirroring
//! [`crate::topics::AutonomousParameters`] and
//! [`crate::topics::SlamSaveRequest`]/[`crate::topics::SlamStatus`].

use super::AlgorithmParameter;
use std::collections::BTreeMap;

/// Name of the topic [`PlanningParameters`] is published on.
pub const PLANNING_PARAMETERS_TOPIC_NAME: &str = "planning_parameters";
/// Name of the topic [`PlanningRequest`] is published on.
pub const PLANNING_REQUEST_TOPIC_NAME: &str = "planning_request";
/// Name of the topic [`PlanningStatus`] is published on.
pub const PLANNING_STATUS_TOPIC_NAME: &str = "planning_status";

/// The parameter values a driver (e.g. `web_gui`) wants the planner to run
/// with, by name. Always the *whole* wanted state, like
/// [`crate::topics::AutonomousParameters`], so two changes landing between
/// two of the planner's reads can't overwrite one another. Applied
/// (sanitized, see [`crate::topics::ParameterKind::sanitize`]) whenever the
/// planner is idle, and reported back in [`PlanningStatus::parameters`].
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PlanningParameters {
    pub values: BTreeMap<String, f64>,
}

/// Asks the planner to plan a race line for the map currently selected
/// (the `map` topic).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub struct PlanningRequest {
    /// Bumped on every request: the planner plans whenever this no longer
    /// matches the last one it handled, as
    /// [`crate::topics::SlamSaveRequest::requested`].
    pub requested: u64,
}

/// What the planner is doing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PlanningState {
    /// Waiting for a request; parameter changes apply right away.
    #[default]
    Idle,
    /// Planning - parameter changes wait until it's done.
    Computing,
}

/// How the planner handled a [`PlanningRequest`]: exactly one of
/// `saved_to` and `error` is set.
#[derive(Debug, Clone, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct PlanningOutcome {
    /// The [`PlanningRequest::requested`] this answers.
    pub requested: u64,
    /// The map folder the race line was planned for, if one was selected.
    pub map: Option<String>,
    /// The race line file written.
    pub saved_to: Option<String>,
    /// Why no race line was written.
    pub error: Option<String>,
    /// Whether the map had no centerline, so one was computed from its
    /// walls (and saved next to the race line).
    pub computed_centerline: bool,
    /// Length of one lap along the race line, in meters.
    pub lap_length_m: f64,
    /// Time of one lap at the race line's speed profile, in seconds.
    pub lap_time_s: f64,
    /// Largest curvature magnitude along the race line, in 1/m.
    pub max_curvature_per_m: f64,
    /// Largest curvature magnitude along the reference line it was planned
    /// from (the centerline), in 1/m - for comparison.
    pub reference_max_curvature_per_m: f64,
    /// How long planning took, in milliseconds.
    pub elapsed_ms: f64,
}

/// What [`crate::planning::Planner`] is currently doing, so a driver can
/// reflect it.
#[derive(Debug, Clone, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct PlanningStatus {
    pub state: PlanningState,
    /// While computing, the step it's on, e.g. `"Optimizing (iteration 2/5)"`.
    pub stage: String,
    /// Every tunable parameter, with the value currently in effect.
    pub parameters: Vec<AlgorithmParameter>,
    /// How the latest request went - `None` before the first.
    pub last_outcome: Option<PlanningOutcome>,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The debug recorder serializes every topic, so the status must
    /// survive a bincode round trip.
    #[test]
    fn a_status_survives_a_bincode_round_trip() {
        let status = PlanningStatus {
            state: PlanningState::Computing,
            stage: "Optimizing".to_string(),
            parameters: vec![AlgorithmParameter::float("vehicle_width_m", 0.1, 1.0, 0.01)],
            last_outcome: Some(PlanningOutcome {
                requested: 3,
                map: Some("track".to_string()),
                saved_to: Some("maps/track/race_lines/race_line.csv".to_string()),
                error: None,
                computed_centerline: true,
                lap_length_m: 42.0,
                lap_time_s: 12.5,
                max_curvature_per_m: 0.8,
                reference_max_curvature_per_m: 1.4,
                elapsed_ms: 250.0,
            }),
        };

        let encoded = bincode::serde::encode_to_vec(&status, bincode::config::standard()).unwrap();
        let (decoded, _): (PlanningStatus, _) =
            bincode::serde::decode_from_slice(&encoded, bincode::config::standard()).unwrap();

        assert_eq!(decoded, status);
    }
}
