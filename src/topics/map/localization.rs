//! The topics picking and steering [`crate::localization::Slam`]'s
//! localization backend, next to the [`crate::topics::SlamCommand`] that
//! starts and pauses it: [`LocalizationCommand`] says which algorithm
//! localizes and asks it to restart from a pose or globally,
//! [`ParticleFilterParameters`] tunes the particle filter, and
//! [`LocalizationStatus`] reports what the backend is doing.
//!
//! Kept apart from `slam_command`/`slam_status`, which mapping sessions and
//! debug recordings store with bincode: a field added to those would make
//! every older recording undecodable.

use crate::topics::AlgorithmParameter;
use std::collections::BTreeMap;

/// Name of the topic [`LocalizationCommand`] is published on.
pub const LOCALIZATION_COMMAND_TOPIC_NAME: &str = "localization_command";
/// Name of the topic [`LocalizationStatus`] is published on.
pub const LOCALIZATION_STATUS_TOPIC_NAME: &str = "localization_status";
/// Name of the topic [`ParticleFilterParameters`] are published on.
pub const PARTICLE_FILTER_PARAMETERS_TOPIC_NAME: &str = "particle_filter_parameters";

/// Which algorithm localizes the vehicle on the selected map.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LocalizationAlgorithm {
    /// SLAM's correlative scan matcher against the map's walls.
    #[default]
    ScanMatching,
    /// The particle filter (Monte Carlo localization).
    ParticleFilter,
}

impl LocalizationAlgorithm {
    /// The name the web API uses for this algorithm - the same as its serde
    /// name.
    pub fn api_str(self) -> &'static str {
        match self {
            LocalizationAlgorithm::ScanMatching => "scan_matching",
            LocalizationAlgorithm::ParticleFilter => "particle_filter",
        }
    }

    /// The algorithm named `name` in the web API, if any.
    pub fn from_api_str(name: &str) -> Option<Self> {
        Self::ALL
            .into_iter()
            .find(|algorithm| algorithm.api_str() == name)
    }

    /// Every algorithm.
    pub const ALL: [LocalizationAlgorithm; 2] = [
        LocalizationAlgorithm::ScanMatching,
        LocalizationAlgorithm::ParticleFilter,
    ];
}

/// What a driver of localization (e.g. `web_gui`) wants of it, beyond
/// starting and pausing it.
#[derive(Debug, Clone, Copy, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct LocalizationCommand {
    /// The algorithm to localize with. Switching while localizing is
    /// refused - the running one carries on - so pause first.
    pub algorithm: LocalizationAlgorithm,
    /// The pose `(x_m, y_m, heading_rad)` on the map to restart from, as
    /// of [`Self::set_pose_requested`].
    pub set_pose: Option<[f64; 3]>,
    /// Bumped on every new [`Self::set_pose`]: localization restarts from it
    /// whenever this no longer matches what it last applied - a counter, as
    /// [`crate::topics::SlamCommand::clear_requested`].
    pub set_pose_requested: u64,
    /// Bumped to restart the particle filter globally: anywhere on the map,
    /// facing anywhere. Scan matching can't, and ignores it.
    pub relocalize_requested: u64,
}

/// The particle filter's parameter values a driver (e.g. `web_gui`) wants,
/// by name - always the *whole* wanted state, as
/// [`crate::topics::SlamParameters`].
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ParticleFilterParameters {
    pub values: BTreeMap<String, f64>,
}

/// What the localization backend is doing - the pose itself is on
/// [`crate::topics::SlamStatus`], whatever the backend.
#[derive(Debug, Clone, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct LocalizationStatus {
    /// The algorithm running - or, while not localizing, the one that will.
    pub algorithm: LocalizationAlgorithm,
    /// Whether it's localizing on a map right now.
    pub active: bool,
    /// How long processing the latest scan took, in milliseconds.
    pub last_process_ms: Option<f64>,
    /// How confident the latest estimate is, in the algorithm's own terms:
    /// scan matching's match response (`0` to `1`, higher is better), the
    /// particle filter's position spread (in meters, lower is better).
    pub quality: Option<f64>,
    /// The particle filter's spread of headings, in radians.
    pub particle_spread_rad: Option<f64>,
    /// The particle filter's effective sample size: how many of its
    /// particles carry the weight.
    pub effective_sample_size: Option<f64>,
    /// How many particles the particle filter has.
    pub particles: Option<usize>,
    /// Whether a global restart of the particle filter hasn't converged
    /// yet.
    pub relocalizing: bool,
    /// The particle filter's tunable parameters, with the values in effect.
    pub particle_filter_parameters: Vec<AlgorithmParameter>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_algorithm_round_trips_through_its_api_name() {
        for algorithm in LocalizationAlgorithm::ALL {
            assert_eq!(
                LocalizationAlgorithm::from_api_str(algorithm.api_str()),
                Some(algorithm)
            );
            assert_eq!(
                serde_json::to_string(&algorithm).unwrap(),
                format!("\"{}\"", algorithm.api_str())
            );
        }
    }

    /// The debug recorder serializes every topic with bincode.
    #[test]
    fn the_status_and_command_survive_a_bincode_round_trip() {
        let status = LocalizationStatus {
            algorithm: LocalizationAlgorithm::ParticleFilter,
            active: true,
            quality: Some(0.1),
            particles: Some(1500),
            ..LocalizationStatus::default()
        };
        let command = LocalizationCommand {
            set_pose: Some([1.0, 2.0, 0.5]),
            set_pose_requested: 3,
            ..LocalizationCommand::default()
        };
        let config = bincode::config::standard();
        let encoded = bincode::serde::encode_to_vec(&status, config).unwrap();
        let (decoded, _): (LocalizationStatus, _) =
            bincode::serde::decode_from_slice(&encoded, config).unwrap();
        assert_eq!(decoded, status);
        let encoded = bincode::serde::encode_to_vec(command, config).unwrap();
        let (decoded, _): (LocalizationCommand, _) =
            bincode::serde::decode_from_slice(&encoded, config).unwrap();
        assert_eq!(decoded, command);
    }
}
