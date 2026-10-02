//! [`VehicleTopics`]: the names of the topics that belong to one simulated
//! vehicle, so the same executors (vehicle, lidar, autonomous algorithms)
//! can run once per vehicle without two of them claiming the same topic.
//!
//! The ego vehicle's names are the plain constants every other module knows
//! (e.g. [`VEHICLE_STATUS_TOPIC_NAME`]); an opponent's are the same names
//! behind its own prefix, e.g. `opponent/1/vehicle_status`.

use crate::topics::{
    ACTUATOR_STATUS_TOPIC_NAME, AUTONOMOUS_CONTROL_INFO_TOPIC_PREFIX,
    AUTONOMOUS_CONTROL_TOPIC_PREFIX, LIDAR_SCAN_TOPIC_NAME, ODOMETRY_TOPIC_NAME,
    RACE_LINE_TOPIC_NAME, VEHICLE_GEOMETRY_TOPIC_NAME, VEHICLE_LIMITS_TOPIC_NAME,
    VEHICLE_STATUS_TOPIC_NAME,
};

/// Prefix of every opponent's topics: `opponent/<n>/`.
pub const OPPONENT_TOPIC_PREFIX: &str = "opponent/";

/// Where one vehicle's topics live: the plain topic name constants for the
/// ego vehicle, the same names behind `opponent/<n>/` for opponent `n`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VehicleTopics {
    /// Empty for the ego vehicle.
    prefix: String,
}

impl VehicleTopics {
    /// The ego vehicle's topics: exactly the plain topic name constants.
    pub fn ego() -> Self {
        Self {
            prefix: String::new(),
        }
    }

    /// Opponent `n`'s topics, all under `opponent/<n>/`.
    pub fn opponent(n: u32) -> Self {
        Self {
            prefix: format!("{OPPONENT_TOPIC_PREFIX}{n}/"),
        }
    }

    /// What every one of these topic names starts with - empty for the ego.
    pub fn prefix(&self) -> &str {
        &self.prefix
    }

    pub fn is_ego(&self) -> bool {
        self.prefix.is_empty()
    }

    fn name(&self, base: &str) -> String {
        format!("{}{base}", self.prefix)
    }

    pub fn vehicle_status(&self) -> String {
        self.name(VEHICLE_STATUS_TOPIC_NAME)
    }

    pub fn actuator_status(&self) -> String {
        self.name(ACTUATOR_STATUS_TOPIC_NAME)
    }

    pub fn vehicle_limits(&self) -> String {
        self.name(VEHICLE_LIMITS_TOPIC_NAME)
    }

    pub fn vehicle_geometry(&self) -> String {
        self.name(VEHICLE_GEOMETRY_TOPIC_NAME)
    }

    pub fn lidar_scan(&self) -> String {
        self.name(LIDAR_SCAN_TOPIC_NAME)
    }

    pub fn race_line(&self) -> String {
        self.name(RACE_LINE_TOPIC_NAME)
    }

    pub fn odometry(&self) -> String {
        self.name(ODOMETRY_TOPIC_NAME)
    }

    /// The command and info topics of the autonomous algorithm `algorithm`
    /// (its file stem) driving this vehicle.
    pub fn algorithm(&self, algorithm: &str) -> AlgorithmTopics {
        AlgorithmTopics {
            command: self.name(&format!("{AUTONOMOUS_CONTROL_TOPIC_PREFIX}{algorithm}")),
            info: self.name(&format!(
                "{AUTONOMOUS_CONTROL_INFO_TOPIC_PREFIX}{algorithm}"
            )),
        }
    }
}

/// An autonomous algorithm instance's own topics - see
/// [`crate::autonomous_control::AutonomousControlExt::claim_autonomous_control`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AlgorithmTopics {
    /// Its [`crate::topics::VescCommand`]s.
    pub command: String,
    /// Its [`crate::topics::AutonomousAlgorithmInfo`].
    pub info: String,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_ego_uses_the_plain_topic_names() {
        let ego = VehicleTopics::ego();
        assert!(ego.is_ego());
        assert_eq!(ego.vehicle_status(), VEHICLE_STATUS_TOPIC_NAME);
        assert_eq!(ego.actuator_status(), ACTUATOR_STATUS_TOPIC_NAME);
        assert_eq!(ego.vehicle_limits(), VEHICLE_LIMITS_TOPIC_NAME);
        assert_eq!(ego.lidar_scan(), LIDAR_SCAN_TOPIC_NAME);
        assert_eq!(ego.race_line(), RACE_LINE_TOPIC_NAME);
        assert_eq!(ego.odometry(), ODOMETRY_TOPIC_NAME);
        assert_eq!(
            ego.algorithm("gap_follower"),
            AlgorithmTopics {
                command: "autonomous_control/gap_follower".into(),
                info: "autonomous_control_info/gap_follower".into(),
            }
        );
    }

    #[test]
    fn an_opponent_prefixes_every_name_and_hides_from_the_algorithm_picker() {
        let opponent = VehicleTopics::opponent(3);
        assert!(!opponent.is_ego());
        assert_eq!(opponent.vehicle_status(), "opponent/3/vehicle_status");
        assert_eq!(opponent.lidar_scan(), "opponent/3/lidar_scan");
        let algorithm = opponent.algorithm("gap_follower");
        assert_eq!(
            algorithm.command,
            "opponent/3/autonomous_control/gap_follower"
        );
        // `AutonomousControlsHandler` finds algorithms by this prefix.
        assert!(
            !algorithm
                .info
                .starts_with(AUTONOMOUS_CONTROL_INFO_TOPIC_PREFIX)
        );
    }
}
