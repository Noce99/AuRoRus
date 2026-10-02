//! The topics tying [`crate::perception::UbmDetector`] to whoever drives it
//! (e.g. `web_gui`'s Detector panel) and whoever uses what it finds:
//! [`DetectorParameters`] tunes it live, [`DetectorStatus`] reports what
//! it's doing and the parameter values it runs with - mirroring
//! [`crate::topics::PlanningParameters`]/[`crate::topics::PlanningStatus`] -
//! and [`DetectedOpponent`] is what it found in the ego vehicle's latest
//! scan.

use crate::topics::AlgorithmParameter;
use std::collections::BTreeMap;

/// Name of the topic [`DetectorParameters`] is published on.
pub const DETECTOR_PARAMETERS_TOPIC_NAME: &str = "detector_parameters";
/// Name of the topic [`DetectorStatus`] is published on.
pub const DETECTOR_STATUS_TOPIC_NAME: &str = "detector_status";
/// Name of the topic [`DetectedOpponent`] is published on.
pub const DETECTED_OPPONENT_TOPIC_NAME: &str = "detected_opponent";

/// The parameter values a driver (e.g. `web_gui`) wants the detector to run
/// with, by name. Always the *whole* wanted state, like
/// [`crate::topics::PlanningParameters`]. Applied (sanitized, see
/// [`crate::topics::ParameterKind::sanitize`]) before the next scan, and
/// reported back in [`DetectorStatus::parameters`].
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DetectorParameters {
    pub values: BTreeMap<String, f64>,
}

/// What the detector is doing, so a driver can reflect it.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DetectorStatus {
    /// Why it isn't detecting, e.g. no map or a stale pose - `None` while
    /// it runs on every scan.
    pub message: Option<String>,
    /// Every tunable parameter, with the value currently in effect.
    pub parameters: Vec<AlgorithmParameter>,
    /// How long the latest scan took to process, in milliseconds.
    pub scan_ms: f64,
}

/// An oriented rectangle around the scan points of a detected object, in
/// the map frame. Only the object's sides facing the lidar are seen, so it
/// can be smaller than the object.
#[derive(Debug, Clone, Copy, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BoundingBox {
    pub center: [f64; 2],
    /// Direction of the `length_m` side, in radians - in `[0, pi/2)`, since
    /// a rectangle looks the same turned by a quarter.
    pub heading_rad: f64,
    pub length_m: f64,
    pub width_m: f64,
    /// Counterclockwise, starting from the one at `-length/2, -width/2` in
    /// the box's own frame.
    pub corners: [[f64; 2]; 4],
}

/// The opponent found in the ego vehicle's latest scan, smoothed by a
/// constant-velocity Kalman filter. Published after every scan.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DetectedOpponent {
    /// Whether the latest scan saw it. When not, the other fields hold
    /// the filter's prediction (no `bounding_box`), which drifts the longer
    /// it goes unseen.
    pub detected: bool,
    /// Filtered position of its center, in the map frame (meters).
    pub position: [f64; 2],
    /// Filtered velocity, in the map frame (m/s).
    pub velocity: [f64; 2],
    /// Fitted around the latest scan's points of it - `None` when not
    /// `detected`.
    pub bounding_box: Option<BoundingBox>,
    /// Where the filter expects it next, one point per
    /// `prediction_dt_s` - empty when not `detected`.
    pub predictions: Vec<[f64; 2]>,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The debug recorder serializes every topic, so everything published
    /// here must survive a bincode round trip.
    #[test]
    fn topics_survive_a_bincode_round_trip() {
        fn round_trip<T>(value: &T) -> T
        where
            T: serde::Serialize + serde::de::DeserializeOwned,
        {
            let encoded =
                bincode::serde::encode_to_vec(value, bincode::config::standard()).unwrap();
            bincode::serde::decode_from_slice(&encoded, bincode::config::standard())
                .unwrap()
                .0
        }

        let opponent = DetectedOpponent {
            detected: true,
            position: [1.0, 2.0],
            velocity: [0.5, -0.5],
            bounding_box: Some(BoundingBox {
                center: [1.0, 2.0],
                heading_rad: 0.3,
                length_m: 0.5,
                width_m: 0.3,
                corners: [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
            }),
            predictions: vec![[1.1, 2.1]],
        };
        assert_eq!(round_trip(&opponent), opponent);

        let status = DetectorStatus {
            message: Some("No map".to_string()),
            parameters: vec![AlgorithmParameter::float("robot_radius_m", 0.0, 1.0, 0.01)],
            scan_ms: 1.5,
        };
        assert_eq!(round_trip(&status), status);
    }
}
