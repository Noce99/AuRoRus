//! Mapping sessions: everything [`crate::localization::Slam`] maps from,
//! recorded while mapping, so a map can be built again offline with other
//! parameters. A session is a `.debug` file (see [`crate::debug_format`])
//! holding only [`recorded_topics`], named `<name>.mapping` under the mapping
//! sessions folder, and starting with a [`MappingSessionInfo`] record.
//!
//! Recorded by a [`DebugRecorder`] from [`recorder`], polling at
//! [`MAPPING_SESSION_FREQUENCY_HZ`] - well above every recorded topic's rate,
//! so a write is only missed if the recorder's thread is held up; how many
//! were is in [`crate::DebugStatus::missed_writes`]. See
//! `documentation/slam.md`.

use crate::DebugPreamble;
use crate::DebugRecorder;
use crate::topics::{
    IMU_TOPIC_NAME, LIDAR_SCAN_TOPIC_NAME, ODOMETRY_TOPIC_NAME, PLACE_AT_START_TOPIC_NAME,
    SLAM_COMMAND_TOPIC_NAME, START_STATE_TOPIC_NAME, VehicleTopics,
};
use std::collections::BTreeMap;

/// Extension every mapping session's file gets.
pub const MAPPING_SESSION_EXTENSION: &str = ".mapping";
/// The group a mapping session's recording runs as - not
/// [`crate::DEBUG_GROUP`], so it can run alongside a debug recording.
pub const MAPPING_SESSION_GROUP: &str = "mapping_session";
/// How often a mapping session's recorder polls its topics, in Hz: faster
/// than any of them is written (odometry, the fastest, at ~200 Hz).
pub const MAPPING_SESSION_FREQUENCY_HZ: f64 = 1000.0;
/// Name of the record holding the session's [`MappingSessionInfo`].
pub const MAPPING_SESSION_INFO_TOPIC_NAME: &str = "mapping_session_info";

/// The topics a mapping session records: SLAM's inputs (`lidar_scan`,
/// `odometry`), what odometry is integrated from (`imu`), where the map's
/// frame sits in the world (`start_state`, `place_at_start`), when mapping
/// was played, paused and cleared (`slam_command`), the vehicle's size, and
/// the true pose (`vehicle_status`, only written in simulation), to score a
/// map against.
pub fn recorded_topics() -> Vec<String> {
    let ego = VehicleTopics::ego();
    vec![
        LIDAR_SCAN_TOPIC_NAME.to_string(),
        ODOMETRY_TOPIC_NAME.to_string(),
        IMU_TOPIC_NAME.to_string(),
        START_STATE_TOPIC_NAME.to_string(),
        PLACE_AT_START_TOPIC_NAME.to_string(),
        SLAM_COMMAND_TOPIC_NAME.to_string(),
        ego.vehicle_geometry(),
        ego.vehicle_status(),
    ]
}

/// A recorder of [`recorded_topics`], as [`MAPPING_SESSION_GROUP`].
pub fn recorder() -> DebugRecorder {
    DebugRecorder::for_topics(MAPPING_SESSION_GROUP, recorded_topics())
}

/// What a session was recorded with, written once at its start. Stored as
/// JSON rather than bincode, so a session stays readable as fields are
/// added.
#[derive(Debug, Clone, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct MappingSessionInfo {
    /// The real car's name, or `None` in simulation.
    #[serde(default)]
    pub car: Option<String>,
    /// The map folder selected when recording started - in simulation, the
    /// true map the session can be compared against.
    #[serde(default)]
    pub map: Option<String>,
    /// The parameter values SLAM ran with when recording started, by name
    /// (see `crate::localization::Slam`'s tunable parameters) - the baseline
    /// to build the map again with.
    #[serde(default)]
    pub slam_parameters: BTreeMap<String, f64>,
}

impl MappingSessionInfo {
    /// The record a session starts with.
    pub fn preamble(&self) -> DebugPreamble {
        DebugPreamble {
            topic: MAPPING_SESSION_INFO_TOPIC_NAME.to_string(),
            writer: "WebGui".to_string(),
            payload: serde_json::to_vec(self).expect("MappingSessionInfo serializes to JSON"),
        }
    }

    /// Reads back what [`Self::preamble`] wrote.
    pub fn from_payload(payload: &[u8]) -> Result<Self, String> {
        serde_json::from_slice(payload).map_err(|err| format!("invalid session info: {err}"))
    }
}

/// The file name a session asked for as `requested` gets: a generated one
/// (date and time) if blank, [`MAPPING_SESSION_EXTENSION`] appended if
/// missing. Only a plain name, never a path.
pub fn file_name(requested: &str) -> Result<String, String> {
    let requested = requested.trim();
    if requested.is_empty() {
        let generated = crate::debug_format::DebugFileReader::generated_filename();
        let stem = generated.strip_suffix(".debug").unwrap_or(&generated);
        return Ok(format!("{stem}{MAPPING_SESSION_EXTENSION}"));
    }
    if requested.starts_with('.') || requested.contains(['/', '\\']) {
        return Err(format!(
            "{requested:?} isn't a plain file name - no folders, no leading dot"
        ));
    }
    Ok(if requested.ends_with(MAPPING_SESSION_EXTENSION) {
        requested.to_string()
    } else {
        format!("{requested}{MAPPING_SESSION_EXTENSION}")
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blank_names_are_generated_with_the_extension() {
        let name = file_name(" ").unwrap();
        assert!(name.ends_with(MAPPING_SESSION_EXTENSION));
        assert!(!name.contains(".debug"));
    }

    #[test]
    fn the_extension_is_added_once() {
        assert_eq!(file_name("lap1").unwrap(), "lap1.mapping");
        assert_eq!(file_name("lap1.mapping").unwrap(), "lap1.mapping");
    }

    #[test]
    fn paths_are_refused() {
        for name in ["../escape", "sub/file", "sub\\file", ".hidden"] {
            assert!(file_name(name).is_err(), "{name:?} should be refused");
        }
    }

    #[test]
    fn the_info_round_trips_through_its_preamble() {
        let info = MappingSessionInfo {
            car: Some("tom".to_string()),
            map: Some("maps/Vienna_2026_Quali".to_string()),
            slam_parameters: BTreeMap::from([("resolution_m".to_string(), 0.05)]),
        };
        let preamble = info.preamble();
        assert_eq!(preamble.topic, MAPPING_SESSION_INFO_TOPIC_NAME);
        assert_eq!(
            MappingSessionInfo::from_payload(&preamble.payload),
            Ok(info)
        );
    }

    #[test]
    fn an_older_info_missing_fields_still_reads() {
        assert_eq!(
            MappingSessionInfo::from_payload(b"{}"),
            Ok(MappingSessionInfo::default())
        );
    }
}
