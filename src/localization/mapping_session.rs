//! Mapping sessions: everything [`crate::localization::Slam`] maps from,
//! recorded while mapping, so a map can be built again offline with other
//! parameters. A session is a `.debug` file (see [`crate::debug_format`])
//! holding only [`recorded_topics`], named `<name>.mapping` under the mapping
//! sessions folder, and starting with a [`MappingSessionInfo`] record.
//!
//! Recorded by a [`DebugRecorder`] from [`recorder`], polling at
//! [`MAPPING_SESSION_FREQUENCY_HZ`] - well above every recorded topic's rate,
//! so a write is only missed if the recorder's thread is held up; how many
//! were is in [`crate::DebugStatus::missed_writes`]. [`MappingSession::load`]
//! reads one back, for [`crate::localization::rebuild`] to map it again. See
//! `documentation/slam.md`.

use crate::DebugPreamble;
use crate::DebugRecorder;
use crate::debug_format::DebugFileReader;
use crate::topics::{
    IMU_TOPIC_NAME, ImuReading, LIDAR_SCAN_TOPIC_NAME, LidarScan, ODOMETRY_TOPIC_NAME, Odometry,
    PLACE_AT_START_TOPIC_NAME, PlaceAtStart, SLAM_COMMAND_TOPIC_NAME, START_STATE_TOPIC_NAME,
    SlamCommand, StartState, VehicleGeometry, VehicleStatus, VehicleTopics,
};
use serde::de::DeserializeOwned;
use std::collections::BTreeMap;
use std::path::Path;

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
    /// Where SLAM's frame sat in the world when recording started (see
    /// [`crate::topics::SlamStatus::anchor`]) - `None` in a session recorded
    /// before it was, see [`MappingSession::anchor`].
    #[serde(default)]
    pub anchor: Option<StartState>,
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

/// A recorded value, and when it was written: microseconds since the
/// session started - `0` for one written before it did.
pub type Timed<T> = (u64, T);

/// A mapping session read back into memory, every topic's values in the
/// order they were written.
#[derive(Debug, Clone, Default)]
pub struct MappingSession {
    pub info: MappingSessionInfo,
    /// The latest timestamp in the file, in microseconds.
    pub duration_us: u64,
    pub scans: Vec<Timed<LidarScan>>,
    pub odometry: Vec<Timed<Odometry>>,
    pub imu: Vec<Timed<ImuReading>>,
    pub slam_commands: Vec<Timed<SlamCommand>>,
    /// The true pose - empty unless recorded in simulation.
    pub ground_truth: Vec<Timed<VehicleStatus>>,
    pub start_states: Vec<Timed<StartState>>,
    pub place_at_start: Vec<Timed<PlaceAtStart>>,
    /// The vehicle's size, if written while recording.
    pub geometry: Option<VehicleGeometry>,
}

impl MappingSession {
    /// Reads the session file at `path`. Fails on a file that isn't one, or
    /// a value that doesn't decode as its topic's type (e.g. recorded by a
    /// build whose topics have since changed). A file cut short, e.g. by a
    /// crash, reads up to its last complete record.
    pub fn load(path: &Path) -> Result<Self, String> {
        let file = DebugFileReader::open(path).map_err(|err| format!("{path:?}: {err}"))?;
        let mut session = Self::default();
        let mut info = None;
        for sample in &file.samples {
            let topic = file.topics[usize::from(sample.topic_id)].name.as_str();
            let t = sample.timestamp_us;
            session.duration_us = session.duration_us.max(t);
            let payload = &sample.payload;
            match topic {
                MAPPING_SESSION_INFO_TOPIC_NAME => {
                    info = Some(MappingSessionInfo::from_payload(payload)?)
                }
                LIDAR_SCAN_TOPIC_NAME => session.scans.push((t, decode(topic, payload)?)),
                ODOMETRY_TOPIC_NAME => session.odometry.push((t, decode(topic, payload)?)),
                IMU_TOPIC_NAME => session.imu.push((t, decode(topic, payload)?)),
                SLAM_COMMAND_TOPIC_NAME => session.slam_commands.push((t, decode(topic, payload)?)),
                START_STATE_TOPIC_NAME => session.start_states.push((t, decode(topic, payload)?)),
                PLACE_AT_START_TOPIC_NAME => {
                    session.place_at_start.push((t, decode(topic, payload)?))
                }
                _ if topic == VehicleTopics::ego().vehicle_status() => {
                    session.ground_truth.push((t, decode(topic, payload)?))
                }
                _ if topic == VehicleTopics::ego().vehicle_geometry() => {
                    session.geometry = Some(decode(topic, payload)?)
                }
                // Anything else a later version records.
                _ => {}
            }
        }
        session.info = info.ok_or_else(|| format!("{path:?} isn't a mapping session"))?;
        Ok(session)
    }

    /// Where SLAM's frame sat in the world: as recorded, or - for a session
    /// recorded before that was - a guess from the recorded topics: where
    /// the vehicle was last placed at a pose of its own, else the start
    /// line. The guess is wrong if the start line changed after such a
    /// placement.
    pub fn anchor(&self) -> StartState {
        if let Some(anchor) = self.info.anchor {
            return anchor;
        }
        let start = self.start_states.first().map(|&(_, start)| start);
        let placed = self
            .place_at_start
            .first()
            .and_then(|&(_, place)| place.pose);
        placed.or(start).unwrap_or_default()
    }
}

/// `payload` decoded as `topic`'s type.
fn decode<T: DeserializeOwned>(topic: &str, payload: &[u8]) -> Result<T, String> {
    bincode::serde::decode_from_slice(payload, bincode::config::standard())
        .map(|(value, _)| value)
        .map_err(|err| format!("a {topic} value doesn't decode: {err}"))
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
            anchor: Some(StartState {
                x_m: 1.0,
                ..StartState::default()
            }),
        };
        let preamble = info.preamble();
        assert_eq!(preamble.topic, MAPPING_SESSION_INFO_TOPIC_NAME);
        assert_eq!(
            MappingSessionInfo::from_payload(&preamble.payload),
            Ok(info)
        );
    }

    #[test]
    fn a_recorded_file_loads_back() {
        use crate::debug_format::DebugFileWriter;
        let path = std::env::temp_dir().join(format!(
            "aurorus_mapping_session_tests_{}.mapping",
            std::process::id()
        ));
        let info = MappingSessionInfo {
            car: None,
            map: Some("maps/test".to_string()),
            slam_parameters: BTreeMap::new(),
            anchor: None,
        };
        let mut writer = DebugFileWriter::create(&path, 1000.0).unwrap();
        let preamble = info.preamble();
        let id = writer.topic_id(&preamble.topic, &preamble.writer).unwrap();
        writer.write_sample(id, 0, &preamble.payload).unwrap();
        let odometry = writer
            .topic_id(ODOMETRY_TOPIC_NAME, "DeadReckoning")
            .unwrap();
        let scans = writer
            .topic_id(LIDAR_SCAN_TOPIC_NAME, "SimulatedLidar")
            .unwrap();
        let start = writer
            .topic_id(START_STATE_TOPIC_NAME, "MapServer")
            .unwrap();
        let placed = StartState {
            x_m: 2.0,
            ..StartState::default()
        };
        writer.write_sample(start, 0, &encode(&placed)).unwrap();
        for t in [10, 20, 30] {
            let sample = Odometry {
                x_m: t as f64,
                ..Odometry::default()
            };
            writer.write_sample(odometry, t, &encode(&sample)).unwrap();
        }
        let scan = LidarScan::new(vec![1.0; 4], vec![1.0; 4], 0.1, 10.0, 1.0);
        writer.write_sample(scans, 25, &encode(&scan)).unwrap();
        writer.flush().unwrap();
        drop(writer);

        let session = MappingSession::load(&path).unwrap();
        std::fs::remove_file(&path).unwrap();
        assert_eq!(session.info, info);
        assert_eq!(session.duration_us, 30);
        let odometry_x: Vec<_> = session.odometry.iter().map(|(t, o)| (*t, o.x_m)).collect();
        assert_eq!(odometry_x, vec![(10, 10.0), (20, 20.0), (30, 30.0)]);
        assert_eq!(session.scans.len(), 1);
        assert_eq!(session.scans[0].1.points, scan.points);
        // No anchor recorded: the start line.
        assert_eq!(session.anchor(), placed);
    }

    #[test]
    fn a_file_without_session_info_is_refused() {
        let path = std::env::temp_dir().join(format!(
            "aurorus_mapping_session_tests_plain_{}.debug",
            std::process::id()
        ));
        crate::debug_format::DebugFileWriter::create(&path, 100.0)
            .unwrap()
            .flush()
            .unwrap();
        let loaded = MappingSession::load(&path);
        std::fs::remove_file(&path).unwrap();
        assert!(loaded.unwrap_err().contains("isn't a mapping session"));
    }

    /// `value` encoded the way the recorder does.
    fn encode<T: serde::Serialize>(value: &T) -> Vec<u8> {
        bincode::serde::encode_to_vec(value, bincode::config::standard()).unwrap()
    }

    #[test]
    fn an_older_info_missing_fields_still_reads() {
        assert_eq!(
            MappingSessionInfo::from_payload(b"{}"),
            Ok(MappingSessionInfo::default())
        );
    }
}
