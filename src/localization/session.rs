//! Recorded sessions: everything localization and mapping run from,
//! recorded while the vehicle drives, so it can be run again offline with
//! other algorithms or parameters - every run on the very same drive. A
//! session is a `.debug` file (see [`crate::debug_format`]) holding only its
//! [`SessionKind`]'s [`SessionKind::recorded_topics`], named
//! `<name><extension>` under its kind's folder, and starting with a
//! [`SessionInfo`] record and - when a map was selected - an
//! [`EmbeddedMap`]: a copy of that map's files, so the session still runs
//! after the map is regenerated, renamed or deleted.
//!
//! - A **mapping** session (`.mapping`, see `mapping_session_api`) is what
//!   [`crate::localization::Slam`] maps from, for
//!   [`crate::localization::rebuild`] to map it again.
//! - A **localization** session (`.localization`, see
//!   `localization_session_api`) is what localization runs from, to
//!   localize it again offline with every algorithm.
//!
//! Recorded by a [`DebugRecorder`] from [`recorder`], polling at
//! [`SESSION_FREQUENCY_HZ`] - well above every recorded topic's rate, so a
//! write is only missed if the recorder's thread is held up; how many were
//! is in [`crate::DebugStatus::missed_writes`]. [`Session::load`] reads one
//! back. See `documentation/slam.md`.

use crate::DebugPreamble;
use crate::DebugRecorder;
use crate::debug_format::DebugFileReader;
use crate::environment::{INFO_FILE_NAME, MAP_TIFF_FILE_NAME, RACE_LINES_DIR_NAME};
use crate::topics::{
    IMU_TOPIC_NAME, ImuReading, LIDAR_SCAN_TOPIC_NAME, LidarScan, LocalizationAlgorithm,
    ODOMETRY_TOPIC_NAME, Odometry, PLACE_AT_START_TOPIC_NAME, PlaceAtStart,
    SLAM_COMMAND_TOPIC_NAME, START_STATE_TOPIC_NAME, SlamCommand, StartState, VehicleGeometry,
    VehicleStatus, VehicleTopics,
};
use serde::de::DeserializeOwned;
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// How often a session's recorder polls its topics, in Hz: faster than any
/// of them is written (odometry, the fastest, at ~200 Hz).
pub const SESSION_FREQUENCY_HZ: f64 = 1000.0;
/// Name of the record holding the session's [`SessionInfo`].
pub const SESSION_INFO_TOPIC_NAME: &str = "session_info";
/// What [`SESSION_INFO_TOPIC_NAME`] was named before sessions had kinds -
/// still read, as a mapping session's.
const LEGACY_INFO_TOPIC_NAME: &str = "mapping_session_info";
/// Name of the record holding the session's [`EmbeddedMap`].
pub const SESSION_MAP_TOPIC_NAME: &str = "session_map";

/// What a session was recorded for - which decides its extension, its
/// folder's default name, its topics and its recording group.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SessionKind {
    /// Recorded while mapping, one session per map.
    #[default]
    Mapping,
    /// Recorded while driving on a known map, started and stopped by hand.
    Localization,
}

impl SessionKind {
    /// Extension every session of this kind's file gets.
    pub fn extension(self) -> &'static str {
        match self {
            Self::Mapping => ".mapping",
            Self::Localization => ".localization",
        }
    }

    /// The group this kind's recording runs as - not [`crate::DEBUG_GROUP`],
    /// and different for each kind, so they can all run at once.
    pub fn group(self) -> &'static str {
        match self {
            Self::Mapping => "mapping_session",
            Self::Localization => "localization_session",
        }
    }

    /// The topics a session of this kind records: localization's and
    /// SLAM's inputs (`lidar_scan`, `odometry`), what odometry is integrated
    /// from (`imu`, to integrate it again offline), where the map's frame
    /// sits in the world (`start_state`, `place_at_start`), the vehicle's
    /// size, and the true pose (`vehicle_status`, only written in
    /// simulation), to score against - plus, for mapping, when it was
    /// played, paused and cleared (`slam_command`).
    pub fn recorded_topics(self) -> Vec<String> {
        let ego = VehicleTopics::ego();
        let mut topics = vec![
            LIDAR_SCAN_TOPIC_NAME.to_string(),
            ODOMETRY_TOPIC_NAME.to_string(),
            IMU_TOPIC_NAME.to_string(),
            START_STATE_TOPIC_NAME.to_string(),
            PLACE_AT_START_TOPIC_NAME.to_string(),
            ego.vehicle_status(),
            ego.vehicle_geometry(),
        ];
        if self == Self::Mapping {
            topics.push(SLAM_COMMAND_TOPIC_NAME.to_string());
        }
        topics
    }
}

/// A recorder of `kind`'s [`SessionKind::recorded_topics`], as its
/// [`SessionKind::group`].
pub fn recorder(kind: SessionKind) -> DebugRecorder {
    DebugRecorder::for_topics(kind.group(), kind.recorded_topics())
}

/// What a session was recorded with, written once at its start. Stored as
/// JSON rather than bincode, so a session stays readable as fields are
/// added.
#[derive(Debug, Clone, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct SessionInfo {
    /// Missing from a session recorded before there were kinds: a mapping
    /// one.
    #[serde(default)]
    pub kind: SessionKind,
    /// The real car's name, or `None` in simulation.
    #[serde(default)]
    pub car: Option<String>,
    /// The map folder selected when recording started - in simulation, the
    /// true map the session can be compared against. Its files are in the
    /// session's [`EmbeddedMap`].
    #[serde(default)]
    pub map: Option<String>,
    /// The parameter values SLAM ran with when recording started, by name
    /// (see `crate::localization::Slam`'s tunable parameters) - the baseline
    /// to build the map again with, and to scan match with.
    #[serde(default)]
    pub slam_parameters: BTreeMap<String, f64>,
    /// Where SLAM's frame sat in the world when recording started (see
    /// [`crate::topics::SlamStatus::anchor`]) - `None` in a session recorded
    /// before it was, see [`Session::anchor`].
    #[serde(default)]
    pub anchor: Option<StartState>,
    /// What only a localization session records.
    #[serde(default)]
    pub localization: Option<LocalizationInfo>,
}

/// What a localization session was recorded with, beyond [`SessionInfo`].
#[derive(Debug, Clone, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct LocalizationInfo {
    /// The algorithm localization ran (or would have) with.
    #[serde(default)]
    pub algorithm: LocalizationAlgorithm,
    /// The particle filter's parameter values, by name (see
    /// `crate::localization::particle_filter_tunable_parameters`).
    #[serde(default)]
    pub particle_filter_parameters: BTreeMap<String, f64>,
    /// Where live localization had odometry's frame on the map when
    /// recording started, `[x_m, y_m, heading_rad]` - `None` if it wasn't
    /// localizing.
    #[serde(default)]
    pub map_to_odom: Option<[f64; 3]>,
    /// The autonomous algorithm driving when recording started - `None` for
    /// a human driving.
    #[serde(default)]
    pub driver: Option<String>,
    /// Whether that algorithm drove on localization's pose rather than the
    /// simulator's ground truth: the drive then depends on the localization
    /// that ran, which a fair comparison of localizations shouldn't.
    #[serde(default)]
    pub driven_on_localization: bool,
    /// The race line selected when recording started - a file of the map's
    /// `race_lines/` - to count laps along.
    #[serde(default)]
    pub race_line: Option<String>,
}

impl SessionInfo {
    /// The record a session starts with.
    pub fn preamble(&self) -> DebugPreamble {
        DebugPreamble {
            topic: SESSION_INFO_TOPIC_NAME.to_string(),
            writer: "WebGui".to_string(),
            payload: serde_json::to_vec(self).expect("SessionInfo serializes to JSON"),
        }
    }

    /// Reads back what [`Self::preamble`] wrote.
    pub fn from_payload(payload: &[u8]) -> Result<Self, String> {
        serde_json::from_slice(payload).map_err(|err| format!("invalid session info: {err}"))
    }
}

/// One file of an [`EmbeddedMap`].
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct EmbeddedFile {
    /// Its path inside the map folder, `/`-separated.
    pub path: String,
    pub bytes: Vec<u8>,
}

/// A copy of a map folder's files - its `info.json`, `map.tiff` and every
/// race line - carried inside a session.
#[derive(Debug, Clone, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct EmbeddedMap {
    pub files: Vec<EmbeddedFile>,
}

impl EmbeddedMap {
    /// Reads the map in `folder`: its `info.json` and `map.tiff`, which it
    /// must have, and every `.csv` of its `race_lines/`.
    pub fn read(folder: &Path) -> Result<Self, String> {
        let read = |path: &Path| std::fs::read(path).map_err(|err| format!("{path:?}: {err}"));
        let mut files = Vec::new();
        for name in [INFO_FILE_NAME, MAP_TIFF_FILE_NAME] {
            files.push(EmbeddedFile {
                path: name.to_string(),
                bytes: read(&folder.join(name))?,
            });
        }
        let race_lines = folder.join(RACE_LINES_DIR_NAME);
        if let Ok(entries) = std::fs::read_dir(&race_lines) {
            let mut names: Vec<String> = entries
                .filter_map(Result::ok)
                .map(|entry| entry.file_name().to_string_lossy().into_owned())
                .filter(|name| name.ends_with(".csv"))
                .collect();
            names.sort();
            for name in names {
                files.push(EmbeddedFile {
                    path: format!("{RACE_LINES_DIR_NAME}/{name}"),
                    bytes: read(&race_lines.join(&name))?,
                });
            }
        }
        Ok(Self { files })
    }

    /// The record that carries it, after the [`SessionInfo`].
    pub fn preamble(&self) -> DebugPreamble {
        DebugPreamble {
            topic: SESSION_MAP_TOPIC_NAME.to_string(),
            writer: "WebGui".to_string(),
            payload: bincode::serde::encode_to_vec(self, bincode::config::standard())
                .expect("EmbeddedMap serializes"),
        }
    }

    /// Reads back what [`Self::preamble`] wrote.
    pub fn from_payload(payload: &[u8]) -> Result<Self, String> {
        decode(SESSION_MAP_TOPIC_NAME, payload)
    }

    /// A hex SHA-256 of every file's path and bytes - two maps with the same
    /// one are the same map.
    pub fn sha256(&self) -> String {
        let mut hasher = Sha256::new();
        for file in &self.files {
            hasher.update(file.path.as_bytes());
            hasher.update([0]);
            hasher.update((file.bytes.len() as u64).to_le_bytes());
            hasher.update(&file.bytes);
        }
        hasher
            .finalize()
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect()
    }

    /// The race line files it holds - names in `race_lines/`.
    pub fn race_lines(&self) -> Vec<String> {
        self.files
            .iter()
            .filter_map(|file| {
                file.path
                    .strip_prefix(RACE_LINES_DIR_NAME)
                    .and_then(|rest| rest.strip_prefix('/'))
                    .map(str::to_string)
            })
            .collect()
    }

    /// Writes its files into `folder` (created if needed), as a map folder
    /// [`crate::environment::Map::load`] reads. A path leaving the folder is
    /// refused.
    pub fn write_to(&self, folder: &Path) -> Result<(), String> {
        for file in &self.files {
            let relative = Path::new(&file.path);
            if relative.is_absolute()
                || relative
                    .components()
                    .any(|part| !matches!(part, std::path::Component::Normal(_)))
            {
                return Err(format!("{:?} isn't a path inside a map folder", file.path));
            }
            let path = folder.join(relative);
            if let Some(parent) = path.parent() {
                std::fs::create_dir_all(parent).map_err(|err| format!("{parent:?}: {err}"))?;
            }
            std::fs::write(&path, &file.bytes).map_err(|err| format!("{path:?}: {err}"))?;
        }
        Ok(())
    }

    /// A folder holding its files, under the system's temporary folder and
    /// named by [`Self::sha256`] - written the first time, reused after.
    pub fn extracted(&self) -> Result<PathBuf, String> {
        let folder = std::env::temp_dir()
            .join("aurorus_session_maps")
            .join(self.sha256());
        if !folder.join(INFO_FILE_NAME).exists() {
            // Written aside and renamed, so a folder that exists is complete.
            let partial = folder.with_extension(format!("partial{}", std::process::id()));
            let _ = std::fs::remove_dir_all(&partial);
            self.write_to(&partial)?;
            if std::fs::rename(&partial, &folder).is_err() {
                // Another process got there first.
                let _ = std::fs::remove_dir_all(&partial);
            }
        }
        Ok(folder)
    }
}

/// A recorded value, and when it was written: microseconds since the
/// session started - `0` for one written before it did.
pub type Timed<T> = (u64, T);

/// A session read back into memory, every topic's values in the order they
/// were written.
#[derive(Debug, Clone, Default)]
pub struct Session {
    pub info: SessionInfo,
    /// The map selected when recording started, if any was and the session
    /// embeds it.
    pub map: Option<EmbeddedMap>,
    /// The latest timestamp in the file, in microseconds.
    pub duration_us: u64,
    pub scans: Vec<Timed<LidarScan>>,
    pub odometry: Vec<Timed<Odometry>>,
    pub imu: Vec<Timed<ImuReading>>,
    /// Empty but in a mapping session.
    pub slam_commands: Vec<Timed<SlamCommand>>,
    /// The true pose - empty unless recorded in simulation.
    pub ground_truth: Vec<Timed<VehicleStatus>>,
    pub start_states: Vec<Timed<StartState>>,
    pub place_at_start: Vec<Timed<PlaceAtStart>>,
    /// The vehicle's size, if written while recording.
    pub geometry: Option<VehicleGeometry>,
}

impl Session {
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
                SESSION_INFO_TOPIC_NAME | LEGACY_INFO_TOPIC_NAME => {
                    info = Some(SessionInfo::from_payload(payload)?)
                }
                SESSION_MAP_TOPIC_NAME => session.map = Some(EmbeddedMap::from_payload(payload)?),
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
        session.info = info.ok_or_else(|| format!("{path:?} isn't a recorded session"))?;
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

    /// A folder holding the map the session was recorded on: its embedded
    /// copy (see [`EmbeddedMap::extracted`]), or - for a session recorded
    /// before maps were embedded - the folder it names, if it's still
    /// there. `None` if there's neither.
    pub fn map_folder(&self) -> Result<Option<PathBuf>, String> {
        if let Some(map) = &self.map {
            return map.extracted().map(Some);
        }
        Ok(self
            .info
            .map
            .as_ref()
            .map(PathBuf::from)
            .filter(|folder| folder.join(INFO_FILE_NAME).exists()))
    }
}

/// `payload` decoded as `topic`'s type.
fn decode<T: DeserializeOwned>(topic: &str, payload: &[u8]) -> Result<T, String> {
    bincode::serde::decode_from_slice(payload, bincode::config::standard())
        .map(|(value, _)| value)
        .map_err(|err| format!("a {topic} value doesn't decode: {err}"))
}

/// The file name a `kind` session asked for as `requested` gets: a
/// generated one (date and time) if blank, the kind's extension appended if
/// missing. Only a plain name, never a path.
pub fn file_name(kind: SessionKind, requested: &str) -> Result<String, String> {
    let extension = kind.extension();
    let requested = requested.trim();
    if requested.is_empty() {
        let generated = crate::debug_format::DebugFileReader::generated_filename();
        let stem = generated.strip_suffix(".debug").unwrap_or(&generated);
        return Ok(format!("{stem}{extension}"));
    }
    if requested.starts_with('.') || requested.contains(['/', '\\']) {
        return Err(format!(
            "{requested:?} isn't a plain file name - no folders, no leading dot"
        ));
    }
    Ok(if requested.ends_with(extension) {
        requested.to_string()
    } else {
        format!("{requested}{extension}")
    })
}

/// `root/file_name`, or - if that's taken, e.g. by the previous session
/// recorded under the same name - `root/<stem>_2<extension>`, `_3`, ...
pub fn unused_path(root: &Path, kind: SessionKind, file_name: &str) -> PathBuf {
    let path = root.join(file_name);
    if !path.exists() {
        return path;
    }
    let extension = kind.extension();
    let stem = file_name.strip_suffix(extension).unwrap_or(file_name);
    (2..)
        .map(|n| root.join(format!("{stem}_{n}{extension}")))
        .find(|path| !path.exists())
        .expect("some suffix is free")
}

/// The session files of `kind` in `root`, newest first, with their sizes.
pub fn list(root: &Path, kind: SessionKind) -> Vec<(String, u64)> {
    let Ok(entries) = std::fs::read_dir(root) else {
        return Vec::new();
    };
    let mut files: Vec<_> = entries
        .filter_map(Result::ok)
        .filter_map(|entry| {
            let name = entry.file_name().to_string_lossy().into_owned();
            let metadata = entry.metadata().ok()?;
            (name.ends_with(kind.extension()) && metadata.is_file()).then(|| {
                let modified = metadata.modified().ok();
                (name, metadata.len(), modified)
            })
        })
        .collect();
    files.sort_by(|a, b| b.2.cmp(&a.2).then_with(|| a.0.cmp(&b.0)));
    files
        .into_iter()
        .map(|(name, size, _)| (name, size))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A fresh, empty folder under the system's temporary folder.
    fn temp_folder(name: &str) -> PathBuf {
        let folder = std::env::temp_dir().join(format!(
            "aurorus_session_tests_{name}_{}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&folder);
        std::fs::create_dir_all(&folder).unwrap();
        folder
    }

    #[test]
    fn blank_names_are_generated_with_the_extension() {
        for kind in [SessionKind::Mapping, SessionKind::Localization] {
            let name = file_name(kind, " ").unwrap();
            assert!(name.ends_with(kind.extension()));
            assert!(!name.contains(".debug"));
        }
    }

    #[test]
    fn the_extension_is_added_once() {
        assert_eq!(
            file_name(SessionKind::Mapping, "lap1").unwrap(),
            "lap1.mapping"
        );
        assert_eq!(
            file_name(SessionKind::Mapping, "lap1.mapping").unwrap(),
            "lap1.mapping"
        );
        assert_eq!(
            file_name(SessionKind::Localization, "lap1").unwrap(),
            "lap1.localization"
        );
    }

    #[test]
    fn paths_are_refused() {
        for name in ["../escape", "sub/file", "sub\\file", ".hidden"] {
            assert!(
                file_name(SessionKind::Mapping, name).is_err(),
                "{name:?} should be refused"
            );
        }
    }

    #[test]
    fn a_taken_name_gets_a_suffix() {
        let root = temp_folder("unused_path");
        let kind = SessionKind::Localization;
        assert_eq!(
            unused_path(&root, kind, "lap.localization"),
            root.join("lap.localization")
        );
        std::fs::write(root.join("lap.localization"), b"").unwrap();
        assert_eq!(
            unused_path(&root, kind, "lap.localization"),
            root.join("lap_2.localization")
        );
        std::fs::write(root.join("lap_2.localization"), b"").unwrap();
        assert_eq!(
            unused_path(&root, kind, "lap.localization"),
            root.join("lap_3.localization")
        );
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn only_slam_commands_tell_the_kinds_topics_apart() {
        let mapping = SessionKind::Mapping.recorded_topics();
        let localization = SessionKind::Localization.recorded_topics();
        assert!(mapping.contains(&SLAM_COMMAND_TOPIC_NAME.to_string()));
        assert!(!localization.contains(&SLAM_COMMAND_TOPIC_NAME.to_string()));
        assert_eq!(mapping.len(), localization.len() + 1);
        assert_ne!(
            SessionKind::Mapping.group(),
            SessionKind::Localization.group()
        );
    }

    #[test]
    fn the_info_round_trips_through_its_preamble() {
        let info = SessionInfo {
            kind: SessionKind::Localization,
            car: Some("tom".to_string()),
            map: Some("maps/Vienna_2026_Quali".to_string()),
            slam_parameters: BTreeMap::from([("resolution_m".to_string(), 0.05)]),
            anchor: Some(StartState {
                x_m: 1.0,
                ..StartState::default()
            }),
            localization: Some(LocalizationInfo {
                algorithm: LocalizationAlgorithm::ParticleFilter,
                map_to_odom: Some([1.0, 2.0, 0.5]),
                driver: Some("pure_pursuit".to_string()),
                driven_on_localization: true,
                race_line: Some("centerline.csv".to_string()),
                ..LocalizationInfo::default()
            }),
        };
        let preamble = info.preamble();
        assert_eq!(preamble.topic, SESSION_INFO_TOPIC_NAME);
        assert_eq!(SessionInfo::from_payload(&preamble.payload), Ok(info));
    }

    #[test]
    fn an_older_info_missing_fields_still_reads_as_a_mapping_one() {
        assert_eq!(SessionInfo::from_payload(b"{}"), Ok(SessionInfo::default()));
        assert_eq!(SessionInfo::default().kind, SessionKind::Mapping);
    }

    /// A map folder with an info, a raster and two race lines - not a real
    /// map, only bytes to carry.
    fn fake_map(name: &str) -> PathBuf {
        let folder = temp_folder(name);
        std::fs::write(folder.join(INFO_FILE_NAME), b"{\"info\": 1}").unwrap();
        std::fs::write(folder.join(MAP_TIFF_FILE_NAME), [1u8, 2, 3, 4]).unwrap();
        std::fs::write(folder.join("map.orig.tiff"), [9u8]).unwrap();
        std::fs::create_dir_all(folder.join(RACE_LINES_DIR_NAME)).unwrap();
        std::fs::write(folder.join("race_lines/centerline.csv"), b"x,y\n").unwrap();
        std::fs::write(folder.join("race_lines/min_time.csv"), b"x,y,v\n").unwrap();
        std::fs::write(folder.join("race_lines/notes.txt"), b"not a line").unwrap();
        folder
    }

    #[test]
    fn a_map_is_embedded_and_extracted_back() {
        let folder = fake_map("embed");
        let map = EmbeddedMap::read(&folder).unwrap();
        let paths: Vec<_> = map.files.iter().map(|file| file.path.as_str()).collect();
        assert_eq!(
            paths,
            [
                "info.json",
                "map.tiff",
                "race_lines/centerline.csv",
                "race_lines/min_time.csv"
            ]
        );
        assert_eq!(map.race_lines(), ["centerline.csv", "min_time.csv"]);
        assert_eq!(
            EmbeddedMap::from_payload(&map.preamble().payload).unwrap(),
            map
        );

        let out = folder.join("out");
        map.write_to(&out).unwrap();
        for path in paths {
            assert_eq!(
                std::fs::read(out.join(path)).unwrap(),
                std::fs::read(folder.join(path)).unwrap()
            );
        }
        let extracted = map.extracted().unwrap();
        assert_eq!(extracted, map.extracted().unwrap());
        assert!(extracted.join("race_lines/min_time.csv").exists());
        std::fs::remove_dir_all(&folder).unwrap();
    }

    #[test]
    fn a_path_leaving_the_map_folder_is_refused() {
        let map = EmbeddedMap {
            files: vec![EmbeddedFile {
                path: "../escape".to_string(),
                bytes: Vec::new(),
            }],
        };
        let folder = temp_folder("escape");
        assert!(map.write_to(&folder.join("map")).is_err());
        std::fs::remove_dir_all(&folder).unwrap();
    }

    #[test]
    fn a_map_without_its_raster_isnt_embedded() {
        let folder = temp_folder("no_raster");
        std::fs::write(folder.join(INFO_FILE_NAME), b"{}").unwrap();
        assert!(EmbeddedMap::read(&folder).is_err());
        std::fs::remove_dir_all(&folder).unwrap();
    }

    /// `value` encoded the way the recorder does.
    fn encode<T: serde::Serialize>(value: &T) -> Vec<u8> {
        bincode::serde::encode_to_vec(value, bincode::config::standard()).unwrap()
    }

    #[test]
    fn a_recorded_file_loads_back() {
        use crate::debug_format::DebugFileWriter;
        let folder = fake_map("load");
        let path = folder.join("session.localization");
        let info = SessionInfo {
            kind: SessionKind::Localization,
            map: Some("maps/test".to_string()),
            ..SessionInfo::default()
        };
        let map = EmbeddedMap::read(&folder).unwrap();
        let mut writer = DebugFileWriter::create(&path, 1000.0).unwrap();
        for preamble in [info.preamble(), map.preamble()] {
            let id = writer.topic_id(&preamble.topic, &preamble.writer).unwrap();
            writer.write_sample(id, 0, &preamble.payload).unwrap();
        }
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

        let session = Session::load(&path).unwrap();
        assert_eq!(session.info, info);
        assert_eq!(session.map.as_ref(), Some(&map));
        assert_eq!(
            session.map_folder().unwrap(),
            Some(map.extracted().unwrap())
        );
        assert_eq!(session.duration_us, 30);
        let odometry_x: Vec<_> = session.odometry.iter().map(|(t, o)| (*t, o.x_m)).collect();
        assert_eq!(odometry_x, vec![(10, 10.0), (20, 20.0), (30, 30.0)]);
        assert_eq!(session.scans.len(), 1);
        assert_eq!(session.scans[0].1.points, scan.points);
        // No anchor recorded: the start line.
        assert_eq!(session.anchor(), placed);
        assert_eq!(list(&folder, SessionKind::Localization).len(), 1);
        assert!(list(&folder, SessionKind::Mapping).is_empty());
        std::fs::remove_dir_all(&folder).unwrap();
    }

    #[test]
    fn a_session_recorded_before_kinds_still_loads() {
        use crate::debug_format::DebugFileWriter;
        let folder = temp_folder("legacy");
        let path = folder.join("old.mapping");
        let mut writer = DebugFileWriter::create(&path, 1000.0).unwrap();
        let id = writer.topic_id(LEGACY_INFO_TOPIC_NAME, "WebGui").unwrap();
        writer
            .write_sample(id, 0, b"{\"car\": null, \"map\": \"maps/gone\"}")
            .unwrap();
        writer.flush().unwrap();
        drop(writer);
        let session = Session::load(&path).unwrap();
        assert_eq!(session.info.kind, SessionKind::Mapping);
        assert_eq!(session.info.map.as_deref(), Some("maps/gone"));
        // The map it names is gone, and none is embedded.
        assert_eq!(session.map_folder(), Ok(None));
        std::fs::remove_dir_all(&folder).unwrap();
    }

    #[test]
    fn a_file_without_session_info_is_refused() {
        let folder = temp_folder("plain");
        let path = folder.join("plain.debug");
        crate::debug_format::DebugFileWriter::create(&path, 100.0)
            .unwrap()
            .flush()
            .unwrap();
        let loaded = Session::load(&path);
        std::fs::remove_dir_all(&folder).unwrap();
        assert!(loaded.unwrap_err().contains("isn't a recorded session"));
    }
}
