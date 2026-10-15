//! The Mapping panel's session recording: a toggle that, while on, records
//! every map built into a mapping session (see
//! [`crate::localization::mapping_session`]) under the mapping sessions
//! folder - and the endpoints for it.
//!
//! Recordings are started and stopped by [`run_controller`], watching
//! `slam_status`, rather than by the endpoints driving SLAM, so every way
//! mapping starts or ends (Play, Clear, a map selected, the vehicle placed, a
//! benchmark) is covered the same way. One session is one map: recording
//! starts when mapping starts on an empty map, and stops on Clear, once Save
//! map succeeds, when odometry is reset, or when the toggle is turned off.
//! Pause, and a save that failed, keep recording.

use crate::localization::mapping_session::{
    self, MAPPING_SESSION_EXTENSION, MAPPING_SESSION_FREQUENCY_HZ, MappingSessionInfo,
};
use crate::topics::{
    MAP_SELECTION_TOPIC_NAME, MapSelection, ODOMETRY_TOPIC_NAME, Odometry, SLAM_STATUS_TOPIC_NAME,
    SlamState, SlamStatus,
};
use crate::web::{bad_request, json_response, read_json};
use crate::{Captain, DebugRecorder, DebugState, DebugStatus};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tiny_http::{Request, ResponseBox};

/// How often [`run_controller`] looks at `slam_status`. Short, so a session
/// misses as little as possible of the start of a map.
const CONTROLLER_PERIOD: Duration = Duration::from_millis(10);

/// What the toggle is set to, and how the last attempt to record went.
#[derive(Debug, Default)]
struct Control {
    armed: bool,
    /// The name every session is recorded under, `""` for the date and time.
    name: String,
    /// Why the last session couldn't be started - cleared by the next one.
    error: Option<String>,
}

/// The session recording's state. Cheap to clone - every clone is the same -
/// so it's kept across restarts, as `WebGui`'s debug recorder is.
#[derive(Debug, Clone)]
pub struct MappingSession {
    /// Folder sessions are recorded into.
    root: PathBuf,
    recorder: DebugRecorder,
    control: Arc<Mutex<Control>>,
}

impl MappingSession {
    /// Records into `root`, toggle off.
    pub fn new(root: impl Into<PathBuf>) -> Self {
        Self {
            root: root.into(),
            recorder: mapping_session::recorder(),
            control: Arc::default(),
        }
    }

    fn control(&self) -> std::sync::MutexGuard<'_, Control> {
        self.control
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }
}

/// What a recording that's running was started against, to tell when to
/// stop it.
struct Recording {
    /// The latest `slam_status` `last_save.requested` already looked at.
    save_requested: Option<u64>,
    /// Odometry's `reset_count` when it started.
    odometry_reset_count: u64,
}

/// Starts and stops `session`'s recordings by watching `slam_status`, until
/// `captain` stops executor `id` - see the module docs. `car_name` is the
/// real car's, when running on one.
pub fn run_controller(
    captain: &Captain,
    id: u16,
    session: &MappingSession,
    car_name: Option<&str>,
) {
    // Only a map seen empty before it was running is a fresh one: arming the
    // toggle halfway through a map waits for the next.
    let mut last_scans: Option<usize> = None;
    let mut recording: Option<Recording> = None;
    let odometry_reset_count = || {
        captain
            .try_topic::<Odometry>(ODOMETRY_TOPIC_NAME)
            .map_or(0, |topic| topic.read().reset_count)
    };

    while captain.is_running(id) {
        std::thread::sleep(CONTROLLER_PERIOD);
        let Some(status_topic) = captain.try_topic::<SlamStatus>(SLAM_STATUS_TOPIC_NAME) else {
            continue;
        };
        let status = status_topic.read().into_value();
        let armed = session.control().armed;
        let save_requested = status.last_save.as_ref().map(|save| save.requested);
        let saved = status
            .last_save
            .as_ref()
            .is_some_and(|save| save.saved_to.is_some());

        // A recording can also end on its own - a restart ends its group.
        if recording.is_some() && session.recorder.status().state != DebugState::Recording {
            recording = None;
        }

        if let Some(running) = &mut recording {
            let new_save = save_requested != running.save_requested;
            running.save_requested = save_requested;
            let ended = !armed
                || status.state == SlamState::Off
                || status.state.is_localization()
                || (new_save && saved)
                || odometry_reset_count() != running.odometry_reset_count;
            if ended {
                // Fails only if it already stopped, which is what's wanted.
                let _ = session.recorder.stop(captain);
                recording = None;
            }
        } else if armed && status.state == SlamState::Running && last_scans == Some(0) {
            recording = start(captain, session, &status, car_name).map(|()| Recording {
                save_requested,
                odometry_reset_count: odometry_reset_count(),
            });
        }
        last_scans = Some(if status.state == SlamState::Off {
            0
        } else {
            status.scans
        });
    }
}

/// Starts recording a session, as of `status`, into a new file - noting why
/// in the control if it can't.
fn start(
    captain: &Captain,
    session: &MappingSession,
    status: &SlamStatus,
    car_name: Option<&str>,
) -> Option<()> {
    let name = session.control().name.clone();
    let started = mapping_session::file_name(&name).and_then(|file_name| {
        let path = unused_path(&session.root, &file_name);
        let info = MappingSessionInfo {
            car: car_name.map(str::to_string),
            map: captain
                .try_topic::<MapSelection>(MAP_SELECTION_TOPIC_NAME)
                .and_then(|topic| topic.read().into_value().path)
                .map(|path| path.display().to_string()),
            slam_parameters: status
                .parameters
                .iter()
                .map(|parameter| (parameter.name.clone(), parameter.value))
                .collect(),
            anchor: Some(status.anchor),
        };
        session.recorder.start_with(
            captain,
            path,
            MAPPING_SESSION_FREQUENCY_HZ,
            vec![info.preamble()],
        )
    });
    let mut control = session.control();
    match started {
        Ok(()) => {
            control.error = None;
            Some(())
        }
        Err(err) => {
            eprintln!("web_gui: couldn't record a mapping session: {err}");
            control.error = Some(err);
            None
        }
    }
}

/// `root/file_name`, or - if that's taken, e.g. by the previous session
/// recorded under the same name - `root/<stem>_2.mapping`, `_3`, ...
fn unused_path(root: &Path, file_name: &str) -> PathBuf {
    let path = root.join(file_name);
    if !path.exists() {
        return path;
    }
    let stem = file_name
        .strip_suffix(MAPPING_SESSION_EXTENSION)
        .unwrap_or(file_name);
    (2..)
        .map(|n| root.join(format!("{stem}_{n}{MAPPING_SESSION_EXTENSION}")))
        .find(|path| !path.exists())
        .expect("some suffix is free")
}

#[derive(serde::Serialize)]
struct Body {
    armed: bool,
    name: String,
    /// Why the last session couldn't be started.
    error: Option<String>,
    /// The current or last recording.
    recording: DebugStatus,
    /// Size of its file on disk, in bytes.
    file_size_bytes: Option<u64>,
    /// The folder sessions go into.
    folder: String,
}

/// `GET /api/mapping_session` - the toggle, and the current or last session.
pub fn status(session: &MappingSession) -> ResponseBox {
    let recording = session.recorder.status();
    let file_size_bytes = recording
        .path
        .as_ref()
        .and_then(|path| std::fs::metadata(path).ok())
        .map(|metadata| metadata.len());
    let control = session.control();
    json_response(
        &Body {
            armed: control.armed,
            name: control.name.clone(),
            error: control.error.clone(),
            recording,
            file_size_bytes,
            folder: session.root.display().to_string(),
        },
        200,
    )
}

#[derive(serde::Deserialize)]
struct ArmBody {
    armed: bool,
    /// The name sessions are recorded under - blank for the date and time.
    #[serde(default)]
    name: String,
}

/// `POST /api/mapping_session` - body `{"armed": true, "name": "..."}` -
/// turns session recording on or off. Off stops a running recording (within
/// one [`CONTROLLER_PERIOD`]); on records from the next map started.
pub fn arm(request: &mut Request, session: &MappingSession) -> ResponseBox {
    let body: ArmBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    if let Err(err) = mapping_session::file_name(&body.name) {
        return bad_request(&err);
    }
    let mut control = session.control();
    control.armed = body.armed;
    control.name = body.name.trim().to_string();
    control.error = None;
    json_response(&(), 200)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_taken_name_gets_a_suffix() {
        let root = std::env::temp_dir().join(format!(
            "aurorus_mapping_session_api_tests_{}",
            std::process::id()
        ));
        std::fs::create_dir_all(&root).unwrap();
        assert_eq!(unused_path(&root, "lap.mapping"), root.join("lap.mapping"));
        std::fs::write(root.join("lap.mapping"), b"").unwrap();
        assert_eq!(
            unused_path(&root, "lap.mapping"),
            root.join("lap_2.mapping")
        );
        std::fs::write(root.join("lap_2.mapping"), b"").unwrap();
        assert_eq!(
            unused_path(&root, "lap.mapping"),
            root.join("lap_3.mapping")
        );
        std::fs::remove_dir_all(&root).unwrap();
    }
}
