//! The Localization panel's session recording: Start and Stop buttons that
//! record the car's drive - its sensors and odometry, not what any
//! localization made of them - into a localization session (see
//! [`crate::localization::session`]) under the localization sessions
//! folder, for `localization_lab` to localize again with every algorithm.
//! And the endpoints for it.
//!
//! Started and stopped by hand. [`run_controller`] only stops a recording
//! on its own when odometry is reset (the vehicle placed, a map selected):
//! a session holds a single odometry frame. A restart ends it too, as it
//! ends any recording.
//!
//! In simulation, a session is only a fair comparison if the drive didn't
//! depend on the localization running: the status warns when the
//! autonomous algorithm driving takes its pose from localization rather
//! than the ground truth (see [`warning`]).

use crate::localization::pose_source::POSE_LOCALIZATION;
use crate::localization::session::{
    self, EmbeddedMap, LocalizationInfo, SESSION_FREQUENCY_HZ, SessionInfo, SessionKind,
};
use crate::topics::{
    AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME, AutonomousAlgorithmStatus,
    LOCALIZATION_STATUS_TOPIC_NAME, LocalizationStatus, MAP_SELECTION_TOPIC_NAME, MapSelection,
    ODOMETRY_TOPIC_NAME, Odometry, RACE_LINE_TOPIC_NAME, SLAM_STATUS_TOPIC_NAME, SelectedRaceLine,
    SlamState, SlamStatus, VehicleStatus, VehicleTopics,
};
use crate::web::{bad_request, json_response, read_json};
use crate::{Captain, DebugRecorder, DebugState, DebugStatus};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tiny_http::{Request, ResponseBox};

/// How often [`run_controller`] looks at odometry while recording.
const CONTROLLER_PERIOD: Duration = Duration::from_millis(10);

/// The last recording's name and how it went, beyond what its
/// [`DebugRecorder`] reports.
#[derive(Debug, Default)]
struct Control {
    /// Why the last Start was refused or failed - cleared by the next one.
    error: Option<String>,
    /// Why the recording stopped on its own, if it did.
    stopped_because: Option<String>,
    /// Odometry's `reset_count` when the running recording started.
    odometry_reset_count: Option<u64>,
}

/// The session recording's state. Cheap to clone - every clone is the same -
/// so it's kept across restarts, as `WebGui`'s debug recorder is.
#[derive(Debug, Clone)]
pub struct LocalizationSession {
    /// Folder sessions are recorded into.
    root: PathBuf,
    recorder: DebugRecorder,
    control: Arc<Mutex<Control>>,
}

impl LocalizationSession {
    /// Records into `root`.
    pub fn new(root: impl Into<PathBuf>) -> Self {
        Self {
            root: root.into(),
            recorder: session::recorder(SessionKind::Localization),
            control: Arc::default(),
        }
    }

    fn control(&self) -> std::sync::MutexGuard<'_, Control> {
        self.control
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }
}

/// Odometry's `reset_count`, if odometry is published.
fn odometry_reset_count(captain: &Captain) -> Option<u64> {
    captain
        .try_topic::<Odometry>(ODOMETRY_TOPIC_NAME)
        .map(|topic| topic.read().reset_count)
}

/// Stops `session`'s recording when odometry is reset, until `captain`
/// stops executor `id` - see the module docs.
pub fn run_controller(captain: &Captain, id: u16, session: &LocalizationSession) {
    while captain.is_running(id) {
        std::thread::sleep(CONTROLLER_PERIOD);
        if session.recorder.status().state != DebugState::Recording {
            continue;
        }
        let started_in = session.control().odometry_reset_count;
        if started_in.is_some() && odometry_reset_count(captain) != started_in {
            // Fails only if it already stopped, which is what's wanted.
            let _ = session.recorder.stop(captain);
            session.control().stopped_because = Some(
                "Odometry was reset (the vehicle was placed, or a map selected): a session holds one drive.".to_string(),
            );
        }
    }
}

/// Why a session recorded now wouldn't compare localizations fairly, if it
/// wouldn't: in simulation, the autonomous algorithm driving takes its pose
/// from localization (`pose_source`), so the drive depends on the
/// localization running.
fn warning(captain: &Captain) -> Option<String> {
    let simulated = captain
        .try_topic::<VehicleStatus>(&VehicleTopics::ego().vehicle_status())
        .is_some();
    if !simulated {
        return None;
    }
    let (driver, on_localization) = driver(captain);
    on_localization.then(|| {
        format!(
            "{} drives on the localized pose (pose_source), not the ground truth: the drive depends on the localization running, so the session isn't a fair comparison. Set its pose_source to ground truth in the Autonomous Algos panel.",
            driver.unwrap_or_default()
        )
    })
}

/// The label of the autonomous algorithm driving right now - `None` for a
/// human - and whether it drives on localization's pose.
fn driver(captain: &Captain) -> (Option<String>, bool) {
    let status = captain
        .try_topic::<AutonomousAlgorithmStatus>(AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME)
        .map(|topic| topic.read().into_value())
        .unwrap_or_default();
    let Some(active) = status.active else {
        return (None, false);
    };
    let algorithm = status
        .available
        .iter()
        .find(|algorithm| algorithm.name == active);
    let on_localization = algorithm.is_some_and(|algorithm| {
        algorithm.parameters.iter().any(|parameter| {
            parameter.name == "pose_source" && parameter.value == f64::from(POSE_LOCALIZATION)
        })
    });
    let label = algorithm.map_or(active.clone(), |algorithm| algorithm.label.clone());
    (Some(label), on_localization)
}

/// What a session recorded now starts with: the map selected and how
/// everything ran when it started.
fn info(captain: &Captain, map: &std::path::Path, car_name: Option<&str>) -> SessionInfo {
    let slam = captain
        .try_topic::<SlamStatus>(SLAM_STATUS_TOPIC_NAME)
        .map(|topic| topic.read().into_value())
        .unwrap_or_default();
    let localization = captain
        .try_topic::<LocalizationStatus>(LOCALIZATION_STATUS_TOPIC_NAME)
        .map(|topic| topic.read().into_value())
        .unwrap_or_default();
    let race_line = captain
        .try_topic::<SelectedRaceLine>(RACE_LINE_TOPIC_NAME)
        .map(|topic| topic.read().into_value())
        .filter(|line| line.map.as_deref() == Some(map))
        .and_then(|line| line.file);
    let (driver, driven_on_localization) = driver(captain);
    let values = |parameters: &[crate::topics::AlgorithmParameter]| {
        parameters
            .iter()
            .map(|parameter| (parameter.name.clone(), parameter.value))
            .collect()
    };
    SessionInfo {
        kind: SessionKind::Localization,
        car: car_name.map(str::to_string),
        map: Some(map.display().to_string()),
        slam_parameters: values(&slam.parameters),
        anchor: Some(slam.anchor),
        localization: Some(LocalizationInfo {
            algorithm: localization.algorithm,
            particle_filter_parameters: values(&localization.particle_filter_parameters),
            map_to_odom: slam
                .map_to_odom
                .filter(|_| slam.state == SlamState::Localizing && localization.active),
            driver,
            driven_on_localization,
            race_line,
        }),
    }
}

#[derive(serde::Serialize)]
struct Body {
    /// Why the last Start was refused or failed.
    error: Option<String>,
    /// Why the last recording stopped on its own, if it did.
    stopped_because: Option<String>,
    /// The current or last recording.
    recording: DebugStatus,
    /// Size of its file on disk, in bytes.
    file_size_bytes: Option<u64>,
    /// The folder sessions go into.
    folder: String,
    /// See [`warning`].
    warning: Option<String>,
    /// Whether a map is selected - a session needs one.
    map_selected: bool,
}

/// `GET /api/localization_session` - the current or last session, and
/// whether one recorded now would be fair.
pub fn status(captain: &Captain, session: &LocalizationSession) -> ResponseBox {
    let recording = session.recorder.status();
    let file_size_bytes = recording
        .path
        .as_ref()
        .and_then(|path| std::fs::metadata(path).ok())
        .map(|metadata| metadata.len());
    let map_selected = selected_map(captain).is_some();
    let control = session.control();
    json_response(
        &Body {
            error: control.error.clone(),
            stopped_because: control.stopped_because.clone(),
            recording,
            file_size_bytes,
            folder: session.root.display().to_string(),
            warning: warning(captain),
            map_selected,
        },
        200,
    )
}

/// The map folder selected, if any.
fn selected_map(captain: &Captain) -> Option<PathBuf> {
    captain
        .try_topic::<MapSelection>(MAP_SELECTION_TOPIC_NAME)
        .and_then(|topic| topic.read().into_value().path)
}

#[derive(serde::Deserialize)]
struct StartBody {
    /// The session's name - blank for the date and time.
    #[serde(default)]
    name: String,
}

/// `POST /api/localization_session/start` - body `{"name": "..."}` - starts
/// recording a session of the selected map. Refused with no map selected,
/// or while one is recording.
pub fn start(
    request: &mut Request,
    captain: &Captain,
    session: &LocalizationSession,
    car_name: Option<&str>,
) -> ResponseBox {
    let body: StartBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let started = session::file_name(SessionKind::Localization, &body.name).and_then(|file_name| {
        let map = selected_map(captain)
            .ok_or("Select a map first: a session records the drive on one.")?;
        let path = session::unused_path(&session.root, SessionKind::Localization, &file_name);
        let preamble = vec![
            info(captain, &map, car_name).preamble(),
            EmbeddedMap::read(&map)?.preamble(),
        ];
        std::fs::create_dir_all(&session.root)
            .map_err(|err| format!("{:?}: {err}", session.root))?;
        let reset_count = odometry_reset_count(captain);
        session
            .recorder
            .start_with(captain, path, SESSION_FREQUENCY_HZ, preamble)?;
        Ok(reset_count)
    });
    let mut control = session.control();
    match started {
        Ok(reset_count) => {
            *control = Control {
                odometry_reset_count: reset_count,
                ..Control::default()
            };
            json_response(&(), 200)
        }
        Err(err) => {
            control.error = Some(err.clone());
            bad_request(&err)
        }
    }
}

/// `POST /api/localization_session/stop` - stops the recording: its file is
/// complete once the status says it's saved.
pub fn stop(captain: &Captain, session: &LocalizationSession) -> ResponseBox {
    match session.recorder.stop(captain) {
        Ok(()) => json_response(&(), 200),
        Err(err) => bad_request(&err),
    }
}
