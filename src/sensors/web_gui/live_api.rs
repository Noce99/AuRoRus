//! The live, topic-backed control API: which map, vehicle model, and
//! autonomous algorithm are currently selected (the `map`,
//! `vehicle_model_status`, and `autonomous_algorithm_status` topics), and the
//! write endpoints a driver uses to steer the vehicle, pick its map, model,
//! and autonomous algorithm, tune that algorithm, place it at the start
//! line, and drive SLAM (`map_selection`, `human_vesc_command`,
//! `vehicle_model_selection`, `autonomous_algorithm_selection`,
//! `autonomous_parameters`, `place_at_start`, `slam_command`) - as
//! opposed to [`super::maps_api`], which lists/generates map folders on
//! disk, and [`super::draw_api`], which serves what's drawn on the map.

use super::WebGuiConfig;
use super::maps_api::safe_map_folder;
use crate::autonomous_control;
use crate::topics::{
    AUTONOMOUS_ALGORITHM_SELECTION_TOPIC_NAME, AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME,
    AUTONOMOUS_PARAMETERS_TOPIC_NAME, AlgorithmParameter, AutonomousAlgorithmSelection,
    AutonomousAlgorithmStatus, AutonomousParameters, HUMAN_VESC_COMMAND_TOPIC_NAME,
    MAP_SELECTION_TOPIC_NAME, MAP_TOPIC_NAME, MapSelection, PLACE_AT_START_TOPIC_NAME,
    PlaceAtStart, SLAM_COMMAND_TOPIC_NAME, SLAM_STATUS_TOPIC_NAME, SelectedMap, SlamCommand,
    SlamState, SlamStatus, VEHICLE_MODEL_SELECTION_TOPIC_NAME, VEHICLE_MODEL_STATUS_TOPIC_NAME,
    VehicleModelKind, VehicleModelSelection, VehicleModelStatus, VescCommand,
};
use crate::web::{bad_request, error_response, json_response, read_json};
use crate::{Captain, WriteMeta};
use std::path::Path;
use std::sync::Mutex;
use tiny_http::{Request, ResponseBox};

/// The subset of [`WebGuiConfig`] the frontend needs, as served by
/// [`config`].
#[derive(serde::Serialize)]
struct FrontendConfig {
    human_max_speed_mps: f64,
    human_max_steering_rad: f64,
}

/// `GET /api/config` - frontend-facing config values (the WASD human
/// control limits), so the UI and server never drift apart.
pub fn config(config: &WebGuiConfig) -> ResponseBox {
    json_response(
        &FrontendConfig {
            human_max_speed_mps: config.human_max_speed_mps,
            human_max_steering_rad: config.human_max_steering_rad,
        },
        200,
    )
}

/// The envelope every topic-reading `GET` endpoint wraps its body in: the
/// topic's value plus the [`WriteMeta`] the topic stamped it with, so the
/// frontend can tell how fresh it is. `age_ms` is measured server-side (on the
/// monotonic clock) at response time; it and `written_at_unix_us` are
/// `null`/`0` while the topic still holds its unwritten seed
/// (`write_count == 0`).
#[derive(serde::Serialize)]
struct StampedBody<'a, T> {
    value: &'a T,
    written_at_unix_us: u64,
    age_ms: Option<f64>,
    write_count: u64,
}

/// Serializes `value` (derived from a topic read) wrapped in a
/// [`StampedBody`] carrying that read's `meta`.
fn stamped_json<T: serde::Serialize>(value: &T, meta: WriteMeta) -> ResponseBox {
    json_response(
        &StampedBody {
            value,
            written_at_unix_us: meta.written_at_unix_us,
            age_ms: meta
                .written_at
                .map(|written_at| written_at.elapsed().as_secs_f64() * 1000.0),
            write_count: meta.write_count,
        },
        200,
    )
}

/// `GET /api/map` - the currently selected map's name and dimensions (not
/// its pixels - those are drawn from `MapServer`'s drawing topic), read
/// from the `map` topic. `name` is
/// derived from the topic's folder path, or `null` if no map is selected.
/// Served as a [`StampedBody`].
#[derive(serde::Serialize)]
struct LiveMap {
    name: Option<String>,
    width_px: u32,
    height_px: u32,
}

pub fn map(captain: &Captain) -> ResponseBox {
    let selected = captain.topic::<SelectedMap>(MAP_TOPIC_NAME).read();
    let name = selected
        .path
        .as_ref()
        .and_then(|p| p.file_name())
        .and_then(|n| n.to_str())
        .map(str::to_string);
    stamped_json(
        &LiveMap {
            name,
            width_px: selected.width_px,
            height_px: selected.height_px,
        },
        selected.meta,
    )
}

#[derive(serde::Deserialize)]
struct SelectMapBody {
    name: Option<String>,
}

/// `POST /api/map_selection` - body `{"name": "..."}` (or `{"name": null}`
/// to deselect) - writes the wanted map folder to `map_selection`, for
/// [`crate::sensors::MapServer`] to pick up. Also turns SLAM off: a map
/// built on the old track means nothing on the new one.
pub fn select_map(
    request: &mut Request,
    captain: &Captain,
    writer_id: u8,
    maps_root: &Path,
) -> ResponseBox {
    let body: SelectMapBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };

    let path = match &body.name {
        None => None,
        Some(name) => match safe_map_folder(name, maps_root) {
            Some(path) => Some(path),
            None => {
                return bad_request(
                    "invalid name: must not be empty or contain '/', '\\', or '..'",
                );
            }
        },
    };

    captain
        .topic::<MapSelection>(MAP_SELECTION_TOPIC_NAME)
        .write(writer_id, MapSelection { path })
        .expect("lost writer authorization for the map_selection topic");
    write_slam_command(captain, writer_id, SlamState::Off);
    json_response(&(), 200)
}

#[derive(serde::Deserialize)]
struct HumanVescCommandBody {
    servo_position_rad: f64,
    speed_mps: f64,
}

/// `POST /api/human_vesc_command` - body `{"servo_position_rad": ...,
/// "speed_mps": ...}` - writes a freshly timestamped [`VescCommand`] to
/// `human_vesc_command`, e.g. from `web_gui`'s WASD control.
pub fn human_vesc_command(request: &mut Request, captain: &Captain, writer_id: u8) -> ResponseBox {
    let body: HumanVescCommandBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };

    captain
        .topic::<VescCommand>(HUMAN_VESC_COMMAND_TOPIC_NAME)
        .write(
            writer_id,
            VescCommand::new(body.servo_position_rad, body.speed_mps),
        )
        .expect("lost writer authorization for the human_vesc_command topic");
    json_response(&(), 200)
}

/// One selectable vehicle model kind, as listed by [`vehicle_models`].
#[derive(serde::Serialize)]
struct VehicleModelOption {
    kind: &'static str,
    label: &'static str,
    description: &'static str,
}

/// `GET /api/vehicle_models` - every selectable vehicle model kind, for a
/// picker in the UI.
pub fn vehicle_models() -> ResponseBox {
    let options: Vec<VehicleModelOption> = VehicleModelKind::ALL
        .iter()
        .map(|(_, kind, label, description)| VehicleModelOption {
            kind,
            label,
            description,
        })
        .collect();
    json_response(&options, 200)
}

#[derive(serde::Serialize)]
struct LiveVehicleModel {
    kind: &'static str,
}

/// `GET /api/vehicle_model` - the vehicle model kind currently running, read
/// from the `vehicle_model_status` topic, as a [`StampedBody`].
pub fn vehicle_model(captain: &Captain) -> ResponseBox {
    let status = captain
        .topic::<VehicleModelStatus>(VEHICLE_MODEL_STATUS_TOPIC_NAME)
        .read();
    stamped_json(
        &LiveVehicleModel {
            kind: status.kind.api_str(),
        },
        status.meta,
    )
}

#[derive(serde::Deserialize)]
struct SelectVehicleModelBody {
    kind: String,
}

/// `POST /api/vehicle_model_selection` - body `{"kind": "..."}` - writes the
/// wanted vehicle model kind to `vehicle_model_selection`, for
/// [`crate::actuators::SimulatedVehicle`] to pick up.
pub fn select_vehicle_model(
    request: &mut Request,
    captain: &Captain,
    writer_id: u8,
) -> ResponseBox {
    let body: SelectVehicleModelBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };

    let Some(kind) = VehicleModelKind::from_api_str(&body.kind) else {
        return bad_request(&format!("unknown vehicle model kind: {:?}", body.kind));
    };

    captain
        .topic::<VehicleModelSelection>(VEHICLE_MODEL_SELECTION_TOPIC_NAME)
        .write(writer_id, VehicleModelSelection { kind })
        .expect("lost writer authorization for the vehicle_model_selection topic");
    json_response(&(), 200)
}

/// `GET /api/autonomous_algorithms` - every autonomous algorithm found, the
/// one in control, and whether its command is fresh, read from the
/// `autonomous_algorithm_status` topic, as a [`StampedBody`]. Empty if
/// nothing publishes that topic.
pub fn autonomous_algorithms(captain: &Captain) -> ResponseBox {
    match captain.try_topic::<AutonomousAlgorithmStatus>(AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME) {
        Some(topic) => {
            let status = topic.read();
            stamped_json(&status.value, status.meta)
        }
        None => stamped_json(&AutonomousAlgorithmStatus::default(), WriteMeta::default()),
    }
}

#[derive(serde::Deserialize)]
struct SelectAutonomousAlgorithmBody {
    name: Option<String>,
    running: bool,
}

/// `POST /api/autonomous_algorithm_selection` - body `{"name": "...",
/// "running": true}` (`false` to pause it; `"name": null` for none) - writes
/// the wanted algorithm, and whether it should drive, to
/// `autonomous_algorithm_selection`, for
/// [`crate::autonomous_control::AutonomousControlsHandler`] to pick up.
/// Rejects a name the handler hasn't listed as available, and running with
/// no algorithm.
pub fn select_autonomous_algorithm(
    request: &mut Request,
    captain: &Captain,
    writer_id: u8,
) -> ResponseBox {
    let body: SelectAutonomousAlgorithmBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };

    if body.running && body.name.is_none() {
        return bad_request("can't run without an algorithm");
    }
    if let Some(name) = &body.name {
        let known = captain
            .try_topic::<AutonomousAlgorithmStatus>(AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME)
            .is_some_and(|topic| {
                topic
                    .read()
                    .available
                    .iter()
                    .any(|algorithm| &algorithm.name == name)
            });
        if !known {
            return bad_request(&format!("unknown autonomous algorithm: {name:?}"));
        }
    }

    captain
        .topic::<AutonomousAlgorithmSelection>(AUTONOMOUS_ALGORITHM_SELECTION_TOPIC_NAME)
        .write(
            writer_id,
            AutonomousAlgorithmSelection {
                name: body.name,
                running: body.running,
            },
        )
        .expect("lost writer authorization for the autonomous_algorithm_selection topic");
    json_response(&(), 200)
}

#[derive(serde::Deserialize)]
struct SetAutonomousParameterBody {
    algorithm: String,
    name: String,
    value: f64,
}

/// Serializes [`set_autonomous_parameter`]'s read-modify-write of
/// `autonomous_parameters` across `WebGui`'s worker threads - two sliders
/// moved at once would otherwise each write a copy missing the other's value.
static AUTONOMOUS_PARAMETERS_LOCK: Mutex<()> = Mutex::new(());

/// `POST /api/autonomous_parameter` - body `{"algorithm": "...", "name":
/// "...", "value": ...}` - sets one parameter's wanted value in
/// `autonomous_parameters`, for that algorithm to apply (see
/// [`crate::autonomous_control::ParameterTuner`]), which reports the value
/// it actually runs with in `autonomous_algorithm_status`. Only the
/// selected algorithm (running or paused) can be tuned, and only by a
/// parameter it declared.
pub fn set_autonomous_parameter(
    request: &mut Request,
    captain: &Captain,
    writer_id: u8,
) -> ResponseBox {
    let body: SetAutonomousParameterBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    if !body.value.is_finite() {
        return bad_request("value must be a finite number");
    }

    let parameters = match selected_parameters(captain, &body.algorithm) {
        Ok(parameters) => parameters,
        Err(response) => return response,
    };
    if !parameters
        .iter()
        .any(|parameter| parameter.name == body.name)
    {
        return bad_request(&format!(
            "{:?} has no tunable parameter {:?}",
            body.algorithm, body.name
        ));
    }

    let _guard = AUTONOMOUS_PARAMETERS_LOCK
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    let topic = captain.topic::<AutonomousParameters>(AUTONOMOUS_PARAMETERS_TOPIC_NAME);
    let mut parameters = topic.read().into_value();
    parameters
        .values
        .entry(body.algorithm)
        .or_default()
        .insert(body.name, body.value);
    topic
        .write(writer_id, parameters)
        .expect("lost writer authorization for the autonomous_parameters topic");
    json_response(&(), 200)
}

#[derive(serde::Deserialize)]
struct SaveAutonomousParametersBody {
    algorithm: String,
}

#[derive(serde::Serialize)]
struct SavedParameters {
    path: String,
}

/// `POST /api/autonomous_parameters_save` - body `{"algorithm": "..."}` -
/// writes the parameter values the selected algorithm currently runs with
/// (as reported in `autonomous_algorithm_status`) into its config file,
/// keeping the file's comments - see
/// [`crate::autonomous_control::save_parameters`]. They're used from the
/// next restart on. Responds with the file's path.
pub fn save_autonomous_parameters(request: &mut Request, captain: &Captain) -> ResponseBox {
    let body: SaveAutonomousParametersBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let parameters = match selected_parameters(captain, &body.algorithm) {
        Ok(parameters) => parameters,
        Err(response) => return response,
    };
    if parameters.is_empty() {
        return bad_request(&format!("{:?} has no tunable parameters", body.algorithm));
    }
    match autonomous_control::save_parameters(&body.algorithm, &parameters) {
        Ok(path) => json_response(
            &SavedParameters {
                path: path.display().to_string(),
            },
            200,
        ),
        Err(err) => error_response(500, &err),
    }
}

/// `algorithm`'s tunable parameters, with the values it currently runs
/// with - or a `400` response if it isn't the selected algorithm (running
/// or paused), the only one that can be tuned or saved.
fn selected_parameters(
    captain: &Captain,
    algorithm: &str,
) -> Result<Vec<AlgorithmParameter>, ResponseBox> {
    let status = captain
        .try_topic::<AutonomousAlgorithmStatus>(AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME)
        .map(|topic| topic.read().into_value())
        .unwrap_or_default();
    if status.selected.as_deref() != Some(algorithm) {
        return Err(bad_request(&format!(
            "{algorithm:?} isn't the selected algorithm - only that one can be tuned"
        )));
    }
    Ok(status
        .available
        .into_iter()
        .find(|available| available.name == algorithm)
        .map(|available| available.parameters)
        .unwrap_or_default())
}

/// `POST /api/restart` - asks the runner to kill every executor and bring
/// the whole system back up completely fresh (see
/// [`Captain::request_restart`]), e.g. from the "R" keyboard shortcut. Note
/// this `WebGui` itself is one of the executors restarted, so the response
/// to this very request is the last thing the old HTTP server sends before
/// it's torn down and rebuilt.
pub fn restart(captain: &Captain) -> ResponseBox {
    captain.request_restart();
    json_response(&(), 200)
}

/// `POST /api/place_at_start` - bumps `place_at_start`'s counter, asking
/// [`crate::actuators::SimulatedVehicle`] to place the vehicle at whatever
/// `start_state` currently holds, e.g. from the "P" keyboard shortcut. Unlike
/// [`restart`], this doesn't tear anything down - just resets the vehicle's
/// simulated position/heading/speed in place. Also turns SLAM off: the
/// vehicle jumps, and dead reckoning - whose frame the map is built in -
/// resets with it.
pub fn place_at_start(captain: &Captain, writer_id: u8) -> ResponseBox {
    let topic = captain.topic::<PlaceAtStart>(PLACE_AT_START_TOPIC_NAME);
    let requested = topic.read().requested.wrapping_add(1);
    topic
        .write(writer_id, PlaceAtStart { requested })
        .expect("lost writer authorization for the place_at_start topic");
    write_slam_command(captain, writer_id, SlamState::Off);
    json_response(&(), 200)
}

/// `GET /api/slam` - what [`crate::localization::Slam`] is doing, read from
/// the `slam_status` topic, as a [`StampedBody`]. The default status (off,
/// no scans) if nothing publishes that topic.
pub fn slam(captain: &Captain) -> ResponseBox {
    match captain.try_topic::<SlamStatus>(SLAM_STATUS_TOPIC_NAME) {
        Some(topic) => {
            let status = topic.read();
            stamped_json(&status.value, status.meta)
        }
        None => stamped_json(&SlamStatus::default(), WriteMeta::default()),
    }
}

#[derive(serde::Deserialize)]
struct SlamCommandBody {
    state: String,
}

/// `POST /api/slam_command` - body `{"state": "running" | "waiting" |
/// "off"}` - writes the wanted state to `slam_command`, for
/// [`crate::localization::Slam`] to pick up. `"off"` clears SLAM's map.
pub fn slam_command(request: &mut Request, captain: &Captain, writer_id: u8) -> ResponseBox {
    let body: SlamCommandBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let Some(state) = SlamState::from_api_str(&body.state) else {
        return bad_request(&format!("unknown SLAM state: {:?}", body.state));
    };
    write_slam_command(captain, writer_id, state);
    json_response(&(), 200)
}

/// Writes `state` to `slam_command`, bumping
/// [`SlamCommand::clear_requested`] when it's [`SlamState::Off`] - so SLAM
/// clears its map even if it never sees `Off` itself, e.g. when Play follows
/// within one of its ticks.
fn write_slam_command(captain: &Captain, writer_id: u8, state: SlamState) {
    let topic = captain.topic::<SlamCommand>(SLAM_COMMAND_TOPIC_NAME);
    let mut command = topic.read().into_value();
    command.state = state;
    if state == SlamState::Off {
        command.clear_requested = command.clear_requested.wrapping_add(1);
    }
    topic
        .write(writer_id, command)
        .expect("lost writer authorization for the slam_command topic");
}
