//! The live, topic-backed control API: which map, vehicle model, and
//! autonomous algorithm are currently selected (the `map`,
//! `vehicle_model_status`, and `autonomous_algorithm_status` topics), and the
//! write endpoints a driver uses to steer the vehicle, pick its map, model,
//! and autonomous algorithm, tune that model and algorithm, place it at the
//! start line, drive SLAM, and plan a race line (`map_selection`,
//! `human_vesc_command`, `vehicle_model_selection`,
//! `vehicle_model_parameters`, `autonomous_algorithm_selection`,
//! `autonomous_parameters`, `place_at_start`, `slam_command`, `slam_save`,
//! `planning_parameters`, `planning_request`) - as
//! opposed to [`super::maps_api`], which lists/generates map folders on
//! disk, and [`super::draw_api`], which serves what's drawn on the map.

use super::WebGuiConfig;
use super::maps_api::safe_map_folder;
use crate::topics::{
    AUTONOMOUS_ALGORITHM_SELECTION_TOPIC_NAME, AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME,
    AUTONOMOUS_PARAMETERS_TOPIC_NAME, ActuatorLimits, AlgorithmParameter,
    AutonomousAlgorithmSelection, AutonomousAlgorithmStatus, AutonomousParameters,
    HUMAN_VESC_COMMAND_TOPIC_NAME, MAP_SELECTION_TOPIC_NAME, MAP_TOPIC_NAME, MapSelection,
    PLACE_AT_START_TOPIC_NAME, PLANNING_PARAMETERS_TOPIC_NAME, PLANNING_REQUEST_TOPIC_NAME,
    PLANNING_STATUS_TOPIC_NAME, PlaceAtStart, PlanningObjective, PlanningParameters,
    PlanningRequest, PlanningState, PlanningStatus, SLAM_COMMAND_TOPIC_NAME, SLAM_SAVE_TOPIC_NAME,
    SLAM_STATUS_TOPIC_NAME, SelectedMap, SlamCommand, SlamSaveRequest, SlamState, SlamStatus,
    VEHICLE_MODEL_PARAMETERS_TOPIC_NAME, VEHICLE_MODEL_SELECTION_TOPIC_NAME,
    VEHICLE_MODEL_STATUS_TOPIC_NAME, VehicleModelKind, VehicleModelParameters,
    VehicleModelSelection, VehicleModelStatus, VescCommand,
};
use crate::web::{bad_request, error_response, json_response, read_json};
use crate::{Captain, WriteMeta};
use crate::{actuators, autonomous_control, planning};
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
    writer_id: u16,
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
pub fn human_vesc_command(request: &mut Request, captain: &Captain, writer_id: u16) -> ResponseBox {
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
    parameters: Vec<AlgorithmParameter>,
    limits: Vec<AlgorithmParameter>,
}

/// `GET /api/vehicle_model` - the vehicle model kind currently running, its
/// tunable parameters, and the actuator limits, with the values in effect,
/// read from the `vehicle_model_status` topic, as a [`StampedBody`].
pub fn vehicle_model(captain: &Captain) -> ResponseBox {
    let status = captain
        .topic::<VehicleModelStatus>(VEHICLE_MODEL_STATUS_TOPIC_NAME)
        .read();
    stamped_json(
        &LiveVehicleModel {
            kind: status.kind.api_str(),
            parameters: status.value.parameters,
            limits: status.value.limits,
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
    writer_id: u16,
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

#[derive(serde::Deserialize)]
struct SetVehicleModelParameterBody {
    kind: String,
    name: String,
    value: f64,
}

/// Serializes [`set_vehicle_model_parameter`]'s read-modify-write of
/// `vehicle_model_parameters`, like [`AUTONOMOUS_PARAMETERS_LOCK`].
static VEHICLE_MODEL_PARAMETERS_LOCK: Mutex<()> = Mutex::new(());

/// `POST /api/vehicle_model_parameter` - body `{"kind": "...", "name":
/// "...", "value": ...}` - sets one parameter's wanted value in
/// `vehicle_model_parameters`, for [`crate::actuators::SimulatedVehicle`]
/// to apply, which reports the value it actually runs with in
/// `vehicle_model_status`. Only the running model can be tuned, and only
/// by a parameter it declared.
pub fn set_vehicle_model_parameter(
    request: &mut Request,
    captain: &Captain,
    writer_id: u16,
) -> ResponseBox {
    let body: SetVehicleModelParameterBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    if !body.value.is_finite() {
        return bad_request("value must be a finite number");
    }

    let (_, parameters) = match running_model_parameters(captain, &body.kind) {
        Ok(running) => running,
        Err(response) => return response,
    };
    if !parameters
        .iter()
        .any(|parameter| parameter.name == body.name)
    {
        return bad_request(&format!(
            "{:?} has no tunable parameter {:?}",
            body.kind, body.name
        ));
    }

    write_vehicle_parameters(captain, writer_id, |wanted| {
        wanted
            .values
            .entry(body.kind)
            .or_default()
            .insert(body.name, body.value);
    });
    json_response(&(), 200)
}

#[derive(serde::Deserialize)]
struct SaveVehicleModelParametersBody {
    kind: String,
}

/// `POST /api/vehicle_model_parameters_save` - body `{"kind": "..."}` -
/// writes the parameter values the running model currently runs with (as
/// reported in `vehicle_model_status`) into its `[<kind>]` table of the
/// vehicle's config file, keeping the rest of the file - see
/// [`actuators::save_vehicle_model_parameters`]. They're used from the next
/// restart on. Responds with the file's path.
pub fn save_vehicle_model_parameters(request: &mut Request, captain: &Captain) -> ResponseBox {
    let body: SaveVehicleModelParametersBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let (kind, parameters) = match running_model_parameters(captain, &body.kind) {
        Ok(running) => running,
        Err(response) => return response,
    };
    if parameters.is_empty() {
        return bad_request(&format!("{:?} has no tunable parameters", body.kind));
    }
    match actuators::save_vehicle_model_parameters(kind, &parameters) {
        Ok(path) => json_response(
            &SavedParameters {
                path: path.display().to_string(),
            },
            200,
        ),
        Err(err) => error_response(500, &err),
    }
}

#[derive(serde::Deserialize)]
struct SetVehicleLimitBody {
    name: String,
    value: f64,
}

/// `POST /api/vehicle_limit` - body `{"name": "...", "value": ...}` - sets
/// one actuator limit's wanted value in `vehicle_model_parameters`, for
/// [`crate::actuators::SimulatedVehicle`] to apply whichever model is
/// running, which reports the value in effect in `vehicle_model_status` and
/// republishes `vehicle_limits`.
pub fn set_vehicle_limit(request: &mut Request, captain: &Captain, writer_id: u16) -> ResponseBox {
    let body: SetVehicleLimitBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    if !body.value.is_finite() {
        return bad_request("value must be a finite number");
    }
    if !ActuatorLimits::tunable_parameters()
        .iter()
        .any(|parameter| parameter.name == body.name)
    {
        return bad_request(&format!("no tunable actuator limit {:?}", body.name));
    }

    write_vehicle_parameters(captain, writer_id, |wanted| {
        wanted.limits.insert(body.name, body.value);
    });
    json_response(&(), 200)
}

/// `POST /api/vehicle_limits_save` - writes the actuator limits currently in
/// effect (as reported in `vehicle_model_status`) into the `[limits]` table
/// of the vehicle's config file, keeping the rest of the file - see
/// [`actuators::save_vehicle_limits`]. Responds with the file's path.
pub fn save_vehicle_limits(captain: &Captain) -> ResponseBox {
    let limits = captain
        .topic::<VehicleModelStatus>(VEHICLE_MODEL_STATUS_TOPIC_NAME)
        .read()
        .into_value()
        .limits;
    if limits.is_empty() {
        return bad_request("no actuator limits reported yet");
    }
    match actuators::save_vehicle_limits(&limits) {
        Ok(path) => json_response(
            &SavedParameters {
                path: path.display().to_string(),
            },
            200,
        ),
        Err(err) => error_response(500, &err),
    }
}

/// `POST /api/vehicle_model_parameters_load` - body `{"kind": "..."}` - asks
/// the running model to run with the values its `[<kind>]` table of the
/// vehicle's config file holds again, undoing any unsaved tuning, by
/// writing them to `vehicle_model_parameters` like a slider would. Responds
/// with the file's path.
pub fn load_vehicle_model_parameters(
    request: &mut Request,
    captain: &Captain,
    writer_id: u16,
) -> ResponseBox {
    let body: SaveVehicleModelParametersBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let (kind, parameters) = match running_model_parameters(captain, &body.kind) {
        Ok(running) => running,
        Err(response) => return response,
    };
    let (values, path) = match actuators::saved_vehicle_model_values(kind, &parameters) {
        Ok(saved) => saved,
        Err(err) => return error_response(500, &err),
    };
    write_vehicle_parameters(captain, writer_id, |wanted| {
        wanted.values.entry(body.kind).or_default().extend(values);
    });
    loaded(&path)
}

/// `POST /api/vehicle_limits_load` - asks the vehicle to run with the
/// actuator limits the `[limits]` table of its config file holds again,
/// undoing any unsaved tuning. Responds with the file's path.
pub fn load_vehicle_limits(captain: &Captain, writer_id: u16) -> ResponseBox {
    let (values, path) = match actuators::saved_vehicle_limits(&ActuatorLimits::tunable_parameters())
    {
        Ok(saved) => saved,
        Err(err) => return error_response(500, &err),
    };
    write_vehicle_parameters(captain, writer_id, |wanted| wanted.limits.extend(values));
    loaded(&path)
}

/// Read-modify-writes `vehicle_model_parameters` with `change`, under
/// [`VEHICLE_MODEL_PARAMETERS_LOCK`].
fn write_vehicle_parameters(
    captain: &Captain,
    writer_id: u16,
    change: impl FnOnce(&mut VehicleModelParameters),
) {
    let _guard = VEHICLE_MODEL_PARAMETERS_LOCK
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    let topic = captain.topic::<VehicleModelParameters>(VEHICLE_MODEL_PARAMETERS_TOPIC_NAME);
    let mut wanted = topic.read().into_value();
    change(&mut wanted);
    topic
        .write(writer_id, wanted)
        .expect("lost writer authorization for the vehicle_model_parameters topic");
}

/// The running vehicle model's kind and tunable parameters, with the values
/// it currently runs with - or a `400` response if `kind` isn't the running
/// model, the only one that can be tuned or saved.
fn running_model_parameters(
    captain: &Captain,
    kind: &str,
) -> Result<(VehicleModelKind, Vec<AlgorithmParameter>), ResponseBox> {
    let status = captain
        .topic::<VehicleModelStatus>(VEHICLE_MODEL_STATUS_TOPIC_NAME)
        .read()
        .into_value();
    if status.kind.api_str() != kind {
        return Err(bad_request(&format!(
            "{kind:?} isn't the running vehicle model - only that one can be tuned"
        )));
    }
    Ok((status.kind, status.parameters))
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
    writer_id: u16,
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
    writer_id: u16,
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

/// `POST /api/autonomous_parameters_load` - body `{"algorithm": "..."}` -
/// asks the selected algorithm to run with the values its config file holds
/// again (see [`autonomous_control::saved_values`]), undoing any unsaved
/// tuning, by writing them to `autonomous_parameters` like a slider would.
/// Responds with the file's path.
pub fn load_autonomous_parameters(
    request: &mut Request,
    captain: &Captain,
    writer_id: u16,
) -> ResponseBox {
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
    let (values, path) = match autonomous_control::saved_values(&body.algorithm, &parameters) {
        Ok(saved) => saved,
        Err(err) => return error_response(500, &err),
    };

    let _guard = AUTONOMOUS_PARAMETERS_LOCK
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    let topic = captain.topic::<AutonomousParameters>(AUTONOMOUS_PARAMETERS_TOPIC_NAME);
    let mut parameters = topic.read().into_value();
    parameters
        .values
        .entry(body.algorithm)
        .or_default()
        .extend(values);
    topic
        .write(writer_id, parameters)
        .expect("lost writer authorization for the autonomous_parameters topic");
    loaded(&path)
}

/// The response to a successful load from the config file at `path`.
fn loaded(path: &Path) -> ResponseBox {
    json_response(
        &SavedParameters {
            path: path.display().to_string(),
        },
        200,
    )
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
/// simulated position/heading/speed in place. Also turns SLAM's mapping
/// off: the vehicle jumps, and dead reckoning - whose frame the map is built
/// in - resets with it. Localization carries on: SLAM restarts it from the
/// start by itself when dead reckoning resets.
pub fn place_at_start(captain: &Captain, writer_id: u16) -> ResponseBox {
    let topic = captain.topic::<PlaceAtStart>(PLACE_AT_START_TOPIC_NAME);
    let requested = topic.read().requested.wrapping_add(1);
    topic
        .write(writer_id, PlaceAtStart { requested })
        .expect("lost writer authorization for the place_at_start topic");
    let slam_state = captain
        .topic::<SlamCommand>(SLAM_COMMAND_TOPIC_NAME)
        .read()
        .state;
    if !slam_state.is_localization() {
        write_slam_command(captain, writer_id, SlamState::Off);
    }
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
/// "off" | "localizing" | "localization_paused"}` - writes the wanted state to `slam_command`, for
/// [`crate::localization::Slam`] to pick up. `"off"` clears SLAM's map.
pub fn slam_command(request: &mut Request, captain: &Captain, writer_id: u16) -> ResponseBox {
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

#[derive(serde::Deserialize)]
struct SlamSaveBody {
    name: String,
}

/// The response to a [`slam_save`]: the request number whose outcome to
/// look for in [`SlamStatus::last_save`].
#[derive(serde::Serialize)]
struct SlamSaveResponse {
    requested: u64,
}

/// `POST /api/slam_save` - body `{"name": "..."}` - asks
/// [`crate::localization::Slam`] to save its map as a new map folder named
/// `name`. SLAM saves it on its next tick, and reports how it went on
/// `slam_status`'s `last_save`, under the returned request number.
pub fn slam_save(request: &mut Request, captain: &Captain, writer_id: u16) -> ResponseBox {
    let body: SlamSaveBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let topic = captain.topic::<SlamSaveRequest>(SLAM_SAVE_TOPIC_NAME);
    let requested = topic.read().requested.wrapping_add(1);
    topic
        .write(
            writer_id,
            SlamSaveRequest {
                name: body.name,
                requested,
            },
        )
        .expect("lost writer authorization for the slam_save topic");
    json_response(&SlamSaveResponse { requested }, 200)
}

/// Writes `state` to `slam_command`, bumping
/// [`SlamCommand::clear_requested`] when it's [`SlamState::Off`] - so SLAM
/// clears its map even if it never sees `Off` itself, e.g. when Play follows
/// within one of its ticks.
fn write_slam_command(captain: &Captain, writer_id: u16, state: SlamState) {
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

/// `GET /api/planning` - what [`crate::planning::Planner`] is doing, the
/// parameter values it runs with, and how its latest request went, read
/// from the `planning_status` topic, as a [`StampedBody`]. The default
/// status (idle, no parameters) if nothing publishes that topic.
pub fn planning(captain: &Captain) -> ResponseBox {
    match captain.try_topic::<PlanningStatus>(PLANNING_STATUS_TOPIC_NAME) {
        Some(topic) => {
            let status = topic.read();
            stamped_json(&status.value, status.meta)
        }
        None => stamped_json(&PlanningStatus::default(), WriteMeta::default()),
    }
}

/// The planner's tunable parameters with the values it currently runs
/// with - or a `400` response if no planner reports any.
fn planning_parameters(captain: &Captain) -> Result<Vec<AlgorithmParameter>, ResponseBox> {
    let parameters = captain
        .try_topic::<PlanningStatus>(PLANNING_STATUS_TOPIC_NAME)
        .map(|topic| topic.read().into_value().parameters)
        .unwrap_or_default();
    if parameters.is_empty() {
        return Err(bad_request("no planner is running"));
    }
    Ok(parameters)
}

/// Serializes the read-modify-writes of `planning_parameters` across
/// `WebGui`'s worker threads, as [`AUTONOMOUS_PARAMETERS_LOCK`].
static PLANNING_PARAMETERS_LOCK: Mutex<()> = Mutex::new(());

/// Read-modify-writes `planning_parameters` with `change`, under
/// [`PLANNING_PARAMETERS_LOCK`].
fn write_planning_parameters(
    captain: &Captain,
    writer_id: u16,
    change: impl FnOnce(&mut PlanningParameters),
) {
    let _guard = PLANNING_PARAMETERS_LOCK
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    let topic = captain.topic::<PlanningParameters>(PLANNING_PARAMETERS_TOPIC_NAME);
    let mut wanted = topic.read().into_value();
    change(&mut wanted);
    topic
        .write(writer_id, wanted)
        .expect("lost writer authorization for the planning_parameters topic");
}

#[derive(serde::Deserialize)]
struct SetPlanningParameterBody {
    name: String,
    value: f64,
}

/// `POST /api/planning_parameter` - body `{"name": "...", "value": ...}` -
/// sets one parameter's wanted value in `planning_parameters`, for the
/// planner to apply (right away while idle, after the current planning
/// otherwise), which reports the value it actually runs with in
/// `planning_status`.
pub fn set_planning_parameter(
    request: &mut Request,
    captain: &Captain,
    writer_id: u16,
) -> ResponseBox {
    let body: SetPlanningParameterBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    if !body.value.is_finite() {
        return bad_request("value must be a finite number");
    }
    let parameters = match planning_parameters(captain) {
        Ok(parameters) => parameters,
        Err(response) => return response,
    };
    if !parameters
        .iter()
        .any(|parameter| parameter.name == body.name)
    {
        return bad_request(&format!(
            "the planner has no tunable parameter {:?}",
            body.name
        ));
    }
    write_planning_parameters(captain, writer_id, |wanted| {
        wanted.values.insert(body.name, body.value);
    });
    json_response(&(), 200)
}

/// `POST /api/planning_parameters_save` - writes the parameter values the
/// planner currently runs with (as reported in `planning_status`) into its
/// config file, keeping the file's comments - see
/// [`planning::save_parameters`]. Responds with the file's path.
pub fn save_planning_parameters(captain: &Captain) -> ResponseBox {
    let parameters = match planning_parameters(captain) {
        Ok(parameters) => parameters,
        Err(response) => return response,
    };
    match planning::save_parameters(&parameters) {
        Ok(path) => loaded(&path),
        Err(err) => error_response(500, &err),
    }
}

/// `POST /api/planning_parameters_load` - asks the planner to run with the
/// values its config file holds again (see [`planning::saved_values`]),
/// undoing any unsaved tuning, by writing them to `planning_parameters`
/// like a slider would. Responds with the file's path.
pub fn load_planning_parameters(captain: &Captain, writer_id: u16) -> ResponseBox {
    let parameters = match planning_parameters(captain) {
        Ok(parameters) => parameters,
        Err(response) => return response,
    };
    let (values, path) = match planning::saved_values(&parameters) {
        Ok(saved) => saved,
        Err(err) => return error_response(500, &err),
    };
    write_planning_parameters(captain, writer_id, |wanted| wanted.values.extend(values));
    loaded(&path)
}

/// The response to a [`planning_start`]: the request number whose outcome
/// to look for in [`PlanningStatus::last_outcome`].
#[derive(serde::Serialize)]
struct PlanningStartResponse {
    requested: u64,
}

#[derive(serde::Deserialize, Default)]
struct PlanningStartBody {
    #[serde(default)]
    objective: PlanningObjective,
}

/// `POST /api/planning_start` - body `{"objective": "min_curvature" |
/// "min_time"}` (empty for minimum curvature) - asks
/// [`crate::planning::Planner`] to plan a race line for the selected map,
/// by bumping `planning_request`'s counter. The planner reports its
/// progress and outcome on `planning_status`, the outcome under the
/// returned request number. Refused while it's already planning, or with
/// no map selected.
pub fn planning_start(request: &mut Request, captain: &Captain, writer_id: u16) -> ResponseBox {
    let mut text = String::new();
    if let Err(err) = request.as_reader().read_to_string(&mut text) {
        return bad_request(&format!("failed to read the request body: {err}"));
    }
    let body: PlanningStartBody = if text.trim().is_empty() {
        PlanningStartBody::default()
    } else {
        match serde_json::from_str(&text) {
            Ok(body) => body,
            Err(err) => return bad_request(&format!("invalid body: {err}")),
        }
    };
    let status = captain
        .try_topic::<PlanningStatus>(PLANNING_STATUS_TOPIC_NAME)
        .map(|topic| topic.read().into_value());
    match status {
        None => return bad_request("no planner is running"),
        Some(status) if status.state == PlanningState::Computing => {
            return bad_request("the planner is already planning");
        }
        Some(_) => {}
    }
    if captain
        .topic::<SelectedMap>(MAP_TOPIC_NAME)
        .read()
        .path
        .is_none()
    {
        return bad_request("no map is selected");
    }
    let topic = captain.topic::<PlanningRequest>(PLANNING_REQUEST_TOPIC_NAME);
    let requested = topic.read().requested.wrapping_add(1);
    topic
        .write(
            writer_id,
            PlanningRequest {
                requested,
                objective: body.objective,
            },
        )
        .expect("lost writer authorization for the planning_request topic");
    json_response(&PlanningStartResponse { requested }, 200)
}
