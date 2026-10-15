//! The SLAM part of the live API: starting, pausing and resetting mapping
//! and localization, saving the map built (`slam_command`, `slam_save`),
//! and tuning SLAM live (`slam_parameters`) and saving or reloading its
//! config file - the same shape as the Detector panel's, see
//! [`super::detector_api`].

use super::live_api::{loaded, stamped_json};
use crate::localization;
use crate::topics::{
    AlgorithmParameter, SLAM_COMMAND_TOPIC_NAME, SLAM_PARAMETERS_TOPIC_NAME, SLAM_SAVE_TOPIC_NAME,
    SLAM_STATUS_TOPIC_NAME, SlamCommand, SlamParameters, SlamSaveRequest, SlamState, SlamStatus,
};
use crate::web::{bad_request, error_response, json_response, read_json};
use crate::{Captain, WriteMeta};
use std::sync::Mutex;
use tiny_http::{Request, ResponseBox};

/// Turns SLAM's mapping off, as any placement of the ego vehicle must: it
/// jumps, and dead reckoning - whose frame the map is built in - resets with
/// it. Localization carries on.
pub(super) fn stop_mapping(captain: &Captain, writer_id: u16) {
    let slam_state = captain
        .topic::<SlamCommand>(SLAM_COMMAND_TOPIC_NAME)
        .read()
        .state;
    if !slam_state.is_localization() {
        write_slam_command(captain, writer_id, SlamState::Off);
    }
}

/// `GET /api/slam` - what [`crate::localization::Slam`] is doing, read from
/// the `slam_status` topic, as a [`StampedBody`](super::live_api::StampedBody). The default status (off,
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
pub(super) fn write_slam_command(captain: &Captain, writer_id: u16, state: SlamState) {
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

/// SLAM's tunable parameters with the values it currently runs with - or a
/// `400` response if SLAM reports none.
fn slam_parameters(captain: &Captain) -> Result<Vec<AlgorithmParameter>, ResponseBox> {
    let parameters = captain
        .try_topic::<SlamStatus>(SLAM_STATUS_TOPIC_NAME)
        .map(|topic| topic.read().into_value().parameters)
        .unwrap_or_default();
    if parameters.is_empty() {
        return Err(bad_request("SLAM is not running"));
    }
    Ok(parameters)
}

/// Serializes the read-modify-writes of `slam_parameters` across
/// `WebGui`'s worker threads, as the detector's own lock does.
static SLAM_PARAMETERS_LOCK: Mutex<()> = Mutex::new(());

/// Read-modify-writes `slam_parameters` with `change`, under
/// [`SLAM_PARAMETERS_LOCK`].
fn write_slam_parameters(
    captain: &Captain,
    writer_id: u16,
    change: impl FnOnce(&mut SlamParameters),
) {
    let _guard = SLAM_PARAMETERS_LOCK
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    let topic = captain.topic::<SlamParameters>(SLAM_PARAMETERS_TOPIC_NAME);
    let mut wanted = topic.read().into_value();
    change(&mut wanted);
    topic
        .write(writer_id, wanted)
        .expect("lost writer authorization for the slam_parameters topic");
}

#[derive(serde::Deserialize)]
struct SetSlamParameterBody {
    name: String,
    value: f64,
}

/// `POST /api/slam_parameter` - body `{"name": "...", "value": ...}` - sets
/// one parameter's wanted value in `slam_parameters`, for SLAM to apply
/// before its next tick; it reports the value it actually runs with in
/// `slam_status`.
pub fn set_slam_parameter(request: &mut Request, captain: &Captain, writer_id: u16) -> ResponseBox {
    let body: SetSlamParameterBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    if !body.value.is_finite() {
        return bad_request("value must be a finite number");
    }
    let parameters = match slam_parameters(captain) {
        Ok(parameters) => parameters,
        Err(response) => return response,
    };
    if !parameters
        .iter()
        .any(|parameter| parameter.name == body.name)
    {
        return bad_request(&format!("SLAM has no tunable parameter {:?}", body.name));
    }
    write_slam_parameters(captain, writer_id, |wanted| {
        wanted.values.insert(body.name, body.value);
    });
    json_response(&(), 200)
}

/// `POST /api/slam_parameters_save` - writes the parameter values SLAM
/// currently runs with into its config file, keeping the file's comments.
/// Responds with the file's path.
pub fn save_slam_parameters(captain: &Captain) -> ResponseBox {
    let parameters = match slam_parameters(captain) {
        Ok(parameters) => parameters,
        Err(response) => return response,
    };
    match localization::save_slam_parameters(&parameters) {
        Ok(path) => loaded(&path),
        Err(err) => error_response(500, &err),
    }
}

/// `POST /api/slam_parameters_load` - asks SLAM to run with the values its
/// config file holds again, undoing any unsaved tuning, by writing them to
/// `slam_parameters` like a slider would. Responds with the file's path.
pub fn load_slam_parameters(captain: &Captain, writer_id: u16) -> ResponseBox {
    let parameters = match slam_parameters(captain) {
        Ok(parameters) => parameters,
        Err(response) => return response,
    };
    let (values, path) = match localization::saved_slam_values(&parameters) {
        Ok(saved) => saved,
        Err(err) => return error_response(500, &err),
    };
    write_slam_parameters(captain, writer_id, |wanted| wanted.values.extend(values));
    loaded(&path)
}
