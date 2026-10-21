//! The localization part of the live API, next to [`super::slam_api`]'s
//! Start/Pause: picking the algorithm, restarting it from a pose or
//! globally (`localization_command`), and tuning the particle filter live
//! (`particle_filter_parameters`) and saving or reloading its config file -
//! the same shape as SLAM's parameters.

use super::live_api::{loaded, stamped_json};
use crate::localization;
use crate::topics::{
    AlgorithmParameter, LOCALIZATION_COMMAND_TOPIC_NAME, LOCALIZATION_STATUS_TOPIC_NAME,
    LocalizationAlgorithm, LocalizationCommand, LocalizationStatus,
    PARTICLE_FILTER_PARAMETERS_TOPIC_NAME, ParticleFilterParameters, SLAM_STATUS_TOPIC_NAME,
    SlamState, SlamStatus,
};
use crate::web::{bad_request, error_response, json_response, read_json};
use crate::{Captain, WriteMeta};
use std::sync::Mutex;
use tiny_http::{Request, ResponseBox};

/// `GET /api/localization` - what the localization backend is doing, read
/// from `localization_status`, as a
/// [`StampedBody`](super::live_api::StampedBody). The default status if
/// nothing publishes that topic.
pub fn localization(captain: &Captain) -> ResponseBox {
    match captain.try_topic::<LocalizationStatus>(LOCALIZATION_STATUS_TOPIC_NAME) {
        Some(topic) => {
            let status = topic.read();
            stamped_json(&status.value, status.meta)
        }
        None => stamped_json(&LocalizationStatus::default(), WriteMeta::default()),
    }
}

/// Serializes the read-modify-writes of `localization_command` across
/// `WebGui`'s worker threads.
static COMMAND_LOCK: Mutex<()> = Mutex::new(());

/// Read-modify-writes `localization_command` with `change`, under
/// [`COMMAND_LOCK`].
fn write_command(captain: &Captain, writer_id: u16, change: impl FnOnce(&mut LocalizationCommand)) {
    let _guard = COMMAND_LOCK
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    let topic = captain.topic::<LocalizationCommand>(LOCALIZATION_COMMAND_TOPIC_NAME);
    let mut command = topic.read().into_value();
    change(&mut command);
    topic
        .write(writer_id, command)
        .expect("lost writer authorization for the localization_command topic");
}

/// Whether SLAM reports it's localizing right now.
fn localizing(captain: &Captain) -> bool {
    captain
        .try_topic::<SlamStatus>(SLAM_STATUS_TOPIC_NAME)
        .is_some_and(|topic| topic.read().state == SlamState::Localizing)
}

/// The algorithm localizing right now, if any.
fn running_algorithm(captain: &Captain) -> Option<LocalizationAlgorithm> {
    if !localizing(captain) {
        return None;
    }
    captain
        .try_topic::<LocalizationStatus>(LOCALIZATION_STATUS_TOPIC_NAME)
        .map(|topic| topic.read().into_value())
        .filter(|status| status.active)
        .map(|status| status.algorithm)
}

#[derive(serde::Deserialize)]
struct AlgorithmBody {
    algorithm: String,
}

/// `POST /api/localization_algorithm` - body `{"algorithm": "scan_matching"
/// | "particle_filter"}` - picks the algorithm localization runs with.
/// Refused (`409`) while localizing with another one: pause first.
pub fn set_algorithm(request: &mut Request, captain: &Captain, writer_id: u16) -> ResponseBox {
    let body: AlgorithmBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let Some(algorithm) = LocalizationAlgorithm::from_api_str(&body.algorithm) else {
        return bad_request(&format!(
            "unknown localization algorithm: {:?}",
            body.algorithm
        ));
    };
    write_algorithm(captain, writer_id, algorithm).unwrap_or_else(|response| response)
}

/// Writes `algorithm` to `localization_command` - or the `409` response if
/// another one is localizing.
pub(super) fn write_algorithm(
    captain: &Captain,
    writer_id: u16,
    algorithm: LocalizationAlgorithm,
) -> Result<ResponseBox, ResponseBox> {
    if running_algorithm(captain).is_some_and(|running| running != algorithm) {
        return Err(error_response(
            409,
            "Localization is running with another algorithm - pause it first.",
        ));
    }
    write_command(captain, writer_id, |command| command.algorithm = algorithm);
    Ok(json_response(&(), 200))
}

#[derive(serde::Deserialize)]
struct SetPoseBody {
    x_m: f64,
    y_m: f64,
    heading_rad: f64,
}

/// `POST /api/localization_set_pose` - body `{"x_m", "y_m", "heading_rad"}`,
/// on the map - restarts localization from that pose.
pub fn set_pose(request: &mut Request, captain: &Captain, writer_id: u16) -> ResponseBox {
    let body: SetPoseBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    if ![body.x_m, body.y_m, body.heading_rad]
        .iter()
        .all(|value| value.is_finite())
    {
        return bad_request("the pose must be finite numbers");
    }
    if !localizing(captain) {
        return error_response(409, "Localization isn't running - start it first.");
    }
    write_command(captain, writer_id, |command| {
        command.set_pose = Some([body.x_m, body.y_m, body.heading_rad]);
        command.set_pose_requested = command.set_pose_requested.wrapping_add(1);
    });
    json_response(&(), 200)
}

/// `POST /api/localization_relocalize` - restarts the particle filter from
/// anywhere on the map. Refused (`409`) unless the particle filter is
/// localizing: scan matching can't search the whole map.
pub fn relocalize(captain: &Captain, writer_id: u16) -> ResponseBox {
    if running_algorithm(captain) != Some(LocalizationAlgorithm::ParticleFilter) {
        return error_response(
            409,
            "Only the particle filter, while localizing, can relocalize globally.",
        );
    }
    write_command(captain, writer_id, |command| {
        command.relocalize_requested = command.relocalize_requested.wrapping_add(1);
    });
    json_response(&(), 200)
}

/// The particle filter's tunable parameters with the values it currently
/// runs with - or a `400` response if nothing reports them.
fn particle_filter_parameters(captain: &Captain) -> Result<Vec<AlgorithmParameter>, ResponseBox> {
    let parameters = captain
        .try_topic::<LocalizationStatus>(LOCALIZATION_STATUS_TOPIC_NAME)
        .map(|topic| topic.read().into_value().particle_filter_parameters)
        .unwrap_or_default();
    if parameters.is_empty() {
        return Err(bad_request("SLAM is not running"));
    }
    Ok(parameters)
}

/// Serializes the read-modify-writes of `particle_filter_parameters`.
static PARAMETERS_LOCK: Mutex<()> = Mutex::new(());

fn write_parameters(
    captain: &Captain,
    writer_id: u16,
    change: impl FnOnce(&mut ParticleFilterParameters),
) {
    let _guard = PARAMETERS_LOCK
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    let topic = captain.topic::<ParticleFilterParameters>(PARTICLE_FILTER_PARAMETERS_TOPIC_NAME);
    let mut wanted = topic.read().into_value();
    change(&mut wanted);
    topic
        .write(writer_id, wanted)
        .expect("lost writer authorization for the particle_filter_parameters topic");
}

#[derive(serde::Deserialize)]
struct SetParameterBody {
    name: String,
    value: f64,
}

/// `POST /api/particle_filter_parameter` - body `{"name": "...", "value":
/// ...}` - sets one parameter's wanted value, for the particle filter to
/// apply before SLAM's next tick.
pub fn set_parameter(request: &mut Request, captain: &Captain, writer_id: u16) -> ResponseBox {
    let body: SetParameterBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    if !body.value.is_finite() {
        return bad_request("value must be a finite number");
    }
    let parameters = match particle_filter_parameters(captain) {
        Ok(parameters) => parameters,
        Err(response) => return response,
    };
    if !parameters
        .iter()
        .any(|parameter| parameter.name == body.name)
    {
        return bad_request(&format!(
            "the particle filter has no tunable parameter {:?}",
            body.name
        ));
    }
    write_parameters(captain, writer_id, |wanted| {
        wanted.values.insert(body.name, body.value);
    });
    json_response(&(), 200)
}

/// `POST /api/particle_filter_parameters_save` - writes the values the
/// particle filter runs with into its config file. Responds with the
/// file's path.
pub fn save_parameters(captain: &Captain) -> ResponseBox {
    let parameters = match particle_filter_parameters(captain) {
        Ok(parameters) => parameters,
        Err(response) => return response,
    };
    match localization::save_particle_filter_parameters(&parameters) {
        Ok(path) => loaded(&path),
        Err(err) => error_response(500, &err),
    }
}

/// `POST /api/particle_filter_parameters_load` - runs the particle filter
/// with the values its config file holds again. Responds with the file's
/// path.
pub fn load_parameters(captain: &Captain, writer_id: u16) -> ResponseBox {
    let parameters = match particle_filter_parameters(captain) {
        Ok(parameters) => parameters,
        Err(response) => return response,
    };
    let (values, path) = match localization::saved_particle_filter_values(&parameters) {
        Ok(saved) => saved,
        Err(err) => return error_response(500, &err),
    };
    write_parameters(captain, writer_id, |wanted| wanted.values.extend(values));
    loaded(&path)
}
