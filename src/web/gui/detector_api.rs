//! The Detector panel's API: what [`crate::perception::UbmDetector`] is
//! doing and found (the `detector_status` and `detected_opponent` topics),
//! and the endpoints tuning it live (`detector_parameters`) and saving or
//! reloading its config file - the same shape as the Planning panel's, see
//! [`super::planning_api::planning`].

use super::live_api::{loaded, stamped_json};
use crate::perception;
use crate::topics::{
    AlgorithmParameter, DETECTED_OPPONENT_TOPIC_NAME, DETECTOR_PARAMETERS_TOPIC_NAME,
    DETECTOR_STATUS_TOPIC_NAME, DetectedOpponent, DetectorParameters, DetectorStatus,
};
use crate::web::{bad_request, error_response, json_response, read_json};
use crate::{Captain, WriteMeta};
use std::sync::Mutex;
use tiny_http::{Request, ResponseBox};

/// The body of [`detector`]: the status and the latest detection together,
/// so the panel needs one request per poll.
#[derive(serde::Serialize)]
struct DetectorBody {
    /// `None` if no detector runs in this binary.
    status: Option<DetectorStatus>,
    opponent: DetectedOpponent,
    /// How old `opponent` is, in milliseconds - `None` before the first.
    opponent_age_ms: Option<f64>,
}

/// `GET /api/detector` - the detector's status (with the parameter values
/// it runs with) and the opponent it last found, as a `StampedBody` stamped
/// by the status topic.
pub fn detector(captain: &Captain) -> ResponseBox {
    let status = captain.try_topic::<DetectorStatus>(DETECTOR_STATUS_TOPIC_NAME);
    let opponent = captain
        .try_topic::<DetectedOpponent>(DETECTED_OPPONENT_TOPIC_NAME)
        .map(|topic| topic.read());
    let meta = status
        .as_ref()
        .map_or_else(WriteMeta::default, |topic| topic.meta());
    let body = DetectorBody {
        status: status.map(|topic| topic.read().into_value()),
        opponent_age_ms: opponent
            .as_ref()
            .and_then(|opponent| opponent.age())
            .map(|age| age.as_secs_f64() * 1000.0),
        opponent: opponent
            .map(|opponent| opponent.into_value())
            .unwrap_or_default(),
    };
    stamped_json(&body, meta)
}

/// The detector's tunable parameters with the values it currently runs
/// with - or a `400` response if no detector reports any.
fn detector_parameters(captain: &Captain) -> Result<Vec<AlgorithmParameter>, ResponseBox> {
    let parameters = captain
        .try_topic::<DetectorStatus>(DETECTOR_STATUS_TOPIC_NAME)
        .map(|topic| topic.read().into_value().parameters)
        .unwrap_or_default();
    if parameters.is_empty() {
        return Err(bad_request("no detector is running"));
    }
    Ok(parameters)
}

/// Serializes the read-modify-writes of `detector_parameters` across
/// `WebGui`'s worker threads, as the planner's own lock does.
static DETECTOR_PARAMETERS_LOCK: Mutex<()> = Mutex::new(());

/// Read-modify-writes `detector_parameters` with `change`, under
/// [`DETECTOR_PARAMETERS_LOCK`].
fn write_detector_parameters(
    captain: &Captain,
    writer_id: u16,
    change: impl FnOnce(&mut DetectorParameters),
) {
    let _guard = DETECTOR_PARAMETERS_LOCK
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    let topic = captain.topic::<DetectorParameters>(DETECTOR_PARAMETERS_TOPIC_NAME);
    let mut wanted = topic.read().into_value();
    change(&mut wanted);
    topic
        .write(writer_id, wanted)
        .expect("lost writer authorization for the detector_parameters topic");
}

#[derive(serde::Deserialize)]
struct SetDetectorParameterBody {
    name: String,
    value: f64,
}

/// `POST /api/detector_parameter` - body `{"name": "...", "value": ...}` -
/// sets one parameter's wanted value in `detector_parameters`, for the
/// detector to apply before its next scan; it reports the value it actually
/// runs with in `detector_status`.
pub fn set_detector_parameter(
    request: &mut Request,
    captain: &Captain,
    writer_id: u16,
) -> ResponseBox {
    let body: SetDetectorParameterBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    if !body.value.is_finite() {
        return bad_request("value must be a finite number");
    }
    let parameters = match detector_parameters(captain) {
        Ok(parameters) => parameters,
        Err(response) => return response,
    };
    if !parameters
        .iter()
        .any(|parameter| parameter.name == body.name)
    {
        return bad_request(&format!(
            "the detector has no tunable parameter {:?}",
            body.name
        ));
    }
    write_detector_parameters(captain, writer_id, |wanted| {
        wanted.values.insert(body.name, body.value);
    });
    json_response(&(), 200)
}

/// `POST /api/detector_parameters_save` - writes the parameter values the
/// detector currently runs with into its config file, keeping the file's
/// comments - see [`perception::save_parameters`]. Responds with the file's
/// path.
pub fn save_detector_parameters(captain: &Captain) -> ResponseBox {
    let parameters = match detector_parameters(captain) {
        Ok(parameters) => parameters,
        Err(response) => return response,
    };
    match perception::save_parameters(&parameters) {
        Ok(path) => loaded(&path),
        Err(err) => error_response(500, &err),
    }
}

/// `POST /api/detector_parameters_load` - asks the detector to run with the
/// values its config file holds again (see [`perception::saved_values`]),
/// undoing any unsaved tuning, by writing them to `detector_parameters`
/// like a slider would. Responds with the file's path.
pub fn load_detector_parameters(captain: &Captain, writer_id: u16) -> ResponseBox {
    let parameters = match detector_parameters(captain) {
        Ok(parameters) => parameters,
        Err(response) => return response,
    };
    let (values, path) = match perception::saved_values(&parameters) {
        Ok(saved) => saved,
        Err(err) => return error_response(500, &err),
    };
    write_detector_parameters(captain, writer_id, |wanted| wanted.values.extend(values));
    loaded(&path)
}
