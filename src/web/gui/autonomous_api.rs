//! The autonomous part of the live API: which algorithm drives, and tuning
//! its parameters (`autonomous_algorithm_selection`,
//! `autonomous_parameters`).

use super::live_api::{SavedParameters, loaded, stamped_json};
use crate::autonomous_control;
use crate::topics::{
    AUTONOMOUS_ALGORITHM_SELECTION_TOPIC_NAME, AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME,
    AUTONOMOUS_PARAMETERS_TOPIC_NAME, AlgorithmParameter, AutonomousAlgorithmSelection,
    AutonomousAlgorithmStatus, AutonomousParameters,
};
use crate::web::{bad_request, error_response, json_response, read_json};
use crate::{Captain, WriteMeta};
use std::sync::Mutex;
use tiny_http::{Request, ResponseBox};

/// `GET /api/autonomous_algorithms` - every autonomous algorithm found, the
/// one in control, and whether its command is fresh, read from the
/// `autonomous_algorithm_status` topic, as a [`StampedBody`](super::live_api::StampedBody). Empty if
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
