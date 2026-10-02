//! The planning part of the live API: tuning the race line planner and
//! asking it for a line (`planning_parameters`, `planning_request`).

use super::live_api::{loaded, stamped_json};
use crate::planning;
use crate::topics::{
    AlgorithmParameter, MAP_TOPIC_NAME, PLANNING_PARAMETERS_TOPIC_NAME,
    PLANNING_REQUEST_TOPIC_NAME, PLANNING_STATUS_TOPIC_NAME, PlanningObjective, PlanningParameters,
    PlanningRequest, PlanningState, PlanningStatus, SelectedMap,
};
use crate::web::{bad_request, error_response, json_response, read_json};
use crate::{Captain, WriteMeta};
use std::sync::Mutex;
use tiny_http::{Request, ResponseBox};

/// `GET /api/planning` - what [`crate::planning::Planner`] is doing, the
/// parameter values it runs with, and how its latest request went, read
/// from the `planning_status` topic, as a [`StampedBody`](super::live_api::StampedBody). The default
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
/// `WebGui`'s worker threads, as [`autonomous_api`](super::autonomous_api)'s `AUTONOMOUS_PARAMETERS_LOCK`.
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
