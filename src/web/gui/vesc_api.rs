//! The VESC part of the live API: what the real car's motor controller
//! reports, and tuning how the car is driven (`vesc_status`,
//! `vesc_parameters`).

use super::live_api::{loaded, stamped_json};
use crate::actuators;
use crate::topics::{
    AlgorithmParameter, VESC_PARAMETERS_STATUS_TOPIC_NAME, VESC_PARAMETERS_TOPIC_NAME,
    VESC_STATUS_TOPIC_NAME, VescParameters, VescParametersStatus, VescStatus,
};
use crate::web::{bad_request, error_response, json_response, read_json};
use crate::{Captain, Stamped};
use std::sync::Mutex;
use tiny_http::{Request, ResponseBox};

/// `GET /api/vesc` - the real car's motor controller's state, from the
/// `vesc_status` topic, as a [`StampedBody`](super::live_api::StampedBody) - `404` in simulation, where
/// nothing publishes it.
pub fn vesc(captain: &Captain) -> ResponseBox {
    match captain.try_topic::<VescStatus>(VESC_STATUS_TOPIC_NAME) {
        Some(topic) => {
            let status = topic.read();
            stamped_json(&status.value, status.meta)
        }
        None => error_response(404, "no VESC in this binary"),
    }
}

/// `GET /api/vesc_parameters` - the real car's calibration and actuator
/// limits, with the values in effect, from the `vesc_parameters_status`
/// topic, as a [`StampedBody`](super::live_api::StampedBody) - `404` in simulation.
pub fn vesc_parameters(captain: &Captain) -> ResponseBox {
    match vesc_parameters_status(captain) {
        Some(status) => stamped_json(&status.value, status.meta),
        None => error_response(404, "no VESC in this binary"),
    }
}

fn vesc_parameters_status(captain: &Captain) -> Option<Stamped<VescParametersStatus>> {
    captain
        .try_topic::<VescParametersStatus>(VESC_PARAMETERS_STATUS_TOPIC_NAME)
        .map(|topic| topic.read())
}

/// Which of the VESC's two parameter groups a request is about: `vesc.toml`'s
/// top-level values, or its `[limits]` table.
#[derive(serde::Deserialize, Clone, Copy, PartialEq)]
#[serde(rename_all = "snake_case")]
enum VescGroup {
    Calibration,
    Limits,
}

impl VescGroup {
    fn is_limits(self) -> bool {
        self == Self::Limits
    }

    /// This group's parameters, with the values in effect, from `status`.
    fn parameters(self, status: VescParametersStatus) -> Vec<AlgorithmParameter> {
        match self {
            Self::Calibration => status.parameters,
            Self::Limits => status.limits,
        }
    }

    /// This group's wanted values, in `wanted`.
    fn wanted(self, wanted: &mut VescParameters) -> &mut std::collections::BTreeMap<String, f64> {
        match self {
            Self::Calibration => &mut wanted.values,
            Self::Limits => &mut wanted.limits,
        }
    }
}

#[derive(serde::Deserialize)]
struct SetVescParameterBody {
    group: VescGroup,
    name: String,
    value: f64,
}

#[derive(serde::Deserialize)]
struct VescGroupBody {
    group: VescGroup,
}

/// Serializes the read-modify-writes of `vesc_parameters`, like
/// [`vehicle_model_api`](super::vehicle_model_api)'s `VEHICLE_MODEL_PARAMETERS_LOCK`.
static VESC_PARAMETERS_LOCK: Mutex<()> = Mutex::new(());

/// Read-modify-writes `vesc_parameters` with `change`, under
/// [`VESC_PARAMETERS_LOCK`].
fn write_vesc_parameters(
    captain: &Captain,
    writer_id: u16,
    change: impl FnOnce(&mut VescParameters),
) {
    let _guard = VESC_PARAMETERS_LOCK
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    let topic = captain.topic::<VescParameters>(VESC_PARAMETERS_TOPIC_NAME);
    let mut wanted = topic.read().into_value();
    change(&mut wanted);
    topic
        .write(writer_id, wanted)
        .expect("lost writer authorization for the vesc_parameters topic");
}

/// `group`'s parameters with the values in effect, or a `400` response in
/// simulation.
fn vesc_group_parameters(
    captain: &Captain,
    group: VescGroup,
) -> Result<Vec<AlgorithmParameter>, ResponseBox> {
    match vesc_parameters_status(captain) {
        Some(status) => Ok(group.parameters(status.into_value())),
        None => Err(bad_request("no VESC in this binary")),
    }
}

/// `POST /api/vesc_parameter` - body `{"group": "calibration"|"limits",
/// "name": "...", "value": ...}` - sets one value's wanted value in
/// `vesc_parameters`, for [`crate::actuators::Vesc`] to apply, which reports
/// the value in effect in `vesc_parameters_status` (unchanged if it refused
/// it - see [`crate::actuators::VescConfig::validate`]).
pub fn set_vesc_parameter(request: &mut Request, captain: &Captain, writer_id: u16) -> ResponseBox {
    let body: SetVescParameterBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    if !body.value.is_finite() {
        return bad_request("value must be a finite number");
    }
    let parameters = match vesc_group_parameters(captain, body.group) {
        Ok(parameters) => parameters,
        Err(response) => return response,
    };
    if !parameters
        .iter()
        .any(|parameter| parameter.name == body.name)
    {
        return bad_request(&format!("the VESC has no tunable {:?}", body.name));
    }
    write_vesc_parameters(captain, writer_id, |wanted| {
        body.group.wanted(wanted).insert(body.name, body.value);
    });
    json_response(&(), 200)
}

/// `POST /api/vesc_parameters_save` - body `{"group": ...}` - writes the
/// group's values in effect into `config/actuators/vesc.toml`, keeping the
/// rest of the file - see [`actuators::vesc::save_parameters`]. Responds
/// with the file's path.
pub fn save_vesc_parameters(request: &mut Request, captain: &Captain) -> ResponseBox {
    let body: VescGroupBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let parameters = match vesc_group_parameters(captain, body.group) {
        Ok(parameters) => parameters,
        Err(response) => return response,
    };
    match actuators::vesc::save_parameters(body.group.is_limits(), &parameters) {
        Ok(path) => loaded(&path),
        Err(err) => error_response(500, &err),
    }
}

/// `POST /api/vesc_parameters_load` - body `{"group": ...}` - asks the VESC
/// to drive with the group's values in `config/actuators/vesc.toml` again,
/// undoing any unsaved tuning. Responds with the file's path.
pub fn load_vesc_parameters(
    request: &mut Request,
    captain: &Captain,
    writer_id: u16,
) -> ResponseBox {
    let body: VescGroupBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let parameters = match vesc_group_parameters(captain, body.group) {
        Ok(parameters) => parameters,
        Err(response) => return response,
    };
    let (values, path) = match actuators::vesc::saved_values(body.group.is_limits(), &parameters) {
        Ok(saved) => saved,
        Err(err) => return error_response(500, &err),
    };
    write_vesc_parameters(captain, writer_id, |wanted| {
        body.group.wanted(wanted).extend(values)
    });
    loaded(&path)
}
