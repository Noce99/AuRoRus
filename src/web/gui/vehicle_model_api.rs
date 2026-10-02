//! The vehicle-model part of the live API: which physics model simulates
//! the vehicle, and tuning its parameters and the actuator limits
//! (`vehicle_model_selection`, `vehicle_model_parameters`).

use super::live_api::{SavedParameters, loaded, stamped_json};
use crate::simulation;
use crate::topics::{
    ActuatorLimits, AlgorithmParameter, VEHICLE_MODEL_PARAMETERS_TOPIC_NAME,
    VEHICLE_MODEL_SELECTION_TOPIC_NAME, VEHICLE_MODEL_STATUS_TOPIC_NAME, VehicleModelKind,
    VehicleModelParameters, VehicleModelSelection, VehicleModelStatus,
};
use crate::web::{bad_request, error_response, json_response, read_json};
use crate::{Captain, Stamped};
use std::sync::Mutex;
use tiny_http::{Request, ResponseBox};

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
/// read from the `vehicle_model_status` topic, as a [`StampedBody`](super::live_api::StampedBody).
pub fn vehicle_model(captain: &Captain) -> ResponseBox {
    let Some(status) = running_model(captain) else {
        return error_response(404, "no simulated vehicle model is running");
    };
    stamped_json(
        &LiveVehicleModel {
            kind: status.kind.api_str(),
            parameters: status.value.parameters,
            limits: status.value.limits,
        },
        status.meta,
    )
}

/// What the running vehicle model publishes on `vehicle_model_status` -
/// `None` on the real car, which runs no model.
fn running_model(captain: &Captain) -> Option<Stamped<VehicleModelStatus>> {
    captain
        .try_topic::<VehicleModelStatus>(VEHICLE_MODEL_STATUS_TOPIC_NAME)
        .map(|topic| topic.read())
}

#[derive(serde::Deserialize)]
struct SelectVehicleModelBody {
    kind: String,
}

/// `POST /api/vehicle_model_selection` - body `{"kind": "..."}` - writes the
/// wanted vehicle model kind to `vehicle_model_selection`, for
/// [`crate::simulation::SimulatedVehicle`] to pick up.
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
/// `vehicle_model_parameters`, like [`autonomous_api`](super::autonomous_api)'s `AUTONOMOUS_PARAMETERS_LOCK`.
static VEHICLE_MODEL_PARAMETERS_LOCK: Mutex<()> = Mutex::new(());

/// `POST /api/vehicle_model_parameter` - body `{"kind": "...", "name":
/// "...", "value": ...}` - sets one parameter's wanted value in
/// `vehicle_model_parameters`, for [`crate::simulation::SimulatedVehicle`]
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
/// [`simulation::save_vehicle_model_parameters`]. They're used from the next
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
    match simulation::save_vehicle_model_parameters(kind, &parameters) {
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
/// [`crate::simulation::SimulatedVehicle`] to apply whichever model is
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
/// [`simulation::save_vehicle_limits`]. Responds with the file's path.
pub fn save_vehicle_limits(captain: &Captain) -> ResponseBox {
    let Some(status) = running_model(captain) else {
        return bad_request("no simulated vehicle model is running");
    };
    let limits = status.into_value().limits;
    if limits.is_empty() {
        return bad_request("no actuator limits reported yet");
    }
    match simulation::save_vehicle_limits(&limits) {
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
    let (values, path) = match simulation::saved_vehicle_model_values(kind, &parameters) {
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
    let (values, path) =
        match simulation::saved_vehicle_limits(&ActuatorLimits::tunable_parameters()) {
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
    let Some(status) = running_model(captain) else {
        return Err(bad_request("no simulated vehicle model is running"));
    };
    let status = status.into_value();
    if status.kind.api_str() != kind {
        return Err(bad_request(&format!(
            "{kind:?} isn't the running vehicle model - only that one can be tuned"
        )));
    }
    Ok((status.kind, status.parameters))
}
