//! The live, topic-backed API: the currently selected map's identity and
//! pixels (from the `map` topic), the vehicle's live status and model, and
//! the write endpoints a driver uses to steer it and pick its model
//! (`map_selection`, `human_vesc_command`, `vehicle_model_selection`) - as
//! opposed to [`super::maps_api`], which lists/generates map folders on
//! disk.

use super::maps_api::{bad_request, json_response, safe_map_folder};
use crate::topics::{
    HUMAN_VESC_COMMAND_TOPIC_NAME, MAP_SELECTION_TOPIC_NAME, MAP_TOPIC_NAME, MapSelection, SelectedMap,
    VEHICLE_MODEL_SELECTION_TOPIC_NAME, VEHICLE_MODEL_STATUS_TOPIC_NAME, VEHICLE_STATUS_TOPIC_NAME, VehicleModelKind,
    VehicleModelSelection, VehicleModelStatus, VehicleStatus, VescCommand,
};
use crate::Captain;
use std::path::Path;
use tiny_http::{Request, Response, ResponseBox};

/// `GET /api/map` - the currently selected map's name and dimensions (not
/// its pixels - see [`raster`]), read from the `map` topic. `name` is
/// derived from the topic's folder path, or `null` if no map is selected.
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
    json_response(&LiveMap { name, width_px: selected.width_px, height_px: selected.height_px }, 200)
}

/// `GET /api/map/raster` - the currently selected map's pixels, read from
/// the `map` topic rather than disk, in the same one-byte-per-pixel format
/// as [`super::maps_api::raster`].
pub fn raster(captain: &Captain) -> ResponseBox {
    let selected = captain.topic::<SelectedMap>(MAP_TOPIC_NAME).read();
    Response::from_data(selected.pixels)
        .with_header(super::header("Content-Type", "application/octet-stream"))
        .boxed()
}

#[derive(serde::Deserialize)]
struct SelectMapBody {
    name: Option<String>,
}

/// `POST /api/map_selection` - body `{"name": "..."}` (or `{"name": null}`
/// to deselect) - writes the wanted map folder to `map_selection`, for
/// [`crate::sensors::MapServer`] to pick up.
pub fn select_map(request: &mut Request, captain: &Captain, writer_id: u8, maps_root: &Path) -> ResponseBox {
    let body: SelectMapBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };

    let path = match &body.name {
        None => None,
        Some(name) => match safe_map_folder(name, maps_root) {
            Some(path) => Some(path),
            None => return bad_request("invalid name: must not be empty or contain '/', '\\', or '..'"),
        },
    };

    captain
        .topic::<MapSelection>(MAP_SELECTION_TOPIC_NAME)
        .write(writer_id, MapSelection { path })
        .expect("lost writer authorization for the map_selection topic");
    json_response(&(), 200)
}

/// `GET /api/vehicle_status` - the vehicle's live position, heading, and
/// speed, read from the `vehicle_status` topic.
pub fn vehicle_status(captain: &Captain) -> ResponseBox {
    json_response(&captain.topic::<VehicleStatus>(VEHICLE_STATUS_TOPIC_NAME).read(), 200)
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
        .write(writer_id, VescCommand::new(body.servo_position_rad, body.speed_mps))
        .expect("lost writer authorization for the human_vesc_command topic");
    json_response(&(), 200)
}

/// One selectable vehicle model kind, as listed by [`vehicle_models`].
#[derive(serde::Serialize)]
struct VehicleModelOption {
    kind: &'static str,
    label: &'static str,
}

/// Every [`VehicleModelKind`] paired with its API string (see [`kind_str`])
/// and a human-readable label, in the order they should appear in a picker.
const VEHICLE_MODEL_OPTIONS: &[(VehicleModelKind, &str, &str)] = &[
    (VehicleModelKind::Bicycle, "bicycle", "Kinematic bicycle"),
    (VehicleModelKind::DynamicBicycle, "dynamic_bicycle", "Dynamic bicycle (tire forces)"),
    (
        VehicleModelKind::NonlinearBicycle,
        "nonlinear_bicycle",
        "Nonlinear bicycle (tire saturation + load transfer)",
    ),
    (
        VehicleModelKind::PacejkaBicycle,
        "pacejka_bicycle",
        "Pacejka bicycle (full Magic Formula)",
    ),
];

/// The API string for `kind` - the inverse of [`parse_kind`].
fn kind_str(kind: VehicleModelKind) -> &'static str {
    VEHICLE_MODEL_OPTIONS
        .iter()
        .find(|(k, ..)| *k == kind)
        .map(|(_, s, _)| *s)
        .expect("VEHICLE_MODEL_OPTIONS lists every VehicleModelKind")
}

/// Parses an API string (as sent to [`select_vehicle_model`]) into a
/// [`VehicleModelKind`], or `None` if it names no known model.
fn parse_kind(s: &str) -> Option<VehicleModelKind> {
    VEHICLE_MODEL_OPTIONS.iter().find(|(_, k, _)| *k == s).map(|(kind, ..)| *kind)
}

/// `GET /api/vehicle_models` - every selectable vehicle model kind, for a
/// picker in the UI.
pub fn vehicle_models() -> ResponseBox {
    let options: Vec<VehicleModelOption> = VEHICLE_MODEL_OPTIONS
        .iter()
        .map(|(_, kind, label)| VehicleModelOption { kind, label })
        .collect();
    json_response(&options, 200)
}

#[derive(serde::Serialize)]
struct LiveVehicleModel {
    kind: &'static str,
}

/// `GET /api/vehicle_model` - the vehicle model kind currently running, read
/// from the `vehicle_model_status` topic.
pub fn vehicle_model(captain: &Captain) -> ResponseBox {
    let status = captain.topic::<VehicleModelStatus>(VEHICLE_MODEL_STATUS_TOPIC_NAME).read();
    json_response(&LiveVehicleModel { kind: kind_str(status.kind) }, 200)
}

#[derive(serde::Deserialize)]
struct SelectVehicleModelBody {
    kind: String,
}

/// `POST /api/vehicle_model_selection` - body `{"kind": "..."}` - writes the
/// wanted vehicle model kind to `vehicle_model_selection`, for
/// [`crate::actuators::SimulatedVehicle`] to pick up.
pub fn select_vehicle_model(request: &mut Request, captain: &Captain, writer_id: u8) -> ResponseBox {
    let body: SelectVehicleModelBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };

    let Some(kind) = parse_kind(&body.kind) else {
        return bad_request(&format!("unknown vehicle model kind: {:?}", body.kind));
    };

    captain
        .topic::<VehicleModelSelection>(VEHICLE_MODEL_SELECTION_TOPIC_NAME)
        .write(writer_id, VehicleModelSelection { kind })
        .expect("lost writer authorization for the vehicle_model_selection topic");
    json_response(&(), 200)
}

fn read_json<T: serde::de::DeserializeOwned>(request: &mut Request) -> Result<T, ResponseBox> {
    let mut body = String::new();
    if let Err(err) = request.as_reader().read_to_string(&mut body) {
        return Err(bad_request(&format!("failed to read request body: {err}")));
    }
    serde_json::from_str(&body).map_err(|err| bad_request(&format!("invalid JSON body: {err}")))
}
