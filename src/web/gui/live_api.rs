//! The live, topic-backed control API: the endpoints that aren't specific
//! to one subsystem - the frontend's config, which map is selected
//! (`map`, `map_selection`), driving by hand (`human_vesc_command`),
//! placing the vehicle (`place_at_start`), restarting, and the lap
//! telemetry and actuator status read-outs - plus the response helpers the
//! per-subsystem APIs share ([`stamped_json`], [`loaded`]).
//!
//! The rest of the live API is split by subsystem: [`super::vesc_api`],
//! [`super::vehicle_model_api`], [`super::autonomous_api`],
//! [`super::slam_api`] and [`super::planning_api`] - as opposed to
//! [`super::maps_api`], which lists/generates map folders on disk, and
//! [`super::draw_api`], which serves what's drawn on the map.

use super::WebGuiConfig;
use super::maps_api::safe_map_folder;
use super::slam_api::{stop_mapping, write_slam_command};
use crate::topics::{
    ACTUATOR_STATUS_TOPIC_NAME, ActuatorStatus, HUMAN_VESC_COMMAND_TOPIC_NAME,
    LAP_TELEMETRY_TOPIC_NAME, LapTelemetry, MAP_SELECTION_TOPIC_NAME, MAP_TOPIC_NAME, MapSelection,
    PLACE_AT_START_TOPIC_NAME, PlaceAtStart, SelectedMap, SlamState, StartState,
    VESC_STATUS_TOPIC_NAME, VescCommand, VescStatus,
};
use crate::web::{bad_request, json_response, read_json, read_optional_json};
use crate::{Captain, WriteMeta};
use std::path::Path;
use tiny_http::{Request, ResponseBox};

/// The subset of [`WebGuiConfig`] the frontend needs, as served by
/// [`config`].
#[derive(serde::Serialize)]
struct FrontendConfig {
    human_max_speed_mps: f64,
    human_max_steering_rad: f64,
    /// Whether this is the real car (`web_gui` on a car): its VESC drives the ego
    /// vehicle, and nothing simulates one.
    hardware: bool,
    /// The real car's name, shown on the map - `None` in simulation.
    car_name: Option<String>,
}

/// `GET /api/config` - frontend-facing config values (the WASD human
/// control limits, and whether this is the real car), so the UI and server
/// never drift apart.
pub fn config(config: &WebGuiConfig, captain: &Captain, car_name: Option<&str>) -> ResponseBox {
    json_response(
        &FrontendConfig {
            human_max_speed_mps: config.human_max_speed_mps,
            human_max_steering_rad: config.human_max_steering_rad,
            hardware: captain
                .try_topic::<VescStatus>(VESC_STATUS_TOPIC_NAME)
                .is_some(),
            car_name: car_name.map(str::to_string),
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
pub(super) struct StampedBody<'a, T> {
    value: &'a T,
    written_at_unix_us: u64,
    age_ms: Option<f64>,
    write_count: u64,
}

/// Serializes `value` (derived from a topic read) wrapped in a
/// [`StampedBody`] carrying that read's `meta`.
pub(super) fn stamped_json<T: serde::Serialize>(value: &T, meta: WriteMeta) -> ResponseBox {
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
/// [`crate::environment::MapServer`] to pick up. Also turns SLAM off: a map
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

    let selection_topic = captain.topic::<MapSelection>(MAP_SELECTION_TOPIC_NAME);
    let revision = selection_topic.read().revision;
    selection_topic
        .write(writer_id, MapSelection { path, revision })
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

/// What a successful save to, or load from, a config file responds with.
#[derive(serde::Serialize)]
pub(super) struct SavedParameters {
    pub(super) path: String,
}

/// The response to a successful load from the config file at `path`.
pub(super) fn loaded(path: &Path) -> ResponseBox {
    json_response(
        &SavedParameters {
            path: path.display().to_string(),
        },
        200,
    )
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

#[derive(serde::Deserialize)]
struct PlaceAtPoseBody {
    x_m: f64,
    y_m: f64,
    heading_rad: f64,
}

/// `POST /api/place_at_start` - bumps `place_at_start`'s counter, asking
/// [`crate::simulation::SimulatedVehicle`] to place the vehicle at whatever
/// `start_state` currently holds, e.g. from the "P" keyboard shortcut - or,
/// given a body `{"x_m", "y_m", "heading_rad"}`, to place the ego vehicle
/// (only - opponents stay put) at rest at that pose instead, e.g. from the
/// map's place-vehicle tool. Unlike [`restart`], this doesn't tear anything
/// down - just resets the vehicle's simulated position/heading/speed in
/// place. Also turns SLAM's mapping off: the vehicle jumps, and dead
/// reckoning - whose frame the map is built in - resets with it.
/// Localization carries on: SLAM restarts it from wherever the vehicle was
/// placed by itself when dead reckoning resets.
pub fn place_at_start(request: &mut Request, captain: &Captain, writer_id: u16) -> ResponseBox {
    let body: Option<PlaceAtPoseBody> = match read_optional_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let pose = match body {
        None => None,
        Some(body) => {
            if ![body.x_m, body.y_m, body.heading_rad]
                .iter()
                .all(|value| value.is_finite())
            {
                return bad_request("x_m, y_m and heading_rad must be finite");
            }
            Some(StartState {
                x_m: body.x_m,
                y_m: body.y_m,
                heading_rad: body.heading_rad,
                speed_mps: 0.0,
            })
        }
    };
    let topic = captain.topic::<PlaceAtStart>(PLACE_AT_START_TOPIC_NAME);
    let requested = topic.read().requested.wrapping_add(1);
    topic
        .write(writer_id, PlaceAtStart { requested, pose })
        .expect("lost writer authorization for the place_at_start topic");
    stop_mapping(captain, writer_id);
    json_response(&(), 200)
}

/// `GET /api/lap_telemetry` - the ego vehicle's laps, read from the
/// `lap_telemetry` topic (see [`crate::telemetry::LapTelemetryRecorder`]),
/// as a [`StampedBody`]. The default (empty) telemetry if nothing publishes
/// that topic.
pub fn lap_telemetry(captain: &Captain) -> ResponseBox {
    match captain.try_topic::<LapTelemetry>(LAP_TELEMETRY_TOPIC_NAME) {
        Some(topic) => {
            let telemetry = topic.read();
            stamped_json(&telemetry.value, telemetry.meta)
        }
        None => stamped_json(&LapTelemetry::default(), WriteMeta::default()),
    }
}

/// `GET /api/actuator_status` - the ego vehicle's steering angle and speed
/// right now, read from the `actuator_status` topic (published by the
/// simulated vehicle or the real car's VESC alike), as a [`StampedBody`].
/// The default (never written) status if nothing publishes that topic.
pub fn actuator_status(captain: &Captain) -> ResponseBox {
    match captain.try_topic::<ActuatorStatus>(ACTUATOR_STATUS_TOPIC_NAME) {
        Some(topic) => {
            let status = topic.read();
            stamped_json(&status.value, status.meta)
        }
        None => stamped_json(&ActuatorStatus::default(), WriteMeta::default()),
    }
}
