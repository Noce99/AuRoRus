//! The Opponents panel's API: the opponents running (the `opponents` topic,
//! see [`crate::opponents::OpponentsManager`]), everything the "add" form
//! offers, and adding or deleting one through `opponent_requests`.

use crate::Captain;
use crate::environment::race_lines;
use crate::opponents::validate;
use crate::topics::{
    AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME, AlgorithmParameter, AlgorithmRequirements,
    AutonomousAlgorithmStatus, AvailableAlgorithm, Color, MAP_TOPIC_NAME,
    OPPONENT_REQUESTS_TOPIC_NAME, OPPONENTS_TOPIC_NAME, Opponent, OpponentColor,
    OpponentOutcome, OpponentRequest, OpponentRequests, OpponentSpec, Opponents,
    RACE_LINE_TOPIC_NAME, SelectedMap, SelectedRaceLine, VEHICLE_MODEL_STATUS_TOPIC_NAME,
    VehicleModelStatus,
};
use crate::web::{bad_request, json_response, read_json};
use std::path::PathBuf;
use tiny_http::{Request, ResponseBox};

/// One color of the palette, as the form shows it.
#[derive(serde::Serialize)]
struct PaletteColor {
    /// What an [`OpponentSpec`] names it, e.g. `"red"`.
    name: OpponentColor,
    /// CSS color, e.g. `"#ff3b3b"`.
    css: String,
}

/// One algorithm an opponent can run.
#[derive(serde::Serialize)]
struct AlgorithmChoice {
    name: String,
    label: String,
    requires: AlgorithmRequirements,
}

/// The race lines an opponent can follow.
#[derive(serde::Serialize)]
struct RaceLineChoices {
    /// Name of the loaded map, or `null` with none loaded.
    map: Option<String>,
    /// The loaded map's race lines' files, newest first.
    files: Vec<String>,
    /// The one the ego vehicle follows, if it's one of `files`.
    selected: Option<String>,
}

#[derive(serde::Serialize)]
struct OpponentsResponse {
    list: Vec<Opponent>,
    last_outcome: Option<OpponentOutcome>,
    palette: Vec<PaletteColor>,
    algorithms: Vec<AlgorithmChoice>,
    race_lines: RaceLineChoices,
    /// Every actuator limit, with its range and the value the ego vehicle
    /// runs with - what a new opponent's limits default to.
    limits: Vec<AlgorithmParameter>,
}

fn css(color: Color) -> String {
    format!("#{:02x}{:02x}{:02x}", color.r, color.g, color.b)
}

/// The folder of the map the `map` topic currently holds.
fn loaded_map(captain: &Captain) -> Option<PathBuf> {
    captain
        .topic::<SelectedMap>(MAP_TOPIC_NAME)
        .read()
        .into_value()
        .path
}

/// The loaded map's race line files, newest first - none without a map.
fn race_line_files(map: Option<&PathBuf>) -> Vec<String> {
    map.map(|folder| {
        race_lines::list(folder)
            .into_iter()
            .map(|entry| entry.file)
            .collect()
    })
    .unwrap_or_default()
}

fn available_algorithms(captain: &Captain) -> Vec<AvailableAlgorithm> {
    captain
        .try_topic::<AutonomousAlgorithmStatus>(AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME)
        .map(|topic| topic.read().into_value().available)
        .unwrap_or_default()
}

fn opponents(captain: &Captain) -> Opponents {
    captain
        .try_topic::<Opponents>(OPPONENTS_TOPIC_NAME)
        .map(|topic| topic.read().into_value())
        .unwrap_or_default()
}

/// `GET /api/opponents` - the opponents running, how the latest request
/// went, and every choice the "add" form offers.
pub fn list(captain: &Captain) -> ResponseBox {
    let map = loaded_map(captain);
    let files = race_line_files(map.as_ref());
    let selected = captain
        .try_topic::<SelectedRaceLine>(RACE_LINE_TOPIC_NAME)
        .and_then(|topic| topic.read().into_value().file)
        .filter(|file| files.contains(file));
    let limits = captain
        .try_topic::<VehicleModelStatus>(VEHICLE_MODEL_STATUS_TOPIC_NAME)
        .map(|topic| topic.read().into_value().limits)
        .unwrap_or_default();
    let Opponents { list, last_outcome } = opponents(captain);
    json_response(
        &OpponentsResponse {
            list,
            last_outcome,
            palette: OpponentColor::ALL
                .into_iter()
                .map(|name| PaletteColor {
                    name,
                    css: css(name.color()),
                })
                .collect(),
            algorithms: available_algorithms(captain)
                .into_iter()
                .map(|algorithm| AlgorithmChoice {
                    name: algorithm.name,
                    label: algorithm.label,
                    requires: algorithm.requires,
                })
                .collect(),
            race_lines: RaceLineChoices {
                map: map
                    .as_ref()
                    .and_then(|folder| folder.file_name())
                    .and_then(|name| name.to_str())
                    .map(str::to_string),
                files,
                selected,
            },
            limits,
        },
        200,
    )
}

#[derive(serde::Serialize)]
struct Queued {
    /// The request's number - its outcome shows up as the `last_outcome`
    /// with this `request`.
    request: u64,
}

/// Appends `request` to `opponent_requests`, returning its number.
fn queue(captain: &Captain, writer_id: u16, request: OpponentRequest) -> u64 {
    let topic = captain.topic::<OpponentRequests>(OPPONENT_REQUESTS_TOPIC_NAME);
    let mut requests = topic.read().into_value();
    let number = requests.push(request);
    topic
        .write(writer_id, requests)
        .expect("lost writer authorization for the opponent_requests topic");
    number
}

/// `POST /api/opponents` - body an [`OpponentSpec`] - asks for a new
/// opponent. Refused right away if the spec is invalid (see
/// [`validate`]); the manager checks it again when it spawns it.
pub fn add(request: &mut Request, captain: &Captain, writer_id: u16) -> ResponseBox {
    let spec: OpponentSpec = match read_json(request) {
        Ok(spec) => spec,
        Err(response) => return response,
    };
    let files = race_line_files(loaded_map(captain).as_ref());
    if let Err(err) = validate(&spec, &available_algorithms(captain), &files) {
        return bad_request(&err);
    }
    let request = queue(captain, writer_id, OpponentRequest::Add(spec));
    json_response(&Queued { request }, 200)
}

#[derive(serde::Deserialize)]
struct DeleteBody {
    id: u32,
}

/// `POST /api/opponents/delete` - body `{"id": ...}` - asks for that
/// opponent to be deleted.
pub fn delete(request: &mut Request, captain: &Captain, writer_id: u16) -> ResponseBox {
    let body: DeleteBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    if !opponents(captain).list.iter().any(|opponent| opponent.id == body.id) {
        return bad_request(&format!("there's no opponent {}", body.id));
    }
    let request = queue(captain, writer_id, OpponentRequest::Delete(body.id));
    json_response(&Queued { request }, 200)
}
