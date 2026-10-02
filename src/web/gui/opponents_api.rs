//! The Opponents panel's API: the opponents running (the `opponents` topic,
//! see [`crate::simulation::OpponentsManager`]), everything the "add" form
//! offers, adding or deleting one through `opponent_requests`, and starting
//! a race through `race_start`.

use super::WebGuiConfig;
use super::slam_api::stop_mapping;
use crate::Captain;
use crate::environment::starting_grid::{self, GridSpacing};
use crate::environment::{CENTERLINE_FILE_NAME, race_lines};
use crate::simulation::opponents::validate;
use crate::topics::{
    AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME, AlgorithmParameter, AlgorithmRequirements,
    AutonomousAlgorithmStatus, AvailableAlgorithm, Color, GridSlot, MAP_TOPIC_NAME,
    OPPONENT_REQUESTS_TOPIC_NAME, OPPONENTS_TOPIC_NAME, Opponent, OpponentColor, OpponentOutcome,
    OpponentRequest, OpponentRequests, OpponentSpec, Opponents, RACE_LINE_TOPIC_NAME,
    RACE_START_TOPIC_NAME, RaceStart, Racer, SelectedMap, SelectedRaceLine,
    VEHICLE_MODEL_STATUS_TOPIC_NAME, VehicleGeometry, VehicleModelStatus, VehicleTopics, now_ms,
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
    race_start: RaceStartAvailability,
}

/// Whether a race can be started, for the "Start race" button.
#[derive(serde::Serialize)]
struct RaceStartAvailability {
    available: bool,
    /// Why not, when it can't.
    reason: Option<String>,
}

impl RaceStartAvailability {
    fn of(map: Option<&PathBuf>, files: &[String]) -> Self {
        let reason = if map.is_none() {
            Some("Load a map first.")
        } else if !files.iter().any(|file| file == CENTERLINE_FILE_NAME) {
            Some("The map has no centerline - a race needs one to line the grid up on.")
        } else {
            None
        };
        Self {
            available: reason.is_none(),
            reason: reason.map(str::to_string),
        }
    }
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

pub(super) fn opponents(captain: &Captain) -> Opponents {
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
    let race_start = RaceStartAvailability::of(map.as_ref(), &files);
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
            race_start,
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
pub(super) fn queue(captain: &Captain, writer_id: u16, request: OpponentRequest) -> u64 {
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
    if !opponents(captain)
        .list
        .iter()
        .any(|opponent| opponent.id == body.id)
    {
        return bad_request(&format!("there's no opponent {}", body.id));
    }
    let request = queue(captain, writer_id, OpponentRequest::Delete(body.id));
    json_response(&Queued { request }, 200)
}

#[derive(serde::Deserialize)]
struct StartRaceBody {
    /// Every racer - the ego vehicle and each opponent running, once each -
    /// pole position first.
    order: Vec<Racer>,
}

#[derive(serde::Serialize)]
struct RaceStarted {
    /// How long until the vehicles are released - the countdown's length.
    go_in_ms: u64,
}

/// `POST /api/race/start` - body `{"order": [...]}`, every [`Racer`] once,
/// pole position first - lines every vehicle up on the loaded map's starting
/// grid (see [`starting_grid::slots`]) and releases them all at once after
/// [`WebGuiConfig::race_countdown_ms`]. Like any placement of the ego
/// vehicle, turns SLAM's mapping off.
pub fn start_race(
    request: &mut Request,
    captain: &Captain,
    writer_id: u16,
    config: &WebGuiConfig,
) -> ResponseBox {
    let body: StartRaceBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let mut expected: Vec<Racer> = std::iter::once(Racer::Ego)
        .chain(
            opponents(captain)
                .list
                .iter()
                .map(|opponent| Racer::Opponent(opponent.id)),
        )
        .collect();
    let mut given = body.order.clone();
    expected.sort();
    given.sort();
    if given != expected {
        return bad_request(
            "The order must list the ego vehicle and every opponent running, once each - \
             the opponents may have changed, reopen the dialog.",
        );
    }

    if let Err(err) = line_up(captain, writer_id, &body.order, config) {
        return bad_request(&err);
    }
    json_response(
        &RaceStarted {
            go_in_ms: config.race_countdown_ms,
        },
        200,
    )
}

/// Lines `order` up on the loaded map's starting grid (see
/// [`starting_grid::slots`]), pole position first, and releases them all at
/// once after [`WebGuiConfig::race_countdown_ms`] - the race start, also
/// used by a benchmark with the ego vehicle alone. Like any placement of the
/// ego vehicle, turns SLAM's mapping off. Returns when the vehicles are
/// released, in [`now_ms`]'s clock.
pub(super) fn line_up(
    captain: &Captain,
    writer_id: u16,
    order: &[Racer],
    config: &WebGuiConfig,
) -> Result<u64, String> {
    let map = captain
        .topic::<SelectedMap>(MAP_TOPIC_NAME)
        .read()
        .into_value();
    let (Some(folder), Some(info)) = (&map.path, &map.info) else {
        return Err("Load a map first.".to_string());
    };
    let centerline = race_lines::read(folder, CENTERLINE_FILE_NAME).unwrap_or_default();
    let is_free = |x_m: f64, y_m: f64| {
        let col = ((x_m - info.origin.x) / info.resolution_m_per_px).floor();
        let row = ((y_m - info.origin.y) / info.resolution_m_per_px).floor();
        col >= 0.0
            && row >= 0.0
            && col < f64::from(map.width_px)
            && row < f64::from(map.height_px)
            && map.pixels[row as usize * map.width_px as usize + col as usize] == 255
    };
    // Every vehicle is the ego's size: opponents copy its model.
    let geometry = captain
        .try_topic::<VehicleGeometry>(&VehicleTopics::ego().vehicle_geometry())
        .map_or_else(VehicleGeometry::default, |topic| topic.read().into_value());
    let spacing = GridSpacing {
        body_length_m: geometry.body_length_m,
        body_width_m: geometry.body_width_m,
        gap_m: config.grid_gap_m,
        margin_m: config.grid_margin_m,
    };
    let poses = starting_grid::slots(
        &centerline,
        &info.start_finish_line,
        order.len(),
        spacing,
        is_free,
    )?;

    let go_at_ms = now_ms() + config.race_countdown_ms;
    let topic = captain.topic::<RaceStart>(RACE_START_TOPIC_NAME);
    let sequence = topic.read().sequence.wrapping_add(1);
    topic
        .write(
            writer_id,
            RaceStart {
                sequence,
                slots: order
                    .iter()
                    .zip(poses)
                    .map(|(&racer, pose)| GridSlot { racer, pose })
                    .collect(),
                go_at_ms,
            },
        )
        .expect("lost writer authorization for the race_start topic");
    stop_mapping(captain, writer_id);
    Ok(go_at_ms)
}
