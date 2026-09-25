//! The Race Lines panel's API: every race line of the map currently loaded
//! (the `map` topic), described from its folder on disk (see
//! [`crate::environment::race_lines`]), which one `MapServer` publishes on
//! `race_line`, and picking another one through `race_line_selection`.

use crate::Captain;
use crate::environment::{RaceLineEntry, race_lines};
use crate::topics::{
    MAP_TOPIC_NAME, RACE_LINE_SELECTION_TOPIC_NAME, RACE_LINE_TOPIC_NAME, RaceLineSelection,
    SelectedMap, SelectedRaceLine,
};
use crate::web::{bad_request, json_response, read_json};
use std::path::PathBuf;
use tiny_http::{Request, ResponseBox};

#[derive(serde::Serialize)]
struct RaceLines {
    /// Name of the loaded map, or `null` with none loaded.
    map: Option<String>,
    /// The line `MapServer` currently publishes, or `null` if none of the
    /// loaded map's.
    selected: Option<String>,
    /// Newest first.
    lines: Vec<RaceLineEntry>,
}

/// The folder of the map the `map` topic currently holds.
fn loaded_map(captain: &Captain) -> Option<PathBuf> {
    captain
        .topic::<SelectedMap>(MAP_TOPIC_NAME)
        .read()
        .into_value()
        .path
}

/// `GET /api/race_lines` - the loaded map's race lines, and which one is
/// followed.
pub fn list(captain: &Captain) -> ResponseBox {
    let Some(folder) = loaded_map(captain) else {
        return json_response(
            &RaceLines {
                map: None,
                selected: None,
                lines: Vec::new(),
            },
            200,
        );
    };
    let published = captain
        .try_topic::<SelectedRaceLine>(RACE_LINE_TOPIC_NAME)
        .map(|topic| topic.read().into_value())
        .filter(|line| line.map.as_ref() == Some(&folder));
    json_response(
        &RaceLines {
            map: folder
                .file_name()
                .and_then(|n| n.to_str())
                .map(str::to_string),
            selected: published.and_then(|line| line.file),
            lines: race_lines::list(&folder),
        },
        200,
    )
}

#[derive(serde::Deserialize)]
struct SelectBody {
    file: String,
}

/// `POST /api/race_line_selection` - body `{"file": "..."}` - asks
/// `MapServer` to follow that race line of the loaded map.
pub fn select(request: &mut Request, captain: &Captain, writer_id: u8) -> ResponseBox {
    let body: SelectBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let Some(folder) = loaded_map(captain) else {
        return bad_request("no map is loaded");
    };
    if let Err(err) = race_lines::read(&folder, &body.file) {
        return bad_request(&err.to_string());
    }
    captain
        .topic::<RaceLineSelection>(RACE_LINE_SELECTION_TOPIC_NAME)
        .write(
            writer_id,
            RaceLineSelection {
                map: Some(folder),
                file: body.file,
            },
        )
        .expect("lost writer authorization for the race_line_selection topic");
    json_response(&(), 200)
}
