//! The SLAM part of the live API: starting, pausing and resetting mapping
//! and localization, and saving the map built (`slam_command`,
//! `slam_save`).

use super::live_api::stamped_json;
use crate::topics::{
    SLAM_COMMAND_TOPIC_NAME, SLAM_SAVE_TOPIC_NAME, SLAM_STATUS_TOPIC_NAME, SlamCommand,
    SlamSaveRequest, SlamState, SlamStatus,
};
use crate::web::{bad_request, json_response, read_json};
use crate::{Captain, WriteMeta};
use tiny_http::{Request, ResponseBox};

/// Turns SLAM's mapping off, as any placement of the ego vehicle must: it
/// jumps, and dead reckoning - whose frame the map is built in - resets with
/// it. Localization carries on.
pub(super) fn stop_mapping(captain: &Captain, writer_id: u16) {
    let slam_state = captain
        .topic::<SlamCommand>(SLAM_COMMAND_TOPIC_NAME)
        .read()
        .state;
    if !slam_state.is_localization() {
        write_slam_command(captain, writer_id, SlamState::Off);
    }
}

/// `GET /api/slam` - what [`crate::localization::Slam`] is doing, read from
/// the `slam_status` topic, as a [`StampedBody`](super::live_api::StampedBody). The default status (off,
/// no scans) if nothing publishes that topic.
pub fn slam(captain: &Captain) -> ResponseBox {
    match captain.try_topic::<SlamStatus>(SLAM_STATUS_TOPIC_NAME) {
        Some(topic) => {
            let status = topic.read();
            stamped_json(&status.value, status.meta)
        }
        None => stamped_json(&SlamStatus::default(), WriteMeta::default()),
    }
}

#[derive(serde::Deserialize)]
struct SlamCommandBody {
    state: String,
}

/// `POST /api/slam_command` - body `{"state": "running" | "waiting" |
/// "off" | "localizing" | "localization_paused"}` - writes the wanted state to `slam_command`, for
/// [`crate::localization::Slam`] to pick up. `"off"` clears SLAM's map.
pub fn slam_command(request: &mut Request, captain: &Captain, writer_id: u16) -> ResponseBox {
    let body: SlamCommandBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let Some(state) = SlamState::from_api_str(&body.state) else {
        return bad_request(&format!("unknown SLAM state: {:?}", body.state));
    };
    write_slam_command(captain, writer_id, state);
    json_response(&(), 200)
}

#[derive(serde::Deserialize)]
struct SlamSaveBody {
    name: String,
}

/// The response to a [`slam_save`]: the request number whose outcome to
/// look for in [`SlamStatus::last_save`].
#[derive(serde::Serialize)]
struct SlamSaveResponse {
    requested: u64,
}

/// `POST /api/slam_save` - body `{"name": "..."}` - asks
/// [`crate::localization::Slam`] to save its map as a new map folder named
/// `name`. SLAM saves it on its next tick, and reports how it went on
/// `slam_status`'s `last_save`, under the returned request number.
pub fn slam_save(request: &mut Request, captain: &Captain, writer_id: u16) -> ResponseBox {
    let body: SlamSaveBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let topic = captain.topic::<SlamSaveRequest>(SLAM_SAVE_TOPIC_NAME);
    let requested = topic.read().requested.wrapping_add(1);
    topic
        .write(
            writer_id,
            SlamSaveRequest {
                name: body.name,
                requested,
            },
        )
        .expect("lost writer authorization for the slam_save topic");
    json_response(&SlamSaveResponse { requested }, 200)
}

/// Writes `state` to `slam_command`, bumping
/// [`SlamCommand::clear_requested`] when it's [`SlamState::Off`] - so SLAM
/// clears its map even if it never sees `Off` itself, e.g. when Play follows
/// within one of its ticks.
pub(super) fn write_slam_command(captain: &Captain, writer_id: u16, state: SlamState) {
    let topic = captain.topic::<SlamCommand>(SLAM_COMMAND_TOPIC_NAME);
    let mut command = topic.read().into_value();
    command.state = state;
    if state == SlamState::Off {
        command.clear_requested = command.clear_requested.wrapping_add(1);
    }
    topic
        .write(writer_id, command)
        .expect("lost writer authorization for the slam_command topic");
}
