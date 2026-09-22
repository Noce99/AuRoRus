//! The read-only JSON/binary API a recorded [`Session`] is served through -
//! the playback-mode counterpart of `web_gui`'s `live_api`. Everything here
//! is `GET`: there's nothing to write, just a fixed recording to read back.

use crate::session::Session;
use aurorus::topics::VehicleModelKind;
use aurorus::web::{header, json_response, not_found};
use tiny_http::{Response, ResponseBox};

#[derive(serde::Serialize)]
struct TopicSummary {
    name: String,
    color_index: usize,
}

#[derive(serde::Serialize)]
struct ExecutorSummary {
    name: String,
    topics: Vec<TopicSummary>,
}

#[derive(serde::Serialize)]
struct MapSummary<'a> {
    name: Option<&'a str>,
    width_px: u32,
    height_px: u32,
    info: Option<&'a aurorus::environment::MapInfo>,
}

#[derive(serde::Serialize)]
struct SessionSummary<'a> {
    frequency_hz: f64,
    duration_us: u64,
    executors: Vec<ExecutorSummary>,
    map: Option<MapSummary<'a>>,
    vehicle_model: Option<&'static str>,
    /// The same kind's human-readable label, so the frontend can display it
    /// without keeping its own copy of the mapping - see
    /// [`VehicleModelKind::ALL`].
    vehicle_model_label: Option<&'static str>,
}

/// `GET /api/session` - everything the frontend needs up front except the
/// per-topic tick timestamps ([`timeline`]) and the map's raw pixels
/// ([`map_raster`]): executor/topic names (for the timeline's row/legend
/// labels), the static map's identity/metadata, and the static vehicle model.
pub fn session(session: &Session) -> ResponseBox {
    let executors = session
        .executors
        .iter()
        .map(|executor| ExecutorSummary {
            name: executor.name.clone(),
            topics: executor
                .topics
                .iter()
                .map(|topic| TopicSummary { name: topic.name.clone(), color_index: topic.color_index })
                .collect(),
        })
        .collect();
    let map = session.map.as_ref().map(|map| MapSummary {
        name: map.name.as_deref(),
        width_px: map.width_px,
        height_px: map.height_px,
        info: map.info.as_ref(),
    });
    json_response(
        &SessionSummary {
            frequency_hz: session.frequency_hz,
            duration_us: session.duration_us,
            executors,
            map,
            vehicle_model: session.vehicle_model_kind.map(VehicleModelKind::api_str),
            vehicle_model_label: session.vehicle_model_kind.map(VehicleModelKind::label),
        },
        200,
    )
}

/// `GET /api/map/raster` - the static map's raw pixel bytes, one byte per
/// pixel, same shape as `web_gui`'s live `/api/map/raster`.
pub fn map_raster(session: &Session) -> ResponseBox {
    match &session.map {
        Some(map) => {
            let len = map.pixels.len();
            Response::new(
                tiny_http::StatusCode(200),
                vec![header("Content-Type", "application/octet-stream")],
                std::io::Cursor::new(map.pixels.clone()),
                Some(len),
                None,
            )
            .boxed()
        }
        None => not_found(),
    }
}

#[derive(serde::Serialize)]
struct TimelineTopic<'a> {
    name: &'a str,
    color_index: usize,
    timestamps_us: &'a [u64],
}

#[derive(serde::Serialize)]
struct TimelineExecutor<'a> {
    name: &'a str,
    topics: Vec<TimelineTopic<'a>>,
}

/// `GET /api/timeline` - every recorded change's timestamp, grouped the same
/// way as [`session`] - fetched once at load so the timeline never round-trips
/// per frame while zooming/scrubbing.
pub fn timeline(session: &Session) -> ResponseBox {
    let executors: Vec<TimelineExecutor> = session
        .executors
        .iter()
        .map(|executor| TimelineExecutor {
            name: &executor.name,
            topics: executor
                .topics
                .iter()
                .map(|topic| TimelineTopic { name: &topic.name, color_index: topic.color_index, timestamps_us: &topic.timestamps_us })
                .collect(),
        })
        .collect();
    json_response(&executors, 200)
}

#[derive(serde::Serialize)]
struct VehicleStatusSample {
    t_us: u64,
    x_m: f64,
    y_m: f64,
    heading_rad: f64,
    speed_mps: f64,
}

/// `GET /api/vehicle_status_timeline` - the full, decoded vehicle status
/// timeline, fetched once so playback drives the main canvas purely from this
/// pre-fetched array (see `PlaybackClock` in `static/app.js`) rather than
/// polling per frame.
pub fn vehicle_status_timeline(session: &Session) -> ResponseBox {
    let samples: Vec<VehicleStatusSample> = session
        .vehicle_status_timeline
        .iter()
        .map(|(t_us, status)| VehicleStatusSample {
            t_us: *t_us,
            x_m: status.x_m,
            y_m: status.y_m,
            heading_rad: status.heading_rad,
            speed_mps: status.speed_mps,
        })
        .collect();
    json_response(&samples, 200)
}

/// `GET /api/debug/state?t_us=...` - a server-side hold-last-value convenience
/// (not the frontend's actual playback hot path - see [`vehicle_status_timeline`]).
pub fn state(session: &Session, query: Option<&str>) -> ResponseBox {
    let t_us: u64 = query
        .and_then(|q| q.split('&').find_map(|pair| pair.strip_prefix("t_us=")))
        .and_then(|value| value.parse().ok())
        .unwrap_or(0);
    match session.vehicle_status_at(t_us) {
        Some(status) => json_response(
            &VehicleStatusSample { t_us, x_m: status.x_m, y_m: status.y_m, heading_rad: status.heading_rad, speed_mps: status.speed_mps },
            200,
        ),
        None => json_response(&serde_json::json!(null), 200),
    }
}
