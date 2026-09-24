//! The read-only API a recorded [`Session`] is served through: the session's
//! summary, its timeline, and the playback side of the drawing protocol (see
//! `aurorus::web::draw`) - the same protocol `web_gui` serves live, so the
//! canvas renders a recording with the very same frontend code.

use crate::session::Session;
use aurorus::web::draw::{
    DrawLayer, DrawRequest, DrawResponse, RasterQuery, raster_response, stale_raster,
    without_raster_pixels,
};
use aurorus::web::{json_response, not_found, read_json};
use tiny_http::{Request, ResponseBox};

#[derive(serde::Serialize)]
struct SessionSummary<'a> {
    /// The recorded file's name, for the page to show.
    file_name: &'a str,
    frequency_hz: f64,
    duration_us: u64,
}

/// `GET /api/session` - what the frontend needs up front besides the
/// timeline ([`timeline`]) and what's drawn ([`draw`]).
pub fn session(session: &Session, file_name: &str) -> ResponseBox {
    json_response(
        &SessionSummary {
            file_name,
            frequency_hz: session.frequency_hz,
            duration_us: session.duration_us,
        },
        200,
    )
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

/// `GET /api/timeline` - every recorded change's timestamp, grouped by
/// writer executor - fetched once at load so the timeline never round-trips
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
                .map(|topic| TimelineTopic {
                    name: &topic.name,
                    color_index: topic.color_index,
                    timestamps_us: &topic.timestamps_us,
                })
                .collect(),
        })
        .collect();
    json_response(&executors, 200)
}

/// `POST /api/draw` - every recorded drawing topic as of playback time
/// `t_us` (holding each one's last value, like a live read), with ages
/// measured against that time rather than the wall clock. See
/// `aurorus::web::draw`.
pub fn draw(request: &mut Request, session: &Session) -> ResponseBox {
    let body: DrawRequest = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let t_us = body.t_us.unwrap_or(0);

    let layers = session
        .drawings
        .iter()
        .map(|track| {
            let version = session.version_at(track, t_us);
            let drawing = if body.holds(session.epoch, &track.topic, version.write_count) {
                None
            } else {
                session
                    .drawing(track, version.write_count)
                    .map(without_raster_pixels)
            };
            DrawLayer {
                topic: track.topic.clone(),
                writer: Some(track.writer.clone()),
                write_count: version.write_count,
                age_ms: version
                    .written_at_us
                    .map(|written_at_us| t_us.saturating_sub(written_at_us) as f64 / 1000.0),
                drawing,
            }
        })
        .collect();

    json_response(
        &DrawResponse {
            epoch: session.epoch,
            layers,
        },
        200,
    )
}

/// `GET /api/draw/raster?...` - see `aurorus::web::draw`. Any recorded
/// version can be fetched, not only the one current at some playback time:
/// a recording never changes, so a version never goes stale - only a
/// different session's epoch does.
pub fn draw_raster(url: &str, session: &Session) -> ResponseBox {
    let query = match RasterQuery::parse(url) {
        Ok(query) => query,
        Err(response) => return response,
    };
    if query.epoch != session.epoch {
        return stale_raster();
    }
    let Some(drawing) = session
        .track(&query.topic)
        .and_then(|track| session.drawing(track, query.write_count))
    else {
        return not_found();
    };
    raster_response(&drawing, query.shape)
}
