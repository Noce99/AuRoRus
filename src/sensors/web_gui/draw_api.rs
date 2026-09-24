//! The drawing API: every [`Drawing`] topic currently registered (every
//! topic named [`DRAW_TOPIC_PREFIX`]...), for the frontend to render on its
//! map canvas without knowing which executor drew what, or why.

use crate::Captain;
use crate::topics::{DRAW_TOPIC_PREFIX, Drawing, Shape};
use crate::web::{bad_request, error_response, header, json_response, not_found, query_param, read_json};
use std::collections::HashMap;
use std::sync::Arc;
use tiny_http::{Request, Response, ResponseBox};

/// `POST /api/draw`'s body: which version of each layer the client already
/// holds, so unchanged ones aren't sent again.
#[derive(serde::Deserialize)]
struct DrawRequest {
    /// The [`Captain::epoch`] the client's `known` counts were read under,
    /// or `null` on its first request. Any other value than the current
    /// epoch means the server restarted since, so `known` is ignored and
    /// every layer is sent in full.
    epoch: Option<u64>,
    /// Topic name -> the `write_count` of the drawing the client holds.
    #[serde(default)]
    known: HashMap<String, u64>,
}

#[derive(serde::Serialize)]
struct DrawResponse {
    epoch: u64,
    layers: Vec<Layer>,
}

/// One drawing topic's current state. `write_count`/`age_ms` are always
/// sent, so the client can keep fading a layer whose drawing it already
/// holds; `drawing` is `null` when the client's copy is current.
#[derive(serde::Serialize)]
struct Layer {
    topic: String,
    writer: Option<String>,
    write_count: u64,
    /// How long ago the drawing was written, measured server-side at
    /// response time - `null` while the topic still holds its seed.
    age_ms: Option<f64>,
    drawing: Option<Drawing>,
}

/// `POST /api/draw` - body `{"epoch": ..., "known": {"draw/...": write_count}}`
/// - every drawing topic, in topic-name order, each with its drawing
/// included only if it differs from the client's copy (see [`DrawRequest`]).
/// Raster pixels are never included - they're fetched separately, as raw
/// bytes, from [`raster`].
pub fn layers(request: &mut Request, captain: &Captain) -> ResponseBox {
    let body: DrawRequest = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let epoch = captain.epoch();
    let known = if body.epoch == Some(epoch) { body.known } else { HashMap::new() };

    let mut names: Vec<String> = captain
        .debug_topics_snapshot()
        .into_iter()
        .map(|(name, _)| name)
        .filter(|name| name.starts_with(DRAW_TOPIC_PREFIX))
        .collect();
    names.sort();

    let layers = names
        .into_iter()
        // A `draw/` topic of another type is someone's mistake - skip it
        // rather than take the whole process down over it.
        .filter_map(|name| captain.try_topic::<Drawing>(&name).map(|topic| (name, topic)))
        .map(|(name, topic)| {
            let meta = topic.meta();
            let (meta, drawing) = if known.get(&name) == Some(&meta.write_count) {
                (meta, None)
            } else {
                // Value and meta from the same read, so the count the client
                // records always matches the drawing it got.
                let stamped = topic.read();
                (stamped.meta, Some(without_raster_pixels(stamped.value)))
            };
            Layer {
                writer: topic.writer().map(|id| captain.name_of(id)),
                topic: name,
                write_count: meta.write_count,
                age_ms: meta.written_at.map(|written_at| written_at.elapsed().as_secs_f64() * 1000.0),
                drawing,
            }
        })
        .collect();

    json_response(&DrawResponse { epoch, layers }, 200)
}

/// `drawing` with every [`Shape::Raster`]'s pixels emptied - a map's raster
/// is megabytes as a JSON array, so the client fetches it from [`raster`]
/// instead.
fn without_raster_pixels(mut drawing: Drawing) -> Drawing {
    for shape in &mut drawing.shapes {
        if let Shape::Raster { pixels, .. } = shape {
            *pixels = Arc::from([]);
        }
    }
    drawing
}

/// `GET /api/draw/raster?topic=...&shape=...&epoch=...&write_count=...` - the
/// raw pixels (one byte each, row-major) of shape number `shape` of `topic`'s
/// drawing, which must be a [`Shape::Raster`]. `epoch`/`write_count` name the
/// drawing the client got from [`layers`]: if the topic has been rewritten
/// since, this answers `409 Conflict` rather than send pixels that may not
/// match the dimensions the client holds - it will pick up the newer drawing
/// on its next poll.
pub fn raster(url: &str, captain: &Captain) -> ResponseBox {
    let (Some(topic_name), Some(index), Some(epoch), Some(write_count)) = (
        query_param(url, "topic"),
        query_param(url, "shape").and_then(|s| s.parse::<usize>().ok()),
        query_param(url, "epoch").and_then(|s| s.parse::<u64>().ok()),
        query_param(url, "write_count").and_then(|s| s.parse::<u64>().ok()),
    ) else {
        return bad_request("expected topic, shape, epoch and write_count query parameters");
    };
    if !topic_name.starts_with(DRAW_TOPIC_PREFIX) {
        return not_found();
    }
    let Some(topic) = captain.try_topic::<Drawing>(&topic_name) else {
        return not_found();
    };

    let stamped = topic.read();
    if epoch != captain.epoch() || write_count != stamped.meta.write_count {
        return error_response(409, "drawing changed since - fetch it again");
    }
    let Some(Shape::Raster { pixels, .. }) = stamped.value.shapes.get(index) else {
        return not_found();
    };

    let pixels = pixels.clone();
    let len = pixels.len();
    Response::new(
        tiny_http::StatusCode(200),
        vec![header("Content-Type", "application/octet-stream")],
        std::io::Cursor::new(pixels),
        Some(len),
        None,
    )
    .boxed()
}
