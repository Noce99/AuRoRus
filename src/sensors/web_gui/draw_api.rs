//! The live side of the drawing protocol (see [`crate::web::draw`]): every
//! [`Drawing`] topic currently registered (every topic named
//! [`DRAW_TOPIC_PREFIX`]...), for the frontend to render on its map canvas
//! without knowing which executor drew what, or why. The `(epoch,
//! write_count)` version of a drawing is the [`Captain::epoch`] and the
//! topic's own [`crate::WriteMeta::write_count`].

use crate::Captain;
use crate::topics::{DRAW_TOPIC_PREFIX, Drawing};
use crate::web::draw::{
    DrawLayer, DrawRequest, DrawResponse, RasterQuery, raster_response, stale_raster,
    without_raster_pixels,
};
use crate::web::{json_response, not_found, read_json};
use tiny_http::{Request, ResponseBox};

/// `POST /api/draw` - see [`crate::web::draw`].
pub fn layers(request: &mut Request, captain: &Captain) -> ResponseBox {
    let body: DrawRequest = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let epoch = captain.epoch();

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
        .filter_map(|name| {
            captain
                .try_topic::<Drawing>(&name)
                .map(|topic| (name, topic))
        })
        .map(|(name, topic)| {
            let meta = topic.meta();
            let (meta, drawing) = if body.holds(epoch, &name, meta.write_count) {
                (meta, None)
            } else {
                // Value and meta from the same read, so the count the client
                // records always matches the drawing it got.
                let stamped = topic.read();
                (stamped.meta, Some(without_raster_pixels(stamped.value)))
            };
            DrawLayer {
                writer: topic.writer().map(|id| captain.name_of(id)),
                topic: name,
                write_count: meta.write_count,
                age_ms: meta
                    .written_at
                    .map(|written_at| written_at.elapsed().as_secs_f64() * 1000.0),
                drawing,
            }
        })
        .collect();

    json_response(&DrawResponse { epoch, layers }, 200)
}

/// `GET /api/draw/raster?...` - see [`crate::web::draw`].
pub fn raster(url: &str, captain: &Captain) -> ResponseBox {
    let query = match RasterQuery::parse(url) {
        Ok(query) => query,
        Err(response) => return response,
    };
    if !query.topic.starts_with(DRAW_TOPIC_PREFIX) {
        return not_found();
    }
    let Some(topic) = captain.try_topic::<Drawing>(&query.topic) else {
        return not_found();
    };

    let stamped = topic.read();
    if query.epoch != captain.epoch() || query.write_count != stamped.meta.write_count {
        return stale_raster();
    }
    raster_response(&stamped.value, query.shape)
}
