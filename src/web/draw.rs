//! The drawing protocol both web UIs serve their map canvas through - `web_gui`
//! from live [`crate::topics::Drawing`] topics, `replay_web_gui` from a
//! recording of them - so one frontend (`/draw_layers.js`) can render either:
//!
//! - `POST /api/draw` with a [`DrawRequest`] body answers a [`DrawResponse`]:
//!   every drawing topic, each with its [`Drawing`] included only when it
//!   differs from the copy the client says it holds. Raster pixels are
//!   never included (see [`without_raster_pixels`]).
//! - `GET /api/draw/raster?topic=...&shape=...&epoch=...&write_count=...`
//!   (see [`RasterQuery`]) answers one raster shape's raw pixels, one byte
//!   each, row-major (see [`raster_response`]).
//!
//! A drawing's version is the pair `(epoch, write_count)`: `epoch` names the
//! source the counts belong to - a live `Captain` generation, or a loaded
//! recording - since `write_count` alone restarts from `0` with each new one.

use super::{bad_request, error_response, header, not_found, query_param};
use crate::topics::{Drawing, Shape};
use std::collections::HashMap;
use std::sync::Arc;
use tiny_http::{Response, ResponseBox};

/// `POST /api/draw`'s body: which version of each layer the client already
/// holds, so unchanged ones aren't sent again.
#[derive(Debug, serde::Deserialize)]
pub struct DrawRequest {
    /// The epoch the client's `known` counts were read under, or `null` on
    /// its first request. Any other value than the current epoch means the
    /// source changed since (e.g. a live restart), so `known` is ignored and
    /// every layer is sent in full.
    pub epoch: Option<u64>,
    /// Topic name -> the `write_count` of the drawing the client holds.
    #[serde(default)]
    pub known: HashMap<String, u64>,
    /// Playback time to draw, in microseconds since the recording started -
    /// only meaningful to a recording; a live source always draws "now".
    #[serde(default)]
    pub t_us: Option<u64>,
}

impl DrawRequest {
    /// Whether the client already holds `topic` at `write_count`, under
    /// `epoch` - i.e. whether its drawing can be left out of the response.
    pub fn holds(&self, epoch: u64, topic: &str, write_count: u64) -> bool {
        self.epoch == Some(epoch) && self.known.get(topic) == Some(&write_count)
    }
}

#[derive(Debug, serde::Serialize)]
pub struct DrawResponse {
    pub epoch: u64,
    /// Every drawing topic, in topic-name order.
    pub layers: Vec<DrawLayer>,
}

/// One drawing topic's current state. `write_count`/`age_ms` are always
/// sent, so the client can keep fading a layer whose drawing it already
/// holds; `drawing` is `null` when the client's copy is current.
#[derive(Debug, serde::Serialize)]
pub struct DrawLayer {
    pub topic: String,
    pub writer: Option<String>,
    /// `0` while nothing has been drawn yet.
    pub write_count: u64,
    /// How long before the drawn moment (now, or the requested playback
    /// time) the drawing was written - `null` while nothing has been drawn
    /// yet.
    pub age_ms: Option<f64>,
    pub drawing: Option<Drawing>,
}

/// `drawing` with every [`Shape::Raster`]'s pixels emptied - a map's raster
/// is megabytes as a JSON array, so the client fetches it from the raster
/// endpoint instead (see [`raster_response`]).
pub fn without_raster_pixels(mut drawing: Drawing) -> Drawing {
    for shape in &mut drawing.shapes {
        if let Shape::Raster { pixels, .. } = shape {
            *pixels = Arc::from([]);
        }
    }
    drawing
}

/// The raster endpoint's query parameters: shape number `shape` of the
/// version `(epoch, write_count)` of `topic`'s drawing - the version the
/// client got from `POST /api/draw`.
#[derive(Debug, PartialEq)]
pub struct RasterQuery {
    pub topic: String,
    pub shape: usize,
    pub epoch: u64,
    pub write_count: u64,
}

impl RasterQuery {
    /// Parses the query string of `url`, or returns the `400` response to
    /// send back instead.
    pub fn parse(url: &str) -> Result<Self, ResponseBox> {
        match (
            query_param(url, "topic"),
            query_param(url, "shape").and_then(|s| s.parse().ok()),
            query_param(url, "epoch").and_then(|s| s.parse().ok()),
            query_param(url, "write_count").and_then(|s| s.parse().ok()),
        ) {
            (Some(topic), Some(shape), Some(epoch), Some(write_count)) => Ok(Self {
                topic,
                shape,
                epoch,
                write_count,
            }),
            _ => Err(bad_request(
                "expected topic, shape, epoch and write_count query parameters",
            )),
        }
    }
}

/// `409 Conflict`, for a raster request naming a version of a drawing that
/// is no longer the one at hand - rather than send pixels that may not match
/// the dimensions the client holds. It picks up the newer drawing on its
/// next `POST /api/draw`.
pub fn stale_raster() -> ResponseBox {
    error_response(409, "drawing changed since - fetch it again")
}

/// The raw pixels of shape number `index` of `drawing`, or `404` if that
/// isn't a [`Shape::Raster`].
pub fn raster_response(drawing: &Drawing, index: usize) -> ResponseBox {
    let Some(Shape::Raster { pixels, .. }) = drawing.shapes.get(index) else {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_client_holds_a_layer_only_at_the_same_epoch_and_write_count() {
        let request: DrawRequest =
            serde_json::from_str(r#"{"epoch": 7, "known": {"draw/A": 3}}"#).unwrap();

        assert!(request.holds(7, "draw/A", 3));
        assert!(!request.holds(7, "draw/A", 4));
        assert!(!request.holds(8, "draw/A", 3));
        assert!(!request.holds(7, "draw/B", 3));
        assert_eq!(request.t_us, None);
    }

    #[test]
    fn raster_pixels_are_stripped_but_everything_else_is_kept() {
        let drawing = Drawing::default().element(
            "Map",
            [Shape::Raster {
                origin_x_m: 1.0,
                origin_y_m: 2.0,
                resolution_m_per_px: 0.5,
                width_px: 2,
                height_px: 1,
                pixels: vec![0u8, 255].into(),
            }],
            true,
        );

        let stripped = without_raster_pixels(drawing);

        let Shape::Raster {
            pixels, width_px, ..
        } = &stripped.shapes[0]
        else {
            unreachable!()
        };
        assert!(pixels.is_empty());
        assert_eq!(*width_px, 2);
    }

    #[test]
    fn raster_query_parses_every_parameter() {
        let query = RasterQuery::parse(
            "/api/draw/raster?topic=draw%2FMapServer&shape=0&epoch=5&write_count=2",
        )
        .ok()
        .expect("every parameter is present and valid");
        assert_eq!(
            query,
            RasterQuery {
                topic: "draw/MapServer".into(),
                shape: 0,
                epoch: 5,
                write_count: 2
            }
        );
        assert!(RasterQuery::parse("/api/draw/raster?topic=draw%2FMapServer").is_err());
    }
}
