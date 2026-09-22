//! The bits every web UI in this project needs: the handful of `tiny_http`
//! response helpers each one was otherwise re-implementing, and the
//! frontend assets they share.
//!
//! Both `web_gui` (driving live) and `debug_web_interface` (replaying a
//! recording) show the same map canvas, so the JavaScript that draws it
//! ([`MAP_VIEW_JS`]) and the CSS that frames it ([`BASE_CSS`]) live here
//! once and are served by both, ahead of each binary's own `app.js` and
//! `style.css`.

use tiny_http::{Response, ResponseBox};

/// The shared map-canvas frontend, served at `/map_view.js`.
pub const MAP_VIEW_JS: &str = include_str!("web/map_view.js");
/// The shared page/canvas styles, served at `/base.css`.
pub const BASE_CSS: &str = include_str!("web/base.css");

/// Builds a `Content-Type: ...`-style header. A header name/value built
/// from a `&'static str` constant is always valid ASCII, so parsing it can
/// never fail.
pub fn header(name: &str, value: &str) -> tiny_http::Header {
    format!("{name}: {value}")
        .parse()
        .expect("header name/value are always valid ASCII")
}

/// Serves one of the shared assets by request path, or `None` if the path
/// names none of them - let the caller's own router try its own assets
/// next.
pub fn shared_asset(path: &str) -> Option<ResponseBox> {
    let (body, content_type) = match path {
        "/map_view.js" => (MAP_VIEW_JS, "text/javascript; charset=utf-8"),
        "/base.css" => (BASE_CSS, "text/css; charset=utf-8"),
        _ => return None,
    };
    Some(
        Response::from_string(body)
            .with_header(header("Content-Type", content_type))
            .boxed(),
    )
}

/// Serializes `value` as a JSON response with the given status code.
pub fn json_response<T: serde::Serialize>(value: &T, status: u16) -> ResponseBox {
    let body = serde_json::to_string(value).expect("serializing a well-formed API response never fails");
    Response::from_string(body)
        .with_status_code(status)
        .with_header(header("Content-Type", "application/json"))
        .boxed()
}

#[derive(serde::Serialize)]
struct ErrorBody {
    error: String,
}

/// A JSON `{"error": "..."}` body with the given status code - the shape
/// both frontends' `fetchJSON` knows how to surface.
pub fn error_response(status: u16, message: &str) -> ResponseBox {
    json_response(&ErrorBody { error: message.to_string() }, status)
}

pub fn bad_request(message: &str) -> ResponseBox {
    error_response(400, message)
}

pub fn not_found() -> ResponseBox {
    error_response(404, "not found")
}
