//! Serves the frontend assets specific to `web_gui` - `index.html`,
//! `style.css`, `app.js` - built into the binary via `include_str!`, so
//! there's no runtime asset path to get wrong. The shared map-canvas
//! script and base stylesheet come from [`crate::web`] instead.

use tiny_http::{Response, ResponseBox};

const INDEX_HTML: &str = include_str!("static/index.html");
const STYLE_CSS: &str = include_str!("static/style.css");
const APP_JS: &str = include_str!("static/app.js");

/// Serves one of the three embedded assets by file name (see
/// [`super::handlers::handle`] for the routes that map to each), with the
/// right `Content-Type`.
pub fn respond(file_name: &str) -> ResponseBox {
    let (body, content_type) = match file_name {
        "index.html" => (INDEX_HTML, "text/html; charset=utf-8"),
        "style.css" => (STYLE_CSS, "text/css; charset=utf-8"),
        "app.js" => (APP_JS, "text/javascript; charset=utf-8"),
        _ => unreachable!("respond is only called with the three routed asset names"),
    };
    Response::from_string(body)
        .with_header(crate::web::header("Content-Type", content_type))
        .boxed()
}
