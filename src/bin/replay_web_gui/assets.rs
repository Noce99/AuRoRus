//! Serves the embedded playback frontend - `index.html`, `style.css`,
//! `app.js`, `timeline.js` - built into the binary via `include_str!`, same
//! approach as `web_gui`'s `assets` module.

use aurorus::web::header;
use tiny_http::{Response, ResponseBox};

const INDEX_HTML: &str = include_str!("static/index.html");
const STYLE_CSS: &str = include_str!("static/style.css");
const APP_JS: &str = include_str!("static/app.js");
const TIMELINE_JS: &str = include_str!("static/timeline.js");

pub fn respond(file_name: &str) -> ResponseBox {
    let (body, content_type) = match file_name {
        "index.html" => (INDEX_HTML, "text/html; charset=utf-8"),
        "style.css" => (STYLE_CSS, "text/css; charset=utf-8"),
        "app.js" => (APP_JS, "text/javascript; charset=utf-8"),
        "timeline.js" => (TIMELINE_JS, "text/javascript; charset=utf-8"),
        _ => unreachable!("respond is only called with the four routed asset names"),
    };
    Response::from_string(body)
        .with_header(header("Content-Type", content_type))
        .boxed()
}
