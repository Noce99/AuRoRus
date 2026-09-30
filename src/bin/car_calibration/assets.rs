//! Serves the embedded calibration page - `index.html`, `style.css`,
//! `app.js` - built into the binary via `include_str!`, same approach as
//! the other web UIs' `assets` modules.

use aurorus::web::header;
use tiny_http::{Response, ResponseBox};

const INDEX_HTML: &str = include_str!("static/index.html");
const STYLE_CSS: &str = include_str!("static/style.css");
const APP_JS: &str = include_str!("static/app.js");

/// Serves one of the embedded assets by request path, or `None` if the path
/// names none of them.
pub fn respond(path: &str) -> Option<ResponseBox> {
    let (body, content_type) = match path {
        "/" | "/index.html" => (INDEX_HTML, "text/html; charset=utf-8"),
        "/style.css" => (STYLE_CSS, "text/css; charset=utf-8"),
        "/app.js" => (APP_JS, "text/javascript; charset=utf-8"),
        _ => return None,
    };
    Some(
        Response::from_string(body)
            .with_header(header("Content-Type", content_type))
            .boxed(),
    )
}
