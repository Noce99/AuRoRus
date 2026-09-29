//! Serves the embedded viewer frontend - `index.html`, `style.css`,
//! `app.js`, `replay.js`, `charts.js`, `track_math.js` - built into the binary via
//! `include_str!`, same approach as the other web UIs' `assets` modules.

use aurorus::web::header;
use tiny_http::{Response, ResponseBox};

const INDEX_HTML: &str = include_str!("static/index.html");
const STYLE_CSS: &str = include_str!("static/style.css");
const APP_JS: &str = include_str!("static/app.js");
const REPLAY_JS: &str = include_str!("static/replay.js");
const CHARTS_JS: &str = include_str!("static/charts.js");
const TRACK_MATH_JS: &str = include_str!("static/track_math.js");

/// Serves one of the embedded assets by request path, or `None` if the path
/// names none of them.
pub fn respond(path: &str) -> Option<ResponseBox> {
    let (body, content_type) = match path {
        "/" | "/index.html" => (INDEX_HTML, "text/html; charset=utf-8"),
        "/style.css" => (STYLE_CSS, "text/css; charset=utf-8"),
        "/app.js" => (APP_JS, "text/javascript; charset=utf-8"),
        "/replay.js" => (REPLAY_JS, "text/javascript; charset=utf-8"),
        "/charts.js" => (CHARTS_JS, "text/javascript; charset=utf-8"),
        "/track_math.js" => (TRACK_MATH_JS, "text/javascript; charset=utf-8"),
        _ => return None,
    };
    Some(
        Response::from_string(body)
            .with_header(header("Content-Type", content_type))
            .boxed(),
    )
}

#[cfg(test)]
mod tests {
    use std::path::Path;
    use std::process::Command;

    /// `static/track_math.js` - the replay's and charts' math - is tested
    /// in JavaScript, by `static/track_math.test.js` under `node`. Skipped,
    /// with a note, where `node` isn't installed.
    #[test]
    fn track_math_js_passes_its_node_tests() {
        let test = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("src/bin/benchmark_viewer/static/track_math.test.js");
        let output = match Command::new("node").arg(&test).output() {
            Ok(output) => output,
            Err(err) if err.kind() == std::io::ErrorKind::NotFound => {
                eprintln!("skipping {}: node isn't installed", test.display());
                return;
            }
            Err(err) => panic!("couldn't run node: {err}"),
        };
        assert!(
            output.status.success(),
            "{} failed:\n{}{}",
            test.display(),
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr),
        );
    }
}
