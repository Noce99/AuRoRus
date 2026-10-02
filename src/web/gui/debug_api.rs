//! The Debug panel's API: starting and stopping a debug recording (see
//! [`crate::DebugRecorder`]) into a file under the debugs folder, and how the
//! current or last one is going.

use crate::debug_format::DebugFileReader;
use crate::web::{bad_request, error_response, json_response, read_json};
use crate::{Captain, DebugRecorder, DebugStatus};
use std::path::Path;
use tiny_http::{Request, ResponseBox};

/// Extension every recording's file gets.
const EXTENSION: &str = ".debug";
/// Recording rates the panel accepts, in Hz.
const MIN_FREQUENCY_HZ: f64 = 1.0;
const MAX_FREQUENCY_HZ: f64 = 1000.0;

#[derive(serde::Serialize)]
struct Debug {
    #[serde(flatten)]
    status: DebugStatus,
    /// Size of the current or last recording's file on disk, in bytes - lags a
    /// running recording by up to one flush.
    file_size_bytes: Option<u64>,
    /// The folder every recording started from here goes into.
    folder: String,
}

/// `GET /api/debug` - the current or last recording.
pub fn status(recorder: &DebugRecorder, debugs_root: &Path) -> ResponseBox {
    let status = recorder.status();
    let file_size_bytes = status
        .path
        .as_ref()
        .and_then(|path| std::fs::metadata(path).ok())
        .map(|metadata| metadata.len());
    json_response(
        &Debug {
            status,
            file_size_bytes,
            folder: debugs_root.display().to_string(),
        },
        200,
    )
}

#[derive(serde::Deserialize)]
struct StartBody {
    /// File name inside the debugs folder, `.debug` added if missing; empty or
    /// absent for a generated one.
    #[serde(default)]
    name: Option<String>,
    frequency_hz: f64,
}

/// `POST /api/debug/start` - body `{"name": "...", "frequency_hz": 100}` -
/// starts recording into a new file under `debugs_root`. Never overwrites
/// an existing file.
pub fn start(
    request: &mut Request,
    captain: &Captain,
    recorder: &DebugRecorder,
    debugs_root: &Path,
) -> ResponseBox {
    let body: StartBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    if !(MIN_FREQUENCY_HZ..=MAX_FREQUENCY_HZ).contains(&body.frequency_hz) {
        return bad_request(&format!(
            "the rate must be between {MIN_FREQUENCY_HZ} and {MAX_FREQUENCY_HZ} Hz"
        ));
    }
    let name = match file_name(body.name.as_deref().unwrap_or("")) {
        Ok(name) => name,
        Err(message) => return bad_request(&message),
    };
    let path = debugs_root.join(&name);
    if path.exists() {
        return error_response(409, &format!("{name} already exists - pick another name"));
    }
    match recorder.start(captain, path, body.frequency_hz) {
        Ok(()) => json_response(&(), 200),
        Err(message) => error_response(409, &message),
    }
}

/// `POST /api/debug/stop` - stops the running recording; it's saved once
/// `GET /api/debug` says `saved`.
pub fn stop(captain: &Captain, recorder: &DebugRecorder) -> ResponseBox {
    match recorder.stop(captain) {
        Ok(()) => json_response(&(), 200),
        Err(message) => error_response(409, &message),
    }
}

/// The file name a recording asked for as `requested` gets: a generated one
/// if blank, `.debug` appended if missing. Only a plain name, never a path.
fn file_name(requested: &str) -> Result<String, String> {
    let requested = requested.trim();
    if requested.is_empty() {
        return Ok(DebugFileReader::generated_filename());
    }
    if requested.starts_with('.') || requested.contains(['/', '\\']) {
        return Err(format!(
            "{requested:?} isn't a plain file name - no folders, no leading dot"
        ));
    }
    Ok(if requested.ends_with(EXTENSION) {
        requested.to_string()
    } else {
        format!("{requested}{EXTENSION}")
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blank_names_are_generated() {
        let name = file_name("  ").unwrap();
        assert!(name.ends_with(EXTENSION));
    }

    #[test]
    fn the_extension_is_added_once() {
        assert_eq!(file_name("lap1").unwrap(), "lap1.debug");
        assert_eq!(file_name("lap1.debug").unwrap(), "lap1.debug");
    }

    #[test]
    fn paths_are_refused() {
        for name in ["../escape", "sub/file", "sub\\file", ".hidden", ".."] {
            assert!(file_name(name).is_err(), "{name:?} should be refused");
        }
    }
}
