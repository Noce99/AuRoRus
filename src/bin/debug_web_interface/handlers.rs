//! Routes an incoming request to the right handler - a smaller, all-`GET`
//! counterpart of `web_gui`'s `handlers::handle`, since there's nothing to
//! write in playback mode.

use crate::session::Session;
use crate::{assets, debug_api};
use tiny_http::{Method, Request};

pub fn handle(request: Request, session: &Session) {
    let method = request.method().clone();
    let (path, query) = {
        let url = request.url();
        match url.split_once('?') {
            Some((path, query)) => (path.to_string(), Some(query.to_string())),
            None => (url.to_string(), None),
        }
    };

    let response = match (&method, path.as_str()) {
        (Method::Get, "/" | "/index.html") => assets::respond("index.html"),
        (Method::Get, "/style.css") => assets::respond("style.css"),
        (Method::Get, "/app.js") => assets::respond("app.js"),
        (Method::Get, "/timeline.js") => assets::respond("timeline.js"),
        (Method::Get, "/api/session") => debug_api::session(session),
        (Method::Get, "/api/map/raster") => debug_api::map_raster(session),
        (Method::Get, "/api/timeline") => debug_api::timeline(session),
        (Method::Get, "/api/vehicle_status_timeline") => debug_api::vehicle_status_timeline(session),
        (Method::Get, "/api/debug/state") => debug_api::state(session, query.as_deref()),
        _ => debug_api::not_found(),
    };

    if let Err(err) = request.respond(response) {
        eprintln!("debug_web_interface: failed to send response: {err}");
    }
}
