//! Routes an incoming request to the right handler - a smaller counterpart
//! of `web_gui`'s `handlers::handle`: there's nothing to write in playback
//! mode, and `POST /api/draw` only carries what the client already holds.

use crate::session::Session;
use crate::{assets, debug_api};
use aurorus::web::not_found;
use tiny_http::{Method, Request};

pub fn handle(mut request: Request, session: &Session, file_name: &str) {
    let method = request.method().clone();
    let url = request.url().to_string();
    let path = url.split('?').next().unwrap_or("/").to_string();

    // Assets shared with `web_gui` (the map canvas and drawing layer
    // scripts, the base stylesheet) are served by `aurorus::web`; the rest
    // are this UI's own.
    if method == Method::Get
        && let Some(response) = aurorus::web::shared_asset(&path)
    {
        if let Err(err) = request.respond(response) {
            eprintln!("replay_web_gui: failed to send response: {err}");
        }
        return;
    }

    let response = match (&method, path.as_str()) {
        (Method::Get, "/" | "/index.html") => assets::respond("index.html"),
        (Method::Get, "/style.css") => assets::respond("style.css"),
        (Method::Get, "/app.js") => assets::respond("app.js"),
        (Method::Get, "/timeline.js") => assets::respond("timeline.js"),
        (Method::Get, "/api/session") => debug_api::session(session, file_name),
        (Method::Get, "/api/timeline") => debug_api::timeline(session),
        (Method::Post, "/api/draw") => debug_api::draw(&mut request, session),
        (Method::Get, "/api/draw/raster") => debug_api::draw_raster(&url, session),
        (Method::Get, "/api/lap_telemetry") => debug_api::lap_telemetry(&url, session),
        _ => not_found(),
    };

    if let Err(err) = request.respond(response) {
        eprintln!("replay_web_gui: failed to send response: {err}");
    }
}
