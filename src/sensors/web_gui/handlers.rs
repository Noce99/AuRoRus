//! Routes an incoming request to the right handler and writes back its
//! response.

use super::assets;
use super::live_api;
use super::maps_api;
use crate::Captain;
use std::path::Path;
use tiny_http::{Method, Request, ResponseBox};

/// Handles one request end to end: routes it, then sends the response.
/// `writer_id` is this `WebGui`'s own executor id, used to authorize its
/// writes to `human_vesc_command`/`map_selection`.
pub fn handle(mut request: Request, maps_root: &Path, captain: &Captain, writer_id: u8) {
    let method = request.method().clone();
    let path = request.url().split('?').next().unwrap_or("/").to_string();

    let response = match (&method, path.as_str()) {
        (Method::Get, "/" | "/index.html") => assets::respond("index.html"),
        (Method::Get, "/style.css") => assets::respond("style.css"),
        (Method::Get, "/app.js") => assets::respond("app.js"),
        (Method::Get, "/api/maps") => maps_api::list(maps_root),
        (Method::Get, "/api/generate/defaults") => maps_api::generate_defaults(),
        (Method::Post, "/api/maps/generate") => maps_api::generate(&mut request, maps_root),
        (Method::Get, path) if path.starts_with("/api/maps/") => route_map_get(path, maps_root),
        (Method::Get, "/api/map") => live_api::map(captain),
        (Method::Get, "/api/map/raster") => live_api::raster(captain),
        (Method::Post, "/api/map_selection") => live_api::select_map(&mut request, captain, writer_id, maps_root),
        (Method::Get, "/api/vehicle_status") => live_api::vehicle_status(captain),
        (Method::Post, "/api/human_vesc_command") => live_api::human_vesc_command(&mut request, captain, writer_id),
        _ => maps_api::not_found(),
    };

    if let Err(err) = request.respond(response) {
        eprintln!("web_gui: failed to send response: {err}");
    }
}

/// Routes `GET /api/maps/{name}/{info,raster}`.
fn route_map_get(path: &str, maps_root: &Path) -> ResponseBox {
    let rest = &path["/api/maps/".len()..];
    let Some((name, suffix)) = rest.split_once('/') else {
        return maps_api::not_found();
    };
    match suffix {
        "info" => maps_api::info(name, maps_root),
        "raster" => maps_api::raster(name, maps_root),
        _ => maps_api::not_found(),
    }
}
