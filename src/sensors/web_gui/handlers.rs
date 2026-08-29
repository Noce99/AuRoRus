//! Routes an incoming request to the right handler and writes back its
//! response.

use super::assets;
use super::maps_api;
use std::path::Path;
use tiny_http::{Method, Request, ResponseBox};

/// Handles one request end to end: routes it, then sends the response.
pub fn handle(mut request: Request, maps_root: &Path) {
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
