//! Routes an incoming request to the right handler.

use crate::lab::Lab;
use crate::{api, assets};
use aurorus::web::{not_found, respond_and_close};
use tiny_http::{Method, Request};

/// Handles one request end to end. `epoch` names this process's drawings -
/// see `aurorus::web::draw`.
pub fn handle(mut request: Request, lab: &Lab, epoch: u64) {
    let method = request.method().clone();
    let url = request.url().to_string();
    let path = url.split('?').next().unwrap_or("/").to_string();

    // The map canvas, its drawing layers, the parameter sliders and the
    // base stylesheet are shared with the other web UIs.
    if method == Method::Get
        && let Some(response) = aurorus::web::shared_asset(&path)
    {
        if let Err(err) = respond_and_close(request, response) {
            eprintln!("mapping_lab: failed to send response: {err}");
        }
        return;
    }

    let response = match (&method, path.as_str()) {
        (Method::Get, "/" | "/index.html") => assets::respond("index.html"),
        (Method::Get, "/style.css") => assets::respond("style.css"),
        (Method::Get, "/app.js") => assets::respond("app.js"),
        (Method::Get, "/api/sessions") => api::sessions(lab),
        (Method::Post, "/api/session") => api::load_session(&mut request, lab),
        (Method::Get, "/api/lab") => api::lab(lab),
        (Method::Post, "/api/draft_parameter") => api::set_draft_parameter(&mut request, lab),
        (Method::Post, "/api/draft_parameters_save") => api::save_draft(lab),
        (Method::Post, "/api/draft_parameters_load") => api::load_draft(lab),
        (Method::Post, "/api/draft_reset") => api::reset_draft(lab),
        (Method::Post, "/api/runs") => api::queue_run(lab),
        (Method::Post, "/api/runs/cancel") => api::cancel_run(&mut request, lab),
        (Method::Post, "/api/runs/delete") => api::delete_run(&mut request, lab),
        (Method::Post, "/api/runs/use") => api::use_run(&mut request, lab),
        (Method::Post, "/api/runs/save_map") => api::save_map(&mut request, lab),
        (Method::Post, "/api/draw") => api::draw(&mut request, lab, epoch),
        (Method::Get, "/api/draw/raster") => api::draw_raster(&url, lab, epoch),
        _ => not_found(),
    };

    if let Err(err) = respond_and_close(request, response) {
        eprintln!("mapping_lab: failed to send response: {err}");
    }
}
