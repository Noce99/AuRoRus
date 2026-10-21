//! Routes an incoming request to the right handler.

use crate::lab::{Group, Lab};
use crate::{api, assets};
use aurorus::web::{not_found, respond_and_close};
use tiny_http::{Method, Request};

/// Handles one request end to end. `epoch` names this process's drawings -
/// see `aurorus::web::draw`.
pub fn handle(mut request: Request, lab: &Lab, epoch: u64) {
    let method = request.method().clone();
    let url = request.url().to_string();
    let path = url.split('?').next().unwrap_or("/").to_string();

    // The map canvas, its drawing layers, the parameter sliders, the chart
    // helpers and the base stylesheet are shared with the other web UIs.
    if method == Method::Get
        && let Some(response) = aurorus::web::shared_asset(&path)
    {
        if let Err(err) = respond_and_close(request, response) {
            eprintln!("localization_lab: failed to send response: {err}");
        }
        return;
    }

    let scan_matching = Group::ScanMatching;
    let particle_filter = Group::ParticleFilter;
    let response = match (&method, path.as_str()) {
        (Method::Get, "/" | "/index.html") => assets::respond("index.html"),
        (Method::Get, "/style.css") => assets::respond("style.css"),
        (Method::Get, "/app.js") => assets::respond("app.js"),
        (Method::Get, "/charts.js") => assets::respond("charts.js"),
        (Method::Get, "/api/colors") => api::colors(),
        (Method::Get, "/api/sessions") => api::sessions(lab),
        (Method::Post, "/api/session") => api::load_session(&mut request, lab),
        (Method::Get, "/api/lab") => api::lab(lab),
        (Method::Post, "/api/reference") => api::set_reference(&mut request, lab),
        (Method::Post, "/api/race_line") => api::set_race_line(&mut request, lab),
        (Method::Post, "/api/draft") => api::set_draft(&mut request, lab),
        (Method::Post, "/api/draft_reset") => api::reset_draft(lab),
        (Method::Post, "/api/draft/scan_matching_parameter") => {
            api::set_draft_parameter(&mut request, lab, scan_matching)
        }
        (Method::Post, "/api/draft/scan_matching_parameters_save") => {
            api::save_draft(lab, scan_matching)
        }
        (Method::Post, "/api/draft/scan_matching_parameters_load") => {
            api::load_draft(lab, scan_matching)
        }
        (Method::Post, "/api/draft/particle_filter_parameter") => {
            api::set_draft_parameter(&mut request, lab, particle_filter)
        }
        (Method::Post, "/api/draft/particle_filter_parameters_save") => {
            api::save_draft(lab, particle_filter)
        }
        (Method::Post, "/api/draft/particle_filter_parameters_load") => {
            api::load_draft(lab, particle_filter)
        }
        (Method::Post, "/api/runs") => api::queue_run(lab),
        (Method::Post, "/api/runs/cancel") => {
            api::run_action(&mut request, |id| lab.cancel_run(id))
        }
        (Method::Post, "/api/runs/delete") => {
            api::run_action(&mut request, |id| lab.delete_run(id))
        }
        (Method::Post, "/api/runs/use") => api::run_action(&mut request, |id| lab.use_run(id)),
        (Method::Get, "/api/series") => api::series(&url, lab),
        (Method::Post, "/api/draw") => api::draw(&mut request, lab, epoch),
        (Method::Get, "/api/draw/raster") => api::draw_raster(&url, lab, epoch),
        _ => not_found(),
    };

    if let Err(err) = respond_and_close(request, response) {
        eprintln!("localization_lab: failed to send response: {err}");
    }
}
