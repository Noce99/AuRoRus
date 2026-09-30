//! Routes an incoming request to the right handler and writes back its
//! response.

use super::BenchmarkSetup;
use super::WebGuiConfig;
use super::assets;
use super::benchmark_api::{self, Benchmark};
use super::debug_api;
use super::detector_api;
use super::draw_api;
use super::live_api;
use super::map_edit_api;
use super::maps_api;
use super::opponents_api;
use super::race_lines_api;
use super::topics_api;
use crate::web::{not_found, respond_and_close};
use crate::{Captain, DebugRecorder};
use std::path::Path;
use tiny_http::{Method, Request, ResponseBox};

/// What the Debug panel's routes need - see [`debug_api`].
pub struct Debug<'a> {
    /// Folder recordings go into.
    pub root: &'a Path,
    pub recorder: &'a DebugRecorder,
}

/// What the Benchmark panel's routes need - see [`benchmark_api`].
pub struct Benchmarks<'a> {
    pub setup: &'a BenchmarkSetup,
    pub benchmark: &'a Benchmark,
}

/// Handles one request end to end: routes it, then sends the response.
/// `writer_id` is this `WebGui`'s own executor id, used to authorize its
/// writes to `human_vesc_command`/`map_selection`.
pub fn handle(
    mut request: Request,
    maps_root: &Path,
    debug: &Debug,
    benchmarks: &Benchmarks,
    captain: &Captain,
    writer_id: u16,
    config: &WebGuiConfig,
) {
    let method = request.method().clone();
    let url = request.url().to_string();
    let path = url.split('?').next().unwrap_or("/").to_string();

    // Assets shared with the other web UI (the map canvas script, the base
    // stylesheet) are served by `crate::web`; the rest are this UI's own.
    if method == Method::Get
        && let Some(response) = crate::web::shared_asset(&path)
    {
        if let Err(err) = respond_and_close(request, response) {
            eprintln!("web_gui: failed to send response: {err}");
        }
        return;
    }

    let response = match (&method, path.as_str()) {
        // Nothing that changes the setup while a benchmark runs.
        (method, path) if benchmark_api::blocks(benchmarks.benchmark, method, path) => {
            benchmark_api::refused()
        }
        (Method::Get, "/" | "/index.html") => assets::respond("index.html"),
        (Method::Get, "/style.css") => assets::respond("style.css"),
        (Method::Get, "/app.js") => assets::respond("app.js"),
        (Method::Get, "/benchmark.js") => assets::respond("benchmark.js"),
        (Method::Get, "/api/config") => live_api::config(config, captain),
        (Method::Get, "/api/vesc") => live_api::vesc(captain),
        (Method::Get, "/api/vesc_parameters") => live_api::vesc_parameters(captain),
        (Method::Post, "/api/vesc_parameter") => {
            live_api::set_vesc_parameter(&mut request, captain, writer_id)
        }
        (Method::Post, "/api/vesc_parameters_save") => {
            live_api::save_vesc_parameters(&mut request, captain)
        }
        (Method::Post, "/api/vesc_parameters_load") => {
            live_api::load_vesc_parameters(&mut request, captain, writer_id)
        }
        (Method::Get, "/api/maps") => maps_api::list(maps_root),
        (Method::Get, "/api/generate/defaults") => maps_api::generate_defaults(),
        (Method::Post, "/api/maps/generate") => maps_api::generate(&mut request, maps_root),
        (Method::Post, "/api/maps/import") => maps_api::import(&mut request, maps_root),
        (Method::Post, "/api/maps/import/decode_tiff") => maps_api::decode_tiff(&mut request),
        (Method::Post, "/api/maps/start_finish_line") => {
            maps_api::set_start_finish_line(&mut request, captain, writer_id, maps_root)
        }
        (Method::Get, path) if path.starts_with("/api/maps/") => route_map_get(path, maps_root),
        (Method::Get, "/api/map_edit") => map_edit_api::info(&url, captain, maps_root),
        (Method::Get, "/api/map_edit/raster") => map_edit_api::raster(&url, maps_root),
        (Method::Post, "/api/map_edit/raster") => {
            map_edit_api::save(&mut request, captain, writer_id, maps_root)
        }
        (Method::Get, "/api/map") => live_api::map(captain),
        (Method::Post, "/api/map_selection") => {
            live_api::select_map(&mut request, captain, writer_id, maps_root)
        }
        (Method::Post, "/api/draw") => draw_api::layers(&mut request, captain),
        (Method::Get, "/api/draw/raster") => draw_api::raster(&url, captain),
        (Method::Get, "/api/topics") => topics_api::list(captain),
        (Method::Get, "/api/topic") => topics_api::value(&url, captain),
        (Method::Post, "/api/human_vesc_command") => {
            live_api::human_vesc_command(&mut request, captain, writer_id)
        }
        (Method::Get, "/api/vehicle_models") => live_api::vehicle_models(),
        (Method::Get, "/api/vehicle_model") => live_api::vehicle_model(captain),
        (Method::Post, "/api/vehicle_model_selection") => {
            live_api::select_vehicle_model(&mut request, captain, writer_id)
        }
        (Method::Post, "/api/vehicle_model_parameter") => {
            live_api::set_vehicle_model_parameter(&mut request, captain, writer_id)
        }
        (Method::Post, "/api/vehicle_model_parameters_save") => {
            live_api::save_vehicle_model_parameters(&mut request, captain)
        }
        (Method::Post, "/api/vehicle_limit") => {
            live_api::set_vehicle_limit(&mut request, captain, writer_id)
        }
        (Method::Post, "/api/vehicle_limits_save") => live_api::save_vehicle_limits(captain),
        (Method::Post, "/api/vehicle_model_parameters_load") => {
            live_api::load_vehicle_model_parameters(&mut request, captain, writer_id)
        }
        (Method::Post, "/api/vehicle_limits_load") => {
            live_api::load_vehicle_limits(captain, writer_id)
        }
        (Method::Get, "/api/autonomous_algorithms") => live_api::autonomous_algorithms(captain),
        (Method::Post, "/api/autonomous_algorithm_selection") => {
            live_api::select_autonomous_algorithm(&mut request, captain, writer_id)
        }
        (Method::Post, "/api/autonomous_parameter") => {
            live_api::set_autonomous_parameter(&mut request, captain, writer_id)
        }
        (Method::Post, "/api/autonomous_parameters_save") => {
            live_api::save_autonomous_parameters(&mut request, captain)
        }
        (Method::Post, "/api/autonomous_parameters_load") => {
            live_api::load_autonomous_parameters(&mut request, captain, writer_id)
        }
        (Method::Post, "/api/restart") => live_api::restart(captain),
        (Method::Post, "/api/place_at_start") => {
            live_api::place_at_start(&mut request, captain, writer_id)
        }
        (Method::Get, "/api/slam") => live_api::slam(captain),
        (Method::Post, "/api/slam_command") => {
            live_api::slam_command(&mut request, captain, writer_id)
        }
        (Method::Post, "/api/slam_save") => live_api::slam_save(&mut request, captain, writer_id),
        (Method::Get, "/api/detector") => detector_api::detector(captain),
        (Method::Post, "/api/detector_parameter") => {
            detector_api::set_detector_parameter(&mut request, captain, writer_id)
        }
        (Method::Post, "/api/detector_parameters_save") => {
            detector_api::save_detector_parameters(captain)
        }
        (Method::Post, "/api/detector_parameters_load") => {
            detector_api::load_detector_parameters(captain, writer_id)
        }
        (Method::Get, "/api/planning") => live_api::planning(captain),
        (Method::Post, "/api/planning_parameter") => {
            live_api::set_planning_parameter(&mut request, captain, writer_id)
        }
        (Method::Post, "/api/planning_parameters_save") => {
            live_api::save_planning_parameters(captain)
        }
        (Method::Post, "/api/planning_parameters_load") => {
            live_api::load_planning_parameters(captain, writer_id)
        }
        (Method::Post, "/api/planning_start") => {
            live_api::planning_start(&mut request, captain, writer_id)
        }
        (Method::Get, "/api/lap_telemetry") => live_api::lap_telemetry(captain),
        (Method::Get, "/api/race_lines") => race_lines_api::list(captain),
        (Method::Get, "/api/opponents") => opponents_api::list(captain),
        (Method::Post, "/api/opponents") => opponents_api::add(&mut request, captain, writer_id),
        (Method::Post, "/api/opponents/delete") => {
            opponents_api::delete(&mut request, captain, writer_id)
        }
        (Method::Post, "/api/race/start") => {
            opponents_api::start_race(&mut request, captain, writer_id, config)
        }
        (Method::Get, "/api/debug") => debug_api::status(debug.recorder, debug.root),
        (Method::Post, "/api/debug/start") => {
            debug_api::start(&mut request, captain, debug.recorder, debug.root)
        }
        (Method::Post, "/api/debug/stop") => debug_api::stop(captain, debug.recorder),
        (Method::Get, "/api/benchmark") => {
            benchmark_api::status(benchmarks.benchmark, benchmarks.setup, config)
        }
        (Method::Get, "/api/benchmark/options") => {
            benchmark_api::options(captain, maps_root, config)
        }
        (Method::Post, "/api/benchmark/start") => {
            benchmark_api::start(&mut request, captain, maps_root, benchmarks.benchmark)
        }
        (Method::Post, "/api/benchmark/abort") => benchmark_api::abort(benchmarks.benchmark),
        (Method::Post, "/api/race_line_selection") => {
            race_lines_api::select(&mut request, captain, writer_id)
        }
        _ => not_found(),
    };

    // Every response closes its connection, so no client is ever left holding
    // one to a server that a restart has since torn down - see
    // `respond_and_close`.
    if let Err(err) = respond_and_close(request, response) {
        eprintln!("web_gui: failed to send response: {err}");
    }
}

/// Routes `GET /api/maps/{name}/{info,raster}`.
fn route_map_get(path: &str, maps_root: &Path) -> ResponseBox {
    let rest = &path["/api/maps/".len()..];
    let Some((name, suffix)) = rest.split_once('/') else {
        return not_found();
    };
    match suffix {
        "info" => maps_api::info(name, maps_root),
        "raster" => maps_api::raster(name, maps_root),
        _ => not_found(),
    }
}
