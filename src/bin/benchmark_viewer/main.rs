//! `benchmark_viewer`: serves a read-only web UI over every benchmark run
//! `web_gui`'s Benchmark panel recorded (see `aurorus::benchmark`) - to
//! filter them, compare their lap times, parameters and telemetry, and
//! replay several of them together on the map from their recorded poses.
//! Nothing is simulated: everything comes from the self-contained run
//! folders under `--benchmarks-root`.

mod assets;
mod cli;
mod runs_api;

use aurorus::web::{not_found, respond_and_close};
use std::path::Path;
use tiny_http::{Method, Request};

fn main() {
    let config = cli::parse_config(std::env::args());

    let server = aurorus::web::bind_http(config.bind_addr.as_str()).unwrap_or_else(|err| {
        eprintln!(
            "benchmark_viewer: failed to bind {}: {err}",
            config.bind_addr
        );
        std::process::exit(1);
    });
    println!(
        "benchmark_viewer: serving {:?} on http://{}",
        config.benchmarks_root, config.bind_addr
    );

    for request in server.incoming_requests() {
        handle(request, &config.benchmarks_root);
    }
}

/// Routes one request and sends its response. Everything is a `GET`:
/// nothing is ever written.
fn handle(request: Request, root: &Path) {
    let url = request.url().to_string();
    let path = url.split('?').next().unwrap_or("/");

    let response = if *request.method() != Method::Get {
        not_found()
    } else if let Some(response) = aurorus::web::shared_asset(path) {
        response
    } else if let Some(response) = assets::respond(path) {
        response
    } else if path == "/api/runs" {
        runs_api::list(root)
    } else if let Some(rest) = path.strip_prefix("/api/runs/") {
        runs_api::route(root, rest)
    } else {
        not_found()
    };

    if let Err(err) = respond_and_close(request, response) {
        eprintln!("benchmark_viewer: failed to send response: {err}");
    }
}
