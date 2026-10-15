//! `mapping_lab`: builds maps offline from mapping sessions recorded by
//! `web_gui`'s Record session toggle, with whatever SLAM parameters, and
//! compares them - the same map canvas as the other web UIs, a list of
//! runs, and the parameter sliders of `web_gui`'s Mapping panel. The chosen
//! map is saved under the maps folder, and its parameters into
//! `config/localization/slam.toml`. See `documentation/mapping_lab.md`.

mod api;
mod assets;
mod cli;
mod handlers;
mod lab;

use lab::Lab;
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

/// How many threads answer HTTP requests: loading a session takes a moment,
/// and the page keeps polling meanwhile.
const HTTP_THREADS: usize = 4;

fn main() {
    let config = cli::parse_config(std::env::args());
    let lab = Arc::new(Lab::new(
        config.sessions_root.clone(),
        config.maps_root.clone(),
        config.config_dir.clone(),
    ));
    if let Some(file) = &config.file
        && let Err(err) = lab.load("", Some(file))
    {
        eprintln!("mapping_lab: failed to load {file:?}: {err}");
        std::process::exit(1);
    }

    let workers = config.workers.unwrap_or_else(|| {
        std::thread::available_parallelism()
            .map_or(1, |n| n.get().saturating_sub(1))
            .max(1)
    });
    lab::run_workers(&lab, workers);

    let server = aurorus::web::bind_http(config.bind_addr.as_str()).unwrap_or_else(|err| {
        eprintln!("mapping_lab: failed to bind {}: {err}", config.bind_addr);
        std::process::exit(1);
    });
    let server = Arc::new(server);
    println!(
        "mapping_lab: sessions from {:?}, {workers} run worker(s), serving on http://{}",
        config.sessions_root, config.bind_addr
    );

    // Names this process's drawings, so a page left open across a restart
    // of the lab fetches every layer again.
    let epoch = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_micros() as u64;
    let threads: Vec<_> = (0..HTTP_THREADS)
        .map(|_| {
            let server = server.clone();
            let lab = lab.clone();
            std::thread::spawn(move || {
                loop {
                    match server.recv() {
                        Ok(request) => handlers::handle(request, &lab, epoch),
                        Err(err) => eprintln!("mapping_lab: connection error: {err}"),
                    }
                }
            })
        })
        .collect();
    for thread in threads {
        let _ = thread.join();
    }
}
