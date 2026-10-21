//! `localization_lab`: localizes the sessions recorded by `web_gui`'s
//! Localization panel again, offline, with every localization algorithm
//! and odometry - each run on the very same drive - and compares them: their
//! trajectories on the map over the ground truth, their error and their
//! cost along each lap, and a table of scores. See
//! `documentation/localization_lab.md`.

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
        config.config_dir.clone(),
        config.workers,
    ));
    if let Some(file) = &config.file
        && let Err(err) = lab.load("", Some(file))
    {
        eprintln!("localization_lab: failed to load {file:?}: {err}");
        std::process::exit(1);
    }

    lab::run_workers(&lab, config.workers);

    let server = aurorus::web::bind_http(config.bind_addr.as_str()).unwrap_or_else(|err| {
        eprintln!(
            "localization_lab: failed to bind {}: {err}",
            config.bind_addr
        );
        std::process::exit(1);
    });
    let server = Arc::new(server);
    println!(
        "localization_lab: sessions from {:?}, {} run worker(s), serving on http://{}",
        config.sessions_root, config.workers, config.bind_addr
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
                        Err(err) => eprintln!("localization_lab: connection error: {err}"),
                    }
                }
            })
        })
        .collect();
    for thread in threads {
        let _ = thread.join();
    }
}
