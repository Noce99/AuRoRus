//! [`WebGui`]: an [`Executor`] that serves a local web UI for browsing and
//! generating maps, and for driving the vehicle - WASD control, live map
//! and vehicle status - on [`BIND_ADDR`]. Claims the writer slot for
//! `human_vesc_command` and `map_selection` (see [`live_api`]); reads
//! `map` and `vehicle_status`, which some other executor in the same
//! [`crate::Runner`] (e.g. [`crate::sensors::MapServer`],
//! [`crate::actuators::SimulatedVehicle`]) is expected to be writing.

mod assets;
mod handlers;
mod live_api;
mod maps_api;

use crate::topics::{
    HUMAN_VESC_COMMAND_TOPIC_NAME, MAP_SELECTION_TOPIC_NAME, MapSelection, VescCommand,
};
use crate::{Captain, Executor};
use std::any::Any;
use std::path::PathBuf;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

/// Address [`WebGui`] binds its HTTP server to.
const BIND_ADDR: &str = "0.0.0.0:1999";
/// Number of worker threads handling requests concurrently - so a slow
/// request (e.g. generating a large map) doesn't stall every other client.
const WORKER_THREADS: usize = 4;
/// How long each worker blocks in `recv_timeout` before re-checking whether
/// it should stop - bounds shutdown latency without busy-waiting.
const POLL_INTERVAL: Duration = Duration::from_millis(200);

/// Builds a `Content-Type: ...`-style header. Used by both [`assets`] and
/// [`maps_api`] - a header name/value built from a `&'static str` constant
/// is always valid ASCII, so parsing it can never fail.
fn header(name: &str, value: &str) -> tiny_http::Header {
    format!("{name}: {value}").parse().expect("header name/value are always valid ASCII")
}

/// Serves the map browser/generator web UI. Reads and writes map folders
/// under `maps_root` directly off disk.
pub struct WebGui {
    id: u8,
    name: String,
    maps_root: PathBuf,
}

impl WebGui {
    /// Creates a `WebGui` that will serve maps under `maps_root` once run.
    pub fn new(name: impl Into<String>, maps_root: impl Into<PathBuf>) -> Self {
        Self { id: 0, name: name.into(), maps_root: maps_root.into() }
    }
}

impl Executor for WebGui {
    fn init(&mut self, id: u8) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_writer::<VescCommand>(HUMAN_VESC_COMMAND_TOPIC_NAME, self.id, VescCommand::default);
        captain.claim_writer::<MapSelection>(MAP_SELECTION_TOPIC_NAME, self.id, MapSelection::default);
    }

    fn run(&mut self, captain: &Captain) {
        let server = match tiny_http::Server::http(BIND_ADDR) {
            Ok(server) => Arc::new(server),
            Err(err) => {
                eprintln!("{}: failed to bind {BIND_ADDR}: {err}", self.name);
                return;
            }
        };
        println!("{}: serving on http://{BIND_ADDR}", self.name);

        let id = self.id;
        let maps_root = &self.maps_root;
        thread::scope(|scope| {
            for _ in 0..WORKER_THREADS {
                let server = server.clone();
                scope.spawn(move || {
                    while captain.is_running(id) {
                        match server.recv_timeout(POLL_INTERVAL) {
                            Ok(Some(request)) => handlers::handle(request, maps_root, captain, id),
                            Ok(None) => continue,
                            Err(err) => eprintln!("web_gui: connection error: {err}"),
                        }
                    }
                });
            }
        });
    }

    fn name(&self) -> String {
        self.name.clone()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}
