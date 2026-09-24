//! [`WebGui`]: an [`Executor`] that serves a local web UI for browsing and
//! generating maps, for driving the vehicle - WASD control and a live map -
//! and for picking which vehicle physics model and autonomous algorithm are
//! running, on [`WebGuiConfig::bind_addr`]. Claims the writer slot for
//! `human_vesc_command`, `map_selection`, `vehicle_model_selection`,
//! `autonomous_algorithm_selection`, and `place_at_start` (see
//! [`live_api`]); reads `map`, `vehicle_model_status`, and
//! `autonomous_algorithm_status` to reflect the current selections.
//!
//! Everything on the map canvas comes from drawing topics (see
//! [`crate::topics::Drawing`] and [`draw_api`]): whatever every other
//! executor in the same [`crate::Runner`] chooses to draw - the map, the
//! vehicle, LIDAR hits, ... - without this module knowing about any of them.
//! The Topics panel likewise inspects any topic generically (see
//! [`topics_api`]).

mod assets;
mod draw_api;
mod handlers;
mod live_api;
mod maps_api;
mod topics_api;

use crate::topics::{
    AUTONOMOUS_ALGORITHM_SELECTION_TOPIC_NAME, AutonomousAlgorithmSelection,
    HUMAN_VESC_COMMAND_TOPIC_NAME, MAP_SELECTION_TOPIC_NAME, MapSelection,
    PLACE_AT_START_TOPIC_NAME, PlaceAtStart, VEHICLE_MODEL_SELECTION_TOPIC_NAME,
    VehicleModelSelection, VescCommand,
};
use crate::{Captain, Executor};
use std::any::Any;
use std::path::PathBuf;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

/// Every tunable parameter [`WebGui`] needs - loaded from
/// `config/sensors/web_gui.toml` (see [`Default`]) or from an arbitrary
/// path via [`crate::config::load`].
#[derive(Debug, Clone, PartialEq, serde::Deserialize)]
pub struct WebGuiConfig {
    /// Address [`WebGui`] binds its HTTP server to.
    pub bind_addr: String,
    /// Number of worker threads handling requests concurrently - so a slow
    /// request (e.g. generating a large map) doesn't stall every other
    /// client.
    pub worker_threads: usize,
    /// How long each worker blocks in `recv_timeout` before re-checking
    /// whether it should stop, in milliseconds - bounds shutdown latency
    /// without busy-waiting.
    pub poll_interval_ms: u64,
    /// Speed, in m/s, that a full W or S key press commands under WASD
    /// human control - served to the frontend via `GET /api/config` so the
    /// UI and server never drift apart.
    pub human_max_speed_mps: f64,
    /// Steering angle, in radians, that a full A or D key press commands
    /// under WASD human control - served to the frontend via
    /// `GET /api/config` so the UI and server never drift apart.
    pub human_max_steering_rad: f64,
}

impl Default for WebGuiConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/sensors/web_gui.toml"))
            .expect("config/sensors/web_gui.toml must deserialize into WebGuiConfig")
    }
}

/// Serves the map browser/generator web UI. Reads and writes map folders
/// under `maps_root` directly off disk.
pub struct WebGui {
    id: u8,
    name: String,
    maps_root: PathBuf,
    config: WebGuiConfig,
}

impl WebGui {
    /// Creates a `WebGui` that will serve maps under `maps_root` once run.
    pub fn new(
        name: impl Into<String>,
        maps_root: impl Into<PathBuf>,
        config: WebGuiConfig,
    ) -> Self {
        Self {
            id: 0,
            name: name.into(),
            maps_root: maps_root.into(),
            config,
        }
    }
}

impl Executor for WebGui {
    fn init(&mut self, id: u8) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_writer::<VescCommand>(
            HUMAN_VESC_COMMAND_TOPIC_NAME,
            self.id,
            VescCommand::default,
        );
        captain.claim_writer::<MapSelection>(
            MAP_SELECTION_TOPIC_NAME,
            self.id,
            MapSelection::default,
        );
        captain.claim_writer::<VehicleModelSelection>(
            VEHICLE_MODEL_SELECTION_TOPIC_NAME,
            self.id,
            VehicleModelSelection::default,
        );
        captain.claim_writer::<AutonomousAlgorithmSelection>(
            AUTONOMOUS_ALGORITHM_SELECTION_TOPIC_NAME,
            self.id,
            AutonomousAlgorithmSelection::default,
        );
        captain.claim_writer::<PlaceAtStart>(
            PLACE_AT_START_TOPIC_NAME,
            self.id,
            PlaceAtStart::default,
        );
    }

    fn run(&mut self, captain: &Captain) {
        let bind_addr = &self.config.bind_addr;
        let server = match crate::web::bind_http(bind_addr.as_str()) {
            Ok(server) => Arc::new(server),
            Err(err) => {
                eprintln!("{}: failed to bind {bind_addr}: {err}", self.name);
                return;
            }
        };
        println!("{}: serving on http://{bind_addr}", self.name);

        let id = self.id;
        let maps_root = &self.maps_root;
        let config = &self.config;
        let poll_interval = Duration::from_millis(self.config.poll_interval_ms);
        thread::scope(|scope| {
            for _ in 0..self.config.worker_threads {
                let server = server.clone();
                scope.spawn(move || {
                    while captain.is_running(id) {
                        match server.recv_timeout(poll_interval) {
                            Ok(Some(request)) => {
                                handlers::handle(request, maps_root, captain, id, config)
                            }
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

    fn fresh(&self) -> Box<dyn Executor> {
        Box::new(WebGui::new(
            self.name.clone(),
            self.maps_root.clone(),
            self.config.clone(),
        ))
    }
}
