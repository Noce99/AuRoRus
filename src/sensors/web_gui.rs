//! [`WebGui`]: an [`Executor`] that serves a local web UI for browsing and
//! generating maps, for driving the vehicle - WASD control and a live map -
//! and for picking which vehicle physics model and autonomous algorithm are
//! running, on [`WebGuiConfig::bind_addr`]. Claims the writer slot for
//! `human_vesc_command`, `map_selection`, `vehicle_model_selection`,
//! `autonomous_algorithm_selection`, `autonomous_parameters`,
//! `place_at_start`, `slam_command`, `slam_save`, `planning_parameters`,
//! `planning_request` (see [`live_api`]), `race_line_selection` (see
//! [`race_lines_api`]), `opponent_requests` and `race_start` (see
//! [`opponents_api`]);
//! reads `map`, `vehicle_model_status`, `autonomous_algorithm_status`,
//! `slam_status`, `planning_status`, `race_line` and `opponents` to reflect
//! the current selections. Starts and stops debug recordings through a
//! [`crate::DebugRecorder`] (see [`debug_api`]), and runs benchmarks (see
//! [`benchmark_api`]).
//!
//! Everything on the map canvas comes from drawing topics (see
//! [`crate::topics::Drawing`] and [`draw_api`]): whatever every other
//! executor in the same [`crate::Runner`] chooses to draw - the map, the
//! vehicle, LIDAR hits, ... - without this module knowing about any of them.
//! The Topics panel likewise inspects any topic generically (see
//! [`topics_api`]).

mod assets;
mod benchmark_api;
mod debug_api;
mod draw_api;
mod handlers;
mod live_api;
mod maps_api;
mod opponents_api;
mod race_lines_api;
mod topics_api;

use crate::telemetry::TelemetryPoseSource;
use crate::topics::{
    AUTONOMOUS_ALGORITHM_SELECTION_TOPIC_NAME, AUTONOMOUS_PARAMETERS_TOPIC_NAME,
    AutonomousAlgorithmSelection, AutonomousParameters, HUMAN_VESC_COMMAND_TOPIC_NAME,
    MAP_SELECTION_TOPIC_NAME, MapSelection, OPPONENT_REQUESTS_TOPIC_NAME, OpponentRequests,
    PLACE_AT_START_TOPIC_NAME, PLANNING_PARAMETERS_TOPIC_NAME, PLANNING_REQUEST_TOPIC_NAME,
    PlaceAtStart, PlanningParameters, PlanningRequest, RACE_LINE_SELECTION_TOPIC_NAME,
    RACE_START_TOPIC_NAME, RaceLineSelection, RaceStart, SLAM_COMMAND_TOPIC_NAME,
    SLAM_SAVE_TOPIC_NAME, SlamCommand, SlamSaveRequest, VEHICLE_MODEL_PARAMETERS_TOPIC_NAME,
    VEHICLE_MODEL_SELECTION_TOPIC_NAME, VehicleModelParameters, VehicleModelSelection, VescCommand,
};
use crate::{Captain, DebugRecorder, Executor};
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
    /// A race's starting grid: added to half a body length between one slot
    /// and the next, in meters - see
    /// [`crate::environment::starting_grid::GridSpacing::gap_m`].
    pub grid_gap_m: f64,
    /// A race's starting grid: between each vehicle's side and the
    /// centerline, in meters.
    pub grid_margin_m: f64,
    /// How long a race's countdown lasts before the vehicles are released,
    /// in milliseconds.
    pub race_countdown_ms: u64,
    /// How many laps one benchmark run drives - see [`benchmark_api`].
    pub benchmark_laps: u32,
    /// How often a benchmark run's trajectory is sampled, in Hz.
    pub benchmark_sample_rate_hz: f64,
    /// A benchmark lap taking longer than the map's centerline driven at
    /// this speed, in meters/second, ends the run as a timeout.
    pub benchmark_timeout_speed_mps: f64,
}

impl Default for WebGuiConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/sensors/web_gui.toml"))
            .expect("config/sensors/web_gui.toml must deserialize into WebGuiConfig")
    }
}

/// Where the Benchmark panel's runs go, and what `summary.toml` records
/// about the lap timing it relies on - see [`benchmark_api`].
#[derive(Debug, Clone, PartialEq)]
pub struct BenchmarkSetup {
    /// Folder every run is written under, one subfolder per map.
    pub root: PathBuf,
    /// Where `crate::telemetry::LapTelemetryRecorder` takes the pose it times
    /// laps with from.
    pub pose_source: TelemetryPoseSource,
}

/// Serves the map browser/generator web UI. Reads and writes map folders
/// under `maps_root` directly off disk, and records debug sessions into
/// `debugs_root`.
pub struct WebGui {
    id: u16,
    name: String,
    maps_root: PathBuf,
    debugs_root: PathBuf,
    /// Kept across restarts (see [`Executor::fresh`]), so the Debug panel
    /// still shows how the recording a restart ended went.
    recorder: DebugRecorder,
    benchmark_setup: BenchmarkSetup,
    /// Kept across restarts like `recorder`, so the Benchmark panel still
    /// shows the last results.
    benchmark: benchmark_api::Benchmark,
    config: WebGuiConfig,
}

impl WebGui {
    /// Creates a `WebGui` that will serve maps under `maps_root` once run,
    /// start recordings through `recorder` into `debugs_root`, and write
    /// benchmarks as `benchmark_setup` says.
    pub fn new(
        name: impl Into<String>,
        maps_root: impl Into<PathBuf>,
        debugs_root: impl Into<PathBuf>,
        recorder: DebugRecorder,
        benchmark_setup: BenchmarkSetup,
        config: WebGuiConfig,
    ) -> Self {
        Self {
            id: 0,
            name: name.into(),
            maps_root: maps_root.into(),
            debugs_root: debugs_root.into(),
            recorder,
            benchmark_setup,
            benchmark: benchmark_api::Benchmark::default(),
            config,
        }
    }
}

impl Executor for WebGui {
    fn init(&mut self, id: u16) {
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
        captain.claim_writer::<VehicleModelParameters>(
            VEHICLE_MODEL_PARAMETERS_TOPIC_NAME,
            self.id,
            VehicleModelParameters::default,
        );
        captain.claim_writer::<AutonomousAlgorithmSelection>(
            AUTONOMOUS_ALGORITHM_SELECTION_TOPIC_NAME,
            self.id,
            AutonomousAlgorithmSelection::default,
        );
        captain.claim_writer::<AutonomousParameters>(
            AUTONOMOUS_PARAMETERS_TOPIC_NAME,
            self.id,
            AutonomousParameters::default,
        );
        captain.claim_writer::<PlaceAtStart>(
            PLACE_AT_START_TOPIC_NAME,
            self.id,
            PlaceAtStart::default,
        );
        captain.claim_writer::<SlamCommand>(SLAM_COMMAND_TOPIC_NAME, self.id, SlamCommand::default);
        captain.claim_writer::<SlamSaveRequest>(
            SLAM_SAVE_TOPIC_NAME,
            self.id,
            SlamSaveRequest::default,
        );
        captain.claim_writer::<PlanningParameters>(
            PLANNING_PARAMETERS_TOPIC_NAME,
            self.id,
            PlanningParameters::default,
        );
        captain.claim_writer::<PlanningRequest>(
            PLANNING_REQUEST_TOPIC_NAME,
            self.id,
            PlanningRequest::default,
        );
        captain.claim_writer::<RaceLineSelection>(
            RACE_LINE_SELECTION_TOPIC_NAME,
            self.id,
            RaceLineSelection::default,
        );
        captain.claim_writer::<OpponentRequests>(
            OPPONENT_REQUESTS_TOPIC_NAME,
            self.id,
            OpponentRequests::default,
        );
        captain.claim_writer::<RaceStart>(RACE_START_TOPIC_NAME, self.id, RaceStart::default);
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
        let debug = handlers::Debug {
            root: &self.debugs_root,
            recorder: &self.recorder,
        };
        let benchmarks = handlers::Benchmarks {
            setup: &self.benchmark_setup,
            benchmark: &self.benchmark,
        };
        let config = &self.config;
        let poll_interval = Duration::from_millis(self.config.poll_interval_ms);
        let orchestrator = benchmark_api::Orchestrator {
            captain,
            id,
            config,
            maps_root,
            setup: &self.benchmark_setup,
            recorder: &self.recorder,
            benchmark: &self.benchmark,
        };
        thread::scope(|scope| {
            scope.spawn(|| benchmark_api::run_orchestrator(&orchestrator));
            for _ in 0..self.config.worker_threads {
                let server = server.clone();
                let debug = &debug;
                let benchmarks = &benchmarks;
                scope.spawn(move || {
                    while captain.is_running(id) {
                        match server.recv_timeout(poll_interval) {
                            Ok(Some(request)) => handlers::handle(
                                request, maps_root, debug, benchmarks, captain, id, config,
                            ),
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
        let mut fresh = WebGui::new(
            self.name.clone(),
            self.maps_root.clone(),
            self.debugs_root.clone(),
            self.recorder.clone(),
            self.benchmark_setup.clone(),
            self.config.clone(),
        );
        fresh.benchmark = self.benchmark.clone();
        Box::new(fresh)
    }
}
