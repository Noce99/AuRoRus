use aurorus::actuators::{SimulatedVehicle, SimulatedVehicleConfig, default_model};
use aurorus::autonomous_control::{self, AutonomousControlsHandler};
use aurorus::localization::{DeadReckoning, DeadReckoningConfig, Slam, SlamConfig};
use aurorus::opponents::OpponentsManager;
use aurorus::planning::{Planner, PlanningConfig};
use aurorus::sensors::{
    BenchmarkSetup, MapServer, MapServerConfig, SimulatedImu, SimulatedImuConfig, SimulatedLidar,
    SimulatedLidarConfig, WebGui, WebGuiConfig,
};
use aurorus::telemetry::{LapTelemetryConfig, LapTelemetryRecorder};
use aurorus::topics::VehicleModelKind;
use aurorus::{DEBUG_GROUP, DebugRecorder, Executor, Runner};

mod cli;

fn main() {
    let config = cli::parse_config(std::env::args());

    let web_gui_config = aurorus::config::load(&config.config_dir.join("sensors/web_gui.toml"))
        .unwrap_or_else(|_| WebGuiConfig::default());
    let map_server_config =
        aurorus::config::load(&config.config_dir.join("sensors/map_server.toml"))
            .unwrap_or_else(|_| MapServerConfig::default());
    let simulated_lidar_config =
        aurorus::config::load(&config.config_dir.join("sensors/simulated_lidar.toml"))
            .unwrap_or_else(|_| SimulatedLidarConfig::default());
    let simulated_imu_config =
        aurorus::config::load(&config.config_dir.join("sensors/simulated_imu.toml"))
            .unwrap_or_else(|_| SimulatedImuConfig::default());
    let dead_reckoning_config =
        aurorus::config::load(&config.config_dir.join("localization/dead_reckoning.toml"))
            .unwrap_or_else(|_| DeadReckoningConfig::default());
    let slam_config = aurorus::config::load(&config.config_dir.join("localization/slam.toml"))
        .unwrap_or_else(|_| SlamConfig::default());
    let vehicle_config =
        aurorus::config::load(&config.config_dir.join("actuators/simulated_vehicle.toml"))
            .unwrap_or_else(|_| SimulatedVehicleConfig::default());
    let planning_config = aurorus::config::load(&config.config_dir.join("planning/race_line.toml"))
        .unwrap_or_else(|_| PlanningConfig::default());

    let lap_telemetry_config =
        aurorus::config::load(&config.config_dir.join("telemetry/lap_telemetry.toml"))
            .unwrap_or_else(|_| LapTelemetryConfig::default());

    let mut runner = Runner::new();
    runner.activate_verbose();

    let recorder = DebugRecorder::new();
    runner.add_executor(
        WebGui::new(
            "WebGui",
            config.maps_root.clone(),
            config.debugs_root.clone(),
            recorder.clone(),
            BenchmarkSetup {
                root: config.benchmarks_root.clone(),
                pose_source: lap_telemetry_config.pose_source,
            },
            web_gui_config,
        )
        .boxed(),
    );
    runner.add_executor(MapServer::new("MapServer", map_server_config).boxed());
    runner.add_executor(SimulatedLidar::new("SimulatedLidar", simulated_lidar_config).boxed());
    runner.add_executor(SimulatedImu::new("SimulatedImu", simulated_imu_config).boxed());
    runner.add_executor(DeadReckoning::new("DeadReckoning", dead_reckoning_config).boxed());
    runner.add_executor(Slam::new("Slam", config.maps_root, slam_config).boxed());
    runner.add_executor(Planner::new("Planner", planning_config).boxed());
    // Opponents copy the ego vehicle's configs - see `OpponentsManager`.
    runner.add_executor(
        OpponentsManager::new(
            "OpponentsManager",
            vehicle_config.clone(),
            simulated_lidar_config,
        )
        .boxed(),
    );
    runner.add_executor(
        SimulatedVehicle::new(
            "SimulatedVehicle",
            default_model(VehicleModelKind::Bicycle, &vehicle_config),
            vehicle_config,
        )
        .boxed(),
    );
    runner.add_executor(
        LapTelemetryRecorder::new("LapTelemetryRecorder", lap_telemetry_config).boxed(),
    );
    runner.add_executor(AutonomousControlsHandler::new("AutonomousControlsHandler").boxed());
    // Every file in src/autonomous_control/ - see `autonomous_control`'s docs.
    for algorithm in autonomous_control::all() {
        runner.add_executor(algorithm);
    }

    // `--debug`: the same recording the Debug panel starts, just started
    // along with everything else.
    if let Some(path) = config.debug_output {
        match recorder.begin(path, config.debug_frequency_hz) {
            Ok(executor) => {
                runner.add_group(DEBUG_GROUP, vec![executor]);
            }
            Err(err) => eprintln!("--debug: {err}"),
        }
    }

    // Every executor here runs until stopped, and nothing ever stops them for
    // good - this blocks for the lifetime of the process, same as any other
    // long-running server. Any executor can still ask for a full restart via
    // `Captain::request_restart`, which `run_until_stopped` handles by
    // rebuilding everything fresh and looping.
    //
    // Ctrl+C stops every executor cleanly (rather than just killing the
    // process) so a running debug recording gets flushed and closed properly: it
    // flips `Captain`'s running flag, every executor's `while
    // captain.is_running(id)` loop (including the debug executor's) exits on
    // its own, and `run_until_stopped` below only returns once every one of
    // those threads - including the debug executor's final flush - has
    // actually finished.
    let stop_handle = runner.stop_handle();
    ctrlc::set_handler(move || {
        println!("Ctrl+C received - stopping...");
        stop_handle.stop();
    })
    .expect("failed to set Ctrl+C handler");

    runner.run_until_stopped();
}
