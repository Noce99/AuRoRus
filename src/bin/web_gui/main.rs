use aurorus::actuators::{ActuatorLimits, SimulatedVehicle, VehicleModel};
use aurorus::environment::simulator::vehicle::BicycleParams;
use aurorus::sensors::{MapServer, WebGui};
use aurorus::topics::{VESC_COMMAND_TOPIC_NAME, VescCommand};
use aurorus::{Executor, Runner};

mod cli;

/// Geometry and actuator limits for the simulated vehicle - a small
/// (roughly 1/10-scale) RC racecar, matching the kind of track
/// `generate_map` produces.
fn vehicle_model() -> VehicleModel {
    VehicleModel::Bicycle {
        params: BicycleParams { lf_m: 0.16, lr_m: 0.16 },
        limits: ActuatorLimits {
            max_steering_angle_rad: 0.4189, // 24 degrees
            max_steering_rate_rad_s: 4.0,
            max_speed_mps: 8.0,
            max_accel_mps2: 4.0,
            max_decel_mps2: 8.0,
        },
    }
}

fn main() {
    let config = cli::parse_config(std::env::args());

    let mut runner = Runner::new();
    runner.activate_verbose();

    // No autonomous controller publishes vesc_command yet - pre-seed it so
    // SimulatedVehicle can read it without a writer ever having claimed it.
    runner.register_topic(VESC_COMMAND_TOPIC_NAME, VescCommand::default());

    runner.add_executor(WebGui::new("WebGui", config.maps_root).boxed());
    runner.add_executor(MapServer::new("MapServer").boxed());
    runner.add_executor(SimulatedVehicle::new("SimulatedVehicle", vehicle_model()).boxed());
    runner.run_all();

    // Every executor here runs until stopped, and nothing ever stops them -
    // this blocks for the lifetime of the process, same as any other
    // long-running server; Ctrl+C just kills the process.
    runner.join_all();
}
