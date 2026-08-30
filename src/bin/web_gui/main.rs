use aurorus::actuators::{SimulatedVehicle, default_model};
use aurorus::sensors::{MapServer, WebGui};
use aurorus::topics::{VESC_COMMAND_TOPIC_NAME, VehicleModelKind, VescCommand};
use aurorus::{Executor, Runner};

mod cli;

fn main() {
    let config = cli::parse_config(std::env::args());

    let mut runner = Runner::new();
    runner.activate_verbose();

    // No autonomous controller publishes vesc_command yet - pre-seed it so
    // SimulatedVehicle can read it without a writer ever having claimed it.
    runner.register_topic(VESC_COMMAND_TOPIC_NAME, VescCommand::default());

    runner.add_executor(WebGui::new("WebGui", config.maps_root).boxed());
    runner.add_executor(MapServer::new("MapServer").boxed());
    runner.add_executor(SimulatedVehicle::new("SimulatedVehicle", default_model(VehicleModelKind::Bicycle)).boxed());
    runner.run_all();

    // Every executor here runs until stopped, and nothing ever stops them -
    // this blocks for the lifetime of the process, same as any other
    // long-running server; Ctrl+C just kills the process.
    runner.join_all();
}
