use aurorus::sensors::WebGui;
use aurorus::{Executor, Runner};

mod cli;

fn main() {
    let config = cli::parse_config(std::env::args());

    let mut runner = Runner::new();
    runner.activate_verbose();
    runner.add_executor(WebGui::new("WebGui", config.maps_root).boxed());
    runner.run_all();

    // WebGui runs until stopped, and nothing ever stops it - this blocks
    // for the lifetime of the process, same as any other long-running
    // server; Ctrl+C just kills the process.
    runner.join_all();
}
