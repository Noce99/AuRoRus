use aurorus::environment::generate;
use std::time::Instant;

mod cli;

fn main() {
    let config = cli::parse_config(std::env::args());
    let start = Instant::now();

    match generate(&config.generation, config.folder_name.as_deref()) {
        Ok(map) => {
            let elapsed = start.elapsed();
            println!("Generated map at {}", map.folder.display());
            println!("  image: {}x{} px", map.width_px, map.height_px);
            println!("  race line points: {}", map.num_race_line_points);
            println!("  seed: {}", config.generation.seed);
            println!("  elapsed: {elapsed:.2?}");
        }
        Err(err) => {
            eprintln!("error: {err}");
            std::process::exit(1);
        }
    }
}
