//! Command-line argument parsing for the map generator demo.

use aurorus::environment::GenerationConfig;
use std::path::PathBuf;

/// The demo's full configuration: a [`GenerationConfig`] plus the output
/// folder name override.
pub struct Config {
    pub generation: GenerationConfig,
    pub folder_name: Option<String>,
}

/// Parses CLI flags from `argv`, each optional and falling back to
/// [`GenerationConfig::default`] (except `--seed`, which defaults to the
/// current unix time so re-running without it produces a fresh map every
/// time). Prints usage and exits the process on `-h`/`--help`, an unknown
/// argument, a flag missing its value, or an invalid value.
pub fn parse_config(mut args: impl Iterator<Item = String>) -> Config {
    let program = args.next().unwrap_or_else(|| env!("CARGO_PKG_NAME").to_string());
    let defaults = GenerationConfig::default();
    let usage = format!(
        "Usage: {program} [OPTIONS]\n\n\
         Options:\n  \
         --seed N                  RNG seed (default: current unix time)\n  \
         --sites N                 number of Voronoi seed points (default: {})\n  \
         --area-width M             bounded area width, meters (default: {})\n  \
         --area-height M            bounded area height, meters (default: {})\n  \
         --resolution M             meters per pixel (default: {})\n  \
         --track-width M            track width, meters (default: {})\n  \
         --spacing M                race-line point spacing, meters (default: {})\n  \
         --target-area-fraction F   0.0-1.0 (default: {})\n  \
         --max-speed M              m/s (default: {})\n  \
         --max-lateral-accel M      m/s^2 (default: {})\n  \
         --out DIR                  output root (default: {:?})\n  \
         --name NAME                override the generated folder's name\n  \
         -h, --help                 print this message",
        defaults.num_sites,
        defaults.area_width_m,
        defaults.area_height_m,
        defaults.resolution_m_per_px,
        defaults.track_width_m,
        defaults.point_spacing_m,
        defaults.target_area_fraction,
        defaults.max_speed_mps,
        defaults.max_lateral_accel_mps2,
        defaults.output_root,
    );

    let fail = |message: String| -> ! {
        eprintln!("{message}\n");
        eprintln!("{usage}");
        std::process::exit(1);
    };

    let default_seed = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    let mut generation = GenerationConfig { seed: default_seed, ..defaults };
    let mut folder_name = None;

    while let Some(flag) = args.next() {
        match flag.as_str() {
            "-h" | "--help" => {
                println!("{usage}");
                std::process::exit(0);
            }
            "--seed" => {
                generation.seed =
                    next_value(&mut args, |s| s.parse::<u64>().ok()).unwrap_or_else(|| fail(invalid_message(&flag)))
            }
            "--sites" => {
                generation.num_sites = next_value(&mut args, |s| s.parse::<usize>().ok().filter(|&v| v > 0))
                    .unwrap_or_else(|| fail(invalid_message(&flag)))
            }
            "--area-width" => {
                generation.area_width_m = next_value(&mut args, |s| s.parse::<f64>().ok().filter(|&v| v > 0.0))
                    .unwrap_or_else(|| fail(invalid_message(&flag)))
            }
            "--area-height" => {
                generation.area_height_m = next_value(&mut args, |s| s.parse::<f64>().ok().filter(|&v| v > 0.0))
                    .unwrap_or_else(|| fail(invalid_message(&flag)))
            }
            "--resolution" => {
                generation.resolution_m_per_px =
                    next_value(&mut args, |s| s.parse::<f64>().ok().filter(|&v| v > 0.0))
                        .unwrap_or_else(|| fail(invalid_message(&flag)))
            }
            "--track-width" => {
                generation.track_width_m = next_value(&mut args, |s| s.parse::<f64>().ok().filter(|&v| v > 0.0))
                    .unwrap_or_else(|| fail(invalid_message(&flag)))
            }
            "--spacing" => {
                generation.point_spacing_m = next_value(&mut args, |s| s.parse::<f64>().ok().filter(|&v| v > 0.0))
                    .unwrap_or_else(|| fail(invalid_message(&flag)))
            }
            "--target-area-fraction" => {
                generation.target_area_fraction = next_value(&mut args, |s| {
                    s.parse::<f64>().ok().filter(|&v| v > 0.0 && v < 1.0)
                })
                .unwrap_or_else(|| fail(invalid_message(&flag)))
            }
            "--max-speed" => {
                generation.max_speed_mps = next_value(&mut args, |s| s.parse::<f64>().ok().filter(|&v| v > 0.0))
                    .unwrap_or_else(|| fail(invalid_message(&flag)))
            }
            "--max-lateral-accel" => {
                generation.max_lateral_accel_mps2 =
                    next_value(&mut args, |s| s.parse::<f64>().ok().filter(|&v| v > 0.0))
                        .unwrap_or_else(|| fail(invalid_message(&flag)))
            }
            "--out" => {
                generation.output_root = next_value(&mut args, |s| Some(PathBuf::from(s)))
                    .unwrap_or_else(|| fail(invalid_message(&flag)))
            }
            "--name" => {
                folder_name = Some(
                    next_value(&mut args, |s| Some(s.to_string())).unwrap_or_else(|| fail(invalid_message(&flag))),
                )
            }
            other => fail(format!("Unknown argument '{other}'")),
        }
    }

    Config { generation, folder_name }
}

/// Consumes the next arg as the current flag's value and runs it through
/// `validate`, or `None` if there's no next arg at all (i.e. the flag was
/// the last argument) or it fails validation.
fn next_value<T>(args: &mut impl Iterator<Item = String>, validate: impl FnOnce(&str) -> Option<T>) -> Option<T> {
    args.next().and_then(|value| validate(&value))
}

fn invalid_message(flag: &str) -> String {
    format!("Missing or invalid value for {flag}")
}
