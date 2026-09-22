//! Command-line argument parsing for the map generator demo.

use aurorus::environment::GenerationConfig;
use std::path::PathBuf;

/// The demo's full configuration: a [`GenerationConfig`] plus the output
/// folder name override.
pub struct Config {
    pub generation: GenerationConfig,
    pub folder_name: Option<String>,
    /// Set by `--force`: replace an existing map of the same name instead
    /// of refusing to clobber it. See [`aurorus::environment::generate`].
    pub overwrite: bool,
}

/// Parses CLI flags from `argv`, each optional and falling back to
/// [`GenerationConfig::default`] (except `--seed`, which defaults to a
/// freshly rolled one so re-running without it produces a different map
/// every time). Prints usage and exits the process on `-h`/`--help`, an unknown
/// argument, a flag missing its value, or an invalid value.
pub fn parse_config(mut args: impl Iterator<Item = String>) -> Config {
    let program = args.next().unwrap_or_else(|| env!("CARGO_PKG_NAME").to_string());

    // A first pass just for --config-dir, so it can be used below to load
    // the defaults every other flag's help text and override base rely on -
    // the full parse (which needs those defaults already loaded) happens
    // further down.
    let raw_args: Vec<String> = args.collect();
    let config_dir = config_dir_from(&raw_args);
    let defaults = aurorus::config::load(&config_dir.join("environment/generation.toml"))
        .unwrap_or_else(|_| GenerationConfig::default());

    let usage = format!(
        "Usage: {program} [OPTIONS]\n\n\
         Options:\n  \
         --seed N                  RNG seed (default: freshly rolled)\n  \
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
         --force                    overwrite an existing map of that name\n  \
         --config-dir DIR           folder to load config/ files from (default: {:?})\n  \
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
        aurorus::config::DEFAULT_CONFIG_ROOT,
    );

    let fail = |message: String| -> ! {
        eprintln!("{message}\n");
        eprintln!("{usage}");
        std::process::exit(1);
    };

    let mut generation = GenerationConfig { seed: aurorus::environment::random_seed(), ..defaults };
    let mut folder_name = None;
    let mut overwrite = false;

    let mut args = raw_args.into_iter();
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
            "--force" => overwrite = true,
            "--config-dir" => {
                // Already consumed by config_dir_from() above to load
                // `defaults` - just skip its value here.
                next_value(&mut args, |s| Some(PathBuf::from(s))).unwrap_or_else(|| fail(invalid_message(&flag)));
            }
            other => fail(format!("Unknown argument '{other}'")),
        }
    }

    Config { generation, folder_name, overwrite }
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

/// Scans `args` for `--config-dir DIR`, falling back to
/// [`aurorus::config::DEFAULT_CONFIG_ROOT`] if it's absent - run before the
/// main flag-parsing loop so the config file it names can be loaded first,
/// to serve as every other flag's default.
fn config_dir_from(args: &[String]) -> PathBuf {
    args.iter()
        .position(|arg| arg == "--config-dir")
        .and_then(|i| args.get(i + 1))
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(aurorus::config::DEFAULT_CONFIG_ROOT))
}
