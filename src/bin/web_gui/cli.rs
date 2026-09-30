//! Command-line argument parsing for the `web_gui` binary.

use std::path::{Path, PathBuf};

/// Default folder `web_gui` serves maps from, matching
/// `GenerationConfig::default().output_root`.
const DEFAULT_MAPS_ROOT: &str = "maps";
/// Folder debug sessions are recorded into - from the Debug panel, or by a
/// `--debug` with no explicit path. A sibling of
/// `DEFAULT_MAPS_ROOT`/`aurorus::config::DEFAULT_CONFIG_ROOT`.
const DEFAULT_DEBUGS_ROOT: &str = "debugs";
/// Default folder the Benchmark panel writes its runs into.
const DEFAULT_BENCHMARKS_ROOT: &str = "benchmarks";
/// Default recording rate for `--debug`, in Hz, when `--debug_frequency` isn't given.
const DEFAULT_DEBUG_FREQUENCY_HZ: f64 = aurorus::DEFAULT_DEBUG_FREQUENCY_HZ;

pub struct Config {
    pub maps_root: PathBuf,
    pub config_dir: PathBuf,
    /// Folder the Debug panel records into.
    pub debugs_root: PathBuf,
    /// Folder the Benchmark panel writes its runs into.
    pub benchmarks_root: PathBuf,
    /// The fully resolved `.debug` file path to record to, or `None` if
    /// `--debug` wasn't given at all. See [`resolve_debug_output`]. Recording
    /// starts right away, as if started from the Debug panel.
    pub debug_output: Option<PathBuf>,
    pub debug_frequency_hz: f64,
    /// `--sim`: simulate, even on a car.
    pub sim: bool,
    /// `--car NAME`: the car to run as, instead of the one `CAR_NAME` names
    /// (see [`aurorus::hardware::read_car_name`]).
    pub car: Option<String>,
}

/// The raw, unresolved state of `--debug`, before filesystem rules are applied -
/// see [`resolve_debug_output`].
enum DebugCliArg {
    /// `--debug` wasn't given at all.
    Disabled,
    /// `--debug` with no following path.
    Generated,
    /// `--debug PATH`.
    Explicit(PathBuf),
}

/// Resolves a raw `--debug` argument into a concrete target file path, per the
/// user-facing contract: no path given records into [`DEFAULT_DEBUGS_ROOT`]
/// under a generated name; an existing directory gets a generated name inside
/// it; anything else is used verbatim as the target file (parent directories
/// created as needed, overwritten if it already exists).
fn resolve_debug_output(arg: DebugCliArg) -> Option<PathBuf> {
    match arg {
        DebugCliArg::Disabled => None,
        DebugCliArg::Generated => Some(
            Path::new(DEFAULT_DEBUGS_ROOT)
                .join(aurorus::debug_format::DebugFileReader::generated_filename()),
        ),
        DebugCliArg::Explicit(path) if path.is_dir() => {
            Some(path.join(aurorus::debug_format::DebugFileReader::generated_filename()))
        }
        DebugCliArg::Explicit(path) => Some(path),
    }
}

/// Parses CLI flags from `argv`. Prints usage and exits the process on
/// `-h`/`--help`, an unknown argument, or a flag missing its value.
pub fn parse_config(args: impl Iterator<Item = String>) -> Config {
    let mut args = args.peekable();
    let program = args
        .next()
        .unwrap_or_else(|| env!("CARGO_PKG_NAME").to_string());
    let usage = format!(
        "Usage: {program} [OPTIONS]\n\n\
         Options:\n  \
         --maps-root DIR       folder to serve/generate maps from (default: {DEFAULT_MAPS_ROOT:?})\n  \
         --config-dir DIR      folder to load config/ files from (default: {:?})\n  \
         --benchmarks-root DIR folder the Benchmark panel writes into (default: {DEFAULT_BENCHMARKS_ROOT:?})\n  \
         --debug [PATH]        start recording every written topic to a .debug session file right\n  \
         \x20                    away, until stopped from the Debug panel, a restart, or Ctrl+C -\n  \
         \x20                    no PATH: {DEFAULT_DEBUGS_ROOT:?}/<generated name>.debug\n  \
         \x20                    PATH is a directory: <PATH>/<generated name>.debug\n  \
         \x20                    otherwise: PATH itself (overwritten if it already exists)\n  \
         --debug_frequency HZ  debug recording rate, in Hz (default: {DEFAULT_DEBUG_FREQUENCY_HZ})\n  \
         --sim                 simulate, even on a car - without it, a machine whose CAR_NAME\n  \
         \x20                    file names a car (config/hardware/<car>.toml) runs on it: its\n  \
         \x20                    Hokuyo lidar and VESC, and no simulated opponents\n  \
         --car NAME            the car to run as, instead of the one CAR_NAME names\n  \
         -h, --help            print this message",
        aurorus::config::DEFAULT_CONFIG_ROOT,
    );

    let fail = |message: String| -> ! {
        eprintln!("{message}\n");
        eprintln!("{usage}");
        std::process::exit(1);
    };

    let mut maps_root = PathBuf::from(DEFAULT_MAPS_ROOT);
    let mut config_dir = PathBuf::from(aurorus::config::DEFAULT_CONFIG_ROOT);
    let mut benchmarks_root = PathBuf::from(DEFAULT_BENCHMARKS_ROOT);
    let mut debug_arg = DebugCliArg::Disabled;
    let mut debug_frequency_hz = DEFAULT_DEBUG_FREQUENCY_HZ;
    let mut sim = false;
    let mut car = None;

    while let Some(flag) = args.next() {
        match flag.as_str() {
            "-h" | "--help" => {
                println!("{usage}");
                std::process::exit(0);
            }
            "--maps-root" => {
                maps_root = args.next().map(PathBuf::from).unwrap_or_else(|| {
                    fail(format!("Missing value for {flag}"));
                })
            }
            "--config-dir" => {
                config_dir = args.next().map(PathBuf::from).unwrap_or_else(|| {
                    fail(format!("Missing value for {flag}"));
                })
            }
            "--benchmarks-root" => {
                benchmarks_root = args.next().map(PathBuf::from).unwrap_or_else(|| {
                    fail(format!("Missing value for {flag}"));
                })
            }
            "--debug" => {
                debug_arg = match args.peek() {
                    Some(next) if !next.starts_with('-') => {
                        DebugCliArg::Explicit(PathBuf::from(args.next().unwrap()))
                    }
                    _ => DebugCliArg::Generated,
                };
            }
            "--debug_frequency" => {
                let value = args
                    .next()
                    .unwrap_or_else(|| fail(format!("Missing value for {flag}")));
                debug_frequency_hz = value.parse().unwrap_or_else(|_| {
                    fail(format!("Invalid numeric value for {flag}: {value:?}"))
                });
            }
            "--sim" => sim = true,
            "--car" => {
                let name = args
                    .next()
                    .unwrap_or_else(|| fail(format!("Missing value for {flag}")));
                if let Err(err) = aurorus::hardware::valid_name(&name) {
                    fail(err);
                }
                car = Some(name);
            }
            other => fail(format!("Unknown argument '{other}'")),
        }
    }

    Config {
        maps_root,
        config_dir,
        debugs_root: PathBuf::from(DEFAULT_DEBUGS_ROOT),
        benchmarks_root,
        debug_output: resolve_debug_output(debug_arg),
        debug_frequency_hz,
        sim,
        car,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(flags: &[&str]) -> Config {
        let mut argv = vec!["web_gui".to_string()];
        argv.extend(flags.iter().map(|s| s.to_string()));
        parse_config(argv.into_iter())
    }

    #[test]
    fn debug_absent_means_no_recording() {
        let config = args(&[]);
        assert_eq!(config.debug_output, None);
        assert_eq!(config.debug_frequency_hz, DEFAULT_DEBUG_FREQUENCY_HZ);
    }

    #[test]
    fn debug_with_no_path_resolves_under_the_default_debugs_root() {
        let config = args(&["--debug"]);
        let path = config
            .debug_output
            .expect("--debug with no path should still enable recording");
        assert_eq!(path.parent(), Some(Path::new(DEFAULT_DEBUGS_ROOT)));
        assert!(path.extension().is_some_and(|ext| ext == "debug"));
    }

    #[test]
    fn debug_followed_by_another_flag_is_treated_as_no_path() {
        let config = args(&["--debug", "--debug_frequency", "30"]);
        let path = config
            .debug_output
            .expect("--debug should still enable recording");
        assert_eq!(path.parent(), Some(Path::new(DEFAULT_DEBUGS_ROOT)));
        assert_eq!(config.debug_frequency_hz, 30.0);
    }

    #[test]
    fn debug_with_an_existing_directory_gets_a_generated_name_inside_it() {
        let dir =
            std::env::temp_dir().join(format!("aurorus_cli_tests_dir_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let config = args(&["--debug", dir.to_str().unwrap()]);
        let path = config
            .debug_output
            .expect("--debug with a directory path should still enable recording");
        assert_eq!(path.parent(), Some(dir.as_path()));
        assert!(path.extension().is_some_and(|ext| ext == "debug"));
    }

    #[test]
    fn debug_with_a_non_existent_path_is_used_verbatim() {
        let path = std::env::temp_dir()
            .join(format!("aurorus_cli_tests_file_{}", std::process::id()))
            .join("session.debug");
        let config = args(&["--debug", path.to_str().unwrap()]);
        assert_eq!(config.debug_output, Some(path));
    }

    #[test]
    fn sim_and_car_are_read() {
        let config = args(&[]);
        assert!(!config.sim);
        assert_eq!(config.car, None);
        let config = args(&["--sim", "--car", "tom"]);
        assert!(config.sim);
        assert_eq!(config.car.as_deref(), Some("tom"));
    }

    #[test]
    fn debug_frequency_defaults_without_debug() {
        let config = args(&["--debug_frequency", "42"]);
        assert_eq!(config.debug_output, None);
        assert_eq!(config.debug_frequency_hz, 42.0);
    }
}
