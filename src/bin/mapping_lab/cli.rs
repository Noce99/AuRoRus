//! Command-line argument parsing for the `mapping_lab` binary.

use std::path::PathBuf;

/// Default address `mapping_lab` binds its HTTP server to - below
/// `web_gui`'s (`1999`), `replay_web_gui`'s (`1998`), `benchmark_viewer`'s
/// (`1997`) and `car_calibration`'s (`1996`), so it can run beside any of
/// them.
const DEFAULT_BIND_ADDR: &str = "0.0.0.0:1995";
/// Default folder session files are listed from, matching `web_gui`'s.
const DEFAULT_SESSIONS_ROOT: &str = aurorus::web::gui::DEFAULT_MAPPING_SESSIONS_ROOT;
/// Default folder maps are saved into, matching `web_gui`'s.
const DEFAULT_MAPS_ROOT: &str = "maps";

pub struct Config {
    pub sessions_root: PathBuf,
    pub maps_root: PathBuf,
    pub config_dir: PathBuf,
    /// A session file to load right away.
    pub file: Option<PathBuf>,
    pub bind_addr: String,
    /// How many runs are built at once - `None` for one per CPU core but
    /// one.
    pub workers: Option<usize>,
}

/// Parses CLI flags from `argv`. Prints usage and exits the process on
/// `-h`/`--help`, an unknown argument, or a flag missing its value.
pub fn parse_config(mut args: impl Iterator<Item = String>) -> Config {
    let program = args
        .next()
        .unwrap_or_else(|| env!("CARGO_PKG_NAME").to_string());
    let usage = format!(
        "Usage: {program} [OPTIONS]\n\n\
         Options:\n  \
         --sessions-root DIR  folder to list .mapping session files from (default: {DEFAULT_SESSIONS_ROOT:?})\n  \
         --maps-root DIR      folder to save maps into (default: {DEFAULT_MAPS_ROOT:?})\n  \
         --config-dir DIR     folder localization/slam.toml is read from and saved to (default: {:?})\n  \
         --file PATH          a session file to load right away\n  \
         --workers N          how many runs are built at once (default: one per CPU core but one)\n  \
         --bind-addr ADDR     address to bind the HTTP server to (default: {DEFAULT_BIND_ADDR:?})\n  \
         -h, --help           print this message",
        aurorus::config::DEFAULT_CONFIG_ROOT,
    );

    let fail = |message: String| -> ! {
        eprintln!("{message}\n");
        eprintln!("{usage}");
        std::process::exit(1);
    };

    let mut config = Config {
        sessions_root: PathBuf::from(DEFAULT_SESSIONS_ROOT),
        maps_root: PathBuf::from(DEFAULT_MAPS_ROOT),
        config_dir: PathBuf::from(aurorus::config::DEFAULT_CONFIG_ROOT),
        file: None,
        bind_addr: DEFAULT_BIND_ADDR.to_string(),
        workers: None,
    };
    while let Some(flag) = args.next() {
        let mut value = || {
            args.next()
                .unwrap_or_else(|| fail(format!("Missing value for {flag}")))
        };
        match flag.as_str() {
            "-h" | "--help" => {
                println!("{usage}");
                std::process::exit(0);
            }
            "--sessions-root" => config.sessions_root = PathBuf::from(value()),
            "--maps-root" => config.maps_root = PathBuf::from(value()),
            "--config-dir" => config.config_dir = PathBuf::from(value()),
            "--file" => config.file = Some(PathBuf::from(value())),
            "--bind-addr" => config.bind_addr = value(),
            "--workers" => {
                let text = value();
                match text.parse::<usize>() {
                    Ok(workers) if workers > 0 => config.workers = Some(workers),
                    _ => fail(format!("Invalid value for {flag}: {text:?}")),
                }
            }
            other => fail(format!("Unknown argument '{other}'")),
        }
    }
    config
}
