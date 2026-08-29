//! Command-line argument parsing for the `web_gui` binary.

use std::path::PathBuf;

/// Default folder `web_gui` serves maps from, matching
/// `GenerationConfig::default().output_root`.
const DEFAULT_MAPS_ROOT: &str = "maps";

pub struct Config {
    pub maps_root: PathBuf,
}

/// Parses CLI flags from `argv`. Prints usage and exits the process on
/// `-h`/`--help`, an unknown argument, or a flag missing its value.
pub fn parse_config(mut args: impl Iterator<Item = String>) -> Config {
    let program = args.next().unwrap_or_else(|| env!("CARGO_PKG_NAME").to_string());
    let usage = format!(
        "Usage: {program} [OPTIONS]\n\n\
         Options:\n  \
         --maps-root DIR   folder to serve/generate maps from (default: {DEFAULT_MAPS_ROOT:?})\n  \
         -h, --help        print this message"
    );

    let fail = |message: String| -> ! {
        eprintln!("{message}\n");
        eprintln!("{usage}");
        std::process::exit(1);
    };

    let mut maps_root = PathBuf::from(DEFAULT_MAPS_ROOT);

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
            other => fail(format!("Unknown argument '{other}'")),
        }
    }

    Config { maps_root }
}
