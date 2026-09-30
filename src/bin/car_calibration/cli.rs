//! Command-line argument parsing for the `car_calibration` binary.

use std::path::PathBuf;

/// Default address the page is served on - distinct from `web_gui`'s
/// (`1999`), `replay_web_gui`'s (`1998`) and `benchmark_viewer`'s (`1997`).
const DEFAULT_BIND_ADDR: &str = "0.0.0.0:1996";

#[derive(Debug, PartialEq)]
pub struct Config {
    pub config_dir: PathBuf,
    pub bind_addr: String,
    /// The car to calibrate - else the one `CAR_NAME` names, else it's asked
    /// on the page.
    pub car: Option<String>,
}

/// Parses CLI flags from `argv`. Prints usage and exits the process on
/// `-h`/`--help`, an unknown argument, or a flag missing its value.
pub fn parse_config(mut args: impl Iterator<Item = String>) -> Config {
    let program = args
        .next()
        .unwrap_or_else(|| env!("CARGO_PKG_NAME").to_string());
    let usage = format!(
        "Usage: {program} [OPTIONS]\n\n\
         A guided calibration of the car's hardware - its size, IMU and lidar mounting,\n\
         steering and motor - served as a web page. Writes config/hardware/<car>.toml\n\
         only when you save.\n\n\
         Options:\n  \
         --car NAME         the car to calibrate (default: the one CAR_NAME names, else asked)\n  \
         --config-dir DIR   folder to load config/ files from (default: {:?})\n  \
         --bind-addr ADDR   address to serve the page on (default: {DEFAULT_BIND_ADDR:?})\n  \
         -h, --help         print this message",
        aurorus::config::DEFAULT_CONFIG_ROOT,
    );

    let fail = |message: String| -> ! {
        eprintln!("{message}\n");
        eprintln!("{usage}");
        std::process::exit(1);
    };

    let mut config_dir = PathBuf::from(aurorus::config::DEFAULT_CONFIG_ROOT);
    let mut bind_addr = DEFAULT_BIND_ADDR.to_string();
    let mut car = None;

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
            "--config-dir" => config_dir = PathBuf::from(value()),
            "--bind-addr" => bind_addr = value(),
            "--car" => {
                let name = value();
                if let Err(err) = aurorus::hardware::valid_name(&name) {
                    fail(err);
                }
                car = Some(name);
            }
            other => fail(format!("Unknown argument '{other}'")),
        }
    }

    Config {
        config_dir,
        bind_addr,
        car,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(flags: &[&str]) -> Config {
        let argv = std::iter::once("car_calibration").chain(flags.iter().copied());
        parse_config(argv.map(String::from))
    }

    #[test]
    fn defaults_and_flags() {
        assert_eq!(
            args(&[]),
            Config {
                config_dir: PathBuf::from("config"),
                bind_addr: DEFAULT_BIND_ADDR.to_string(),
                car: None,
            }
        );
        let config = args(&["--car", "tom", "--bind-addr", "127.0.0.1:1"]);
        assert_eq!(config.car.as_deref(), Some("tom"));
        assert_eq!(config.bind_addr, "127.0.0.1:1");
    }
}
