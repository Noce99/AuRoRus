//! Command-line argument parsing for the `benchmark_viewer` binary.

use std::path::PathBuf;

/// Default folder benchmarks are read from - where `web_gui`'s Benchmark
/// panel writes them by default.
const DEFAULT_BENCHMARKS_ROOT: &str = "benchmarks";
/// Default address `benchmark_viewer` binds its HTTP server to - distinct
/// from `web_gui`'s (`1999`) and `replay_web_gui`'s (`1998`), so all three
/// can run side by side.
const DEFAULT_BIND_ADDR: &str = "0.0.0.0:1997";

#[derive(Debug, PartialEq)]
pub struct Config {
    pub benchmarks_root: PathBuf,
    pub bind_addr: String,
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
         --benchmarks-root DIR  folder to read benchmarks from (default: {DEFAULT_BENCHMARKS_ROOT:?})\n  \
         --bind-addr ADDR       address to bind the HTTP server to (default: {DEFAULT_BIND_ADDR:?})\n  \
         -h, --help             print this message",
    );

    let fail = |message: String| -> ! {
        eprintln!("{message}\n");
        eprintln!("{usage}");
        std::process::exit(1);
    };

    let mut benchmarks_root = PathBuf::from(DEFAULT_BENCHMARKS_ROOT);
    let mut bind_addr = DEFAULT_BIND_ADDR.to_string();

    while let Some(flag) = args.next() {
        match flag.as_str() {
            "-h" | "--help" => {
                println!("{usage}");
                std::process::exit(0);
            }
            "--benchmarks-root" => {
                benchmarks_root = args.next().map(PathBuf::from).unwrap_or_else(|| {
                    fail(format!("Missing value for {flag}"));
                })
            }
            "--bind-addr" => {
                bind_addr = args.next().unwrap_or_else(|| {
                    fail(format!("Missing value for {flag}"));
                })
            }
            other => fail(format!("Unknown argument '{other}'")),
        }
    }

    Config {
        benchmarks_root,
        bind_addr,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(flags: &[&str]) -> Config {
        let mut argv = vec!["benchmark_viewer".to_string()];
        argv.extend(flags.iter().map(|s| s.to_string()));
        parse_config(argv.into_iter())
    }

    #[test]
    fn defaults_without_flags() {
        assert_eq!(
            args(&[]),
            Config {
                benchmarks_root: PathBuf::from(DEFAULT_BENCHMARKS_ROOT),
                bind_addr: DEFAULT_BIND_ADDR.to_string(),
            }
        );
    }

    #[test]
    fn flags_override_the_defaults() {
        assert_eq!(
            args(&[
                "--bind-addr",
                "127.0.0.1:5000",
                "--benchmarks-root",
                "elsewhere"
            ]),
            Config {
                benchmarks_root: PathBuf::from("elsewhere"),
                bind_addr: "127.0.0.1:5000".to_string(),
            }
        );
    }
}
