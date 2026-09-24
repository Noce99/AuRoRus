//! Command-line argument parsing for the `replay_web_gui` binary.

use std::path::PathBuf;

/// Default address `replay_web_gui` binds its HTTP server to - distinct
/// from `web_gui`'s default (`1999`) so both can run side by side, e.g. to
/// compare a live session against a recorded one.
const DEFAULT_BIND_ADDR: &str = "0.0.0.0:1998";

pub struct Config {
    pub file: PathBuf,
    pub bind_addr: String,
}

/// Parses CLI flags from `argv`. Prints usage and exits the process on
/// `-h`/`--help`, an unknown argument, a flag missing its value, or a missing
/// required `--file`.
pub fn parse_config(mut args: impl Iterator<Item = String>) -> Config {
    let program = args
        .next()
        .unwrap_or_else(|| env!("CARGO_PKG_NAME").to_string());
    let usage = format!(
        "Usage: {program} --file PATH [OPTIONS]\n\n\
         Options:\n  \
         --file PATH        the .debug session file to serve for playback (required)\n  \
         --bind-addr ADDR   address to bind the HTTP server to (default: {DEFAULT_BIND_ADDR:?})\n  \
         -h, --help         print this message",
    );

    let fail = |message: String| -> ! {
        eprintln!("{message}\n");
        eprintln!("{usage}");
        std::process::exit(1);
    };

    let mut file: Option<PathBuf> = None;
    let mut bind_addr = DEFAULT_BIND_ADDR.to_string();

    while let Some(flag) = args.next() {
        match flag.as_str() {
            "-h" | "--help" => {
                println!("{usage}");
                std::process::exit(0);
            }
            "--file" => {
                file = Some(args.next().map(PathBuf::from).unwrap_or_else(|| {
                    fail(format!("Missing value for {flag}"));
                }))
            }
            "--bind-addr" => {
                bind_addr = args.next().unwrap_or_else(|| {
                    fail(format!("Missing value for {flag}"));
                })
            }
            other => fail(format!("Unknown argument '{other}'")),
        }
    }

    let file = file.unwrap_or_else(|| fail("Missing required argument --file".to_string()));
    Config { file, bind_addr }
}
