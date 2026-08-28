//! Command-line argument parsing for the benchmark's run duration.

pub const DEFAULT_DURATION_SECS: f64 = 10.0;

/// Parses the optional `[DURATION_SECS]` positional argument from `argv`, falling
/// back to [`DEFAULT_DURATION_SECS`]. Prints usage and exits the process on
/// `-h`/`--help` or an invalid value.
pub fn parse_duration_secs(mut args: impl Iterator<Item = String>) -> f64 {
    let program = args
        .next()
        .unwrap_or_else(|| env!("CARGO_PKG_NAME").to_string());
    let usage = format!(
        "Usage: {program} [DURATION_SECS]\n\n\
         Runs the benchmark for DURATION_SECS seconds (default: {DEFAULT_DURATION_SECS})."
    );

    match args.next() {
        None => DEFAULT_DURATION_SECS,
        Some(arg) if arg == "-h" || arg == "--help" => {
            println!("{usage}");
            std::process::exit(0);
        }
        Some(arg) => match arg.parse::<f64>() {
            Ok(secs) if secs > 0.0 => secs,
            _ => {
                eprintln!("Invalid DURATION_SECS '{arg}': expected a positive number of seconds\n");
                eprintln!("{usage}");
                std::process::exit(1);
            }
        },
    }
}
