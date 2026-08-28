//! Command-line argument parsing for the benchmark's configuration.

pub const DEFAULT_TOPIC_SIZE: usize = 1000;
pub const DEFAULT_READERS_NUM: usize = 10;
pub const DEFAULT_WRITER_FREQUENCY_HZ: f64 = 50.0;
pub const DEFAULT_DURATION_SECS: f64 = 10.0;

/// The benchmark's full configuration, parsed from CLI flags by [`parse_config`].
pub struct Config {
    /// Number of `f32` values in the shared topic's payload.
    pub topic_size: usize,
    /// Total number of reader threads, spread across the 5 rate tiers.
    pub readers_num: usize,
    /// The writer's publish rate, in Hz.
    pub writer_frequency_hz: f64,
    /// How long to run the benchmark for, in seconds.
    pub duration_secs: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            topic_size: DEFAULT_TOPIC_SIZE,
            readers_num: DEFAULT_READERS_NUM,
            writer_frequency_hz: DEFAULT_WRITER_FREQUENCY_HZ,
            duration_secs: DEFAULT_DURATION_SECS,
        }
    }
}

/// Parses `--topic_size N`, `--readers_num N`, `--writer_frequency HZ`, and
/// `--duration SECS` from `argv`, in any order - each optional, falling back to
/// its default. Prints usage and exits the process on `-h`/`--help`, an unknown
/// argument, a flag missing its value, or an invalid (non-positive) value.
pub fn parse_config(mut args: impl Iterator<Item = String>) -> Config {
    let program = args
        .next()
        .unwrap_or_else(|| env!("CARGO_PKG_NAME").to_string());
    let usage = format!(
        "Usage: {program} [OPTIONS]\n\n\
         Options:\n  \
         --topic_size N         f32 values per topic payload (default: {DEFAULT_TOPIC_SIZE})\n  \
         --readers_num N        total reader threads across 5 rate tiers (default: {DEFAULT_READERS_NUM})\n  \
         --writer_frequency HZ  writer publish rate in Hz (default: {DEFAULT_WRITER_FREQUENCY_HZ})\n  \
         --duration SECS        benchmark duration in seconds (default: {DEFAULT_DURATION_SECS})\n  \
         -h, --help              print this message"
    );

    let fail = |message: String| -> ! {
        eprintln!("{message}\n");
        eprintln!("{usage}");
        std::process::exit(1);
    };

    let mut config = Config::default();

    while let Some(flag) = args.next() {
        match flag.as_str() {
            "-h" | "--help" => {
                println!("{usage}");
                std::process::exit(0);
            }
            "--topic_size" => {
                config.topic_size = next_value(&mut args, |s| {
                    s.parse::<usize>().ok().filter(|&v| v > 0)
                })
                .unwrap_or_else(|| fail(invalid_message(&flag)))
            }
            "--readers_num" => {
                config.readers_num = next_value(&mut args, |s| s.parse::<usize>().ok())
                    .unwrap_or_else(|| {
                        fail(format!(
                            "Missing or invalid value for {flag}: expected a non-negative number"
                        ))
                    })
            }
            "--writer_frequency" => {
                config.writer_frequency_hz =
                    next_value(&mut args, |s| s.parse::<f64>().ok().filter(|&v| v > 0.0))
                        .unwrap_or_else(|| fail(invalid_message(&flag)))
            }
            "--duration" => {
                config.duration_secs =
                    next_value(&mut args, |s| s.parse::<f64>().ok().filter(|&v| v > 0.0))
                        .unwrap_or_else(|| fail(invalid_message(&flag)))
            }
            other => fail(format!("Unknown argument '{other}'")),
        }
    }

    config
}

/// Consumes the next arg as the current flag's value and runs it through
/// `validate`, or `None` if there's no next arg at all (i.e. the flag was the
/// last argument) or it fails validation.
fn next_value<T>(
    args: &mut impl Iterator<Item = String>,
    validate: impl FnOnce(&str) -> Option<T>,
) -> Option<T> {
    args.next().and_then(|value| validate(&value))
}

fn invalid_message(flag: &str) -> String {
    format!("Missing or invalid value for {flag}: expected a positive number")
}
