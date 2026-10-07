//! `probe`: small bring-up tools for the real car, each testing one piece of
//! hardware on its own, without the rest of the stack - see
//! `documentation/car_calibration.md`.
//!
//! `probe <vesc|hokuyo|servo|motor> [ARGS]`; `probe` alone lists them.
//! Run from the repository root, like the other binaries.

mod hokuyo;
mod motor;
mod servo;
mod vesc;

/// One subcommand.
struct Probe {
    name: &'static str,
    usage: &'static str,
    description: &'static str,
    run: fn(std::vec::IntoIter<String>),
}

const PROBES: [Probe; 4] = [
    Probe {
        name: "vesc",
        usage: vesc::USAGE,
        description: "Read-only: prints the VESC's firmware, state and IMU ten times a second.",
        run: vesc::run,
    },
    Probe {
        name: "hokuyo",
        usage: hokuyo::USAGE,
        description: "Prints the LIDAR's scan rate and what it sees ahead, left, right and closest.",
        run: hokuyo::run,
    },
    Probe {
        name: "servo",
        usage: servo::USAGE,
        description: "Moves the steering servo to one position (0.15..=0.85) and exits. Never spins the motor.",
        run: servo::run,
    },
    Probe {
        name: "motor",
        usage: motor::USAGE,
        description: "Wheels off the ground: spins the motor at a low ERPM, then brakes. Never moves the servo.",
        run: motor::run,
    },
];

fn main() {
    let mut args = std::env::args().skip(1);
    let command = args.next();
    let rest: Vec<String> = args.collect();
    match PROBES
        .iter()
        .find(|probe| Some(probe.name) == command.as_deref())
    {
        Some(probe) => (probe.run)(rest.into_iter()),
        None => {
            if let Some(command) =
                command.filter(|c| !matches!(c.as_str(), "-h" | "--help" | "help"))
            {
                eprintln!("unknown probe {command:?}\n");
            }
            eprintln!("usage: probe <PROBE> [ARGS]\n");
            for probe in &PROBES {
                eprintln!("  probe {}\n      {}", probe.usage, probe.description);
            }
            std::process::exit(2);
        }
    }
}
