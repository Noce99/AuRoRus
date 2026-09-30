//! Steering servo probe: moves the VESC's servo output to one position and
//! exits, the servo holding it - for finding the centre and the end stops one
//! step at a time. Never sends a motor command, so the wheels can't spin.
//!
//! `cargo run --release --example servo_probe POSITION [PORT]`: POSITION is
//! in `0..=1` (0.5 is the pulse width's middle), or `center`; the port
//! defaults to `/dev/sensors/vesc`.
use aurorus::actuators::vesc::VescPort;
use std::path::PathBuf;
use std::time::Duration;

/// The only positions this probe sends - wider ones could drive the
/// steering linkage into its end stops before they're known.
const WINDOW: std::ops::RangeInclusive<f64> = 0.15..=0.85;

fn main() {
    let mut args = std::env::args().skip(1);
    let fail = |message: String| -> ! {
        eprintln!("{message}");
        std::process::exit(1);
    };
    let position = match args.next().as_deref() {
        Some("center") => 0.5,
        Some(value) => value
            .parse::<f64>()
            .unwrap_or_else(|_| fail(format!("not a position: {value:?}"))),
        None => fail("usage: servo_probe POSITION|center [PORT]".into()),
    };
    if !WINDOW.contains(&position) {
        fail(format!(
            "{position} is outside {:?}..={:?} - refusing to send it",
            WINDOW.start(),
            WINDOW.end()
        ));
    }
    let path = args.next().map_or_else(|| PathBuf::from("/dev/sensors/vesc"), PathBuf::from);

    let mut port = VescPort::open(&path, Duration::from_millis(200)).unwrap_or_else(|e| fail(e));
    port.set_servo(position).unwrap_or_else(|e| fail(e));
    // The servo command has no reply: asking for the state checks the VESC
    // got it, over a link that works, and isn't in a fault.
    let values = port.values().unwrap_or_else(|e| fail(e));
    println!(
        "servo at {position:.3} ({:.0} thousandths) - {:.2} V, fault {}",
        position * 1000.0,
        values.input_voltage_v,
        values.fault.name()
    );
}
