//! Motor probe, for the bench with the wheels off the ground: ramps the
//! motor up to one low ERPM, holds it for a few seconds, then brakes and
//! releases it - printing the tachometer so wheel turns counted by eye give
//! the speed-to-ERPM gain, and which sign drives forward. Never moves the
//! steering servo.
//!
//! `cargo run --release --example motor_probe ERPM SECONDS [PORT]` - |ERPM|
//! at most [`MAX_ERPM`], SECONDS at most [`MAX_SECONDS`], the port
//! defaulting to `/dev/sensors/vesc`. Ctrl+C brakes early.
use aurorus::actuators::vesc::VescPort;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

/// The fastest this probe spins the motor, in ERPM.
const MAX_ERPM: i32 = 3000;
/// The longest it holds the speed, in seconds.
const MAX_SECONDS: f64 = 15.0;
/// How long it takes to ramp up to the speed, in seconds.
const RAMP_S: f64 = 0.5;
/// How often it resends the command (the VESC stops the motor on its own
/// if the commands stop coming) and reads the state.
const PERIOD: Duration = Duration::from_millis(20);
/// The current it brakes with, in amperes.
const BRAKE_A: f64 = 2.0;
/// The longest it brakes before releasing the motor anyway.
const BRAKE_FOR: Duration = Duration::from_secs(2);
/// Below this, in ERPM, the motor counts as stopped.
const STOPPED_ERPM: f64 = 100.0;

fn main() {
    let mut args = std::env::args().skip(1);
    let fail = |message: String| -> ! {
        eprintln!("{message}");
        std::process::exit(1);
    };
    let usage = "usage: motor_probe ERPM SECONDS [PORT]";
    let erpm: i32 = args
        .next()
        .and_then(|s| s.parse().ok())
        .unwrap_or_else(|| fail(usage.into()));
    let seconds: f64 = args
        .next()
        .and_then(|s| s.parse().ok())
        .unwrap_or_else(|| fail(usage.into()));
    if erpm == 0 || erpm.abs() > MAX_ERPM {
        fail(format!(
            "ERPM must be nonzero and within +-{MAX_ERPM}, got {erpm}"
        ));
    }
    if !(seconds > 0.0 && seconds <= MAX_SECONDS) {
        fail(format!(
            "SECONDS must be in (0, {MAX_SECONDS}], got {seconds}"
        ));
    }
    let path = args
        .next()
        .map_or_else(|| PathBuf::from("/dev/sensors/vesc"), PathBuf::from);

    let stop = Arc::new(AtomicBool::new(false));
    let handler_stop = stop.clone();
    ctrlc::set_handler(move || handler_stop.store(true, Ordering::SeqCst))
        .expect("failed to set Ctrl+C handler");

    let mut port = VescPort::open(&path, Duration::from_millis(100)).unwrap_or_else(|e| fail(e));
    let before = port.values().unwrap_or_else(|e| fail(e));
    if !before.fault.is_none() {
        fail(format!(
            "the VESC reports fault {} - not spinning",
            before.fault.name()
        ));
    }
    println!(
        "{:.2} V, tachometer {} - spinning at {erpm} ERPM for {seconds} s",
        before.input_voltage_v, before.tachometer
    );

    // Spin: ramp up, then hold. Any failure goes straight to braking.
    let start = Instant::now();
    let (mut last_print, mut held_erpm_s, mut held_s) = (start, 0.0, 0.0);
    let mut last_tick = start;
    let mut failure = None;
    while !stop.load(Ordering::SeqCst) && start.elapsed().as_secs_f64() < RAMP_S + seconds {
        let t = start.elapsed().as_secs_f64();
        let target = (f64::from(erpm) * (t / RAMP_S).min(1.0)).round() as i32;
        if let Err(err) = port.set_rpm(target) {
            failure = Some(err);
            break;
        }
        let values = match port.values() {
            Ok(values) => values,
            Err(err) => {
                failure = Some(err);
                break;
            }
        };
        if !values.fault.is_none() {
            failure = Some(format!("fault {}", values.fault.name()));
            break;
        }
        let dt = last_tick.elapsed().as_secs_f64();
        last_tick = Instant::now();
        if t >= RAMP_S {
            held_erpm_s += values.erpm * dt;
            held_s += dt;
        }
        if last_print.elapsed() >= Duration::from_millis(500) {
            last_print = Instant::now();
            println!(
                "  t {t:5.2} s  command {target:5}  erpm {:7.0}  motor {:+5.2} A  battery {:+5.2} A  tach {}",
                values.erpm, values.motor_current_a, values.input_current_a, values.tachometer
            );
        }
        std::thread::sleep(PERIOD);
    }

    // Brake until stopped, then release.
    let braking = Instant::now();
    while braking.elapsed() < BRAKE_FOR {
        if port.brake(BRAKE_A).is_err() {
            break;
        }
        match port.values() {
            Ok(values) if values.erpm.abs() < STOPPED_ERPM => break,
            Ok(_) => {}
            Err(_) => break,
        }
        std::thread::sleep(PERIOD);
    }
    let released = port.release();
    if let Some(err) = failure {
        eprintln!("stopped early: {err}");
    }
    if let Err(err) = released {
        eprintln!("{err} - the VESC's own timeout stops the motor");
    }

    std::thread::sleep(Duration::from_millis(300));
    let after = port.values().unwrap_or_else(|e| fail(e));
    let steps = after.tachometer - before.tachometer;
    println!(
        "stopped: tachometer {} ({steps:+} steps, {:+.1} electrical turns)",
        after.tachometer,
        f64::from(steps) / 6.0
    );
    if held_s > 0.0 {
        println!(
            "held {held_s:.2} s at {:.0} ERPM on average",
            held_erpm_s / held_s
        );
    }
    println!(
        "count the wheel turns N and measure the wheel diameter D (m): \
         speed_to_erpm_gain = {:.1} / (N * pi * D)",
        f64::from(steps) / 6.0 * 60.0
    );
}
