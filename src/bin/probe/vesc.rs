//! `probe vesc`, read-only probe for the VESC: prints its firmware, then ten
//! times a second its state and IMU readings, and how long each question took.
//! Sends nothing that can drive the motor or the servo.
//!
//! `probe vesc [SECONDS] [PORT]` (default 10 seconds, `/dev/sensors/vesc`).
use aurorus::actuators::vesc::VescPort;
use std::path::PathBuf;
use std::time::{Duration, Instant};

pub const USAGE: &str = "vesc [SECONDS] [PORT]";

pub fn run(mut args: std::vec::IntoIter<String>) {
    let seconds: f64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(10.0);
    let path = args
        .next()
        .map_or_else(|| PathBuf::from("/dev/sensors/vesc"), PathBuf::from);

    let mut port = VescPort::open(&path, Duration::from_millis(200)).unwrap_or_else(|err| {
        eprintln!("{err}");
        std::process::exit(1);
    });
    match port.firmware() {
        Ok(fw) => println!(
            "{}: firmware {}.{:02} on {} (uuid {})",
            path.display(),
            fw.major,
            fw.minor,
            fw.hardware,
            fw.uuid
        ),
        Err(err) => {
            eprintln!("{err}");
            std::process::exit(1);
        }
    }

    let start = Instant::now();
    while start.elapsed().as_secs_f64() < seconds {
        let asked = Instant::now();
        let values = port.values();
        let values_ms = asked.elapsed().as_secs_f64() * 1e3;
        let asked = Instant::now();
        let imu = port.imu();
        let imu_ms = asked.elapsed().as_secs_f64() * 1e3;

        match values {
            Ok(v) => print!(
                "{:5.2} V  fault {:<13} timeout {:<5}  erpm {:7.0}  tach {:7}  duty {:+.3}  \
                 I {:+5.2}/{:+5.2} A  fet {:4.1} C  motor {:4.1} C  ({values_ms:.1} ms)",
                v.input_voltage_v,
                v.fault.name(),
                v.timed_out.map_or("?".into(), |t| t.to_string()),
                v.erpm,
                v.tachometer,
                v.duty,
                v.motor_current_a,
                v.input_current_a,
                v.temp_fet_c,
                v.temp_motor_c,
            ),
            Err(err) => print!("values: {err}"),
        }
        match imu {
            Ok(i) => {
                let norm = i.accel_g.iter().map(|a| a * a).sum::<f64>().sqrt();
                println!(
                    "\n    accel [{:+.3} {:+.3} {:+.3}] (|a| {norm:.3})  gyro [{:+7.2} {:+7.2} {:+7.2}]  \
                     rpy [{:+6.1} {:+6.1} {:+6.1}] deg  ({imu_ms:.1} ms)",
                    i.accel_g[0],
                    i.accel_g[1],
                    i.accel_g[2],
                    i.gyro_deg_s[0],
                    i.gyro_deg_s[1],
                    i.gyro_deg_s[2],
                    i.rpy_rad[0].to_degrees(),
                    i.rpy_rad[1].to_degrees(),
                    i.rpy_rad[2].to_degrees(),
                );
            }
            Err(err) => println!("\n    imu: {err}"),
        }
        std::thread::sleep(Duration::from_millis(100));
    }
}
