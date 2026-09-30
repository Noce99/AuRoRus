//! Probe for the real LIDAR: runs [`HokuyoLidar`] alone and, every half
//! second, prints the scan rate and what it sees ahead, left, right and
//! closest - e.g. to check the mounting by holding a hand on one side.
//!
//! `cargo run --release --example hokuyo_probe [SECONDS]` (default 10), with
//! `config/sensors/hokuyo_lidar.toml` and the mounting of the car `CAR_NAME`
//! names (unmounted without one).
use aurorus::sensors::{HokuyoLidar, HokuyoLidarConfig, LidarMounting};
use aurorus::topics::{LIDAR_SCAN_TOPIC_NAME, LidarScan};
use aurorus::{Captain, Executor, Runner};
use std::any::Any;
use std::path::Path;
use std::time::{Duration, Instant};

struct Probe {
    id: u16,
}

impl Executor for Probe {
    fn init(&mut self, id: u16) {
        self.id = id;
    }
    fn run(&mut self, captain: &Captain) {
        let scan_topic = captain.topic::<LidarScan>(LIDAR_SCAN_TOPIC_NAME);
        let (mut last_print, mut last_written, mut scans) = (Instant::now(), None, 0);
        while captain.is_running(self.id) {
            let scan = scan_topic.read();
            if scan.meta.written_at != last_written {
                last_written = scan.meta.written_at;
                scans += 1;
            }
            if last_print.elapsed() >= Duration::from_millis(500) && scan.num_lidar_points > 0 {
                let rate_hz = f64::from(scans) / last_print.elapsed().as_secs_f64();
                (last_print, scans) = (Instant::now(), 0);
                print_scan(&scan, rate_hz);
            }
            std::thread::sleep(Duration::from_millis(2));
        }
    }
    fn name(&self) -> String {
        "Probe".into()
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn fresh(&self) -> Box<dyn Executor> {
        Box::new(Probe { id: 0 })
    }
}

/// The reading closest to `angle_deg`, as text.
fn at(scan: &LidarScan, angle_deg: f32) -> String {
    let index = (0..scan.num_lidar_points)
        .min_by(|&a, &b| {
            let off = |i: usize| (scan.angle_rad(i).to_degrees() - angle_deg).abs();
            off(a).total_cmp(&off(b))
        })
        .unwrap();
    match scan.points[index] {
        d if d >= scan.max_distance => "  miss".into(),
        d => format!("{d:5.2}m"),
    }
}

fn print_scan(scan: &LidarScan, rate_hz: f64) {
    let hits: Vec<usize> = (0..scan.num_lidar_points)
        .filter(|&i| scan.points[i] < scan.max_distance)
        .collect();
    let closest = hits
        .iter()
        .min_by(|&&a, &&b| scan.points[a].total_cmp(&scan.points[b]));
    let closest = closest.map_or("none".into(), |&i| {
        format!(
            "{:.2}m at {:+.1} deg",
            scan.points[i],
            scan.angle_rad(i).to_degrees()
        )
    });
    println!(
        "{rate_hz:4.1} Hz  {} pts ({} hits)  left(-90) {}  ahead {}  right(+90) {}  closest {closest}",
        scan.num_lidar_points,
        hits.len(),
        at(scan, -90.0),
        at(scan, 0.0),
        at(scan, 90.0),
    );
}

fn main() {
    let seconds: f64 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(10.0);
    let config = aurorus::config::load(Path::new("config/sensors/hokuyo_lidar.toml"))
        .unwrap_or_else(|_| HokuyoLidarConfig::default());

    let mut runner = Runner::new();
    let mounting = match aurorus::hardware::load_this_car(Path::new("config")) {
        Ok(Some(car)) => LidarMounting::of(&car),
        Ok(None) => LidarMounting::default(),
        Err(err) => panic!("{err}"),
    };
    runner.add_executor(HokuyoLidar::new("HokuyoLidar", config, mounting).boxed());
    runner.add_executor(Probe { id: 0 }.boxed());
    let stop_handle = runner.stop_handle();
    runner.run_all();
    std::thread::sleep(Duration::from_secs_f64(seconds));
    stop_handle.stop();
    runner.join_all();
}
