//! Temporary probe: drives the simulated vehicle around and logs how far
//! dead reckoning's world-frame estimate is from the truth.
use aurorus::actuators::{SimulatedVehicle, SimulatedVehicleConfig, default_model};
use aurorus::localization::{DeadReckoning, DeadReckoningConfig};
use aurorus::sensors::{SimulatedImu, SimulatedImuConfig};
use aurorus::topics::*;
use aurorus::{Captain, Executor, Runner};
use std::any::Any;
use std::time::{Duration, Instant};

struct Probe {
    id: u16,
}
impl Executor for Probe {
    fn init(&mut self, id: u16) {
        self.id = id;
    }
    fn claim_writing_topics(&mut self, c: &Captain) {
        c.claim_writer::<VescCommand>(HUMAN_VESC_COMMAND_TOPIC_NAME, self.id, VescCommand::default);
    }
    fn run(&mut self, c: &Captain) {
        let cmd = c.topic::<VescCommand>(HUMAN_VESC_COMMAND_TOPIC_NAME);
        let status = c.topic::<VehicleStatus>(VEHICLE_STATUS_TOPIC_NAME);
        let odom = c.topic::<Odometry>(ODOMETRY_TOPIC_NAME);
        let t0 = Instant::now();
        let mut last_print = t0;
        let mut max_err: f64 = 0.0;
        while c.is_running(self.id) && t0.elapsed() < Duration::from_secs(9) {
            let t = t0.elapsed().as_secs_f64();
            // Speed ramps like a real lap, steering constant-ish: a circle.
            let steer: f64 = std::env::var("STEER")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.25);
            let speed: f64 = std::env::var("SPEED")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(2.0);
            cmd.write(self.id, VescCommand::new(steer, speed)).unwrap();
            let s = status.read().value;
            let mut o = odom.read().value;
            let a = c.topic::<StartState>(START_STATE_TOPIC_NAME).read().value;
            let (sn, cs) = a.heading_rad.sin_cos();
            (o.x_m, o.y_m) = (
                a.x_m + o.x_m * cs - o.y_m * sn,
                a.y_m + o.x_m * sn + o.y_m * cs,
            );
            o.heading_rad += a.heading_rad;
            let err = (o.x_m - s.x_m).hypot(o.y_m - s.y_m);
            max_err = max_err.max(err);
            if last_print.elapsed() > Duration::from_millis(500) {
                last_print = Instant::now();
                println!(
                    "t={t:5.2} truth=({:6.2},{:6.2},{:6.1}°) odom=({:6.2},{:6.2},{:6.1}°) err={:.3} m  dist_from_start={:.2}",
                    s.x_m,
                    s.y_m,
                    s.heading_rad.to_degrees(),
                    o.x_m,
                    o.y_m,
                    o.heading_rad.to_degrees(),
                    err,
                    (s.x_m - a.x_m).hypot(s.y_m - a.y_m)
                );
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        println!("max err {max_err:.3} m");
        std::process::exit(0);
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

fn main() {
    let vc = SimulatedVehicleConfig::default();
    let mut runner = Runner::new();
    runner.register_topic::<VescCommand>(AUTONOMOUS_VESC_COMMAND_TOPIC_NAME, VescCommand::default);
    runner.register_topic::<VehicleModelSelection>(VEHICLE_MODEL_SELECTION_TOPIC_NAME, || {
        VehicleModelSelection {
            kind: match std::env::var("MODEL").as_deref() {
                Ok("dyn") => VehicleModelKind::DynamicBicycle,
                Ok("pac") => VehicleModelKind::PacejkaBicycle,
                Ok("nl") => VehicleModelKind::NonlinearBicycle,
                _ => VehicleModelKind::Bicycle,
            },
        }
    });
    runner.register_topic::<StartState>(START_STATE_TOPIC_NAME, || StartState {
        x_m: 5.0,
        y_m: -3.0,
        heading_rad: 2.0,
        speed_mps: 0.0,
    });
    runner.register_topic::<PlaceAtStart>(PLACE_AT_START_TOPIC_NAME, PlaceAtStart::default);
    runner.add_executor(Probe { id: 0 }.boxed());
    runner.add_executor(SimulatedImu::new("Imu", SimulatedImuConfig::default()).boxed());
    runner.add_executor(
        DeadReckoning::new(
            "DR",
            DeadReckoningConfig {
                draw: false,
                ..DeadReckoningConfig::default()
            },
        )
        .boxed(),
    );
    runner.add_executor(
        SimulatedVehicle::new(
            "Veh",
            default_model(
                match std::env::var("MODEL").as_deref() {
                    Ok("dyn") => VehicleModelKind::DynamicBicycle,
                    Ok("pac") => VehicleModelKind::PacejkaBicycle,
                    Ok("nl") => VehicleModelKind::NonlinearBicycle,
                    _ => VehicleModelKind::Bicycle,
                },
                &vc,
            ),
            vc,
        )
        .boxed(),
    );
    runner.run_all();
    runner.join_all();
}
