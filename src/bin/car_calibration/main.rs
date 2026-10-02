//! `car_calibration`: a guided calibration of a car's hardware, served as a
//! web page to use from a phone or laptop next to the car - its size and
//! weight, how its IMU and lidar are mounted, its steering's servo range and
//! its motor's ERPM per meter/second, minimum speed and speed compensation.
//! Saving writes `config/calibration/<car>.toml` (moving the one it replaces
//! into its history), which every other binary then drives with - see
//! `aurorus::calibration` and `documentation/car_calibration.md`.
//!
//! The motor only turns while a hold-to-run button is held (see [`bench`]).
//! It holds the VESC's port exclusively, so it can't run alongside
//! `web_gui` on the car.

mod analysis;
mod assets;
mod bench;
mod cli;
mod session;

use aurorus::actuators::VescConfig;
use aurorus::sensors::{HokuyoLidar, HokuyoLidarConfig, LidarMounting};
use aurorus::topics::{LIDAR_SCAN_TOPIC_NAME, LidarScan};
use aurorus::web::{bad_request, json_response, not_found, read_json, respond_and_close};
use aurorus::{Executor, Runner};
use bench::Bench;
use session::{GeometryBody, HoldKind, Session};
use tiny_http::{Method, Request, ResponseBox};

fn main() {
    let config = cli::parse_config(std::env::args());
    let load = |file: &str| config.config_dir.join(file);
    let vesc_config: VescConfig =
        aurorus::config::load(&load("actuators/vesc.toml")).unwrap_or_else(|err| exit(err));
    let lidar_config: HokuyoLidarConfig =
        aurorus::config::load(&load("sensors/hokuyo_lidar.toml")).unwrap_or_else(|err| exit(err));

    // The lidar's readings as the sensor sends them - unmounted, never
    // reversed - since which way round they come is one of the things
    // calibrated.
    let mut runner = Runner::new();
    runner.add_executor(
        HokuyoLidar::new("HokuyoLidar", lidar_config, LidarMounting::default()).boxed(),
    );
    runner.run_all();
    let lidar = runner.topic::<LidarScan>(LIDAR_SCAN_TOPIC_NAME);

    let bench = Bench::start(vesc_config, lidar.clone());
    let mut session = Session::new(config.config_dir.clone(), bench.clone(), lidar);
    let car = match config.car {
        Some(name) => Some(name),
        None => {
            aurorus::calibration::read_car_name(&config.config_dir).unwrap_or_else(|err| exit(err))
        }
    };
    if let Some(name) = car
        && let Err(err) = session.choose_car(&name)
    {
        exit(format!("car {name:?}: {err}"));
    }

    let server = aurorus::web::bind_http(config.bind_addr.as_str())
        .unwrap_or_else(|err| exit(format!("failed to bind {}: {err}", config.bind_addr)));
    println!(
        "car_calibration: open http://{} next to the car",
        config.bind_addr
    );

    // One request at a time: every handler is quick, and the motor's
    // hold-to-run requests arrive every 100 ms.
    for request in server.incoming_requests() {
        handle(request, &mut session);
    }
    // The server never stops, but if it did: brake.
    bench.stop();
}

fn exit(message: String) -> ! {
    eprintln!("car_calibration: {message}");
    std::process::exit(1);
}

/// Routes one request and sends its response. Every `POST` answers with the
/// whole state (see [`Session::state`]), or a `400` saying what's wrong.
fn handle(mut request: Request, session: &mut Session) {
    let url = request.url().to_string();
    let path = url.split('?').next().unwrap_or("/").to_string();
    let response = match *request.method() {
        Method::Get => aurorus::web::shared_asset(&path)
            .or_else(|| assets::respond(&path))
            .unwrap_or_else(|| match path.as_str() {
                "/api/state" => json_response(&session.state(), 200),
                _ => not_found(),
            }),
        Method::Post => match post(&path, &mut request, session) {
            Ok(Ok(())) => json_response(&session.state(), 200),
            Ok(Err(err)) => bad_request(&err),
            Err(response) => response,
        },
        _ => not_found(),
    };
    if let Err(err) = respond_and_close(request, response) {
        eprintln!("car_calibration: failed to send response: {err}");
    }
}

#[derive(serde::Deserialize)]
struct Name {
    name: String,
}
#[derive(serde::Deserialize)]
struct Cells {
    cells: u32,
}
#[derive(serde::Deserialize)]
struct Which {
    which: String,
}
#[derive(serde::Deserialize)]
struct Position {
    position: f64,
}
#[derive(serde::Deserialize)]
struct Angles {
    left_deg: f64,
    right_deg: f64,
}
#[derive(serde::Deserialize)]
struct Hold {
    #[serde(flatten)]
    request: HoldKind,
    /// A fresh press of the button - see [`Bench::hold`].
    start: bool,
}
#[derive(serde::Deserialize)]
struct Forward {
    forward: bool,
}
#[derive(serde::Deserialize)]
struct Turns {
    wheel_turns: f64,
    wheel_diameter_m: f64,
}
#[derive(serde::Deserialize)]
struct Floor {
    speed_mps: f64,
    stop_m: f64,
}
#[derive(serde::Deserialize)]
struct Index {
    index: usize,
}
#[derive(serde::Deserialize)]
struct Save {
    write_car_name: bool,
}

/// Runs the `POST` at `path`: `Err` a response for a malformed request,
/// else whether the step went through.
fn post(
    path: &str,
    request: &mut Request,
    session: &mut Session,
) -> Result<Result<(), String>, ResponseBox> {
    Ok(match path {
        "/api/car" => session.choose_car(&read_json::<Name>(request)?.name),
        "/api/battery" => session.set_battery(read_json::<Cells>(request)?.cells),
        "/api/geometry" => session.set_geometry(&read_json::<GeometryBody>(request)?),
        "/api/imu/capture" => session.capture_imu(&read_json::<Which>(request)?.which),
        "/api/lidar/capture" => session.capture_lidar(&read_json::<Which>(request)?.which),
        "/api/servo" => session.set_servo(read_json::<Position>(request)?.position),
        "/api/steering/straighten" => session.straighten(),
        "/api/steering/mark" => session.mark_steering(&read_json::<Which>(request)?.which),
        "/api/steering/table" => {
            let angles = read_json::<Angles>(request)?;
            session.set_steering(angles.left_deg, angles.right_deg)
        }
        "/api/motor/hold" => {
            let hold = read_json::<Hold>(request)?;
            session.hold_motor(hold.request, hold.start)
        }
        "/api/motor/stop" => {
            session.stop_motor();
            Ok(())
        }
        "/api/motor/direction" => session.set_direction(read_json::<Forward>(request)?.forward),
        "/api/motor/counter_zero" => session.zero_counter(),
        "/api/motor/gain" => {
            let turns = read_json::<Turns>(request)?;
            session.set_gain(turns.wheel_turns, turns.wheel_diameter_m)
        }
        "/api/motor/apply_ramp" => session.apply_ramp(),
        "/api/floor/settings" => {
            let floor = read_json::<Floor>(request)?;
            session.set_floor(floor.speed_mps, floor.stop_m)
        }
        "/api/floor/straight" => session.analyze_straight(),
        "/api/floor/apply_straight" => session.apply_straight(),
        "/api/floor/arc" => session.analyze_arc(read_json::<Index>(request)?.index),
        "/api/floor/apply_arcs" => session.apply_arcs(),
        "/api/save" => session.save(read_json::<Save>(request)?.write_car_name),
        _ => return Err(not_found()),
    })
}
