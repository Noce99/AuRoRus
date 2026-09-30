//! A car's hardware calibration - every physical fact about one car the
//! stack needs: its geometry, how its servo steers, how its motor's ERPM
//! becomes speed, how its IMU and lidar are mounted. Measured by the
//! `car_calibration` binary (see `documentation/car_calibration.md`).
//!
//! Each calibrated car has one tracked file, `config/hardware/<name>.toml`:
//! its latest accepted calibration. Saving a new one (see [`save_car`])
//! first moves the old file into the gitignored
//! `config/hardware/history/<name>/`, so a bad calibration can be reverted.
//! Which car a machine drives is named by the gitignored [`CAR_NAME_FILE`]
//! in the repository root (see [`read_car_name`]): with none, binaries run
//! in simulation.

use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};
use time::OffsetDateTime;

/// The file, in the repository root (the parent of the config root), naming
/// the car this machine drives - gitignored, one per machine.
pub const CAR_NAME_FILE: &str = "CAR_NAME";

/// The version of [`CarCalibration`]'s layout; a file written with another
/// one is refused rather than misread.
pub const SCHEMA_VERSION: u32 = 1;

/// Folder, under the config root, of every car's file.
const HARDWARE_DIR: &str = "hardware";
/// Folder, under [`HARDWARE_DIR`], of every car's older calibrations.
const HISTORY_DIR: &str = "history";

/// One car's calibration - see the module docs. Positive y and angles are
/// the car's right everywhere, as in the simulator (whose frame has y down,
/// as the GUI draws it).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CarCalibration {
    /// Must be [`SCHEMA_VERSION`].
    pub schema_version: u32,
    /// The car's name - also its file's stem (see [`valid_name`]).
    pub name: String,
    /// When it was calibrated, local time, `YYYY-MM-DD HH:MM:SS`.
    pub calibrated_at: String,
    pub geometry: Geometry,
    pub steering: SteeringTable,
    pub motor: MotorCalibration,
    pub battery: Battery,
    pub imu: ImuMounting,
    pub lidar: LidarMounting,
}

/// The car's size and mass.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Geometry {
    /// Between the front and rear axles, in meters.
    pub wheelbase_m: f64,
    /// From the rear axle forward to the center of gravity, in meters - the
    /// point every pose and `vehicle_status` refers to.
    pub rear_axle_to_cg_m: f64,
    /// Between the left and right wheels' centers, in meters.
    pub track_width_m: f64,
    /// The body's overall length and width, bumpers and wheels included, in
    /// meters.
    pub body_length_m: f64,
    pub body_width_m: f64,
    /// The whole car, battery included, in kilograms.
    pub mass_kg: f64,
}

impl Geometry {
    /// From the center of gravity forward to the front axle, in meters.
    pub fn lf_m(&self) -> f64 {
        self.wheelbase_m - self.rear_axle_to_cg_m
    }

    /// From the center of gravity back to the rear axle, in meters.
    pub fn lr_m(&self) -> f64 {
        self.rear_axle_to_cg_m
    }

    fn validate(&self) -> Result<(), String> {
        for (name, value) in [
            ("wheelbase_m", self.wheelbase_m),
            ("track_width_m", self.track_width_m),
            ("body_length_m", self.body_length_m),
            ("body_width_m", self.body_width_m),
            ("mass_kg", self.mass_kg),
        ] {
            if !(value.is_finite() && value > 0.0) {
                return Err(format!("geometry.{name} must be positive"));
            }
        }
        if !(0.0..=self.wheelbase_m).contains(&self.rear_axle_to_cg_m) {
            return Err("geometry.rear_axle_to_cg_m must be within the wheelbase".to_string());
        }
        Ok(())
    }
}

/// One measured point of the steering: at this servo position, the wheels
/// steer this much.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SteeringPoint {
    /// The servo position, in `0..=1`.
    pub servo: f64,
    /// The effective (bicycle-model) steering angle, in radians, positive
    /// right.
    pub angle_rad: f64,
}

/// How the servo steers: a lookup table of measured points, linearly
/// interpolated between them. Its first and last points are the servo
/// positions never exceeded - just short of the steering's end stops.
///
/// Valid when the servo positions strictly increase, the angles strictly
/// increase or strictly decrease along them (a servo can steer right either
/// way), and the angles span straight ahead - see [`Self::validate`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SteeringTable {
    pub points: Vec<SteeringPoint>,
}

impl SteeringTable {
    pub fn validate(&self) -> Result<(), String> {
        let points = &self.points;
        if points.len() < 2 {
            return Err("steering.points needs at least two points".to_string());
        }
        if points
            .iter()
            .any(|p| !p.angle_rad.is_finite() || !(0.0..=1.0).contains(&p.servo))
        {
            return Err("steering.points' servo must be in 0..=1 and angles finite".to_string());
        }
        if points.windows(2).any(|w| w[1].servo <= w[0].servo) {
            return Err("steering.points' servo positions must strictly increase".to_string());
        }
        let increasing = points[1].angle_rad > points[0].angle_rad;
        if points.windows(2).any(|w| {
            (w[1].angle_rad > w[0].angle_rad) != increasing || w[1].angle_rad == w[0].angle_rad
        }) {
            return Err(
                "steering.points' angles must strictly increase or strictly decrease".to_string(),
            );
        }
        let (low, high) = self.angle_range();
        if !(low < 0.0 && high > 0.0) {
            return Err("steering.points must steer both left and right".to_string());
        }
        Ok(())
    }

    /// The servo position steering `angle_rad`, clamped to the table's ends.
    pub fn servo_for(&self, angle_rad: f64) -> f64 {
        let (servos, angles): (Vec<f64>, Vec<f64>) =
            self.points.iter().map(|p| (p.servo, p.angle_rad)).unzip();
        if self.angle_increases() {
            interpolate(&angles, &servos, angle_rad)
        } else {
            let reversed = |v: Vec<f64>| v.into_iter().rev().collect::<Vec<_>>();
            interpolate(&reversed(angles), &reversed(servos), angle_rad)
        }
    }

    /// The angle `servo` steers, clamped to the table's ends.
    pub fn angle_for(&self, servo: f64) -> f64 {
        let (servos, angles): (Vec<f64>, Vec<f64>) =
            self.points.iter().map(|p| (p.servo, p.angle_rad)).unzip();
        interpolate(&servos, &angles, servo)
    }

    /// The servo position steering straight ahead.
    pub fn straight_servo(&self) -> f64 {
        self.servo_for(0.0)
    }

    /// The lowest and highest servo positions ever sent.
    pub fn servo_range(&self) -> (f64, f64) {
        (
            self.points[0].servo,
            self.points[self.points.len() - 1].servo,
        )
    }

    /// The largest angle the car steers *both* ways, in radians - the
    /// smaller of its two sides', so a limit given to the algorithms holds
    /// either way.
    pub fn max_angle_rad(&self) -> f64 {
        let (low, high) = self.angle_range();
        (-low).min(high)
    }

    /// The most-left and most-right angles.
    fn angle_range(&self) -> (f64, f64) {
        let (first, last) = (
            self.points[0].angle_rad,
            self.points[self.points.len() - 1].angle_rad,
        );
        (first.min(last), first.max(last))
    }

    fn angle_increases(&self) -> bool {
        self.points[self.points.len() - 1].angle_rad > self.points[0].angle_rad
    }
}

/// `ys` at `x`, linearly interpolated between `xs` (strictly increasing),
/// clamped to their ends.
fn interpolate(xs: &[f64], ys: &[f64], x: f64) -> f64 {
    let last = xs.len() - 1;
    if x.is_nan() || x <= xs[0] {
        return ys[0];
    }
    if x >= xs[last] {
        return ys[last];
    }
    let above = xs.partition_point(|&v| v <= x);
    let (x0, x1, y0, y1) = (xs[above - 1], xs[above], ys[above - 1], ys[above]);
    y0 + (y1 - y0) * (x - x0) / (x1 - x0)
}

/// How the motor's ERPM becomes speed. Positive ERPM must drive forward -
/// if the motor turns the wrong way, invert it in VESC Tool.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotorCalibration {
    /// Motor ERPM per meter/second of speed: `erpm = speed *
    /// speed_to_erpm_gain`, also how measured ERPM becomes wheel speed.
    pub speed_to_erpm_gain: f64,
    /// Multiplies every commanded ERPM, making up for the VESC's speed
    /// controller settling short of it. `1.0` leaves it as is.
    pub speed_compensation: f64,
    /// The slowest speed the motor holds smoothly, in meters/second: slower
    /// nonzero speeds are raised to it.
    pub min_speed_mps: f64,
}

impl MotorCalibration {
    fn validate(&self) -> Result<(), String> {
        if !(self.speed_to_erpm_gain.is_finite() && self.speed_to_erpm_gain > 0.0) {
            return Err("motor.speed_to_erpm_gain must be positive".to_string());
        }
        if !(self.speed_compensation.is_finite() && self.speed_compensation > 0.0) {
            return Err("motor.speed_compensation must be positive".to_string());
        }
        if !(self.min_speed_mps.is_finite() && self.min_speed_mps >= 0.0) {
            return Err("motor.min_speed_mps must not be negative".to_string());
        }
        Ok(())
    }
}

/// The battery.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Battery {
    /// Its LiPo cells in series.
    pub cells: u32,
}

/// Which of the VESC's IMU axes gives the car's x (forward), y (right) and
/// z - so a right turn reads as a positive yaw rate, as in the simulator.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImuMounting {
    pub x: ImuAxis,
    pub y: ImuAxis,
    pub z: ImuAxis,
}

impl ImuMounting {
    fn validate(&self) -> Result<(), String> {
        let (x, y, z) = (self.x.index(), self.y.index(), self.z.index());
        if x == y || y == z || x == z {
            return Err("imu.x, imu.y and imu.z must be three different axes".to_string());
        }
        Ok(())
    }
}

/// One of the IMU's axes, possibly reversed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ImuAxis {
    #[serde(rename = "+x")]
    PlusX,
    #[serde(rename = "-x")]
    MinusX,
    #[serde(rename = "+y")]
    PlusY,
    #[serde(rename = "-y")]
    MinusY,
    #[serde(rename = "+z")]
    PlusZ,
    #[serde(rename = "-z")]
    MinusZ,
}

impl ImuAxis {
    /// The component of `v` (in the IMU's axes) along this axis.
    pub fn of(self, v: [f64; 3]) -> f64 {
        let sign = match self {
            Self::PlusX | Self::PlusY | Self::PlusZ => 1.0,
            Self::MinusX | Self::MinusY | Self::MinusZ => -1.0,
        };
        sign * v[self.index()]
    }

    /// The IMU's axis `index` (0, 1 or 2 for x, y, z), `positive` or
    /// reversed.
    ///
    /// # Panics
    ///
    /// Panics if `index` isn't 0, 1 or 2.
    pub fn new(index: usize, positive: bool) -> Self {
        match (index, positive) {
            (0, true) => Self::PlusX,
            (0, false) => Self::MinusX,
            (1, true) => Self::PlusY,
            (1, false) => Self::MinusY,
            (2, true) => Self::PlusZ,
            (2, false) => Self::MinusZ,
            _ => panic!("an IMU has no axis {index}"),
        }
    }

    /// This axis as a unit vector in the IMU's axes.
    pub fn unit(self) -> [f64; 3] {
        let mut v = [0.0; 3];
        v[self.index()] = self.of([1.0, 1.0, 1.0]);
        v
    }

    /// Which of the IMU's axes, whichever way: 0, 1 or 2.
    pub fn index(self) -> usize {
        match self {
            Self::PlusX | Self::MinusX => 0,
            Self::PlusY | Self::MinusY => 1,
            Self::PlusZ | Self::MinusZ => 2,
        }
    }
}

/// Where and how the lidar is mounted.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LidarMounting {
    /// Whether its readings come right-most first, reversed from a scan's
    /// left-most first - see `HokuyoLidar`.
    pub upside_down: bool,
    /// Its center, forward of the rear axle, in meters - what a tape
    /// measures (see [`CarCalibration::lidar_mount_m`] for it from the CG).
    pub x_from_rear_axle_m: f64,
    /// Its center, right of the car's centerline, in meters.
    pub y_m: f64,
}

impl CarCalibration {
    /// Checks everything the car couldn't be driven with.
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != SCHEMA_VERSION {
            return Err(format!(
                "schema_version {} isn't the supported {SCHEMA_VERSION}",
                self.schema_version
            ));
        }
        valid_name(&self.name)?;
        self.geometry.validate()?;
        self.steering.validate()?;
        self.motor.validate()?;
        if self.battery.cells == 0 {
            return Err("battery.cells must be positive".to_string());
        }
        self.imu.validate()?;
        if !(self.lidar.x_from_rear_axle_m.is_finite() && self.lidar.y_m.is_finite()) {
            return Err("lidar's position must be finite".to_string());
        }
        Ok(())
    }

    /// Where the lidar sits from the center of gravity (the vehicle's
    /// reference point), in meters: forward, right - see
    /// `LidarScan::mount_x_m`.
    pub fn lidar_mount_m(&self) -> (f64, f64) {
        (
            self.lidar.x_from_rear_axle_m - self.geometry.rear_axle_to_cg_m,
            self.lidar.y_m,
        )
    }

    /// Its size, as published for the algorithms.
    pub fn vehicle_geometry(&self) -> crate::topics::VehicleGeometry {
        let g = &self.geometry;
        crate::topics::VehicleGeometry {
            wheelbase_m: g.wheelbase_m,
            rear_axle_to_cg_m: g.rear_axle_to_cg_m,
            track_width_m: g.track_width_m,
            body_length_m: g.body_length_m,
            body_width_m: g.body_width_m,
        }
    }

    /// The starting point of a new car's first calibration:
    /// `config/car_template.toml` (a roughly 1/10-scale car), named `name`
    /// and dated now.
    pub fn template(name: &str) -> Self {
        let mut car: Self = toml::from_str(include_str!("../config/car_template.toml"))
            .expect("config/car_template.toml must deserialize into CarCalibration");
        car.name = name.to_string();
        car.calibrated_at = now_local_string();
        car
    }
}

/// Checks `name` makes a car file's stem: 1-64 lowercase letters, digits,
/// `_` or `-`.
pub fn valid_name(name: &str) -> Result<(), String> {
    let ok = !name.is_empty()
        && name.len() <= 64
        && name
            .chars()
            .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_' || c == '-');
    if ok {
        Ok(())
    } else {
        Err(format!(
            "car name {name:?} must be 1-64 lowercase letters, digits, '_' or '-'"
        ))
    }
}

/// [`CAR_NAME_FILE`]'s path for the config root `config_root`: beside it.
pub fn car_name_path(config_root: &Path) -> PathBuf {
    config_root
        .parent()
        .unwrap_or(Path::new(""))
        .join(CAR_NAME_FILE)
}

/// The car this machine drives, from [`car_name_path`] - `None` without the
/// file. Fails on an unreadable or invalid name.
pub fn read_car_name(config_root: &Path) -> Result<Option<String>, String> {
    let path = car_name_path(config_root);
    let text = match fs::read_to_string(&path) {
        Ok(text) => text,
        Err(err) if err.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(err) => return Err(format!("failed to read {path:?}: {err}")),
    };
    let name = text.trim();
    valid_name(name).map_err(|err| format!("{path:?}: {err}"))?;
    Ok(Some(name.to_string()))
}

/// The tracked file of the car `name`: `<config_root>/hardware/<name>.toml`.
pub fn car_path(config_root: &Path, name: &str) -> PathBuf {
    config_root.join(HARDWARE_DIR).join(format!("{name}.toml"))
}

/// The folder of the car `name`'s older calibrations:
/// `<config_root>/hardware/history/<name>/`.
pub fn history_dir(config_root: &Path, name: &str) -> PathBuf {
    config_root.join(HARDWARE_DIR).join(HISTORY_DIR).join(name)
}

/// The names of every car with a file under `config_root`, sorted.
pub fn car_names(config_root: &Path) -> Vec<String> {
    let Ok(entries) = fs::read_dir(config_root.join(HARDWARE_DIR)) else {
        return Vec::new();
    };
    let mut names: Vec<String> = entries
        .filter_map(Result::ok)
        .filter_map(|entry| {
            let path = entry.path();
            (path.extension()? == "toml").then_some(())?;
            let name = path.file_stem()?.to_str()?.to_string();
            valid_name(&name).ok().map(|()| name)
        })
        .collect();
    names.sort();
    names
}

/// Loads and [validates](CarCalibration::validate) the car `name`'s file.
pub fn load_car(config_root: &Path, name: &str) -> Result<CarCalibration, String> {
    valid_name(name)?;
    let path = car_path(config_root, name);
    let car: CarCalibration = crate::config::load(&path)?;
    car.validate().map_err(|err| format!("{path:?}: {err}"))?;
    if car.name != name {
        return Err(format!("{path:?} names the car {:?}", car.name));
    }
    Ok(car)
}

/// Loads the car [`read_car_name`] names, if any.
pub fn load_this_car(config_root: &Path) -> Result<Option<CarCalibration>, String> {
    read_car_name(config_root)?
        .map(|name| load_car(config_root, &name))
        .transpose()
}

/// Writes `car` as its car's file, first moving the file it replaces (if
/// any) into [`history_dir`], named after when that one was calibrated.
/// Returns the new file's path.
pub fn save_car(config_root: &Path, car: &CarCalibration) -> Result<PathBuf, String> {
    car.validate()?;
    let path = car_path(config_root, &car.name);
    let io =
        |what: &str, path: &Path, err: std::io::Error| format!("failed to {what} {path:?}: {err}");
    if path.exists() {
        let history = history_dir(config_root, &car.name);
        fs::create_dir_all(&history).map_err(|err| io("create", &history, err))?;
        let stamp = crate::config::load::<CarCalibration>(&path)
            .map(|old| old.calibrated_at)
            .unwrap_or_else(|_| now_local_string());
        let archived = unused_path(&history, &file_stamp(&stamp));
        fs::rename(&path, &archived).map_err(|err| io("move", &path, err))?;
    }
    let dir = config_root.join(HARDWARE_DIR);
    fs::create_dir_all(&dir).map_err(|err| io("create", &dir, err))?;
    let body =
        toml::to_string_pretty(car).map_err(|err| format!("can't write {:?}: {err}", car.name))?;
    let text = format!(
        "# The hardware calibration of the car {:?}, written by car_calibration -\n\
         # see src/hardware.rs for what each value means. Older ones are in\n\
         # config/hardware/history/{}/ (gitignored).\n\n{body}",
        car.name, car.name
    );
    let temporary = path.with_extension("toml.tmp");
    fs::write(&temporary, text)
        .and_then(|()| fs::rename(&temporary, &path))
        .map_err(|err| io("write", &path, err))?;
    Ok(path)
}

/// `dir/<stem>.toml`, or `dir/<stem>_<n>.toml` for the first `n` not taken.
fn unused_path(dir: &Path, stem: &str) -> PathBuf {
    let first = dir.join(format!("{stem}.toml"));
    if !first.exists() {
        return first;
    }
    (2..)
        .map(|n| dir.join(format!("{stem}_{n}.toml")))
        .find(|path| !path.exists())
        .expect("some suffix is free")
}

/// `calibrated_at` as a file stem: `2026-09-30 15:18:02` ->
/// `2026-09-30_15-18-02`.
fn file_stamp(calibrated_at: &str) -> String {
    calibrated_at
        .chars()
        .map(|c| match c {
            '0'..='9' | '-' => c,
            _ => {
                if c == ' ' {
                    '_'
                } else {
                    '-'
                }
            }
        })
        .collect()
}

/// Now, local time (UTC if the offset is unknown), `YYYY-MM-DD HH:MM:SS`.
pub fn now_local_string() -> String {
    let now = OffsetDateTime::now_local().unwrap_or_else(|_| OffsetDateTime::now_utc());
    format!(
        "{:04}-{:02}-{:02} {:02}:{:02}:{:02}",
        now.year(),
        u8::from(now.month()),
        now.day(),
        now.hour(),
        now.minute(),
        now.second(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    fn table(points: &[(f64, f64)]) -> SteeringTable {
        SteeringTable {
            points: points
                .iter()
                .map(|&(servo, angle_rad)| SteeringPoint { servo, angle_rad })
                .collect(),
        }
    }

    fn temp_root(test: &str) -> PathBuf {
        let root = std::env::temp_dir()
            .join(format!("aurorus_hardware_{test}_{}", std::process::id()))
            .join("config");
        let _ = fs::remove_dir_all(root.parent().unwrap());
        fs::create_dir_all(&root).unwrap();
        root
    }

    #[test]
    fn the_template_and_every_tracked_car_are_valid() {
        CarCalibration::template("new").validate().unwrap();
        let root = Path::new(crate::config::DEFAULT_CONFIG_ROOT);
        for name in car_names(root) {
            load_car(root, &name).unwrap();
        }
    }

    #[test]
    fn the_steering_table_interpolates_both_ways_and_clamps() {
        // Asymmetric: more to the right than to the left.
        let steering = table(&[(0.2, -0.3), (0.5, 0.0), (0.8, 0.4)]);
        steering.validate().unwrap();
        assert!(close(steering.straight_servo(), 0.5));
        assert!(close(steering.servo_for(0.2), 0.65));
        assert!(close(steering.servo_for(-0.15), 0.35));
        assert!(close(steering.angle_for(0.65), 0.2));
        assert!(close(steering.servo_for(1.0), 0.8));
        assert!(close(steering.servo_for(-1.0), 0.2));
        assert!(close(steering.angle_for(0.0), -0.3));
        assert_eq!(steering.servo_range(), (0.2, 0.8));
        // The smaller side.
        assert!(close(steering.max_angle_rad(), 0.3));
    }

    #[test]
    fn a_servo_steering_left_as_it_rises_works_too() {
        let steering = table(&[(0.2, 0.4), (0.45, 0.0), (0.8, -0.35)]);
        steering.validate().unwrap();
        assert!(close(steering.straight_servo(), 0.45));
        assert!(close(steering.servo_for(0.2), 0.325));
        assert!(close(steering.angle_for(0.325), 0.2));
        assert!(close(steering.max_angle_rad(), 0.35));
    }

    #[test]
    fn a_broken_steering_table_is_refused() {
        assert!(table(&[(0.5, 0.0)]).validate().is_err());
        assert!(table(&[(0.5, -0.1), (0.4, 0.1)]).validate().is_err());
        assert!(
            table(&[(0.2, -0.3), (0.5, 0.1), (0.8, 0.0)])
                .validate()
                .is_err()
        );
        assert!(table(&[(0.2, 0.1), (0.8, 0.3)]).validate().is_err());
        assert!(table(&[(0.2, -0.3), (1.2, 0.3)]).validate().is_err());
    }

    #[test]
    fn imu_axes_must_differ() {
        let mut car = CarCalibration::template("a");
        car.imu.y = car.imu.x;
        assert!(car.validate().is_err());
    }

    #[test]
    fn names_must_make_a_plain_file_stem() {
        assert!(valid_name("tom").is_ok());
        assert!(valid_name("car_2-b").is_ok());
        for bad in ["", "Tom", "../tom", "a b", "a.toml"] {
            assert!(valid_name(bad).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn the_car_name_file_sits_beside_the_config_root() {
        let root = temp_root("car_name");
        assert_eq!(read_car_name(&root), Ok(None));
        fs::write(car_name_path(&root), "tom\n").unwrap();
        assert_eq!(read_car_name(&root), Ok(Some("tom".to_string())));
        fs::write(car_name_path(&root), "../etc").unwrap();
        assert!(read_car_name(&root).is_err());
        assert_eq!(
            car_name_path(Path::new("config")),
            Path::new("CAR_NAME").to_path_buf()
        );
        fs::remove_dir_all(root.parent().unwrap()).unwrap();
    }

    #[test]
    fn saving_archives_the_previous_calibration() {
        let root = temp_root("save");
        let mut car = CarCalibration::template("tom");
        car.calibrated_at = "2026-09-30 15:18:02".to_string();
        let path = save_car(&root, &car).unwrap();
        assert_eq!(path, car_path(&root, "tom"));
        assert_eq!(load_car(&root, "tom").unwrap(), car);
        assert!(!history_dir(&root, "tom").exists());

        let mut next = car.clone();
        next.calibrated_at = "2026-10-01 09:00:00".to_string();
        next.motor.speed_compensation = 1.2;
        save_car(&root, &next).unwrap();
        assert_eq!(load_car(&root, "tom").unwrap(), next);
        let archived = history_dir(&root, "tom").join("2026-09-30_15-18-02.toml");
        assert_eq!(
            crate::config::load::<CarCalibration>(&archived).unwrap(),
            car
        );

        // Saving the same calibration again never overwrites the archive.
        save_car(&root, &next).unwrap();
        save_car(&root, &next).unwrap();
        let history = history_dir(&root, "tom");
        assert!(history.join("2026-10-01_09-00-00.toml").exists());
        assert!(history.join("2026-10-01_09-00-00_2.toml").exists());
        assert_eq!(car_names(&root), vec!["tom".to_string()]);
        fs::remove_dir_all(root.parent().unwrap()).unwrap();
    }

    #[test]
    fn a_file_naming_another_car_is_refused() {
        let root = temp_root("mismatch");
        save_car(&root, &CarCalibration::template("tom")).unwrap();
        fs::copy(car_path(&root, "tom"), car_path(&root, "jerry")).unwrap();
        assert!(load_car(&root, "jerry").is_err());
        fs::remove_dir_all(root.parent().unwrap()).unwrap();
    }
}
