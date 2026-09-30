//! One calibration: the car being calibrated, the draft of its new
//! calibration, what each step has measured so far - and the web API that
//! drives it. Nothing is written until [`Session::save`].

use crate::analysis::{self, RampResult, ScanHalf};
use crate::bench::{Bench, ImuCapture, MAX_ERPM, MotorRequest, RAMP_ERPM};
use aurorus::RwLockTopic;
use aurorus::hardware::{self, CarCalibration, ImuMounting};
use aurorus::topics::LidarScan;
use serde_json::{Value, json};
use std::collections::BTreeSet;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

/// How long the car must have been still for an IMU capture, and what it
/// averages over.
const IMU_CAPTURE: Duration = Duration::from_millis(1500);
/// A lidar scan older than this counts as the lidar being away.
const LIDAR_STALE: Duration = Duration::from_millis(500);

/// One calibration - see the module docs.
pub struct Session {
    config_root: PathBuf,
    bench: Bench,
    lidar: Arc<RwLockTopic<LidarScan>>,
    /// The car's file as it was when chosen - `None` for a new car.
    original: Option<CarCalibration>,
    /// The new calibration - `None` until a car is chosen.
    draft: Option<CarCalibration>,
    /// The steps done this session - see [`Step`].
    done: BTreeSet<Step>,
    imu_flat: Option<ImuCapture>,
    imu_nose_up: Option<ImuCapture>,
    /// Whether lifting the left side agreed with the mounting found.
    imu_left_up_agrees: Option<bool>,
    lidar_baseline: Option<Vec<f32>>,
    /// The servo positions marked as full left, straight and full right.
    steering_marks: [Option<f64>; 3],
    /// Whether positive ERPM turned the wheels forward.
    motor_forward: Option<bool>,
    ramp_result: Option<RampResult>,
    /// Where the last save went.
    saved_to: Option<PathBuf>,
}

/// A calibration step, as the page lists them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Step {
    Battery,
    Geometry,
    Imu,
    Lidar,
    Steering,
    Direction,
    Gain,
    Ramp,
}

/// The steering marks' order in [`Session::steering_marks`].
const MARKS: [&str; 3] = ["left", "straight", "right"];

impl Session {
    pub fn new(config_root: PathBuf, bench: Bench, lidar: Arc<RwLockTopic<LidarScan>>) -> Self {
        Self {
            config_root,
            bench,
            lidar,
            original: None,
            draft: None,
            done: BTreeSet::new(),
            imu_flat: None,
            imu_nose_up: None,
            imu_left_up_agrees: None,
            lidar_baseline: None,
            steering_marks: [None; 3],
            motor_forward: None,
            ramp_result: None,
            saved_to: None,
        }
    }

    /// Starts calibrating the car `name`: from its file if it has one, else
    /// from the template. Forgets every step done so far.
    pub fn choose_car(&mut self, name: &str) -> Result<(), String> {
        hardware::valid_name(name)?;
        let original = if hardware::car_path(&self.config_root, name).exists() {
            Some(hardware::load_car(&self.config_root, name)?)
        } else {
            None
        };
        let draft = original
            .clone()
            .unwrap_or_else(|| CarCalibration::template(name));
        *self = Self::new(
            self.config_root.clone(),
            self.bench.clone(),
            self.lidar.clone(),
        );
        self.original = original;
        self.draft = Some(draft);
        Ok(())
    }

    fn draft(&mut self) -> Result<&mut CarCalibration, String> {
        self.draft
            .as_mut()
            .ok_or_else(|| "choose a car first".to_string())
    }

    /// Changes the draft with `change`, keeping it only if it's still valid.
    fn update(
        &mut self,
        step: Step,
        change: impl FnOnce(&mut CarCalibration) -> Result<(), String>,
    ) -> Result<(), String> {
        let mut next = self.draft()?.clone();
        change(&mut next)?;
        next.validate()?;
        *self.draft()? = next;
        self.done.insert(step);
        Ok(())
    }

    pub fn set_battery(&mut self, cells: u32) -> Result<(), String> {
        if !(1..=12).contains(&cells) {
            return Err("a battery has 1 to 12 cells".to_string());
        }
        self.update(Step::Battery, |car| {
            car.battery.cells = cells;
            Ok(())
        })
    }

    pub fn set_geometry(&mut self, body: &GeometryBody) -> Result<(), String> {
        let (rear_axle_to_cg_m, mass_kg) = match (body.front_axle_kg, body.rear_axle_kg) {
            (Some(front), Some(rear)) if front > 0.0 && rear > 0.0 => {
                // The CG splits the weight inversely to its distances from
                // the axles: the front axle carries lr / L of it.
                (
                    round_to(body.wheelbase_m * front / (front + rear), 0.001),
                    round_to(front + rear, 0.01),
                )
            }
            (None, None) => (
                body.rear_axle_to_cg_m
                    .ok_or("give the CG's distance from the rear axle, or both axle loads")?,
                body.mass_kg.ok_or("give the mass, or both axle loads")?,
            ),
            _ => return Err("weigh both axles, or neither".to_string()),
        };
        self.update(Step::Geometry, |car| {
            car.geometry.wheelbase_m = body.wheelbase_m;
            car.geometry.rear_axle_to_cg_m = rear_axle_to_cg_m;
            car.geometry.mass_kg = mass_kg;
            car.geometry.track_width_m = body.track_width_m;
            car.geometry.body_length_m = body.body_length_m;
            car.geometry.body_width_m = body.body_width_m;
            car.lidar.x_from_rear_axle_m = body.lidar_x_from_rear_axle_m;
            car.lidar.y_m = body.lidar_y_m;
            Ok(())
        })
    }

    /// Captures the IMU in `pose`: `flat`, then `nose_up` (which finds the
    /// mounting), then `left_up` (which checks it).
    pub fn capture_imu(&mut self, pose: &str) -> Result<(), String> {
        let capture = self
            .bench
            .state()
            .imu_over(IMU_CAPTURE)
            .ok_or("no IMU readings - is the VESC connected?")?;
        if !capture.still() {
            return Err("the car moved - hold it still for a couple of seconds".to_string());
        }
        match pose {
            "flat" => {
                analysis::down_axis(capture.accel_g)?;
                self.imu_flat = Some(capture);
                self.imu_nose_up = None;
                self.imu_left_up_agrees = None;
            }
            "nose_up" => {
                let flat = self.imu_flat.ok_or("capture the car flat first")?.accel_g;
                let z = analysis::down_axis(flat)?;
                let x = analysis::forward_axis(flat, capture.accel_g, z)?;
                let y = analysis::right_axis(x, z);
                self.update(Step::Imu, |car| {
                    car.imu = ImuMounting { x, y, z };
                    Ok(())
                })?;
                self.imu_nose_up = Some(capture);
                self.imu_left_up_agrees = None;
            }
            "left_up" => {
                let flat = self.imu_flat.ok_or("capture the car flat first")?.accel_g;
                if self.imu_nose_up.is_none() {
                    return Err("capture the nose lifted first".to_string());
                }
                let mounting = self.draft()?.imu;
                let agrees = analysis::left_up_agrees(&mounting, flat, capture.accel_g);
                self.imu_left_up_agrees = Some(agrees);
                if !agrees {
                    return Err(
                        "lifting the left side didn't read as expected - lift the LEFT side \
                         (the driver's left, facing forward), or redo the captures"
                            .to_string(),
                    );
                }
            }
            _ => return Err(format!("unknown pose {pose:?}")),
        }
        Ok(())
    }

    /// The latest lidar scan's readings, if fresh.
    fn lidar_readings(&self) -> Option<Vec<f32>> {
        let scan = self.lidar.read();
        let fresh = scan.age().is_some_and(|age| age < LIDAR_STALE);
        (fresh && scan.num_lidar_points > 0).then(|| scan.value.points.clone())
    }

    /// Captures the lidar's scan: the `baseline`, then with an `object` on
    /// the car's left - which finds whether it's upside down.
    pub fn capture_lidar(&mut self, which: &str) -> Result<(), String> {
        let readings = self.lidar_readings().ok_or("no scan from the lidar")?;
        match which {
            "baseline" => self.lidar_baseline = Some(readings),
            "object" => {
                let baseline = self
                    .lidar_baseline
                    .as_ref()
                    .ok_or("capture the scan without the object first")?;
                let half = analysis::object_half(baseline, &readings)?;
                // A scan's first reading is the car's left: seen last, the
                // sensor sends them the other way round.
                self.update(Step::Lidar, |car| {
                    car.lidar.upside_down = half == ScanHalf::Last;
                    Ok(())
                })?;
            }
            _ => return Err(format!("unknown capture {which:?}")),
        }
        Ok(())
    }

    pub fn set_servo(&mut self, position: f64) -> Result<(), String> {
        if !position.is_finite() {
            return Err("the servo position must be a number".to_string());
        }
        self.bench.set_servo(position);
        Ok(())
    }

    /// Marks the servo's current position as full `left`, `straight` or
    /// full `right`.
    pub fn mark_steering(&mut self, which: &str) -> Result<(), String> {
        let index = MARKS
            .iter()
            .position(|&mark| mark == which)
            .ok_or_else(|| format!("unknown mark {which:?}"))?;
        let servo = self.bench.state().servo.ok_or("move the servo first")?;
        self.steering_marks[index] = Some(servo);
        Ok(())
    }

    /// Builds the steering table from the marks and the angles measured at
    /// full lock, in degrees.
    pub fn set_steering(&mut self, left_deg: f64, right_deg: f64) -> Result<(), String> {
        let [Some(left), Some(straight), Some(right)] = self.steering_marks else {
            return Err("mark full left, straight and full right first".to_string());
        };
        let table = analysis::bench_steering(
            left,
            straight,
            right,
            left_deg.to_radians(),
            right_deg.to_radians(),
        )?;
        self.update(Step::Steering, |car| {
            car.steering = table;
            Ok(())
        })
    }

    pub fn hold_motor(&mut self, request: MotorRequest, start: bool) -> Result<(), String> {
        self.draft()?;
        self.bench.hold(request, start)
    }

    pub fn stop_motor(&mut self) {
        self.bench.stop();
    }

    pub fn set_direction(&mut self, forward: bool) -> Result<(), String> {
        self.draft()?;
        self.motor_forward = Some(forward);
        if forward {
            self.done.insert(Step::Direction);
            Ok(())
        } else {
            self.done.remove(&Step::Direction);
            Err(
                "positive ERPM must drive forward: invert the motor's direction in VESC Tool \
                 (Motor Settings > General > Invert Motor Direction), then try again"
                    .to_string(),
            )
        }
    }

    pub fn zero_counter(&mut self) -> Result<(), String> {
        self.bench.zero_counter()
    }

    /// Sets the speed gain from the wheel turns counted since the counter was
    /// zeroed.
    pub fn set_gain(&mut self, wheel_turns: f64, wheel_diameter_m: f64) -> Result<(), String> {
        let steps = self
            .bench
            .state()
            .counted_steps()
            .ok_or("zero the counter first")?;
        let gain = round_to(
            analysis::speed_to_erpm_gain(steps, wheel_turns, wheel_diameter_m)?,
            1.0,
        );
        self.update(Step::Gain, |car| {
            car.motor.speed_to_erpm_gain = gain;
            Ok(())
        })
    }

    /// Sets the speed compensation and minimum speed from the finished ramp.
    pub fn apply_ramp(&mut self) -> Result<(), String> {
        let ramp = self
            .bench
            .state()
            .ramp
            .clone()
            .filter(|ramp| ramp.finished)
            .ok_or("run the whole ramp first")?;
        let result = analysis::analyze_ramp(&ramp.steps)?;
        self.update(Step::Ramp, |car| {
            car.motor.speed_compensation = round_to(result.speed_compensation, 0.001);
            car.motor.min_speed_mps =
                analysis::min_speed_mps(&result, car.motor.speed_to_erpm_gain);
            Ok(())
        })?;
        self.ramp_result = Some(result);
        Ok(())
    }

    /// Writes the draft as the car's file (the old one moving to its
    /// history), and - if `write_car_name` - names it in `CAR_NAME`.
    pub fn save(&mut self, write_car_name: bool) -> Result<(), String> {
        if self.motor_forward == Some(false) {
            return Err("the motor turns backwards - fix it in VESC Tool first".to_string());
        }
        let root = self.config_root.clone();
        let draft = self.draft()?;
        draft.calibrated_at = hardware::now_local_string();
        let car = draft.clone();
        let path = hardware::save_car(&root, &car)?;
        if write_car_name {
            let car_name = hardware::car_name_path(&root);
            std::fs::write(&car_name, format!("{}\n", car.name)).map_err(|err| {
                format!("saved {path:?}, but failed to write {car_name:?}: {err}")
            })?;
        }
        self.original = Some(car);
        self.saved_to = Some(path);
        Ok(())
    }

    /// Everything the page shows.
    pub fn state(&self) -> Value {
        let bench = self.bench.state();
        let values = bench.values.as_ref();
        let voltage = values.map(|values| values.input_voltage_v);
        let imu = bench.imu_over(Duration::from_millis(500));
        let lidar = self.lidar.read();
        let lidar_fresh = lidar.age().is_some_and(|age| age < LIDAR_STALE);
        let draft = self
            .draft
            .as_ref()
            .map(|car| serde_json::to_value(car).unwrap());
        let original = self
            .original
            .as_ref()
            .map(|car| serde_json::to_value(car).unwrap());
        let mut changes = Vec::new();
        if let Some(draft) = &draft {
            diff("", original.as_ref(), draft, &mut changes);
        }
        json!({
            "car": {
                "name": self.draft.as_ref().map(|car| car.name.clone()),
                "is_new": self.draft.is_some() && self.original.is_none(),
                "draft": draft,
                "changes": changes,
                "saved_to": self.saved_to,
            },
            "car_name_file": hardware::read_car_name(&self.config_root).ok().flatten(),
            "known_cars": hardware::car_names(&self.config_root),
            "done": self.done,
            "vesc": {
                "connected": bench.connected,
                "error": bench.error,
                "firmware": bench.firmware.as_ref().map(|firmware| json!({
                    "version": format!("{}.{:02}", firmware.major, firmware.minor),
                    "hardware": firmware.hardware,
                })),
                "voltage_v": voltage,
                "possible_cells": voltage.map(analysis::possible_cells),
                "erpm": values.map(|values| values.erpm),
                "tachometer": values.map(|values| values.tachometer),
                "fault": values.map(|values| values.fault.name()),
                "servo": bench.servo,
                "motor_running": bench.motor_running(),
                "counted_steps": bench.counted_steps(),
                "imu": imu,
                "still": imu.map(|imu| imu.still()),
            },
            "lidar": {
                "connected": lidar_fresh,
                "readings": lidar.num_lidar_points,
            },
            "imu_steps": {
                "flat": self.imu_flat.is_some(),
                "nose_up": self.imu_nose_up.is_some(),
                "left_up_agrees": self.imu_left_up_agrees,
            },
            "lidar_baseline": self.lidar_baseline.is_some(),
            "steering_marks": {
                "left": self.steering_marks[0],
                "straight": self.steering_marks[1],
                "right": self.steering_marks[2],
            },
            "motor_forward": self.motor_forward,
            "ramp": bench.ramp.clone().unwrap_or_default(),
            "ramp_result": self.ramp_result,
            "limits": { "max_erpm": MAX_ERPM, "ramp_erpm": RAMP_ERPM },
        })
    }
}

/// `value` rounded to a multiple of `step` - what a computed value is
/// written with, rather than every digit of a float.
fn round_to(value: f64, step: f64) -> f64 {
    ((value / step).round() * step * 1e9).round() / 1e9
}

/// The body of `POST /api/geometry`.
#[derive(serde::Deserialize)]
pub struct GeometryBody {
    pub wheelbase_m: f64,
    /// Either both axle loads, or the CG's distance and the mass directly.
    pub front_axle_kg: Option<f64>,
    pub rear_axle_kg: Option<f64>,
    pub rear_axle_to_cg_m: Option<f64>,
    pub mass_kg: Option<f64>,
    pub track_width_m: f64,
    pub body_length_m: f64,
    pub body_width_m: f64,
    pub lidar_x_from_rear_axle_m: f64,
    pub lidar_y_m: f64,
}

/// Every value of `new` (a JSON object, nested tables and all) that differs
/// from `old`'s, as `{field, old, new}` into `changes` - `field` its dotted
/// path, arrays compared whole.
fn diff(path: &str, old: Option<&Value>, new: &Value, changes: &mut Vec<Value>) {
    match new {
        Value::Object(fields) => {
            for (key, value) in fields {
                let field = if path.is_empty() {
                    key.clone()
                } else {
                    format!("{path}.{key}")
                };
                diff(&field, old.and_then(|old| old.get(key)), value, changes);
            }
        }
        _ if old == Some(new) => {}
        _ => changes.push(json!({ "field": path, "old": old, "new": new })),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn computed_values_are_rounded_to_their_step() {
        assert_eq!(round_to(0.146_666_7, 0.001), 0.147);
        assert_eq!(round_to(4853.72, 1.0), 4854.0);
        assert_eq!(round_to(1.162_79, 0.001), 1.163);
    }

    #[test]
    fn changed_values_are_listed_by_their_dotted_path() {
        let old = json!({ "a": 1, "t": { "b": 2.0, "c": [1, 2] }, "same": "x" });
        let new = json!({ "a": 1, "t": { "b": 2.5, "c": [1, 3] }, "same": "x" });
        let mut changes = Vec::new();
        diff("", Some(&old), &new, &mut changes);
        let fields: Vec<&str> = changes
            .iter()
            .map(|c| c["field"].as_str().unwrap())
            .collect();
        assert_eq!(fields, vec!["t.b", "t.c"]);
        // A new car: everything is new.
        let mut changes = Vec::new();
        diff("", None, &new, &mut changes);
        assert_eq!(changes.len(), 4);
    }
}
