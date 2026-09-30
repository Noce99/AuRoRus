//! What `car_calibration` works out from its measurements - pure
//! functions, with no I/O, so they're tested without the car.
//!
//! The car's frame is the project's: x forward, y to the right, and so -
//! right-handed - z down, a right turn being a positive yaw rate. At rest
//! an accelerometer reads the reaction to gravity, pointing up: -1 g along
//! the car's z.

use aurorus::hardware::{CarCalibration, ImuAxis, ImuMounting, SteeringPoint, SteeringTable};
use aurorus::topics::LidarScan;

/// The VESC's tachometer steps per electrical turn of the motor (verified on
/// the first car, a VESC 6 MkV with firmware 7.00).
pub const TACH_STEPS_PER_ELECTRICAL_TURN: f64 = 6.0;

/// A LiPo cell's voltage can be read anywhere in this range, in volts - a
/// battery's voltage over its cells says how many there are.
const CELL_V: std::ops::RangeInclusive<f64> = 3.2..=4.25;

/// Every cell count a battery at `voltage_v` could have, fewest first.
pub fn possible_cells(voltage_v: f64) -> Vec<u32> {
    (1..=12)
        .filter(|&cells| CELL_V.contains(&(voltage_v / f64::from(cells))))
        .collect()
}

/// The mean of `vectors`, component by component.
pub fn mean(vectors: &[[f64; 3]]) -> [f64; 3] {
    let n = vectors.len().max(1) as f64;
    let mut sum = [0.0; 3];
    for v in vectors {
        for (s, x) in sum.iter_mut().zip(v) {
            *s += x;
        }
    }
    sum.map(|s| s / n)
}

/// The largest standard deviation of any component of `vectors`.
pub fn max_std(vectors: &[[f64; 3]]) -> f64 {
    let m = mean(vectors);
    let n = vectors.len().max(1) as f64;
    (0..3)
        .map(|k| (vectors.iter().map(|v| (v[k] - m[k]).powi(2)).sum::<f64>() / n).sqrt())
        .fold(0.0, f64::max)
}

/// The car's z axis (down), from the accelerometer's mean with the car flat
/// and still, in g: the IMU axis gravity lies along, reversed from the way
/// the reading points (up).
pub fn down_axis(flat_g: [f64; 3]) -> Result<ImuAxis, String> {
    let (index, value) = largest(flat_g, None);
    let others = (0..3)
        .filter(|&k| k != index)
        .map(|k| flat_g[k].abs())
        .fold(0.0, f64::max);
    if !(0.8..=1.2).contains(&value.abs()) || others > 0.25 {
        return Err(format!(
            "the accelerometer reads {:.2} g, {:.2} g, {:.2} g - is the car flat and still?",
            flat_g[0], flat_g[1], flat_g[2]
        ));
    }
    Ok(ImuAxis::new(index, value < 0.0))
}

/// The car's x axis (forward), from the accelerometer's mean flat and with
/// the nose lifted, in g: lifting the nose tilts forward up, so the reading
/// along it grows.
pub fn forward_axis(
    flat_g: [f64; 3],
    nose_up_g: [f64; 3],
    down: ImuAxis,
) -> Result<ImuAxis, String> {
    let change = [0, 1, 2].map(|k| nose_up_g[k] - flat_g[k]);
    let (index, value) = largest(change, Some(down.index()));
    let other = (0..3)
        .filter(|&k| k != index && k != down.index())
        .map(|k| change[k].abs())
        .fold(0.0, f64::max);
    if value.abs() < 0.15 {
        return Err(
            "the nose hardly moved - lift it higher (a hand's width is plenty)".to_string(),
        );
    }
    if other > 0.5 * value.abs() {
        return Err("the car also rolled sideways - lift the nose straight up".to_string());
    }
    Ok(ImuAxis::new(index, value > 0.0))
}

/// The car's y axis (right): `down` x `forward`, completing the
/// right-handed frame.
pub fn right_axis(forward: ImuAxis, down: ImuAxis) -> ImuAxis {
    let (z, x) = (down.unit(), forward.unit());
    let y = [
        z[1] * x[2] - z[2] * x[1],
        z[2] * x[0] - z[0] * x[2],
        z[0] * x[1] - z[1] * x[0],
    ];
    let index = (0..3)
        .find(|&k| y[k] != 0.0)
        .expect("two different axes' cross product is an axis");
    ImuAxis::new(index, y[index] > 0.0)
}

/// Whether lifting the car's left side reads as it should with `mounting`:
/// its right side drops, so the reading along y (right) falls.
pub fn left_up_agrees(mounting: &ImuMounting, flat_g: [f64; 3], left_up_g: [f64; 3]) -> bool {
    mounting.y.of(left_up_g) - mounting.y.of(flat_g) < -0.15
}

/// The component of `v` with the largest magnitude, skipping `skip`, and its
/// value.
fn largest(v: [f64; 3], skip: Option<usize>) -> (usize, f64) {
    (0..3)
        .filter(|&k| Some(k) != skip)
        .map(|k| (k, v[k]))
        .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
        .expect("at least two components")
}

/// Which half of a raw lidar scan (as the sensor sends it) an object put on
/// the car's left shows up in - comparing a scan with it to one without.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScanHalf {
    First,
    Last,
}

/// Where an object added between `baseline` and `with_object` (readings in
/// meters, the same layout) is: the readings that came at least 0.15 m
/// closer, which must be a handful, well off straight ahead.
pub fn object_half(baseline: &[f32], with_object: &[f32]) -> Result<ScanHalf, String> {
    if baseline.len() != with_object.len() || baseline.len() < 10 {
        return Err("the two scans don't match - capture both again".to_string());
    }
    let closer: Vec<usize> = baseline
        .iter()
        .zip(with_object)
        .enumerate()
        .filter(|(_, (before, now))| **now < **before - 0.15)
        .map(|(i, _)| i)
        .collect();
    if closer.len() < 3 {
        return Err(
            "nothing new showed up - hold the object within a meter of the car's side".to_string(),
        );
    }
    let center = (baseline.len() - 1) as f64 / 2.0;
    let mean = closer.iter().map(|&i| i as f64).sum::<f64>() / closer.len() as f64;
    if (mean - center).abs() < 0.1 * baseline.len() as f64 {
        return Err(
            "the object is too close to straight ahead - hold it to the car's side".to_string(),
        );
    }
    Ok(if mean < center {
        ScanHalf::First
    } else {
        ScanHalf::Last
    })
}

/// The steering table measured on the bench: the servo at full left lock,
/// straight and full right lock, steering `left_rad` and `right_rad` (both
/// positive) at the locks - whichever way the servo turns.
pub fn bench_steering(
    left_servo: f64,
    straight_servo: f64,
    right_servo: f64,
    left_rad: f64,
    right_rad: f64,
) -> Result<SteeringTable, String> {
    if !(left_rad > 0.0 && right_rad > 0.0) {
        return Err("both full-lock angles must be positive".to_string());
    }
    let between = |a: f64, b: f64| a.min(b) < straight_servo && straight_servo < a.max(b);
    if !between(left_servo, right_servo) {
        return Err("straight must lie between full left and full right".to_string());
    }
    let mut points = vec![
        SteeringPoint {
            servo: left_servo,
            angle_rad: -left_rad,
        },
        SteeringPoint {
            servo: straight_servo,
            angle_rad: 0.0,
        },
        SteeringPoint {
            servo: right_servo,
            angle_rad: right_rad,
        },
    ];
    points.sort_by(|a, b| a.servo.total_cmp(&b.servo));
    let table = SteeringTable { points };
    table.validate()?;
    Ok(table)
}

/// Motor ERPM per meter/second of wheel speed, from the tachometer's steps
/// while the wheel turned `wheel_turns` times, `wheel_diameter_m` across.
pub fn speed_to_erpm_gain(
    tach_steps: i64,
    wheel_turns: f64,
    wheel_diameter_m: f64,
) -> Result<f64, String> {
    if !(wheel_turns > 0.0 && wheel_diameter_m > 0.0) {
        return Err("the wheel turns and its diameter must be positive".to_string());
    }
    let electrical_turns = tach_steps.unsigned_abs() as f64 / TACH_STEPS_PER_ELECTRICAL_TURN;
    if electrical_turns < 10.0 * wheel_turns {
        return Err(format!(
            "only {electrical_turns:.0} electrical turns for {wheel_turns} wheel turns - zero \
             the counter, then spin the wheel before counting"
        ));
    }
    let meters = wheel_turns * std::f64::consts::PI * wheel_diameter_m;
    Ok(electrical_turns / meters * 60.0)
}

/// One step of the motor's speed ramp: the ERPM commanded, and what the
/// motor held once settled.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct RampStep {
    pub commanded_erpm: f64,
    pub measured_erpm: f64,
    /// How much the measured ERPM wandered while settled.
    pub measured_std_erpm: f64,
}

impl RampStep {
    /// Whether the motor held this ERPM smoothly: close to it (the VESC's
    /// speed controller settles somewhat short) and steadily - a cogging
    /// sensorless motor stutters well below it.
    pub fn smooth(&self) -> bool {
        self.measured_erpm >= 0.6 * self.commanded_erpm
            && self.measured_std_erpm <= 0.1 * self.commanded_erpm
    }
}

/// What the ramp says about the motor.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct RampResult {
    /// The lowest commanded ERPM from which every step ran smoothly.
    pub min_smooth_erpm: f64,
    /// Commanded over measured ERPM, over the smooth steps - what makes up
    /// for the speed controller settling short.
    pub speed_compensation: f64,
}

/// Analyzes a ramp of increasing commanded ERPM.
pub fn analyze_ramp(steps: &[RampStep]) -> Result<RampResult, String> {
    let first_smooth = (0..steps.len())
        .find(|&i| steps[i..].iter().all(RampStep::smooth))
        .ok_or("the motor never ran smoothly - check it in VESC Tool")?;
    let smooth = &steps[first_smooth..];
    if smooth.len() < 2 {
        return Err("the motor only ran smoothly at the very top of the ramp".to_string());
    }
    let commanded: f64 = smooth.iter().map(|s| s.commanded_erpm).sum();
    let measured: f64 = smooth.iter().map(|s| s.measured_erpm).sum();
    Ok(RampResult {
        min_smooth_erpm: smooth[0].commanded_erpm,
        speed_compensation: commanded / measured,
    })
}

/// The slowest speed, in meters/second, the car should be driven at: the one
/// commanding `min_smooth_erpm` once compensated, plus 10%, rounded up to
/// the centimeter per second.
pub fn min_speed_mps(result: &RampResult, speed_to_erpm_gain: f64) -> f64 {
    let exact = result.min_smooth_erpm / (speed_to_erpm_gain * result.speed_compensation);
    (exact * 1.1 * 100.0).ceil() / 100.0
}

/// The median of `scan`'s readings within `half_width_rad` of straight
/// ahead, in meters - `None` if none is.
pub fn distance_ahead_m(scan: &LidarScan, half_width_rad: f32) -> Option<f64> {
    let mut ahead: Vec<f32> = readings_ahead(scan, half_width_rad).collect();
    if ahead.is_empty() {
        return None;
    }
    ahead.sort_by(f32::total_cmp);
    Some(f64::from(ahead[ahead.len() / 2]))
}

/// The nearest of `scan`'s readings within `half_width_rad` of straight
/// ahead, in meters - `None` if none is.
pub fn nearest_ahead_m(scan: &LidarScan, half_width_rad: f32) -> Option<f64> {
    readings_ahead(scan, half_width_rad)
        .min_by(f32::total_cmp)
        .map(f64::from)
}

/// `scan`'s readings within `half_width_rad` of straight ahead - either way
/// round, so whether the sensor is upside down doesn't matter.
fn readings_ahead(scan: &LidarScan, half_width_rad: f32) -> impl Iterator<Item = f32> + '_ {
    (0..scan.points.len())
        .filter(move |&i| scan.angle_rad(i).abs() <= half_width_rad)
        .map(|i| scan.points[i])
        .filter(|&r| r > scan.min_distance && r < scan.max_distance)
}

/// One reading while the car drives itself on the floor.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct DriveSample {
    /// Since the drive started, in seconds.
    pub t_s: f64,
    pub erpm: f64,
    pub tachometer: i32,
    /// The gyroscope, in the IMU's axes, in deg/s.
    pub gyro_deg_s: [f64; 3],
}

/// The part of a drive the car was cruising: past the first `skip` of the
/// distance it drove (the steering and speed settling), and before it
/// started slowing down.
fn cruise(samples: &[DriveSample], skip: f64) -> &[DriveSample] {
    let (Some(first), Some(last)) = (samples.first(), samples.last()) else {
        return &[];
    };
    let total = i64::from(last.tachometer) - i64::from(first.tachometer);
    let from = samples
        .iter()
        .position(|s| {
            (i64::from(s.tachometer) - i64::from(first.tachometer)).abs() as f64
                >= skip * total.abs() as f64
        })
        .unwrap_or(samples.len());
    let peak = samples.iter().map(|s| s.erpm.abs()).fold(0.0, f64::max);
    let to = samples
        .iter()
        .rposition(|s| s.erpm.abs() >= 0.8 * peak)
        .map_or(0, |i| i + 1);
    if from < to { &samples[from..to] } else { &[] }
}

/// The car's speed (m/s) and yaw rate (rad/s, positive right) while
/// cruising, `gyro_bias_deg_s` (read standing still) taken out - `None` if
/// it didn't cruise for at least half a second.
pub fn cruise_motion(
    samples: &[DriveSample],
    skip: f64,
    gyro_bias_deg_s: [f64; 3],
    imu: &ImuMounting,
    speed_to_erpm_gain: f64,
) -> Option<(f64, f64)> {
    let cruise = cruise(samples, skip);
    let duration = cruise.last()?.t_s - cruise.first()?.t_s;
    if duration < 0.5 {
        return None;
    }
    let n = cruise.len() as f64;
    let erpm = cruise.iter().map(|s| s.erpm).sum::<f64>() / n;
    let yaw_deg_s = cruise
        .iter()
        .map(|s| {
            imu.z
                .of([0, 1, 2].map(|k| s.gyro_deg_s[k] - gyro_bias_deg_s[k]))
        })
        .sum::<f64>()
        / n;
    Some((erpm / speed_to_erpm_gain, yaw_deg_s.to_radians()))
}

/// What driving straight at a wall found.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct StraightResult {
    /// How far the car drove, by the lidar, in meters.
    pub distance_m: f64,
    /// Motor ERPM per meter/second, from that distance.
    pub speed_to_erpm_gain: f64,
    /// How much the car curved while cruising, in 1/m, positive right.
    pub curvature_per_m: f64,
    /// The servo position that steers it straight instead.
    pub straight_servo: f64,
}

/// Works out a drive straight at a wall: the lidar saw the wall
/// `start_m` then `end_m` away (standing still), the tachometer counted
/// `tach_steps` in between.
#[allow(clippy::too_many_arguments)]
pub fn straight_result(
    start_m: f64,
    end_m: f64,
    tach_steps: i64,
    samples: &[DriveSample],
    gyro_bias_deg_s: [f64; 3],
    car: &CarCalibration,
) -> Result<StraightResult, String> {
    let distance_m = start_m - end_m;
    if distance_m < 1.0 {
        return Err(format!(
            "the car only drove {distance_m:.2} m by the lidar - start 2.5 to 4 m from the wall"
        ));
    }
    let electrical_turns = tach_steps.unsigned_abs() as f64 / TACH_STEPS_PER_ELECTRICAL_TURN;
    let gain = electrical_turns / distance_m * 60.0;
    let (speed_mps, yaw_rad_s) = cruise_motion(samples, 0.3, gyro_bias_deg_s, &car.imu, gain)
        .ok_or("the car didn't cruise long enough - start farther from the wall")?;
    let curvature_per_m = yaw_rad_s / speed_mps.max(0.05);
    // The servo that, by the steering table, turns back as much as it
    // curved.
    let correction_rad = -(car.geometry.wheelbase_m * curvature_per_m).atan();
    Ok(StraightResult {
        distance_m,
        speed_to_erpm_gain: gain,
        curvature_per_m,
        straight_servo: car.steering.servo_for(correction_rad),
    })
}

/// `table` steering straight at `servo`: its straight point moved there (or
/// added, if it had none).
pub fn with_straight(table: &SteeringTable, servo: f64) -> Result<SteeringTable, String> {
    let mut points: Vec<SteeringPoint> = table
        .points
        .iter()
        .copied()
        .filter(|p| p.angle_rad != 0.0)
        .collect();
    points.push(SteeringPoint {
        servo,
        angle_rad: 0.0,
    });
    points.sort_by(|a, b| a.servo.total_cmp(&b.servo));
    let table = SteeringTable { points };
    table
        .validate()
        .map_err(|err| format!("straight at servo {servo:.3} breaks the steering table: {err}"))?;
    Ok(table)
}

/// The fractions of each side's range the arcs are driven at.
pub const ARC_FRACTIONS: [f64; 4] = [0.25, 0.5, 0.75, 1.0];

/// The servo positions the arcs are driven at: [`ARC_FRACTIONS`] of the way
/// from straight to each end of `table`, the lowest servo first.
pub fn arc_servos(table: &SteeringTable) -> Vec<f64> {
    let straight = table.straight_servo();
    let (low, high) = table.servo_range();
    let lower = ARC_FRACTIONS
        .iter()
        .rev()
        .map(|f| straight + f * (low - straight));
    let upper = ARC_FRACTIONS
        .iter()
        .map(|f| straight + f * (high - straight));
    lower.chain(upper).collect()
}

/// The steering angle an arc driven at `servo` measured: the bicycle
/// model's `atan(wheelbase * curvature)`, curvature = yaw rate / speed.
pub fn arc_point(
    servo: f64,
    samples: &[DriveSample],
    gyro_bias_deg_s: [f64; 3],
    car: &CarCalibration,
) -> Result<SteeringPoint, String> {
    let (speed_mps, yaw_rad_s) = cruise_motion(
        samples,
        0.3,
        gyro_bias_deg_s,
        &car.imu,
        car.motor.speed_to_erpm_gain,
    )
    .ok_or("the car didn't cruise long enough - drive the whole arc")?;
    if speed_mps < 0.1 {
        return Err("the car hardly moved".to_string());
    }
    Ok(SteeringPoint {
        servo,
        angle_rad: (car.geometry.wheelbase_m * yaw_rad_s / speed_mps).atan(),
    })
}

/// The steering table the arcs measured, with straight at
/// `straight_servo`: each side's full-lock arc is needed, since the table's
/// ends are the servo's limits.
pub fn table_from_arcs(
    arcs: &[SteeringPoint],
    straight_servo: f64,
    servo_range: (f64, f64),
) -> Result<SteeringTable, String> {
    let has = |servo: f64| arcs.iter().any(|p| (p.servo - servo).abs() < 1e-9);
    if !has(servo_range.0) || !has(servo_range.1) {
        return Err("drive both full-lock arcs first".to_string());
    }
    let mut points = arcs.to_vec();
    points.push(SteeringPoint {
        servo: straight_servo,
        angle_rad: 0.0,
    });
    points.sort_by(|a, b| a.servo.total_cmp(&b.servo));
    let table = SteeringTable { points };
    table.validate().map_err(|err| {
        format!("the arcs don't make a steering table ({err}) - drive the odd one again")
    })?;
    Ok(table)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64, tolerance: f64) -> bool {
        (a - b).abs() < tolerance
    }

    #[test]
    fn a_batterys_cells_follow_from_its_voltage() {
        assert_eq!(possible_cells(15.4), vec![4]);
        assert_eq!(possible_cells(12.6), vec![3]);
        // A full 4S or a nearly empty 5S: only the label tells.
        assert_eq!(possible_cells(16.8), vec![4, 5]);
        assert_eq!(possible_cells(1.0), Vec::<u32>::new());
    }

    /// The first car's VESC: flat, z up; x to the car's left, y backwards
    /// (measured by tilting it: nose up -> accel y -0.38, left side up ->
    /// accel x +0.58).
    #[test]
    fn the_first_cars_mounting_is_found_from_three_poses() {
        let flat = [0.0, 0.0, 1.0];
        let nose_up = [0.0, -0.38, 0.92];
        let left_up = [0.58, 0.0, 0.81];
        let z = down_axis(flat).unwrap();
        assert_eq!(z, ImuAxis::MinusZ);
        let x = forward_axis(flat, nose_up, z).unwrap();
        assert_eq!(x, ImuAxis::MinusY);
        let y = right_axis(x, z);
        assert_eq!(y, ImuAxis::MinusX);
        let mounting = ImuMounting { x, y, z };
        assert!(left_up_agrees(&mounting, flat, left_up));
        // The other way round would be a mirror image.
        assert!(!left_up_agrees(&mounting, flat, [-0.58, 0.0, 0.81]));
    }

    #[test]
    fn an_upside_down_or_tilted_imu_is_handled_or_refused() {
        assert_eq!(down_axis([0.02, -0.98, 0.05]).unwrap(), ImuAxis::PlusY);
        assert!(down_axis([0.5, 0.0, 0.85]).is_err());
        assert!(down_axis([0.0, 0.0, 0.3]).is_err());
        let z = ImuAxis::MinusZ;
        assert!(forward_axis([0.0, 0.0, 1.0], [0.05, 0.0, 1.0], z).is_err());
        assert!(forward_axis([0.0, 0.0, 1.0], [0.3, 0.3, 0.9], z).is_err());
        assert_eq!(
            forward_axis([0.0, 0.0, 1.0], [0.4, 0.02, 0.9], z).unwrap(),
            ImuAxis::PlusX
        );
    }

    #[test]
    fn a_right_handed_frame_is_completed() {
        // x forward, z down: y right, for the IMU's own axes.
        assert_eq!(right_axis(ImuAxis::PlusX, ImuAxis::PlusZ), ImuAxis::PlusY);
        assert_eq!(right_axis(ImuAxis::PlusX, ImuAxis::MinusZ), ImuAxis::MinusY);
    }

    #[test]
    fn an_object_on_the_left_is_found_in_its_half_of_the_scan() {
        let baseline = vec![5.0_f32; 100];
        let mut left_first = baseline.clone();
        left_first[10..15].fill(0.4);
        assert_eq!(object_half(&baseline, &left_first), Ok(ScanHalf::First));
        let mut left_last = baseline.clone();
        left_last[80..86].fill(0.4);
        assert_eq!(object_half(&baseline, &left_last), Ok(ScanHalf::Last));
        let mut ahead = baseline.clone();
        ahead[48..52].fill(0.4);
        assert!(object_half(&baseline, &ahead).is_err());
        assert!(object_half(&baseline, &baseline).is_err());
    }

    #[test]
    fn the_bench_steering_table_follows_the_servo_either_way() {
        // Higher servo steers right (the first car).
        let table = bench_steering(0.17, 0.5, 0.83, 0.475, 0.475).unwrap();
        assert_eq!(table.points[0].servo, 0.17);
        assert_eq!(table.points[0].angle_rad, -0.475);
        // Higher servo steers left, and unevenly.
        let table = bench_steering(0.8, 0.45, 0.2, 0.3, 0.4).unwrap();
        assert_eq!(table.points[0].servo, 0.2);
        assert_eq!(table.points[0].angle_rad, 0.4);
        assert!(close(table.max_angle_rad(), 0.3, 1e-12));
        assert!(bench_steering(0.2, 0.9, 0.8, 0.4, 0.4).is_err());
        assert!(bench_steering(0.2, 0.5, 0.8, 0.0, 0.4).is_err());
    }

    #[test]
    fn the_first_cars_speed_gain_is_found_from_its_count() {
        // 366 electrical turns for 16 turns of a 9 cm wheel.
        let gain = speed_to_erpm_gain(366 * 6, 16.0, 0.09).unwrap();
        assert!(close(gain, 4854.0, 1.0), "{gain}");
        assert!(speed_to_erpm_gain(-366 * 6, 16.0, 0.09).is_ok());
        assert!(speed_to_erpm_gain(0, 16.0, 0.09).is_err());
        assert!(speed_to_erpm_gain(600, 0.0, 0.09).is_err());
    }

    fn step(commanded: f64, measured: f64, std: f64) -> RampStep {
        RampStep {
            commanded_erpm: commanded,
            measured_erpm: measured,
            measured_std_erpm: std,
        }
    }

    #[test]
    fn the_ramp_finds_where_the_motor_runs_smoothly_and_how_short_it_settles() {
        // The first car: stalls at 1000, ~15% short once running.
        let steps = [
            step(1000.0, 200.0, 150.0),
            step(1500.0, 1400.0, 300.0),
            step(2000.0, 1720.0, 40.0),
            step(2500.0, 2150.0, 30.0),
            step(3000.0, 2580.0, 30.0),
        ];
        let result = analyze_ramp(&steps).unwrap();
        assert_eq!(result.min_smooth_erpm, 2000.0);
        assert!(close(result.speed_compensation, 1.163, 0.001));
        let min = min_speed_mps(&result, 4854.0);
        assert!(close(min, 0.39, 1e-9), "{min}");
        // A smooth step below a stuttering one doesn't count.
        let steps = [
            step(1000.0, 900.0, 10.0),
            step(1500.0, 300.0, 200.0),
            step(2000.0, 1800.0, 20.0),
        ];
        assert!(analyze_ramp(&steps).is_err());
        assert!(analyze_ramp(&[step(1000.0, 100.0, 100.0)]).is_err());
    }

    /// Samples of a car cruising at `erpm` and yawing at `yaw_deg_s` about
    /// the IMU's z axis, one every 20 ms for `seconds`, the tachometer
    /// counting on.
    fn cruising(erpm: f64, yaw_deg_s: f64, seconds: f64) -> Vec<DriveSample> {
        let n = (seconds / 0.02) as usize;
        (0..n)
            .map(|i| {
                let t_s = i as f64 * 0.02;
                DriveSample {
                    t_s,
                    erpm,
                    tachometer: (erpm / 60.0 * 6.0 * t_s) as i32,
                    gyro_deg_s: [0.0, 0.0, yaw_deg_s],
                }
            })
            .collect()
    }

    #[test]
    fn readings_ahead_are_found_either_way_round() {
        let points: Vec<f32> = (0..181)
            .map(|i| if (88..=92).contains(&i) { 2.0 } else { 5.0 })
            .collect();
        let scan = LidarScan::new(points, vec![1.0; 181], 0.1, 30.0, std::f32::consts::PI);
        assert_eq!(distance_ahead_m(&scan, 2f32.to_radians()), Some(2.0));
        assert_eq!(nearest_ahead_m(&scan, 35f32.to_radians()), Some(2.0));
    }

    #[test]
    fn driving_straight_at_a_wall_finds_the_gain_and_the_trim() {
        // The template car, IMU z up (read reversed: the car's z is down).
        let mut car = CarCalibration::template("test");
        car.imu = ImuMounting {
            x: ImuAxis::PlusX,
            y: ImuAxis::MinusY,
            z: ImuAxis::MinusZ,
        };
        // 2 m at 2400 ERPM, 5000 ERPM per m/s: 0.48 m/s for 4.17 s.
        let samples = cruising(2400.0, 0.0, 2.0 / 0.48);
        let steps = (2.0 * 5000.0 / 60.0 * 6.0) as i64;
        let result = straight_result(3.0, 1.0, steps, &samples, [0.0; 3], &car).unwrap();
        assert!(close(result.speed_to_erpm_gain, 5000.0, 1.0));
        assert!(close(result.curvature_per_m, 0.0, 1e-9));
        assert!(close(
            result.straight_servo,
            car.steering.straight_servo(),
            1e-9
        ));
        // Curving right (a negative reading on the IMU's z, which points
        // up): steer a little left.
        let samples = cruising(2400.0, -2.0, 2.0 / 0.48);
        let result = straight_result(3.0, 1.0, steps, &samples, [0.0; 3], &car).unwrap();
        assert!(result.curvature_per_m > 0.0);
        assert!(result.straight_servo < car.steering.straight_servo());
        // A drift read standing still is taken out.
        let result = straight_result(3.0, 1.0, steps, &samples, [0.0, 0.0, -2.0], &car).unwrap();
        assert!(close(result.curvature_per_m, 0.0, 1e-9));
        assert!(straight_result(3.0, 2.5, steps, &samples, [0.0; 3], &car).is_err());
    }

    #[test]
    fn an_arc_measures_its_steering_angle() {
        let mut car = CarCalibration::template("test");
        car.motor.speed_to_erpm_gain = 5000.0;
        car.imu.z = ImuAxis::MinusZ;
        // 0.5 m/s on a 1 m radius: 0.5 rad/s to the right.
        let samples = cruising(2500.0, -(0.5f64.to_degrees()), 4.0);
        let point = arc_point(0.8, &samples, [0.0; 3], &car).unwrap();
        let expected = car.geometry.wheelbase_m.atan();
        assert!(close(point.angle_rad, expected, 1e-6), "{point:?}");
    }

    #[test]
    fn the_arcs_make_a_steering_table() {
        let bench = bench_steering(0.2, 0.5, 0.8, 0.4, 0.4).unwrap();
        let servos = arc_servos(&bench);
        assert_eq!(servos.len(), 8);
        assert!(close(servos[0], 0.2, 1e-12) && close(servos[7], 0.8, 1e-12));
        assert!(close(servos[3], 0.425, 1e-12) && close(servos[4], 0.575, 1e-12));
        let arcs: Vec<SteeringPoint> = servos
            .iter()
            .map(|&servo| SteeringPoint {
                servo,
                angle_rad: (servo - 0.5) * 0.9,
            })
            .collect();
        let table = table_from_arcs(&arcs, 0.5, (0.2, 0.8)).unwrap();
        assert_eq!(table.points.len(), 9);
        assert!(table_from_arcs(&arcs[1..], 0.5, (0.2, 0.8)).is_err());
        let mut odd = arcs.clone();
        odd[5].angle_rad = -0.1;
        assert!(table_from_arcs(&odd, 0.5, (0.2, 0.8)).is_err());
    }

    #[test]
    fn straight_can_be_moved_within_the_table() {
        let bench = bench_steering(0.2, 0.5, 0.8, 0.4, 0.4).unwrap();
        let moved = with_straight(&bench, 0.52).unwrap();
        assert!(close(moved.straight_servo(), 0.52, 1e-12));
        assert_eq!(moved.points.len(), 3);
        assert!(with_straight(&bench, 0.9).is_err());
    }
}
