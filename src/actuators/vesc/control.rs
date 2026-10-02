//! What [`super::Vesc`] sends the VESC for a [`VescCommand`], and how it
//! reads the VESC's IMU - from the car's calibration
//! ([`CarCalibration`]) and how it's driven ([`VescConfig`]), with no I/O,
//! so it can be tested without the car.

use super::protocol::Imu;
use crate::calibration::CarCalibration;
use crate::topics::{
    ActuatorLimits, AlgorithmParameter, ImuReading, VescCommand, VescParameters,
    VescParametersStatus,
};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// A LiPo cell's resting voltage at each tenth of its charge, `0.0` (empty)
/// to `1.0` (full).
const LIPO_CELL_V: [f64; 11] = [
    3.30, 3.69, 3.73, 3.77, 3.80, 3.84, 3.87, 3.95, 4.02, 4.11, 4.20,
];

/// Standard gravity, in meters/second^2 - the VESC reports accelerations in g.
const G_MPS2: f64 = 9.806_65;

/// How [`super::Vesc`] drives the car - loaded from
/// `config/actuators/vesc.toml` (see [`Default`]) or from an arbitrary path
/// via [`crate::config::load`]. Its numeric values can also be tuned live
/// (see [`VescConfig::tunable_parameters`]). What the car itself is - how
/// its servo steers, its motor's ERPM per meter/second, its IMU's axes - is
/// its [`CarCalibration`] instead.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VescConfig {
    /// The VESC's serial port.
    pub port: String,
    /// How often the command is sent and the VESC read, in Hz.
    pub rate_hz: f64,
    /// How long the VESC may take to answer, in seconds.
    pub reply_timeout_s: f64,
    /// How long to wait before reconnecting after losing the VESC, in
    /// seconds.
    pub reconnect_delay_s: f64,
    /// Commands older than this, in seconds, count as stale: the car stops.
    pub command_timeout_s: f64,

    /// Speeds below this, in meters/second, brake instead.
    pub stop_speed_mps: f64,
    /// The current the motor brakes with, in amperes.
    pub brake_current_a: f64,
    /// Below this voltage per battery cell, in volts, the battery should be
    /// recharged - flagged in [`crate::topics::VescStatus::low_battery`].
    pub low_battery_cell_v: f64,

    /// The limits the car is driven within - see [`VescLimits`].
    pub limits: VescLimits,
}

/// [`ActuatorLimits`] but the steering angle, which is the car's
/// ([`crate::calibration::SteeringTable::max_angle_rad`]) - see
/// [`VescConfig::actuator_limits`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VescLimits {
    /// Fastest the steering angle can change, in radians/second.
    pub max_steering_rate_rad_s: f64,
    /// Fastest the car is driven, either way, in meters/second - a hard cap
    /// on every command.
    pub max_speed_mps: f64,
    /// Largest acceleration and deceleration, in meters/second^2.
    pub max_accel_mps2: f64,
    pub max_decel_mps2: f64,
}

impl VescLimits {
    /// The live-tunable limits: [`ActuatorLimits::tunable_parameters`] but
    /// the steering angle.
    pub fn tunable_parameters() -> Vec<AlgorithmParameter> {
        ActuatorLimits::tunable_parameters_but_steering_angle()
    }
}

impl Default for VescConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../../config/actuators/vesc.toml"))
            .expect("config/actuators/vesc.toml must deserialize into VescConfig")
    }
}

impl VescConfig {
    /// Every top-level value that can be tuned while the car runs - all the
    /// numeric ones: the port only changes in the file. The `[limits]` are
    /// [`VescLimits::tunable_parameters`]. Values read once per connection
    /// (`reply_timeout_s`) apply from the next reconnect.
    pub fn tunable_parameters() -> Vec<AlgorithmParameter> {
        vec![
            AlgorithmParameter::float("rate_hz", 10.0, 200.0, 1.0)
                .unit("Hz")
                .description("How often the command is sent and the VESC read."),
            AlgorithmParameter::float("reply_timeout_s", 0.005, 0.5, 0.005)
                .unit("s")
                .description("How long the VESC may take to answer - from the next reconnect."),
            AlgorithmParameter::float("reconnect_delay_s", 0.1, 10.0, 0.1)
                .unit("s")
                .description("How long to wait before reconnecting after losing the VESC."),
            AlgorithmParameter::float("command_timeout_s", 0.05, 2.0, 0.05)
                .unit("s")
                .description("Commands older than this count as stale: the car stops."),
            AlgorithmParameter::float("stop_speed_mps", 0.0, 1.0, 0.01)
                .unit("m/s")
                .description("Speeds below this brake instead."),
            AlgorithmParameter::float("brake_current_a", 0.0, 20.0, 0.1)
                .unit("A")
                .description("The current the motor brakes with."),
            AlgorithmParameter::float("low_battery_cell_v", 3.0, 4.2, 0.01)
                .unit("V")
                .description("Below this voltage per cell, the GUI warns to recharge."),
        ]
    }

    /// The limits the car is driven within: [`Self::limits`], steering as
    /// far as `car` does both ways.
    pub fn actuator_limits(&self, car: &CarCalibration) -> ActuatorLimits {
        ActuatorLimits {
            max_steering_angle_rad: car.steering.max_angle_rad(),
            max_steering_rate_rad_s: self.limits.max_steering_rate_rad_s,
            max_speed_mps: self.limits.max_speed_mps,
            max_accel_mps2: self.limits.max_accel_mps2,
            max_decel_mps2: self.limits.max_decel_mps2,
        }
    }

    /// Checks the values `car` couldn't be driven with - e.g. a
    /// `stop_speed_mps` above the car's `min_speed_mps`.
    pub fn validate(&self, car: &CarCalibration) -> Result<(), String> {
        if self.rate_hz <= 0.0 {
            return Err("rate_hz must be positive".to_string());
        }
        if self.stop_speed_mps > car.motor.min_speed_mps {
            return Err("stop_speed_mps must not be above the car's min_speed_mps".to_string());
        }
        self.actuator_limits(car).validate()
    }

    /// Applies `wanted`, sanitized (see [`crate::config::apply_parameters`]),
    /// but only if the result still [`validate`](Self::validate)s for `car`:
    /// a bad combination is refused whole and the values in effect stay.
    /// Returns whether anything changed.
    pub fn apply(&mut self, wanted: &VescParameters, car: &CarCalibration) -> Result<bool, String> {
        use crate::config::apply_parameters;
        let mut next = self.clone();
        let values_changed =
            apply_parameters(&mut next, &Self::tunable_parameters(), &wanted.values);
        let limits_changed = apply_parameters(
            &mut next.limits,
            &VescLimits::tunable_parameters(),
            &wanted.limits,
        );
        if !values_changed && !limits_changed {
            return Ok(false);
        }
        next.validate(car)?;
        *self = next;
        Ok(true)
    }

    /// What [`super::Vesc`] publishes on
    /// [`crate::topics::VESC_PARAMETERS_STATUS_TOPIC_NAME`] while driving
    /// with this config.
    pub fn parameters_status(&self) -> VescParametersStatus {
        let mut parameters = Self::tunable_parameters();
        crate::config::refresh_parameter_values(&mut parameters, self);
        let mut limits = VescLimits::tunable_parameters();
        crate::config::refresh_parameter_values(&mut limits, &self.limits);
        VescParametersStatus { parameters, limits }
    }
}

/// The file the VESC's settings are saved to and reloaded from:
/// `config/actuators/vesc.toml`.
pub fn config_path() -> PathBuf {
    Path::new(crate::config::DEFAULT_CONFIG_ROOT)
        .join("actuators")
        .join("vesc.toml")
}

/// The table of [`config_path`] `parameters` live in: the top level for
/// [`VescConfig::tunable_parameters`], `[limits]` for the limits.
fn table(limits: bool) -> Option<&'static str> {
    limits.then_some("limits")
}

/// Writes `parameters`' values into [`config_path`] - its `[limits]` table
/// if `limits` - leaving everything else in the file untouched. Returns the
/// file's path.
pub fn save_parameters(limits: bool, parameters: &[AlgorithmParameter]) -> Result<PathBuf, String> {
    let path = config_path();
    let values: Vec<(&str, String)> = parameters
        .iter()
        .map(|parameter| {
            (
                parameter.name.as_str(),
                crate::config::parameter_toml_value(parameter),
            )
        })
        .collect();
    crate::config::save_toml_values(&path, table(limits), &values)?;
    Ok(path)
}

/// The values [`config_path`] holds for `parameters` - in its `[limits]`
/// table if `limits` - by name, and the file's path.
pub fn saved_values(
    limits: bool,
    parameters: &[AlgorithmParameter],
) -> Result<(BTreeMap<String, f64>, PathBuf), String> {
    let path = config_path();
    let names: Vec<&str> = parameters
        .iter()
        .map(|parameter| parameter.name.as_str())
        .collect();
    Ok((
        crate::config::load_toml_values(&path, table(limits), &names)?,
        path,
    ))
}

/// Where the car is driven towards: the command, once the limits allow it
/// to get there.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Setpoint {
    pub steering_rad: f64,
    pub speed_mps: f64,
}

impl Setpoint {
    /// This setpoint moved towards `command` over `dt_s`, as fast as
    /// `limits` allow - the same limits [`crate::simulation::SimulatedVehicle`]
    /// applies, so the car responds like the simulated one.
    pub fn toward(self, command: VescCommand, limits: &ActuatorLimits, dt_s: f64) -> Self {
        let max_angle = limits.max_steering_angle_rad;
        let target_rad = command.servo_position_rad.clamp(-max_angle, max_angle);
        let max_turn = limits.max_steering_rate_rad_s * dt_s;
        let steering_rad = (self.steering_rad
            + (target_rad - self.steering_rad).clamp(-max_turn, max_turn))
        .clamp(-max_angle, max_angle);

        let target_mps = command
            .speed_mps
            .clamp(-limits.max_speed_mps, limits.max_speed_mps);
        // Speeding up means away from zero, either way.
        let speeding_up = target_mps.abs() > self.speed_mps.abs()
            && target_mps.signum() * self.speed_mps.signum() >= 0.0;
        let rate = if speeding_up {
            limits.max_accel_mps2
        } else {
            limits.max_decel_mps2
        };
        let max_change = rate * dt_s;
        let speed_mps =
            self.speed_mps + (target_mps - self.speed_mps).clamp(-max_change, max_change);
        Self {
            steering_rad,
            speed_mps,
        }
    }
}

/// What the motor is told.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Motor {
    /// Hold this ERPM.
    Erpm(i32),
    /// Brake with this current, in amperes.
    Brake(f64),
}

/// The servo position steering the wheels `steering_rad`, never past the
/// ends of `car`'s steering table.
pub fn servo_position(car: &CarCalibration, steering_rad: f64) -> f64 {
    car.steering.servo_for(steering_rad)
}

/// What the motor is told for `speed_mps`: a brake below
/// `stop_speed_mps`, and otherwise at least the car's `min_speed_mps`,
/// never past `max_speed_mps`, compensated.
pub fn motor(config: &VescConfig, car: &CarCalibration, speed_mps: f64) -> Motor {
    let magnitude = speed_mps.abs();
    if magnitude < config.stop_speed_mps || !speed_mps.is_finite() {
        return Motor::Brake(config.brake_current_a);
    }
    let magnitude = magnitude
        .max(car.motor.min_speed_mps)
        .min(config.limits.max_speed_mps);
    let erpm = speed_mps.signum()
        * magnitude
        * car.motor.speed_to_erpm_gain
        * car.motor.speed_compensation;
    Motor::Erpm(erpm.round() as i32)
}

/// Whether a battery of `cells` cells at `voltage_v` should be recharged.
pub fn low_battery(config: &VescConfig, cells: u32, voltage_v: f64) -> bool {
    voltage_v < config.low_battery_cell_v * f64::from(cells)
}

/// The estimated charge of a battery of `cells` cells, `0.0` (empty) to
/// `1.0` (full), from its voltage - interpolated along a LiPo cell's
/// discharge curve. Only a resting battery's voltage says it well: under
/// load it sags, reading emptier than it is.
pub fn battery_charge(cells: u32, voltage_v: f64) -> f64 {
    let cell_v = voltage_v / f64::from(cells.max(1));
    let above = LIPO_CELL_V.partition_point(|&v| v < cell_v);
    if above == 0 {
        return 0.0;
    }
    if above == LIPO_CELL_V.len() {
        return 1.0;
    }
    let (low, high) = (LIPO_CELL_V[above - 1], LIPO_CELL_V[above]);
    ((above - 1) as f64 + (cell_v - low) / (high - low)) / (LIPO_CELL_V.len() - 1) as f64
}

/// The VESC's readings as an [`ImuReading`]: wheel speed from the motor's
/// measured ERPM, and the IMU's in the car's frame and SI units.
pub fn imu_reading(car: &CarCalibration, erpm: f64, imu: &Imu) -> ImuReading {
    ImuReading {
        wheel_speed_mps: erpm / car.motor.speed_to_erpm_gain,
        yaw_rate_rad_s: car.imu.z.of(imu.gyro_deg_s).to_radians(),
        ax_mps2: car.imu.x.of(imu.accel_g) * G_MPS2,
        ay_mps2: car.imu.y.of(imu.accel_g) * G_MPS2,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calibration::{ImuAxis, ImuMounting};

    fn config() -> VescConfig {
        VescConfig::default()
    }

    /// The template car, its IMU mounted like tom's: rotated 90 degrees,
    /// x to the car's left, y backwards, z up.
    fn car() -> CarCalibration {
        let mut car = CarCalibration::template("test");
        car.imu = ImuMounting {
            x: ImuAxis::MinusY,
            y: ImuAxis::MinusX,
            z: ImuAxis::MinusZ,
        };
        car
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn the_config_loads_and_is_valid_for_every_tracked_car() {
        let config = config();
        assert_eq!(config.port, "/dev/sensors/vesc");
        config.validate(&car()).unwrap();
        let root = Path::new(crate::config::DEFAULT_CONFIG_ROOT);
        for name in crate::calibration::car_names(root) {
            let car = crate::calibration::load_car(root, &name).unwrap();
            config.validate(&car).unwrap();
        }
    }

    #[test]
    fn the_steering_angle_limit_is_the_cars_smaller_side() {
        let mut car = car();
        car.steering.points[0].angle_rad = -0.3;
        let limits = config().actuator_limits(&car);
        assert_eq!(limits.max_steering_angle_rad, 0.3);
        assert_eq!(limits.max_speed_mps, config().limits.max_speed_mps);
    }

    #[test]
    fn the_servo_follows_the_cars_steering_table() {
        let car = car();
        assert_eq!(servo_position(&car, 0.0), car.steering.straight_servo());
        // Positive steers right, which the template's servo does upwards.
        assert!(servo_position(&car, 0.2) > car.steering.straight_servo());
        assert!(servo_position(&car, -0.2) < car.steering.straight_servo());
        let (min, max) = car.steering.servo_range();
        assert_eq!(servo_position(&car, 10.0), max);
        assert_eq!(servo_position(&car, -10.0), min);
    }

    #[test]
    fn slow_speeds_brake_and_creeping_ones_are_raised_to_the_minimum() {
        let (config, car) = (config(), car());
        let brake = Motor::Brake(config.brake_current_a);
        assert_eq!(motor(&config, &car, 0.0), brake);
        assert_eq!(motor(&config, &car, 0.01), brake);
        assert_eq!(motor(&config, &car, f64::NAN), brake);
        let minimum = motor(&config, &car, car.motor.min_speed_mps);
        assert_eq!(motor(&config, &car, 0.1), minimum);
        let Motor::Erpm(reverse) = motor(&config, &car, -0.1) else {
            panic!("reverse should drive");
        };
        assert_eq!(Motor::Erpm(-reverse), minimum);
    }

    #[test]
    fn erpm_is_compensated_and_capped_at_the_maximum_speed() {
        let (config, mut car) = (config(), car());
        car.motor.speed_compensation = 1.16;
        let expected = 0.8 * car.motor.speed_to_erpm_gain * 1.16;
        assert_eq!(
            motor(&config, &car, 0.8),
            Motor::Erpm(expected.round() as i32)
        );
        assert_eq!(
            motor(&config, &car, 50.0),
            motor(&config, &car, config.limits.max_speed_mps)
        );
    }

    #[test]
    fn the_setpoint_follows_the_command_within_the_limits() {
        let limits = config().actuator_limits(&car());
        let command = VescCommand::new(1.0, 5.0);
        let dt_s = 0.01;
        let next = Setpoint::default().toward(command, &limits, dt_s);
        assert!(close(
            next.steering_rad,
            limits.max_steering_rate_rad_s * dt_s
        ));
        assert!(close(next.speed_mps, limits.max_accel_mps2 * dt_s));

        let mut setpoint = Setpoint::default();
        for _ in 0..1000 {
            setpoint = setpoint.toward(command, &limits, dt_s);
        }
        assert!(close(setpoint.steering_rad, limits.max_steering_angle_rad));
        assert!(close(setpoint.speed_mps, limits.max_speed_mps));
    }

    #[test]
    fn slowing_down_and_reversing_use_the_deceleration_limit() {
        let limits = config().actuator_limits(&car());
        let moving = Setpoint {
            steering_rad: 0.0,
            speed_mps: 1.0,
        };
        let dt_s = 0.01;
        let stopping = moving.toward(VescCommand::default(), &limits, dt_s);
        assert!(close(
            stopping.speed_mps,
            1.0 - limits.max_decel_mps2 * dt_s
        ));
        let reversing = moving.toward(VescCommand::new(0.0, -1.0), &limits, dt_s);
        assert!(close(reversing.speed_mps, stopping.speed_mps));
    }

    #[test]
    fn the_battery_charge_follows_the_lipo_curve() {
        let config = config();
        let cells = 4;
        let volts = |per_cell: f64| per_cell * f64::from(cells);
        assert_eq!(battery_charge(cells, volts(4.3)), 1.0);
        assert_eq!(battery_charge(cells, volts(4.2)), 1.0);
        assert_eq!(battery_charge(cells, volts(3.0)), 0.0);
        assert!(close(battery_charge(cells, volts(3.84)), 0.5));
        assert!(close(battery_charge(cells, volts(3.82)), 0.45));
        let low = battery_charge(cells, volts(config.low_battery_cell_v));
        assert!(low > 0.0 && low < 0.1, "{low}");
        assert!(low_battery(&config, cells, volts(3.4)));
        assert!(!low_battery(&config, cells, volts(3.8)));
        assert!(low_battery(&config, 4, 13.9) && !low_battery(&config, 3, 13.9));
    }

    #[test]
    fn the_imu_is_read_in_the_cars_frame_and_si_units() {
        let car = car();
        let imu = Imu {
            rpy_rad: [0.0; 3],
            // Nose up (the car's forward is the VESC's -y) and left side up
            // (the car's left is the VESC's +x), turning left (+z).
            accel_g: [0.5, -0.25, 1.0],
            gyro_deg_s: [0.0, 0.0, 90.0],
        };
        let reading = imu_reading(&car, 2.0 * car.motor.speed_to_erpm_gain, &imu);
        assert!(close(reading.wheel_speed_mps, 2.0));
        // A left turn, and the left side up: both negative, the right being
        // positive.
        assert!(close(reading.yaw_rate_rad_s, -std::f64::consts::FRAC_PI_2));
        assert!(close(reading.ax_mps2, 0.25 * G_MPS2));
        assert!(close(reading.ay_mps2, -0.5 * G_MPS2));
    }

    fn wanted(values: &[(&str, f64)], limits: &[(&str, f64)]) -> VescParameters {
        let map = |pairs: &[(&str, f64)]| {
            pairs
                .iter()
                .map(|&(name, value)| (name.to_string(), value))
                .collect()
        };
        VescParameters {
            values: map(values),
            limits: map(limits),
        }
    }

    #[test]
    fn every_numeric_setting_is_tunable_and_reported_but_the_steering_angle() {
        let config = config();
        let status = config.parameters_status();
        let value = |name: &str| {
            status
                .parameters
                .iter()
                .find(|parameter| parameter.name == name)
                .unwrap_or_else(|| panic!("{name} isn't tunable"))
                .value
        };
        assert_eq!(value("brake_current_a"), config.brake_current_a);
        assert_eq!(value("low_battery_cell_v"), config.low_battery_cell_v);
        let limits: Vec<&str> = status.limits.iter().map(|p| p.name.as_str()).collect();
        assert_eq!(limits.len(), ActuatorLimits::tunable_parameters().len() - 1);
        assert!(!limits.contains(&"max_steering_angle_rad"));
    }

    #[test]
    fn tuning_applies_settings_and_limits() {
        let (mut config, car) = (config(), car());
        let changed = config
            .apply(
                &wanted(&[("brake_current_a", 3.0)], &[("max_speed_mps", 0.8)]),
                &car,
            )
            .unwrap();
        assert!(changed);
        assert_eq!(config.brake_current_a, 3.0);
        assert_eq!(config.limits.max_speed_mps, 0.8);
        // Asking for what's already in effect changes nothing, nor does the
        // steering angle, which is the car's.
        let same = wanted(
            &[("brake_current_a", 3.0)],
            &[("max_steering_angle_rad", 0.1)],
        );
        assert!(!config.apply(&same, &car).unwrap());
    }

    #[test]
    fn tuning_to_an_invalid_config_is_refused_whole() {
        let (mut config, car) = (config(), car());
        let before = config.clone();
        let too_fast_to_stop = car.motor.min_speed_mps + 0.05;
        let refused = config.apply(
            &wanted(
                &[
                    ("brake_current_a", 3.0),
                    ("stop_speed_mps", too_fast_to_stop),
                ],
                &[],
            ),
            &car,
        );
        assert!(refused.is_err());
        assert_eq!(config, before);
    }

    #[test]
    fn saving_the_values_in_effect_keeps_the_config() {
        let original = std::fs::read_to_string(config_path()).unwrap();
        let copy = std::env::temp_dir().join(format!("aurorus_vesc_{}.toml", std::process::id()));
        std::fs::write(&copy, &original).unwrap();
        let status = config().parameters_status();
        for (section, parameters) in [(None, &status.parameters), (Some("limits"), &status.limits)]
        {
            let values: Vec<(&str, String)> = parameters
                .iter()
                .map(|parameter| {
                    (
                        parameter.name.as_str(),
                        crate::config::parameter_toml_value(parameter),
                    )
                })
                .collect();
            crate::config::save_toml_values(&copy, section, &values).unwrap();
        }
        // Only how a number is written may change (`0.50` -> `0.5`).
        let saved: VescConfig = crate::config::load(&copy).unwrap();
        assert_eq!(saved, config());
        std::fs::remove_file(copy).unwrap();
    }
}
