//! What [`super::Vesc`] sends the VESC for a [`VescCommand`], and how it
//! reads the VESC's IMU - the car's calibration, with no I/O, so it can be
//! tested without the car.

use super::protocol::Imu;
use crate::topics::{ActuatorLimits, ImuReading, VescCommand};

/// A LiPo cell's resting voltage at each tenth of its charge, `0.0` (empty)
/// to `1.0` (full).
const LIPO_CELL_V: [f64; 11] = [
    3.30, 3.69, 3.73, 3.77, 3.80, 3.84, 3.87, 3.95, 4.02, 4.11, 4.20,
];

/// Standard gravity, in meters/second^2 - the VESC reports accelerations in g.
const G_MPS2: f64 = 9.806_65;

/// Every parameter [`super::Vesc`] needs - the car's calibration, loaded from
/// `config/actuators/vesc.toml` (see [`Default`]) or from an arbitrary path
/// via [`crate::config::load`].
#[derive(Debug, Clone, PartialEq, serde::Deserialize)]
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

    /// The servo position that points the wheels straight ahead, in `0..=1`.
    pub servo_offset: f64,
    /// Servo units per radian of steering: `servo = servo_offset +
    /// steering_gain * angle`. A positive angle steers right, as
    /// [`VescCommand::servo_position_rad`] does in the simulator, so this is
    /// positive when a larger servo position steers right.
    pub steering_gain: f64,
    /// The servo positions never exceeded, whatever the angle: just short of
    /// the steering's end stops.
    pub servo_min: f64,
    pub servo_max: f64,

    /// Motor ERPM per meter/second of speed: `erpm = speed *
    /// speed_to_erpm_gain`, also how measured ERPM becomes wheel speed.
    pub speed_to_erpm_gain: f64,
    /// Multiplies every commanded ERPM, making up for the VESC's speed
    /// controller settling short of it. `1.0` leaves it as is.
    pub speed_compensation: f64,
    /// The slowest speed the motor holds smoothly, in meters/second: slower
    /// nonzero speeds are raised to it.
    pub min_speed_mps: f64,
    /// Speeds below this, in meters/second, brake instead.
    pub stop_speed_mps: f64,
    /// The current the motor brakes with, in amperes.
    pub brake_current_a: f64,
    /// Below this battery voltage, in volts, the battery should be
    /// recharged - flagged in [`crate::topics::VescStatus::low_battery`].
    pub low_battery_v: f64,
    /// The battery's cells in series - see [`battery_charge`].
    pub battery_cells: u32,

    /// Which of the VESC's IMU axes gives [`ImuReading`]'s x (forward), y and
    /// z - the simulator's frame, whose y is the car's right as the GUI draws
    /// it (y down), so a right turn is a positive yaw rate.
    pub imu_x: ImuAxis,
    pub imu_y: ImuAxis,
    pub imu_z: ImuAxis,

    /// The limits the car is driven within, published for the autonomous
    /// algorithms - `max_speed_mps` is a hard cap on every command.
    pub limits: ActuatorLimits,
}

impl Default for VescConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../../config/actuators/vesc.toml"))
            .expect("config/actuators/vesc.toml must deserialize into VescConfig")
    }
}

/// One of the IMU's axes, possibly reversed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Deserialize)]
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
        match self {
            Self::PlusX => v[0],
            Self::MinusX => -v[0],
            Self::PlusY => v[1],
            Self::MinusY => -v[1],
            Self::PlusZ => v[2],
            Self::MinusZ => -v[2],
        }
    }
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
    /// `limits` allow - the same limits [`crate::actuators::SimulatedVehicle`]
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

/// The servo position steering the wheels `steering_rad`, never past
/// `servo_min`/`servo_max`.
pub fn servo_position(config: &VescConfig, steering_rad: f64) -> f64 {
    (config.servo_offset + config.steering_gain * steering_rad)
        .clamp(config.servo_min, config.servo_max)
}

/// What the motor is told for `speed_mps`: a brake below
/// `stop_speed_mps`, and otherwise at least `min_speed_mps`, never past
/// `max_speed_mps`, compensated.
pub fn motor(config: &VescConfig, speed_mps: f64) -> Motor {
    let magnitude = speed_mps.abs();
    if magnitude < config.stop_speed_mps || !speed_mps.is_finite() {
        return Motor::Brake(config.brake_current_a);
    }
    let magnitude = magnitude
        .max(config.min_speed_mps)
        .min(config.limits.max_speed_mps);
    let erpm =
        speed_mps.signum() * magnitude * config.speed_to_erpm_gain * config.speed_compensation;
    Motor::Erpm(erpm.round() as i32)
}

/// The battery's estimated charge, `0.0` (empty) to `1.0` (full), from its
/// voltage - interpolated along a LiPo cell's discharge curve. Only a
/// resting battery's voltage says it well: under load it sags, reading
/// emptier than it is.
pub fn battery_charge(config: &VescConfig, voltage_v: f64) -> f64 {
    let cell_v = voltage_v / f64::from(config.battery_cells.max(1));
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
pub fn imu_reading(config: &VescConfig, erpm: f64, imu: &Imu) -> ImuReading {
    ImuReading {
        wheel_speed_mps: erpm / config.speed_to_erpm_gain,
        yaw_rate_rad_s: config.imu_z.of(imu.gyro_deg_s).to_radians(),
        ax_mps2: config.imu_x.of(imu.accel_g) * G_MPS2,
        ay_mps2: config.imu_y.of(imu.accel_g) * G_MPS2,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> VescConfig {
        VescConfig::default()
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn the_calibrated_config_loads() {
        let config = config();
        assert_eq!(config.port, "/dev/sensors/vesc");
        assert!(config.limits.validate().is_ok());
        assert_eq!(config.limits.max_speed_mps, 1.0);
    }

    #[test]
    fn straight_is_the_offset_and_right_raises_the_servo() {
        let config = config();
        assert_eq!(servo_position(&config, 0.0), config.servo_offset);
        // Positive steers right, which this car's servo does above its offset.
        assert!(servo_position(&config, 0.2) > config.servo_offset);
        assert!(servo_position(&config, -0.2) < config.servo_offset);
    }

    #[test]
    fn the_servo_never_passes_its_end_stops() {
        let config = config();
        assert_eq!(servo_position(&config, 10.0), config.servo_max);
        assert_eq!(servo_position(&config, -10.0), config.servo_min);
    }

    #[test]
    fn slow_speeds_brake_and_creeping_ones_are_raised_to_the_minimum() {
        let config = config();
        assert_eq!(motor(&config, 0.0), Motor::Brake(config.brake_current_a));
        assert_eq!(motor(&config, 0.01), Motor::Brake(config.brake_current_a));
        assert_eq!(
            motor(&config, f64::NAN),
            Motor::Brake(config.brake_current_a)
        );
        let minimum = motor(&config, config.min_speed_mps);
        assert_eq!(motor(&config, 0.1), minimum);
        let Motor::Erpm(reverse) = motor(&config, -0.1) else {
            panic!("reverse should drive");
        };
        assert_eq!(Motor::Erpm(-reverse), minimum);
    }

    #[test]
    fn erpm_is_compensated_and_capped_at_the_maximum_speed() {
        let config = config();
        let expected = 0.8 * config.speed_to_erpm_gain * config.speed_compensation;
        assert_eq!(motor(&config, 0.8), Motor::Erpm(expected.round() as i32));
        assert_eq!(
            motor(&config, 50.0),
            motor(&config, config.limits.max_speed_mps)
        );
    }

    #[test]
    fn the_setpoint_follows_the_command_within_the_limits() {
        let limits = config().limits;
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
        let limits = config().limits;
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
        let cells = f64::from(config.battery_cells);
        assert_eq!(battery_charge(&config, 4.3 * cells), 1.0);
        assert_eq!(battery_charge(&config, 4.2 * cells), 1.0);
        assert_eq!(battery_charge(&config, 3.0 * cells), 0.0);
        assert!(close(battery_charge(&config, 3.84 * cells), 0.5));
        assert!(close(battery_charge(&config, 3.82 * cells), 0.45));
        let low = battery_charge(&config, config.low_battery_v);
        assert!(low > 0.0 && low < 0.1, "{low}");
    }

    #[test]
    fn the_imu_is_read_in_the_cars_frame_and_si_units() {
        let config = config();
        let imu = Imu {
            rpy_rad: [0.0; 3],
            // Nose up (the car's forward is the VESC's -y) and left side up
            // (the car's left is the VESC's +x), turning left (+z).
            accel_g: [0.5, -0.25, 1.0],
            gyro_deg_s: [0.0, 0.0, 90.0],
        };
        let reading = imu_reading(&config, 2.0 * config.speed_to_erpm_gain, &imu);
        assert!(close(reading.wheel_speed_mps, 2.0));
        // A left turn, and the left side up: both negative, the right being
        // positive.
        assert!(close(reading.yaw_rate_rad_s, -std::f64::consts::FRAC_PI_2));
        assert!(close(reading.ax_mps2, 0.25 * G_MPS2));
        assert!(close(reading.ay_mps2, -0.5 * G_MPS2));
    }
}
