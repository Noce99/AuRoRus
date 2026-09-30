//! [`Vesc`]: drives the real car through its VESC - the counterpart of
//! [`crate::actuators::SimulatedVehicle`] when `web_gui` runs on a car.

use super::VescPort;
use super::control::{
    Motor, Setpoint, VescConfig, battery_charge, imu_reading, low_battery, motor, servo_position,
};
use super::protocol::Fault;
use crate::actuators::simulated_vehicle::select_command_within;
use crate::hardware::CarCalibration;
use crate::topics::{
    AUTONOMOUS_VESC_COMMAND_TOPIC_NAME, ActuatorLimits, HUMAN_VESC_COMMAND_TOPIC_NAME,
    IMU_TOPIC_NAME, ImuReading, VESC_PARAMETERS_STATUS_TOPIC_NAME, VESC_PARAMETERS_TOPIC_NAME,
    VESC_STATUS_TOPIC_NAME, VehicleGeometry, VehicleTopics, VescCommand, VescParameters,
    VescParametersStatus, VescStatus,
};
use crate::{Captain, Executor, RwLockTopic, Ticker};
use std::any::Any;
use std::path::Path;
use std::sync::Arc;
use std::time::{Duration, Instant};

/// How long [`Vesc`] brakes, at most, when it stops.
const STOP_BRAKING_FOR: Duration = Duration::from_secs(1);
/// Below this, in ERPM, the motor counts as stopped.
const STOPPED_ERPM: f64 = 100.0;
/// How often a wait before reconnecting checks whether to stop.
const POLL: Duration = Duration::from_millis(50);
/// The time constant the battery voltage is averaged over for estimating its
/// charge, in seconds - long enough to ride out the motor's bursts.
const BATTERY_SMOOTHING_S: f64 = 3.0;

/// The real car's actuators: every [`VescConfig::rate_hz`], sends the VESC the
/// command to act on - the human's over the autonomous one, as
/// [`crate::actuators::SimulatedVehicle`] picks it, stale after
/// [`VescConfig::command_timeout_s`] - moved towards within
/// [`VescConfig::actuator_limits`] and steered through the car's
/// [`crate::hardware::SteeringTable`], and publishes what the VESC reads: its IMU and
/// wheel speed on [`IMU_TOPIC_NAME`] (for dead reckoning), and its own state
/// on [`VESC_STATUS_TOPIC_NAME`]. Publishes the limits on
/// [`VehicleTopics::vehicle_limits`], and the car's size on
/// [`VehicleTopics::vehicle_geometry`].
///
/// Its [`VescConfig`] can be tuned live - the car's calibration can't: it
/// publishes the values in effect on
/// [`VESC_PARAMETERS_STATUS_TOPIC_NAME`] and applies what
/// [`VESC_PARAMETERS_TOPIC_NAME`] asks for (see [`VescConfig::apply`]),
/// republishing the limits when they change.
///
/// Brakes and steers straight whenever it stops. Losing the VESC is
/// logged and retried every [`VescConfig::reconnect_delay_s`], publishing
/// nothing meanwhile; the VESC's own timeout stops the motor.
pub struct Vesc {
    id: u16,
    name: String,
    config: VescConfig,
    car: CarCalibration,
    vehicle: VehicleTopics,
}

impl Vesc {
    /// Drives `car` as `config` says.
    ///
    /// # Panics
    ///
    /// Panics if `config` isn't valid for `car` (see
    /// [`VescConfig::validate`]) - caught at startup, before anything moves.
    pub fn new(name: impl Into<String>, config: VescConfig, car: CarCalibration) -> Self {
        let name = name.into();
        if let Err(err) = config.validate(&car) {
            panic!(
                "{name}: config/actuators/vesc.toml doesn't suit the car {:?}: {err}",
                car.name
            );
        }
        Self {
            id: 0,
            name,
            config,
            car,
            vehicle: VehicleTopics::ego(),
        }
    }

    /// One connection to the VESC, driving until told to stop (`Ok`) or the
    /// VESC is lost (`Err`) - either way, braked and centered as well as it
    /// still can be.
    fn drive(&self, captain: &Captain, tuning: &mut Tuning) -> Result<(), String> {
        let path = Path::new(&tuning.config.port);
        let mut port =
            VescPort::open(path, Duration::from_secs_f64(tuning.config.reply_timeout_s))?;
        let firmware = port.firmware()?;
        println!(
            "{}: VESC firmware {}.{:02} on {} at {}",
            self.name,
            firmware.major,
            firmware.minor,
            firmware.hardware,
            path.display()
        );
        let result = self.control(captain, &mut port, tuning);
        self.stop(&mut port, &tuning.config);
        result
    }

    /// Sends the command and publishes the readings every tick, until told
    /// to stop or the VESC fails to answer.
    fn control(
        &self,
        captain: &Captain,
        port: &mut VescPort,
        tuning: &mut Tuning,
    ) -> Result<(), String> {
        let autonomous_topic = captain.topic::<VescCommand>(AUTONOMOUS_VESC_COMMAND_TOPIC_NAME);
        let human_topic = captain.topic::<VescCommand>(HUMAN_VESC_COMMAND_TOPIC_NAME);
        let imu_topic = captain.topic::<ImuReading>(IMU_TOPIC_NAME);
        let status_topic = captain.topic::<VescStatus>(VESC_STATUS_TOPIC_NAME);
        let mut rate_hz = tuning.config.rate_hz;
        let mut ticker = Ticker::new(rate_hz);
        let mut setpoint = Setpoint::default();
        let mut fault = Fault(0);
        let mut battery_v: Option<f64> = None;

        while captain.is_running(self.id) {
            tuning.update(self.id);
            let config = &tuning.config;
            let car = &self.car;
            if config.rate_hz != rate_hz {
                rate_hz = config.rate_hz;
                ticker = Ticker::new(rate_hz);
            }
            let dt_s = 1.0 / rate_hz;
            let command = select_command_within(
                autonomous_topic.read(),
                human_topic.read(),
                Duration::from_secs_f64(config.command_timeout_s),
            );
            setpoint = setpoint.toward(command, &config.actuator_limits(car), dt_s);
            let servo = servo_position(car, setpoint.steering_rad);
            let motor = motor(config, car, setpoint.speed_mps);
            port.set_servo(servo)?;
            match motor {
                Motor::Erpm(erpm) => port.set_rpm(erpm)?,
                Motor::Brake(amps) => port.brake(amps)?,
            }
            let values = port.values()?;
            let imu = port.imu()?;

            if values.fault != fault {
                fault = values.fault;
                eprintln!("{}: VESC fault {}", self.name, fault.name());
            }
            let smoothed_v = match battery_v {
                Some(v) => v + (values.input_voltage_v - v) * dt_s / (BATTERY_SMOOTHING_S + dt_s),
                None => values.input_voltage_v,
            };
            battery_v = Some(smoothed_v);
            imu_topic
                .write(self.id, imu_reading(car, values.erpm, &imu))
                .expect("lost writer authorization for the imu topic");
            status_topic
                .write(
                    self.id,
                    VescStatus {
                        input_voltage_v: values.input_voltage_v,
                        low_battery: low_battery(config, car.battery.cells, values.input_voltage_v),
                        battery_charge: battery_charge(car.battery.cells, smoothed_v),
                        input_current_a: values.input_current_a,
                        motor_current_a: values.motor_current_a,
                        temp_fet_c: values.temp_fet_c,
                        temp_motor_c: values.temp_motor_c,
                        erpm: values.erpm,
                        wheel_speed_mps: values.erpm / car.motor.speed_to_erpm_gain,
                        tachometer: values.tachometer,
                        fault: fault.name().to_string(),
                        timed_out: values.timed_out,
                        servo_position: servo,
                        commanded_erpm: match motor {
                            Motor::Erpm(erpm) => Some(erpm),
                            Motor::Brake(_) => None,
                        },
                    },
                )
                .expect("lost writer authorization for the vesc_status topic");

            ticker.wait();
        }
        Ok(())
    }

    /// Brakes until the motor stops (or [`STOP_BRAKING_FOR`] passes),
    /// releases it and steers straight - best effort, since the VESC
    /// may already be gone.
    fn stop(&self, port: &mut VescPort, config: &VescConfig) {
        let braking = Instant::now();
        while braking.elapsed() < STOP_BRAKING_FOR {
            if port.brake(config.brake_current_a).is_err() {
                return;
            }
            match port.values() {
                Ok(values) if values.erpm.abs() < STOPPED_ERPM => break,
                Ok(_) => std::thread::sleep(Duration::from_secs_f64(1.0 / config.rate_hz)),
                Err(_) => return,
            }
        }
        let _ = port.release();
        let _ = port.set_servo(self.car.steering.straight_servo());
    }
}

/// The config [`Vesc`] drives with, tuned live - kept across
/// reconnects, so losing the VESC doesn't undo any tuning.
struct Tuning {
    config: VescConfig,
    /// `write_count` of the last [`VescParameters`] looked at, so an
    /// unchanged request costs one counter read per tick.
    seen_write_count: u64,
    /// `None` when nothing (e.g. no `web_gui`) tunes it.
    wanted: Option<Arc<RwLockTopic<VescParameters>>>,
    status: Arc<RwLockTopic<VescParametersStatus>>,
    limits: Arc<RwLockTopic<ActuatorLimits>>,
    /// The car, which `config` must suit.
    car: CarCalibration,
    name: String,
}

impl Tuning {
    /// Applies a new [`VescParameters`] request, if there is one, and
    /// publishes the result - refused whole, and logged, if it would leave an
    /// invalid config (see [`VescConfig::validate`]); the status is
    /// republished anyway, so a slider moved to a refused value snaps back.
    fn update(&mut self, id: u16) {
        let Some(wanted) = &self.wanted else {
            return;
        };
        if wanted.meta().write_count == self.seen_write_count {
            return;
        }
        let request = wanted.read();
        self.seen_write_count = request.meta.write_count;
        let limits_before = self.config.limits;
        if let Err(err) = self.config.apply(&request.value, &self.car) {
            eprintln!("{}: tuning refused - {err}", self.name);
        }
        if self.config.limits != limits_before {
            self.limits
                .write(id, self.config.actuator_limits(&self.car))
                .expect("lost writer authorization for the vehicle_limits topic");
        }
        self.status
            .write(id, self.config.parameters_status())
            .expect("lost writer authorization for the vesc_parameters_status topic");
    }
}

impl Executor for Vesc {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_writer::<ImuReading>(IMU_TOPIC_NAME, self.id, ImuReading::default);
        captain.claim_writer::<VescStatus>(VESC_STATUS_TOPIC_NAME, self.id, VescStatus::default);
        let limits: ActuatorLimits = self.config.actuator_limits(&self.car);
        captain.claim_writer::<ActuatorLimits>(
            &self.vehicle.vehicle_limits(),
            self.id,
            move || limits,
        );
        let geometry = self.car.vehicle_geometry();
        captain.claim_writer::<VehicleGeometry>(
            &self.vehicle.vehicle_geometry(),
            self.id,
            move || geometry,
        );
        let parameters = self.config.parameters_status();
        captain.claim_writer::<VescParametersStatus>(
            VESC_PARAMETERS_STATUS_TOPIC_NAME,
            self.id,
            move || parameters.clone(),
        );
    }

    fn run(&mut self, captain: &Captain) {
        // Tuned live, so kept apart from `self.config`: a restart rereads the
        // file (see `fresh`) rather than keeping unsaved values.
        let mut tuning = Tuning {
            config: self.config.clone(),
            seen_write_count: 0,
            wanted: captain.try_topic::<VescParameters>(VESC_PARAMETERS_TOPIC_NAME),
            status: captain.topic::<VescParametersStatus>(VESC_PARAMETERS_STATUS_TOPIC_NAME),
            limits: captain.topic::<ActuatorLimits>(&self.vehicle.vehicle_limits()),
            car: self.car.clone(),
            name: self.name.clone(),
        };
        while captain.is_running(self.id) {
            tuning.update(self.id);
            let reconnect_delay = Duration::from_secs_f64(tuning.config.reconnect_delay_s);
            match self.drive(captain, &mut tuning) {
                Ok(()) => break,
                Err(err) => eprintln!(
                    "{}: {err} - retrying in {:.1} s",
                    self.name,
                    reconnect_delay.as_secs_f64()
                ),
            }
            let retry_at = Instant::now() + reconnect_delay;
            while captain.is_running(self.id) && Instant::now() < retry_at {
                tuning.update(self.id);
                std::thread::sleep(POLL);
            }
        }
    }

    fn name(&self) -> String {
        self.name.clone()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn fresh(&self) -> Box<dyn Executor> {
        Box::new(Vesc::new(
            self.name.clone(),
            self.config.clone(),
            self.car.clone(),
        ))
    }
}
