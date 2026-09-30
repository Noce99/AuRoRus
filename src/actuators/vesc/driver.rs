//! [`Vesc`]: drives the real car through its VESC - the counterpart of
//! [`crate::actuators::SimulatedVehicle`] under `web_gui --hardware`.

use super::VescPort;
use super::control::{Motor, Setpoint, VescConfig, imu_reading, motor, servo_position};
use super::protocol::Fault;
use crate::actuators::simulated_vehicle::select_command_within;
use crate::topics::{
    AUTONOMOUS_VESC_COMMAND_TOPIC_NAME, ActuatorLimits, HUMAN_VESC_COMMAND_TOPIC_NAME,
    IMU_TOPIC_NAME, ImuReading, VESC_STATUS_TOPIC_NAME, VehicleTopics, VescCommand, VescStatus,
};
use crate::{Captain, Executor, Ticker};
use std::any::Any;
use std::path::Path;
use std::time::{Duration, Instant};

/// How long [`Vesc`] brakes, at most, when it stops.
const STOP_BRAKING_FOR: Duration = Duration::from_secs(1);
/// Below this, in ERPM, the motor counts as stopped.
const STOPPED_ERPM: f64 = 100.0;
/// How often a wait before reconnecting checks whether to stop.
const POLL: Duration = Duration::from_millis(50);

/// The real car's actuators: every [`VescConfig::rate_hz`], sends the VESC the
/// command to act on - the human's over the autonomous one, as
/// [`crate::actuators::SimulatedVehicle`] picks it, stale after
/// [`VescConfig::command_timeout_s`] - moved towards within
/// [`VescConfig::limits`], and publishes what the VESC reads: its IMU and
/// wheel speed on [`IMU_TOPIC_NAME`] (for dead reckoning), and its own state
/// on [`VESC_STATUS_TOPIC_NAME`]. Publishes the limits on
/// [`VehicleTopics::vehicle_limits`].
///
/// Brakes and centers the steering whenever it stops. Losing the VESC is
/// logged and retried every [`VescConfig::reconnect_delay_s`], publishing
/// nothing meanwhile; the VESC's own timeout stops the motor.
pub struct Vesc {
    id: u16,
    name: String,
    config: VescConfig,
    vehicle: VehicleTopics,
}

impl Vesc {
    pub fn new(name: impl Into<String>, config: VescConfig) -> Self {
        Self {
            id: 0,
            name: name.into(),
            config,
            vehicle: VehicleTopics::ego(),
        }
    }

    /// One connection to the VESC, driving until told to stop (`Ok`) or the
    /// VESC is lost (`Err`) - either way, braked and centered as well as it
    /// still can be.
    fn drive(&self, captain: &Captain) -> Result<(), String> {
        let path = Path::new(&self.config.port);
        let mut port =
            VescPort::open(path, Duration::from_secs_f64(self.config.reply_timeout_s))?;
        let firmware = port.firmware()?;
        println!(
            "{}: VESC firmware {}.{:02} on {} at {}",
            self.name,
            firmware.major,
            firmware.minor,
            firmware.hardware,
            path.display()
        );
        let result = self.control(captain, &mut port);
        self.stop(&mut port);
        result
    }

    /// Sends the command and publishes the readings every tick, until told
    /// to stop or the VESC fails to answer.
    fn control(&self, captain: &Captain, port: &mut VescPort) -> Result<(), String> {
        let autonomous_topic = captain.topic::<VescCommand>(AUTONOMOUS_VESC_COMMAND_TOPIC_NAME);
        let human_topic = captain.topic::<VescCommand>(HUMAN_VESC_COMMAND_TOPIC_NAME);
        let imu_topic = captain.topic::<ImuReading>(IMU_TOPIC_NAME);
        let status_topic = captain.topic::<VescStatus>(VESC_STATUS_TOPIC_NAME);
        let command_timeout = Duration::from_secs_f64(self.config.command_timeout_s);
        let dt_s = 1.0 / self.config.rate_hz;
        let mut ticker = Ticker::new(self.config.rate_hz);
        let mut setpoint = Setpoint::default();
        let mut fault = Fault(0);

        while captain.is_running(self.id) {
            let command =
                select_command_within(autonomous_topic.read(), human_topic.read(), command_timeout);
            setpoint = setpoint.toward(command, &self.config.limits, dt_s);
            let servo = servo_position(&self.config, setpoint.steering_rad);
            let motor = motor(&self.config, setpoint.speed_mps);
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
            imu_topic
                .write(self.id, imu_reading(&self.config, values.erpm, &imu))
                .expect("lost writer authorization for the imu topic");
            status_topic
                .write(
                    self.id,
                    VescStatus {
                        input_voltage_v: values.input_voltage_v,
                        low_battery: values.input_voltage_v < self.config.low_battery_v,
                        input_current_a: values.input_current_a,
                        motor_current_a: values.motor_current_a,
                        temp_fet_c: values.temp_fet_c,
                        temp_motor_c: values.temp_motor_c,
                        erpm: values.erpm,
                        wheel_speed_mps: values.erpm / self.config.speed_to_erpm_gain,
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
    /// releases it and centers the steering - best effort, since the VESC
    /// may already be gone.
    fn stop(&self, port: &mut VescPort) {
        let braking = Instant::now();
        while braking.elapsed() < STOP_BRAKING_FOR {
            if port.brake(self.config.brake_current_a).is_err() {
                return;
            }
            match port.values() {
                Ok(values) if values.erpm.abs() < STOPPED_ERPM => break,
                Ok(_) => std::thread::sleep(Duration::from_secs_f64(1.0 / self.config.rate_hz)),
                Err(_) => return,
            }
        }
        let _ = port.release();
        let _ = port.set_servo(self.config.servo_offset);
    }
}

impl Executor for Vesc {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_writer::<ImuReading>(IMU_TOPIC_NAME, self.id, ImuReading::default);
        captain.claim_writer::<VescStatus>(VESC_STATUS_TOPIC_NAME, self.id, VescStatus::default);
        let limits: ActuatorLimits = self.config.limits;
        captain.claim_writer::<ActuatorLimits>(
            &self.vehicle.vehicle_limits(),
            self.id,
            move || limits,
        );
    }

    fn run(&mut self, captain: &Captain) {
        let reconnect_delay = Duration::from_secs_f64(self.config.reconnect_delay_s);
        while captain.is_running(self.id) {
            match self.drive(captain) {
                Ok(()) => break,
                Err(err) => eprintln!(
                    "{}: {err} - retrying in {:.1} s",
                    self.name,
                    reconnect_delay.as_secs_f64()
                ),
            }
            let retry_at = Instant::now() + reconnect_delay;
            while captain.is_running(self.id) && Instant::now() < retry_at {
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
        Box::new(Vesc::new(self.name.clone(), self.config.clone()))
    }
}
