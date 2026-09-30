//! The VESC, driven for calibration by a thread of its own ([`Bench::start`])
//! that the web handlers talk to through [`BenchState`].
//!
//! Safety: the motor only turns while the web page keeps asking it to - hold
//! to run. Every motor request ([`Bench::hold`]) keeps it going for
//! [`HOLD_FOR`] only; without a new one it brakes. The servo only moves
//! when asked ([`Bench::set_servo`]), and stays where it was put. The VESC's
//! own timeout stops the motor if this program dies.

use crate::analysis::{RampStep, max_std, mean};
use aurorus::actuators::VescConfig;
use aurorus::actuators::vesc::VescPort;
use aurorus::actuators::vesc::protocol::{Firmware, Imu, Values};
use std::collections::VecDeque;
use std::path::Path;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

/// How long one hold-to-run request keeps the motor going.
pub const HOLD_FOR: Duration = Duration::from_millis(300);
/// The fastest the motor is ever commanded here, in ERPM - the wheels are
/// off the ground.
pub const MAX_ERPM: i32 = 6000;
/// The ramp's commanded ERPMs, in order.
pub const RAMP_ERPM: [i32; 9] = [600, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 5000];
/// How long each ramp step lasts, and how much of its end is measured.
const RAMP_STEP: Duration = Duration::from_millis(1800);
const RAMP_MEASURED: Duration = Duration::from_millis(800);
/// How often the VESC is commanded and read.
const PERIOD: Duration = Duration::from_millis(20);
/// How much IMU history is kept, for the captures.
const IMU_HISTORY: Duration = Duration::from_secs(5);
/// How long the motor brakes after running, at most.
const BRAKE_FOR: Duration = Duration::from_secs(1);
/// Below this, in ERPM, the motor counts as stopped.
const STOPPED_ERPM: f64 = 100.0;

/// What the motor is asked to do.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum MotorRequest {
    /// Hold this ERPM.
    Spin { erpm: i32 },
    /// Run the ramp - see [`RAMP_ERPM`].
    Ramp,
}

/// The ramp's progress.
#[derive(Debug, Clone, Default, serde::Serialize)]
pub struct RampRun {
    /// The steps measured so far.
    pub steps: Vec<RampStep>,
    /// Whether every step was measured.
    pub finished: bool,
    /// Why it stopped early, if it did.
    pub aborted: Option<String>,
}

/// Everything the VESC thread and the web handlers share.
#[derive(Default)]
pub struct BenchState {
    pub connected: bool,
    /// The last connection problem, while there is one.
    pub error: Option<String>,
    pub firmware: Option<Firmware>,
    pub values: Option<Values>,
    /// The latest IMU readings, oldest first, with when they were read.
    pub imu: VecDeque<(Instant, Imu)>,
    /// The servo position last sent, if any was.
    pub servo: Option<f64>,
    /// A servo position waiting to be sent.
    servo_request: Option<f64>,
    /// What the motor is asked to do, and until when.
    motor: Option<(MotorRequest, Instant)>,
    /// The ramp's progress, since it was last started.
    pub ramp: Option<RampRun>,
    /// The tachometer when the wheel-turn counter was zeroed.
    pub counter_zero: Option<i32>,
}

impl BenchState {
    /// The accelerometer's (in g) and gyroscope's (in deg/s) means over the
    /// last `window`, and how much they wandered - `None` without enough
    /// readings.
    pub fn imu_over(&self, window: Duration) -> Option<ImuCapture> {
        let since = Instant::now().checked_sub(window)?;
        let recent: Vec<&Imu> = self
            .imu
            .iter()
            .filter(|(at, _)| *at >= since)
            .map(|(_, imu)| imu)
            .collect();
        if recent.len() < 10 {
            return None;
        }
        let accel: Vec<[f64; 3]> = recent.iter().map(|imu| imu.accel_g).collect();
        let gyro: Vec<[f64; 3]> = recent.iter().map(|imu| imu.gyro_deg_s).collect();
        Some(ImuCapture {
            accel_g: mean(&accel),
            accel_std_g: max_std(&accel),
            gyro_deg_s: mean(&gyro),
            gyro_std_deg_s: max_std(&gyro),
        })
    }

    /// The tachometer's steps since the counter was zeroed.
    pub fn counted_steps(&self) -> Option<i64> {
        let now = self.values.as_ref()?.tachometer;
        Some(i64::from(now) - i64::from(self.counter_zero?))
    }

    /// Whether the motor is being driven.
    pub fn motor_running(&self) -> bool {
        self.motor.is_some()
    }
}

/// The IMU's readings averaged over a moment - see [`BenchState::imu_over`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct ImuCapture {
    pub accel_g: [f64; 3],
    pub accel_std_g: f64,
    pub gyro_deg_s: [f64; 3],
    pub gyro_std_deg_s: f64,
}

impl ImuCapture {
    /// Whether the car was still: the readings barely moved.
    pub fn still(&self) -> bool {
        self.accel_std_g < 0.03 && self.gyro_std_deg_s < 2.0
    }
}

/// The handle the web handlers drive the VESC through.
#[derive(Clone)]
pub struct Bench {
    state: Arc<Mutex<BenchState>>,
}

impl Bench {
    /// Starts the thread driving the VESC `config` names, reconnecting to it
    /// whenever it's lost.
    pub fn start(config: VescConfig) -> Self {
        let bench = Self {
            state: Arc::new(Mutex::new(BenchState::default())),
        };
        let thread = bench.clone();
        std::thread::spawn(move || thread.run(&config));
        bench
    }

    pub fn state(&self) -> MutexGuard<'_, BenchState> {
        self.state
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    /// Moves the servo to `position`, clamped to `0..=1`.
    pub fn set_servo(&self, position: f64) {
        self.state().servo_request = Some(position.clamp(0.0, 1.0));
    }

    /// Keeps the motor doing `request` for [`HOLD_FOR`] more. Only a
    /// `start` (a fresh press of a hold-to-run button) starts it: once it
    /// stopped - released, timed out, or the ramp done - holding on doesn't
    /// start it again. Refused while the VESC is away.
    pub fn hold(&self, request: MotorRequest, start: bool) -> Result<(), String> {
        let mut state = self.state();
        if !state.connected {
            return Err("the VESC isn't connected".to_string());
        }
        if let MotorRequest::Spin { erpm } = request
            && !(1..=MAX_ERPM).contains(&erpm.abs())
        {
            return Err(format!("spin between 1 and {MAX_ERPM} ERPM"));
        }
        let running = state.motor.is_some_and(|(current, _)| current == request);
        if !running && !start {
            return Err("stopped - press again to start".to_string());
        }
        if start && request == MotorRequest::Ramp {
            state.ramp = Some(RampRun::default());
        }
        state.motor = Some((request, Instant::now() + HOLD_FOR));
        Ok(())
    }

    /// Stops the motor now.
    pub fn stop(&self) {
        let mut state = self.state();
        state.motor = None;
        if let Some(ramp) = state.ramp.as_mut()
            && !ramp.finished
            && ramp.aborted.is_none()
        {
            ramp.aborted = Some("stopped".to_string());
        }
    }

    /// Zeroes the wheel-turn counter at the tachometer's current count.
    pub fn zero_counter(&self) -> Result<(), String> {
        let mut state = self.state();
        let tachometer = state
            .values
            .as_ref()
            .ok_or("no reading from the VESC yet")?
            .tachometer;
        state.counter_zero = Some(tachometer);
        Ok(())
    }

    /// Connects to the VESC and drives it, forever - reconnecting after a
    /// second whenever it's lost.
    fn run(&self, config: &VescConfig) {
        loop {
            if let Err(err) = self.drive(config) {
                let mut state = self.state();
                state.connected = false;
                state.error = Some(err);
                state.motor = None;
                if let Some(ramp) = state.ramp.as_mut()
                    && !ramp.finished
                {
                    ramp.aborted
                        .get_or_insert_with(|| "the VESC was lost".to_string());
                }
            }
            std::thread::sleep(Duration::from_secs(1));
        }
    }

    /// One connection: every [`PERIOD`], sends what's asked and reads the
    /// VESC, until it fails to answer.
    fn drive(&self, config: &VescConfig) -> Result<(), String> {
        let mut port = VescPort::open(
            Path::new(&config.port),
            Duration::from_secs_f64(config.reply_timeout_s),
        )?;
        let firmware = port.firmware()?;
        {
            let mut state = self.state();
            state.connected = true;
            state.error = None;
            state.firmware = Some(firmware);
        }
        // When the motor last ran, to brake it after.
        let mut ran_at: Option<Instant> = None;
        // The ramp step running, since when, and its settled readings.
        let mut ramp_step: Option<(usize, Instant, Vec<f64>)> = None;

        loop {
            let started = Instant::now();
            let (servo, motor) = {
                let mut state = self.state();
                if let Some((request, until)) = state.motor
                    && started > until
                {
                    state.motor = None;
                    if request == MotorRequest::Ramp
                        && let Some(ramp) = state.ramp.as_mut()
                        && !ramp.finished
                    {
                        ramp.aborted.get_or_insert_with(|| "released".to_string());
                    }
                }
                (
                    state.servo_request.take(),
                    state.motor.map(|(request, _)| request),
                )
            };
            if motor != Some(MotorRequest::Ramp) {
                ramp_step = None;
            }

            if let Some(position) = servo {
                port.set_servo(position)?;
                self.state().servo = Some(position);
            }
            match motor {
                Some(MotorRequest::Spin { erpm }) => {
                    port.set_rpm(erpm.clamp(-MAX_ERPM, MAX_ERPM))?;
                    ran_at = Some(started);
                }
                Some(MotorRequest::Ramp) => {
                    let (index, ..) = ramp_step.get_or_insert_with(|| (0, started, Vec::new()));
                    port.set_rpm(RAMP_ERPM[*index])?;
                    ran_at = Some(started);
                }
                None => {
                    if let Some(at) = ran_at {
                        port.brake(config.brake_current_a)?;
                        let stopped = self
                            .state()
                            .values
                            .as_ref()
                            .is_some_and(|values| values.erpm.abs() < STOPPED_ERPM);
                        if stopped || at.elapsed() > BRAKE_FOR {
                            port.release()?;
                            ran_at = None;
                        }
                    }
                }
            }

            let values = port.values()?;
            let imu = port.imu()?;
            let now = Instant::now();
            let mut state = self.state();
            if let Some((index, since, settled)) = ramp_step.as_mut() {
                let elapsed = now - *since;
                if elapsed > RAMP_STEP - RAMP_MEASURED {
                    settled.push(values.erpm);
                }
                if elapsed >= RAMP_STEP {
                    let measured = settled.iter().sum::<f64>() / settled.len().max(1) as f64;
                    let std = (settled.iter().map(|e| (e - measured).powi(2)).sum::<f64>()
                        / settled.len().max(1) as f64)
                        .sqrt();
                    let ramp = state.ramp.get_or_insert_with(RampRun::default);
                    ramp.steps.push(RampStep {
                        commanded_erpm: f64::from(RAMP_ERPM[*index]),
                        measured_erpm: measured,
                        measured_std_erpm: std,
                    });
                    if *index + 1 == RAMP_ERPM.len() {
                        ramp.finished = true;
                        state.motor = None;
                        ramp_step = None;
                    } else {
                        *index += 1;
                        *since = now;
                        settled.clear();
                    }
                }
            }
            state.values = Some(values);
            state.imu.push_back((now, imu));
            while state
                .imu
                .front()
                .is_some_and(|(at, _)| now - *at > IMU_HISTORY)
            {
                state.imu.pop_front();
            }
            drop(state);

            if let Some(rest) = PERIOD.checked_sub(started.elapsed()) {
                std::thread::sleep(rest);
            }
        }
    }
}
