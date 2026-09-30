//! The VESC, driven for calibration by a thread of its own ([`Bench::start`])
//! that the web handlers talk to through [`BenchState`].
//!
//! Safety: the motor only turns while the web page keeps asking it to - hold
//! to run. Every motor request ([`Bench::hold`]) keeps it going for
//! [`HOLD_FOR`] only; without a new one it brakes. The servo only moves
//! when asked ([`Bench::set_servo`]), and stays where it was put. The VESC's
//! own timeout stops the motor if this program dies.
//!
//! On the floor ([`MotorRequest::Drive`]) the car also stops by itself: when
//! the lidar sees anything within the stop distance ahead, when the lidar
//! goes quiet, once it has driven its distance, or after [`DRIVE_FOR`].

use crate::analysis::{DriveSample, RampStep, distance_ahead_m, max_std, mean, nearest_ahead_m};
use aurorus::RwLockTopic;
use aurorus::actuators::VescConfig;
use aurorus::actuators::vesc::VescPort;
use aurorus::actuators::vesc::protocol::{Firmware, Imu, Values};
use aurorus::topics::LidarScan;
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
pub const STOPPED_ERPM: f64 = 100.0;
/// The longest a floor drive lasts.
const DRIVE_FOR: Duration = Duration::from_secs(20);
/// A lidar scan older than this, while driving on the floor, stops the car.
const LIDAR_STALE: Duration = Duration::from_millis(300);
/// How far either side of straight ahead the lidar guards, in radians.
const GUARD_HALF_WIDTH_RAD: f32 = 35.0 * std::f32::consts::PI / 180.0;
/// How far either side of straight ahead the distance to a wall is read,
/// in radians.
const WALL_HALF_WIDTH_RAD: f32 = 2.0 * std::f32::consts::PI / 180.0;

/// What the motor is asked to do.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotorRequest {
    /// Hold this ERPM - wheels off the ground.
    Spin { erpm: i32 },
    /// Run the ramp - see [`RAMP_ERPM`] - wheels off the ground.
    Ramp,
    /// Drive on the floor: the servo at `servo`, the motor at `erpm`, for
    /// `max_steps` of the tachometer at most, stopping `stop_m` from
    /// anything ahead.
    Drive {
        test: FloorTest,
        servo: f64,
        erpm: i32,
        max_steps: i64,
        stop_m: f64,
    },
}

/// Which floor test a drive is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(tag = "kind", content = "index", rename_all = "snake_case")]
pub enum FloorTest {
    /// Straight at a wall.
    Straight,
    /// One of the arcs, by index.
    Arc(usize),
}

/// A floor drive, since it started.
#[derive(Debug, Clone, serde::Serialize)]
pub struct DriveRun {
    pub test: FloorTest,
    /// The servo position driven at.
    pub servo: f64,
    /// The gyroscope standing still just before, in deg/s - its drift.
    pub gyro_bias_deg_s: [f64; 3],
    /// The lidar's distance straight ahead standing still just before, in
    /// meters.
    pub start_ahead_m: Option<f64>,
    /// The tachometer just before.
    pub start_tachometer: i32,
    /// Every reading while driving.
    #[serde(skip)]
    pub samples: Vec<DriveSample>,
    /// Whether it's over, and why.
    pub stop_reason: Option<String>,
    /// Whether it stopped by itself (its distance driven, or at the stop
    /// distance) rather than being stopped early.
    pub completed: bool,
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
    /// The last floor drive.
    pub drive: Option<DriveRun>,
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

    /// Whether the motor has come to a stop - `false` without a reading.
    pub fn motor_stopped(&self) -> bool {
        !self.motor_running()
            && self
                .values
                .as_ref()
                .is_some_and(|values| values.erpm.abs() < STOPPED_ERPM)
    }

    /// Stops the motor, ending the ramp or floor drive running (if any)
    /// with `reason`.
    fn stop_motor(&mut self, reason: &str) {
        self.motor = None;
        if let Some(ramp) = self.ramp.as_mut()
            && !ramp.finished
        {
            ramp.aborted.get_or_insert_with(|| reason.to_string());
        }
        if let Some(drive) = self.drive.as_mut() {
            drive.stop_reason.get_or_insert_with(|| reason.to_string());
        }
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
    /// The lidar's raw scans - see [`Self::lidar_scan`].
    lidar: Arc<RwLockTopic<LidarScan>>,
}

impl Bench {
    /// Starts the thread driving the VESC `config` names, reconnecting to it
    /// whenever it's lost - guarding floor drives with `lidar`'s scans.
    pub fn start(config: VescConfig, lidar: Arc<RwLockTopic<LidarScan>>) -> Self {
        let bench = Self {
            state: Arc::new(Mutex::new(BenchState::default())),
            lidar,
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

    /// The latest lidar scan, if younger than `max_age`.
    pub fn lidar_scan(&self, max_age: Duration) -> Option<LidarScan> {
        let scan = self.lidar.read();
        let fresh = scan.age().is_some_and(|age| age < max_age);
        (fresh && scan.num_lidar_points > 0).then(|| scan.into_value())
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
        if start
            && let MotorRequest::Drive {
                test,
                servo,
                stop_m,
                ..
            } = request
        {
            state.drive = Some(self.drive_start(&state, test, servo, stop_m)?);
        }
        state.motor = Some((request, Instant::now() + HOLD_FOR));
        Ok(())
    }

    /// A floor drive about to start - refused unless the car is still and
    /// the lidar sees the way clear.
    fn drive_start(
        &self,
        state: &BenchState,
        test: FloorTest,
        servo: f64,
        stop_m: f64,
    ) -> Result<DriveRun, String> {
        if !state.motor_stopped() {
            return Err("wait for the wheels to stop".to_string());
        }
        let imu = state
            .imu_over(Duration::from_secs(1))
            .filter(|imu| imu.still())
            .ok_or("hold the car still for a second first")?;
        let scan = self
            .lidar_scan(LIDAR_STALE)
            .ok_or("no scan from the lidar - it guards every floor drive")?;
        if nearest_ahead_m(&scan, GUARD_HALF_WIDTH_RAD).is_some_and(|d| d < stop_m + 0.3) {
            return Err("something is too close ahead - give the car room".to_string());
        }
        Ok(DriveRun {
            test,
            servo,
            gyro_bias_deg_s: imu.gyro_deg_s,
            start_ahead_m: distance_ahead_m(&scan, WALL_HALF_WIDTH_RAD),
            start_tachometer: state
                .values
                .as_ref()
                .ok_or("no reading from the VESC yet")?
                .tachometer,
            samples: Vec::new(),
            stop_reason: None,
            completed: false,
        })
    }

    /// The lidar's distance straight ahead now, in meters.
    pub fn distance_ahead_m(&self) -> Option<f64> {
        distance_ahead_m(&self.lidar_scan(LIDAR_STALE)?, WALL_HALF_WIDTH_RAD)
    }

    /// Stops the motor now.
    pub fn stop(&self) {
        self.state().stop_motor("stopped");
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
                state.stop_motor("the VESC was lost");
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
        // When the floor drive running started.
        let mut drive_started: Option<Instant> = None;

        loop {
            let started = Instant::now();
            let (servo, motor) = {
                let mut state = self.state();
                if let Some((_, until)) = state.motor
                    && started > until
                {
                    state.stop_motor("released");
                }
                (
                    state.servo_request.take(),
                    state.motor.map(|(request, _)| request),
                )
            };
            if motor != Some(MotorRequest::Ramp) {
                ramp_step = None;
            }
            if !matches!(motor, Some(MotorRequest::Drive { .. })) {
                drive_started = None;
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
                Some(MotorRequest::Drive { servo, erpm, .. }) => {
                    if drive_started.is_none() {
                        drive_started = Some(started);
                        port.set_servo(servo)?;
                        self.state().servo = Some(servo);
                    }
                    port.set_rpm(erpm.clamp(-MAX_ERPM, MAX_ERPM))?;
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
            if let (
                Some(since),
                Some(MotorRequest::Drive {
                    max_steps, stop_m, ..
                }),
            ) = (drive_started, motor)
            {
                let stop =
                    self.drive_tick(&mut state, since, now, &values, &imu, max_steps, stop_m);
                if let Some((reason, completed)) = stop {
                    if let Some(drive) = state.drive.as_mut() {
                        drive.completed = completed;
                    }
                    state.stop_motor(reason);
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

    /// Records one reading of the floor drive started at `since`, and says
    /// whether to stop it - why, and whether it completed.
    #[allow(clippy::too_many_arguments)]
    fn drive_tick(
        &self,
        state: &mut BenchState,
        since: Instant,
        now: Instant,
        values: &Values,
        imu: &Imu,
        max_steps: i64,
        stop_m: f64,
    ) -> Option<(&'static str, bool)> {
        let drive = state.drive.as_mut()?;
        drive.samples.push(DriveSample {
            t_s: (now - since).as_secs_f64(),
            erpm: values.erpm,
            tachometer: values.tachometer,
            gyro_deg_s: imu.gyro_deg_s,
        });
        let driven = (i64::from(values.tachometer) - i64::from(drive.start_tachometer)).abs();
        if driven >= max_steps {
            return Some(("drove its whole distance", true));
        }
        let Some(scan) = self.lidar_scan(LIDAR_STALE) else {
            return Some(("the lidar went quiet", false));
        };
        if nearest_ahead_m(&scan, GUARD_HALF_WIDTH_RAD).is_some_and(|d| d < stop_m) {
            return Some(("reached the stop distance", true));
        }
        if now - since > DRIVE_FOR {
            return Some(("took too long", false));
        }
        None
    }
}
