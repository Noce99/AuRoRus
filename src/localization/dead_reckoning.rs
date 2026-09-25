//! [`DeadReckoning`]: integrates wheel speed and yaw rate from
//! [`IMU_TOPIC_NAME`] into a pose on [`ODOMETRY_TOPIC_NAME`], with a
//! covariance that grows as the assumed measurement noise accumulates.
//! Hardware-agnostic - the same code runs on
//! [`crate::sensors::SimulatedImu`]'s readings and, on the real car, on the
//! VESC driver's. Optionally draws where it thinks the vehicle is - a
//! translucent vehicle over the true one, plus the trail it dead-reckoned to
//! get there - on its own drawing topic (see [`crate::topics::Drawing`]), to
//! compare its drift against the truth.

use crate::topics::{
    Color, Drawing, IMU_TOPIC_NAME, ImuReading, ODOMETRY_TOPIC_NAME, Odometry,
    PLACE_AT_START_TOPIC_NAME, PlaceAtStart, START_STATE_TOPIC_NAME, Shape, StartState,
};
use crate::{Captain, Executor, Ticker};
use std::any::Any;
use std::collections::VecDeque;
use std::time::{Duration, Instant};

/// How [`DeadReckoning`] integrates the interval between two readings -
/// see [`integrate`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Integration {
    Euler,
    Midpoint,
    Rk4,
}

/// Every tunable parameter [`DeadReckoning`] needs - loaded from
/// `config/localization/dead_reckoning.toml` (see [`Default`]) or from an
/// arbitrary path via [`crate::config::load`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Deserialize)]
pub struct DeadReckoningConfig {
    /// How often [`DeadReckoning`] polls [`IMU_TOPIC_NAME`], in Hz.
    pub rate_hz: f64,
    /// How each interval between two readings is integrated.
    pub integration: Integration,
    /// Wheel-speed noise the covariance assumes, standard deviation per
    /// reading in meters/second.
    pub speed_std_mps: f64,
    /// Yaw-rate noise the covariance assumes, standard deviation per reading
    /// in radians/second.
    pub yaw_rate_std_rad_s: f64,
    /// Distance from the rear axle forward to the point the pose tracks (the
    /// CG, like [`crate::topics::VehicleStatus`]), in meters. Wheel speed
    /// and yaw rate alone describe the rear axle's motion; any point ahead
    /// of it also slides sideways at `yaw_rate * rear_axle_to_cg_m` while
    /// turning (assuming the rear tires don't slip).
    pub rear_axle_to_cg_m: f64,
    /// Whether to draw the dead-reckoned vehicle and trail, anchored at the
    /// start pose.
    pub draw: bool,
}

impl Default for DeadReckoningConfig {
    fn default() -> Self {
        toml::from_str(include_str!(
            "../../config/localization/dead_reckoning.toml"
        ))
        .expect("config/localization/dead_reckoning.toml must deserialize into DeadReckoningConfig")
    }
}

/// A gap between two readings longer than this means the sensor stalled
/// (or the vehicle was just placed): integrating a constant speed across it
/// would invent motion that never happened, so the reading only restarts the
/// clock instead.
const MAX_INTEGRATION_GAP: Duration = Duration::from_millis(500);

/// Spacing between two consecutive points of the drawn trail, in meters.
const TRAIL_SPACING_M: f64 = 0.05;
/// Most points the drawn trail keeps - the oldest are dropped past this.
const MAX_TRAIL_POINTS: usize = 4000;
/// How often the drawing is republished - below the integration rate, since
/// it's only for a human watching; a viewer dead-reckons the drawn vehicle
/// forward between two drawings, so it still moves smoothly.
const DRAWING_PERIOD: Duration = Duration::from_millis(50);

/// Size the dead-reckoned vehicle is drawn at - the same roughly
/// 1/10-scale RC car [`crate::actuators::SimulatedVehicle`] draws, so the two
/// overlap exactly when the estimate is right.
const DRAWN_BODY_LENGTH_M: f64 = 0.45;
const DRAWN_BODY_WIDTH_M: f64 = 0.25;
const DRAWN_AXLE_M: f64 = 0.16;
/// Translucent, so the true vehicle stays visible underneath.
const DRAWN_COLOR: Color = Color::PURPLE.with_alpha(150);
/// Above [`crate::actuators::SimulatedVehicle`]'s own drawing (`z_index`
/// 10), which would otherwise hide the estimate whenever it's close to the
/// truth.
const DRAWN_Z_INDEX: i32 = 11;

/// Dead reckoning: claims [`ODOMETRY_TOPIC_NAME`] and, polling
/// [`IMU_TOPIC_NAME`] at [`DeadReckoningConfig::rate_hz`], integrates every
/// new [`ImuReading`] over the time since the previous one (both taken from
/// the topic's own [`crate::WriteMeta`]), publishing the resulting
/// [`Odometry`] once per new reading. Resets to the `odom` origin - zero
/// covariance - whenever [`START_STATE_TOPIC_NAME`] changes or
/// [`PLACE_AT_START_TOPIC_NAME`] is bumped, the same events
/// [`crate::actuators::SimulatedVehicle`] places the vehicle on.
pub struct DeadReckoning {
    id: u8,
    name: String,
    config: DeadReckoningConfig,
}

impl DeadReckoning {
    pub fn new(name: impl Into<String>, config: DeadReckoningConfig) -> Self {
        Self {
            id: 0,
            name: name.into(),
            config,
        }
    }
}

impl Executor for DeadReckoning {
    fn init(&mut self, id: u8) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_writer::<Odometry>(ODOMETRY_TOPIC_NAME, self.id, Odometry::default);
        if self.config.draw {
            captain.claim_drawing(self.id);
        }
    }

    fn run(&mut self, captain: &Captain) {
        let imu_topic = captain.topic::<ImuReading>(IMU_TOPIC_NAME);
        let odometry_topic = captain.topic::<Odometry>(ODOMETRY_TOPIC_NAME);
        let start_state_topic = captain.topic::<StartState>(START_STATE_TOPIC_NAME);
        let place_at_start_topic = captain.topic::<PlaceAtStart>(PLACE_AT_START_TOPIC_NAME);
        let drawing_topic = self.config.draw.then(|| captain.drawing(self.id));

        let mut applied_start = start_state_topic.read().into_value();
        let mut applied_place_request = place_at_start_topic.read().requested;
        let mut odometry = Odometry::default();
        let mut trail = Trail::new(applied_start);
        // The write count of the last reading consumed, so a re-read of the
        // same one is skipped - starting at whatever is there now, so a
        // reading left over from before this executor started isn't
        // integrated.
        let initial = imu_topic.read();
        let mut last_write_count = initial.meta.write_count;
        let mut last_written_at: Option<Instant> = None;
        let mut last_drawn = Instant::now();
        let mut ticker = Ticker::new(self.config.rate_hz);

        while captain.is_running(self.id) {
            let wanted_start = start_state_topic.read().into_value();
            let wanted_place_request = place_at_start_topic.read().requested;
            if wanted_start != applied_start || wanted_place_request != applied_place_request {
                odometry = Odometry {
                    reset_count: odometry.reset_count.wrapping_add(1),
                    ..Odometry::default()
                };
                trail = Trail::new(wanted_start);
                last_written_at = None;
                applied_start = wanted_start;
                applied_place_request = wanted_place_request;
                odometry_topic
                    .write(self.id, odometry)
                    .expect("lost writer authorization for the odometry topic");
            }

            let reading = imu_topic.read();
            if reading.meta.write_count != last_write_count {
                last_write_count = reading.meta.write_count;
                let written_at = reading
                    .meta
                    .written_at
                    .expect("a written reading always has a write instant");
                if let Some(previous) = last_written_at {
                    let dt = written_at.saturating_duration_since(previous);
                    if !dt.is_zero() && dt <= MAX_INTEGRATION_GAP {
                        odometry = step(odometry, &reading, dt.as_secs_f64(), &self.config);
                        trail.extend(&odometry);
                    }
                }
                last_written_at = Some(written_at);
                // Accelerations are passed through even on a reading that
                // only restarted the clock.
                odometry.ax_mps2 = reading.ax_mps2;
                odometry.ay_mps2 = reading.ay_mps2;
                odometry_topic
                    .write(self.id, odometry)
                    .expect("lost writer authorization for the odometry topic");
            }

            if let Some(drawing_topic) = &drawing_topic
                && last_drawn.elapsed() >= DRAWING_PERIOD
            {
                last_drawn = Instant::now();
                drawing_topic
                    .write(self.id, trail.drawing(&odometry))
                    .expect("lost writer authorization for dead reckoning's drawing topic");
            }

            ticker.wait();
        }
    }

    fn name(&self) -> String {
        self.name.clone()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn fresh(&self) -> Box<dyn Executor> {
        Box::new(DeadReckoning::new(self.name.clone(), self.config))
    }
}

/// `odometry` advanced by one `dt_s` interval under `reading`'s wheel speed
/// and yaw rate (held constant across it), its covariance propagated as
/// `P' = F P F^T + G Q G^T` - `F` the pose Jacobian, `G` the Jacobian with
/// respect to the (speed, yaw rate) inputs, and `Q` their assumed noise from
/// `config` - both linearized about the interval's mid-heading.
fn step(
    odometry: Odometry,
    reading: &ImuReading,
    dt_s: f64,
    config: &DeadReckoningConfig,
) -> Odometry {
    let v = reading.wheel_speed_mps;
    let w = reading.yaw_rate_rad_s;
    let lever_m = config.rear_axle_to_cg_m;
    let (x_m, y_m, heading_rad) = integrate(
        (odometry.x_m, odometry.y_m, odometry.heading_rad),
        (v, w * lever_m),
        w,
        dt_s,
        config.integration,
    );

    // The tracked point's world-frame velocity at the mid-heading.
    let (sin_m, cos_m) = (odometry.heading_rad + 0.5 * w * dt_s).sin_cos();
    let vx_world = v * cos_m - w * lever_m * sin_m;
    let vy_world = v * sin_m + w * lever_m * cos_m;
    let f = [
        [1.0, 0.0, -vy_world * dt_s],
        [0.0, 1.0, vx_world * dt_s],
        [0.0, 0.0, 1.0],
    ];
    let g = [
        [
            dt_s * cos_m,
            -lever_m * dt_s * sin_m - 0.5 * dt_s * dt_s * vy_world,
        ],
        [
            dt_s * sin_m,
            lever_m * dt_s * cos_m + 0.5 * dt_s * dt_s * vx_world,
        ],
        [0.0, dt_s],
    ];
    let q = [
        config.speed_std_mps.powi(2),
        config.yaw_rate_std_rad_s.powi(2),
    ];
    let p = odometry.covariance;
    let mut covariance = [[0.0; 3]; 3];
    for (i, row) in covariance.iter_mut().enumerate() {
        for (j, cell) in row.iter_mut().enumerate() {
            let fpft: f64 = (0..3)
                .flat_map(|k| (0..3).map(move |l| (k, l)))
                .map(|(k, l)| f[i][k] * p[k][l] * f[j][l])
                .sum();
            let gqgt: f64 = (0..2).map(|k| g[i][k] * q[k] * g[j][k]).sum();
            *cell = fpft + gqgt;
        }
    }

    Odometry {
        x_m,
        y_m,
        heading_rad,
        speed_mps: v,
        yaw_rate_rad_s: w,
        ax_mps2: reading.ax_mps2,
        ay_mps2: reading.ay_mps2,
        covariance,
        reset_count: odometry.reset_count,
    }
}

/// `(x_m, y_m, heading_rad)` advanced by `dt_s` of planar rigid-body motion
/// at constant body-frame velocity `(vx, vy)` and yaw rate `w` - `dx/dt = vx
/// cos(heading) - vy sin(heading)`, `dy/dt = vx sin(heading) + vy
/// cos(heading)`, `dheading/dt = w` - with the heading wrapped to
/// `(-pi, pi]`.
fn integrate(
    pose: (f64, f64, f64),
    (vx, vy): (f64, f64),
    w: f64,
    dt_s: f64,
    method: Integration,
) -> (f64, f64, f64) {
    let (x, y, heading) = pose;
    // Only the heading varies within the interval, and linearly, so every
    // method reduces to averaging the velocity direction at a few headings.
    let velocity_at = |fraction: f64| {
        let (sin, cos) = (heading + fraction * w * dt_s).sin_cos();
        (vx * cos - vy * sin, vx * sin + vy * cos)
    };
    let (dx, dy) = match method {
        Integration::Euler => velocity_at(0.0),
        Integration::Midpoint => velocity_at(0.5),
        Integration::Rk4 => {
            let (k1, k2, k4) = (velocity_at(0.0), velocity_at(0.5), velocity_at(1.0));
            // k2 == k3: both stages sit at the mid-heading.
            (
                (k1.0 + 4.0 * k2.0 + k4.0) / 6.0,
                (k1.1 + 4.0 * k2.1 + k4.1) / 6.0,
            )
        }
    };
    (x + dx * dt_s, y + dy * dt_s, wrap_to_pi(heading + w * dt_s))
}

/// Wraps an angle in radians to `(-pi, pi]`.
fn wrap_to_pi(angle_rad: f64) -> f64 {
    angle_rad.sin().atan2(angle_rad.cos())
}

/// The dead-reckoned path so far, in world coordinates - the `odom` frame
/// placed at `anchor`, the start pose it was last reset at.
struct Trail {
    anchor: StartState,
    points: VecDeque<[f32; 2]>,
}

impl Trail {
    fn new(anchor: StartState) -> Self {
        Self {
            anchor,
            points: VecDeque::from([[anchor.x_m as f32, anchor.y_m as f32]]),
        }
    }

    /// Where the `odom`-frame point `(x_m, y_m)` lands in the world.
    fn to_world(&self, x_m: f64, y_m: f64) -> (f64, f64) {
        let (sin, cos) = self.anchor.heading_rad.sin_cos();
        (
            self.anchor.x_m + x_m * cos - y_m * sin,
            self.anchor.y_m + x_m * sin + y_m * cos,
        )
    }

    /// Appends `odometry`'s position if it's moved at least
    /// [`TRAIL_SPACING_M`] from the last point.
    fn extend(&mut self, odometry: &Odometry) {
        let (x, y) = self.to_world(odometry.x_m, odometry.y_m);
        let point = [x as f32, y as f32];
        let far_enough = self.points.back().is_none_or(|last| {
            f64::from(last[0] - point[0]).hypot(f64::from(last[1] - point[1])) >= TRAIL_SPACING_M
        });
        if far_enough {
            if self.points.len() == MAX_TRAIL_POINTS {
                self.points.pop_front();
            }
            self.points.push_back(point);
        }
    }

    /// The trail, plus a vehicle at `odometry`'s pose - both in world
    /// coordinates.
    fn drawing(&self, odometry: &Odometry) -> Drawing {
        let (x_m, y_m) = self.to_world(odometry.x_m, odometry.y_m);
        Drawing::default()
            .element(
                "Trail",
                [Shape::Polyline {
                    points: self.points.iter().copied().collect(),
                    closed: false,
                    width_px: 2.0,
                    color: DRAWN_COLOR,
                }],
            )
            .element(
                "Vehicle",
                [Shape::Vehicle {
                    x_m,
                    y_m,
                    heading_rad: self.anchor.heading_rad + odometry.heading_rad,
                    speed_mps: odometry.speed_mps,
                    // Dead reckoning never sees the steering command.
                    steering_rad: 0.0,
                    length_m: DRAWN_BODY_LENGTH_M,
                    width_m: DRAWN_BODY_WIDTH_M,
                    front_axle_m: DRAWN_AXLE_M,
                    rear_axle_m: DRAWN_AXLE_M,
                    color: DRAWN_COLOR,
                }],
            )
            .stale_after(Drawing::DEFAULT_STALE_AFTER.max(3 * DRAWING_PERIOD))
            .z_index(DRAWN_Z_INDEX)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    /// A config tracking the rear axle itself, so the pose follows plain
    /// unicycle motion.
    fn config(integration: Integration) -> DeadReckoningConfig {
        DeadReckoningConfig {
            integration,
            rear_axle_to_cg_m: 0.0,
            ..DeadReckoningConfig::default()
        }
    }

    fn reading(wheel_speed_mps: f64, yaw_rate_rad_s: f64) -> ImuReading {
        ImuReading {
            wheel_speed_mps,
            yaw_rate_rad_s,
            ..ImuReading::default()
        }
    }

    /// Integrates `seconds` of a constant `reading`, sampled every `dt_s`.
    fn drive(
        reading: ImuReading,
        seconds: f64,
        dt_s: f64,
        config: &DeadReckoningConfig,
    ) -> Odometry {
        let steps = (seconds / dt_s).round() as usize;
        (0..steps).fold(Odometry::default(), |odometry, _| {
            step(odometry, &reading, dt_s, config)
        })
    }

    #[test]
    fn the_checked_in_config_deserializes() {
        assert!(DeadReckoningConfig::default().rate_hz > 0.0);
    }

    #[test]
    fn a_straight_drive_advances_along_the_heading() {
        let odometry = drive(reading(2.0, 0.0), 3.0, 0.01, &config(Integration::Euler));
        assert!((odometry.x_m - 6.0).abs() < 1e-9);
        assert!(odometry.y_m.abs() < 1e-12);
        assert_eq!(odometry.heading_rad, 0.0);
    }

    #[test]
    fn a_constant_turn_traces_the_circle_for_every_integration_method() {
        // Speed 2 m/s at 1 rad/s: a 2 m radius circle, centered at (0, 2).
        // A quarter turn ends at (2, 2), facing +y.
        for (method, tolerance_m) in [
            (Integration::Euler, 0.05),
            (Integration::Midpoint, 1e-4),
            (Integration::Rk4, 1e-6),
        ] {
            let odometry = drive(
                reading(2.0, 1.0),
                FRAC_PI_2,
                FRAC_PI_2 / 100.0,
                &config(method),
            );
            let error_m = (odometry.x_m - 2.0).hypot(odometry.y_m - 2.0);
            assert!(error_m < tolerance_m, "{method:?}: ended {error_m} m off");
            assert!((odometry.heading_rad - FRAC_PI_2).abs() < 1e-9);
        }
    }

    #[test]
    fn a_half_turn_of_the_kinematic_bicycle_tracks_its_cg() {
        // Fed the CG's exact vx and yaw rate, dead reckoning must follow the
        // CG - not the rear axle, which ends 2 * lr off after a half turn.
        use crate::environment::simulator::vehicle::{
            BicycleParams, BicycleState, step as bicycle_step,
        };
        let params = BicycleParams {
            lf_m: 0.16,
            lr_m: 0.16,
        };
        let config = DeadReckoningConfig {
            integration: Integration::Midpoint,
            rear_axle_to_cg_m: params.lr_m,
            ..DeadReckoningConfig::default()
        };
        let (steering_rad, dt_s) = (0.3, 0.01);
        let beta = (0.5 * f64::tan(steering_rad)).atan();
        let mut truth = BicycleState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            speed_mps: 2.0,
        };
        let mut odometry = Odometry::default();
        while truth.heading_rad < PI - 0.05 {
            let reading = reading(
                truth.speed_mps * beta.cos(),
                truth.speed_mps * beta.sin() / params.lr_m,
            );
            odometry = step(odometry, &reading, dt_s, &config);
            truth = bicycle_step(truth, params, steering_rad, 0.0, dt_s);
        }
        let error_m = (odometry.x_m - truth.x_m).hypot(odometry.y_m - truth.y_m);
        assert!(error_m < 1e-3, "ended {error_m} m off the CG");
    }

    #[test]
    fn the_heading_stays_wrapped() {
        let odometry = drive(reading(0.0, 1.0), 4.0, 0.01, &config(Integration::Midpoint));
        assert!(odometry.heading_rad > -PI && odometry.heading_rad <= PI);
        assert!((odometry.heading_rad - (4.0 - 2.0 * PI)).abs() < 1e-9);
    }

    #[test]
    fn a_constant_yaw_rate_bias_drifts_the_heading_linearly() {
        // A stationary car whose gyro reads 0.01 rad/s: 60 s -> 0.6 rad.
        let odometry = drive(
            reading(0.0, 0.01),
            60.0,
            0.01,
            &config(Integration::Midpoint),
        );
        assert!((odometry.heading_rad - 0.6).abs() < 1e-9);
        assert_eq!((odometry.x_m, odometry.y_m), (0.0, 0.0));
    }

    #[test]
    fn the_covariance_grows_while_driving_and_stays_symmetric() {
        let config = config(Integration::Midpoint);
        let mut odometry = Odometry::default();
        let mut previous_trace = 0.0;
        for _ in 0..500 {
            odometry = step(odometry, &reading(2.0, 0.5), 0.01, &config);
            let p = odometry.covariance;
            let trace = p[0][0] + p[1][1] + p[2][2];
            assert!(trace > previous_trace);
            previous_trace = trace;
            for (i, row) in p.iter().enumerate() {
                for (j, cell) in row.iter().enumerate() {
                    assert!((cell - p[j][i]).abs() < 1e-15);
                }
            }
        }
        // Heading variance is exactly n * (sigma_w * dt)^2.
        let expected = 500.0 * (config.yaw_rate_std_rad_s * 0.01).powi(2);
        assert!((odometry.covariance[2][2] - expected).abs() < 1e-15);
    }

    #[test]
    fn a_step_keeps_the_reset_count() {
        let odometry = Odometry {
            reset_count: 7,
            ..Odometry::default()
        };
        let stepped = step(
            odometry,
            &reading(1.0, 0.1),
            0.01,
            &config(Integration::Euler),
        );
        assert_eq!(stepped.reset_count, 7);
    }

    #[test]
    fn the_trail_is_placed_at_the_start_pose() {
        let trail = Trail::new(StartState {
            x_m: 10.0,
            y_m: 5.0,
            heading_rad: FRAC_PI_2,
            speed_mps: 0.0,
        });
        // 1 m forward in odom = 1 m along +y in the world, from (10, 5).
        let (x, y) = trail.to_world(1.0, 0.0);
        assert!((x - 10.0).abs() < 1e-12);
        assert!((y - 6.0).abs() < 1e-12);
    }

    #[test]
    fn the_drawn_vehicle_sits_at_the_estimate_in_world_coordinates() {
        let trail = Trail::new(StartState {
            x_m: 10.0,
            y_m: 5.0,
            heading_rad: FRAC_PI_2,
            speed_mps: 0.0,
        });
        let odometry = Odometry {
            x_m: 1.0,
            heading_rad: 0.3,
            speed_mps: 2.0,
            ..Odometry::default()
        };
        let drawing = trail.drawing(&odometry);
        let vehicle = drawing
            .shapes
            .iter()
            .find_map(|shape| match shape {
                Shape::Vehicle {
                    x_m,
                    y_m,
                    heading_rad,
                    speed_mps,
                    ..
                } => Some((*x_m, *y_m, *heading_rad, *speed_mps)),
                _ => None,
            })
            .expect("the drawing must include the estimated vehicle");

        assert!((vehicle.0 - 10.0).abs() < 1e-12);
        assert!((vehicle.1 - 6.0).abs() < 1e-12);
        assert!((vehicle.2 - (FRAC_PI_2 + 0.3)).abs() < 1e-12);
        assert_eq!(vehicle.3, 2.0);
    }
}
