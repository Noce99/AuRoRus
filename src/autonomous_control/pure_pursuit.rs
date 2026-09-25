//! Pure pursuit: follows the selected map's race line (the
//! [`RACE_LINE_TOPIC_NAME`] topic) by steering the rear axle along the circular
//! arc that reaches a point a lookahead distance ahead on the line, with the
//! speed read from the line's speed profile.
//!
//! The pose comes either from localization (odometry composed onto SLAM's
//! `map_to_odom`, as on the real car) or, for debugging in simulation, from
//! the ground truth - see [`PurePursuitConfig::pose_source`]. Without a
//! trustworthy pose, a race line, or while too far from the line, the vehicle
//! is held stopped, and why is reported in the autonomous algorithms panel
//! (see [`report_message`]). See `documentation/autonomous_algorithms.md`.

use crate::autonomous_control::{ParameterTuner, load_config, report_message};
use crate::environment::SpeedPoint;
use crate::topics::{
    ActuatorLimits, AlgorithmParameter, AutonomousAlgorithmInfo, Color, Drawing,
    ODOMETRY_TOPIC_NAME, Odometry, RACE_LINE_TOPIC_NAME, SLAM_STATUS_TOPIC_NAME, SelectedRaceLine,
    Shape, SlamState, SlamStatus, VEHICLE_LIMITS_TOPIC_NAME, VEHICLE_STATUS_TOPIC_NAME,
    VehicleStatus, VescCommand,
};
use crate::{Captain, Executor, Ticker};
use std::any::Any;
use std::f64::consts::PI;
use std::time::Duration;

/// How old the pose may get before it's no longer trusted.
const POSE_TIMEOUT: Duration = Duration::from_millis(300);

/// Entry point build.rs calls - required, with exactly this signature.
pub fn new(name: &str) -> Box<dyn Executor> {
    Box::new(PurePursuit {
        id: 0,
        name: name.to_string(),
        config: load_config(name),
    })
}

/// Every tunable parameter [`PurePursuit`] needs - loaded from
/// `config/autonomous_control/pure_pursuit.toml` at runtime (see [`load_config`]), falling back
/// to the copy compiled in (see [`Default`]). Every field can also be tuned live - see
/// [`parameters`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PurePursuitConfig {
    /// Rate at which a new control is published, in Hz.
    pub rate_hz: f32,
    /// Where the pose comes from: [`POSE_LOCALIZATION`] or [`POSE_GROUND_TRUTH`].
    pub pose_source: u8,
    /// Distance between the front and rear axles, in meters.
    pub wheelbase_m: f64,
    /// Distance from the pose's reference point back to the rear axle, in meters.
    pub lr_m: f64,
    /// Lower clamp of the lookahead distance, in meters.
    pub lookahead_min_m: f64,
    /// Upper clamp of the lookahead distance, in meters.
    pub lookahead_max_m: f64,
    /// Lookahead distance at zero reference speed, in meters.
    pub lookahead_base_m: f64,
    /// Lookahead growth with the reference speed, in seconds.
    pub lookahead_gain_s: f64,
    /// Multiplies the race line's speed profile, pure number.
    pub speed_scale: f64,
    /// How far ahead, in time at the reference speed, the profile speed is read, in seconds.
    pub speed_preview_s: f64,
    /// If > 0, the speed to drive at instead of the profile's, in m/s.
    pub constant_speed: f64,
    /// Farther than this from the line, the vehicle stops, in meters.
    pub max_cross_track_m: f64,
}

/// [`PurePursuitConfig::pose_source`]: odometry composed onto SLAM's `map_to_odom`.
const POSE_LOCALIZATION: u8 = 0;
/// [`PurePursuitConfig::pose_source`]: the simulator's `vehicle_status`.
const POSE_GROUND_TRUTH: u8 = 1;

impl Default for PurePursuitConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/autonomous_control/pure_pursuit.toml"))
            .expect("config/autonomous_control/pure_pursuit.toml must deserialize into PurePursuitConfig")
    }
}

/// The live-tunable parameters, one per [`PurePursuitConfig`] field - see
/// [`ParameterTuner`].
fn parameters() -> [AlgorithmParameter; 12] {
    [
        // At least a few Hz: below 1 Hz every command would be stale on arrival
        // (see `VESC_COMMAND_TIMEOUT`), holding the vehicle stopped.
        AlgorithmParameter::float("rate_hz", 5.0, 200.0, 1.0)
            .unit("Hz")
            .description("Rate at which a new control is published."),
        AlgorithmParameter::int("pose_source", 0, 1, 1)
            .description("0 = localization (only while localizing), 1 = ground truth (simulation only)."),
        AlgorithmParameter::float("wheelbase_m", 0.1, 1.0, 0.01)
            .unit("m")
            .description("Distance between the front and rear axles."),
        AlgorithmParameter::float("lr_m", 0.0, 0.5, 0.01)
            .unit("m")
            .description("Distance from the center of gravity back to the rear axle."),
        AlgorithmParameter::float("lookahead_min_m", 0.2, 5.0, 0.05)
            .unit("m")
            .description("Shortest lookahead distance."),
        AlgorithmParameter::float("lookahead_max_m", 0.5, 10.0, 0.1)
            .unit("m")
            .description("Longest lookahead distance."),
        AlgorithmParameter::float("lookahead_base_m", 0.0, 5.0, 0.05)
            .unit("m")
            .description("Lookahead distance at zero reference speed."),
        AlgorithmParameter::float("lookahead_gain_s", 0.0, 2.0, 0.01)
            .unit("s")
            .description("Lookahead growth with the race line's reference speed."),
        AlgorithmParameter::float("speed_scale", 0.0, 1.5, 0.05)
            .description("Multiplies the race line's speed profile."),
        AlgorithmParameter::float("speed_preview_s", 0.0, 1.0, 0.01)
            .unit("s")
            .description("How far ahead (in time) the profile speed is read, to cover actuator lag."),
        AlgorithmParameter::float("constant_speed", 0.0, 10.0, 0.1)
            .unit("m/s")
            .description("If > 0, drive at this speed instead of the profile's."),
        AlgorithmParameter::float("max_cross_track_m", 0.2, 5.0, 0.1)
            .unit("m")
            .description("Farther than this from the line, the vehicle stops."),
    ]
}

struct PurePursuit {
    id: u8,
    name: String,
    config: PurePursuitConfig,
}

impl Executor for PurePursuit {
    fn init(&mut self, id: u8) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_autonomous_control(
            self.id,
            AutonomousAlgorithmInfo::new(
                "Pure pursuit",
                "Follows the race line, steering toward a point a lookahead distance ahead on it",
            )
            .with_parameters(&self.config, parameters()),
        );
        captain.claim_drawing(self.id);
    }

    fn run(&mut self, captain: &Captain) {
        let command_topic = captain.autonomous_control(self.id);
        let limits_topic = captain.topic::<ActuatorLimits>(VEHICLE_LIMITS_TOPIC_NAME);
        let drawing_topic = captain.drawing(self.id);
        let mut tuner = ParameterTuner::new(captain, self.id);

        // Both derive from `rate_hz`, so are rebuilt whenever it's tuned.
        let mut ticker = Ticker::new(self.config.rate_hz as f64);
        let mut stale_after = drawing_stale_after(&self.config);

        // The race line in use, and the `write_count` it was read at.
        let mut line: Option<(u64, Option<Line>)> = None;
        // Segment the vehicle was nearest last tick - `None` searches the whole line.
        let mut hint: Option<usize> = None;

        while captain.is_running(self.id) {
            if tuner.update(captain, &mut self.config) {
                ticker = Ticker::new(self.config.rate_hz as f64);
                stale_after = drawing_stale_after(&self.config);
            }

            // Nothing may publish a race line at all (e.g. a binary without `MapServer`).
            if let Some(topic) = captain.try_topic::<SelectedRaceLine>(RACE_LINE_TOPIC_NAME) {
                let write_count = topic.meta().write_count;
                if line.as_ref().is_none_or(|(seen, _)| *seen != write_count) {
                    line = Some((write_count, Line::new(topic.read().into_value().points)));
                    hint = None;
                }
            }
            let line = line.as_ref().and_then(|(_, line)| line.as_ref());
            let pose = match self.config.pose_source {
                POSE_LOCALIZATION => localization_pose(captain),
                POSE_GROUND_TRUTH => ground_truth_pose(captain),
                // Unreachable: the tuner keeps it in range.
                _ => Err(format!("Unknown pose source {}.", self.config.pose_source)),
            };

            // A stationary command, and why, unless following the line.
            let stopped = |why: String| (VescCommand::new(0.0, 0.0), Vec::new(), Some(why));
            let (command, shapes, message) = match (line, pose) {
                (None, _) => stopped("No race line on the selected map - vehicle held stopped.".into()),
                (Some(_), Err(why)) => stopped(format!("{why} Vehicle held stopped.")),
                (Some(line), Ok(pose)) => {
                    let limits = limits_topic.read();
                    match control(&self.config, line, pose, hint, &limits) {
                        Ok(control) => {
                            hint = Some(control.nearest.segment);
                            (
                                VescCommand::new(control.steering_rad, control.speed_mps),
                                control.shapes(),
                                None,
                            )
                        }
                        Err(nearest) => {
                            // Lost: next tick searches the whole line again.
                            hint = None;
                            stopped(format!(
                                "{:.2} m off the race line (more than max_cross_track_m) - vehicle held stopped.",
                                nearest.distance_m
                            ))
                        }
                    }
                }
            };

            report_message(captain, self.id, message);
            command_topic
                .write(self.id, command)
                .expect("lost writer authorization for this algorithm's command topic");
            drawing_topic
                .write(self.id, Drawing::new(shapes).stale_after(stale_after))
                .expect("lost writer authorization for the pure pursuit drawing topic");
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
        new(&self.name)
    }
}

/// How long the drawing stays valid: a few publishing periods, but never
/// less than the default.
fn drawing_stale_after(config: &PurePursuitConfig) -> Duration {
    Drawing::DEFAULT_STALE_AFTER.max(Duration::from_secs_f64(3.0 / config.rate_hz as f64))
}

/// A pose in the map frame, heading wrapped to `(-pi, pi]`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Pose {
    x_m: f64,
    y_m: f64,
    heading_rad: f64,
}

impl Pose {
    /// `other`, given in this pose's own frame, expressed in the frame this
    /// pose is expressed in.
    fn compose(&self, other: &Pose) -> Pose {
        let (sin, cos) = self.heading_rad.sin_cos();
        Pose {
            x_m: self.x_m + other.x_m * cos - other.y_m * sin,
            y_m: self.y_m + other.x_m * sin + other.y_m * cos,
            heading_rad: wrap_to_pi(self.heading_rad + other.heading_rad),
        }
    }

    /// This pose moved `distance_m` backward along its heading.
    fn moved_back(&self, distance_m: f64) -> Pose {
        let (sin, cos) = self.heading_rad.sin_cos();
        Pose {
            x_m: self.x_m - distance_m * cos,
            y_m: self.y_m - distance_m * sin,
            ..*self
        }
    }
}

/// The simulator's ground truth, if fresh - else why not.
fn ground_truth_pose(captain: &Captain) -> Result<Pose, String> {
    let status = captain
        .try_topic::<VehicleStatus>(VEHICLE_STATUS_TOPIC_NAME)
        .ok_or("No ground truth pose (vehicle_status) in this binary.")?
        .read();
    if status.age().is_none_or(|age| age > POSE_TIMEOUT) {
        return Err("The ground truth pose (vehicle_status) is stale.".into());
    }
    Ok(Pose {
        x_m: status.x_m,
        y_m: status.y_m,
        heading_rad: status.heading_rad,
    })
}

/// Odometry's latest pose composed onto SLAM's `map_to_odom` - the pose on
/// the map at odometry's rate - while SLAM is localizing (not paused, where
/// the pose would only be dead-reckoned), odometry is fresh, and both agree
/// on odometry's frame - else why not.
fn localization_pose(captain: &Captain) -> Result<Pose, String> {
    let slam = captain
        .try_topic::<SlamStatus>(SLAM_STATUS_TOPIC_NAME)
        .ok_or("No localization (slam_status) in this binary.")?
        .read()
        .into_value();
    if slam.state != SlamState::Localizing {
        return Err("Localization isn't running - start it in the Localization panel.".into());
    }
    let [x_m, y_m, heading_rad] = slam
        .map_to_odom
        .ok_or("Localization has no pose yet.")?;
    let odometry = captain
        .try_topic::<Odometry>(ODOMETRY_TOPIC_NAME)
        .ok_or("No odometry in this binary.")?
        .read();
    if odometry.age().is_none_or(|age| age > POSE_TIMEOUT) {
        return Err("Odometry is stale.".into());
    }
    if odometry.reset_count != slam.odometry_reset_count {
        return Err("Odometry was reset - waiting for localization to catch up.".into());
    }
    let map_to_odom = Pose { x_m, y_m, heading_rad };
    Ok(map_to_odom.compose(&Pose {
        x_m: odometry.x_m,
        y_m: odometry.y_m,
        heading_rad: odometry.heading_rad,
    }))
}

/// A closed race line, with the arc length at each of its points.
struct Line {
    points: Vec<SpeedPoint>,
    /// `cumulative_m[i]` is the arc length from point 0 to point `i`;
    /// `cumulative_m[n]`, one past the last point, is the lap length.
    cumulative_m: Vec<f64>,
}

/// Where a pose projects onto a [`Line`].
#[derive(Debug, Clone, Copy, PartialEq)]
struct Nearest {
    /// Segment from point `segment` to the next one.
    segment: usize,
    /// Arc length of the projection.
    s_m: f64,
    x_m: f64,
    y_m: f64,
    /// Distance from the pose to the projection.
    distance_m: f64,
}

impl Line {
    /// `None` if `points` can't make a closed line.
    fn new(points: Vec<SpeedPoint>) -> Option<Self> {
        if points.len() < 3 {
            return None;
        }
        let n = points.len();
        let mut cumulative_m = Vec::with_capacity(n + 1);
        cumulative_m.push(0.0);
        for i in 0..n {
            let (a, b) = (points[i], points[(i + 1) % n]);
            cumulative_m.push(cumulative_m[i] + (b.x - a.x).hypot(b.y - a.y));
        }
        (cumulative_m[n] > 0.0).then_some(Self {
            points,
            cumulative_m,
        })
    }

    fn lap_m(&self) -> f64 {
        self.cumulative_m[self.points.len()]
    }

    fn segment_len_m(&self, segment: usize) -> f64 {
        self.cumulative_m[segment + 1] - self.cumulative_m[segment]
    }

    /// The projection of `(x_m, y_m)` onto the line: onto every segment if
    /// there's no `hint`, else only onto those from a couple before `hint` up
    /// to `window_m` of arc length past it - so the vehicle never jumps to
    /// another stretch of track that happens to run close by.
    fn nearest(&self, x_m: f64, y_m: f64, hint: Option<usize>, window_m: f64) -> Nearest {
        let n = self.points.len();
        let segments: Box<dyn Iterator<Item = usize>> = match hint {
            None => Box::new(0..n),
            Some(hint) => {
                let first = (hint % n + n - 2) % n;
                let mut covered_m = 0.0;
                Box::new(
                    (0..n)
                        .map(move |k| (first + k) % n)
                        .take_while(move |&segment| {
                            let inside = covered_m <= window_m;
                            covered_m += self.segment_len_m(segment);
                            inside
                        }),
                )
            }
        };
        segments
            .map(|segment| {
                let (a, b) = (self.points[segment], self.points[(segment + 1) % n]);
                let (dx, dy) = (b.x - a.x, b.y - a.y);
                let len2 = dx * dx + dy * dy;
                let t = if len2 > 0.0 {
                    (((x_m - a.x) * dx + (y_m - a.y) * dy) / len2).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let (px, py) = (a.x + t * dx, a.y + t * dy);
                Nearest {
                    segment,
                    s_m: self.cumulative_m[segment] + t * self.segment_len_m(segment),
                    x_m: px,
                    y_m: py,
                    distance_m: (x_m - px).hypot(y_m - py),
                }
            })
            .min_by(|a, b| a.distance_m.total_cmp(&b.distance_m))
            .expect("a line has at least 3 segments, and a window at least one")
    }

    /// The point at arc length `s_m`, wrapped around the lap, interpolated
    /// between its segment's ends - speed included.
    fn at(&self, s_m: f64) -> SpeedPoint {
        let n = self.points.len();
        let s_m = s_m.rem_euclid(self.lap_m());
        let segment = (self.cumulative_m.partition_point(|&c| c <= s_m) - 1).min(n - 1);
        let len_m = self.segment_len_m(segment);
        let t = if len_m > 0.0 {
            (s_m - self.cumulative_m[segment]) / len_m
        } else {
            0.0
        };
        let (a, b) = (self.points[segment], self.points[(segment + 1) % n]);
        SpeedPoint {
            x: a.x + t * (b.x - a.x),
            y: a.y + t * (b.y - a.y),
            speed_mps: a.speed_mps + t * (b.speed_mps - a.speed_mps),
        }
    }
}

/// What one tick decided.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Control {
    steering_rad: f64,
    speed_mps: f64,
    rear_axle: Pose,
    nearest: Nearest,
    target: SpeedPoint,
}

/// Pure pursuit for `pose` (its reference point `lr_m` ahead of the rear
/// axle) on `line`, or - if farther than `max_cross_track_m` from it - the
/// projection that was too far.
fn control(
    config: &PurePursuitConfig,
    line: &Line,
    pose: Pose,
    hint: Option<usize>,
    limits: &ActuatorLimits,
) -> Result<Control, Nearest> {
    let rear_axle = pose.moved_back(config.lr_m);
    // Past the farthest lookahead, with room for a tick's worth of travel.
    let window_m = 1.5 * config.lookahead_max_m + 1.0;
    let nearest = line.nearest(rear_axle.x_m, rear_axle.y_m, hint, window_m);
    if nearest.distance_m > config.max_cross_track_m {
        return Err(nearest);
    }

    let reference_mps = line.at(nearest.s_m).speed_mps.max(0.0);
    let lookahead_m = (config.lookahead_base_m + config.lookahead_gain_s * reference_mps)
        .clamp(config.lookahead_min_m, config.lookahead_max_m.max(config.lookahead_min_m));
    let target = line.at(nearest.s_m + lookahead_m);
    let steering_rad = steering(rear_axle, target.x, target.y, config.wheelbase_m)
        .clamp(-limits.max_steering_angle_rad, limits.max_steering_angle_rad);

    let speed_mps = if config.constant_speed > 0.0 {
        config.constant_speed
    } else {
        let preview_m = reference_mps * config.speed_preview_s;
        config.speed_scale * line.at(nearest.s_m + preview_m).speed_mps
    };

    Ok(Control {
        steering_rad,
        speed_mps: speed_mps.clamp(0.0, limits.max_speed_mps),
        rear_axle,
        nearest,
        target,
    })
}

/// The front-wheel angle that puts `rear_axle` on the circular arc through
/// `(target_x_m, target_y_m)`: `atan(2 L sin(alpha) / ld)`. Positive toward
/// increasing heading, like [`VescCommand::servo_position_rad`].
fn steering(rear_axle: Pose, target_x_m: f64, target_y_m: f64, wheelbase_m: f64) -> f64 {
    let (dx, dy) = (target_x_m - rear_axle.x_m, target_y_m - rear_axle.y_m);
    let distance_m = dx.hypot(dy);
    if distance_m < 1e-9 {
        return 0.0;
    }
    let alpha = wrap_to_pi(dy.atan2(dx) - rear_axle.heading_rad);
    (2.0 * wheelbase_m * alpha.sin() / distance_m).atan()
}

impl Control {
    /// The nearest point, the lookahead point, the chord to it, and the arc
    /// the rear axle is steered along.
    fn shapes(&self) -> Vec<Shape> {
        let rear = self.rear_axle;
        let (x, y) = (self.target.x, self.target.y);
        let mut shapes = vec![
            Shape::Circle {
                x_m: self.nearest.x_m,
                y_m: self.nearest.y_m,
                radius_m: 0.06,
                filled: true,
                color: Color::BLUE,
            },
            Shape::Circle {
                x_m: x,
                y_m: y,
                radius_m: 0.08,
                filled: true,
                color: Color::PURPLE,
            },
            Shape::Polyline {
                points: vec![[rear.x_m as f32, rear.y_m as f32], [x as f32, y as f32]],
                closed: false,
                width_px: 1.0,
                color: Color::PURPLE.with_alpha(128),
            },
        ];

        // Signed radius, positive toward increasing heading: ld / (2 sin(alpha)).
        let (dx, dy) = (x - rear.x_m, y - rear.y_m);
        let alpha = wrap_to_pi(dy.atan2(dx) - rear.heading_rad);
        let radius_m = dx.hypot(dy) / (2.0 * alpha.sin());
        if radius_m.is_finite() && radius_m.abs() < 100.0 {
            let (sin, cos) = rear.heading_rad.sin_cos();
            let (cx, cy) = (rear.x_m - radius_m * sin, rear.y_m + radius_m * cos);
            let from = (rear.y_m - cy).atan2(rear.x_m - cx);
            let to = (y - cy).atan2(x - cx);
            // Driven in increasing angle for a positive radius.
            let (start, end) = if radius_m > 0.0 { (from, to) } else { (to, from) };
            shapes.push(Shape::CircularArc {
                x_m: cx,
                y_m: cy,
                radius_m: radius_m.abs(),
                start_rad: start,
                end_rad: if end < start { end + 2.0 * PI } else { end },
                width_px: 2.0,
                color: Color::PURPLE,
            });
        }
        shapes
    }
}

fn wrap_to_pi(angle_rad: f64) -> f64 {
    let wrapped = (angle_rad + PI).rem_euclid(2.0 * PI) - PI;
    if wrapped <= -PI { wrapped + 2.0 * PI } else { wrapped }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(x: f64, y: f64) -> SpeedPoint {
        SpeedPoint { x, y, speed_mps: 2.0 }
    }

    fn limits() -> ActuatorLimits {
        ActuatorLimits {
            max_steering_angle_rad: 1.5,
            max_steering_rate_rad_s: 4.0,
            max_speed_mps: 10.0,
            max_accel_mps2: 4.0,
            max_decel_mps2: 8.0,
        }
    }

    /// A dense circle of radius `radius_m` around the origin, driven in
    /// increasing angle.
    fn circle(radius_m: f64) -> Line {
        let n = 2000;
        Line::new(
            (0..n)
                .map(|i| {
                    let angle = 2.0 * PI * i as f64 / n as f64;
                    point(radius_m * angle.cos(), radius_m * angle.sin())
                })
                .collect(),
        )
        .unwrap()
    }

    /// A 10 m x 0.5 m loop: out along y = 0, back along y = 0.5.
    fn hairpin() -> Line {
        let mut points: Vec<SpeedPoint> = (0..=50).map(|i| point(i as f64 * 0.2, 0.0)).collect();
        points.extend((0..=50).rev().map(|i| point(i as f64 * 0.2, 0.5)));
        Line::new(points).unwrap()
    }

    #[test]
    fn on_a_straight_line_steering_is_zero() {
        let line = hairpin();
        let config = PurePursuitConfig { lr_m: 0.0, ..Default::default() };
        let pose = Pose { x_m: 3.0, y_m: 0.0, heading_rad: 0.0 };
        let control = control(&config, &line, pose, None, &limits()).unwrap();
        assert!(control.steering_rad.abs() < 1e-9, "{}", control.steering_rad);
        let lookahead_m = (config.lookahead_base_m + config.lookahead_gain_s * 2.0)
            .clamp(config.lookahead_min_m, config.lookahead_max_m);
        assert!((control.target.x - (3.0 + lookahead_m)).abs() < 1e-9, "{:?}", control.target);
    }

    #[test]
    fn on_a_circle_steering_matches_its_curvature() {
        let radius_m = 3.0;
        let line = circle(radius_m);
        let config = PurePursuitConfig {
            lr_m: 0.0,
            wheelbase_m: 0.32,
            ..Default::default()
        };
        // On the circle at angle 0, heading along increasing angle.
        let pose = Pose { x_m: radius_m, y_m: 0.0, heading_rad: PI / 2.0 };
        let control = control(&config, &line, pose, None, &limits()).unwrap();
        let expected = (config.wheelbase_m / radius_m).atan();
        assert!(
            (control.steering_rad - expected).abs() < 1e-3,
            "{} vs {expected}",
            control.steering_rad
        );
    }

    #[test]
    fn a_target_on_the_increasing_heading_side_steers_positive() {
        let pose = Pose { x_m: 0.0, y_m: 0.0, heading_rad: 0.0 };
        // Increasing heading rotates +x toward +y.
        assert!(steering(pose, 1.0, 0.5, 0.32) > 0.0);
        assert!(steering(pose, 1.0, -0.5, 0.32) < 0.0);
    }

    #[test]
    fn steering_is_clamped_to_the_limit() {
        let line = circle(0.5);
        let config = PurePursuitConfig { lr_m: 0.0, ..Default::default() };
        let pose = Pose { x_m: 0.5, y_m: 0.0, heading_rad: PI / 2.0 };
        let limits = ActuatorLimits { max_steering_angle_rad: 0.2, ..limits() };
        let control = control(&config, &line, pose, None, &limits).unwrap();
        assert_eq!(control.steering_rad, 0.2);
    }

    #[test]
    fn the_lookahead_wraps_around_the_lap() {
        let line = hairpin();
        let lap_m = line.lap_m();
        let wrapped = line.at(lap_m + 0.1);
        let direct = line.at(0.1);
        assert!((wrapped.x - direct.x).abs() < 1e-9 && (wrapped.y - direct.y).abs() < 1e-9);
        // Just before the seam (the closing segment from (0, 0.5) to (0, 0)),
        // looking 0.5 m ahead lands past point 0.
        let ahead = line.at(lap_m - 0.2 + 0.5);
        assert!((ahead.x - 0.3).abs() < 1e-9 && ahead.y.abs() < 1e-9, "{ahead:?}");
    }

    #[test]
    fn a_windowed_search_wraps_around_the_seam() {
        let line = hairpin();
        let last = line.points.len() - 1;
        // On the first segment, hinted at the closing one.
        let nearest = line.nearest(0.1, 0.0, Some(last), 2.0);
        assert_eq!(nearest.segment, 0);
        assert!(nearest.distance_m < 1e-9);
    }

    #[test]
    fn a_windowed_search_does_not_jump_to_a_nearby_stretch() {
        let line = hairpin();
        // Closer to the way back (y = 0.5) than to the way out, but tracked
        // on the way out.
        let hint = line.nearest(5.0, 0.0, None, 0.0).segment;
        let nearest = line.nearest(5.0, 0.4, Some(hint), 3.0);
        assert!(nearest.y_m.abs() < 1e-9, "{nearest:?}");
        // Without the hint, the way back wins.
        assert!((line.nearest(5.0, 0.4, None, 3.0).y_m - 0.5).abs() < 1e-9);
    }

    #[test]
    fn too_far_from_the_line_is_an_error() {
        let line = hairpin();
        let config = PurePursuitConfig { lr_m: 0.0, max_cross_track_m: 1.0, ..Default::default() };
        let pose = Pose { x_m: 5.0, y_m: -2.0, heading_rad: 0.0 };
        let err = control(&config, &line, pose, None, &limits()).unwrap_err();
        assert!((err.distance_m - 2.0).abs() < 1e-9);
    }

    #[test]
    fn speed_follows_the_profile_or_the_constant() {
        let line = hairpin();
        let pose = Pose { x_m: 3.0, y_m: 0.0, heading_rad: 0.0 };
        let config = PurePursuitConfig { speed_scale: 0.5, constant_speed: 0.0, ..Default::default() };
        let control_profile = control(&config, &line, pose, None, &limits()).unwrap();
        assert!((control_profile.speed_mps - 1.0).abs() < 1e-9);
        let config = PurePursuitConfig { constant_speed: 20.0, ..config };
        let control_constant = control(&config, &line, pose, None, &limits()).unwrap();
        assert_eq!(control_constant.speed_mps, limits().max_speed_mps);
    }

    #[test]
    fn the_rear_axle_is_behind_the_reference_point() {
        let pose = Pose { x_m: 1.0, y_m: 1.0, heading_rad: PI / 2.0 };
        let rear = pose.moved_back(0.16);
        assert!((rear.x_m - 1.0).abs() < 1e-12 && (rear.y_m - 0.84).abs() < 1e-12);
    }

    #[test]
    fn composing_puts_the_odometry_pose_on_the_map() {
        let map_to_odom = Pose { x_m: 2.0, y_m: 1.0, heading_rad: PI / 2.0 };
        let odometry = Pose { x_m: 1.0, y_m: 0.0, heading_rad: 0.3 };
        let pose = map_to_odom.compose(&odometry);
        assert!((pose.x_m - 2.0).abs() < 1e-12);
        assert!((pose.y_m - 2.0).abs() < 1e-12);
        assert!((pose.heading_rad - (PI / 2.0 + 0.3)).abs() < 1e-12);
    }

    #[test]
    fn every_config_field_is_tunable() {
        let config = PurePursuitConfig::default();
        let info = AutonomousAlgorithmInfo::new("Pure pursuit", "").with_parameters(&config, parameters());
        let serde_json::Value::Object(fields) = serde_json::to_value(config).unwrap() else {
            panic!("the config serializes to an object");
        };
        let mut declared: Vec<&str> = info.parameters.iter().map(|p| p.name.as_str()).collect();
        let mut fields: Vec<&str> = fields.keys().map(String::as_str).collect();
        declared.sort_unstable();
        fields.sort_unstable();
        assert_eq!(declared, fields);
    }
}
