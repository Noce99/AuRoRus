//! UBM Path Follower: follows the selected map's race line (the
//! [`RACE_LINE_TOPIC_NAME`] topic) with one of three steering laws - a PD
//! controller or ubm's "P-enhanced" controller on the heading error toward a
//! lookahead point, or the Stanley controller - plus an optional feedforward
//! term, at the line's profile speed. Ported from ubm's
//! `path_follower_node.cpp` and `steering_controller.cpp`.
//!
//! The pose comes from localization or, in simulation, the ground truth -
//! see [`UbmPathFollowerConfig::pose_source`]. Without a trustworthy pose, a
//! race line, or while too far from the line, the vehicle is held stopped,
//! and why is reported in the autonomous algorithms panel (see
//! [`report_message`]). See `documentation/autonomous_algorithms.md`.

use crate::autonomous_control::shared::race_line::{Line, Nearest, POSE_GROUND_TRUTH, Pose, pose, speed, wrap_to_pi};
use crate::autonomous_control::shared::steering::{SteeringGains, p_enhanced, pd};
use crate::autonomous_control::{Instance, ParameterTuner, load_config, report_message};
use crate::topics::{
    ActuatorLimits, AlgorithmParameter, AutonomousAlgorithmInfo, Color, Drawing,
    SelectedRaceLine, Shape, VescCommand,
};
use crate::{Captain, Executor, Ticker};
use std::any::Any;
use std::time::{Duration, Instant};

/// Entry point build.rs calls - required, with exactly this signature.
pub fn new(instance: Instance) -> Box<dyn Executor> {
    let mut config: UbmPathFollowerConfig = load_config(&instance.config_name);
    // An opponent has no localization of its own - see `Instance::opponent`.
    if instance.is_opponent() {
        config.pose_source = POSE_GROUND_TRUTH;
    }
    Box::new(UbmPathFollower { id: 0, instance, config })
}

/// [`UbmPathFollowerConfig::controller`]: PD on the heading error toward the lookahead point.
const CONTROLLER_PD: u8 = 0;
/// [`UbmPathFollowerConfig::controller`]: ubm's P-enhanced controller.
const CONTROLLER_P_ENHANCED: u8 = 1;
/// [`UbmPathFollowerConfig::controller`]: the Stanley controller.
const CONTROLLER_STANLEY: u8 = 2;

/// [`UbmPathFollowerConfig::feedforward`]: from the line's curvature.
const FEEDFORWARD_PATH: u8 = 1;
/// [`UbmPathFollowerConfig::feedforward`]: learned per race line point.
const FEEDFORWARD_LEARNED: u8 = 2;

/// Stanley divides by the speed, but never by less than this, in m/s.
const STANLEY_MIN_SPEED_MPS: f64 = 0.5;

/// Every tunable parameter [`UbmPathFollower`] needs - loaded from
/// `config/autonomous_control/ubm_path_follower.toml` at runtime (see [`load_config`]), falling
/// back to the copy compiled in (see [`Default`]). Every field can also be tuned live - see
/// [`parameters`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct UbmPathFollowerConfig {
    /// Rate at which a new control is published, in Hz.
    pub rate_hz: f32,
    /// Where the pose and speed come from: [`POSE_LOCALIZATION`](crate::autonomous_control::shared::race_line::POSE_LOCALIZATION)
    /// or [`POSE_GROUND_TRUTH`](crate::autonomous_control::shared::race_line::POSE_GROUND_TRUTH).
    pub pose_source: u8,
    /// Steering law: [`CONTROLLER_PD`], [`CONTROLLER_P_ENHANCED`] or [`CONTROLLER_STANLEY`].
    pub controller: u8,
    /// Feedforward term: 0 = none, [`FEEDFORWARD_PATH`] or [`FEEDFORWARD_LEARNED`].
    pub feedforward: u8,
    /// Distance between the front and rear axles, in meters.
    pub wheelbase_m: f64,
    /// Proportional gain on the heading error, pure number.
    pub kk_s: f64,
    /// Derivative gain on the heading error (PD only), in seconds.
    pub kd_s: f64,
    /// Gain on the cross-track error (Stanley only), in m/s per meter.
    pub k_stanley: f64,
    /// Race line points past the nearest one Stanley tracks (a time-delay
    /// compensation), pure number.
    pub tdp: usize,
    /// Above this speed, P-enhanced steers less, in m/s.
    pub min_speed: f64,
    /// Heading errors below this are damped further by P-enhanced, in radians.
    pub max_error: f64,
    /// How much P-enhanced damps with speed, pure number.
    pub decay_v: f64,
    /// How much P-enhanced damps small errors, per m/s above `min_speed`.
    pub decay_e: f64,
    /// Lookahead distance growth with the speed, in seconds.
    pub look_ahead_gain_s: f64,
    /// Lookahead distance at standstill, in meters.
    pub min_look_ahead_m: f64,
    /// Feedforward delay: meters of the line ahead (path-based), or race
    /// line points behind (learned).
    pub delay_ff_action: f64,
    /// Feedforward gain, pure number.
    pub beta_ff_gain: f64,
    /// Learned feedforward: weight of the previous value when updating it
    /// (1 = never update), pure number.
    pub averaging_ff_gain: f64,
    /// Multiplies the race line's speed profile, pure number.
    pub scale_speed: f64,
    /// If > 0, the speed to drive at instead of the profile's, in m/s.
    pub constant_speed: f64,
    /// Farther than this from the line, the vehicle stops, in meters.
    pub max_cross_track_m: f64,
}

impl Default for UbmPathFollowerConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/autonomous_control/ubm_path_follower.toml"))
            .expect("config/autonomous_control/ubm_path_follower.toml must deserialize into UbmPathFollowerConfig")
    }
}

impl UbmPathFollowerConfig {
    fn gains(&self) -> SteeringGains {
        SteeringGains {
            kk_s: self.kk_s,
            kd_s: self.kd_s,
            min_speed: self.min_speed,
            max_error: self.max_error,
            decay_v: self.decay_v,
            decay_e: self.decay_e,
        }
    }
}

/// The live-tunable parameters, one per [`UbmPathFollowerConfig`] field -
/// see [`ParameterTuner`].
fn parameters() -> [AlgorithmParameter; 21] {
    [
        // At least a few Hz: below 1 Hz every command would be stale on arrival
        // (see `VESC_COMMAND_TIMEOUT`), holding the vehicle stopped.
        AlgorithmParameter::float("rate_hz", 5.0, 200.0, 1.0)
            .unit("Hz")
            .description("Rate at which a new control is published."),
        AlgorithmParameter::int("pose_source", 0, 1, 1)
            .description("0 = localization (only while localizing), 1 = ground truth (simulation only)."),
        AlgorithmParameter::int("controller", CONTROLLER_PD.into(), CONTROLLER_STANLEY.into(), 1)
            .description("Steering law: 0 = PD, 1 = P-enhanced, 2 = Stanley."),
        AlgorithmParameter::int("feedforward", 0, 2, 1)
            .description("Feedforward: 0 = none, 1 = from the line's curvature, 2 = learned per point."),
        AlgorithmParameter::float("wheelbase_m", 0.1, 1.0, 0.01)
            .unit("m")
            .description("Distance between the front and rear axles."),
        AlgorithmParameter::float("kk_s", 0.0, 5.0, 0.05)
            .description("Proportional gain on the heading error."),
        AlgorithmParameter::float("kd_s", 0.0, 1.0, 0.01)
            .unit("s")
            .description("Derivative gain on the heading error (PD only)."),
        AlgorithmParameter::float("k_stanley", 0.0, 10.0, 0.1)
            .description("Gain on the cross-track error (Stanley only)."),
        AlgorithmParameter::int("tdp", 0, 50, 1)
            .unit("points")
            .description("Race line points past the nearest one Stanley tracks."),
        AlgorithmParameter::float("min_speed", 0.0, 10.0, 0.1)
            .unit("m/s")
            .description("Above this speed, P-enhanced steers less."),
        AlgorithmParameter::float("max_error", 0.01, 1.0, 0.01)
            .unit("rad")
            .description("Heading errors below this are damped further by P-enhanced."),
        AlgorithmParameter::float("decay_v", 0.0, 2.0, 0.05)
            .description("How much P-enhanced damps with speed."),
        AlgorithmParameter::float("decay_e", 0.0, 2.0, 0.05)
            .description("How much P-enhanced damps small errors, per m/s above min_speed."),
        AlgorithmParameter::float("look_ahead_gain_s", 0.0, 2.0, 0.01)
            .unit("s")
            .description("Lookahead distance growth with the speed."),
        AlgorithmParameter::float("min_look_ahead_m", 0.1, 5.0, 0.05)
            .unit("m")
            .description("Lookahead distance at standstill."),
        AlgorithmParameter::float("delay_ff_action", 0.0, 50.0, 1.0)
            .description("Feedforward delay: meters ahead (path-based) or points behind (learned)."),
        AlgorithmParameter::float("beta_ff_gain", 0.0, 2.0, 0.05)
            .description("Feedforward gain."),
        AlgorithmParameter::float("averaging_ff_gain", 0.0, 1.0, 0.05)
            .description("Learned feedforward: weight of the previous value (1 = never update)."),
        AlgorithmParameter::float("scale_speed", 0.0, 1.5, 0.05)
            .description("Multiplies the race line's speed profile."),
        AlgorithmParameter::float("constant_speed", 0.0, 10.0, 0.1)
            .unit("m/s")
            .description("If > 0, drive at this speed instead of the profile's."),
        AlgorithmParameter::float("max_cross_track_m", 0.2, 5.0, 0.1)
            .unit("m")
            .description("Farther than this from the line, the vehicle stops."),
    ]
}

struct UbmPathFollower {
    id: u16,
    instance: Instance,
    config: UbmPathFollowerConfig,
}

impl Executor for UbmPathFollower {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_autonomous_control(
            self.id,
            &self.instance.algorithm_topics(),
            AutonomousAlgorithmInfo::new(
                "UBM Path Follower",
                "Follows the race line with a PD, P-enhanced or Stanley steering law, plus feedforward",
            )
            .requires_race_line()
            .with_parameters(&self.config, parameters()),
        );
        captain.claim_drawing(self.id);
    }

    fn run(&mut self, captain: &Captain) {
        let command_topic = captain.autonomous_control(&self.instance.algorithm_topics());
        let limits_topic = captain.topic::<ActuatorLimits>(&self.instance.vehicle.vehicle_limits());
        let drawing_topic = captain.drawing(self.id);
        let mut tuner = ParameterTuner::new(self.id, &self.instance);

        // Both derive from `rate_hz`, so are rebuilt whenever it's tuned.
        let mut ticker = Ticker::new(self.config.rate_hz as f64);
        let mut stale_after = drawing_stale_after(&self.config);

        // The race line in use, and the `write_count` it was read at.
        let mut line: Option<(u64, Option<Line>)> = None;
        // Segment the vehicle was nearest last tick - `None` searches the whole line.
        let mut hint: Option<usize> = None;
        let mut state = State::default();

        while captain.is_running(self.id) {
            if tuner.update(captain, &mut self.config) {
                ticker = Ticker::new(self.config.rate_hz as f64);
                stale_after = drawing_stale_after(&self.config);
            }

            // Nothing may publish a race line at all (e.g. a binary without `MapServer`).
            if let Some(topic) = captain.try_topic::<SelectedRaceLine>(&self.instance.vehicle.race_line()) {
                let write_count = topic.meta().write_count;
                if line.as_ref().is_none_or(|(seen, _)| *seen != write_count) {
                    line = Some((write_count, Line::new(topic.read().into_value().points)));
                    hint = None;
                    state = State::default();
                }
            }
            let line = line.as_ref().and_then(|(_, line)| line.as_ref());
            let pose = pose(captain, &self.instance.vehicle, self.config.pose_source)
                .and_then(|pose| Ok((pose, speed(captain, &self.instance.vehicle, self.config.pose_source)?)));

            // A stationary command, and why, unless following the line.
            let stopped = |why: String| (VescCommand::new(0.0, 0.0), Drawing::default(), Some(why));
            let (command, drawing, message) = match (line, pose) {
                (None, _) => stopped("No race line on the selected map - vehicle held stopped.".into()),
                (Some(_), Err(why)) => stopped(format!("{why} Vehicle held stopped.")),
                (Some(line), Ok((pose, speed_mps))) => {
                    let limits = limits_topic.read();
                    match control(&self.config, line, pose, speed_mps, hint, &limits, &mut state, Instant::now()) {
                        Ok(control) => {
                            hint = Some(control.nearest.segment);
                            (
                                VescCommand::new(control.steering_rad, control.speed_mps),
                                control.drawing(),
                                None,
                            )
                        }
                        Err(nearest) => {
                            // Lost: next tick searches the whole line again.
                            hint = None;
                            state.previous_error = None;
                            stopped(format!(
                                "{:.2} m off the race line (more than max_cross_track_m) - vehicle held stopped.",
                                nearest.distance_m
                            ))
                        }
                    }
                }
            };

            report_message(captain, self.id, &self.instance, message);
            command_topic
                .write(self.id, command)
                .expect("lost writer authorization for this algorithm's command topic");
            drawing_topic
                .write(self.id, drawing.stale_after(stale_after))
                .expect("lost writer authorization for the UBM path follower drawing topic");
            ticker.wait();
        }
    }

    fn name(&self) -> String {
        self.instance.name.clone()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn fresh(&self) -> Box<dyn Executor> {
        new(self.instance.clone())
    }
}

/// How long the drawing stays valid: a few publishing periods, but never
/// less than the default.
fn drawing_stale_after(config: &UbmPathFollowerConfig) -> Duration {
    Drawing::DEFAULT_STALE_AFTER.max(Duration::from_secs_f64(3.0 / config.rate_hz as f64))
}

/// What the controllers remember between ticks - reset when the race line changes.
#[derive(Debug, Clone, Default, PartialEq)]
struct State {
    /// The PD controller's last heading error, and when it was computed.
    previous_error: Option<(f64, Instant)>,
    /// The learned feedforward, one value per race line point.
    learned: Vec<f64>,
}

/// What one tick decided.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Control {
    steering_rad: f64,
    speed_mps: f64,
    nearest: Nearest,
    /// Where the steering law measured from (the rear axle for PD and
    /// P-enhanced, the pose for Stanley), and the point it steered toward.
    from: Pose,
    target: [f64; 2],
}

/// The UBM path follower for `pose`, driving at `speed_mps`, on `line` -
/// or, if farther than `max_cross_track_m` from it, the projection that was
/// too far.
#[allow(clippy::too_many_arguments)]
fn control(
    config: &UbmPathFollowerConfig,
    line: &Line,
    pose: Pose,
    speed_mps: f64,
    hint: Option<usize>,
    limits: &ActuatorLimits,
    state: &mut State,
    now: Instant,
) -> Result<Control, Nearest> {
    let lookahead_m = config.look_ahead_gain_s * speed_mps.max(0.0) + config.min_look_ahead_m;
    let nearest = line.nearest(pose.x_m, pose.y_m, hint, 1.5 * lookahead_m + 1.0);
    if nearest.distance_m > config.max_cross_track_m {
        return Err(nearest);
    }
    let max_steering = limits.max_steering_angle_rad;

    let (steering, from, target) = if config.controller == CONTROLLER_STANLEY {
        state.previous_error = None;
        let (steering, target) = stanley(config, line, &nearest, pose, speed_mps);
        (steering, pose, target)
    } else {
        let back = pose.moved_back(config.wheelbase_m);
        let target = line.at(nearest.s_m + lookahead_m);
        let error = wrap_to_pi((target.y - back.y_m).atan2(target.x - back.x_m) - back.heading_rad);
        let steering = match config.controller {
            CONTROLLER_P_ENHANCED => {
                state.previous_error = None;
                p_enhanced(&config.gains(), error, speed_mps)
            }
            _ => pd(&config.gains(), error, &mut state.previous_error, now),
        };
        (steering, back, [target.x, target.y])
    };
    let steering = steering.clamp(-max_steering, max_steering);
    let steering_rad = steering + feedforward(config, line, &nearest, steering, max_steering, &mut state.learned);

    let speed_mps = if config.constant_speed > 0.0 {
        config.constant_speed
    } else {
        config.scale_speed * line.at(nearest.s_m).speed_mps
    };

    Ok(Control {
        steering_rad: steering_rad.clamp(-max_steering, max_steering),
        speed_mps: speed_mps.clamp(0.0, limits.max_speed_mps),
        nearest,
        from,
        target,
    })
}

/// The Stanley controller on the race line point `tdp` points past the
/// nearest one: the heading error to the line there, minus
/// `atan(k_stanley d / speed)` for the signed distance `d` from it (positive
/// toward increasing heading). Also returns the point, projected across.
fn stanley(
    config: &UbmPathFollowerConfig,
    line: &Line,
    nearest: &Nearest,
    pose: Pose,
    speed_mps: f64,
) -> (f64, [f64; 2]) {
    let n = line.points.len();
    let index = (nearest.segment + config.tdp) % n;
    let reference = line.points[index];
    let heading = line.segment_heading(index);
    let (sin, cos) = heading.sin_cos();
    let d = -sin * (pose.x_m - reference.x) + cos * (pose.y_m - reference.y);
    let cross_track = (config.k_stanley * d / speed_mps.max(STANLEY_MIN_SPEED_MPS)).atan();
    let steering = wrap_to_pi(heading - pose.heading_rad) - cross_track;
    (steering, [reference.x - d * sin, reference.y + d * cos])
}

/// The feedforward term added to `steering` (0 if disabled), clamped so the
/// sum stays within `max_steering`. The learned kind updates `learned`.
fn feedforward(
    config: &UbmPathFollowerConfig,
    line: &Line,
    nearest: &Nearest,
    steering: f64,
    max_steering: f64,
    learned: &mut Vec<f64>,
) -> f64 {
    let n = line.points.len();
    let action = match config.feedforward {
        FEEDFORWARD_PATH => {
            let curvature = line.curvature_at(nearest.s_m + config.delay_ff_action);
            config.beta_ff_gain * (curvature * config.wheelbase_m).atan()
        }
        FEEDFORWARD_LEARNED => {
            if learned.len() != n {
                *learned = vec![0.0; n];
            }
            let delay = config.delay_ff_action.max(0.0) as usize % n;
            let update = (nearest.segment + n - delay) % n;
            learned[update] = config.beta_ff_gain * (1.0 - config.averaging_ff_gain) * steering
                + config.averaging_ff_gain * learned[update];
            learned[nearest.segment]
        }
        _ => return 0.0,
    };
    if (action + steering).abs() > max_steering {
        (max_steering - steering.abs()).copysign(if steering < 0.0 { -1.0 } else { 1.0 })
    } else {
        action
    }
}

impl Control {
    /// The nearest point, the target, and the chord to it.
    fn drawing(&self) -> Drawing {
        let nearest = Shape::Circle {
            x_m: self.nearest.x_m,
            y_m: self.nearest.y_m,
            radius_m: 0.06,
            filled: true,
            color: Color::BLUE,
        };
        let target = Shape::Circle {
            x_m: self.target[0],
            y_m: self.target[1],
            radius_m: 0.08,
            filled: true,
            color: Color::PURPLE,
        };
        let chord = Shape::Polyline {
            points: vec![
                [self.from.x_m as f32, self.from.y_m as f32],
                [self.target[0] as f32, self.target[1] as f32],
            ],
            closed: false,
            width_px: 1.0,
            color: Color::PURPLE.with_alpha(128),
        };
        Drawing::default()
            .element("Nearest point", [nearest], false)
            .element("Target point", [target], false)
            .element("Chord", [chord], false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::environment::SpeedPoint;
    use std::f64::consts::PI;

    fn point(x: f64, y: f64) -> SpeedPoint {
        SpeedPoint { x, y, speed_mps: 2.0 }
    }

    fn limits() -> ActuatorLimits {
        ActuatorLimits {
            max_steering_angle_rad: 0.4,
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

    /// A 20 m x 0.5 m loop: out along y = 0, back along y = 0.5.
    fn hairpin() -> Line {
        let mut points: Vec<SpeedPoint> = (0..=100).map(|i| point(i as f64 * 0.2, 0.0)).collect();
        points.extend((0..=100).rev().map(|i| point(i as f64 * 0.2, 0.5)));
        Line::new(points).unwrap()
    }

    fn config(controller: u8) -> UbmPathFollowerConfig {
        UbmPathFollowerConfig { controller, feedforward: 0, constant_speed: 0.0, ..Default::default() }
    }

    fn run(config: &UbmPathFollowerConfig, line: &Line, pose: Pose, speed_mps: f64) -> Control {
        control(config, line, pose, speed_mps, None, &limits(), &mut State::default(), Instant::now()).unwrap()
    }

    #[test]
    fn on_a_straight_line_every_controller_steers_straight() {
        let line = hairpin();
        let pose = Pose { x_m: 5.0, y_m: 0.0, heading_rad: 0.0 };
        for controller in [CONTROLLER_PD, CONTROLLER_P_ENHANCED, CONTROLLER_STANLEY] {
            let control = run(&config(controller), &line, pose, 2.0);
            assert!(control.steering_rad.abs() < 1e-9, "{controller}: {}", control.steering_rad);
        }
    }

    #[test]
    fn on_a_circle_the_controllers_steer_toward_increasing_heading() {
        let line = circle(3.0);
        let pose = Pose { x_m: 3.0, y_m: 0.0, heading_rad: PI / 2.0 };
        for controller in [CONTROLLER_PD, CONTROLLER_P_ENHANCED, CONTROLLER_STANLEY] {
            let control = run(&config(controller), &line, pose, 2.0);
            assert!(control.steering_rad > 0.0, "{controller}: {}", control.steering_rad);
        }
    }

    #[test]
    fn stanley_steers_back_toward_the_line() {
        let line = hairpin();
        let config = UbmPathFollowerConfig { tdp: 0, ..config(CONTROLLER_STANLEY) };
        // Heading along +x, displaced toward increasing heading (+y).
        let pose = Pose { x_m: 5.0, y_m: 0.2, heading_rad: 0.0 };
        let nearest = line.nearest(pose.x_m, pose.y_m, None, 0.0);
        let (steering, target) = stanley(&config, &line, &nearest, pose, 2.0);
        assert!(steering < 0.0, "{steering}");
        assert!((target[1] - 0.2).abs() < 1e-9, "{target:?}");
    }

    #[test]
    fn path_feedforward_follows_the_curvature() {
        let radius_m = 3.0;
        let line = circle(radius_m);
        let config = UbmPathFollowerConfig { feedforward: FEEDFORWARD_PATH, delay_ff_action: 0.0, ..config(0) };
        let nearest = line.nearest(radius_m, 0.0, None, 0.0);
        let action = feedforward(&config, &line, &nearest, 0.0, 0.4, &mut Vec::new());
        let expected = config.beta_ff_gain * (config.wheelbase_m / radius_m).atan();
        assert!((action - expected).abs() < 1e-3, "{action} vs {expected}");
        // Clamped so the sum stays within the limit.
        let clamped = feedforward(&config, &line, &nearest, 0.39, 0.4, &mut Vec::new());
        assert!((clamped - 0.01).abs() < 1e-12, "{clamped}");
    }

    #[test]
    fn learned_feedforward_remembers_per_point() {
        let line = hairpin();
        let config = UbmPathFollowerConfig {
            feedforward: FEEDFORWARD_LEARNED,
            delay_ff_action: 0.0,
            beta_ff_gain: 0.5,
            averaging_ff_gain: 0.0,
            ..config(0)
        };
        let nearest = line.nearest(5.0, 0.0, None, 0.0);
        let mut learned = Vec::new();
        assert_eq!(feedforward(&config, &line, &nearest, 0.2, 0.4, &mut learned), 0.1);
        assert_eq!(learned.len(), line.points.len());
        assert_eq!(learned[nearest.segment], 0.1);
    }

    #[test]
    fn speed_follows_the_scaled_profile_or_the_constant() {
        let line = hairpin();
        let pose = Pose { x_m: 5.0, y_m: 0.0, heading_rad: 0.0 };
        let config = UbmPathFollowerConfig { scale_speed: 0.5, ..config(CONTROLLER_PD) };
        assert!((run(&config, &line, pose, 1.0).speed_mps - 1.0).abs() < 1e-9);
        let config = UbmPathFollowerConfig { constant_speed: 3.0, ..config };
        assert_eq!(run(&config, &line, pose, 1.0).speed_mps, 3.0);
    }

    #[test]
    fn too_far_from_the_line_is_an_error() {
        let line = hairpin();
        let pose = Pose { x_m: 5.0, y_m: -2.0, heading_rad: 0.0 };
        let err = control(&config(0), &line, pose, 1.0, None, &limits(), &mut State::default(), Instant::now())
            .unwrap_err();
        assert!((err.distance_m - 2.0).abs() < 1e-9);
    }

    #[test]
    fn every_config_field_is_tunable() {
        let config = UbmPathFollowerConfig::default();
        let info = AutonomousAlgorithmInfo::new("UBM Path Follower", "").with_parameters(&config, parameters());
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
