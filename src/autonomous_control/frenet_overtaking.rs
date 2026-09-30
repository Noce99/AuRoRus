//! Frenet overtaking: follows the selected map's race line like the
//! [`path_follower`](super::path_follower)'s PD or P-enhanced law,
//! and - while the LIDAR sees something on the map's free space, e.g. an
//! opponent - steers along a Frenet path around it instead (see
//! [`frenet`](crate::autonomous_control::shared::frenet)). Ported from
//! ubm's `frenet_map_based_node.cpp`, minus its ROS-only switches (the
//! external detector, map B, the basic planner).
//!
//! Fixed from ubm: the LIDAR field of view is honoured (ubm's filter let
//! every reading through), and `lidar_downsample` can't be 0 (ubm's scan
//! loop never ended). Improved: see the `frenet` module.
//!
//! The pose comes from localization or, in simulation, the ground truth -
//! see [`FrenetOvertakingConfig::pose_source`]. Without a trustworthy
//! pose, a race line, or while too far from the line, the vehicle is held
//! stopped, and why is reported in the autonomous algorithms panel (see
//! [`report_message`]). See `documentation/autonomous_algorithms.md`.

use crate::autonomous_control::shared::frenet::{
    FreeGrid, FrenetParams, Plan, Planner, stopping_speed,
};
use crate::autonomous_control::shared::race_line::{
    Line, Nearest, POSE_GROUND_TRUTH, Pose, pose, speed, wrap_to_pi,
};
use crate::autonomous_control::shared::reactive::fov_window;
use crate::autonomous_control::shared::steering::{SteeringGains, p_enhanced, pd};
use crate::autonomous_control::{Instance, ParameterTuner, load_config, report_message};
use crate::topics::{
    ActuatorLimits, AlgorithmParameter, AutonomousAlgorithmInfo, Color, Drawing, LidarScan,
    MAP_TOPIC_NAME, SelectedMap, SelectedRaceLine, Shape, VehicleGeometry, VescCommand,
};
use crate::{Captain, Executor, Ticker};
use std::any::Any;
use std::time::{Duration, Instant};

/// Entry point build.rs calls - required, with exactly this signature.
pub fn new(instance: Instance) -> Box<dyn Executor> {
    let mut config: FrenetOvertakingConfig = load_config(&instance.config_name);
    // An opponent has no localization of its own - see `Instance::opponent`.
    if instance.is_opponent() {
        config.pose_source = POSE_GROUND_TRUTH;
    }
    Box::new(FrenetOvertaking {
        id: 0,
        instance,
        config,
    })
}

/// [`FrenetOvertakingConfig::controller`]: PD on the heading error toward the lookahead point.
const CONTROLLER_PD: u8 = 0;
/// [`FrenetOvertakingConfig::controller`]: ubm's P-enhanced controller.
const CONTROLLER_P_ENHANCED: u8 = 1;

/// Every tunable parameter [`FrenetOvertaking`] needs - loaded from
/// `config/autonomous_control/frenet_overtaking.toml` at runtime (see [`load_config`]), falling
/// back to the copy compiled in (see [`Default`]). Every field can also be tuned live - see
/// [`parameters`]. The TOML file documents each one.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FrenetOvertakingConfig {
    pub rate_hz: f32,
    /// [`POSE_LOCALIZATION`](crate::autonomous_control::shared::race_line::POSE_LOCALIZATION)
    /// or [`POSE_GROUND_TRUTH`].
    pub pose_source: u8,
    /// [`CONTROLLER_PD`] or [`CONTROLLER_P_ENHANCED`].
    pub controller: u8,
    pub kk_s: f64,
    pub kd_s: f64,
    pub min_speed: f64,
    pub max_error: f64,
    pub decay_v: f64,
    pub decay_e: f64,
    pub look_ahead_gain_s: f64,
    pub min_look_ahead_m: f64,
    pub scale_speed: f64,
    pub max_cross_track_m: f64,
    pub lidar_downsample: usize,
    pub desired_fov_deg: f64,
    pub wall_clearance_m: f64,
    pub switch_on_s: f64,
    pub hysteresis_s: f64,
    pub max_road_width_m: f64,
    pub delta_road_width_m: f64,
    pub path_point_distance_m: f64,
    pub min_path_length_m: f64,
    pub max_path_length_m: f64,
    pub delta_path_length_m: f64,
    pub path_fov_deg: f64,
    pub robot_radius_m: f64,
    pub k_jerk: f64,
    pub k_length: f64,
    pub k_distance: f64,
    pub speed_decay_factor: f64,
    pub min_speed_reduction_gain: f64,
    pub decay_last_d_factor: f64,
    pub weight_last_d: f64,
    pub speed_curvature_exponent: f64,
    pub braking_margin_m: f64,
}

impl Default for FrenetOvertakingConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/autonomous_control/frenet_overtaking.toml"))
            .expect("config/autonomous_control/frenet_overtaking.toml must deserialize into FrenetOvertakingConfig")
    }
}

impl FrenetOvertakingConfig {
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

    fn frenet(&self) -> FrenetParams {
        FrenetParams {
            max_road_width_m: self.max_road_width_m,
            delta_road_width_m: self.delta_road_width_m,
            path_point_distance_m: self.path_point_distance_m,
            min_path_length_m: self.min_path_length_m,
            max_path_length_m: self.max_path_length_m.max(self.min_path_length_m),
            delta_path_length_m: self.delta_path_length_m,
            path_fov_rad: self.path_fov_deg.to_radians(),
            robot_radius_m: self.robot_radius_m,
            k_jerk: self.k_jerk,
            k_length: self.k_length,
            k_distance: self.k_distance,
            speed_decay_factor: self.speed_decay_factor,
            min_speed_reduction_gain: self.min_speed_reduction_gain,
            decay_last_d_factor: self.decay_last_d_factor,
            weight_last_d: self.weight_last_d,
            speed_curvature_exponent: self.speed_curvature_exponent,
        }
    }
}

/// The live-tunable parameters, one per [`FrenetOvertakingConfig`]
/// field - see [`ParameterTuner`]. Every step and length has a positive
/// minimum, so the path sampling always ends.
fn parameters() -> [AlgorithmParameter; 35] {
    [
        // At least a few Hz: below 1 Hz every command would be stale on arrival
        // (see `VESC_COMMAND_TIMEOUT`), holding the vehicle stopped.
        AlgorithmParameter::float("rate_hz", 5.0, 200.0, 1.0)
            .unit("Hz")
            .description("Rate at which a new control is published."),
        AlgorithmParameter::int("pose_source", 0, 1, 1)
            .description("0 = localization (only while localizing), 1 = ground truth (simulation only)."),
        AlgorithmParameter::int("controller", CONTROLLER_PD.into(), CONTROLLER_P_ENHANCED.into(), 1)
            .description("Steering law: 0 = PD, 1 = P-enhanced."),
        AlgorithmParameter::float("kk_s", 0.0, 5.0, 0.05).description("Proportional gain on the heading error."),
        AlgorithmParameter::float("kd_s", 0.0, 1.0, 0.01)
            .unit("s")
            .description("Derivative gain on the heading error (PD only)."),
        AlgorithmParameter::float("min_speed", 0.0, 10.0, 0.1)
            .unit("m/s")
            .description("Above this speed, P-enhanced steers less."),
        AlgorithmParameter::float("max_error", 0.01, 1.0, 0.01)
            .unit("rad")
            .description("Heading errors below this are damped further by P-enhanced."),
        AlgorithmParameter::float("decay_v", 0.0, 2.0, 0.05).description("How much P-enhanced damps with speed."),
        AlgorithmParameter::float("decay_e", 0.0, 2.0, 0.05)
            .description("How much P-enhanced damps small errors, per m/s above min_speed."),
        AlgorithmParameter::float("look_ahead_gain_s", 0.0, 2.0, 0.01)
            .unit("s")
            .description("Lookahead distance growth with the speed."),
        AlgorithmParameter::float("min_look_ahead_m", 0.1, 5.0, 0.05)
            .unit("m")
            .description("Lookahead distance at standstill."),
        AlgorithmParameter::float("scale_speed", 0.0, 1.5, 0.05).description("Multiplies the race line's speed profile."),
        AlgorithmParameter::float("max_cross_track_m", 0.2, 5.0, 0.1)
            .unit("m")
            .description("Farther than this from the line, the vehicle stops."),
        AlgorithmParameter::int("lidar_downsample", 1, 50, 1)
            .description("Only every this-many-th LIDAR reading is looked at."),
        AlgorithmParameter::float("desired_fov_deg", 10.0, 360.0, 5.0)
            .unit("deg")
            .description("LIDAR readings within this angle straight ahead are looked at."),
        AlgorithmParameter::float("wall_clearance_m", 0.0, 1.0, 0.05)
            .unit("m")
            .description("The map's free space is shrunk by this: hits closer to a wall aren't obstacles, paths can't go there."),
        AlgorithmParameter::float("switch_on_s", 0.0, 2.0, 0.01)
            .unit("s")
            .description("Obstacles seen for this long switch to the Frenet planner."),
        AlgorithmParameter::float("hysteresis_s", 0.0, 10.0, 0.05)
            .unit("s")
            .description("No obstacles for this long switch back to following the line."),
        AlgorithmParameter::float("max_road_width_m", 0.0, 3.0, 0.05)
            .unit("m")
            .description("Paths end at most this far either side of the line."),
        AlgorithmParameter::float("delta_road_width_m", 0.02, 1.0, 0.01)
            .unit("m")
            .description("Spacing of the sampled end offsets."),
        AlgorithmParameter::float("path_point_distance_m", 0.05, 1.0, 0.05)
            .unit("m")
            .description("Spacing of a path's points along the line."),
        AlgorithmParameter::float("min_path_length_m", 0.3, 10.0, 0.05)
            .unit("m")
            .description("Shortest sampled path."),
        AlgorithmParameter::float("max_path_length_m", 0.3, 15.0, 0.05)
            .unit("m")
            .description("Longest sampled path - also how far ahead obstacles are looked for."),
        AlgorithmParameter::float("delta_path_length_m", 0.1, 5.0, 0.05)
            .unit("m")
            .description("Spacing of the sampled path lengths."),
        AlgorithmParameter::float("path_fov_deg", 1.0, 89.0, 1.0)
            .unit("deg")
            .description("Paths ending at a steeper angle than this from the vehicle are skipped."),
        AlgorithmParameter::float("robot_radius_m", 0.05, 1.5, 0.01)
            .unit("m")
            .description("A path point closer than this to an obstacle collides."),
        AlgorithmParameter::float("k_jerk", 0.0, 1.0, 0.001).description("Path cost weight of the lateral jerk."),
        AlgorithmParameter::float("k_length", 0.0, 2.0, 0.01).description("Path cost weight of 1 / length."),
        AlgorithmParameter::float("k_distance", 0.0, 2.0, 0.001)
            .description("Path cost weight of the end offset's distance from the remembered one."),
        AlgorithmParameter::float("speed_decay_factor", 0.5, 1.0, 0.001)
            .description("Each tick no path is free, the speed is multiplied by this once more."),
        AlgorithmParameter::float("min_speed_reduction_gain", 0.0, 1.0, 0.05)
            .description("The planner never slows below this fraction of the reference speed."),
        AlgorithmParameter::float("decay_last_d_factor", 0.0, 1.0, 0.001)
            .description("The remembered end offset decays by this every tick."),
        AlgorithmParameter::float("weight_last_d", 0.0, 1.0, 0.05)
            .description("How far the remembered end offset moves toward the chosen one each tick."),
        AlgorithmParameter::float("speed_curvature_exponent", 0.0, 2.0, 0.05)
            .description("Exponent of the slowdown where the path curves more than the line."),
        AlgorithmParameter::float("braking_margin_m", 0.0, 2.0, 0.05)
            .unit("m")
            .description("When no full-length path is clear, keep a speed that stops this far short of the obstacle."),
    ]
}

struct FrenetOvertaking {
    id: u16,
    instance: Instance,
    config: FrenetOvertakingConfig,
}

/// The map's free space in use, and what it was built from: the map
/// topic's `write_count` and the clearance.
type GridCache = Option<(u64, f64, Option<FreeGrid>)>;

impl Executor for FrenetOvertaking {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_autonomous_control(
            self.id,
            &self.instance.algorithm_topics(),
            AutonomousAlgorithmInfo::new(
                "Frenet overtaking",
                "Follows the race line, and plans Frenet paths around what the LIDAR sees on the track",
            )
            .requires_race_line()
            .requires_lidar()
            .with_parameters(&self.config, parameters()),
        );
        captain.claim_drawing(self.id);
    }

    fn run(&mut self, captain: &Captain) {
        let command_topic = captain.autonomous_control(&self.instance.algorithm_topics());
        let limits_topic = captain.topic::<ActuatorLimits>(&self.instance.vehicle.vehicle_limits());
        let geometry_topic =
            captain.topic::<VehicleGeometry>(&self.instance.vehicle.vehicle_geometry());
        let drawing_topic = captain.drawing(self.id);
        let mut tuner = ParameterTuner::new(self.id, &self.instance);

        // Both derive from `rate_hz`, so are rebuilt whenever it's tuned.
        let mut ticker = Ticker::new(self.config.rate_hz as f64);
        let mut stale_after = drawing_stale_after(&self.config);

        // The race line in use, and the `write_count` it was read at.
        let mut line: Option<(u64, Option<Line>)> = None;
        let mut grid: GridCache = None;
        // Segment the vehicle was nearest last tick - `None` searches the whole line.
        let mut hint: Option<usize> = None;
        let mut state = State::default();

        while captain.is_running(self.id) {
            if tuner.update(captain, &mut self.config) {
                ticker = Ticker::new(self.config.rate_hz as f64);
                stale_after = drawing_stale_after(&self.config);
            }

            // Nothing may publish a race line or a map at all (e.g. a binary without `MapServer`).
            if let Some(topic) =
                captain.try_topic::<SelectedRaceLine>(&self.instance.vehicle.race_line())
            {
                let write_count = topic.meta().write_count;
                if line.as_ref().is_none_or(|(seen, _)| *seen != write_count) {
                    line = Some((write_count, Line::new(topic.read().into_value().points)));
                    hint = None;
                    state = State::default();
                }
            }
            if let Some(topic) = captain.try_topic::<SelectedMap>(MAP_TOPIC_NAME) {
                let write_count = topic.meta().write_count;
                let clearance = self.config.wall_clearance_m;
                if grid.as_ref().is_none_or(|(seen, built_for, _)| {
                    *seen != write_count || *built_for != clearance
                }) {
                    grid = Some((
                        write_count,
                        clearance,
                        FreeGrid::new(&topic.read().into_value(), clearance),
                    ));
                }
            }
            let line = line.as_ref().and_then(|(_, line)| line.as_ref());
            let grid = grid.as_ref().and_then(|(_, _, grid)| grid.as_ref());
            let pose =
                pose(captain, &self.instance.vehicle, self.config.pose_source).and_then(|pose| {
                    Ok((
                        pose,
                        speed(captain, &self.instance.vehicle, self.config.pose_source)?,
                    ))
                });
            let scan = captain
                .try_topic::<LidarScan>(&self.instance.vehicle.lidar_scan())
                .map(|topic| topic.read())
                .filter(|scan| scan.meta.written_at.is_some())
                .map(|scan| scan.into_value());

            // A stationary command, and why, unless driving.
            let stopped = |why: String| (VescCommand::new(0.0, 0.0), Drawing::default(), Some(why));
            let (command, drawing, message) = match (line, pose) {
                (None, _) => {
                    stopped("No race line on the selected map - vehicle held stopped.".into())
                }
                (Some(_), Err(why)) => stopped(format!("{why} Vehicle held stopped.")),
                (Some(line), Ok((pose, speed_mps))) => {
                    let obstacles = match (grid, &scan) {
                        (Some(grid), Some(scan)) => find_obstacles(&self.config, scan, pose, grid),
                        _ => Vec::new(),
                    };
                    let limits = limits_topic.read();
                    let input = Input {
                        line,
                        grid,
                        obstacles: &obstacles,
                        pose,
                        speed_mps,
                        hint,
                        limits: &limits,
                        wheelbase_m: geometry_topic.read().wheelbase_m,
                    };
                    match control(&self.config, &input, &mut state, Instant::now()) {
                        Ok(control) => {
                            hint = Some(control.nearest.segment);
                            let message = match (grid, &scan, &control.plan) {
                                (None, _, _) => Some(
                                    "No map loaded - following the race line, not overtaking."
                                        .into(),
                                ),
                                (_, None, _) => Some(
                                    "No LIDAR scan yet - following the race line, not overtaking."
                                        .into(),
                                ),
                                (_, _, Some(plan)) if !plan.free => Some(format!(
                                    "Frenet: no free path around {} obstacle points - slowing down.",
                                    obstacles.len()
                                )),
                                (_, _, Some(_)) => Some(format!(
                                    "Frenet: avoiding {} obstacle points.",
                                    obstacles.len()
                                )),
                                (_, _, None) => None,
                            };
                            (
                                VescCommand::new(control.steering_rad, control.speed_mps),
                                control.drawing(&obstacles),
                                message,
                            )
                        }
                        Err(nearest) => {
                            // Lost: next tick searches the whole line again.
                            hint = None;
                            state = State::default();
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
                .expect("lost writer authorization for the Frenet overtaking drawing topic");
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
fn drawing_stale_after(config: &FrenetOvertakingConfig) -> Duration {
    Drawing::DEFAULT_STALE_AFTER.max(Duration::from_secs_f64(3.0 / config.rate_hz as f64))
}

/// The world points of `scan`'s readings, taken from `pose`, that land on
/// `grid`'s free space: every `lidar_downsample`-th reading within
/// `desired_fov_deg` straight ahead, closer than `max_path_length_m` - what
/// the map doesn't know about, e.g. an opponent.
fn find_obstacles(
    config: &FrenetOvertakingConfig,
    scan: &LidarScan,
    pose: Pose,
    grid: &FreeGrid,
) -> Vec<[f64; 2]> {
    let (x_m, y_m) = scan.origin_m(pose.x_m, pose.y_m, pose.heading_rad);
    let pose = Pose { x_m, y_m, ..pose };
    let window = fov_window(scan, config.desired_fov_deg.to_radians() as f32);
    window
        .step_by(config.lidar_downsample.max(1))
        .filter_map(|i| {
            let range = f64::from(scan.points[i]);
            if range <= f64::from(scan.min_distance)
                || range >= f64::from(scan.max_distance)
                || range >= config.max_path_length_m
            {
                return None;
            }
            let angle = pose.heading_rad + f64::from(scan.angle_rad(i));
            let point = [
                pose.x_m + range * angle.cos(),
                pose.y_m + range * angle.sin(),
            ];
            grid.is_free(point[0], point[1]).then_some(point)
        })
        .collect()
}

/// Whether the line is being followed or the Frenet planner is in charge,
/// and since when the obstacles have been there (or gone) while waiting to
/// switch.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct Switch {
    avoiding: bool,
    since: Option<Instant>,
}

impl Switch {
    /// Switches to avoiding once obstacles have been seen for `switch_on_s`,
    /// back once none have been for `hysteresis_s`. Returns whether it just
    /// started avoiding.
    fn update(
        &mut self,
        obstacles: bool,
        now: Instant,
        switch_on_s: f64,
        hysteresis_s: f64,
    ) -> bool {
        // Waiting to switch while obstacles are there and not avoiding, or
        // gone while avoiding.
        if obstacles == self.avoiding {
            self.since = None;
            return false;
        }
        let since = *self.since.get_or_insert(now);
        let wait_s = if self.avoiding {
            hysteresis_s
        } else {
            switch_on_s
        };
        if now.saturating_duration_since(since).as_secs_f64() < wait_s {
            return false;
        }
        self.avoiding = !self.avoiding;
        self.since = None;
        self.avoiding
    }
}

/// What the algorithm remembers between ticks - reset when the race line
/// changes or the vehicle gets lost.
#[derive(Debug, Clone, Default, PartialEq)]
struct State {
    switch: Switch,
    planner: Planner,
    /// The PD controller's last heading error, and when it was computed.
    previous_error: Option<(f64, Instant)>,
}

/// What one tick works from.
struct Input<'a> {
    line: &'a Line,
    /// `None` without a map: then there are never obstacles.
    grid: Option<&'a FreeGrid>,
    obstacles: &'a [[f64; 2]],
    pose: Pose,
    speed_mps: f64,
    hint: Option<usize>,
    limits: &'a ActuatorLimits,
    /// The vehicle's, between its axles, in meters.
    wheelbase_m: f64,
}

/// What one tick decided.
#[derive(Debug, Clone, PartialEq)]
struct Control {
    steering_rad: f64,
    speed_mps: f64,
    nearest: Nearest,
    /// Where the steering law measured from (the rear axle), and the point
    /// it steered toward.
    from: Pose,
    target: [f64; 2],
    /// The Frenet plan, while avoiding.
    plan: Option<Plan>,
}

/// One tick: along the line, or along a Frenet path while avoiding - or, if
/// farther than `max_cross_track_m` from the line, the projection that was
/// too far.
fn control(
    config: &FrenetOvertakingConfig,
    input: &Input,
    state: &mut State,
    now: Instant,
) -> Result<Control, Nearest> {
    let line = input.line;
    let lookahead_m = config.look_ahead_gain_s * input.speed_mps.max(0.0) + config.min_look_ahead_m;
    let nearest = line.nearest(
        input.pose.x_m,
        input.pose.y_m,
        input.hint,
        1.5 * lookahead_m + 1.0,
    );
    if nearest.distance_m > config.max_cross_track_m {
        return Err(nearest);
    }

    if state.switch.update(
        !input.obstacles.is_empty(),
        now,
        config.switch_on_s,
        config.hysteresis_s,
    ) {
        state.planner = Planner::default();
    }
    let reference_mps = config.scale_speed * line.at(nearest.s_m).speed_mps;
    let (target, speed_mps, plan) = match input.grid {
        Some(grid) if state.switch.avoiding => {
            let (d0, slope0) = line.lateral(input.pose, &nearest);
            let plan = state.planner.plan(
                &config.frenet(),
                line,
                grid,
                input.obstacles,
                nearest.s_m,
                d0,
                slope0,
                lookahead_m,
            );
            let mut speed_mps = reference_mps * plan.speed_gain;
            if let Some(reach_m) = plan.reach_m {
                speed_mps = speed_mps.min(stopping_speed(
                    reach_m,
                    config.braking_margin_m,
                    input.limits.max_decel_mps2,
                ));
            }
            (plan.target, speed_mps, Some(plan))
        }
        _ => {
            let target = line.at(nearest.s_m + lookahead_m);
            ([target.x, target.y], reference_mps, None)
        }
    };

    let back = input.pose.moved_back(input.wheelbase_m);
    let error = wrap_to_pi((target[1] - back.y_m).atan2(target[0] - back.x_m) - back.heading_rad);
    let steering = match config.controller {
        CONTROLLER_P_ENHANCED => {
            state.previous_error = None;
            p_enhanced(&config.gains(), error, input.speed_mps)
        }
        _ => pd(&config.gains(), error, &mut state.previous_error, now),
    };
    let max_steering = input.limits.max_steering_angle_rad;

    Ok(Control {
        steering_rad: steering.clamp(-max_steering, max_steering),
        speed_mps: speed_mps.clamp(0.0, input.limits.max_speed_mps),
        nearest,
        from: back,
        target,
        plan,
    })
}

/// A polyline through `x`/`y`.
fn polyline(x: &[f64], y: &[f64], width_px: f32, color: Color) -> Shape {
    Shape::Polyline {
        points: x
            .iter()
            .zip(y)
            .map(|(&x, &y)| [x as f32, y as f32])
            .collect(),
        closed: false,
        width_px,
        color,
    }
}

impl Control {
    /// The obstacles, the Frenet paths (while avoiding), the nearest point,
    /// the target, and the chord to it.
    fn drawing(&self, obstacles: &[[f64; 2]]) -> Drawing {
        let obstacles = Shape::Points {
            points: obstacles
                .iter()
                .map(|&[x, y]| [x as f32, y as f32])
                .collect(),
            radius_px: 3.0,
            color: Color::RED,
        };
        let (candidates, chosen) = match &self.plan {
            Some(plan) => {
                let best = plan.best();
                let color = if plan.free {
                    Color::GREEN
                } else {
                    Color::AMBER
                };
                (
                    plan.candidates
                        .iter()
                        .map(|path| polyline(&path.x, &path.y, 1.0, Color::WHITE.with_alpha(50)))
                        .collect(),
                    vec![polyline(&best.x, &best.y, 3.0, color)],
                )
            }
            None => (Vec::new(), Vec::new()),
        };
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
        let chord = polyline(
            &[self.from.x_m, self.target[0]],
            &[self.from.y_m, self.target[1]],
            1.0,
            Color::PURPLE.with_alpha(128),
        );
        Drawing::default()
            .element("Obstacles", [obstacles], true)
            .element("Candidate paths", candidates, false)
            .element("Chosen path", chosen, true)
            .element("Nearest point", [nearest], false)
            .element("Target point", [target], false)
            .element("Chord", [chord], false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::environment::SpeedPoint;

    fn limits() -> ActuatorLimits {
        ActuatorLimits {
            max_steering_angle_rad: 0.4,
            max_steering_rate_rad_s: 4.0,
            max_speed_mps: 10.0,
            max_accel_mps2: 4.0,
            max_decel_mps2: 8.0,
        }
    }

    /// A 40 m x 6 m loop: out along y = 0, back along y = 6.
    fn loop_line() -> Line {
        let point = |x: f64, y: f64| SpeedPoint {
            x,
            y,
            speed_mps: 4.0,
        };
        let mut points: Vec<SpeedPoint> = (0..=200).map(|i| point(i as f64 * 0.2, 0.0)).collect();
        points.extend((0..=200).rev().map(|i| point(i as f64 * 0.2, 6.0)));
        Line::new(points).unwrap()
    }

    /// A map free within 1.5 m of y = 0 between x = 0 and 40, walls elsewhere.
    fn corridor_map() -> SelectedMap {
        use crate::environment::{ImageOrigin, MapInfo, MapSource, StartFinishLine, WorldPoint};
        let resolution = 0.05;
        let (width, height) = (800u32, 100u32);
        let origin = ImageOrigin {
            x: 0.0,
            y: -2.5,
            theta_rad: 0.0,
        };
        let pixels: Vec<u8> = (0..height)
            .flat_map(|row| {
                (0..width).map(move |_| {
                    let y = origin.y + (row as f64 + 0.5) * resolution;
                    if y.abs() < 1.5 { 255 } else { 0 }
                })
            })
            .collect();
        SelectedMap {
            path: None,
            width_px: width,
            height_px: height,
            pixels: pixels.into(),
            info: Some(MapInfo {
                resolution_m_per_px: resolution,
                width_px: width,
                height_px: height,
                origin,
                start_finish_line: StartFinishLine {
                    a: WorldPoint { x: 1.0, y: 1.0 },
                    b: WorldPoint { x: 1.0, y: -1.0 },
                },
                generated_at: String::new(),
                source: MapSource::Real,
                generation: None,
            }),
        }
    }

    /// A 360-degree scan from a pose heading +x at (5, 0), seeing the
    /// corridor's walls at y = +-1.5 and a box across y in [-0.2, 0.2] at x = 7.
    fn scan() -> LidarScan {
        let n = 721;
        let fov = std::f32::consts::TAU * 720.0 / 721.0;
        let points: Vec<f32> = (0..n)
            .map(|i| {
                let angle = LidarScan::ray_angle_rad(fov, n, i) as f64;
                let (sin, cos) = angle.sin_cos();
                let mut range = 10.0f64;
                if sin.abs() > 1e-6 {
                    range = range.min(1.5 / sin.abs());
                }
                if cos > 1e-6 {
                    let to_box = 2.0 / cos;
                    if (to_box * sin).abs() <= 0.2 {
                        range = range.min(to_box);
                    }
                }
                range as f32
            })
            .collect();
        LidarScan::new(points, vec![0.0; n], 0.05, 10.0, fov)
    }

    fn config() -> FrenetOvertakingConfig {
        FrenetOvertakingConfig {
            lidar_downsample: 1,
            ..Default::default()
        }
    }

    #[test]
    fn only_readings_on_the_free_space_are_obstacles() {
        let grid = FreeGrid::new(&corridor_map(), 0.2).unwrap();
        let pose = Pose {
            x_m: 5.0,
            y_m: 0.0,
            heading_rad: 0.0,
        };
        let obstacles = find_obstacles(&config(), &scan(), pose, &grid);
        assert!(!obstacles.is_empty());
        for [x, y] in &obstacles {
            assert!(
                (x - 7.0).abs() < 0.05 && y.abs() <= 0.21,
                "({x}, {y}) isn't the box"
            );
        }
        // The box spans about +-5.7 degrees: a 6 degree field of view sees
        // only its middle.
        let narrow = FrenetOvertakingConfig {
            desired_fov_deg: 6.0,
            ..config()
        };
        let seen = find_obstacles(&narrow, &scan(), pose, &grid);
        assert!(
            !seen.is_empty() && seen.len() < obstacles.len(),
            "{} of {}",
            seen.len(),
            obstacles.len()
        );
        assert!(seen.iter().all(|[_, y]| y.abs() < 0.11));
    }

    #[test]
    fn the_switch_waits_before_switching_either_way() {
        let mut switch = Switch::default();
        let start = Instant::now();
        let at = |s: f64| start + Duration::from_secs_f64(s);
        assert!(!switch.update(true, at(0.0), 0.07, 2.0));
        assert!(!switch.update(true, at(0.05), 0.07, 2.0));
        assert!(switch.update(true, at(0.08), 0.07, 2.0));
        assert!(switch.avoiding);
        // Obstacles briefly gone don't switch back...
        assert!(!switch.update(false, at(1.0), 0.07, 2.0));
        assert!(!switch.update(true, at(1.5), 0.07, 2.0));
        assert!(!switch.update(false, at(2.0), 0.07, 2.0));
        assert!(switch.avoiding);
        // ...gone for long enough do.
        assert!(!switch.update(false, at(4.1), 0.07, 2.0));
        assert!(!switch.avoiding);
    }

    #[test]
    fn it_steers_around_an_obstacle_on_the_line() {
        let line = loop_line();
        let grid = FreeGrid::new(&corridor_map(), 0.2).unwrap();
        let pose = Pose {
            x_m: 5.0,
            y_m: 0.0,
            heading_rad: 0.0,
        };
        let obstacles = find_obstacles(&config(), &scan(), pose, &grid);
        let input = Input {
            line: &line,
            grid: Some(&grid),
            obstacles: &obstacles,
            pose,
            speed_mps: 2.0,
            hint: None,
            limits: &limits(),
            wheelbase_m: 0.32,
        };
        let mut state = State::default();
        let start = Instant::now();
        let following = control(&config(), &input, &mut state, start).unwrap();
        assert!(following.plan.is_none());
        assert!(following.steering_rad.abs() < 1e-9);

        let avoiding = control(
            &config(),
            &input,
            &mut state,
            start + Duration::from_millis(100),
        )
        .unwrap();
        let plan = avoiding.plan.as_ref().expect("switched to the planner");
        assert!(plan.free);
        assert!(
            avoiding.steering_rad.abs() > 0.01,
            "{}",
            avoiding.steering_rad
        );
        assert!(avoiding.speed_mps < config().scale_speed * 4.0);
    }

    #[test]
    fn without_a_map_it_just_follows_the_line() {
        let line = loop_line();
        let obstacles = [[7.0, 0.0]];
        let input = Input {
            line: &line,
            grid: None,
            obstacles: &obstacles,
            pose: Pose {
                x_m: 5.0,
                y_m: 0.0,
                heading_rad: 0.0,
            },
            speed_mps: 2.0,
            hint: None,
            limits: &limits(),
            wheelbase_m: 0.32,
        };
        let mut state = State::default();
        let start = Instant::now();
        control(&config(), &input, &mut state, start).unwrap();
        let control = control(
            &config(),
            &input,
            &mut state,
            start + Duration::from_secs(1),
        )
        .unwrap();
        assert!(control.plan.is_none());
        assert!((control.speed_mps - config().scale_speed * 4.0).abs() < 1e-9);
    }

    #[test]
    fn too_far_from_the_line_is_an_error() {
        let line = loop_line();
        let input = Input {
            line: &line,
            grid: None,
            obstacles: &[],
            pose: Pose {
                x_m: 5.0,
                y_m: -2.0,
                heading_rad: 0.0,
            },
            speed_mps: 1.0,
            hint: None,
            limits: &limits(),
            wheelbase_m: 0.32,
        };
        let err = control(&config(), &input, &mut State::default(), Instant::now()).unwrap_err();
        assert!((err.distance_m - 2.0).abs() < 1e-9);
    }

    #[test]
    fn every_config_field_is_tunable() {
        let config = FrenetOvertakingConfig::default();
        let info = AutonomousAlgorithmInfo::new("Frenet overtaking", "")
            .with_parameters(&config, parameters());
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
