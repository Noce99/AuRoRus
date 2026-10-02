//! MPC: follows the selected map's race line (the
//! [`RACE_LINE_TOPIC_NAME`] topic) with a model predictive controller over
//! a kinematic bicycle - see [`mpc`](crate::autonomous_control::shared::mpc).
//! Solved with PANOC.
//!
//! The speed is capped by `max_speed_gain`, the steering by the vehicle's
//! limit, and every weight can be tuned live. The walls term is always on,
//! opponent or not, from the map's distance field.
//!
//! The opponent is the one [`UbmDetector`](crate::perception::UbmDetector)
//! publishes on [`DETECTED_OPPONENT_TOPIC_NAME`] - the closest, with its
//! `selection` at 1 - while it's detected and fresh, and only for the ego
//! vehicle, whose lidar the detector looks at.
//!
//! The pose comes from localization or, in simulation, the ground truth -
//! see [`MpcConfig::pose_source`]. Without a trustworthy pose, a race
//! line, a solution, or while too far from the line, the vehicle is held
//! stopped, and why is reported in the autonomous algorithms panel (see
//! [`report_message`]). See `documentation/autonomous_algorithms.md`.

use crate::autonomous_control::shared::mpc::{
    Bounds, Cache, DistanceField, MIN_HORIZON, Mpc as MpcProblem, Opponent, Solution,
    SolverSettings, State as MpcState, Target, Walls, Weights, solve,
};
use crate::autonomous_control::{
    AutonomousControlExt, Instance, ParameterTuner, load_config, report_message, report_stats,
};
use crate::geometry::{Line, Nearest, Pose};
use crate::localization::pose_source::{POSE_GROUND_TRUTH, pose, speed};
use crate::topics::{
    ActuatorLimits, AlgorithmParameter, AutonomousAlgorithmInfo, Color,
    DETECTED_OPPONENT_TOPIC_NAME, DetectedOpponent, Drawing, DrawingExt, MAP_TOPIC_NAME,
    SelectedMap, SelectedRaceLine, Shape, VehicleGeometry, VescCommand,
};
use crate::{Captain, Executor, Ticker};
use std::any::Any;
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Entry point build.rs calls - required, with exactly this signature.
pub fn new(instance: Instance) -> Box<dyn Executor> {
    let mut config: MpcConfig = load_config(&instance.config_name);
    // An opponent has no localization of its own - see `Instance::opponent`.
    if instance.is_opponent() {
        config.pose_source = POSE_GROUND_TRUTH;
    }
    Box::new(Mpc {
        id: 0,
        instance,
        config,
    })
}

/// A speed below this is never commanded, in m/s.
const MIN_COMMAND_SPEED_MPS: f64 = 0.1;

/// The first steering of a solve is what the vehicle already steers,
/// but never more than this fraction of the limit, so the solver can
/// always turn back.
const FIRST_STEERING_MARGIN: f64 = 0.9;

/// Every tunable parameter [`Mpc`] needs - loaded from
/// `config/autonomous_control/mpc.toml` at runtime (see [`load_config`]), falling back to the
/// copy compiled in (see [`Default`]). Every field can also be tuned live - see [`parameters`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct MpcConfig {
    /// Rate at which a new control is published, in Hz.
    pub rate_hz: f32,
    /// Where the pose and speed come from: [`POSE_LOCALIZATION`](crate::localization::pose_source::POSE_LOCALIZATION)
    /// or [`POSE_GROUND_TRUTH`](crate::localization::pose_source::POSE_GROUND_TRUTH).
    pub pose_source: u8,
    /// Steps in the prediction, the vehicle's own pose included.
    pub horizon: usize,
    /// Arc length of one prediction step, in meters.
    pub step_m: f64,
    /// How far into the horizon, in percent, the vehicle must be along the
    /// previous prediction before solving again from it.
    pub percentage_of_mpc_prediction_to_follow: f64,
    /// See [`Weights::distance`].
    pub distance_weight: f64,
    /// See [`Weights::steering_smoothness`].
    pub steering_smoothness_weight: f64,
    /// See [`Weights::go_fast`].
    pub go_fast_weight: f64,
    /// See [`Weights::speed_smoothness`].
    pub speed_smoothness_weight: f64,
    /// See [`Weights::centripetal`].
    pub centripetal_weight: f64,
    /// Lowest speed, as a multiple of the race line's, pure number.
    pub min_speed_gain: f64,
    /// Highest speed, as a multiple of the race line's, pure number.
    pub max_speed_gain: f64,
    /// Multiplies the race line's speed profile, pure number.
    pub scale_speed: f64,
    /// Farther than this from the line, the vehicle stops, in meters.
    pub max_cross_track_m: f64,
    /// PANOC's tolerance on its fixed-point residual.
    pub solver_tolerance: f64,
    /// Most PANOC iterations per solve.
    pub solver_max_iterations: usize,
    /// Longest a solve may take, in milliseconds.
    pub solver_max_ms: f64,
    /// See [`Weights::walls`].
    pub walls_cost_weight: f64,
    /// Closer than this to a wall costs, in meters - see [`Walls`].
    pub walls_margin_m: f64,
    /// See [`Weights::opponent`].
    pub opponent_distance_weight: f64,
    /// How far the opponent cost reaches, in meters - see [`Opponent`].
    pub opponent_radius_m: f64,
    /// Opponent cost: 1 = Gaussian, 0 = inverse square - see [`Opponent`].
    pub use_gaussian: u8,
    /// How long after its last detection the detector's prediction of the
    /// opponent is still used, in seconds.
    pub opponent_timeout_s: f64,
}

impl Default for MpcConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/autonomous_control/mpc.toml"))
            .expect("config/autonomous_control/mpc.toml must deserialize into MpcConfig")
    }
}

impl MpcConfig {
    fn weights(&self) -> Weights {
        Weights {
            distance: self.distance_weight,
            steering_smoothness: self.steering_smoothness_weight,
            speed_smoothness: self.speed_smoothness_weight,
            go_fast: self.go_fast_weight,
            centripetal: self.centripetal_weight,
            walls: self.walls_cost_weight,
            opponent: self.opponent_distance_weight,
        }
    }

    /// The detector's opponent, if it was detected within
    /// `opponent_timeout_s` (the latest detection's time kept in
    /// `last_detected`): the detection itself, or the detector's prediction
    /// while a few scans miss it.
    fn opponent(
        &self,
        detected: &crate::Stamped<DetectedOpponent>,
        last_detected: &mut Option<Instant>,
    ) -> Option<Opponent> {
        if detected.detected {
            *last_detected = detected.meta.written_at.or(*last_detected);
        }
        let recent = |at: Option<Instant>| {
            at.is_some_and(|at| at.elapsed().as_secs_f64() <= self.opponent_timeout_s)
        };
        (recent(detected.meta.written_at) && recent(*last_detected)).then(|| Opponent {
            position: detected.position,
            velocity: detected.velocity,
            radius_m: self.opponent_radius_m,
            gaussian: self.use_gaussian != 0,
        })
    }

    fn solver(&self) -> SolverSettings {
        SolverSettings {
            tolerance: self.solver_tolerance,
            max_iterations: self.solver_max_iterations,
            max_duration: Duration::from_secs_f64(self.solver_max_ms.max(0.1) / 1000.0),
        }
    }

    /// The horizon, never below what the cost needs.
    fn horizon(&self) -> usize {
        self.horizon.max(MIN_HORIZON)
    }
}

/// The live-tunable parameters, one per [`MpcConfig`] field - see [`ParameterTuner`].
fn parameters() -> [AlgorithmParameter; 23] {
    [
        // At least a few Hz: below 1 Hz every command would be stale on arrival
        // (see `VESC_COMMAND_TIMEOUT`), holding the vehicle stopped.
        AlgorithmParameter::float("rate_hz", 5.0, 200.0, 1.0)
            .unit("Hz")
            .description("Rate at which a new control is published."),
        AlgorithmParameter::int("pose_source", 0, 1, 1).description(
            "0 = localization (only while localizing), 1 = ground truth (simulation only).",
        ),
        AlgorithmParameter::int("horizon", MIN_HORIZON as i64, 60, 1)
            .unit("steps")
            .description("Steps in the prediction, the vehicle's own pose included."),
        AlgorithmParameter::float("step_m", 0.05, 1.0, 0.01)
            .unit("m")
            .description("Arc length of one prediction step."),
        AlgorithmParameter::float("percentage_of_mpc_prediction_to_follow", 0.0, 100.0, 0.5)
            .unit("%")
            .description("How far into the prediction the vehicle must be before solving again."),
        AlgorithmParameter::float("distance_weight", 0.0, 100.0, 0.1)
            .description("Cost of the prediction's squared distance from the race line."),
        AlgorithmParameter::float("steering_smoothness_weight", 0.0, 2000.0, 1.0)
            .description("Cost of the steering's second difference."),
        AlgorithmParameter::float("go_fast_weight", 0.0, 10.0, 0.01)
            .description("Cost of the deviation from the race line's speed."),
        AlgorithmParameter::float("speed_smoothness_weight", 0.0, 100.0, 0.1)
            .description("Cost of the speed's second difference."),
        AlgorithmParameter::float("centripetal_weight", 0.0, 10.0, 0.001)
            .description("Cost of the heading change times the speed, squared."),
        AlgorithmParameter::float("min_speed_gain", 0.0, 2.0, 0.05)
            .description("Lowest speed, as a multiple of the race line's."),
        AlgorithmParameter::float("max_speed_gain", 0.0, 5.0, 0.05)
            .description("Highest speed, as a multiple of the race line's."),
        AlgorithmParameter::float("scale_speed", 0.0, 1.5, 0.05)
            .description("Multiplies the race line's speed profile."),
        AlgorithmParameter::float("max_cross_track_m", 0.2, 5.0, 0.1)
            .unit("m")
            .description("Farther than this from the line, the vehicle stops."),
        AlgorithmParameter::float("solver_tolerance", 1e-8, 1e-2, 1e-6)
            .description("PANOC's tolerance on its fixed-point residual."),
        AlgorithmParameter::int("solver_max_iterations", 1, 5000, 10)
            .description("Most PANOC iterations per solve."),
        AlgorithmParameter::float("solver_max_ms", 0.5, 100.0, 0.5)
            .unit("ms")
            .description("Longest a solve may take."),
        AlgorithmParameter::float("walls_cost_weight", 0.0, 100.0, 0.5)
            .description("Cost of the prediction's squared depth into walls_margin_m."),
        AlgorithmParameter::float("walls_margin_m", 0.0, 1.5, 0.01)
            .unit("m")
            .description("Closer than this to a wall costs."),
        AlgorithmParameter::float("opponent_distance_weight", 0.0, 200.0, 0.5)
            .description("Cost of the prediction's closeness to the opponent."),
        AlgorithmParameter::float("opponent_radius_m", 0.05, 3.0, 0.05)
            .unit("m")
            .description("How far the opponent cost reaches (Gaussian only)."),
        AlgorithmParameter::int("use_gaussian", 0, 1, 1)
            .description("Opponent cost: 1 = exp(-4 distance / radius), 0 = 1 / distance²."),
        AlgorithmParameter::float("opponent_timeout_s", 0.0, 2.0, 0.05)
            .unit("s")
            .description(
                "How long after its last detection the opponent's prediction is still used.",
            ),
    ]
}

struct Mpc {
    id: u16,
    instance: Instance,
    config: MpcConfig,
}

impl Executor for Mpc {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_autonomous_control(
            self.id,
            &self.instance.algorithm_topics(),
            AutonomousAlgorithmInfo::new(
                "MPC",
                "Follows the race line with a model predictive controller over a kinematic bicycle",
            )
            .requires_race_line()
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
        // The map's distance field, and the `write_count` it was built at.
        let mut field: Option<(u64, Option<Arc<DistanceField>>)> = None;
        // When the detector last saw the opponent.
        let mut last_detected: Option<Instant> = None;
        let mut state = State::default();

        while captain.is_running(self.id) {
            if tuner.update(captain, &mut self.config) {
                ticker = Ticker::new(self.config.rate_hz as f64);
                stale_after = drawing_stale_after(&self.config);
            }

            // Nothing may publish a race line at all (e.g. a binary without `MapServer`).
            if let Some(topic) =
                captain.try_topic::<SelectedRaceLine>(&self.instance.vehicle.race_line())
            {
                let write_count = topic.meta().write_count;
                if line.as_ref().is_none_or(|(seen, _)| *seen != write_count) {
                    line = Some((write_count, Line::new(topic.read().into_value().points)));
                    state = State::default();
                }
            }
            if let Some(topic) = captain.try_topic::<SelectedMap>(MAP_TOPIC_NAME) {
                let write_count = topic.meta().write_count;
                if field.as_ref().is_none_or(|(seen, _)| *seen != write_count) {
                    let built = DistanceField::new(&topic.read().into_value()).map(Arc::new);
                    field = Some((write_count, built));
                }
            }
            let line = line.as_ref().and_then(|(_, line)| line.as_ref());
            let field = field.as_ref().and_then(|(_, field)| field.clone());
            // Only the ego vehicle's lidar is the detector's.
            let opponent = (!self.instance.is_opponent())
                .then(|| captain.try_topic::<DetectedOpponent>(DETECTED_OPPONENT_TOPIC_NAME))
                .flatten()
                .and_then(|topic| self.config.opponent(&topic.read(), &mut last_detected));
            let pose =
                pose(captain, &self.instance.vehicle, self.config.pose_source).and_then(|pose| {
                    Ok((
                        pose,
                        speed(captain, &self.instance.vehicle, self.config.pose_source)?,
                    ))
                });

            // A stationary command, and why, unless following the line.
            let stopped = |why: String| (VescCommand::new(0.0, 0.0), Drawing::default(), Some(why));
            let (command, drawing, message) = match (line, pose) {
                (None, _) => {
                    stopped("No race line on the selected map - vehicle held stopped.".into())
                }
                (Some(_), Err(why)) => stopped(format!("{why} Vehicle held stopped.")),
                (Some(line), Ok((pose, _speed_mps))) => {
                    let limits = limits_topic.read();
                    let input = Input {
                        line,
                        pose,
                        limits: &limits,
                        wheelbase_m: geometry_topic.read().wheelbase_m,
                        walls: field.map(|field| Walls {
                            field,
                            margin_m: self.config.walls_margin_m,
                        }),
                        opponent,
                    };
                    match control(&self.config, &input, &mut state) {
                        Ok(control) => {
                            let message = (!state
                                .plan
                                .as_ref()
                                .is_some_and(|plan| plan.solution.converged))
                            .then(|| {
                                "MPC did not converge in time - following its best iterate."
                                    .to_string()
                            });
                            let drawing = state.drawing(&control, limits.max_speed_mps);
                            (
                                VescCommand::new(control.steering_rad, control.speed_mps),
                                drawing,
                                message,
                            )
                        }
                        Err(why) => {
                            state = State::default();
                            stopped(format!("{why} Vehicle held stopped."))
                        }
                    }
                }
            };

            report_message(captain, self.id, &self.instance, message);
            report_stats(captain, self.id, &self.instance, state.stats());
            command_topic
                .write(self.id, command)
                .expect("lost writer authorization for this algorithm's command topic");
            drawing_topic
                .write(self.id, drawing.stale_after(stale_after))
                .expect("lost writer authorization for the MPC drawing topic");
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
fn drawing_stale_after(config: &MpcConfig) -> Duration {
    Drawing::DEFAULT_STALE_AFTER.max(Duration::from_secs_f64(3.0 / config.rate_hz as f64))
}

/// The latest solve, and what it was solved for.
#[derive(Debug, Clone, PartialEq)]
struct Plan {
    solution: Solution,
    targets: Vec<Target>,
    /// Where the initial guess led, if the solve started from scratch
    /// rather than from the previous solution.
    initialization: Option<Vec<MpcState>>,
    /// Where the opponent was predicted to be as the vehicle reached each
    /// state after the start - empty without one.
    opponent: Vec<[f64; 2]>,
    solve_ms: f64,
}

/// What one tick works from.
struct Input<'a> {
    line: &'a Line,
    pose: Pose,
    limits: &'a ActuatorLimits,
    /// The vehicle's, between its axles, in meters.
    wheelbase_m: f64,
    /// `None` without a map.
    walls: Option<Walls>,
    /// `None` without a (fresh) detection.
    opponent: Option<Opponent>,
}

/// What the algorithm remembers between ticks - reset when the race line
/// changes or the vehicle gets lost.
#[derive(Default)]
struct State {
    /// The race line segment the vehicle was nearest last tick.
    hint: Option<usize>,
    plan: Option<Plan>,
    /// The last commanded steering - the next solve's first one.
    last_steering_rad: f64,
    cache: Cache,
}

/// What one tick decided.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Control {
    steering_rad: f64,
    speed_mps: f64,
    nearest: Nearest,
}

/// One tick of the control loop: solve again if there's no prediction
/// to follow, the vehicle strayed more than a step from it, it's far enough
/// along it, or there's an opponent; then command the controls interpolated at where the
/// vehicle is along the prediction. Errors if too far from the line or the
/// solver fails.
fn control(config: &MpcConfig, input: &Input, state: &mut State) -> Result<Control, String> {
    let (line, pose, limits) = (input.line, input.pose, input.limits);
    let horizon = config.horizon();
    let span_m = horizon as f64 * config.step_m;
    let nearest = line.nearest(pose.x_m, pose.y_m, state.hint, span_m + 1.0);
    if nearest.distance_m > config.max_cross_track_m {
        state.hint = None;
        return Err(format!(
            "{:.2} m off the race line (more than max_cross_track_m) -",
            nearest.distance_m
        ));
    }
    state.hint = Some(nearest.segment);

    // Where the vehicle is along the prediction, if there's one of the
    // right length to follow.
    let along = state
        .plan
        .as_ref()
        .filter(|plan| plan.solution.states.len() == horizon)
        .map(|plan| closer_state_index(&plan.solution.states, pose.x_m, pose.y_m));
    let mut index = 0.5;
    match along {
        Some((_, squared_m2)) if squared_m2 > config.step_m * config.step_m => {
            solve_from_scratch(config, input, &nearest, state)?
        }
        None => solve_from_scratch(config, input, &nearest, state)?,
        // While there's an opponent, every tick: it moves, and may have
        // just appeared.
        Some((along, _))
            if along > config.percentage_of_mpc_prediction_to_follow / 100.0 * horizon as f64
                || input.opponent.is_some() =>
        {
            let plan = state.plan.as_ref().expect("followed a prediction");
            let u = shifted(&plan.solution.controls, along as usize);
            solve_from(config, input, &nearest, state, u, None)?
        }
        Some((along, _)) => index = along,
    }

    let controls = &state.plan.as_ref().expect("solved above").solution.controls;
    let [steering, speed] = interpolate(controls, index);
    let max_steering = limits.max_steering_angle_rad;
    let steering_rad = steering.clamp(-max_steering, max_steering);
    state.last_steering_rad = steering_rad;
    Ok(Control {
        steering_rad,
        speed_mps: speed.clamp(
            MIN_COMMAND_SPEED_MPS,
            limits.max_speed_mps.max(MIN_COMMAND_SPEED_MPS),
        ),
        nearest,
    })
}

/// A solve with no previous solution to start from: initialized straight
/// ahead at the race line's speed.
fn solve_from_scratch(
    config: &MpcConfig,
    input: &Input,
    nearest: &Nearest,
    state: &mut State,
) -> Result<(), String> {
    let mpc = problem(config, input, nearest);
    let u: Vec<f64> = mpc.targets[..mpc.targets.len() - 1]
        .iter()
        .flat_map(|target| [0.0, target[2]])
        .collect();
    let mut initialization = u.clone();
    bounds(config, &mpc, input.limits, state.last_steering_rad).project(&mut initialization);
    let initialization = mpc.rollout(&initialization);
    solve_from(config, input, nearest, state, u, Some(initialization))
}

/// Solves from the initial guess `u`, keeping the result as the plan.
fn solve_from(
    config: &MpcConfig,
    input: &Input,
    nearest: &Nearest,
    state: &mut State,
    mut u: Vec<f64>,
    initialization: Option<Vec<MpcState>>,
) -> Result<(), String> {
    let mpc = problem(config, input, nearest);
    let bounds = bounds(config, &mpc, input.limits, state.last_steering_rad);
    let start = Instant::now();
    let solution = solve(&mpc, &bounds, &mut u, &mut state.cache, config.solver())?;
    let solve_ms = start.elapsed().as_secs_f64() * 1000.0;
    let opponent = mpc
        .opponent
        .map(|opponent| mpc.times(&u).into_iter().map(|t| opponent.at(t)).collect())
        .unwrap_or_default();
    state.plan = Some(Plan {
        solution,
        targets: mpc.targets,
        initialization,
        opponent,
        solve_ms,
    });
    Ok(())
}

/// The problem from the vehicle's pose: one target every `step_m` along the
/// line from its projection, at the scaled profile speed.
fn problem(config: &MpcConfig, input: &Input, nearest: &Nearest) -> MpcProblem {
    let targets = (0..config.horizon())
        .map(|i| {
            let point = input.line.at(nearest.s_m + i as f64 * config.step_m);
            [point.x, point.y, config.scale_speed * point.speed_mps]
        })
        .collect();
    MpcProblem {
        start: [input.pose.x_m, input.pose.y_m, input.pose.heading_rad],
        targets,
        step_m: config.step_m,
        wheelbase_m: input.wheelbase_m,
        weights: config.weights(),
        walls: input.walls.clone(),
        opponent: input.opponent,
    }
}

fn bounds(
    config: &MpcConfig,
    mpc: &MpcProblem,
    limits: &ActuatorLimits,
    last_steering_rad: f64,
) -> Bounds {
    let max_steering = limits.max_steering_angle_rad;
    let margin = FIRST_STEERING_MARGIN * max_steering;
    Bounds::new(
        mpc,
        last_steering_rad.clamp(-margin, margin),
        max_steering,
        config.min_speed_gain,
        config.max_speed_gain,
        limits.max_speed_mps,
    )
}

/// Where `(x, y)` is along `states`, as a
/// fractional index between the nearest state and its closer neighbor,
/// and the squared distance to that nearest state. The last state is
/// never the nearest, so there's always a control after the index.
fn closer_state_index(states: &[MpcState], x: f64, y: f64) -> (f64, f64) {
    let squared = |i: usize| (x - states[i][0]).powi(2) + (y - states[i][1]).powi(2);
    let (nearest, nearest_m2) = (0..states.len() - 1)
        .map(|i| (i, squared(i)))
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .expect("a horizon has at least two states");
    let next_m2 = squared(nearest + 1);
    let fraction = |a_m2: f64, b_m2: f64| {
        let (a, b) = (a_m2.sqrt(), b_m2.sqrt());
        if a + b > 0.0 { a / (a + b) } else { 0.0 }
    };
    if nearest > 0 && squared(nearest - 1) < next_m2 {
        let previous_m2 = squared(nearest - 1);
        (
            (nearest - 1) as f64 + fraction(previous_m2, nearest_m2),
            nearest_m2,
        )
    } else {
        (nearest as f64 + fraction(nearest_m2, next_m2), nearest_m2)
    }
}

/// The controls at fractional index `index`, linearly between its
/// neighbors - clamped to the horizon, so nothing is read past its end
/// when the vehicle is near the last state.
fn interpolate(controls: &[[f64; 2]], index: f64) -> [f64; 2] {
    let last = controls.len() - 1;
    let before = (index.max(0.0) as usize).min(last);
    let after = (before + 1).min(last);
    let t = (index - before as f64).clamp(0.0, 1.0);
    [0, 1].map(|k| controls[before][k] * (1.0 - t) + controls[after][k] * t)
}

/// `controls` from `from` on, padded to their length with the last one,
/// flattened into a solver's initial guess.
fn shifted(controls: &[[f64; 2]], from: usize) -> Vec<f64> {
    let last = *controls.last().expect("a horizon has at least one control");
    controls
        .iter()
        .skip(from)
        .copied()
        .chain(std::iter::repeat(last))
        .take(controls.len())
        .flatten()
        .collect()
}

impl State {
    /// The prediction colored by its speed, the target
    /// points, the initialization of the latest solve from scratch, where
    /// the opponent was predicted to be, and the nearest point on the line.
    fn drawing(&self, control: &Control, max_speed_mps: f64) -> Drawing {
        let Some(plan) = &self.plan else {
            return Drawing::default();
        };
        let polyline = |states: &[[f64; 3]], color: Color| Shape::Polyline {
            points: states.iter().map(|s| [s[0] as f32, s[1] as f32]).collect(),
            closed: false,
            width_px: 2.0,
            color,
        };
        let states = &plan.solution.states;
        let speeds = plan.solution.controls.iter().map(|c| c[1]);
        let prediction = std::iter::once(polyline(states, Color::CYAN.with_alpha(160))).chain(
            states[1..]
                .iter()
                .zip(speeds)
                .map(|(state, speed)| Shape::Circle {
                    x_m: state[0],
                    y_m: state[1],
                    radius_m: 0.04,
                    filled: true,
                    color: speed_color(speed, max_speed_mps),
                }),
        );
        let targets = Shape::Points {
            points: plan
                .targets
                .iter()
                .map(|t| [t[0] as f32, t[1] as f32])
                .collect(),
            radius_px: 3.0,
            color: Color::PURPLE,
        };
        let initialization = plan
            .initialization
            .as_deref()
            .map(|states| polyline(states, Color::WHITE.with_alpha(90)));
        let nearest = Shape::Circle {
            x_m: control.nearest.x_m,
            y_m: control.nearest.y_m,
            radius_m: 0.06,
            filled: true,
            color: Color::BLUE,
        };
        let opponent = plan.opponent.iter().map(|&[x_m, y_m]| Shape::Circle {
            x_m,
            y_m,
            radius_m: 0.05,
            filled: false,
            color: Color::PINK,
        });
        Drawing::default()
            .element("Prediction", prediction, true)
            .element("Target points", [targets], true)
            .element("Initialization", initialization, false)
            .element("Opponent prediction", opponent, true)
            .element("Nearest point", [nearest], false)
    }

    /// The latest solve's time and iterations, for the algorithm panel -
    /// `None` without a plan.
    fn stats(&self) -> Option<String> {
        self.plan.as_ref().map(|plan| {
            format!(
                "{:.1} ms, {} iterations",
                plan.solve_ms, plan.solution.iterations
            )
        })
    }
}

/// Green at a standstill to red at `max_speed_mps`.
fn speed_color(speed_mps: f64, max_speed_mps: f64) -> Color {
    let t = if max_speed_mps > 0.0 {
        (speed_mps / max_speed_mps).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let mix = |a: u8, b: u8| (f64::from(a) + t * (f64::from(b) - f64::from(a))).round() as u8;
    Color::rgb(
        mix(Color::GREEN.r, Color::RED.r),
        mix(Color::GREEN.g, Color::RED.g),
        mix(Color::GREEN.b, Color::RED.b),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::environment::SpeedPoint;
    use std::f64::consts::PI;

    fn input<'a>(line: &'a Line, pose: Pose, limits: &'a ActuatorLimits) -> Input<'a> {
        Input {
            line,
            pose,
            limits,
            wheelbase_m: WHEELBASE_M,
            walls: None,
            opponent: None,
        }
    }

    const WHEELBASE_M: f64 = 0.32;

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
    /// increasing angle at 2 m/s.
    fn circle(radius_m: f64) -> Line {
        let n = 2000;
        Line::new(
            (0..n)
                .map(|i| {
                    let angle = 2.0 * PI * i as f64 / n as f64;
                    SpeedPoint {
                        x: radius_m * angle.cos(),
                        y: radius_m * angle.sin(),
                        speed_mps: 2.0,
                    }
                })
                .collect(),
        )
        .unwrap()
    }

    #[test]
    fn on_a_circle_it_settles_on_its_steering() {
        let (radius, config) = (3.0, MpcConfig::default());
        let line = circle(radius);
        let mut state = State::default();
        let mut pose = Pose {
            x_m: radius,
            y_m: 0.0,
            heading_rad: PI / 2.0,
        };
        // Drive the commands a while with the model itself.
        let mut control = None;
        for _ in 0..400 {
            let c = super::control(&config, &input(&line, pose, &limits()), &mut state).unwrap();
            let mpc = MpcProblem {
                start: [pose.x_m, pose.y_m, pose.heading_rad],
                targets: vec![[0.0; 3]; 2],
                step_m: 0.05,
                wheelbase_m: WHEELBASE_M,
                weights: Weights::default(),
                walls: None,
                opponent: None,
            };
            let [x, y, heading] = mpc.rollout(&[c.steering_rad, c.speed_mps])[1];
            pose = Pose {
                x_m: x,
                y_m: y,
                heading_rad: heading,
            };
            control = Some(c);
        }
        let control = control.unwrap();
        let expected = (WHEELBASE_M / radius).atan();
        assert!(
            (control.steering_rad - expected).abs() < 0.02,
            "{} vs {expected}",
            control.steering_rad
        );
        assert!(
            (control.speed_mps - 2.0).abs() < 0.1,
            "{}",
            control.speed_mps
        );
        assert!(
            control.nearest.distance_m < 0.05,
            "{}",
            control.nearest.distance_m
        );
    }

    #[test]
    fn an_opponent_appearing_is_planned_for_at_once() {
        let (config, line, limits) = (MpcConfig::default(), circle(3.0), limits());
        let pose = Pose {
            x_m: 3.0,
            y_m: 0.0,
            heading_rad: PI / 2.0,
        };
        let mut state = State::default();
        control(&config, &input(&line, pose, &limits), &mut state).unwrap();
        assert!(state.plan.as_ref().unwrap().opponent.is_empty());
        // Standing still, the prediction would be followed without solving
        // again - but not with an opponent.
        let with_opponent = Input {
            opponent: Some(Opponent {
                position: [2.9, 1.0],
                velocity: [0.0, 0.0],
                radius_m: 1.0,
                gaussian: true,
            }),
            ..input(&line, pose, &limits)
        };
        control(&config, &with_opponent, &mut state).unwrap();
        let plan = state.plan.as_ref().unwrap();
        assert_eq!(plan.opponent.len(), config.horizon() - 1);
    }

    #[test]
    fn too_far_from_the_line_is_an_error() {
        let line = circle(3.0);
        let pose = Pose {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
        };
        assert!(
            control(
                &MpcConfig::default(),
                &input(&line, pose, &limits()),
                &mut State::default()
            )
            .is_err()
        );
    }

    #[test]
    fn the_prediction_is_used_for_a_while_after_a_detection() {
        use crate::WriteMeta;
        let config = MpcConfig {
            opponent_timeout_s: 0.2,
            ..MpcConfig::default()
        };
        let stamped = |detected: bool, age: Duration| crate::Stamped {
            value: DetectedOpponent {
                detected,
                position: [1.0, 2.0],
                velocity: [0.5, 0.0],
                ..Default::default()
            },
            meta: WriteMeta {
                write_count: 1,
                written_at: Instant::now().checked_sub(age),
                written_at_unix_us: 0,
            },
        };
        // Never detected: nothing, prediction or not.
        let mut last = None;
        assert!(
            config
                .opponent(&stamped(false, Duration::ZERO), &mut last)
                .is_none()
        );
        // Detected: the detection.
        let opponent = config
            .opponent(&stamped(true, Duration::ZERO), &mut last)
            .unwrap();
        assert_eq!(
            (opponent.position, opponent.velocity),
            ([1.0, 2.0], [0.5, 0.0])
        );
        assert_eq!(opponent.radius_m, config.opponent_radius_m);
        assert!(last.is_some());
        // Missed by a scan right after: the prediction.
        assert!(
            config
                .opponent(&stamped(false, Duration::ZERO), &mut last)
                .is_some()
        );
        // Missed for longer than the timeout: nothing.
        let mut long_ago = Instant::now().checked_sub(Duration::from_secs(1));
        assert!(
            config
                .opponent(&stamped(false, Duration::ZERO), &mut long_ago)
                .is_none()
        );
        // A stale topic: nothing, even if it says detected.
        let mut last = None;
        assert!(
            config
                .opponent(&stamped(true, Duration::from_secs(1)), &mut last)
                .is_none()
        );
        // Never written: only the seed.
        let seed = crate::Stamped {
            value: DetectedOpponent::default(),
            meta: WriteMeta::default(),
        };
        assert!(config.opponent(&seed, &mut None).is_none());
    }

    #[test]
    fn the_closer_state_index_is_fractional_between_neighbors() {
        let states: Vec<MpcState> = (0..5).map(|i| [i as f64, 0.0, 0.0]).collect();
        let (index, squared) = closer_state_index(&states, 1.25, 0.0);
        assert!((index - 1.25).abs() < 1e-12, "{index}");
        assert!((squared - 0.0625).abs() < 1e-12);
        let (index, _) = closer_state_index(&states, 1.75, 0.0);
        assert!((index - 1.75).abs() < 1e-12, "{index}");
        // Past the end: never beyond the last control.
        let (index, _) = closer_state_index(&states, 10.0, 0.0);
        assert!(index <= 4.0, "{index}");
    }

    #[test]
    fn controls_are_interpolated_within_the_horizon() {
        let controls = [[0.0, 1.0], [0.2, 2.0], [0.4, 3.0]];
        assert_eq!(interpolate(&controls, 0.5), [0.1, 1.5]);
        assert_eq!(interpolate(&controls, 5.0), [0.4, 3.0]);
    }

    #[test]
    fn a_shift_pads_with_the_last_control() {
        let controls = [[0.0, 1.0], [0.2, 2.0], [0.4, 3.0]];
        assert_eq!(shifted(&controls, 1), vec![0.2, 2.0, 0.4, 3.0, 0.4, 3.0]);
        assert_eq!(shifted(&controls, 0), vec![0.0, 1.0, 0.2, 2.0, 0.4, 3.0]);
    }

    #[test]
    fn every_config_field_is_tunable() {
        let config = MpcConfig::default();
        let info = AutonomousAlgorithmInfo::new("MPC", "").with_parameters(&config, parameters());
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
