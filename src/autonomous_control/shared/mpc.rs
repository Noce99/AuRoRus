//! The optimal-control problem behind [`mpc`](crate::autonomous_control):
//! ubm's `MPCEuclidianDistance` (`mpc_casadi.cpp`), solved with PANOC
//! instead of CasADi/IPOPT.
//!
//! ubm's multiple shooting (states as variables, the dynamics as equality
//! constraints) becomes single shooting here: the states are simulated
//! forward from the controls, so the only decision variables are the
//! `horizon - 1` controls `(steering, speed)` and the only constraints are
//! boxes on them - exactly what PANOC handles. The gradient is the reverse
//! pass through that simulation.
//!
//! The model is ubm's kinematic bicycle stepped over a fixed *arc length*
//! (not a fixed time): the path doesn't depend on the speed, which only
//! enters the cost - and the time at which the vehicle reaches each step,
//! against which an opponent is predicted.

use crate::planning::track::squared_distance_transform;
use crate::topics::SelectedMap;
use optimization_engine::constraints::Rectangle;
use optimization_engine::core::ExitStatus;
use optimization_engine::panoc::{PANOCCache, PANOCOptimizer};
use optimization_engine::{Problem, SolverError};
use std::sync::Arc;
use std::time::Duration;

/// Memory of PANOC's L-BFGS directions.
const LBFGS_MEMORY: usize = 10;

/// Fewest steps the horizon may have: the smoothness terms take second
/// differences of the controls, of which there are `horizon - 1`.
pub(crate) const MIN_HORIZON: usize = 4;

/// Weight of each term of the cost - each one already averaged over the
/// horizon, as ubm does, so a weight doesn't change meaning with it.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct Weights {
    /// Squared distance of each predicted position from its target point.
    pub(crate) distance: f64,
    /// Squared second difference of the steering.
    pub(crate) steering_smoothness: f64,
    /// Squared second difference of the speed.
    pub(crate) speed_smoothness: f64,
    /// Squared difference of the speed from the target's.
    pub(crate) go_fast: f64,
    /// Squared heading change times squared speed, per step.
    pub(crate) centripetal: f64,
    /// Squared depth of each predicted position into [`Walls::margin_m`].
    pub(crate) walls: f64,
    /// Closeness of each predicted position to the opponent's, then - see
    /// [`Opponent`].
    pub(crate) opponent: f64,
}

/// Added to the speed when turning a step's length into time, so a
/// standstill doesn't take forever - ubm's.
const TIME_SPEED_EPSILON_MPS: f64 = 0.01;

/// Keeps the Gaussian opponent cost's distance differentiable where it's 0.
const DISTANCE_EPSILON_M2: f64 = 1e-6;

/// Keeps the inverse-square opponent cost finite where the distance is 0 - ubm's.
const INVERSE_SQUARE_EPSILON_M2: f64 = 1e-4;

/// Distance to the nearest wall across a map, in meters - interpolated
/// bilinearly between pixel centers, so it has a gradient everywhere on it.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct DistanceField {
    width_px: usize,
    height_px: usize,
    origin_x_m: f64,
    origin_y_m: f64,
    resolution_m_per_px: f64,
    /// Row-major, like [`SelectedMap::pixels`].
    distance_m: Vec<f64>,
}

impl DistanceField {
    /// Every pixel's distance to the nearest non-white one of `map`, or
    /// `None` if no map is loaded (or its pixels don't match its size).
    pub(crate) fn new(map: &SelectedMap) -> Option<Self> {
        let info = map.info.as_ref()?;
        let (width_px, height_px) = (map.width_px as usize, map.height_px as usize);
        if width_px == 0 || height_px == 0 || map.pixels.len() != width_px * height_px {
            return None;
        }
        let wall: Vec<bool> = map.pixels.iter().map(|&pixel| pixel != 255).collect();
        let resolution = info.resolution_m_per_px;
        let distance_m = squared_distance_transform(&wall, width_px, height_px)
            .into_iter()
            .map(|d2| d2.sqrt() * resolution)
            .collect();
        Some(Self {
            width_px,
            height_px,
            origin_x_m: info.origin.x,
            origin_y_m: info.origin.y,
            resolution_m_per_px: resolution,
            distance_m,
        })
    }

    /// The distance at `(x_m, y_m)` and its gradient - 0, flat, off the map
    /// (as good as a wall).
    pub(crate) fn sample(&self, x_m: f64, y_m: f64) -> (f64, [f64; 2]) {
        let u = (x_m - self.origin_x_m) / self.resolution_m_per_px - 0.5;
        let v = (y_m - self.origin_y_m) / self.resolution_m_per_px - 0.5;
        if !(u >= 0.0 && v >= 0.0) {
            return (0.0, [0.0, 0.0]);
        }
        let (col, row) = (u.floor() as usize, v.floor() as usize);
        if col + 1 >= self.width_px || row + 1 >= self.height_px {
            return (0.0, [0.0, 0.0]);
        }
        let (fu, fv) = (u - col as f64, v - row as f64);
        let at = |c: usize, r: usize| self.distance_m[r * self.width_px + c];
        let (d00, d10, d01, d11) = (
            at(col, row),
            at(col + 1, row),
            at(col, row + 1),
            at(col + 1, row + 1),
        );
        let distance = d00 * (1.0 - fu) * (1.0 - fv)
            + d10 * fu * (1.0 - fv)
            + d01 * (1.0 - fu) * fv
            + d11 * fu * fv;
        let du = (d10 - d00) * (1.0 - fv) + (d11 - d01) * fv;
        let dv = (d01 - d00) * (1.0 - fu) + (d11 - d10) * fu;
        (
            distance,
            [du / self.resolution_m_per_px, dv / self.resolution_m_per_px],
        )
    }
}

/// The walls term: every predicted position closer than `margin_m` to a
/// wall costs the square of how much closer. (ubm's was a precomputed
/// CasADi interpolant of its own, looked up with y flipped.)
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Walls {
    pub(crate) field: Arc<DistanceField>,
    pub(crate) margin_m: f64,
}

/// The opponent term: the opponent is predicted at constant velocity to
/// the time the vehicle reaches each step, and each predicted position
/// costs `exp(-4 distance / radius_m)` if `gaussian`, else
/// `1 / distance²` - ubm's.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Opponent {
    pub(crate) position: [f64; 2],
    pub(crate) velocity: [f64; 2],
    pub(crate) radius_m: f64,
    pub(crate) gaussian: bool,
}

impl Opponent {
    /// The cost at squared distance `d2`, and its derivative with respect to `d2`.
    fn cost(&self, d2: f64) -> (f64, f64) {
        if self.gaussian {
            let k = 4.0 / self.radius_m.max(1e-3);
            let d = (d2 + DISTANCE_EPSILON_M2).sqrt();
            let cost = (-k * d).exp();
            (cost, -k * cost / (2.0 * d))
        } else {
            let cost = 1.0 / (d2 + INVERSE_SQUARE_EPSILON_M2);
            (cost, -cost * cost)
        }
    }

    /// Where it is after `time_s`.
    pub(crate) fn at(&self, time_s: f64) -> [f64; 2] {
        [
            self.position[0] + self.velocity[0] * time_s,
            self.position[1] + self.velocity[1] * time_s,
        ]
    }
}

/// A pose: position in meters, heading in radians.
pub(crate) type State = [f64; 3];

/// One target point: position in meters and speed in m/s.
pub(crate) type Target = [f64; 3];

/// One solve's data. `targets[i]` is where the vehicle should be after `i`
/// steps (`targets[0]` is unused) and the speed there.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Mpc {
    pub(crate) start: State,
    pub(crate) targets: Vec<Target>,
    /// Arc length of one step, in meters.
    pub(crate) step_m: f64,
    pub(crate) wheelbase_m: f64,
    pub(crate) weights: Weights,
    /// Without it, there's no walls term.
    pub(crate) walls: Option<Walls>,
    /// Without it, there's no opponent term.
    pub(crate) opponent: Option<Opponent>,
}

/// Where one step of the bicycle model goes, and what the reverse pass
/// needs of it.
struct Step {
    /// Heading change over the step.
    dtheta: f64,
    /// Derivative of `dtheta` with respect to the steering.
    ddtheta: f64,
    /// Direction of travel over the step (heading plus slip angle).
    direction: f64,
    /// Derivative of the slip angle with respect to the steering.
    dbeta: f64,
}

impl Mpc {
    /// Steps in the horizon, the start included.
    pub(crate) fn horizon(&self) -> usize {
        self.targets.len()
    }

    /// Number of decision variables: `(steering, speed)` per control.
    pub(crate) fn len(&self) -> usize {
        2 * (self.horizon() - 1)
    }

    /// One step of ubm's bicycle model with steering `delta` from heading
    /// `theta`: slip angle `beta = atan(tan(delta) / 2)`, travel along
    /// `theta + beta`, heading change `step tan(delta) cos(beta) / wheelbase`.
    fn step(&self, theta: f64, delta: f64) -> Step {
        let t = delta.tan();
        let q = 1.0 + t * t / 4.0;
        let beta = (t / 2.0).atan();
        let k = self.step_m / self.wheelbase_m;
        Step {
            // tan(delta) cos(beta) = t / sqrt(q)
            dtheta: k * t / q.sqrt(),
            ddtheta: k * (1.0 + t * t) / q.powf(1.5),
            direction: theta + beta,
            dbeta: (1.0 + t * t) / (2.0 * q),
        }
    }

    /// The states the controls `u` lead to, `start` first.
    pub(crate) fn rollout(&self, u: &[f64]) -> Vec<State> {
        let mut states = Vec::with_capacity(self.horizon());
        states.push(self.start);
        for i in 0..self.horizon() - 1 {
            let [x, y, theta] = states[i];
            let step = self.step(theta, u[2 * i]);
            let (sin, cos) = step.direction.sin_cos();
            states.push([
                x + self.step_m * cos,
                y + self.step_m * sin,
                theta + step.dtheta,
            ]);
        }
        states
    }

    /// When the vehicle reaches each step after the start, driving `u`'s
    /// speeds: `times[i]` is state `i + 1`'s.
    pub(crate) fn times(&self, u: &[f64]) -> Vec<f64> {
        u.iter()
            .skip(1)
            .step_by(2)
            .scan(0.0, |time, v| {
                *time += self.step_m / (v + TIME_SPEED_EPSILON_MPS);
                Some(*time)
            })
            .collect()
    }

    /// The cost of the controls `u`, and its gradient into `grad` if given.
    pub(crate) fn evaluate(&self, u: &[f64], grad: Option<&mut [f64]>) -> f64 {
        let h = self.horizon();
        let controls = h - 1;
        let w = &self.weights;
        let c_distance = w.distance / controls as f64;
        let c_centripetal = w.centripetal / controls as f64;
        let c_go_fast = w.go_fast / controls as f64;
        let c_steering = w.steering_smoothness / (h - 3) as f64;
        let c_speed = w.speed_smoothness / (h - 3) as f64;
        let c_walls = w.walls / controls as f64;
        let c_opponent = w.opponent / controls as f64;
        let walls = self.walls.as_ref().filter(|_| c_walls != 0.0);
        let opponent = self.opponent.filter(|_| c_opponent != 0.0);

        let states = self.rollout(u);
        let times = if opponent.is_some() {
            self.times(u)
        } else {
            Vec::new()
        };
        // Per state after the start: the walls' and the opponent's cost,
        // and their derivatives with respect to its position (and, for the
        // opponent, to the time it's reached).
        let mut walls_terms = vec![(0.0, [0.0; 2]); controls];
        let mut opponent_terms = vec![(0.0, [0.0; 2], 0.0); controls];
        for i in 0..controls {
            let [x, y, _] = states[i + 1];
            if let Some(walls) = walls {
                let (distance, gradient) = walls.field.sample(x, y);
                let depth = (walls.margin_m - distance).max(0.0);
                walls_terms[i] = (
                    c_walls * depth * depth,
                    gradient.map(|g| -2.0 * c_walls * depth * g),
                );
            }
            if let Some(opponent) = opponent {
                let [ox, oy] = opponent.at(times[i]);
                let (ex, ey) = (x - ox, y - oy);
                let (cost, dcost) = opponent.cost(ex * ex + ey * ey);
                let dcost = c_opponent * dcost;
                opponent_terms[i] = (
                    c_opponent * cost,
                    [2.0 * dcost * ex, 2.0 * dcost * ey],
                    -2.0 * dcost * (ex * opponent.velocity[0] + ey * opponent.velocity[1]),
                );
            }
        }
        let steps: Vec<Step> = (0..controls)
            .map(|i| self.step(states[i][2], u[2 * i]))
            .collect();

        let mut cost = 0.0;
        for i in 0..controls {
            let [x, y, _] = states[i + 1];
            let [tx, ty, tv] = self.targets[i + 1];
            let v = u[2 * i + 1];
            cost += c_distance * ((x - tx).powi(2) + (y - ty).powi(2));
            cost += c_centripetal * (steps[i].dtheta * v).powi(2);
            cost += c_go_fast * (tv - v).powi(2);
            cost += walls_terms[i].0 + opponent_terms[i].0;
        }
        for i in 0..controls - 2 {
            let steering = u[2 * i + 4] - 2.0 * u[2 * i + 2] + u[2 * i];
            let speed = u[2 * i + 5] - 2.0 * u[2 * i + 3] + u[2 * i + 1];
            cost += c_steering * steering * steering + c_speed * speed * speed;
        }

        let Some(grad) = grad else {
            return cost;
        };
        grad.fill(0.0);
        // Reverse pass: (ax, ay, atheta) is the derivative of the cost with
        // respect to state i + 1, through everything after it.
        let (mut ax, mut ay, mut atheta) = (0.0, 0.0, 0.0);
        for i in (0..controls).rev() {
            let [x, y, _] = states[i + 1];
            let [tx, ty, tv] = self.targets[i + 1];
            let v = u[2 * i + 1];
            ax += 2.0 * c_distance * (x - tx) + walls_terms[i].1[0] + opponent_terms[i].1[0];
            ay += 2.0 * c_distance * (y - ty) + walls_terms[i].1[1] + opponent_terms[i].1[1];

            let step = &steps[i];
            let (sin, cos) = step.direction.sin_cos();
            // d(state i + 1) / d(direction): the position turns with it.
            let adirection = self.step_m * (-ax * sin + ay * cos);
            let adtheta = atheta + 2.0 * c_centripetal * step.dtheta * v * v;
            grad[2 * i] += adirection * step.dbeta + adtheta * step.ddtheta;
            grad[2 * i + 1] +=
                2.0 * c_centripetal * step.dtheta.powi(2) * v - 2.0 * c_go_fast * (tv - v);
            // State i's heading moves the direction and state i + 1's heading one for one.
            atheta += adirection;
        }
        if opponent.is_some() {
            // Speed j sets the time of every state from j + 1 on.
            let mut atime = 0.0;
            for j in (0..controls).rev() {
                atime += opponent_terms[j].2;
                let v = u[2 * j + 1] + TIME_SPEED_EPSILON_MPS;
                grad[2 * j + 1] -= atime * self.step_m / (v * v);
            }
        }
        for i in 0..controls - 2 {
            let steering = 2.0 * c_steering * (u[2 * i + 4] - 2.0 * u[2 * i + 2] + u[2 * i]);
            let speed = 2.0 * c_speed * (u[2 * i + 5] - 2.0 * u[2 * i + 3] + u[2 * i + 1]);
            grad[2 * i] += steering;
            grad[2 * i + 2] -= 2.0 * steering;
            grad[2 * i + 4] += steering;
            grad[2 * i + 1] += speed;
            grad[2 * i + 3] -= 2.0 * speed;
            grad[2 * i + 5] += speed;
        }
        cost
    }
}

/// The boxes the controls must stay in.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Bounds {
    pub(crate) lower: Vec<f64>,
    pub(crate) upper: Vec<f64>,
}

impl Bounds {
    /// Steering within `max_steering_rad` either way, the first one fixed
    /// to `first_steering_rad` (what the vehicle is already steering);
    /// the speed of control `i` between `min_gain` and `max_gain` times the
    /// target speed there, never above `max_speed_mps` - and never an empty
    /// range.
    pub(crate) fn new(
        mpc: &Mpc,
        first_steering_rad: f64,
        max_steering_rad: f64,
        min_gain: f64,
        max_gain: f64,
        max_speed_mps: f64,
    ) -> Self {
        let n = mpc.len();
        let (mut lower, mut upper) = (vec![0.0; n], vec![0.0; n]);
        for i in 0..n / 2 {
            let (lo, hi) = if i == 0 {
                let first = first_steering_rad.clamp(-max_steering_rad, max_steering_rad);
                (first, first)
            } else {
                (-max_steering_rad, max_steering_rad)
            };
            lower[2 * i] = lo;
            upper[2 * i] = hi;
            let v_ref = mpc.targets[i][2].max(0.0);
            let hi = (max_gain * v_ref).min(max_speed_mps).max(0.0);
            lower[2 * i + 1] = (min_gain * v_ref).min(hi);
            upper[2 * i + 1] = hi;
        }
        Self { lower, upper }
    }

    /// `u` moved into the boxes.
    pub(crate) fn project(&self, u: &mut [f64]) {
        for ((u, lo), hi) in u.iter_mut().zip(&self.lower).zip(&self.upper) {
            *u = u.clamp(*lo, *hi);
        }
    }
}

/// What a solve found.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Solution {
    /// `(steering, speed)`, `horizon - 1` of them.
    pub(crate) controls: Vec<[f64; 2]>,
    /// Where they lead, the start first - `horizon` of them.
    pub(crate) states: Vec<State>,
    pub(crate) cost: f64,
    /// Whether PANOC converged - if not, the solution is its best iterate
    /// when it ran out of iterations or time, as ubm used `opti.debug()`.
    pub(crate) converged: bool,
    pub(crate) iterations: usize,
}

/// PANOC's memory, reused between solves of the same size and tolerance -
/// see [`solve`].
#[derive(Default)]
pub(crate) struct Cache(Option<(usize, f64, PANOCCache)>);

/// PANOC's settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SolverSettings {
    pub(crate) tolerance: f64,
    pub(crate) max_iterations: usize,
    pub(crate) max_duration: Duration,
}

/// Minimizes `mpc`'s cost within `bounds` from `u`, the initial guess
/// (projected into `bounds` first), left holding the solution. Errors if the solver fails or
/// the solution isn't finite.
pub(crate) fn solve(
    mpc: &Mpc,
    bounds: &Bounds,
    u: &mut [f64],
    cache: &mut Cache,
    settings: SolverSettings,
) -> Result<Solution, String> {
    let n = mpc.len();
    assert!(
        mpc.horizon() >= MIN_HORIZON,
        "an MPC horizon has at least {MIN_HORIZON} steps"
    );
    assert_eq!(u.len(), n, "one steering and one speed per control");
    bounds.project(u);

    let cache = match &mut cache.0 {
        Some((size, tolerance, cache)) if *size == n && *tolerance == settings.tolerance => cache,
        slot => {
            &mut slot
                .insert((
                    n,
                    settings.tolerance,
                    PANOCCache::new(n, settings.tolerance, LBFGS_MEMORY),
                ))
                .2
        }
    };
    let rectangle = Rectangle::new(Some(&bounds.lower), Some(&bounds.upper));
    let gradient = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
        mpc.evaluate(u, Some(grad));
        Ok(())
    };
    let cost = |u: &[f64], value: &mut f64| -> Result<(), SolverError> {
        *value = mpc.evaluate(u, None);
        Ok(())
    };
    let problem = Problem::new(&rectangle, gradient, cost);
    let mut optimizer = PANOCOptimizer::new(problem, cache)
        .with_max_iter(settings.max_iterations.max(1))
        .with_max_duration(settings.max_duration);
    let status = optimizer
        .solve(u)
        .map_err(|err| format!("MPC solver error: {err:?}"))?;

    let cost = mpc.evaluate(u, None);
    if !cost.is_finite() || u.iter().any(|u| !u.is_finite()) {
        return Err("MPC solution is not finite".into());
    }
    Ok(Solution {
        controls: u.chunks_exact(2).map(|c| [c[0], c[1]]).collect(),
        states: mpc.rollout(u),
        cost,
        converged: status.exit_status() == ExitStatus::Converged,
        iterations: status.iterations(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::{RngExt, SeedableRng};

    fn weights() -> Weights {
        Weights {
            distance: 1.5,
            steering_smoothness: 60.0,
            speed_smoothness: 1.0,
            go_fast: 0.2,
            centripetal: 0.3,
            walls: 10.0,
            opponent: 30.0,
        }
    }

    /// A 40 m x 3 m corridor along +x, centered on y = 0 - see [`corridor`].
    fn corridor_map() -> SelectedMap {
        use crate::environment::{ImageOrigin, MapInfo, MapSource, StartFinishLine, WorldPoint};
        let resolution = 0.05;
        let (width, height) = (800u32, 100u32);
        let origin = ImageOrigin {
            x: -20.0,
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

    fn corridor(margin_m: f64) -> Walls {
        Walls {
            field: Arc::new(DistanceField::new(&corridor_map()).unwrap()),
            margin_m,
        }
    }

    /// Targets along y = `y`, toward +x, `step_m` apart, from the origin
    /// heading along the line.
    fn straight(y: f64, step_m: f64, horizon: usize) -> Mpc {
        Mpc {
            start: [0.0, y, 0.0],
            targets: (0..horizon).map(|i| [step_m * i as f64, y, 2.0]).collect(),
            step_m,
            wheelbase_m: 0.32,
            weights: weights(),
            walls: None,
            opponent: None,
        }
    }

    /// Targets along a circle of radius `radius_m` around the origin,
    /// counterclockwise from `(radius, 0)`, `step_m` apart.
    fn circle(radius_m: f64, step_m: f64, horizon: usize) -> Mpc {
        let targets = (0..horizon)
            .map(|i| {
                let angle = i as f64 * step_m / radius_m;
                [radius_m * angle.cos(), radius_m * angle.sin(), 2.0]
            })
            .collect();
        Mpc {
            start: [radius_m, 0.0, std::f64::consts::FRAC_PI_2],
            targets,
            step_m,
            wheelbase_m: 0.32,
            weights: weights(),
            walls: None,
            opponent: None,
        }
    }

    fn settings() -> SolverSettings {
        SolverSettings {
            tolerance: 1e-6,
            max_iterations: 2000,
            max_duration: Duration::from_secs(5),
        }
    }

    #[test]
    fn the_gradient_matches_finite_differences() {
        let mpc = circle(3.0, 0.2, 15);
        let mut rng = rand::rngs::StdRng::seed_from_u64(7);
        for _ in 0..20 {
            let u: Vec<f64> = (0..mpc.len())
                .map(|i| {
                    if i % 2 == 0 {
                        rng.random_range(-0.4..0.4)
                    } else {
                        rng.random_range(0.5..4.0)
                    }
                })
                .collect();
            let mut grad = vec![0.0; u.len()];
            mpc.evaluate(&u, Some(&mut grad));
            for (i, &analytic) in grad.iter().enumerate() {
                let h = 1e-6;
                let (mut plus, mut minus) = (u.clone(), u.clone());
                plus[i] += h;
                minus[i] -= h;
                let numeric = (mpc.evaluate(&plus, None) - mpc.evaluate(&minus, None)) / (2.0 * h);
                assert!(
                    (analytic - numeric).abs() < 1e-5 * (1.0 + numeric.abs()),
                    "{i}: {analytic} vs {numeric}"
                );
            }
        }
    }

    #[test]
    fn the_gradient_with_walls_and_an_opponent_matches_finite_differences() {
        for gaussian in [true, false] {
            let mpc = Mpc {
                walls: Some(corridor(0.8)),
                opponent: Some(Opponent {
                    position: [1.5, 0.3],
                    velocity: [1.0, -0.2],
                    radius_m: 1.0,
                    gaussian,
                }),
                ..straight(0.5, 0.2, 15)
            };
            let mut rng = rand::rngs::StdRng::seed_from_u64(11);
            for _ in 0..20 {
                let u: Vec<f64> = (0..mpc.len())
                    .map(|i| {
                        if i % 2 == 0 {
                            rng.random_range(-0.4..0.4)
                        } else {
                            rng.random_range(0.5..4.0)
                        }
                    })
                    .collect();
                let mut grad = vec![0.0; u.len()];
                mpc.evaluate(&u, Some(&mut grad));
                for (i, &analytic) in grad.iter().enumerate() {
                    let h = 1e-7;
                    let (mut plus, mut minus) = (u.clone(), u.clone());
                    plus[i] += h;
                    minus[i] -= h;
                    let numeric =
                        (mpc.evaluate(&plus, None) - mpc.evaluate(&minus, None)) / (2.0 * h);
                    assert!(
                        (analytic - numeric).abs() < 1e-4 * (1.0 + numeric.abs()),
                        "{gaussian} {i}: {analytic} vs {numeric}"
                    );
                }
            }
        }
    }

    #[test]
    fn the_distance_field_measures_to_the_nearest_wall() {
        let walls = corridor(0.0);
        let (distance, gradient) = walls.field.sample(0.0, 1.0);
        assert!((distance - 0.5).abs() < 0.03, "{distance}");
        assert!(
            (gradient[1] + 1.0).abs() < 0.05 && gradient[0].abs() < 1e-9,
            "{gradient:?}"
        );
        let (distance, _) = walls.field.sample(0.0, 0.0);
        assert!((distance - 1.5).abs() < 0.03, "{distance}");
        assert_eq!(walls.field.sample(100.0, 0.0), (0.0, [0.0, 0.0]));
    }

    #[test]
    fn the_walls_keep_the_prediction_off_them() {
        // Targets 0.3 m from the wall at y = 1.5.
        let solve_along = |walls: Option<Walls>| {
            let mpc = Mpc {
                walls,
                ..straight(1.2, 0.2, 15)
            };
            let bounds = Bounds::new(&mpc, 0.0, 0.4, 0.5, 3.0, 10.0);
            let mut u = vec![1.0; mpc.len()];
            let solution = solve(&mpc, &bounds, &mut u, &mut Cache::default(), settings()).unwrap();
            solution.states.last().unwrap()[1]
        };
        let free = solve_along(None);
        let kept_off = solve_along(Some(corridor(0.6)));
        assert!((free - 1.2).abs() < 1e-3, "{free}");
        assert!(kept_off < 1.1, "{kept_off}");
    }

    #[test]
    fn it_swerves_around_an_opponent_on_the_line() {
        let opponent = Opponent {
            position: [2.0, 0.0],
            velocity: [0.0, 0.0],
            radius_m: 1.0,
            gaussian: true,
        };
        let mpc = Mpc {
            opponent: Some(opponent),
            ..straight(0.0, 0.2, 15)
        };
        // Nudged off-center, so it's clear which side to pass on.
        let mpc = Mpc {
            start: [0.0, 0.05, 0.0],
            ..mpc
        };
        let bounds = Bounds::new(&mpc, 0.0, 0.4, 0.5, 3.0, 10.0);
        let mut u = vec![1.0; mpc.len()];
        let solution = solve(&mpc, &bounds, &mut u, &mut Cache::default(), settings()).unwrap();
        let passing = solution
            .states
            .iter()
            .find(|s| s[0] >= 2.0)
            .expect("reaches the opponent");
        assert!(passing[1] > 0.3, "{passing:?}");
    }

    #[test]
    fn a_moving_opponent_is_met_later() {
        let mpc = straight(0.0, 0.5, 5);
        let u = [0.0, 1.0, 0.0, 2.0, 0.0, 1.0, 0.0, 0.5];
        let times = mpc.times(&u);
        let expected = [0.5 / 1.01, 0.5 / 1.01 + 0.5 / 2.01];
        assert!(
            (times[0] - expected[0]).abs() < 1e-12 && (times[1] - expected[1]).abs() < 1e-12,
            "{times:?}"
        );
        assert_eq!(times.len(), 4);
    }

    #[test]
    fn a_constant_steering_drives_a_circle() {
        let (wheelbase, radius) = (0.32, 3.0);
        let mpc = Mpc {
            wheelbase_m: wheelbase,
            ..circle(radius, 0.01, 1000)
        };
        // Kinematic bicycle about the mid point: R = L / (cos(beta) tan(delta)).
        let tan = (wheelbase / radius) / (1.0 - (wheelbase / radius).powi(2) / 4.0).sqrt();
        let u: Vec<f64> = (0..mpc.len())
            .map(|i| if i % 2 == 0 { tan.atan() } else { 1.0 })
            .collect();
        let states = mpc.rollout(&u);
        // Circumradius of three points spread along the path.
        let [a, b, c] = [states[0], states[300], states[600]].map(|s| [s[0], s[1]]);
        let side = |p: [f64; 2], q: [f64; 2]| (p[0] - q[0]).hypot(p[1] - q[1]);
        let area2 = ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])).abs();
        let r = side(a, b) * side(b, c) * side(c, a) / (2.0 * area2);
        assert!((r - radius).abs() < 1e-3, "{r}");
    }

    #[test]
    fn on_a_straight_line_it_steers_straight_at_the_target_speed() {
        let targets: Vec<Target> = (0..15).map(|i| [0.2 * i as f64, 0.0, 2.0]).collect();
        let mpc = Mpc {
            start: [0.0, 0.0, 0.0],
            targets,
            step_m: 0.2,
            wheelbase_m: 0.32,
            weights: weights(),
            walls: None,
            opponent: None,
        };
        let bounds = Bounds::new(&mpc, 0.0, 0.4, 0.5, 3.0, 10.0);
        let mut u = vec![1.0; mpc.len()];
        let solution = solve(&mpc, &bounds, &mut u, &mut Cache::default(), settings()).unwrap();
        assert!(solution.converged);
        for [steering, speed] in solution.controls {
            assert!(steering.abs() < 1e-3, "{steering}");
            assert!((speed - 2.0).abs() < 1e-2, "{speed}");
        }
    }

    #[test]
    fn on_a_circle_it_steers_toward_it() {
        let (radius, wheelbase) = (3.0, 0.32);
        let mpc = circle(radius, 0.2, 15);
        let expected = (wheelbase / radius).atan();
        let bounds = Bounds::new(&mpc, expected, 0.4, 0.5, 3.0, 10.0);
        let mut u = vec![0.0; mpc.len()];
        let solution = solve(&mpc, &bounds, &mut u, &mut Cache::default(), settings()).unwrap();
        for [steering, _] in &solution.controls[..10] {
            assert!(
                (steering - expected).abs() < 0.02,
                "{steering} vs {expected}"
            );
        }
    }

    #[test]
    fn the_bounds_hold_the_steering_and_speed() {
        // A target far to the left and fast: the steering and speed saturate.
        let targets: Vec<Target> = (0..10)
            .map(|i| [0.2 * i as f64, 3.0 * i as f64, 2.0])
            .collect();
        let mpc = Mpc {
            start: [0.0, 0.0, 0.0],
            targets,
            step_m: 0.2,
            wheelbase_m: 0.32,
            weights: Weights {
                go_fast: 0.0,
                ..weights()
            },
            walls: None,
            opponent: None,
        };
        let bounds = Bounds::new(&mpc, 0.1, 0.3, 0.5, 1.5, 2.5);
        let mut u = vec![0.0; mpc.len()];
        let solution = solve(&mpc, &bounds, &mut u, &mut Cache::default(), settings()).unwrap();
        assert_eq!(solution.controls[0][0], 0.1);
        for [steering, speed] in solution.controls {
            assert!(steering.abs() <= 0.3 + 1e-12, "{steering}");
            assert!((1.0 - 1e-12..=2.5 + 1e-12).contains(&speed), "{speed}");
        }
        // max_gain, not ubm's MAX_SPEED, caps the speed under max_speed_mps.
        let bounds = Bounds::new(&mpc, 0.0, 0.3, 0.5, 1.5, 10.0);
        assert!(bounds.upper.iter().skip(1).step_by(2).all(|&v| v == 3.0));
        // Never an empty range, even with min_gain above max_gain.
        let bounds = Bounds::new(&mpc, 0.0, 0.3, 2.0, 1.0, 10.0);
        assert!(
            bounds
                .lower
                .iter()
                .zip(&bounds.upper)
                .all(|(lo, hi)| lo <= hi)
        );
    }
}
