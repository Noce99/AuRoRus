//! The minimum-curvature race line: the closed curve through the track,
//! kept `margin` away from both walls, whose squared curvature summed over
//! the lap is smallest - solved with OpEn's PANOC (`optimization_engine`).
//!
//! Every point may only move sideways, along the normal of a reference line
//! (the centerline at first): `p_i = c_i + alpha_i * n_i`. The discrete
//! curvature at each point, from central differences,
//!
//! ```text
//! d_i = (p_{i+1} - p_{i-1}) / 2,   s_i = p_{i+1} - 2 p_i + p_{i-1},
//! kappa_i = (d_i x s_i) / |d_i|^3
//! ```
//!
//! is linearized in the offsets around the reference (a Gauss-Newton
//! step), so the cost `sum kappa_i^2 + lambda * sum (alpha_{i+1} -
//! alpha_i)^2` becomes a quadratic, plus a penalty `w * sum max(|kappa_i| -
//! kappa_max, 0)^2` on turning tighter than the vehicle may (convex, and
//! zero wherever the line is within the limit); each point's free space
//! across the track makes the constraints a box: `-w_right + margin <=
//! alpha_i <= w_left - margin`, further limited to `max_step` either way so the
//! linearization stays accurate. Cost and gradient only couple neighbors,
//! so both cost O(n) to evaluate. The problem is then solved again around
//! each solution - the iterative scheme of Heilmeier et al., the method
//! behind TUM's global race trajectory optimization - until the line stops
//! moving. A line still tighter than `kappa_max` anywhere then is refused:
//! the track is too tight for the vehicle.
//!
//! Linearizing only the second difference, with the spacing held fixed,
//! would be simpler but wrong: moving points toward a corner's inside
//! shrinks their second difference, so that model mistakes the inside of
//! every corner for the flattest line - a shortest path, not a minimum
//! curvature one.

use super::PlanError;
use super::geometry::{
    Point2, curvature_jacobian, curvatures, left_normal, max_abs_curvature, resample_even_spacing,
    tangents,
};
use super::track::TrackGrid;
use optimization_engine::constraints::Rectangle;
use optimization_engine::core::ExitStatus;
use optimization_engine::panoc::{PANOCCache, PANOCOptimizer};
use optimization_engine::{Problem, SolverError};

/// How far a free-space ray is marched before giving up, in meters - wider
/// than any track.
const MAX_FREE_DISTANCE_M: f64 = 50.0;
/// Memory of PANOC's L-BFGS directions.
const LBFGS_MEMORY: usize = 10;
/// Weight `w` of the penalty on curvature beyond the limit, relative to the
/// curvature cost's, in the first iteration. Each iteration whose line is
/// still beyond the limit multiplies it by [`CURVATURE_LIMIT_WEIGHT_GROWTH`],
/// up to [`CURVATURE_LIMIT_WEIGHT_MAX`]: a large weight from the start
/// makes the problem too stiff for PANOC while the line is still rough
/// (e.g. a centerline computed from the walls), though it's rarely needed
/// once smooth.
const CURVATURE_LIMIT_WEIGHT: f64 = 1.0;
const CURVATURE_LIMIT_WEIGHT_GROWTH: f64 = 10.0;
/// Large enough that the line exceeds the limit by about a percent at most.
const CURVATURE_LIMIT_WEIGHT_MAX: f64 = 1000.0;
/// Slack on the limit, as a fraction of it: the penalty aims this much
/// below the limit, and the final line may exceed it by this much - the
/// penalty is not a hard constraint, and resampling moves the points a
/// little. Aiming below leaves the minimum-time optimization, which starts
/// from this line, a feasible start.
const CURVATURE_LIMIT_SLACK: f64 = 0.02;

/// Every tunable of [`optimize`].
#[derive(Debug, Clone, Copy)]
pub struct MinCurvatureConfig {
    /// Distance to keep from either wall, in meters - half the vehicle's
    /// width plus a safety margin.
    pub margin_m: f64,
    /// Tightest the line may turn, in 1/m - `f64::INFINITY` for no limit.
    pub max_curvature_per_m: f64,
    /// Spacing of the line's points, in meters.
    pub spacing_m: f64,
    /// Weight of the penalty on neighboring offsets differing (`lambda`),
    /// in 1/m^4 - `0` for pure minimum curvature.
    pub smoothness_weight: f64,
    /// Farthest any point may move in one iteration, in meters - keeps each
    /// step within the linearization's accuracy.
    pub max_step_m: f64,
    /// Most times the problem is solved around the latest solution.
    pub iterations: usize,
    /// The line has converged once no point moved further than this in an
    /// iteration, in meters.
    pub tolerance_m: f64,
    /// PANOC's tolerance on its fixed-point residual.
    pub solver_tolerance: f64,
    /// Most PANOC iterations per solve.
    pub solver_max_iterations: usize,
}

/// One outer iteration's result, for [`optimize`]'s progress callback.
pub struct Iteration<'a> {
    /// Which iteration this is, from `1`.
    pub number: usize,
    /// The line solved around.
    pub reference: &'a [Point2],
    /// Its solution.
    pub solution: &'a [Point2],
    /// How far the furthest point moved, in meters.
    pub max_move_m: f64,
}

/// The minimum-curvature line from `reference` (a closed loop inside
/// `grid`'s track, evenly spaced, starting at the start/finish line).
/// `progress` is told about every iteration, and stops the optimization -
/// with [`PlanError::Cancelled`] - by returning `false`.
///
/// Fails if the track is narrower than `2 * margin_m` somewhere along the
/// reference, or if the line still turns tighter than `max_curvature_per_m`
/// somewhere (beyond [`CURVATURE_LIMIT_SLACK`]).
pub fn optimize(
    reference: &[Point2],
    grid: &TrackGrid,
    config: &MinCurvatureConfig,
    progress: &mut dyn FnMut(Iteration) -> bool,
) -> Result<Vec<Point2>, PlanError> {
    let mut line = reference.to_vec();
    let mut limit_weight = CURVATURE_LIMIT_WEIGHT;
    for number in 1..=config.iterations.max(1) {
        let offsets = Offsets::along(&line, grid);
        let (mut lower, mut upper) = offsets.bounds(config.margin_m, number == 1)?;
        for (lo, hi) in lower.iter_mut().zip(upper.iter_mut()) {
            // Within reach of this step, but never excluding the box itself
            // when the line starts outside it (then it's moved inside first).
            *lo = lo.max(-config.max_step_m).min(*hi);
            *hi = hi.min(config.max_step_m).max(*lo);
        }
        let alpha = solve(
            &line,
            &offsets.normals,
            &lower,
            &upper,
            limit_weight,
            config,
        )?;

        let moved: Vec<Point2> = line
            .iter()
            .zip(&offsets.normals)
            .zip(&alpha)
            .map(|((point, normal), a)| Point2 {
                x: point.x + a * normal.x,
                y: point.y + a * normal.y,
            })
            .collect();
        let max_move_m = alpha.iter().fold(0.0, |max: f64, a| max.max(a.abs()));
        let solution = resample_even_spacing(&moved, config.spacing_m);
        let keep_going = progress(Iteration {
            number,
            reference: &line,
            solution: &solution,
            max_move_m,
        });
        line = solution;
        if !keep_going {
            return Err(PlanError::Cancelled);
        }
        let within_limit = max_abs_curvature(&line) <= config.max_curvature_per_m;
        if !within_limit {
            limit_weight =
                (limit_weight * CURVATURE_LIMIT_WEIGHT_GROWTH).min(CURVATURE_LIMIT_WEIGHT_MAX);
        }
        // Still moving, or still beyond the limit with more weight to put
        // on it.
        if max_move_m < config.tolerance_m
            && (within_limit || limit_weight >= CURVATURE_LIMIT_WEIGHT_MAX)
        {
            break;
        }
    }
    check_curvature(&line, config.max_curvature_per_m)?;
    Ok(line)
}

/// Fails with [`PlanError::TooCurvy`] where `line` turns tighter than
/// `limit_per_m`, beyond [`CURVATURE_LIMIT_SLACK`].
fn check_curvature(line: &[Point2], limit_per_m: f64) -> Result<(), PlanError> {
    let tightest = curvatures(line)
        .into_iter()
        .map(f64::abs)
        .enumerate()
        .max_by(|a, b| a.1.total_cmp(&b.1));
    match tightest {
        Some((i, curvature_per_m))
            if curvature_per_m > limit_per_m * (1.0 + CURVATURE_LIMIT_SLACK) =>
        {
            Err(PlanError::TooCurvy {
                x_m: line[i].x,
                y_m: line[i].y,
                curvature_per_m,
                limit_per_m,
            })
        }
        _ => Ok(()),
    }
}

/// Each point's normal and free space either side of it.
pub(super) struct Offsets {
    pub(super) normals: Vec<Point2>,
    /// Free space toward the normal (to the left), in meters.
    left_m: Vec<f64>,
    /// Free space away from it (to the right), in meters.
    right_m: Vec<f64>,
    points: Vec<Point2>,
}

impl Offsets {
    pub(super) fn along(line: &[Point2], grid: &TrackGrid) -> Offsets {
        let normals: Vec<Point2> = tangents(line).into_iter().map(left_normal).collect();
        let free = |point: &Point2, normal: &Point2, sign: f64| {
            let direction = Point2 {
                x: sign * normal.x,
                y: sign * normal.y,
            };
            grid.free_distance(*point, direction, MAX_FREE_DISTANCE_M)
        };
        Offsets {
            left_m: line
                .iter()
                .zip(&normals)
                .map(|(p, n)| free(p, n, 1.0))
                .collect(),
            right_m: line
                .iter()
                .zip(&normals)
                .map(|(p, n)| free(p, n, -1.0))
                .collect(),
            normals,
            points: line.to_vec(),
        }
    }

    /// Each offset's bounds, `margin_m` inside either wall. On the first
    /// iteration, a point where that leaves no room at all is an error;
    /// later on, the line solved around is already accepted, so every box
    /// is only widened to keep it (offset `0`) feasible - it can end up a
    /// hair outside after resampling or the raycast's rounding.
    pub(super) fn bounds(
        &self,
        margin_m: f64,
        first: bool,
    ) -> Result<(Vec<f64>, Vec<f64>), PlanError> {
        let mut lower = Vec::with_capacity(self.points.len());
        let mut upper = Vec::with_capacity(self.points.len());
        for i in 0..self.points.len() {
            let (mut lo, mut hi) = (margin_m - self.right_m[i], self.left_m[i] - margin_m);
            if lo > hi {
                if first {
                    return Err(PlanError::TooNarrow {
                        x_m: self.points[i].x,
                        y_m: self.points[i].y,
                        width_m: self.left_m[i] + self.right_m[i],
                        needed_m: 2.0 * margin_m,
                    });
                }
                let middle = (lo + hi) / 2.0;
                (lo, hi) = (middle, middle);
            }
            if !first {
                (lo, hi) = (lo.min(0.0), hi.max(0.0));
            }
            lower.push(lo);
            upper.push(hi);
        }
        Ok((lower, upper))
    }
}

/// The linear map from offsets to curvatures around `line`: `kappa_i =
/// base_i + prev_i * alpha_{i-1} + this_i * alpha_i + next_i * alpha_{i+1}`,
/// the first-order expansion of the discrete curvature (see the module
/// docs) - and the limit on its magnitude.
struct CurvatureModel {
    base: Vec<f64>,
    prev: Vec<f64>,
    this: Vec<f64>,
    next: Vec<f64>,
    /// Where the penalty starts: just below `kappa_max`, in 1/m.
    limit: f64,
    /// `w`, the weight of the penalty beyond it.
    limit_weight: f64,
}

impl CurvatureModel {
    /// The model around `line`, penalizing curvature beyond `limit` (less
    /// [`CURVATURE_LIMIT_SLACK`]) with `limit_weight`.
    fn around(
        line: &[Point2],
        normals: &[Point2],
        limit: f64,
        limit_weight: f64,
    ) -> CurvatureModel {
        let rows = curvature_jacobian(line, normals);
        CurvatureModel {
            base: rows.iter().map(|row| row.kappa).collect(),
            prev: rows.iter().map(|row| row.prev).collect(),
            this: rows.iter().map(|row| row.this).collect(),
            next: rows.iter().map(|row| row.next).collect(),
            limit: limit * (1.0 - CURVATURE_LIMIT_SLACK),
            limit_weight,
        }
    }

    /// How far `kappa` is beyond the limit, signed like it - `0` within.
    fn excess(&self, kappa: f64) -> f64 {
        kappa.signum() * (kappa.abs() - self.limit).max(0.0)
    }

    /// Every point's curvature for offsets `alpha`.
    fn curvatures(&self, alpha: &[f64]) -> Vec<f64> {
        let n = alpha.len();
        (0..n)
            .map(|i| {
                self.base[i]
                    + self.prev[i] * alpha[(i + n - 1) % n]
                    + self.this[i] * alpha[i]
                    + self.next[i] * alpha[(i + 1) % n]
            })
            .collect()
    }

    /// The cost `sum kappa_i^2 + w * sum max(|kappa_i| - kappa_max, 0)^2 +
    /// weight * sum (alpha_{i+1} - alpha_i)^2`.
    fn cost(&self, alpha: &[f64], weight: f64) -> f64 {
        let n = alpha.len();
        let curvature: f64 = self
            .curvatures(alpha)
            .iter()
            .map(|&k| k * k + self.limit_weight * self.excess(k).powi(2))
            .sum();
        let smoothness: f64 = (0..n)
            .map(|i| (alpha[(i + 1) % n] - alpha[i]).powi(2))
            .sum();
        curvature + weight * smoothness
    }

    /// [`cost`](Self::cost)'s gradient, written into `grad`.
    fn gradient(&self, alpha: &[f64], weight: f64, grad: &mut [f64]) {
        let n = alpha.len();
        // Half of each point's cost's derivative with respect to its
        // curvature.
        let kappa: Vec<f64> = self
            .curvatures(alpha)
            .into_iter()
            .map(|k| k + self.limit_weight * self.excess(k))
            .collect();
        for j in 0..n {
            let (p, q) = ((j + n - 1) % n, (j + 1) % n);
            // alpha_j appears in kappa_{j-1} (as its "next"), kappa_j, and
            // kappa_{j+1} (as its "prev").
            let curvature =
                self.next[p] * kappa[p] + self.this[j] * kappa[j] + self.prev[q] * kappa[q];
            let smoothness = 2.0 * alpha[j] - alpha[p] - alpha[q];
            grad[j] = 2.0 * curvature + 2.0 * weight * smoothness;
        }
    }
}

/// The offsets minimizing the curvature cost around `line`, within
/// `lower..=upper`, with `limit_weight` on curvature beyond the limit.
fn solve(
    line: &[Point2],
    normals: &[Point2],
    lower: &[f64],
    upper: &[f64],
    limit_weight: f64,
    config: &MinCurvatureConfig,
) -> Result<Vec<f64>, PlanError> {
    let model = CurvatureModel::around(line, normals, config.max_curvature_per_m, limit_weight);
    let weight = config.smoothness_weight;
    let bounds = Rectangle::new(Some(lower), Some(upper));
    let gradient = |alpha: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
        model.gradient(alpha, weight, grad);
        Ok(())
    };
    let cost = |alpha: &[f64], value: &mut f64| -> Result<(), SolverError> {
        *value = model.cost(alpha, weight);
        Ok(())
    };
    let problem = Problem::new(&bounds, gradient, cost);
    let mut cache = PANOCCache::new(line.len(), config.solver_tolerance, LBFGS_MEMORY);
    let mut optimizer =
        PANOCOptimizer::new(problem, &mut cache).with_max_iter(config.solver_max_iterations);

    // Start from the line itself, moved into its box where it isn't.
    let mut alpha: Vec<f64> = lower
        .iter()
        .zip(upper)
        .map(|(lo, hi)| 0.0f64.clamp(*lo, *hi))
        .collect();
    let status = optimizer
        .solve(&mut alpha)
        .map_err(|err| PlanError::Solver(err.to_string()))?;
    // PANOC hands back its iterate with the smallest fixed-point residual,
    // which before convergence can be one of its very first - taking it
    // would look like the line had stopped moving.
    if status.exit_status() != ExitStatus::Converged {
        return Err(PlanError::NotConverged {
            iterations: status.iterations(),
        });
    }
    Ok(alpha)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::geometry::tests::circle;
    use crate::planning::track::tests::ring_map;

    fn config() -> MinCurvatureConfig {
        MinCurvatureConfig {
            margin_m: 0.2,
            max_curvature_per_m: f64::INFINITY,
            spacing_m: 0.05,
            smoothness_weight: 0.0,
            max_step_m: 0.3,
            iterations: 20,
            tolerance_m: 0.005,
            solver_tolerance: 1e-6,
            solver_max_iterations: 100_000,
        }
    }

    #[test]
    fn the_gradient_matches_finite_differences() {
        // An ellipse, so no two points are alike.
        let line: Vec<Point2> = (0..40)
            .map(|i| {
                let angle = std::f64::consts::TAU * i as f64 / 40.0;
                Point2 {
                    x: 2.0 * angle.cos(),
                    y: angle.sin(),
                }
            })
            .collect();
        let normals: Vec<Point2> = tangents(&line).into_iter().map(left_normal).collect();
        // Below most points' curvature, so the penalty is on at some.
        let model = CurvatureModel::around(&line, &normals, 1.0, 1000.0);
        let alpha: Vec<f64> = (0..40).map(|i| 0.05 * (i as f64 * 0.7).sin()).collect();
        let weight = 0.3;

        let mut grad = vec![0.0; 40];
        model.gradient(&alpha, weight, &mut grad);
        let h = 1e-6;
        for j in 0..40 {
            let mut plus = alpha.clone();
            plus[j] += h;
            let mut minus = alpha.clone();
            minus[j] -= h;
            let numeric = (model.cost(&plus, weight) - model.cost(&minus, weight)) / (2.0 * h);
            assert!(
                (numeric - grad[j]).abs() < 1e-4 * numeric.abs().max(1.0),
                "j {j}: numeric {numeric}, analytic {}",
                grad[j]
            );
        }
    }

    #[test]
    fn a_zero_offset_reproduces_the_circle_s_curvature() {
        let line = circle(2.0, 400);
        let normals: Vec<Point2> = tangents(&line).into_iter().map(left_normal).collect();
        let model = CurvatureModel::around(&line, &normals, f64::INFINITY, 1.0);
        for kappa in model.curvatures(&vec![0.0; 400]) {
            assert!((kappa - 0.5).abs() < 1e-3, "{kappa}");
        }
    }

    #[test]
    fn a_ring_s_race_line_hugs_its_outer_wall() {
        // The flattest circle through a ring is the widest one: the outer
        // wall, `margin` inside it.
        let map = ring_map(1.0, 2.0, 0.02);
        let grid = TrackGrid::build(&map).unwrap();
        let reference = resample_even_spacing(&circle(1.5, 600), 0.05);
        let line = optimize(&reference, &grid, &config(), &mut |_| true).unwrap();

        for point in &line {
            let radius = (point.x * point.x + point.y * point.y).sqrt();
            assert!((radius - 1.8).abs() < 0.03, "radius {radius}");
        }
    }

    #[test]
    fn a_ring_tighter_than_the_limit_is_refused() {
        // Its flattest line, along the outer wall, turns with a 1.8 m radius.
        let map = ring_map(1.0, 2.0, 0.02);
        let grid = TrackGrid::build(&map).unwrap();
        let reference = resample_even_spacing(&circle(1.5, 600), 0.05);
        let config = MinCurvatureConfig {
            max_curvature_per_m: 1.0 / 2.5,
            ..config()
        };
        let result = optimize(&reference, &grid, &config, &mut |_| true);
        assert!(
            matches!(result, Err(PlanError::TooCurvy { curvature_per_m, .. })
                if (curvature_per_m - 1.0 / 1.8).abs() < 0.05),
            "{result:?}"
        );
    }

    #[test]
    fn a_track_narrower_than_the_vehicle_is_refused() {
        let map = ring_map(1.0, 1.3, 0.02);
        let grid = TrackGrid::build(&map).unwrap();
        let reference = resample_even_spacing(&circle(1.15, 600), 0.05);
        let result = optimize(&reference, &grid, &config(), &mut |_| true);
        assert!(matches!(result, Err(PlanError::TooNarrow { .. })));
    }

    #[test]
    fn progress_can_cancel() {
        let map = ring_map(1.0, 2.0, 0.02);
        let grid = TrackGrid::build(&map).unwrap();
        let reference = resample_even_spacing(&circle(1.5, 600), 0.05);
        let result = optimize(&reference, &grid, &config(), &mut |_| false);
        assert!(matches!(result, Err(PlanError::Cancelled)));
    }
}
