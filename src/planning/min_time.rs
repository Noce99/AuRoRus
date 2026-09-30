//! The minimum-time race line: the path through the track *and* the speed
//! along it that complete a lap fastest, for a point mass within a friction
//! ellipse plus a motor limit (see [`super::speed_profile`]) - solved with
//! OpEn's augmented Lagrangian method (ALM), whose inner problems PANOC
//! solves.
//!
//! Variables: the sideways offsets `n_i` of a reference line's points (the
//! minimum-curvature line) along its normals, and every point's squared
//! speed `V_i = v_i^2`. The offsets aren't free per point: they follow a
//! periodic cubic B-spline through control values `z_j` spaced
//! `control_spacing` apart, `n = B z`. Free offsets 0.1-0.2 m apart make the
//! problem far too stiff for PANOC - curvature reacts to them like
//! `1/spacing^2`, squared by the ALM penalty - and a line that wiggles at
//! that scale is useless anyway. The box stays exact: B-spline weights are
//! nonnegative and sum to one, so each point's offset lies between its
//! control values, and bounding every control value by the tightest point
//! limit over its support keeps every point on the track. Positions
//! `p_i = c_i + n_i nu_i`, segment lengths
//! `d_i = |p_{i+1} - p_i|` and curvatures `kappa_i` (central differences,
//! see [`curvature_jacobian`]) are computed exactly from the offsets -
//! nothing is linearized.
//!
//! - **Cost:** the lap time `sum 2 d_i / (v_i + v_{i+1})`, exact for a
//!   constant acceleration along each segment.
//! - **Box** (projected exactly): each control value within the track,
//!   `margin` inside both walls, over its whole support; each squared speed
//!   within `[v_min^2, v_max^2]`.
//! - **`F1(u) in C`** (the ALM part), with `a_lon,i = (V_{i+1} - V_i) /
//!   (2 d_i)` and `a_lat,i = V_i kappa_i`: `(a_lon,i / max_decel, a_lat,i /
//!   max_lateral)` in the unit disk (the friction ellipse), `a_lon,i /
//!   max_accel <= 1` (the motor), and `kappa_i / kappa_max` in `[-1, 1]`
//!   (the vehicle's steering).
//!
//! Cost, gradient, `F1` and `JF1' y` only couple neighboring points, so
//! each costs O(n). The ALM's outer iterations run one at a time here, the
//! Lagrange multipliers and penalty carried from one to the next, so
//! progress can be reported - and the planning cancelled - in between.

use super::PlanError;
use super::geometry::{Point2, curvature_jacobian, loop_length, resample_even_spacing};
use super::min_curvature::Offsets;
use super::speed_profile::{SpeedLimits, speeds};
use super::track::TrackGrid;
use optimization_engine::alm::{
    AlmCache, AlmFactory, AlmOptimizer, AlmProblem, NO_JACOBIAN_MAPPING, NO_MAPPING,
};
use optimization_engine::constraints::{Ball2, Constraint, Rectangle};
use optimization_engine::core::ExitStatus;
use optimization_engine::panoc::PANOCCache;
use optimization_engine::{FunctionCallResult, SolverError};
use std::time::{Duration, Instant};

/// Memory of PANOC's L-BFGS directions.
const LBFGS_MEMORY: usize = 10;
/// PANOC's tolerance on its fixed-point residual, in every inner problem.
const INNER_TOLERANCE: f64 = 1e-6;
/// ALM penalty to start from.
const INITIAL_PENALTY: f64 = 10.0;
/// Factor the penalty grows by when the constraint violation didn't shrink
/// enough in an outer iteration.
const PENALTY_GROWTH: f64 = 5.0;
/// The violation must shrink by this factor per outer iteration, or the
/// penalty grows.
const SUFFICIENT_DECREASE: f64 = 0.25;

/// Every tunable of [`optimize`].
#[derive(Debug, Clone, Copy)]
pub struct MinTimeConfig {
    /// Distance to keep from either wall, in meters.
    pub margin_m: f64,
    /// Tightest the line may turn, in 1/m - `f64::INFINITY` for no limit.
    pub max_curvature_per_m: f64,
    /// Spacing of the optimized points, in meters.
    pub spacing_m: f64,
    /// Spacing of the B-spline control values the offsets follow, in
    /// meters.
    pub control_spacing_m: f64,
    /// Lowest speed allowed anywhere, in m/s - keeps the lap time finite
    /// and smooth.
    pub min_speed_mps: f64,
    /// The vehicle's limits.
    pub limits: SpeedLimits,
    /// Most ALM outer iterations.
    pub max_outer_iterations: usize,
    /// Most PANOC iterations per inner problem.
    pub max_inner_iterations: usize,
    /// Time budget for the whole optimization.
    pub max_duration: Duration,
    /// Converged once the constraints are violated by at most this much,
    /// relative to their limits (e.g. `0.01`: 1% over the ellipse).
    pub tolerance: f64,
}

/// One outer iteration's result, for [`optimize`]'s progress callback.
pub struct Iteration<'a> {
    /// Which iteration this is, from `1`.
    pub number: usize,
    /// The line so far.
    pub points: &'a [Point2],
    /// Its lap time at the optimized speeds, in seconds.
    pub lap_time_s: f64,
    /// Largest constraint violation, relative to the limits.
    pub violation: f64,
}

/// The minimum-time line from `reference` (a closed loop inside `grid`'s
/// track, starting at the start/finish line - the minimum-curvature line).
/// Returns its points; their speeds are left to the caller's speed profile,
/// which satisfies the limits exactly where the ALM only does up to
/// `tolerance`. `progress` is told about every outer iteration, and stops
/// the optimization - with [`PlanError::Cancelled`] - by returning `false`.
pub fn optimize(
    reference: &[Point2],
    grid: &TrackGrid,
    config: &MinTimeConfig,
    progress: &mut dyn FnMut(Iteration) -> bool,
) -> Result<Vec<Point2>, PlanError> {
    let started = Instant::now();
    // A whole number of points per control interval, all evenly spaced.
    let fine = resample_even_spacing(reference, config.spacing_m);
    let per_interval = ((config.control_spacing_m / config.spacing_m).round() as usize).max(1);
    let controls = (fine.len() / per_interval).max(4);
    let wanted = controls * per_interval;
    let reference = resample_even_spacing(&fine, loop_length(&fine) / wanted as f64);
    if reference.len() != wanted {
        return Err(PlanError::Solver(format!(
            "resampling gave {} points instead of {wanted}",
            reference.len()
        )));
    }
    let n = reference.len();
    let offsets = Offsets::along(&reference, grid);
    // The reference is feasible, so every point's box is widened to contain it.
    let (point_lower, point_upper) = offsets.bounds(config.margin_m, false)?;
    let limits = &config.limits;
    let model = Model {
        reference: &reference,
        normals: &offsets.normals,
        limits: *limits,
        max_curvature_per_m: config.max_curvature_per_m,
        per_interval,
    };
    let (mut lower, mut upper) = model.control_bounds(&point_lower, &point_upper);
    lower.extend(std::iter::repeat_n(config.min_speed_mps.powi(2), n));
    upper.extend(std::iter::repeat_n(limits.max_speed_mps.powi(2), n));

    // Start from the reference, at its own speed profile - feasible.
    let mut u = vec![0.0; controls];
    u.extend(
        speeds(&reference, limits)
            .into_iter()
            .zip(&lower[controls..])
            .map(|(v, lo)| (v * v).max(*lo)),
    );

    let set_c = MinTimeSet { points: n };
    let n1 = ROWS_PER_POINT * n;
    let mut multipliers = vec![0.0; n1];
    let mut penalty = INITIAL_PENALTY;
    let mut inner_tolerance = 1e-2_f64;
    let mut last_violation = model.violation(&u);
    let mut alm_cache = AlmCache::new(
        PANOCCache::new(controls + n, INNER_TOLERANCE, LBFGS_MEMORY),
        n1,
        0,
    );

    for number in 1..=config.max_outer_iterations.max(1) {
        let Some(time_left) = config.max_duration.checked_sub(started.elapsed()) else {
            return Err(PlanError::MinTimeNotConverged {
                reason: format!("out of time after {} s", config.max_duration.as_secs_f64()),
            });
        };
        let status = {
            let bounds = Rectangle::new(Some(&lower), Some(&upper));
            let factory = AlmFactory::new(
                |u: &[f64], cost: &mut f64| -> FunctionCallResult {
                    *cost = model.lap_time(u);
                    Ok(())
                },
                |u: &[f64], grad: &mut [f64]| -> FunctionCallResult {
                    model.lap_time_gradient(u, grad);
                    Ok(())
                },
                Some(|u: &[f64], out: &mut [f64]| -> FunctionCallResult {
                    model.constraints(u, out);
                    Ok(())
                }),
                Some(
                    |u: &[f64], y: &[f64], out: &mut [f64]| -> FunctionCallResult {
                        model.constraints_jacobian_transpose(u, y, out);
                        Ok(())
                    },
                ),
                NO_MAPPING,
                NO_JACOBIAN_MAPPING,
                Some(set_c),
                0,
            );
            let problem = AlmProblem::new(
                bounds,
                Some(set_c),
                Some(Ball2::new(None, 1e12)),
                |u: &[f64], xi: &[f64], cost: &mut f64| -> FunctionCallResult {
                    factory.psi(u, xi, cost)
                },
                |u: &[f64], xi: &[f64], grad: &mut [f64]| -> FunctionCallResult {
                    factory.d_psi(u, xi, grad)
                },
                Some(|u: &[f64], out: &mut [f64]| -> FunctionCallResult {
                    model.constraints(u, out);
                    Ok(())
                }),
                NO_MAPPING,
                n1,
                0,
            );
            AlmOptimizer::new(&mut alm_cache, problem)
                .with_max_outer_iterations(1)
                .with_max_inner_iterations(config.max_inner_iterations)
                .with_max_duration(time_left)
                .with_initial_lagrange_multipliers(&multipliers)
                .with_initial_penalty(penalty)
                .with_initial_inner_tolerance(inner_tolerance)
                .with_epsilon_tolerance(INNER_TOLERANCE)
                .solve(&mut u)
                .map_err(|err: SolverError| PlanError::Solver(err.to_string()))?
        };
        if status.exit_status() == ExitStatus::NotConvergedOutOfTime {
            return Err(PlanError::MinTimeNotConverged {
                reason: format!("out of time after {} s", config.max_duration.as_secs_f64()),
            });
        }
        if let Some(updated) = status.lagrange_multipliers() {
            multipliers.clone_from(updated);
        }

        let violation = model.violation(&u);
        let points = model.positions(&u);
        let keep_going = progress(Iteration {
            number,
            points: &points,
            lap_time_s: model.lap_time(&u),
            violation,
        });
        if !keep_going {
            return Err(PlanError::Cancelled);
        }
        let inner_converged =
            status.last_problem_norm_fpr() <= inner_tolerance.max(INNER_TOLERANCE);
        if violation <= config.tolerance
            && inner_tolerance <= INNER_TOLERANCE * 10.0
            && inner_converged
        {
            return Ok(points);
        }
        if violation > SUFFICIENT_DECREASE * last_violation && violation > config.tolerance {
            penalty *= PENALTY_GROWTH;
        }
        last_violation = violation;
        inner_tolerance = (inner_tolerance * 0.1).max(INNER_TOLERANCE);
    }
    Err(PlanError::MinTimeNotConverged {
        reason: format!(
            "the constraints were still violated by {:.1}% after {} outer iterations",
            100.0 * last_violation,
            config.max_outer_iterations
        ),
    })
}

/// Rows of `F1` per point: the ellipse pair, the motor term, the curvature.
const ROWS_PER_POINT: usize = 4;

/// The ALM's constraint set `C`: `points` unit disks (the scaled friction
/// ellipses, one `(a_lon / max_decel, a_lat / max_lateral)` pair per point),
/// then `points` half-lines `x <= 1` (the scaled motor limits), then
/// `points` intervals `[-1, 1]` (the scaled curvatures).
#[derive(Debug, Clone, Copy)]
struct MinTimeSet {
    points: usize,
}

impl Constraint for MinTimeSet {
    fn project(&self, x: &mut [f64]) -> FunctionCallResult {
        let (disks, rest) = x.split_at_mut(2 * self.points);
        let (motor, curvature) = rest.split_at_mut(self.points);
        for pair in disks.as_chunks_mut::<2>().0 {
            let norm = (pair[0] * pair[0] + pair[1] * pair[1]).sqrt();
            if norm > 1.0 {
                pair[0] /= norm;
                pair[1] /= norm;
            }
        }
        for value in motor {
            *value = value.min(1.0);
        }
        for value in curvature {
            *value = value.clamp(-1.0, 1.0);
        }
        Ok(())
    }

    fn is_convex(&self) -> bool {
        true
    }
}

/// The problem around one reference line: `u = (z_1..z_M, V_1..V_N)`, the
/// offsets' B-spline control values then every point's squared speed.
struct Model<'a> {
    reference: &'a [Point2],
    normals: &'a [Point2],
    limits: SpeedLimits,
    /// `kappa_max`, in 1/m.
    max_curvature_per_m: f64,
    /// Points per control interval: `N = per_interval * M`.
    per_interval: usize,
}

/// Every segment's length and unit direction, from point `i` to `i + 1`.
struct Segments {
    length: Vec<f64>,
    direction: Vec<Point2>,
}

/// The four uniform cubic B-spline weights at `s` in `[0, 1)` along a
/// control interval, for its control values `j - 1`, `j`, `j + 1`, `j + 2`.
fn basis(s: f64) -> [f64; 4] {
    let (s2, s3) = (s * s, s * s * s);
    [
        (1.0 - s).powi(3) / 6.0,
        (3.0 * s3 - 6.0 * s2 + 4.0) / 6.0,
        (-3.0 * s3 + 3.0 * s2 + 3.0 * s + 1.0) / 6.0,
        s3 / 6.0,
    ]
}

impl Model<'_> {
    /// Number of points, `N`.
    fn len(&self) -> usize {
        self.reference.len()
    }

    /// Number of control values, `M`.
    fn controls(&self) -> usize {
        self.len() / self.per_interval
    }

    /// Point `i`'s four control values (indices) and their weights.
    fn stencil(&self, i: usize) -> ([usize; 4], [f64; 4]) {
        let m = self.controls();
        let interval = i / self.per_interval;
        let s = (i % self.per_interval) as f64 / self.per_interval as f64;
        let indices = [
            (interval + m - 1) % m,
            interval % m,
            (interval + 1) % m,
            (interval + 2) % m,
        ];
        (indices, basis(s))
    }

    /// Every point's offset, `n = B z`.
    fn offsets(&self, z: &[f64]) -> Vec<f64> {
        (0..self.len())
            .map(|i| {
                let (indices, weights) = self.stencil(i);
                indices.iter().zip(weights).map(|(&j, w)| w * z[j]).sum()
            })
            .collect()
    }

    /// Adds `B' g` - a gradient with respect to the offsets, taken back to
    /// the control values - into `out`.
    fn add_transposed(&self, g: &[f64], out: &mut [f64]) {
        for (i, gi) in g.iter().enumerate() {
            let (indices, weights) = self.stencil(i);
            for (j, w) in indices.into_iter().zip(weights) {
                out[j] += w * gi;
            }
        }
    }

    /// Bounds on every control value keeping every point it moves within
    /// `lower..=upper` (per point): the tightest over its support, points
    /// `(j - 2) k .. (j + 2) k`.
    fn control_bounds(&self, lower: &[f64], upper: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let (n, k) = (self.len(), self.per_interval);
        (0..self.controls())
            .map(|j| {
                (0..4 * k)
                    .map(|offset| (j * k + n + offset - 2 * k) % n)
                    .fold((f64::NEG_INFINITY, f64::INFINITY), |(lo, hi), i| {
                        (lo.max(lower[i]), hi.min(upper[i]))
                    })
            })
            .unzip()
    }

    fn positions(&self, u: &[f64]) -> Vec<Point2> {
        let offsets = self.offsets(&u[..self.controls()]);
        self.reference
            .iter()
            .zip(self.normals)
            .zip(offsets)
            .map(|((c, nu), offset)| Point2 {
                x: c.x + offset * nu.x,
                y: c.y + offset * nu.y,
            })
            .collect()
    }

    fn segments(points: &[Point2]) -> Segments {
        let n = points.len();
        let mut length = Vec::with_capacity(n);
        let mut direction = Vec::with_capacity(n);
        for i in 0..n {
            let (a, b) = (points[i], points[(i + 1) % n]);
            let d = a.distance(&b).max(1e-9);
            length.push(d);
            direction.push(Point2 {
                x: (b.x - a.x) / d,
                y: (b.y - a.y) / d,
            });
        }
        Segments { length, direction }
    }

    /// `d d_i / d n_i` and `d d_i / d n_{i+1}`.
    fn length_derivatives(&self, segments: &Segments, i: usize) -> (f64, f64) {
        let n = self.len();
        let e = segments.direction[i];
        let dot = |a: Point2, b: Point2| a.x * b.x + a.y * b.y;
        (-dot(e, self.normals[i]), dot(e, self.normals[(i + 1) % n]))
    }

    /// `sum 2 d_i / (sqrt(V_i) + sqrt(V_{i+1}))`.
    fn lap_time(&self, u: &[f64]) -> f64 {
        let n = self.len();
        let segments = Self::segments(&self.positions(u));
        let v = &u[self.controls()..];
        (0..n)
            .map(|i| 2.0 * segments.length[i] / (v[i].sqrt() + v[(i + 1) % n].sqrt()))
            .sum()
    }

    fn lap_time_gradient(&self, u: &[f64], grad: &mut [f64]) {
        let (n, m) = (self.len(), self.controls());
        let segments = Self::segments(&self.positions(u));
        let v = &u[m..];
        let s: Vec<f64> = v.iter().map(|x| x.sqrt()).collect();
        grad.fill(0.0);
        let mut by_offset = vec![0.0; n];
        for i in 0..n {
            let q = (i + 1) % n;
            let sum = s[i] + s[q];
            // d/dd_i of 2 d_i / sum, through the offsets moving p_i, p_{i+1}.
            let per_length = 2.0 / sum;
            let (d_this, d_next) = self.length_derivatives(&segments, i);
            by_offset[i] += per_length * d_this;
            by_offset[q] += per_length * d_next;
            // d/dV of 2 d / (sqrt(V_i) + sqrt(V_{i+1})).
            let per_root = -2.0 * segments.length[i] / (sum * sum);
            grad[m + i] += per_root / (2.0 * s[i]);
            grad[m + q] += per_root / (2.0 * s[q]);
        }
        self.add_transposed(&by_offset, &mut grad[..m]);
    }

    /// `F1(u)`: the scaled ellipse pairs, then the scaled motor terms, then
    /// the scaled curvatures.
    fn constraints(&self, u: &[f64], out: &mut [f64]) {
        let n = self.len();
        let points = self.positions(u);
        let segments = Self::segments(&points);
        let rows = curvature_jacobian(&points, self.normals);
        let v = &u[self.controls()..];
        for i in 0..n {
            let a_lon = (v[(i + 1) % n] - v[i]) / (2.0 * segments.length[i]);
            let a_lat = v[i] * rows[i].kappa;
            out[2 * i] = a_lon / self.limits.max_decel_mps2;
            out[2 * i + 1] = a_lat / self.limits.max_lateral_accel_mps2;
            out[2 * n + i] = a_lon / self.limits.max_accel_mps2;
            out[3 * n + i] = rows[i].kappa / self.max_curvature_per_m;
        }
    }

    /// `JF1(u)' y`, written into `out`.
    fn constraints_jacobian_transpose(&self, u: &[f64], y: &[f64], out: &mut [f64]) {
        let (n, m) = (self.len(), self.controls());
        let points = self.positions(u);
        let segments = Self::segments(&points);
        let rows = curvature_jacobian(&points, self.normals);
        let v = &u[m..];
        out.fill(0.0);
        let mut by_offset = vec![0.0; n];
        for i in 0..n {
            let (p, q) = ((i + n - 1) % n, (i + 1) % n);
            // Both the ellipse and the motor term are multiples of a_lon,i.
            let on_lon =
                y[2 * i] / self.limits.max_decel_mps2 + y[2 * n + i] / self.limits.max_accel_mps2;
            let on_lat = y[2 * i + 1] / self.limits.max_lateral_accel_mps2;
            let on_curvature = y[3 * n + i] / self.max_curvature_per_m;

            // a_lon,i = (V_{i+1} - V_i) / (2 d_i).
            let d = segments.length[i];
            out[m + q] += on_lon / (2.0 * d);
            out[m + i] -= on_lon / (2.0 * d);
            let per_length = -on_lon * (v[q] - v[i]) / (2.0 * d * d);
            let (d_this, d_next) = self.length_derivatives(&segments, i);
            by_offset[i] += per_length * d_this;
            by_offset[q] += per_length * d_next;

            // a_lat,i = V_i kappa_i(n_{i-1}, n_i, n_{i+1}).
            let row = rows[i];
            out[m + i] += on_lat * row.kappa;
            by_offset[p] += on_lat * v[i] * row.prev;
            by_offset[i] += on_lat * v[i] * row.this;
            by_offset[q] += on_lat * v[i] * row.next;

            // kappa_i / kappa_max.
            by_offset[p] += on_curvature * row.prev;
            by_offset[i] += on_curvature * row.this;
            by_offset[q] += on_curvature * row.next;
        }
        self.add_transposed(&by_offset, &mut out[..m]);
    }

    /// Largest violation of `C`, relative to the limits: how far any
    /// ellipse pair sits outside the unit disk, or any motor term or
    /// curvature magnitude above 1.
    fn violation(&self, u: &[f64]) -> f64 {
        let n = self.len();
        let mut f1 = vec![0.0; ROWS_PER_POINT * n];
        self.constraints(u, &mut f1);
        let disks = f1[..2 * n]
            .as_chunks::<2>()
            .0
            .iter()
            .map(|pair| (pair[0] * pair[0] + pair[1] * pair[1]).sqrt() - 1.0);
        let motor = f1[2 * n..3 * n].iter().map(|m| m - 1.0);
        let curvature = f1[3 * n..].iter().map(|k| k.abs() - 1.0);
        disks.chain(motor).chain(curvature).fold(0.0, f64::max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::geometry::tests::circle;
    use crate::planning::geometry::{curvatures, left_normal, tangents};
    use crate::planning::track::tests::ring_map;

    fn limits() -> SpeedLimits {
        SpeedLimits {
            max_speed_mps: 8.0,
            max_lateral_accel_mps2: 4.0,
            max_accel_mps2: 3.0,
            max_decel_mps2: 5.0,
        }
    }

    /// An ellipse, so no two points are alike, with offsets and speeds that
    /// vary along it.
    fn setup() -> (Vec<Point2>, Vec<Point2>, Vec<f64>) {
        let reference: Vec<Point2> = (0..40)
            .map(|i| {
                let angle = std::f64::consts::TAU * i as f64 / 40.0;
                Point2 {
                    x: 3.0 * angle.cos(),
                    y: 2.0 * angle.sin(),
                }
            })
            .collect();
        let normals: Vec<Point2> = tangents(&reference).into_iter().map(left_normal).collect();
        // 10 control values, 4 points each.
        let mut u: Vec<f64> = (0..10).map(|i| 0.1 * (i as f64 * 0.7).sin()).collect();
        u.extend((0..40).map(|i| 4.0 + 2.0 * (i as f64 * 0.3).cos()));
        (reference, normals, u)
    }

    fn assert_close(numeric: f64, analytic: f64, what: &str) {
        assert!(
            (numeric - analytic).abs() < 1e-5 * numeric.abs().max(1.0),
            "{what}: numeric {numeric}, analytic {analytic}"
        );
    }

    #[test]
    fn the_lap_time_gradient_matches_finite_differences() {
        let (reference, normals, u) = setup();
        let model = Model {
            reference: &reference,
            normals: &normals,
            limits: limits(),
            max_curvature_per_m: 0.4,
            per_interval: 4,
        };
        let mut grad = vec![0.0; u.len()];
        model.lap_time_gradient(&u, &mut grad);
        let h = 1e-6;
        for j in 0..u.len() {
            let (mut plus, mut minus) = (u.clone(), u.clone());
            plus[j] += h;
            minus[j] -= h;
            let numeric = (model.lap_time(&plus) - model.lap_time(&minus)) / (2.0 * h);
            assert_close(numeric, grad[j], &format!("d T / d u_{j}"));
        }
    }

    #[test]
    fn the_constraint_jacobian_transpose_matches_finite_differences() {
        let (reference, normals, u) = setup();
        let model = Model {
            reference: &reference,
            normals: &normals,
            limits: limits(),
            max_curvature_per_m: 0.4,
            per_interval: 4,
        };
        let n1 = ROWS_PER_POINT * reference.len();
        let y: Vec<f64> = (0..n1).map(|i| (i as f64 * 0.37).sin()).collect();
        let mut analytic = vec![0.0; u.len()];
        model.constraints_jacobian_transpose(&u, &y, &mut analytic);
        // y . F1(u), differentiated numerically.
        let weighted = |u: &[f64]| {
            let mut f1 = vec![0.0; n1];
            model.constraints(u, &mut f1);
            f1.iter().zip(&y).map(|(f, y)| f * y).sum::<f64>()
        };
        let h = 1e-6;
        for j in 0..u.len() {
            let (mut plus, mut minus) = (u.clone(), u.clone());
            plus[j] += h;
            minus[j] -= h;
            let numeric = (weighted(&plus) - weighted(&minus)) / (2.0 * h);
            assert_close(numeric, analytic[j], &format!("(JF1' y)_{j}"));
        }
    }

    #[test]
    fn the_constraint_set_projects_onto_disks_and_half_lines() {
        let set = MinTimeSet { points: 2 };
        let mut x = [3.0, 4.0, 0.3, 0.4, 2.0, 0.5, -1.5, 0.7];
        set.project(&mut x).unwrap();
        assert!((x[0] - 0.6).abs() < 1e-12 && (x[1] - 0.8).abs() < 1e-12);
        assert_eq!(&x[2..], &[0.3, 0.4, 1.0, 0.5, -1.0, 0.7]);
    }

    fn config() -> MinTimeConfig {
        MinTimeConfig {
            margin_m: 0.2,
            max_curvature_per_m: f64::INFINITY,
            spacing_m: 0.1,
            control_spacing_m: 0.5,
            min_speed_mps: 0.5,
            limits: limits(),
            max_outer_iterations: 30,
            max_inner_iterations: 100_000,
            max_duration: Duration::from_secs(60),
            tolerance: 0.01,
        }
    }

    #[test]
    fn a_ring_s_fastest_line_is_its_innermost_circle() {
        // At the cornering limit sqrt(a_lat R) all the way round, a lap of
        // a circle takes 2 pi sqrt(R / a_lat): the smallest circle wins -
        // unlike minimum curvature, which picks the widest. Started from
        // the centerline, so it has to get there.
        let map = ring_map(1.0, 2.0, 0.02);
        let grid = TrackGrid::build(&map).unwrap();
        let reference = resample_even_spacing(&circle(1.5, 600), 0.1);
        let mut iterations = 0;
        let line = optimize(&reference, &grid, &config(), &mut |_| {
            iterations += 1;
            true
        })
        .unwrap();
        assert!(iterations < 30, "took {iterations} outer iterations");
        for point in &line {
            let radius = (point.x * point.x + point.y * point.y).sqrt();
            assert!((radius - 1.2).abs() < 0.03, "radius {radius}");
        }
    }

    #[test]
    fn a_ring_s_fastest_line_turns_no_tighter_than_the_limit() {
        // Without the limit, the innermost circle (radius 1.2 m) - with it,
        // a circle as small as the vehicle may turn on (anywhere in the
        // ring: all are as fast).
        let map = ring_map(1.0, 2.0, 0.02);
        let grid = TrackGrid::build(&map).unwrap();
        let reference = resample_even_spacing(&circle(1.8, 600), 0.1);
        let config = MinTimeConfig {
            max_curvature_per_m: 1.0 / 1.5,
            ..config()
        };
        let line = optimize(&reference, &grid, &config, &mut |_| true).unwrap();
        for curvature in curvatures(&line) {
            assert!(curvature.abs() < 1.02 / 1.5, "curvature {curvature} 1/m");
        }
        let length = loop_length(&line);
        assert!(
            (length - std::f64::consts::TAU * 1.5).abs() < 0.1,
            "length {length} m"
        );
    }

    #[test]
    fn control_bounds_hold_every_point_they_move() {
        let (reference, normals, _) = setup();
        let model = Model {
            reference: &reference,
            normals: &normals,
            limits: limits(),
            max_curvature_per_m: 0.4,
            per_interval: 4,
        };
        let lower: Vec<f64> = (0..40).map(|i| -0.3 - 0.1 * (i as f64).sin()).collect();
        let upper: Vec<f64> = (0..40).map(|i| 0.3 + 0.1 * (i as f64).cos()).collect();
        let (lo, hi) = model.control_bounds(&lower, &upper);
        // Every corner of the control box keeps every point in bounds.
        for corner in 0..64u32 {
            let z: Vec<f64> = (0..10)
                .map(|j| {
                    if corner >> (j % 6) & 1 == 1 {
                        hi[j]
                    } else {
                        lo[j]
                    }
                })
                .collect();
            for (i, offset) in model.offsets(&z).into_iter().enumerate() {
                assert!(offset >= lower[i] - 1e-12 && offset <= upper[i] + 1e-12);
            }
        }
    }

    #[test]
    fn progress_can_cancel() {
        let map = ring_map(1.0, 2.0, 0.02);
        let grid = TrackGrid::build(&map).unwrap();
        let reference = resample_even_spacing(&circle(1.5, 600), 0.1);
        let result = optimize(&reference, &grid, &config(), &mut |_| false);
        assert!(matches!(result, Err(PlanError::Cancelled)));
    }
}
