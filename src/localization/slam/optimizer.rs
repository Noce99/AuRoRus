//! [`optimize`]: moves every scan's pose to agree as well as possible with
//! every [`Edge`] of the pose graph - what slam_toolbox hands to Ceres
//! (`solvers/ceres_solver.cpp`). Levenberg-Marquardt over the same residual
//! as its `PoseGraph2dErrorTerm` (`solvers/ceres_utils.h`), each step's
//! sparse normal equations solved by `faer`'s sparse Cholesky.

use super::matrix3::{mul, mul_vec, transpose};
use super::pose::{Pose2, wrap_to_pi};
use super::pose_graph::Edge;
use faer::Mat;
use faer::Side;
use faer::linalg::solvers::Solve;
use faer::sparse::linalg::solvers::{Llt, SymbolicLlt};
use faer::sparse::{SparseColMat, Triplet};
use std::collections::HashMap;

/// Stop once an accepted step lowers the cost by less than this fraction -
/// Ceres' `function_tolerance`, as slam_toolbox sets it.
const FUNCTION_TOLERANCE: f64 = 1e-3;
/// Stop once no pose moves by more than this in a step (meters or
/// radians).
const PARAMETER_TOLERANCE: f64 = 1e-4;
/// Starting damping, relative to the normal equations' diagonal.
const INITIAL_LAMBDA: f64 = 1e-4;
/// How many times a rejected step may be retried with more damping before
/// giving up.
const MAX_REJECTED_STEPS: usize = 10;

/// The poses minimizing `Σ eᵀ Ω e` over `edges`, starting from `poses` -
/// vertex `0` held fixed, as slam_toolbox fixes the first node. `None` if
/// the normal equations couldn't be factorized (e.g. some vertex isn't
/// connected to vertex `0` at all), in which case the poses should be left
/// as they are.
pub fn optimize(poses: &[Pose2], edges: &[Edge], max_iterations: usize) -> Option<Vec<Pose2>> {
    let n_variables = 3 * poses.len().saturating_sub(1);
    if n_variables == 0 || edges.is_empty() {
        return Some(poses.to_vec());
    }

    let mut poses = poses.to_vec();
    let mut cost = total_cost(&poses, edges);
    let mut lambda = INITIAL_LAMBDA;
    let mut symbolic: Option<SymbolicLlt<usize>> = None;

    for _ in 0..max_iterations {
        let (hessian, gradient) = normal_equations(&poses, edges, n_variables);

        let mut accepted = None;
        for _ in 0..MAX_REJECTED_STEPS {
            let step = solve(&hessian, &gradient, lambda, n_variables, &mut symbolic)?;
            let candidate = apply(&poses, &step);
            let candidate_cost = total_cost(&candidate, edges);
            if candidate_cost < cost {
                lambda = (lambda / 10.0).max(1e-12);
                let largest_move = step.iter().fold(0.0f64, |max, v| max.max(v.abs()));
                accepted = Some((candidate, candidate_cost, largest_move));
                break;
            }
            lambda *= 10.0;
        }

        // No step lowers the cost anymore: converged (or stuck).
        let Some((candidate, candidate_cost, largest_move)) = accepted else {
            break;
        };
        let relative_decrease = (cost - candidate_cost) / cost.max(f64::MIN_POSITIVE);
        poses = candidate;
        cost = candidate_cost;
        if relative_decrease < FUNCTION_TOLERANCE || largest_move < PARAMETER_TOLERANCE {
            break;
        }
    }
    Some(poses)
}

/// `edge`'s residual at `poses` - `PoseGraph2dErrorTerm`: where `to` sits
/// in `from`'s frame, minus where the edge measured it.
fn residual(poses: &[Pose2], edge: &Edge) -> [f64; 3] {
    let a = poses[edge.from];
    let b = poses[edge.to];
    let (sin, cos) = a.heading_rad.sin_cos();
    let (dx, dy) = (b.x_m - a.x_m, b.y_m - a.y_m);
    [
        cos * dx + sin * dy - edge.diff.x_m,
        -sin * dx + cos * dy - edge.diff.y_m,
        wrap_to_pi(b.heading_rad - a.heading_rad - edge.diff.heading_rad),
    ]
}

/// `Σ eᵀ Ω e` over every edge.
fn total_cost(poses: &[Pose2], edges: &[Edge]) -> f64 {
    edges
        .iter()
        .map(|edge| {
            let e = residual(poses, edge);
            (0..3)
                .flat_map(|i| (0..3).map(move |j| (i, j)))
                .map(|(i, j)| e[i] * edge.information[i][j] * e[j])
                .sum::<f64>()
        })
        .sum()
}

/// The residual's Jacobians with respect to `from`'s and `to`'s `(x, y,
/// heading)`.
fn jacobians(poses: &[Pose2], edge: &Edge) -> ([[f64; 3]; 3], [[f64; 3]; 3]) {
    let a = poses[edge.from];
    let b = poses[edge.to];
    let (sin, cos) = a.heading_rad.sin_cos();
    let (dx, dy) = (b.x_m - a.x_m, b.y_m - a.y_m);
    let j_from = [
        [-cos, -sin, -sin * dx + cos * dy],
        [sin, -cos, -cos * dx - sin * dy],
        [0.0, 0.0, -1.0],
    ];
    let j_to = [[cos, sin, 0.0], [-sin, cos, 0.0], [0.0, 0.0, 1.0]];
    (j_from, j_to)
}

/// The normal equations `H = Σ JᵀΩJ`, `g = Σ JᵀΩe` over the free
/// variables (every vertex but `0`, three variables each, vertex `v` at
/// `3 * (v - 1)`). `H` holds its lower triangle only, every entry of every
/// touched 3x3 block present (even when zero), so its sparsity pattern
/// only depends on which vertices share an edge.
fn normal_equations(
    poses: &[Pose2],
    edges: &[Edge],
    n_variables: usize,
) -> (HashMap<(usize, usize), f64>, Vec<f64>) {
    let mut hessian: HashMap<(usize, usize), f64> = HashMap::new();
    let mut gradient = vec![0.0; n_variables];
    // Every diagonal block, so every variable has a pivot.
    for block in 0..n_variables / 3 {
        add_block(&mut hessian, block, block, &[[0.0; 3]; 3]);
    }

    for edge in edges {
        let e = residual(poses, edge);
        let (j_from, j_to) = jacobians(poses, edge);
        let omega = &edge.information;
        let blocks = [(edge.from, j_from), (edge.to, j_to)];

        for &(vertex_i, j_i) in &blocks {
            let Some(block_i) = vertex_i.checked_sub(1) else {
                continue;
            };
            // Jᵢᵀ Ω, shared by the gradient and every block of this row.
            let jt_omega = mul(&transpose(&j_i), omega);
            let g = mul_vec(&jt_omega, &e);
            for k in 0..3 {
                gradient[3 * block_i + k] += g[k];
            }
            for &(vertex_j, j_j) in &blocks {
                let Some(block_j) = vertex_j.checked_sub(1) else {
                    continue;
                };
                if block_i >= block_j {
                    add_block(&mut hessian, block_i, block_j, &mul(&jt_omega, &j_j));
                }
            }
        }
    }
    (hessian, gradient)
}

/// Adds `block` at block row `row`, block column `column` (`row >=
/// column`), keeping only the lower triangle.
fn add_block(
    hessian: &mut HashMap<(usize, usize), f64>,
    row: usize,
    column: usize,
    block: &[[f64; 3]; 3],
) {
    for (i, block_row) in block.iter().enumerate() {
        for (j, &value) in block_row.iter().enumerate() {
            let (r, c) = (3 * row + i, 3 * column + j);
            if r >= c {
                *hessian.entry((r, c)).or_insert(0.0) += value;
            }
        }
    }
}

/// Solves `(H + λ diag(H)) δ = −g` for the step `δ`, reusing (or
/// computing, the first time) the symbolic factorization.
fn solve(
    hessian: &HashMap<(usize, usize), f64>,
    gradient: &[f64],
    lambda: f64,
    n_variables: usize,
    symbolic: &mut Option<SymbolicLlt<usize>>,
) -> Option<Vec<f64>> {
    let mut entries: Vec<(usize, usize, f64)> = hessian
        .iter()
        .map(|(&(r, c), &value)| {
            // Damping, plus a floor so a variable no edge constrains yet
            // doesn't make the matrix singular outright.
            let value = if r == c {
                value * (1.0 + lambda) + 1e-9
            } else {
                value
            };
            (r, c, value)
        })
        .collect();
    // A deterministic order, so the pattern matches the symbolic
    // factorization every time.
    entries.sort_unstable_by_key(|&(r, c, _)| (c, r));
    let triplets: Vec<Triplet<usize, usize, f64>> = entries
        .into_iter()
        .map(|(r, c, value)| Triplet::new(r, c, value))
        .collect();
    let matrix =
        SparseColMat::<usize, f64>::try_new_from_triplets(n_variables, n_variables, &triplets)
            .ok()?;

    if symbolic.is_none() {
        *symbolic = Some(SymbolicLlt::try_new(matrix.symbolic(), Side::Lower).ok()?);
    }
    let llt = Llt::try_new_with_symbolic(
        symbolic.clone().expect("just set"),
        matrix.as_ref(),
        Side::Lower,
    )
    .ok()?;

    let mut rhs = Mat::<f64>::from_fn(n_variables, 1, |i, _| -gradient[i]);
    llt.solve_in_place(rhs.as_mut());
    let step: Vec<f64> = (0..n_variables).map(|i| rhs[(i, 0)]).collect();
    step.iter().all(|v| v.is_finite()).then_some(step)
}

/// `poses` moved by `step` (vertex `0` stays put), headings wrapped -
/// Karto's `AngleManifold`.
fn apply(poses: &[Pose2], step: &[f64]) -> Vec<Pose2> {
    poses
        .iter()
        .enumerate()
        .map(|(vertex, pose)| match vertex.checked_sub(1) {
            None => *pose,
            Some(block) => Pose2::new(
                pose.x_m + step[3 * block],
                pose.y_m + step[3 * block + 1],
                pose.heading_rad + step[3 * block + 2],
            ),
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::localization::slam::matrix3;
    use crate::localization::slam::pose::wrap_to_pi;
    use std::f64::consts::{FRAC_PI_2, PI};

    fn edge(from: usize, to: usize, diff: Pose2, weight: f64) -> Edge {
        Edge {
            from,
            to,
            diff,
            information: matrix3::diagonal(weight, weight, weight),
            loop_closure: false,
        }
    }

    /// The corners of a 10 m square, driven counterclockwise from the
    /// origin.
    fn square() -> Vec<Pose2> {
        vec![
            Pose2::new(0.0, 0.0, 0.0),
            Pose2::new(10.0, 0.0, FRAC_PI_2),
            Pose2::new(10.0, 10.0, PI),
            Pose2::new(0.0, 10.0, -FRAC_PI_2),
        ]
    }

    fn position_error(a: &[Pose2], b: &[Pose2]) -> f64 {
        a.iter()
            .zip(b)
            .map(|(a, b)| a.squared_distance(b).sqrt())
            .fold(0.0, f64::max)
    }

    #[test]
    fn a_loop_edge_pulls_drifted_odometry_back_onto_the_square() {
        let truth = square();
        // Every leg measured 10 m and a quarter turn, but the dead-reckoned
        // start poses drifted.
        let leg = Pose2::new(10.0, 0.0, FRAC_PI_2);
        let mut edges: Vec<Edge> = (0..3).map(|i| edge(i, i + 1, leg, 1.0)).collect();
        // The loop closure: the last corner, seen from the first.
        edges.push(edge(0, 3, truth[0].inverse().compose(&truth[3]), 100.0));
        let drifted = vec![
            truth[0],
            Pose2::new(10.3, 0.4, FRAC_PI_2 + 0.05),
            Pose2::new(10.9, 10.6, PI + 0.1),
            Pose2::new(1.5, 11.2, -FRAC_PI_2 + 0.15),
        ];

        let optimized = optimize(&drifted, &edges, 20).expect("the graph is connected");

        assert!(total_cost(&optimized, &edges) < 1e-6 * total_cost(&drifted, &edges).max(1.0));
        assert!(position_error(&optimized, &truth) < 1e-3, "{optimized:?}");
    }

    #[test]
    fn a_consistent_graph_does_not_move() {
        let truth = square();
        let edges: Vec<Edge> = (0..4)
            .map(|i| {
                let j = (i + 1) % 4;
                edge(i, j, truth[i].inverse().compose(&truth[j]), 1.0)
            })
            .collect();
        let optimized = optimize(&truth, &edges, 20).expect("the graph is connected");
        assert!(position_error(&optimized, &truth) < 1e-9);
    }

    #[test]
    fn the_first_vertex_stays_fixed() {
        // A single edge saying vertex 1 is 1 m ahead of vertex 0 - but
        // vertex 0 starts 5 m off: only vertex 1 may move to satisfy it.
        let start = vec![Pose2::new(5.0, 0.0, 0.0), Pose2::new(0.0, 0.0, 0.0)];
        let edges = [edge(0, 1, Pose2::new(1.0, 0.0, 0.0), 1.0)];
        let optimized = optimize(&start, &edges, 20).expect("the graph is connected");
        assert_eq!(optimized[0], start[0]);
        assert!(optimized[1].squared_distance(&Pose2::new(6.0, 0.0, 0.0)) < 1e-12);
    }

    #[test]
    fn headings_across_plus_minus_pi_are_handled() {
        // Vertex 1 is measured 0.2 rad further left than vertex 0, which
        // faces just short of pi: vertex 1 ends up just past -pi.
        let start = vec![
            Pose2::new(0.0, 0.0, PI - 0.1),
            Pose2::new(-1.0, 0.0, PI - 0.1),
        ];
        let edges = [edge(0, 1, Pose2::new(1.0, 0.0, 0.2), 1.0)];
        let optimized = optimize(&start, &edges, 20).expect("the graph is connected");
        assert!(wrap_to_pi(optimized[1].heading_rad - (-PI + 0.1)).abs() < 1e-9);
        assert!(optimized[1].heading_rad > -PI && optimized[1].heading_rad <= PI);
    }

    #[test]
    fn a_graph_without_edges_is_returned_unchanged() {
        let poses = square();
        assert_eq!(optimize(&poses, &[], 20), Some(poses));
    }
}
