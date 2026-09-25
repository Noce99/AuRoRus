//! [`PoseGraph`]: which scans have been matched to which - Karto's
//! `MapperGraph` bookkeeping. Every kept scan is a vertex (numbered like
//! the scans themselves); every accepted match between two scans is an
//! [`Edge`] measuring where the second one sits relative to the first.
//! [`super::optimizer`] then finds the poses agreeing best with every edge.

use super::matrix3::{self, Matrix3};
use super::pose::Pose2;
use std::collections::{HashSet, VecDeque};

/// A measured relative pose between two scans - Karto's `LinkInfo`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Edge {
    pub from: usize,
    pub to: usize,
    /// Where `to` sits in `from`'s frame.
    pub diff: Pose2,
    /// How much to trust `diff`: the inverse of the match's covariance,
    /// expressed in `from`'s frame.
    pub information: Matrix3,
    /// Whether this edge closed a loop - only used to draw it.
    pub loop_closure: bool,
}

/// The graph: edges plus, per vertex, the vertices it shares an edge with.
#[derive(Default)]
pub struct PoseGraph {
    edges: Vec<Edge>,
    adjacency: Vec<Vec<usize>>,
    /// `(from, to)` of every edge - Karto never links the same pair twice.
    linked: HashSet<(usize, usize)>,
}

impl PoseGraph {
    pub fn clear(&mut self) {
        *self = Self::default();
    }

    /// Adds the next vertex, numbered like the next scan.
    pub fn add_vertex(&mut self) {
        self.adjacency.push(Vec::new());
    }

    pub fn edges(&self) -> &[Edge] {
        &self.edges
    }

    /// Links `from` (currently at `from_pose`) to `to`, measured at `mean`
    /// with `covariance` (both in the SLAM frame) - Karto's
    /// `MapperGraph::LinkScans` plus `LinkInfo::Update`. Returns whether a
    /// new edge was added: `false` if the pair is already linked, or if the
    /// covariance can't be inverted.
    pub fn link(
        &mut self,
        from: usize,
        to: usize,
        from_pose: Pose2,
        mean: Pose2,
        covariance: &Matrix3,
        loop_closure: bool,
    ) -> bool {
        if from == to || self.linked.contains(&(from, to)) {
            return false;
        }
        // The covariance, rotated into `from`'s frame.
        let rotation = matrix3::rotation(-from_pose.heading_rad);
        let rotated = matrix3::mul(
            &matrix3::mul(&rotation, covariance),
            &matrix3::transpose(&rotation),
        );
        let Some(information) = matrix3::inverse(&rotated) else {
            return false;
        };

        self.linked.insert((from, to));
        self.adjacency[from].push(to);
        self.adjacency[to].push(from);
        self.edges.push(Edge {
            from,
            to,
            diff: from_pose.inverse().compose(&mean),
            information,
            loop_closure,
        });
        true
    }

    /// Every vertex reachable from `start` through edges without ever
    /// leaving `max_distance_m` of it, `start` included - Karto's
    /// `FindNearLinkedScans` (a breadth-first traversal with a
    /// `NearScanVisitor`). `pose` gives each vertex's current pose.
    pub fn near_linked(
        &self,
        start: usize,
        max_distance_m: f64,
        pose: impl Fn(usize) -> Pose2,
    ) -> Vec<usize> {
        let center = pose(start);
        let max_squared = max_distance_m.powi(2);
        let mut seen = HashSet::from([start]);
        let mut to_visit = VecDeque::from([start]);
        let mut near = Vec::new();
        while let Some(vertex) = to_visit.pop_front() {
            if pose(vertex).squared_distance(&center) > max_squared - TOLERANCE {
                continue;
            }
            near.push(vertex);
            for &neighbor in &self.adjacency[vertex] {
                if seen.insert(neighbor) {
                    to_visit.push_back(neighbor);
                }
            }
        }
        near
    }
}

/// Karto's `KT_TOLERANCE`.
const TOLERANCE: f64 = 1e-6;

/// The covariance-weighted mean of `means` - Karto's
/// `MapperGraph::ComputeWeightedMean`: positions weighted by each
/// covariance's inverse, headings averaged on the circle.
///
/// # Panics
///
/// Panics if `means` is empty or its length differs from `covariances`'.
pub fn weighted_mean(means: &[Pose2], covariances: &[Matrix3]) -> Pose2 {
    assert!(!means.is_empty() && means.len() == covariances.len());
    let inverses: Vec<Matrix3> = covariances
        .iter()
        .map(|covariance| matrix3::inverse(covariance).unwrap_or(matrix3::identity()))
        .collect();
    let sum = inverses
        .iter()
        .fold([[0.0; 3]; 3], |sum, inverse| matrix3::add(&sum, inverse));
    let inverse_of_sum = matrix3::inverse(&sum).unwrap_or(matrix3::identity());

    let (mut x, mut y, mut cos, mut sin) = (0.0, 0.0, 0.0, 0.0);
    for (mean, inverse) in means.iter().zip(&inverses) {
        let weight = matrix3::mul(&inverse_of_sum, inverse);
        let weighted = matrix3::mul_vec(&weight, &[mean.x_m, mean.y_m, mean.heading_rad]);
        x += weighted[0];
        y += weighted[1];
        cos += mean.heading_rad.cos();
        sin += mean.heading_rad.sin();
    }
    Pose2::new(x, y, sin.atan2(cos))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    #[test]
    fn an_edge_measures_the_target_in_the_source_frame() {
        let mut graph = PoseGraph::default();
        graph.add_vertex();
        graph.add_vertex();
        // Facing +y at (1, 1); the target 2 m ahead (at (1, 3)) and turned
        // 0.1 rad further left.
        let from = Pose2::new(1.0, 1.0, FRAC_PI_2);
        let to = Pose2::new(1.0, 3.0, FRAC_PI_2 + 0.1);
        let covariance = matrix3::diagonal(0.01, 0.04, 0.001);
        assert!(graph.link(0, 1, from, to, &covariance, false));

        let edge = graph.edges()[0];
        assert!((edge.diff.x_m - 2.0).abs() < 1e-9 && edge.diff.y_m.abs() < 1e-9);
        assert!((edge.diff.heading_rad - 0.1).abs() < 1e-9);
        // The world's y variance (0.04) is `from`'s forward (x) variance.
        assert!((edge.information[0][0] - 1.0 / 0.04).abs() < 1e-6);
        assert!((edge.information[1][1] - 1.0 / 0.01).abs() < 1e-6);
    }

    #[test]
    fn a_pair_is_only_linked_once() {
        let mut graph = PoseGraph::default();
        graph.add_vertex();
        graph.add_vertex();
        let covariance = matrix3::identity();
        assert!(graph.link(0, 1, Pose2::default(), Pose2::default(), &covariance, false));
        assert!(!graph.link(0, 1, Pose2::default(), Pose2::default(), &covariance, false));
        assert_eq!(graph.edges().len(), 1);
    }

    #[test]
    fn near_linked_stops_at_vertices_beyond_the_distance() {
        // A chain 0-1-2-3 along x at 1 m spacing, and 4 linked to 3 but
        // placed right next to 0: it's near 0, but only reachable through 3,
        // which is too far.
        let poses = [0.0, 1.0, 2.0, 3.0, 0.1].map(|x| Pose2::new(x, 0.0, 0.0));
        let mut graph = PoseGraph::default();
        let covariance = matrix3::identity();
        for _ in poses {
            graph.add_vertex();
        }
        for (from, to) in [(0, 1), (1, 2), (2, 3), (3, 4)] {
            graph.link(from, to, poses[from], poses[to], &covariance, false);
        }

        let mut near = graph.near_linked(0, 1.5, |v| poses[v]);
        near.sort();
        assert_eq!(near, vec![0, 1]);
    }

    #[test]
    fn equally_certain_means_average_to_their_midpoint() {
        let covariance = matrix3::diagonal(0.1, 0.1, 0.01);
        let mean = weighted_mean(
            &[Pose2::new(0.0, 0.0, 0.2), Pose2::new(1.0, 2.0, 0.4)],
            &[covariance, covariance],
        );
        assert!((mean.x_m - 0.5).abs() < 1e-9 && (mean.y_m - 1.0).abs() < 1e-9);
        assert!((mean.heading_rad - 0.3).abs() < 1e-9);
    }

    #[test]
    fn a_more_certain_mean_weighs_more() {
        let mean = weighted_mean(
            &[Pose2::new(0.0, 0.0, 0.0), Pose2::new(1.0, 0.0, 0.0)],
            &[
                matrix3::diagonal(0.01, 0.01, 0.01),
                matrix3::diagonal(1.0, 1.0, 0.01),
            ],
        );
        assert!(mean.x_m < 0.05, "{mean:?}");
    }
}
