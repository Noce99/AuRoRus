//! [`Mapper`]: turns a stream of odometry-stamped scans into a pose graph
//! and an occupancy map - Karto's `Mapper::Process`, including its
//! `MapperGraph` (linking scans, closing loops, optimizing poses). See
//! `documentation/slam.md`.

use super::matrix3::Matrix3;
use super::occupancy_grid::OccupancyGrid;
use super::optimizer;
use super::pose::{Pose2, transform_pose, wrap_to_pi};
use super::pose_graph::{self, PoseGraph};
use super::scan::LocalizedScan;
use super::scan_matcher::{MatchParams, MatchResult, ScanMatcher};
use crate::topics::SlamMap;
use std::collections::{HashSet, VecDeque};
use std::time::Instant;

/// Karto's `KT_TOLERANCE`.
const TOLERANCE: f64 = 1e-6;

/// Everything [`Mapper`] needs to know - the subset of
/// [`crate::localization::SlamConfig`] that isn't about scheduling.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MapperParams {
    pub minimum_time_interval_s: f64,
    pub minimum_travel_distance_m: f64,
    pub minimum_travel_heading_rad: f64,
    pub scan_buffer_size: usize,
    pub scan_buffer_maximum_scan_distance_m: f64,
    pub correlation_search_space_dimension_m: f64,
    pub correlation_search_space_resolution_m: f64,
    pub correlation_search_space_smear_deviation_m: f64,
    pub max_laser_range_m: f64,
    pub resolution_m: f64,
    pub min_pass_through: u32,
    pub occupancy_threshold: f64,
    pub matching: MatchParams,
    pub do_loop_closing: bool,
    pub link_match_minimum_response_fine: f64,
    pub link_scan_maximum_distance_m: f64,
    pub loop_search_maximum_distance_m: f64,
    pub loop_match_minimum_chain_size: usize,
    pub loop_match_maximum_variance_coarse: f64,
    pub loop_match_minimum_response_coarse: f64,
    pub loop_match_minimum_response_fine: f64,
    pub loop_search_space_dimension_m: f64,
    pub loop_search_space_resolution_m: f64,
    pub loop_search_space_smear_deviation_m: f64,
    pub optimizer_max_iterations: usize,
}

/// What [`Mapper::process`] did with a scan.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Processed {
    /// The vehicle hadn't moved enough since the last scan kept: dropped.
    Skipped,
    /// The first scan: kept as is, since there's nothing to match it to.
    First,
    /// Matched against the recent scans and kept. `result` is that
    /// sequential match; the scan may have moved further since, when linked
    /// to other chains or when it closed a loop.
    Matched {
        result: MatchResult,
        loop_closed: bool,
    },
}

/// A scan matcher, built for one range threshold - see
/// [`Mapper::matcher`].
type LazyMatcher = Option<(f64, ScanMatcher)>;

/// The mapping state: every scan kept so far, the graph linking them, the
/// recent ones the next scan is matched against, and the occupancy grid
/// built from them.
pub struct Mapper {
    params: MapperParams,
    /// Matches each new scan against the recent ones - Karto's sequential
    /// scan matcher. Created on the first scan, once the range threshold
    /// (which depends on the sensor's maximum distance) is known.
    matcher: LazyMatcher,
    /// Matches a scan against an old chain it may close a loop with - a
    /// much wider, coarser search. Karto's loop scan matcher.
    loop_matcher: LazyMatcher,
    /// Every scan kept, oldest first - vertex `i` of `graph` is `scans[i]`.
    scans: Vec<LocalizedScan>,
    graph: PoseGraph,
    /// Indices into `scans` of the ones the next scan is matched against -
    /// Karto's running scans.
    running: VecDeque<usize>,
    grid: OccupancyGrid,
    loop_closures: usize,
    last_optimization_ms: Option<f64>,
}

impl Mapper {
    pub fn new(params: MapperParams) -> Self {
        Self {
            params,
            matcher: None,
            loop_matcher: None,
            scans: Vec::new(),
            graph: PoseGraph::default(),
            running: VecDeque::new(),
            grid: OccupancyGrid::new(
                params.resolution_m,
                params.min_pass_through,
                params.occupancy_threshold,
            ),
            loop_closures: 0,
            last_optimization_ms: None,
        }
    }

    /// Forgets everything - Karto's `Mapper::Reset`. The scan matchers are
    /// kept, since they only depend on the configuration and the sensor.
    pub fn reset(&mut self) {
        self.scans.clear();
        self.graph.clear();
        self.running.clear();
        self.grid.clear();
        self.loop_closures = 0;
        self.last_optimization_ms = None;
    }

    /// How many scans the map is built from.
    pub fn scan_count(&self) -> usize {
        self.scans.len()
    }

    /// How many loops have been closed.
    pub fn loop_closures(&self) -> usize {
        self.loop_closures
    }

    /// How long the latest pose graph optimization took, in milliseconds.
    pub fn last_optimization_ms(&self) -> Option<f64> {
        self.last_optimization_ms
    }

    /// The first kept scan's corrected pose - where mapping started. The
    /// optimizer holds it fixed, so it never moves.
    pub fn first_pose(&self) -> Option<Pose2> {
        self.scans.first().map(LocalizedScan::corrected_pose)
    }

    /// The latest kept scan's corrected pose.
    pub fn pose(&self) -> Option<Pose2> {
        self.scans.last().map(LocalizedScan::corrected_pose)
    }

    /// Where the `odom` frame sits in the map's, going by the latest kept
    /// scan: the correction that takes its odometric pose to its corrected
    /// one, so odometry's latest pose composed onto it is the vehicle's pose
    /// on the map, between two scans too.
    pub fn map_to_odom(&self) -> Option<Pose2> {
        self.scans
            .last()
            .map(|scan| scan.corrected_pose().compose(&scan.odometric_pose.inverse()))
    }

    /// Both ends of every edge that closed a loop, at their current poses.
    pub fn loop_edges(&self) -> Vec<(Pose2, Pose2)> {
        self.graph
            .edges()
            .iter()
            .filter(|edge| edge.loop_closure)
            .map(|edge| {
                (
                    self.scans[edge.from].corrected_pose(),
                    self.scans[edge.to].corrected_pose(),
                )
            })
            .collect()
    }

    /// The range threshold a scan from a sensor reaching up to
    /// `max_distance_m` must be built with.
    pub fn range_threshold_m(&self, max_distance_m: f64) -> f64 {
        self.params.max_laser_range_m.min(max_distance_m)
    }

    /// The map built so far, plus the trajectory it was built along.
    pub fn map(&self) -> SlamMap {
        let trajectory = self
            .scans
            .iter()
            .map(|scan| {
                let pose = scan.corrected_pose();
                [pose.x_m as f32, pose.y_m as f32]
            })
            .collect();
        self.grid.to_map(trajectory)
    }

    /// Adds `scan` (its corrected pose still equal to its odometric one)
    /// to the map - Karto's `Mapper::Process`:
    ///
    /// 1. carry the last correction over: the new scan starts from where
    ///    odometry moved it relative to the last scan's *corrected* pose;
    /// 2. drop it unless the vehicle moved or turned enough (or enough time
    ///    passed) since the last scan kept;
    /// 3. match it against the running scans and move it to the best pose;
    /// 4. add it to the graph, linked to the previous scan, the closest
    ///    running scan, and any nearby chain already linked to it;
    /// 5. add it to the running scans;
    /// 6. try to close a loop with any older chain nearby - optimizing the
    ///    whole graph if it does;
    /// 7. add it to the occupancy grid, or rebuild the grid from every scan
    ///    if a loop was closed and every pose may have moved.
    pub fn process(&mut self, mut scan: LocalizedScan) -> Processed {
        if let Some(last) = self.scans.last() {
            scan.set_corrected_pose(transform_pose(
                &last.odometric_pose,
                &last.corrected_pose(),
                &scan.odometric_pose,
            ));
            if !self.has_moved_enough(&scan, last) {
                return Processed::Skipped;
            }
        }

        let range_threshold_m = scan.range_threshold_m();
        let result = (!self.scans.is_empty()).then(|| {
            let base = self.running_scans();
            let base: Vec<&LocalizedScan> = base.iter().map(|&i| &self.scans[i]).collect();
            let result = Self::matcher(&mut self.matcher, &self.params, range_threshold_m, false)
                .match_scan(&scan, &base, true, true);
            scan.set_corrected_pose(result.pose);
            result
        });

        let index = self.scans.len();
        self.scans.push(scan);
        self.graph.add_vertex();
        if let Some(result) = &result {
            self.add_edges(index, &result.covariance);
        }
        self.add_running_scan(index);

        let loop_closed =
            result.is_some() && self.params.do_loop_closing && self.try_close_loop(index);
        if loop_closed {
            self.grid.rebuild(&self.scans);
        } else {
            self.grid.add_scan(&self.scans[index]);
        }

        match result {
            None => Processed::First,
            Some(result) => Processed::Matched {
                result,
                loop_closed,
            },
        }
    }

    /// The running scans, oldest first.
    fn running_scans(&self) -> Vec<usize> {
        self.running.iter().copied().collect()
    }

    /// The scan matcher in `slot` for `range_threshold_m`, (re)built if
    /// there's none yet or it was built for another threshold. `looping`
    /// picks the loop-closure search window over the sequential one.
    fn matcher<'a>(
        slot: &'a mut LazyMatcher,
        params: &MapperParams,
        range_threshold_m: f64,
        looping: bool,
    ) -> &'a mut ScanMatcher {
        if slot
            .as_ref()
            .is_none_or(|(threshold, _)| *threshold != range_threshold_m)
        {
            let (dimension, resolution, smear) = if looping {
                (
                    params.loop_search_space_dimension_m,
                    params.loop_search_space_resolution_m,
                    params.loop_search_space_smear_deviation_m,
                )
            } else {
                (
                    params.correlation_search_space_dimension_m,
                    params.correlation_search_space_resolution_m,
                    params.correlation_search_space_smear_deviation_m,
                )
            };
            *slot = Some((
                range_threshold_m,
                ScanMatcher::new(
                    dimension,
                    resolution,
                    smear,
                    range_threshold_m,
                    params.matching,
                ),
            ));
        }
        &mut slot.as_mut().expect("just set").1
    }

    /// Links the new scan `index` into the graph - Karto's
    /// `MapperGraph::AddEdges`: to the previous scan and the closest
    /// running scan (both measured by the sequential match, `covariance`),
    /// and to any nearby chain of scans already linked to it. The scan then
    /// moves to the covariance-weighted mean of every match.
    fn add_edges(&mut self, index: usize, covariance: &Matrix3) {
        let pose = self.scans[index].corrected_pose();
        self.graph.link(
            index - 1,
            index,
            self.scans[index - 1].corrected_pose(),
            pose,
            covariance,
            false,
        );

        let mut means = vec![pose];
        let mut covariances = vec![*covariance];
        let running = self.running_scans();
        self.link_chain_to_scan(&running, index, pose, covariance, false);
        if self.params.do_loop_closing {
            self.link_near_chains(index, &mut means, &mut covariances);
        }

        if means.len() > 1 {
            let mean = pose_graph::weighted_mean(&means, &covariances);
            self.scans[index].set_corrected_pose(mean);
        }
    }

    /// Links `chain`'s scan closest to scan `index` to it, if it's within
    /// `link_scan_maximum_distance_m` - Karto's
    /// `MapperGraph::LinkChainToScan`.
    fn link_chain_to_scan(
        &mut self,
        chain: &[usize],
        index: usize,
        mean: Pose2,
        covariance: &Matrix3,
        loop_closure: bool,
    ) {
        let pose = self.scans[index].corrected_pose();
        let Some(&closest) = chain.iter().min_by(|&&a, &&b| {
            let distance = |i: usize| self.scans[i].corrected_pose().squared_distance(&pose);
            distance(a).total_cmp(&distance(b))
        }) else {
            return;
        };
        let closest_pose = self.scans[closest].corrected_pose();
        if closest_pose.squared_distance(&pose)
            < self.params.link_scan_maximum_distance_m.powi(2) + TOLERANCE
        {
            self.graph
                .link(closest, index, closest_pose, mean, covariance, loop_closure);
        }
    }

    /// Matches scan `index` against every nearby chain of scans already
    /// linked to it and long enough, linking it to those that match well -
    /// Karto's `MapperGraph::LinkNearChains`. Each match's mean and
    /// covariance are appended to `means` and `covariances`.
    fn link_near_chains(
        &mut self,
        index: usize,
        means: &mut Vec<Pose2>,
        covariances: &mut Vec<Matrix3>,
    ) {
        let range_threshold_m = self.scans[index].range_threshold_m();
        for chain in self.find_near_chains(index) {
            if chain.len() < self.params.loop_match_minimum_chain_size {
                continue;
            }
            let base: Vec<&LocalizedScan> = chain.iter().map(|&i| &self.scans[i]).collect();
            let result = Self::matcher(&mut self.matcher, &self.params, range_threshold_m, false)
                .match_scan(&self.scans[index], &base, false, true);
            if result.response > self.params.link_match_minimum_response_fine - TOLERANCE {
                means.push(result.pose);
                covariances.push(result.covariance);
                self.link_chain_to_scan(&chain, index, result.pose, &result.covariance, false);
            }
        }
    }

    /// Chains of consecutive scans within `link_scan_maximum_distance_m` of
    /// scan `index`, grown from every scan linked to it (through the graph)
    /// within that distance - Karto's `MapperGraph::FindNearChains`. A
    /// chain reaching `index` itself is the current chain, not a nearby
    /// one, and is left out.
    fn find_near_chains(&self, index: usize) -> Vec<Vec<usize>> {
        let center = self.scans[index].corrected_pose();
        let max_squared = self.params.link_scan_maximum_distance_m.powi(2) + TOLERANCE;
        let is_near =
            |i: usize| self.scans[i].corrected_pose().squared_distance(&center) < max_squared;

        let mut processed = HashSet::new();
        let mut chains = Vec::new();
        let near_linked =
            self.graph
                .near_linked(index, self.params.link_scan_maximum_distance_m, |i| {
                    self.scans[i].corrected_pose()
                });
        for near in near_linked {
            if near == index || !processed.insert(near) {
                continue;
            }
            let mut valid = true;
            let mut chain = VecDeque::from([near]);
            for candidate in (0..near).rev() {
                valid &= candidate != index;
                if !is_near(candidate) {
                    break;
                }
                chain.push_front(candidate);
                processed.insert(candidate);
            }
            for candidate in near + 1..self.scans.len() {
                valid &= candidate != index;
                if !is_near(candidate) {
                    break;
                }
                chain.push_back(candidate);
                processed.insert(candidate);
            }
            if valid {
                chains.push(chain.into());
            }
        }
        chains
    }

    /// Looks for older chains scan `index` closes a loop with, and closes
    /// every one that matches well enough: links the scan to it and
    /// optimizes the whole graph - Karto's `MapperGraph::TryCloseLoop`.
    /// Returns whether any loop was closed.
    fn try_close_loop(&mut self, index: usize) -> bool {
        let range_threshold_m = self.scans[index].range_threshold_m();
        let mut closed = false;
        let mut start = 0;
        loop {
            let chain = self.find_possible_loop_closure(index, &mut start);
            if chain.is_empty() {
                break;
            }
            let base: Vec<&LocalizedScan> = chain.iter().map(|&i| &self.scans[i]).collect();

            // A wide, coarse search first...
            let coarse = Self::matcher(
                &mut self.loop_matcher,
                &self.params,
                range_threshold_m,
                true,
            )
            .match_scan(&self.scans[index], &base, false, false);
            let max_variance = self.params.loop_match_maximum_variance_coarse;
            if coarse.response <= self.params.loop_match_minimum_response_coarse
                || coarse.covariance[0][0] >= max_variance
                || coarse.covariance[1][1] >= max_variance
            {
                continue;
            }

            // ... then a fine one around its result.
            let mut moved = self.scans[index].clone();
            moved.set_corrected_pose(coarse.pose);
            let fine = Self::matcher(&mut self.matcher, &self.params, range_threshold_m, false)
                .match_scan(&moved, &base, false, true);
            if fine.response < self.params.loop_match_minimum_response_fine {
                continue;
            }

            self.scans[index].set_corrected_pose(fine.pose);
            self.link_chain_to_scan(&chain, index, fine.pose, &fine.covariance, true);
            self.correct_poses();
            self.loop_closures += 1;
            closed = true;
        }
        closed
    }

    /// The next chain of consecutive scans, from scan `*start` on, within
    /// `loop_search_maximum_distance_m` of scan `index` but not linked to
    /// it (through the graph, within that distance) and long enough -
    /// Karto's `MapperGraph::FindPossibleLoopClosure`. `*start` is left
    /// where the search stopped, for the next call.
    fn find_possible_loop_closure(&self, index: usize, start: &mut usize) -> Vec<usize> {
        let center = self.scans[index].corrected_pose();
        let max_squared = self.params.loop_search_maximum_distance_m.powi(2) + TOLERANCE;
        let near_linked: HashSet<usize> = self
            .graph
            .near_linked(index, self.params.loop_search_maximum_distance_m, |i| {
                self.scans[i].corrected_pose()
            })
            .into_iter()
            .collect();

        let mut chain = Vec::new();
        while *start < self.scans.len() {
            let candidate = *start;
            if self.scans[candidate]
                .corrected_pose()
                .squared_distance(&center)
                < max_squared
            {
                // A linked scan can't be part of a loop - it's the same
                // stretch of the path.
                if near_linked.contains(&candidate) {
                    chain.clear();
                } else {
                    chain.push(candidate);
                }
            } else if chain.len() >= self.params.loop_match_minimum_chain_size {
                return chain;
            } else {
                chain.clear();
            }
            *start += 1;
        }
        chain
    }

    /// Optimizes the pose graph and moves every scan to its optimized pose
    /// - Karto's `MapperGraph::CorrectPoses`.
    fn correct_poses(&mut self) {
        let started = Instant::now();
        let poses: Vec<Pose2> = self
            .scans
            .iter()
            .map(LocalizedScan::corrected_pose)
            .collect();
        match optimizer::optimize(
            &poses,
            self.graph.edges(),
            self.params.optimizer_max_iterations,
        ) {
            Some(optimized) => {
                for (scan, pose) in self.scans.iter_mut().zip(optimized) {
                    scan.set_corrected_pose(pose);
                }
            }
            None => eprintln!("Slam: pose graph optimization failed, keeping the current poses"),
        }
        self.last_optimization_ms = Some(started.elapsed().as_secs_f64() * 1000.0);
    }

    /// Karto's `Mapper::HasMovedEnough`, on odometric poses.
    fn has_moved_enough(&self, scan: &LocalizedScan, last: &LocalizedScan) -> bool {
        let elapsed_s = scan.time.saturating_duration_since(last.time).as_secs_f64();
        if elapsed_s >= self.params.minimum_time_interval_s {
            return true;
        }
        let turned_rad =
            wrap_to_pi(scan.odometric_pose.heading_rad - last.odometric_pose.heading_rad);
        if turned_rad.abs() >= self.params.minimum_travel_heading_rad {
            return true;
        }
        scan.odometric_pose.squared_distance(&last.odometric_pose)
            >= self.params.minimum_travel_distance_m.powi(2)
    }

    /// Adds `scans[index]` to the running scans, then drops the oldest ones
    /// while there are too many or they span too long a distance - Karto's
    /// `ScanManager::AddRunningScan`.
    fn add_running_scan(&mut self, index: usize) {
        self.running.push_back(index);
        let max_squared_distance = self.params.scan_buffer_maximum_scan_distance_m.powi(2);
        while let (Some(&front), Some(&back)) = (self.running.front(), self.running.back()) {
            let span = self.scans[front]
                .corrected_pose()
                .squared_distance(&self.scans[back].corrected_pose());
            if self.running.len() > self.params.scan_buffer_size || span > max_squared_distance {
                self.running.pop_front();
            } else {
                break;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::localization::slam::scan_matcher::tests::{params as match_params, room_scan};
    use crate::topics::LidarScan;
    use std::f64::consts::{FRAC_PI_2, PI};
    use std::time::Duration;

    fn params() -> MapperParams {
        MapperParams {
            minimum_time_interval_s: 0.5,
            minimum_travel_distance_m: 0.2,
            minimum_travel_heading_rad: 0.2,
            scan_buffer_size: 10,
            scan_buffer_maximum_scan_distance_m: 10.0,
            correlation_search_space_dimension_m: 0.5,
            correlation_search_space_resolution_m: 0.01,
            correlation_search_space_smear_deviation_m: 0.03,
            max_laser_range_m: 12.0,
            resolution_m: 0.05,
            min_pass_through: 2,
            occupancy_threshold: 0.1,
            matching: match_params(),
            do_loop_closing: true,
            link_match_minimum_response_fine: 0.1,
            link_scan_maximum_distance_m: 1.5,
            loop_search_maximum_distance_m: 3.0,
            loop_match_minimum_chain_size: 10,
            loop_match_maximum_variance_coarse: 3.0,
            loop_match_minimum_response_coarse: 0.35,
            loop_match_minimum_response_fine: 0.45,
            loop_search_space_dimension_m: 8.0,
            loop_search_space_resolution_m: 0.05,
            loop_search_space_smear_deviation_m: 0.03,
            optimizer_max_iterations: 20,
        }
    }

    fn scan_at(truth: Pose2, odometric: Pose2, time: Instant) -> LocalizedScan {
        LocalizedScan::new(&room_scan(truth, 360), time, odometric, 12.0)
    }

    #[test]
    fn scans_that_barely_moved_are_skipped() {
        let t0 = Instant::now();
        let mut mapper = Mapper::new(params());
        let first = Pose2::default();
        assert_eq!(mapper.process(scan_at(first, first, t0)), Processed::First);

        let near = Pose2::new(0.05, 0.0, 0.0);
        let soon = t0 + Duration::from_millis(100);
        assert_eq!(
            mapper.process(scan_at(near, near, soon)),
            Processed::Skipped
        );

        // Same place, but enough time has passed.
        let later = t0 + Duration::from_millis(600);
        assert!(matches!(
            mapper.process(scan_at(near, near, later)),
            Processed::Matched { .. }
        ));
        assert_eq!(mapper.scan_count(), 2);
    }

    #[test]
    fn matching_corrects_drifting_odometry() {
        // The vehicle drives along +x while odometry overestimates every
        // step by 10% and drifts its heading by 0.02 rad per step.
        let t0 = Instant::now();
        let mut mapper = Mapper::new(params());
        let mut odometric = Pose2::default();
        let mut worst_odometry_error: f64 = 0.0;
        for step in 0u32..8 {
            let truth = Pose2::new(0.25 * f64::from(step), 0.0, 0.0);
            let time = t0 + Duration::from_millis(100 * u64::from(step));
            mapper.process(scan_at(truth, odometric, time));
            worst_odometry_error =
                worst_odometry_error.max(odometric.squared_distance(&truth).sqrt());

            let corrected = mapper.pose().expect("a scan was kept");
            assert!(
                corrected.squared_distance(&truth).sqrt() < 0.03,
                "step {step}: corrected {corrected:?}, truth {truth:?}"
            );
            // The correction takes odometry's pose to the corrected one.
            let on_map = mapper.map_to_odom().expect("a scan was kept").compose(&odometric);
            assert!(on_map.squared_distance(&corrected).sqrt() < 1e-9, "step {step}");
            assert!((on_map.heading_rad - corrected.heading_rad).abs() < 1e-9, "step {step}");
            odometric = odometric.compose(&Pose2::new(0.275, 0.0, 0.02));
        }
        assert!(worst_odometry_error > 0.1, "odometry must actually drift");
    }

    #[test]
    fn the_running_buffer_is_capped() {
        let t0 = Instant::now();
        let mut mapper = Mapper::new(MapperParams {
            scan_buffer_size: 3,
            ..params()
        });
        for step in 0..6 {
            let pose = Pose2::new(0.0, 0.0, 0.0);
            mapper.process(scan_at(pose, pose, t0 + Duration::from_secs(step)));
        }
        assert_eq!(mapper.scan_count(), 6);
        assert_eq!(mapper.running.len(), 3);
    }

    /// Distance from `(x, y)` along `angle` to the first surface of a
    /// ring-shaped corridor: outer walls at x = ±6, y = ±4, an inner block
    /// at x = ±4, y = ±2, and pillars everywhere but along the top
    /// corridor. That one is featureless: seen with a short-range lidar,
    /// every stretch of it looks the same, so matching can't tell how far
    /// along it the vehicle is, and the map comes out with it too short -
    /// an error only closing the loop can reveal. The corridor's center line
    /// is the rectangle x = ±5, y = ±3, 32 m around.
    fn ray_to_ring(x: f64, y: f64, angle: f64) -> f64 {
        let (dx, dy) = (angle.cos(), angle.sin());
        let mut best = f64::INFINITY;
        let mut hit = |t: f64| {
            if t > 1e-9 {
                best = best.min(t);
            }
        };
        // Axis-aligned segments: (fixed coordinate, along x?, from, to).
        let segments = [
            (-6.0, true, -4.0, 4.0),
            (6.0, true, -4.0, 4.0),
            (-4.0, false, -6.0, 6.0),
            (4.0, false, -6.0, 6.0),
            (-4.0, true, -2.0, 2.0),
            (4.0, true, -2.0, 2.0),
            (-2.0, false, -4.0, 4.0),
            (2.0, false, -4.0, 4.0),
        ];
        for (fixed, is_x, from, to) in segments {
            let (origin, direction, other_origin, other_direction) =
                if is_x { (x, dx, y, dy) } else { (y, dy, x, dx) };
            let t = (fixed - origin) / direction;
            let along = other_origin + t * other_direction;
            if (from..=to).contains(&along) {
                hit(t);
            }
        }
        for (cx, cy) in [
            (5.7, 1.0),
            (4.3, -1.5),
            (-5.7, -0.5),
            (-4.3, 1.8),
            (-2.5, -3.7),
            (0.5, -2.3),
            (2.5, -3.7),
        ] {
            let (ox, oy, r) = (cx - x, cy - y, 0.3);
            let b = ox * dx + oy * dy;
            let disc = b * b - (ox * ox + oy * oy - r * r);
            if disc >= 0.0 {
                hit(b - disc.sqrt());
            }
        }
        best
    }

    /// A 360-reading scan from `pose` inside the ring corridor.
    fn ring_scan(pose: Pose2) -> LidarScan {
        let n = 360;
        let fov = 2.0 * std::f32::consts::PI * (n - 1) as f32 / n as f32;
        let points = (0..n)
            .map(|i| {
                let angle = pose.heading_rad + f64::from(LidarScan::ray_angle_rad(fov, n, i));
                ray_to_ring(pose.x_m, pose.y_m, angle).min(29.0) as f32
            })
            .collect();
        LidarScan::new(points, vec![1.0; n], 0.1, 30.0, fov)
    }

    /// The pose `distance_m` along the corridor's center line,
    /// counterclockwise from (5, -3) facing +y.
    fn along_ring(distance_m: f64) -> Pose2 {
        let d = distance_m.rem_euclid(32.0);
        let (x, y, heading) = if d < 6.0 {
            (5.0, -3.0 + d, FRAC_PI_2)
        } else if d < 16.0 {
            (5.0 - (d - 6.0), 3.0, PI)
        } else if d < 22.0 {
            (-5.0, 3.0 - (d - 16.0), -FRAC_PI_2)
        } else {
            (-5.0 + (d - 22.0), -3.0, 0.0)
        };
        Pose2::new(x, y, heading)
    }

    /// Drives 1.25 laps of the ring, with a 3 m lidar and odometry that
    /// over-reads distance by 3% and drifts 0.004 rad left per step, and
    /// returns the mapper plus the worst error, once done, of any scan's
    /// corrected pose.
    fn drive_ring(do_loop_closing: bool) -> (Mapper, f64) {
        let t0 = Instant::now();
        let mut mapper = Mapper::new(MapperParams {
            do_loop_closing,
            max_laser_range_m: 3.0,
            ..params()
        });
        let step_m = 0.4;
        let steps = (1.25 * 32.0 / step_m) as u32;
        let mut odometric = Pose2::default();
        let mut truths = Vec::new();
        for step in 0..steps {
            let truth = along_ring(f64::from(step) * step_m);
            if let Some(previous) = truths.last() {
                // The true motion since the previous step, corrupted.
                let motion = along_ring(0.0).compose(previous).inverse().compose(&truth);
                odometric = odometric.compose(&Pose2::new(
                    motion.x_m * 1.03,
                    motion.y_m * 1.03,
                    motion.heading_rad + 0.004,
                ));
            }
            // SLAM's frame is the one of the first scan: truth in it.
            truths.push(along_ring(0.0).inverse().compose(&truth));
            let time = t0 + Duration::from_millis(100 * u64::from(step));
            mapper.process(LocalizedScan::new(&ring_scan(truth), time, odometric, 3.0));
        }

        assert_eq!(mapper.scan_count(), truths.len(), "every step moves enough");
        let worst_error = mapper
            .scans
            .iter()
            .zip(&truths)
            .map(|(scan, truth)| scan.corrected_pose().squared_distance(truth).sqrt())
            .fold(0.0, f64::max);
        (mapper, worst_error)
    }

    #[test]
    fn closing_the_loop_straightens_the_whole_trajectory() {
        let (without, error_without) = drive_ring(false);
        let (with, error_with) = drive_ring(true);

        assert_eq!(without.loop_closures(), 0);
        assert!(with.loop_closures() >= 1, "no loop closed");
        assert!(!with.loop_edges().is_empty());
        assert!(
            error_with < 0.5 * error_without,
            "with loop closure {error_with} m, without {error_without} m"
        );
    }

    #[test]
    fn reset_forgets_everything() {
        let t0 = Instant::now();
        let mut mapper = Mapper::new(params());
        mapper.process(scan_at(Pose2::default(), Pose2::default(), t0));
        assert!(mapper.map().width_px > 0);

        mapper.reset();
        assert_eq!(mapper.scan_count(), 0);
        assert_eq!(mapper.pose(), None);
        assert_eq!(mapper.map().width_px, 0);
    }
}
