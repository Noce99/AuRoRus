//! [`Mapper`]: turns a stream of odometry-stamped scans into a chain of
//! corrected poses and an occupancy map - Karto's `Mapper::Process`,
//! without the pose graph and loop closure (see `documentation/slam.md`).

use super::occupancy_grid::OccupancyGrid;
use super::pose::{Pose2, transform_pose, wrap_to_pi};
use super::scan::LocalizedScan;
use super::scan_matcher::{MatchParams, MatchResult, ScanMatcher};
use crate::topics::SlamMap;
use std::collections::VecDeque;

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
}

/// What [`Mapper::process`] did with a scan.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Processed {
    /// The vehicle hadn't moved enough since the last scan kept: dropped.
    Skipped,
    /// The first scan: kept as is, since there's nothing to match it to.
    First,
    /// Matched against the recent scans and kept, at the matched pose.
    Matched(MatchResult),
}

/// The mapping state: every scan kept so far, the recent ones the next
/// scan is matched against, and the occupancy grid built from them.
pub struct Mapper {
    params: MapperParams,
    /// Created on the first scan, once the range threshold - which depends
    /// on the sensor's maximum distance - is known.
    matcher: Option<(f64, ScanMatcher)>,
    /// Every scan kept, oldest first. Only the latest is needed without
    /// loop closure, but loop closure needs them all - see
    /// `documentation/slam.md`.
    scans: Vec<LocalizedScan>,
    /// Indices into `scans` of the ones the next scan is matched against -
    /// Karto's running scans.
    running: VecDeque<usize>,
    grid: OccupancyGrid,
}

impl Mapper {
    pub fn new(params: MapperParams) -> Self {
        Self {
            params,
            matcher: None,
            scans: Vec::new(),
            running: VecDeque::new(),
            grid: OccupancyGrid::new(
                params.resolution_m,
                params.min_pass_through,
                params.occupancy_threshold,
            ),
        }
    }

    /// Forgets everything - Karto's `Mapper::Reset`. The scan matcher is
    /// kept, since it only depends on the configuration and the sensor.
    pub fn reset(&mut self) {
        self.scans.clear();
        self.running.clear();
        self.grid.clear();
    }

    /// How many scans the map is built from.
    pub fn scan_count(&self) -> usize {
        self.scans.len()
    }

    /// The latest kept scan's corrected pose.
    pub fn pose(&self) -> Option<Pose2> {
        self.scans.last().map(LocalizedScan::corrected_pose)
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
    /// 4. add it to the running scans and the occupancy grid.
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

        let processed = if self.scans.is_empty() {
            Processed::First
        } else {
            let base: Vec<&LocalizedScan> = self
                .running
                .iter()
                .map(|&index| &self.scans[index])
                .collect();
            let matcher = Self::matcher(&mut self.matcher, &self.params, scan.range_threshold_m());
            let result = matcher.match_scan(&scan, &base);
            scan.set_corrected_pose(result.pose);
            Processed::Matched(result)
        };

        self.grid.add_scan(&scan);
        self.scans.push(scan);
        self.add_running_scan(self.scans.len() - 1);
        processed
    }

    /// The scan matcher for `range_threshold_m`, (re)built if there's none
    /// yet or it was built for another threshold.
    fn matcher<'a>(
        matcher: &'a mut Option<(f64, ScanMatcher)>,
        params: &MapperParams,
        range_threshold_m: f64,
    ) -> &'a mut ScanMatcher {
        if matcher
            .as_ref()
            .is_none_or(|(threshold, _)| *threshold != range_threshold_m)
        {
            *matcher = Some((
                range_threshold_m,
                ScanMatcher::new(
                    params.correlation_search_space_dimension_m,
                    params.correlation_search_space_resolution_m,
                    params.correlation_search_space_smear_deviation_m,
                    range_threshold_m,
                    params.matching,
                ),
            ));
        }
        &mut matcher.as_mut().expect("just set").1
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
    use std::time::{Duration, Instant};

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
            Processed::Matched(_)
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
