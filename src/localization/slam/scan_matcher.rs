//! [`ScanMatcher`]: finds the pose at which a scan best overlaps a set of
//! reference scans - Karto's correlative `ScanMatcher` (`Mapper.cpp`). It
//! brute-forces every candidate pose in a window around the prior: a
//! coarse pass over the whole window, then a fine pass around the coarse
//! winner.

use super::correlation_grid::{CorrelationGrid, OCCUPIED};
use super::matrix3::{Matrix3, diagonal, identity};
use super::pose::{Pose2, wrap_to_pi};
use super::scan::LocalizedScan;

/// Variance reported along an axis the match couldn't pin down at all -
/// Karto's `MAX_VARIANCE`.
pub const MAX_VARIANCE: f64 = 500.0;
/// How much a candidate's distance from the prior costs it - Karto's
/// `DISTANCE_PENALTY_GAIN`.
const DISTANCE_PENALTY_GAIN: f64 = 0.2;
/// How much a candidate's heading change from the prior costs it - Karto's
/// `ANGLE_PENALTY_GAIN`.
const ANGLE_PENALTY_GAIN: f64 = 0.2;
/// Two responses closer than this are the same - Karto's `KT_TOLERANCE`.
const TOLERANCE: f64 = 1e-6;
/// Minimum distance between two points for [`valid_points`] to compare
/// which side of the viewpoint they're on, in meters.
const MIN_VALID_POINT_SPACING_M: f64 = 0.1;

/// A 3x3 covariance of `(x_m, y_m, heading_rad)`, row-major.
pub type Covariance = Matrix3;

/// The search's tunable knobs - see `config/localization/slam.toml`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MatchParams {
    pub coarse_search_angle_offset_rad: f64,
    pub coarse_angle_resolution_rad: f64,
    pub fine_search_angle_offset_rad: f64,
    pub distance_variance_penalty: f64,
    pub angle_variance_penalty: f64,
    pub minimum_distance_penalty: f64,
    pub minimum_angle_penalty: f64,
    pub use_response_expansion: bool,
}

/// The outcome of [`ScanMatcher::match_scan`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MatchResult {
    /// The best pose found.
    pub pose: Pose2,
    /// How well the scan overlaps the reference scans there, in `[0, 1]`.
    pub response: f64,
    /// How uncertain `pose` is.
    pub covariance: Covariance,
}

/// A correlative scan matcher searching a square window of
/// `search_size_m` around the prior.
pub struct ScanMatcher {
    grid: CorrelationGrid,
    /// Cells per side of the search window.
    search_side: i32,
    /// The best response found at each cell of the search window during
    /// the coarse pass, for [`Self::positional_covariance`] - Karto's
    /// `m_pSearchSpaceProbs`.
    search_probs: Vec<f64>,
    /// World position of `search_probs`'s cell `(0, 0)`.
    search_probs_offset: (f64, f64),
    /// For every candidate heading, the offset of every point of the scan
    /// being matched into the grid's data, relative to the candidate
    /// position's cell - Karto's `GridIndexLookup`.
    lookup: Vec<Vec<isize>>,
    /// Every candidate of the current pass and its response.
    candidates: Vec<(f64, Pose2)>,
    /// Whether the current match penalizes candidates far from the prior.
    penalize: bool,
    params: MatchParams,
}

impl ScanMatcher {
    /// A matcher searching `search_size_m` (square, centered on the prior)
    /// in steps of `resolution_m`, against reference points up to
    /// `range_threshold_m` away. Karto's `ScanMatcher::Create`.
    pub fn new(
        search_size_m: f64,
        resolution_m: f64,
        smear_deviation_m: f64,
        range_threshold_m: f64,
        params: MatchParams,
    ) -> Self {
        let search_side = (search_size_m / resolution_m).round() as i32 + 1;
        // Pad the grid so reference points can't fall off it even when the
        // candidate sits on the search window's edge.
        let margin = (range_threshold_m / resolution_m).ceil() as i32;
        let grid = CorrelationGrid::new(search_side + 2 * margin, resolution_m, smear_deviation_m);
        Self {
            grid,
            search_side,
            search_probs: vec![0.0; (search_side * search_side) as usize],
            search_probs_offset: (0.0, 0.0),
            lookup: Vec::new(),
            candidates: Vec::new(),
            penalize: true,
            params,
        }
    }

    /// Where `scan` - starting from its corrected pose - best overlaps
    /// `base`. Karto's `ScanMatcher::MatchScan`: `penalize` makes candidates
    /// far from the prior score lower (trusting odometry), `refine` adds the
    /// fine pass after the coarse one.
    pub fn match_scan(
        &mut self,
        scan: &LocalizedScan,
        base: &[&LocalizedScan],
        penalize: bool,
        refine: bool,
    ) -> MatchResult {
        self.match_with(scan, penalize, refine, |matcher, prior| {
            matcher.add_scans(base, (prior.x_m, prior.y_m));
        })
    }

    /// Where `scan` - starting from its corrected pose - best overlaps the
    /// fixed reference `points`, e.g. the walls of a known map, as
    /// [`Self::match_scan`] does for reference scans. Every point is used:
    /// unlike a reference scan's, a map's points weren't seen from anywhere
    /// in particular, so none can be dropped for facing away.
    pub fn match_points(
        &mut self,
        scan: &LocalizedScan,
        points: &[(f64, f64)],
        penalize: bool,
        refine: bool,
    ) -> MatchResult {
        self.match_with(scan, penalize, refine, |matcher, _| {
            matcher.grid.clear();
            for &(x_m, y_m) in points {
                matcher.grid.add_point(x_m, y_m);
            }
        })
    }

    /// [`Self::match_scan`]'s search, with the grid - already centered on
    /// the prior - filled by `fill_grid(self, prior)`.
    fn match_with(
        &mut self,
        scan: &LocalizedScan,
        penalize: bool,
        refine: bool,
        fill_grid: impl FnOnce(&mut Self, Pose2),
    ) -> MatchResult {
        let prior = scan.corrected_pose();
        self.penalize = penalize;
        if scan.world_points().is_empty() {
            return MatchResult {
                pose: prior,
                response: 0.0,
                covariance: diagonal(
                    MAX_VARIANCE,
                    MAX_VARIANCE,
                    4.0 * self.params.coarse_angle_resolution_rad.powi(2),
                ),
            };
        }

        // Center the grid on the prior.
        let resolution_m = self.grid.resolution_m();
        let half_roi_m = 0.5 * f64::from(self.grid.roi_side() - 1) * resolution_m;
        self.grid
            .set_offset(prior.x_m - half_roi_m, prior.y_m - half_roi_m);
        fill_grid(self, prior);

        let search_offset_m = 0.5 * f64::from(self.search_side - 1) * resolution_m;
        // The coarse pass only checks every other cell.
        let coarse_resolution_m = 2.0 * resolution_m;
        let mut covariance = identity();
        let mut angle_offset_rad = self.params.coarse_search_angle_offset_rad;
        let (mut response, mut pose) = self.correlate(
            scan,
            prior,
            search_offset_m,
            coarse_resolution_m,
            angle_offset_rad,
            self.params.coarse_angle_resolution_rad,
            false,
            &mut covariance,
        );

        if self.params.use_response_expansion && response.abs() < TOLERANCE {
            // Nothing overlapped at all: widen the heading search by 20
            // degrees at a time, up to three times.
            for _ in 0..3 {
                angle_offset_rad += 20f64.to_radians();
                (response, pose) = self.correlate(
                    scan,
                    prior,
                    search_offset_m,
                    coarse_resolution_m,
                    angle_offset_rad,
                    self.params.coarse_angle_resolution_rad,
                    false,
                    &mut covariance,
                );
                if response.abs() >= TOLERANCE {
                    break;
                }
            }
        }

        if !refine {
            return MatchResult {
                pose,
                response,
                covariance,
            };
        }

        // Karto passes `fine_search_angle_offset` as the fine pass's angular
        // *resolution*, over half a coarse step either side - kept as is.
        (response, pose) = self.correlate(
            scan,
            pose,
            0.5 * coarse_resolution_m,
            resolution_m,
            0.5 * self.params.coarse_angle_resolution_rad,
            self.params.fine_search_angle_offset_rad,
            true,
            &mut covariance,
        );

        MatchResult {
            pose,
            response,
            covariance,
        }
    }

    /// Marks every valid point of `scans` in the grid - Karto's
    /// `ScanMatcher::AddScans`.
    fn add_scans(&mut self, scans: &[&LocalizedScan], viewpoint: (f64, f64)) {
        self.grid.clear();
        for scan in scans {
            for (x_m, y_m) in valid_points(scan.world_points(), viewpoint) {
                self.grid.add_point(x_m, y_m);
            }
        }
    }

    /// Scores every candidate pose around `center` - positions `±offset_m`
    /// in steps of `resolution_m`, headings `±angle_offset_rad` in steps of
    /// `angle_resolution_rad` - and returns the best response and the
    /// average of every candidate reaching it. Fills in the positional
    /// covariance on the coarse pass and the angular one on the fine pass.
    /// Karto's `ScanMatcher::CorrelateScan`.
    #[allow(clippy::too_many_arguments)]
    fn correlate(
        &mut self,
        scan: &LocalizedScan,
        center: Pose2,
        offset_m: f64,
        resolution_m: f64,
        angle_offset_rad: f64,
        angle_resolution_rad: f64,
        fine: bool,
        covariance: &mut Covariance,
    ) -> (f64, Pose2) {
        let n_angles = (angle_offset_rad * 2.0 / angle_resolution_rad).round() as usize + 1;
        let start_angle_rad = center.heading_rad - angle_offset_rad;
        self.compute_lookup(scan, start_angle_rad, angle_resolution_rad, n_angles);

        if !fine {
            self.search_probs.fill(0.0);
            self.search_probs_offset = (center.x_m - offset_m, center.y_m - offset_m);
        }

        let n_steps = (offset_m * 2.0 / resolution_m).round() as usize + 1;
        let steps: Vec<f64> = (0..n_steps)
            .map(|i| -offset_m + i as f64 * resolution_m)
            .collect();

        self.candidates.clear();
        for &dy in &steps {
            for &dx in &steps {
                let x_m = center.x_m + dx;
                let y_m = center.y_m + dy;
                let (gx, gy) = self.grid.world_to_grid(x_m, y_m);
                let grid_index = self.grid.index(gx, gy);
                let squared_distance = dx * dx + dy * dy;
                for angle_index in 0..n_angles {
                    let angle_rad = start_angle_rad + angle_index as f64 * angle_resolution_rad;
                    let mut response = self.response(angle_index, grid_index);
                    if self.penalize && response != 0.0 {
                        // An approximate Gaussian around the prior, so
                        // odometry still counts for something.
                        let distance_penalty = (1.0
                            - DISTANCE_PENALTY_GAIN * squared_distance
                                / self.params.distance_variance_penalty)
                            .max(self.params.minimum_distance_penalty);
                        let squared_angle = (angle_rad - center.heading_rad).powi(2);
                        let angle_penalty = (1.0
                            - ANGLE_PENALTY_GAIN * squared_angle
                                / self.params.angle_variance_penalty)
                            .max(self.params.minimum_angle_penalty);
                        response *= distance_penalty * angle_penalty;
                    }
                    self.candidates
                        .push((response, Pose2::new(x_m, y_m, angle_rad)));
                }
            }
        }

        let best_response = self
            .candidates
            .iter()
            .fold(-1.0f64, |best, &(response, _)| best.max(response));

        if !fine {
            for &(response, pose) in &self.candidates {
                let index = self.search_probs_index(pose.x_m, pose.y_m);
                let cell = &mut self.search_probs[index];
                *cell = cell.max(response);
            }
        }

        // Average every candidate that reached the best response.
        let (mut sum_x, mut sum_y, mut sum_cos, mut sum_sin, mut count) =
            (0.0, 0.0, 0.0, 0.0, 0usize);
        for &(response, pose) in &self.candidates {
            if (response - best_response).abs() < TOLERANCE {
                sum_x += pose.x_m;
                sum_y += pose.y_m;
                sum_cos += pose.heading_rad.cos();
                sum_sin += pose.heading_rad.sin();
                count += 1;
            }
        }
        let n = count as f64;
        let average = Pose2::new(sum_x / n, sum_y / n, (sum_sin / n).atan2(sum_cos / n));

        if fine {
            covariance[2][2] = self.angular_variance(
                average,
                best_response,
                center,
                angle_offset_rad,
                angle_resolution_rad,
            );
        } else {
            *covariance = self.positional_covariance(
                average,
                best_response,
                center,
                offset_m,
                resolution_m,
                angle_resolution_rad,
            );
        }

        (best_response.min(1.0), average)
    }

    /// Fills [`Self::lookup`] for `n_angles` headings from
    /// `start_angle_rad` - Karto's `GridIndexLookup::ComputeOffsets`.
    fn compute_lookup(
        &mut self,
        scan: &LocalizedScan,
        start_angle_rad: f64,
        angle_resolution_rad: f64,
        n_angles: usize,
    ) {
        self.lookup.resize_with(n_angles, Vec::new);
        for (angle_index, offsets) in self.lookup.iter_mut().enumerate() {
            let (sin, cos) =
                (start_angle_rad + angle_index as f64 * angle_resolution_rad).sin_cos();
            offsets.clear();
            offsets.extend(scan.matchable_readings().map(|reading| {
                self.grid.index_offset(
                    cos * reading.x_m - sin * reading.y_m,
                    sin * reading.x_m + cos * reading.y_m,
                )
            }));
        }
    }

    /// How much of the scan, rotated to heading `angle_index` and placed at
    /// grid cell `grid_index`, lands on reference points, in `[0, 1]` -
    /// Karto's `ScanMatcher::GetResponse`.
    fn response(&self, angle_index: usize, grid_index: usize) -> f64 {
        let offsets = &self.lookup[angle_index];
        if offsets.is_empty() {
            return 0.0;
        }
        let data = self.grid.data();
        let sum: u64 = offsets
            .iter()
            .filter_map(|&offset| data.get(grid_index.checked_add_signed(offset)?))
            .map(|&cell| u64::from(cell))
            .sum();
        sum as f64 / (offsets.len() as f64 * f64::from(OCCUPIED))
    }

    fn search_probs_index(&self, x_m: f64, y_m: f64) -> usize {
        let resolution_m = self.grid.resolution_m();
        let max = self.search_side - 1;
        let gx = (((x_m - self.search_probs_offset.0) / resolution_m).round() as i32).clamp(0, max);
        let gy = (((y_m - self.search_probs_offset.1) / resolution_m).round() as i32).clamp(0, max);
        (gy * self.search_side + gx) as usize
    }

    /// Karto's `ScanMatcher::ComputePositionalCovariance`: the spread of
    /// the search window's near-best responses around `best`.
    fn positional_covariance(
        &self,
        best: Pose2,
        best_response: f64,
        center: Pose2,
        offset_m: f64,
        resolution_m: f64,
        angle_resolution_rad: f64,
    ) -> Covariance {
        let mut covariance = identity();
        if best_response < TOLERANCE {
            covariance[0][0] = MAX_VARIANCE;
            covariance[1][1] = MAX_VARIANCE;
            covariance[2][2] = 4.0 * angle_resolution_rad.powi(2);
            return covariance;
        }

        let dx = best.x_m - center.x_m;
        let dy = best.y_m - center.y_m;
        let n_steps = (offset_m * 2.0 / resolution_m).round() as usize + 1;
        let (mut xx, mut xy, mut yy, mut norm) = (0.0, 0.0, 0.0, 0.0);
        for yi in 0..n_steps {
            let y = -offset_m + yi as f64 * resolution_m;
            for xi in 0..n_steps {
                let x = -offset_m + xi as f64 * resolution_m;
                let response =
                    self.search_probs[self.search_probs_index(center.x_m + x, center.y_m + y)];
                if response >= best_response - 0.1 {
                    norm += response;
                    xx += (x - dx).powi(2) * response;
                    xy += (x - dx) * (y - dy) * response;
                    yy += (y - dy).powi(2) * response;
                }
            }
        }

        if norm > TOLERANCE {
            // Lower-bounded so links are never too tight, and inflated for
            // poorer responses.
            let multiplier = 1.0 / best_response;
            covariance[0][0] = (xx / norm).max(0.1 * resolution_m.powi(2)) * multiplier;
            covariance[0][1] = xy / norm * multiplier;
            covariance[1][0] = covariance[0][1];
            covariance[1][1] = (yy / norm).max(0.1 * resolution_m.powi(2)) * multiplier;
            covariance[2][2] = 4.0 * angle_resolution_rad.powi(2);
        }
        // Too sparse to ever overlap: nothing is known along that axis.
        if covariance[0][0].abs() < TOLERANCE {
            covariance[0][0] = MAX_VARIANCE;
        }
        if covariance[1][1].abs() < TOLERANCE {
            covariance[1][1] = MAX_VARIANCE;
        }
        covariance
    }

    /// Karto's `ScanMatcher::ComputeAngularCovariance`: the spread of the
    /// near-best responses across headings, at `best`'s position.
    fn angular_variance(
        &self,
        best: Pose2,
        best_response: f64,
        center: Pose2,
        angle_offset_rad: f64,
        angle_resolution_rad: f64,
    ) -> f64 {
        // `best`'s heading, unwrapped to within pi of `center`'s - the
        // candidates' headings aren't wrapped either.
        let best_angle_rad = center.heading_rad + wrap_to_pi(best.heading_rad - center.heading_rad);
        let (gx, gy) = self.grid.world_to_grid(best.x_m, best.y_m);
        let grid_index = self.grid.index(gx, gy);
        let n_angles = (angle_offset_rad * 2.0 / angle_resolution_rad).round() as usize + 1;
        let start_angle_rad = center.heading_rad - angle_offset_rad;

        let (mut norm, mut accumulated) = (0.0, 0.0);
        for angle_index in 0..n_angles {
            let angle_rad = start_angle_rad + angle_index as f64 * angle_resolution_rad;
            let response = self.response(angle_index, grid_index);
            if response >= best_response - 0.1 {
                norm += response;
                accumulated += (angle_rad - best_angle_rad).powi(2) * response;
            }
        }

        if norm > TOLERANCE {
            if accumulated < TOLERANCE {
                accumulated = angle_resolution_rad.powi(2);
            }
            accumulated / norm
        } else {
            1000.0 * angle_resolution_rad.powi(2)
        }
    }
}

/// The points of `points` (in scan order) facing `viewpoint` - a
/// reference scan's points on surfaces seen from the other side can't
/// line up with what the scan being matched sees. Karto's
/// `ScanMatcher::FindValidPoints`.
fn valid_points(points: &[(f64, f64)], viewpoint: (f64, f64)) -> Vec<(f64, f64)> {
    let min_squared_distance = MIN_VALID_POINT_SPACING_M.powi(2);
    let mut valid = Vec::with_capacity(points.len());
    let Some(&first) = points.first() else {
        return valid;
    };
    let mut first_point = first;
    // Lags behind, only catching up (and adding the points in between)
    // when they're on the right side.
    let mut trailing = 0;
    for (i, &current) in points.iter().enumerate() {
        let delta = (first_point.0 - current.0, first_point.1 - current.1);
        if delta.0 * delta.0 + delta.1 * delta.1 <= min_squared_distance {
            continue;
        }
        // Which way (viewpoint -> first_point -> current) turns: clockwise
        // means that stretch of surface faces away from the viewpoint.
        let a = viewpoint.1 - first_point.1;
        let b = first_point.0 - viewpoint.0;
        let c = first_point.1 * viewpoint.0 - first_point.0 * viewpoint.1;
        let side = current.0 * a + current.1 * b + c;
        first_point = current;
        if side < 0.0 {
            trailing = i;
        } else {
            valid.extend_from_slice(&points[trailing..i]);
            trailing = i;
        }
    }
    valid
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;
    use crate::topics::LidarScan;
    use std::time::Instant;

    pub fn params() -> MatchParams {
        MatchParams {
            coarse_search_angle_offset_rad: 0.349,
            coarse_angle_resolution_rad: 0.0349,
            fine_search_angle_offset_rad: 0.00349,
            distance_variance_penalty: 0.5,
            angle_variance_penalty: 1.0,
            minimum_distance_penalty: 0.5,
            minimum_angle_penalty: 0.9,
            use_response_expansion: true,
        }
    }

    /// Ranges a 360-degree lidar at `pose` would read inside an irregular
    /// room: walls at x = -3 and x = 4, y = -2 and y = 5, plus a pillar -
    /// asymmetric enough for a match to be unambiguous.
    pub fn room_scan(pose: Pose2, n: usize) -> LidarScan {
        let fov = 2.0 * std::f32::consts::PI * (n - 1) as f32 / n as f32;
        let points = (0..n)
            .map(|i| {
                let angle = pose.heading_rad + f64::from(LidarScan::ray_angle_rad(fov, n, i));
                ray_to_room(pose.x_m, pose.y_m, angle) as f32
            })
            .collect::<Vec<_>>();
        LidarScan::new(points, vec![1.0; n], 0.1, 30.0, fov)
    }

    /// Distance from `(x, y)` along `angle` to the room's first surface.
    pub fn ray_to_room(x: f64, y: f64, angle: f64) -> f64 {
        let (dx, dy) = (angle.cos(), angle.sin());
        let mut best = f64::INFINITY;
        for (wall, along_x) in [(-3.0, true), (4.0, true), (-2.0, false), (5.0, false)] {
            let t = if along_x {
                (wall - x) / dx
            } else {
                (wall - y) / dy
            };
            if t > 0.0 {
                best = best.min(t);
            }
        }
        // A 0.5 m radius pillar centered at (1.5, 2).
        let (cx, cy, r) = (1.5 - x, 2.0 - y, 0.5);
        let b = cx * dx + cy * dy;
        let disc = b * b - (cx * cx + cy * cy - r * r);
        if disc >= 0.0 {
            let t = b - disc.sqrt();
            if t > 0.0 {
                best = best.min(t);
            }
        }
        best
    }

    #[test]
    fn matching_recovers_a_wrong_prior() {
        let truth = Pose2::new(0.3, 0.2, 0.1);
        let reference = LocalizedScan::new(
            &room_scan(Pose2::default(), 360),
            Instant::now(),
            Pose2::default(),
            12.0,
        );
        // Odometry is off by (0.1, -0.05, 0.05).
        let prior = Pose2::new(truth.x_m + 0.1, truth.y_m - 0.05, truth.heading_rad + 0.05);
        let scan = LocalizedScan::new(&room_scan(truth, 360), Instant::now(), prior, 12.0);

        let mut matcher = ScanMatcher::new(0.5, 0.01, 0.03, 12.0, params());
        let result = matcher.match_scan(&scan, &[&reference], true, true);

        assert!(
            result.pose.squared_distance(&truth).sqrt() < 0.02,
            "matched {:?}, truth {truth:?}",
            result.pose
        );
        assert!(wrap_to_pi(result.pose.heading_rad - truth.heading_rad).abs() < 0.01);
        assert!(result.response > 0.5, "response {}", result.response);
    }

    /// The room of [`room_scan`] as a map's wall points, `spacing_m` apart.
    pub fn room_points(spacing_m: f64) -> Vec<(f64, f64)> {
        let mut points = Vec::new();
        let steps = |from: f64, to: f64| {
            let n = ((to - from) / spacing_m).round() as usize;
            (0..=n).map(move |i| from + i as f64 * spacing_m)
        };
        for x in steps(-3.0, 4.0) {
            points.extend([(x, -2.0), (x, 5.0)]);
        }
        for y in steps(-2.0, 5.0) {
            points.extend([(-3.0, y), (4.0, y)]);
        }
        let n = (2.0 * std::f64::consts::PI * 0.5 / spacing_m).ceil() as usize;
        for i in 0..n {
            let (sin, cos) = (2.0 * std::f64::consts::PI * i as f64 / n as f64).sin_cos();
            points.push((1.5 + 0.5 * cos, 2.0 + 0.5 * sin));
        }
        points
    }

    #[test]
    fn matching_against_map_points_recovers_a_wrong_prior() {
        let truth = Pose2::new(0.3, 0.2, 0.1);
        let prior = Pose2::new(truth.x_m - 0.12, truth.y_m + 0.08, truth.heading_rad - 0.06);
        let scan = LocalizedScan::new(&room_scan(truth, 360), Instant::now(), prior, 12.0);

        let mut matcher = ScanMatcher::new(0.5, 0.01, 0.1, 12.0, params());
        let result = matcher.match_points(&scan, &room_points(0.05), true, true);

        assert!(
            result.pose.squared_distance(&truth).sqrt() < 0.02,
            "matched {:?}, truth {truth:?}",
            result.pose
        );
        assert!(wrap_to_pi(result.pose.heading_rad - truth.heading_rad).abs() < 0.01);
        assert!(result.response > 0.5, "response {}", result.response);
    }

    #[test]
    fn an_identical_scan_matches_in_place_with_a_high_response() {
        let pose = Pose2::new(0.5, -0.5, 0.3);
        let reference = LocalizedScan::new(&room_scan(pose, 360), Instant::now(), pose, 12.0);
        let scan = LocalizedScan::new(&room_scan(pose, 360), Instant::now(), pose, 12.0);

        let mut matcher = ScanMatcher::new(0.5, 0.01, 0.03, 12.0, params());
        let result = matcher.match_scan(&scan, &[&reference], true, true);

        assert!(result.pose.squared_distance(&pose).sqrt() < 0.01);
        assert!(result.response > 0.9, "response {}", result.response);
        assert!(result.covariance[0][0] < 0.01 && result.covariance[1][1] < 0.01);
    }

    #[test]
    fn a_coarse_only_unpenalized_match_lands_within_a_coarse_step() {
        let truth = Pose2::new(0.3, 0.2, 0.1);
        let reference = LocalizedScan::new(
            &room_scan(Pose2::default(), 360),
            Instant::now(),
            Pose2::default(),
            12.0,
        );
        // A prior 2 m off: far outside the sequential window, well inside a
        // loop-closure one.
        let prior = Pose2::new(truth.x_m + 1.5, truth.y_m - 1.3, truth.heading_rad + 0.1);
        let scan = LocalizedScan::new(&room_scan(truth, 360), Instant::now(), prior, 12.0);

        let mut matcher = ScanMatcher::new(8.0, 0.05, 0.03, 12.0, params());
        let result = matcher.match_scan(&scan, &[&reference], false, false);

        assert!(
            result.pose.squared_distance(&truth).sqrt() < 0.1,
            "matched {:?}, truth {truth:?}",
            result.pose
        );
        assert!(result.response > 0.35, "response {}", result.response);
    }

    #[test]
    fn a_scan_without_points_keeps_its_prior_with_maximum_variance() {
        let prior = Pose2::new(1.0, 2.0, 0.0);
        let empty = LidarScan::new(vec![30.0; 10], vec![0.0; 10], 0.1, 30.0, 3.0);
        let scan = LocalizedScan::new(&empty, Instant::now(), prior, 12.0);

        let mut matcher = ScanMatcher::new(0.5, 0.01, 0.03, 12.0, params());
        let result = matcher.match_scan(&scan, &[], true, true);

        assert_eq!(result.pose, prior);
        assert_eq!(result.covariance[0][0], MAX_VARIANCE);
    }

    #[test]
    fn points_on_surfaces_facing_away_from_the_viewpoint_are_not_valid() {
        // A wall along y = 1 from x = 0 to 1, scanned left to right: seen
        // from below it's valid, from above it faces away.
        let wall: Vec<(f64, f64)> = (0..=10).map(|i| (i as f64 * 0.1 + 0.001, 1.0)).collect();
        assert!(valid_points(&wall, (0.5, -1.0)).is_empty());
        assert!(!valid_points(&wall, (0.5, 3.0)).is_empty());
    }
}
