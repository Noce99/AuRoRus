//! [`score`]: how good a map [`super::rebuild`] built is, as numbers to rank
//! runs by.
//!
//! A session recorded in simulation holds the truth - the true pose, and the
//! map driven on - so a map is scored against it directly: how far SLAM's
//! poses are from the true ones ([`Scores::trajectory`]), and how far its
//! walls from the true walls ([`Scores::walls`]). Odometry alone is scored
//! the same way ([`Scores::odometry`]), as the baseline SLAM has to beat.
//!
//! On the car there's no truth, only clues: how thick the walls are (a map
//! whose scans don't line up has doubled, smeared walls), how well scans
//! matched, and whether processing would keep up with the lidar.

use super::pose::{Pose2, wrap_to_pi};
use super::rebuild::{KeptScan, RebuildOutcome};
use crate::localization::session::Session;
use crate::topics::{SlamMap, StartState};
use std::collections::HashMap;

/// A wall at least this close to a true one counts as in place, in meters -
/// see [`WallError::within`].
pub const WALL_TOLERANCE_M: f64 = 0.1;
/// Distances to the true walls are capped at this, in meters, so a stray
/// cell far from any wall doesn't swamp the mean - see [`WallError`].
pub const WALL_DISTANCE_CAP_M: f64 = 1.0;
/// A match below this response counts as poor - see
/// [`Scores::poor_matches`].
pub const POOR_MATCH_RESPONSE: f64 = 0.5;

/// How far a trajectory is from the true one.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct TrajectoryError {
    /// Root mean square of the position errors, in meters.
    pub rms_m: f64,
    /// The largest position error, in meters.
    pub max_m: f64,
    /// Root mean square of the heading errors, in radians.
    pub heading_rms_rad: f64,
    /// The position error at the last kept scan, in meters.
    pub final_m: f64,
}

/// How far a map's walls are from the true map's.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct WallError {
    /// Mean distance from each of the map's occupied cells to the nearest
    /// true wall, each capped at [`WALL_DISTANCE_CAP_M`], in meters.
    pub mean_m: f64,
    /// The share of occupied cells within [`WALL_TOLERANCE_M`] of a true
    /// wall, from `0` to `1`.
    pub within: f64,
}

/// Every score of one run - see the module docs. Lower is better for every
/// error, distance, thickness and time; higher for responses and
/// [`WallError::within`].
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct Scores {
    /// SLAM's trajectory against the true one - `None` without ground
    /// truth.
    pub trajectory: Option<TrajectoryError>,
    /// Odometry's trajectory against the true one, at the same scans.
    pub odometry: Option<TrajectoryError>,
    /// The map's walls against the true map's - `None` without a true map.
    pub walls: Option<WallError>,
    /// The map's occupied cells per occupied cell bordering free space:
    /// `1` for walls one cell thick, more as scans that don't line up
    /// double and smear them - `None` for a map with no wall.
    pub wall_thickness: Option<f64>,
    /// Mean and lowest match response of the scans kept after the first.
    pub mean_response: Option<f64>,
    pub min_response: Option<f64>,
    /// Scans kept with a response below [`POOR_MATCH_RESPONSE`].
    pub poor_matches: usize,
    /// Mean and longest time a scan took to process, in milliseconds.
    pub mean_process_ms: Option<f64>,
    pub max_process_ms: Option<f64>,
    /// The lidar's period in the session (the median time between two
    /// scans), in milliseconds.
    pub scan_period_ms: Option<f64>,
    /// Scans that took longer to process than [`Self::scan_period_ms`] -
    /// live, SLAM would have fallen behind on them.
    pub slow_scans: usize,
}

/// Scores `outcome`, built from `session`, against the session's ground
/// truth (if recorded) and `true_walls` - the walls of the map driven on,
/// as [`super::rebuild::map_walls`] reads them - if known.
pub fn score(
    session: &Session,
    outcome: &RebuildOutcome,
    true_walls: Option<&[[f32; 2]]>,
) -> Scores {
    let anchor = session.anchor();
    let trajectory_error = |pose: fn(&KeptScan) -> [f64; 3]| {
        trajectory_error(
            session,
            &anchor,
            outcome.kept.iter().map(|kept| (kept.t_us, pose(kept))),
        )
    };
    let kept: Vec<_> = outcome.processed.iter().filter(|scan| scan.kept).collect();
    let responses: Vec<f64> = kept.iter().filter_map(|scan| scan.response).collect();
    let times: Vec<f64> = outcome
        .processed
        .iter()
        .map(|scan| scan.process_ms)
        .collect();
    let scan_period_ms = median_period_ms(session);
    Scores {
        trajectory: trajectory_error(|kept| kept.pose),
        odometry: trajectory_error(|kept| kept.odometric_pose),
        walls: true_walls.and_then(|walls| wall_error(&outcome.map, &anchor, walls)),
        wall_thickness: wall_thickness(&outcome.map),
        mean_response: mean(&responses),
        min_response: responses.iter().copied().reduce(f64::min),
        poor_matches: responses
            .iter()
            .filter(|&&response| response < POOR_MATCH_RESPONSE)
            .count(),
        mean_process_ms: mean(&times),
        max_process_ms: times.iter().copied().reduce(f64::max),
        scan_period_ms,
        slow_scans: scan_period_ms.map_or(0, |period| {
            times.iter().filter(|&&time| time > period).count()
        }),
    }
}

fn mean(values: &[f64]) -> Option<f64> {
    (!values.is_empty()).then(|| values.iter().sum::<f64>() / values.len() as f64)
}

/// `poses` - `(t_us, pose)` in SLAM's frame - placed in the world at
/// `anchor`, against the session's true pose at the same instants. `None`
/// without ground truth, or without poses.
fn trajectory_error(
    session: &Session,
    anchor: &StartState,
    poses: impl Iterator<Item = (u64, [f64; 3])>,
) -> Option<TrajectoryError> {
    let anchor = Pose2::new(anchor.x_m, anchor.y_m, anchor.heading_rad);
    let mut squared = 0.0;
    let mut heading_squared = 0.0;
    let mut max_m: f64 = 0.0;
    let mut final_m = 0.0;
    let mut count = 0;
    for (t_us, pose) in poses {
        let truth = true_pose_at(session, t_us)?;
        let world = anchor.compose(&Pose2::new(pose[0], pose[1], pose[2]));
        let error_m = (world.x_m - truth.x_m).hypot(world.y_m - truth.y_m);
        squared += error_m * error_m;
        heading_squared += wrap_to_pi(world.heading_rad - truth.heading_rad).powi(2);
        max_m = max_m.max(error_m);
        final_m = error_m;
        count += 1;
    }
    (count > 0).then(|| TrajectoryError {
        rms_m: (squared / f64::from(count)).sqrt(),
        max_m,
        heading_rms_rad: (heading_squared / f64::from(count)).sqrt(),
        final_m,
    })
}

/// The session's true pose at `t_us`, interpolated between the two samples
/// around it - the nearest one past either end. `None` without ground
/// truth.
fn true_pose_at(session: &Session, t_us: u64) -> Option<Pose2> {
    let truth = &session.ground_truth;
    let pose = |i: usize| {
        let (_, status) = &truth[i];
        Pose2::new(status.x_m, status.y_m, status.heading_rad)
    };
    let after = truth.partition_point(|(t, _)| *t <= t_us);
    if truth.is_empty() {
        return None;
    }
    if after == 0 {
        return Some(pose(0));
    }
    if after == truth.len() {
        return Some(pose(after - 1));
    }
    let (t0, t1) = (truth[after - 1].0, truth[after].0);
    let (a, b) = (pose(after - 1), pose(after));
    let f = if t1 > t0 {
        (t_us - t0) as f64 / (t1 - t0) as f64
    } else {
        0.0
    };
    Some(Pose2::new(
        a.x_m + (b.x_m - a.x_m) * f,
        a.y_m + (b.y_m - a.y_m) * f,
        a.heading_rad + wrap_to_pi(b.heading_rad - a.heading_rad) * f,
    ))
}

/// The world position of every occupied cell of `map`, its frame placed at
/// `anchor`.
fn occupied_cells(map: &SlamMap, anchor: &StartState) -> Vec<(f64, f64)> {
    let anchor = Pose2::new(anchor.x_m, anchor.y_m, anchor.heading_rad);
    let resolution_m = map.resolution_m_per_px;
    let width = map.width_px as usize;
    map.pixels
        .iter()
        .enumerate()
        .filter(|&(_, &pixel)| pixel == SlamMap::OCCUPIED)
        .map(|(i, _)| {
            anchor.transform_point(
                map.origin_x_m + ((i % width) as f64 + 0.5) * resolution_m,
                map.origin_y_m + ((i / width) as f64 + 0.5) * resolution_m,
            )
        })
        .collect()
}

/// `map`'s walls against `true_walls`, its frame placed at `anchor` -
/// `None` if either has none.
fn wall_error(map: &SlamMap, anchor: &StartState, true_walls: &[[f32; 2]]) -> Option<WallError> {
    let cells = occupied_cells(map, anchor);
    if cells.is_empty() || true_walls.is_empty() {
        return None;
    }
    let index = PointIndex::new(true_walls, WALL_DISTANCE_CAP_M);
    let distances: Vec<f64> = cells
        .iter()
        .map(|&(x, y)| index.nearest_m(x, y).min(WALL_DISTANCE_CAP_M))
        .collect();
    Some(WallError {
        mean_m: distances.iter().sum::<f64>() / distances.len() as f64,
        within: distances
            .iter()
            .filter(|&&distance| distance <= WALL_TOLERANCE_M)
            .count() as f64
            / distances.len() as f64,
    })
}

/// Points bucketed into square cells, for nearest-point queries up to the
/// cell's size.
struct PointIndex {
    cell_m: f64,
    cells: HashMap<(i64, i64), Vec<(f64, f64)>>,
}

impl PointIndex {
    fn new(points: &[[f32; 2]], cell_m: f64) -> Self {
        let mut cells: HashMap<(i64, i64), Vec<(f64, f64)>> = HashMap::new();
        for &[x, y] in points {
            let (x, y) = (f64::from(x), f64::from(y));
            cells
                .entry(((x / cell_m).floor() as i64, (y / cell_m).floor() as i64))
                .or_default()
                .push((x, y));
        }
        Self { cell_m, cells }
    }

    /// The distance from `(x, y)` to the nearest point, exact up to
    /// [`Self::cell_m`] - infinite if none is within one cell around.
    fn nearest_m(&self, x: f64, y: f64) -> f64 {
        let (cx, cy) = (
            (x / self.cell_m).floor() as i64,
            (y / self.cell_m).floor() as i64,
        );
        let mut best = f64::INFINITY;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for &(px, py) in self.cells.get(&(cx + dx, cy + dy)).into_iter().flatten() {
                    best = best.min((px - x).hypot(py - y));
                }
            }
        }
        best
    }
}

/// `map`'s occupied cells per occupied cell next to a free one - see
/// [`Scores::wall_thickness`].
fn wall_thickness(map: &SlamMap) -> Option<f64> {
    let (width, height) = (map.width_px as usize, map.height_px as usize);
    let pixel = |col: usize, row: usize| map.pixels[row * width + col];
    let mut occupied = 0usize;
    let mut bordering = 0usize;
    for row in 0..height {
        for col in 0..width {
            if pixel(col, row) != SlamMap::OCCUPIED {
                continue;
            }
            occupied += 1;
            let free = |c: usize, r: usize| pixel(c, r) == SlamMap::FREE;
            if (col > 0 && free(col - 1, row))
                || (col + 1 < width && free(col + 1, row))
                || (row > 0 && free(col, row - 1))
                || (row + 1 < height && free(col, row + 1))
            {
                bordering += 1;
            }
        }
    }
    (bordering > 0).then(|| occupied as f64 / bordering as f64)
}

/// The median time between two of `session`'s scans, in milliseconds.
fn median_period_ms(session: &Session) -> Option<f64> {
    let mut periods: Vec<u64> = session
        .scans
        .windows(2)
        .map(|pair| pair[1].0.saturating_sub(pair[0].0))
        .filter(|&period| period > 0)
        .collect();
    if periods.is_empty() {
        return None;
    }
    periods.sort_unstable();
    Some(periods[periods.len() / 2] as f64 / 1000.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topics::VehicleStatus;

    fn truth(samples: &[(u64, f64, f64, f64)]) -> Session {
        Session {
            ground_truth: samples
                .iter()
                .map(|&(t, x_m, y_m, heading_rad)| {
                    (
                        t,
                        VehicleStatus {
                            x_m,
                            y_m,
                            heading_rad,
                            ..VehicleStatus::default()
                        },
                    )
                })
                .collect(),
            ..Session::default()
        }
    }

    #[test]
    fn the_true_pose_is_interpolated_between_samples() {
        let session = truth(&[(0, 0.0, 0.0, 3.0), (100, 1.0, 2.0, -3.0)]);
        let pose = true_pose_at(&session, 50).unwrap();
        assert!((pose.x_m - 0.5).abs() < 1e-9 && (pose.y_m - 1.0).abs() < 1e-9);
        // Across the wrap: halfway from 3 to -3 rad is pi, not 0.
        assert!(wrap_to_pi(pose.heading_rad - std::f64::consts::PI).abs() < 1e-9);
        assert_eq!(true_pose_at(&session, 500).unwrap().x_m, 1.0);
        assert!(true_pose_at(&Session::default(), 0).is_none());
    }

    #[test]
    fn the_trajectory_error_is_measured_in_the_world() {
        // The anchor turns SLAM's frame a quarter left: SLAM's (1, 0) is the
        // world's (0, 1).
        let mut session = truth(&[(0, 0.0, 1.0, 0.0), (100, 0.0, 1.0, 0.0)]);
        session.info.anchor = Some(StartState {
            heading_rad: std::f64::consts::FRAC_PI_2,
            ..StartState::default()
        });
        let anchor = session.anchor();
        let exact = trajectory_error(
            &session,
            &anchor,
            [(0, [1.0, 0.0, -std::f64::consts::FRAC_PI_2])].into_iter(),
        );
        assert!(exact.unwrap().rms_m < 1e-9);
        let off = trajectory_error(&session, &anchor, [(50, [1.0, 0.3, 0.0])].into_iter()).unwrap();
        assert!((off.rms_m - 0.3).abs() < 1e-9);
        assert_eq!(off.max_m, off.final_m);
    }

    fn map(width_px: u32, height_px: u32, pixels: Vec<u8>) -> SlamMap {
        SlamMap {
            resolution_m_per_px: 0.1,
            origin_x_m: 0.0,
            origin_y_m: 0.0,
            width_px,
            height_px,
            pixels: pixels.into(),
            trajectory: Vec::new(),
        }
    }

    #[test]
    fn walls_on_the_true_walls_score_zero() {
        const F: u8 = SlamMap::FREE;
        const O: u8 = SlamMap::OCCUPIED;
        // One occupied cell, centered on (0.15, 0.05).
        let map = map(2, 1, vec![F, O]);
        let anchor = StartState::default();
        let on = wall_error(&map, &anchor, &[[0.15, 0.05]]).unwrap();
        assert!(on.mean_m < 1e-6 && on.within == 1.0);
        let off = wall_error(&map, &anchor, &[[0.15, 0.55]]).unwrap();
        assert!((off.mean_m - 0.5).abs() < 1e-6 && off.within == 0.0);
        let far = wall_error(&map, &anchor, &[[50.0, 50.0]]).unwrap();
        assert_eq!(far.mean_m, WALL_DISTANCE_CAP_M);
    }

    #[test]
    fn doubled_walls_are_thicker() {
        const F: u8 = SlamMap::FREE;
        const O: u8 = SlamMap::OCCUPIED;
        const U: u8 = SlamMap::UNKNOWN;
        let thin = map(3, 1, vec![F, O, U]);
        let thick = map(4, 1, vec![F, O, O, U]);
        assert_eq!(wall_thickness(&thin), Some(1.0));
        assert_eq!(wall_thickness(&thick), Some(2.0));
        assert_eq!(wall_thickness(&map(1, 1, vec![F])), None);
    }
}
