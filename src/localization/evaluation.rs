//! [`evaluate`]: localizes a recorded [`Session`] again, offline, with a
//! [`Pipeline`] - an odometry and a localization algorithm, each with its
//! parameters - and [`score`]s the result against a reference, lap by lap:
//! in simulation the ground truth, on the car another run.
//!
//! Every pipeline sees the very same drive, so their errors and their cost
//! compare fairly. See `documentation/localization_lab.md`.
//!
//! - **The pose scored** is the one a driving algorithm would have read:
//!   odometry composed onto the latest correction, at every odometry
//!   sample, each correction applied only once its scan would have been
//!   processed live (see [`super::slam`]'s `localize`).
//! - **Laps** are counted along a race line from where the reference
//!   projects onto it: lap `0` up to the first forward crossing of the
//!   start/finish line, then `1`, `2`, ...
//! - **Absolute errors** (position, split along and across the reference's
//!   heading, and heading) say how far off the estimate is. **Drift** - the
//!   error of the motion over the last [`DRIFT_WINDOW_M`] driven - says how
//!   fast it wanders, which is what tells odometry settings apart: alone,
//!   odometry's absolute error only grows.

use super::dead_reckoning::{DeadReckoningConfig, integrate_offline};
use super::particle_filter::ParticleFilterConfig;
use super::session::{Session, Timed};
use super::slam::pose::{Pose2, wrap_to_pi};
use super::slam::{SlamConfig, localize};
use crate::environment::{Map, SpeedPoint};
use crate::geometry::Line;
use crate::topics::{LocalizationAlgorithm, Odometry, SelectedMap};
use std::ops::ControlFlow;
use std::path::Path;
use std::time::{Duration, Instant};

/// An estimate this far from the reference or more counts as lost, in
/// meters - see [`LapScore::lost_s`].
pub const LOST_M: f64 = 0.5;
/// The distance driven [`ErrorSample::drift_m`] measures the motion's error
/// over, in meters.
pub const DRIFT_WINDOW_M: f64 = 5.0;
/// Only a crossing of the start/finish line this far around the lap at
/// least since the last one completes a lap, as a fraction of its length -
/// so driving back and forth over the line doesn't count laps.
const MIN_LAP_FRACTION: f64 = 0.5;
/// How far along the race line past the last projection the next one is
/// searched, in meters.
const PROJECTION_WINDOW_M: f64 = 10.0;

/// Where a pipeline's odometry comes from.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OdometrySource {
    /// The odometry recorded with the session, as it ran live.
    Recorded,
    /// The recorded IMU integrated again offline with these settings.
    Integrated(DeadReckoningConfig),
}

/// What localizes a session: an odometry, and an algorithm correcting it -
/// or none, to score odometry alone.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pipeline {
    pub odometry: OdometrySource,
    pub algorithm: Option<LocalizationAlgorithm>,
    /// Scan matching's parameters (and the odometry buffer's).
    pub slam: SlamConfig,
    pub particle_filter: ParticleFilterConfig,
}

/// A pose `[x_m, y_m, heading_rad]` at a time, in microseconds since the
/// session started.
pub type TimedPose = Timed<[f64; 3]>;

/// One scan an [`Evaluation`] processed.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct ScanStep {
    /// When it was taken, in microseconds since the session started.
    pub t_us: u64,
    /// How long the algorithm took on it, in milliseconds.
    pub process_ms: f64,
    /// How confident the algorithm is, in its own terms: scan matching's
    /// response (`0` to `1`), the particle filter's position spread
    /// (meters).
    pub quality: Option<f64>,
}

/// What [`evaluate`] made of a session.
#[derive(Debug, Clone, Default)]
pub struct Evaluation {
    /// The pose a driving algorithm would have read at every odometry
    /// sample, oldest first.
    pub track: Vec<TimedPose>,
    /// Every scan processed, oldest first - none for odometry alone.
    pub scans: Vec<ScanStep>,
    /// Scans never processed: odometry didn't cover the instant they were
    /// taken at.
    pub dropped_scans: usize,
    /// How long integrating odometry took per sample, in microseconds -
    /// `None` for recorded odometry.
    pub odometry_us_per_sample: Option<f64>,
    /// How long the whole evaluation took.
    pub elapsed: Duration,
}

/// The map in `folder`, as localization takes it.
pub fn load_map(folder: &Path) -> Result<SelectedMap, String> {
    let map = Map::load(folder).map_err(|err| format!("{folder:?}: {err}"))?;
    Ok(SelectedMap {
        path: Some(folder.to_path_buf()),
        width_px: map.raster.width_px,
        height_px: map.raster.height_px,
        pixels: map.raster.to_bytes().into(),
        info: Some(map.info),
    })
}

/// The session's odometry from `source`.
pub fn odometry(session: &Session, source: &OdometrySource) -> Vec<Timed<Odometry>> {
    match source {
        OdometrySource::Recorded => session.odometry.clone(),
        OdometrySource::Integrated(config) => {
            let Some(&(first_t_us, _)) = session.imu.first() else {
                return Vec::new();
            };
            // Integrated from what odometry was when the IMU starts: the
            // latest recorded sample by then, else the first.
            let seed = session
                .odometry
                .iter()
                .rev()
                .find(|(t, _)| *t <= first_t_us)
                .or(session.odometry.first())
                .map(|&(_, odometry)| odometry)
                .unwrap_or_default();
            let lever_m = session.geometry.unwrap_or_default().rear_axle_to_cg_m;
            integrate_offline(&session.imu, seed, config, lever_m)
        }
    }
}

/// The session's true pose, as recorded - empty unless recorded in
/// simulation.
pub fn ground_truth(session: &Session) -> Vec<TimedPose> {
    session
        .ground_truth
        .iter()
        .map(|(t, status)| (*t, [status.x_m, status.y_m, status.heading_rad]))
        .collect()
}

/// Where odometry's frame starts on the map: from the true pose at the
/// first odometry sample in simulation, else where live localization had
/// it when recording started, else the session's anchor (where odometry
/// was last reset).
pub fn initial_map_to_odom(session: &Session, odometry: &[Timed<Odometry>]) -> [f64; 3] {
    let truth = ground_truth(session);
    if let Some(&(t_us, first)) = odometry.first()
        && let Some(true_pose) = pose_at(&truth, t_us)
    {
        let pose = to_pose(true_pose).compose(&Pose2::from_odometry(&first).inverse());
        return from_pose(pose);
    }
    if let Some(map_to_odom) = session
        .info
        .localization
        .as_ref()
        .and_then(|info| info.map_to_odom)
    {
        return map_to_odom;
    }
    let anchor = session.anchor();
    [anchor.x_m, anchor.y_m, anchor.heading_rad]
}

/// Localizes `session` again on `map` with `pipeline` - see the module
/// docs. `progress` is called every few scans with how far it got, from
/// `0` to `1`; [`ControlFlow::Break`] stops it, and `Ok(None)` is returned.
pub fn evaluate(
    session: &Session,
    map: &SelectedMap,
    pipeline: &Pipeline,
    progress: impl FnMut(f64) -> ControlFlow<()>,
) -> Result<Option<Evaluation>, String> {
    let started = Instant::now();
    let odometry = odometry(session, &pipeline.odometry);
    let odometry_us_per_sample = match pipeline.odometry {
        OdometrySource::Recorded => None,
        OdometrySource::Integrated(_) => {
            Some(started.elapsed().as_secs_f64() * 1e6 / odometry.len().max(1) as f64)
        }
    };
    if odometry.is_empty() {
        return Err("the session has no odometry to localize from".to_string());
    }
    let map_to_odom = to_pose(initial_map_to_odom(session, &odometry));
    let Some(localized) = localize::localize(
        &odometry,
        &session.scans,
        map,
        pipeline.algorithm,
        &pipeline.slam,
        pipeline.particle_filter,
        map_to_odom,
        progress,
    )?
    else {
        return Ok(None);
    };
    Ok(Some(Evaluation {
        track: localized
            .track
            .into_iter()
            .map(|(t, pose)| (t, from_pose(pose)))
            .collect(),
        scans: localized
            .steps
            .iter()
            .map(|step| ScanStep {
                t_us: step.t_us,
                process_ms: step.process_ms,
                quality: step.quality,
            })
            .collect(),
        dropped_scans: localized.dropped_scans,
        odometry_us_per_sample,
        elapsed: started.elapsed(),
    }))
}

fn to_pose(pose: [f64; 3]) -> Pose2 {
    Pose2::new(pose[0], pose[1], pose[2])
}

fn from_pose(pose: Pose2) -> [f64; 3] {
    [pose.x_m, pose.y_m, pose.heading_rad]
}

/// The pose of `poses` (oldest first) at `t_us`, interpolated between the
/// two around it. `None` outside them, or without any.
pub fn pose_at(poses: &[TimedPose], t_us: u64) -> Option<[f64; 3]> {
    let after = poses.partition_point(|(t, _)| *t <= t_us);
    if after == 0 {
        return None;
    }
    let (t0, a) = poses[after - 1];
    if t0 == t_us {
        return Some(a);
    }
    let &(t1, b) = poses.get(after)?;
    let f = (t_us - t0) as f64 / (t1 - t0) as f64;
    Some([
        a[0] + (b[0] - a[0]) * f,
        a[1] + (b[1] - a[1]) * f,
        wrap_to_pi(a[2] + wrap_to_pi(b[2] - a[2]) * f),
    ])
}

/// The reference, placed on a race line: the lap and the arc length at
/// each of its poses, and how far it had driven.
#[derive(Debug, Clone)]
pub struct Laps {
    /// One per reference pose: `(t_us, lap, s_m, driven_m)`.
    samples: Vec<(u64, u32, f64, f64)>,
    /// The race line's length, in meters.
    pub lap_m: f64,
    /// When each lap from `1` on started, in microseconds.
    pub lap_starts_us: Vec<u64>,
}

impl Laps {
    /// `reference` (oldest first) placed on `race_line` - `None` if the
    /// line can't make a closed one.
    pub fn new(reference: &[TimedPose], race_line: &[SpeedPoint]) -> Option<Self> {
        let line = Line::new(race_line.to_vec())?;
        let lap_m = line.lap_m();
        let mut samples = Vec::with_capacity(reference.len());
        let mut lap_starts_us = Vec::new();
        let mut lap = 0;
        let mut hint = None;
        let mut last: Option<(f64, f64, f64)> = None; // (s, x, y)
        let mut driven_m = 0.0;
        let mut since_crossing_m = 0.0;
        for &(t_us, [x, y, _]) in reference {
            let nearest = line.nearest(x, y, hint, PROJECTION_WINDOW_M);
            hint = Some(nearest.segment);
            let s_m = nearest.s_m;
            if let Some((last_s, last_x, last_y)) = last {
                driven_m += (x - last_x).hypot(y - last_y);
                let ds = wrapped_half(s_m - last_s, lap_m);
                since_crossing_m += ds;
                // Forward over the start/finish line - the first time
                // whatever came before, as the drive may well start just
                // behind the line.
                let crossed = ds > 0.0 && s_m < last_s;
                if crossed && (lap == 0 || since_crossing_m >= MIN_LAP_FRACTION * lap_m) {
                    lap += 1;
                    lap_starts_us.push(t_us);
                    since_crossing_m = 0.0;
                }
            }
            last = Some((s_m, x, y));
            samples.push((t_us, lap, s_m, driven_m));
        }
        Some(Self {
            samples,
            lap_m,
            lap_starts_us,
        })
    }

    /// How many laps there are, counting lap `0`.
    pub fn count(&self) -> u32 {
        self.samples.last().map_or(0, |&(_, lap, _, _)| lap + 1)
    }

    /// The lap, arc length and distance driven of the reference at `t_us`:
    /// those of its latest pose by then. `None` before its first.
    pub fn at(&self, t_us: u64) -> Option<(u32, f64, f64)> {
        let after = self.samples.partition_point(|(t, ..)| *t <= t_us);
        let &(_, lap, s_m, driven_m) = self.samples.get(after.checked_sub(1)?)?;
        Some((lap, s_m, driven_m))
    }
}

/// `ds` wrapped into `(-lap_m / 2, lap_m / 2]`.
fn wrapped_half(ds: f64, lap_m: f64) -> f64 {
    let wrapped = ds.rem_euclid(lap_m);
    if wrapped > lap_m / 2.0 {
        wrapped - lap_m
    } else {
        wrapped
    }
}

/// How far an estimate was from the reference at one instant.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct ErrorSample {
    pub t_us: u64,
    pub lap: u32,
    /// Where along the race line, in meters.
    pub s_m: f64,
    /// Distance between the two positions, in meters.
    pub position_m: f64,
    /// The position error along the reference's heading - positive ahead.
    pub along_m: f64,
    /// ... and across it - positive to its left.
    pub cross_m: f64,
    /// Heading error, in radians, wrapped to `(-pi, pi]`.
    pub heading_rad: f64,
    /// How wrong the motion over the last [`DRIFT_WINDOW_M`] driven is, in
    /// meters - `None` until that far was driven.
    pub drift_m: Option<f64>,
}

/// The error of `track` against `reference` at every pose of `track` the
/// reference covers (both oldest first), placed on `laps`.
pub fn errors(track: &[TimedPose], reference: &[TimedPose], laps: &Laps) -> Vec<ErrorSample> {
    // The reference and the track, every pose the reference covers, with
    // the distance driven by then - for the drift's window.
    let paired: Vec<(u64, Pose2, Pose2, u32, f64, f64)> = track
        .iter()
        .filter_map(|&(t_us, estimate)| {
            let truth = pose_at(reference, t_us)?;
            let (lap, s_m, driven_m) = laps.at(t_us)?;
            Some((t_us, to_pose(estimate), to_pose(truth), lap, s_m, driven_m))
        })
        .collect();
    let mut window_start = 0;
    paired
        .iter()
        .map(|&(t_us, estimate, truth, lap, s_m, driven_m)| {
            let (dx, dy) = (estimate.x_m - truth.x_m, estimate.y_m - truth.y_m);
            let (sin, cos) = truth.heading_rad.sin_cos();
            while window_start + 1 < paired.len()
                && driven_m - paired[window_start + 1].5 >= DRIFT_WINDOW_M
            {
                window_start += 1;
            }
            let (_, estimate_then, truth_then, _, _, driven_then) = paired[window_start];
            let drift_m = (driven_m - driven_then >= DRIFT_WINDOW_M).then(|| {
                let moved = estimate_then.inverse().compose(&estimate);
                let truly_moved = truth_then.inverse().compose(&truth);
                let wrong = truly_moved.inverse().compose(&moved);
                wrong.x_m.hypot(wrong.y_m)
            });
            ErrorSample {
                t_us,
                lap,
                s_m,
                position_m: dx.hypot(dy),
                along_m: dx * cos + dy * sin,
                cross_m: -dx * sin + dy * cos,
                heading_rad: wrap_to_pi(estimate.heading_rad - truth.heading_rad),
                drift_m,
            }
        })
        .collect()
}

/// One lap's (or the whole drive's) scores.
#[derive(Debug, Clone, Copy, PartialEq, Default, serde::Serialize)]
pub struct LapScore {
    /// The lap - `None` for the whole drive.
    pub lap: Option<u32>,
    /// How long it lasted, in seconds.
    pub duration_s: f64,
    /// The position error: RMS, 95th percentile and worst, in meters.
    pub rms_m: f64,
    pub p95_m: f64,
    pub max_m: f64,
    /// RMS of the error across the reference's heading, in meters.
    pub cross_rms_m: f64,
    /// RMS of the heading error, in radians.
    pub heading_rms_rad: f64,
    /// RMS of [`ErrorSample::drift_m`], where there's one.
    pub drift_rms_m: Option<f64>,
    /// How long the estimate was [`LOST_M`] or more off, in seconds.
    pub lost_s: f64,
    /// How long a scan took: on average, 95th percentile and worst, in
    /// milliseconds - `None` without scans.
    pub mean_ms: Option<f64>,
    pub p95_ms: Option<f64>,
    pub max_ms: Option<f64>,
    /// Scans that took longer than the lidar's period: live, the algorithm
    /// would have fallen behind on them.
    pub slow_scans: usize,
    pub scans: usize,
}

/// The scores of `errors` and `scans`, for every lap of `laps` and - first -
/// for the whole drive. `scan_period_ms` is the lidar's.
pub fn score(
    errors: &[ErrorSample],
    scans: &[ScanStep],
    laps: &Laps,
    scan_period_ms: Option<f64>,
) -> Vec<LapScore> {
    let mut scores = vec![lap_score(None, errors, scans, scan_period_ms)];
    for lap in 0..laps.count() {
        let errors: Vec<_> = errors.iter().filter(|e| e.lap == lap).copied().collect();
        let scans: Vec<_> = scans
            .iter()
            .filter(|scan| laps.at(scan.t_us).is_some_and(|(l, ..)| l == lap))
            .copied()
            .collect();
        if !errors.is_empty() {
            scores.push(lap_score(Some(lap), &errors, &scans, scan_period_ms));
        }
    }
    scores
}

fn lap_score(
    lap: Option<u32>,
    errors: &[ErrorSample],
    scans: &[ScanStep],
    scan_period_ms: Option<f64>,
) -> LapScore {
    let rms = |values: &mut dyn Iterator<Item = f64>| {
        let (sum, count) = values.fold((0.0, 0usize), |(sum, n), v| (sum + v * v, n + 1));
        (count > 0).then(|| (sum / count as f64).sqrt())
    };
    let mut positions: Vec<f64> = errors.iter().map(|e| e.position_m).collect();
    positions.sort_by(f64::total_cmp);
    let mut times: Vec<f64> = scans.iter().map(|scan| scan.process_ms).collect();
    times.sort_by(f64::total_cmp);
    let lost_s = errors
        .windows(2)
        .filter(|pair| pair[0].position_m >= LOST_M)
        .map(|pair| (pair[1].t_us - pair[0].t_us) as f64 / 1e6)
        .sum();
    LapScore {
        lap,
        duration_s: match (errors.first(), errors.last()) {
            (Some(first), Some(last)) => (last.t_us - first.t_us) as f64 / 1e6,
            _ => 0.0,
        },
        rms_m: rms(&mut errors.iter().map(|e| e.position_m)).unwrap_or(0.0),
        p95_m: percentile(&positions, 0.95).unwrap_or(0.0),
        max_m: positions.last().copied().unwrap_or(0.0),
        cross_rms_m: rms(&mut errors.iter().map(|e| e.cross_m)).unwrap_or(0.0),
        heading_rms_rad: rms(&mut errors.iter().map(|e| e.heading_rad)).unwrap_or(0.0),
        drift_rms_m: rms(&mut errors.iter().filter_map(|e| e.drift_m)),
        lost_s,
        mean_ms: (!times.is_empty()).then(|| times.iter().sum::<f64>() / times.len() as f64),
        p95_ms: percentile(&times, 0.95),
        max_ms: times.last().copied(),
        slow_scans: scan_period_ms
            .map_or(0, |period| times.iter().filter(|&&t| t > period).count()),
        scans: scans.len(),
    }
}

/// The `q` quantile of `sorted` (ascending), nearest rank.
fn percentile(sorted: &[f64], q: f64) -> Option<f64> {
    if sorted.is_empty() {
        return None;
    }
    let rank = ((q * sorted.len() as f64).ceil() as usize).clamp(1, sorted.len());
    Some(sorted[rank - 1])
}

/// The median time between two of `session`'s scans, in milliseconds.
pub fn scan_period_ms(session: &Session) -> Option<f64> {
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

    /// A square race line, 10 m a side, counter-clockwise from the origin -
    /// a point every 0.5 m, as dense as a real line, which projections rely
    /// on.
    fn square() -> Vec<SpeedPoint> {
        let corners = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
        (0..4)
            .flat_map(|side| {
                let ((x0, y0), (x1, y1)) = (corners[side], corners[(side + 1) % 4]);
                (0..20).map(move |i| {
                    let f = f64::from(i) / 20.0;
                    SpeedPoint {
                        x: x0 + (x1 - x0) * f,
                        y: y0 + (y1 - y0) * f,
                        speed_mps: 1.0,
                    }
                })
            })
            .collect()
    }

    /// Driving the square at 1 m/s, from `start_m` along it, for `seconds`,
    /// one pose every 0.1 s.
    fn around(start_m: f64, seconds: f64) -> Vec<TimedPose> {
        let line = Line::new(square()).unwrap();
        (0..=(seconds * 10.0) as u64)
            .map(|i| {
                let s = (start_m + i as f64 * 0.1).rem_euclid(line.lap_m());
                let side = (s / 10.0).floor();
                let point = line.at(s);
                let heading = side * std::f64::consts::FRAC_PI_2;
                (i * 100_000, [point.x, point.y, wrap_to_pi(heading)])
            })
            .collect()
    }

    #[test]
    fn laps_start_at_each_forward_crossing() {
        // Starting 5 m before the line: lap 0, then two whole laps of 40 m
        // and part of a third.
        let reference = around(35.0, 90.0);
        let laps = Laps::new(&reference, &square()).unwrap();
        assert_eq!(laps.count(), 4);
        assert_eq!(laps.lap_starts_us.len(), 3);
        assert_eq!(laps.at(0).unwrap().0, 0);
        assert_eq!(laps.at(6_000_000).unwrap().0, 1);
        assert_eq!(laps.at(46_000_000).unwrap().0, 2);
        assert!((laps.lap_m - 40.0).abs() < 1e-9);
    }

    #[test]
    fn going_back_and_forth_over_the_line_counts_one_lap() {
        let mut reference = around(39.0, 2.0); // over the line, to s = 1
        let back: Vec<_> = around(39.0, 2.0).into_iter().rev().collect();
        let offset = reference.last().unwrap().0 + 100_000;
        reference.extend(back.iter().map(|&(t, p)| (offset + 2_000_000 - t, p)));
        let offset = reference.last().unwrap().0 + 100_000;
        reference.extend(around(39.0, 2.0).iter().map(|&(t, p)| (offset + t, p)));
        let laps = Laps::new(&reference, &square()).unwrap();
        assert_eq!(laps.count(), 2);
    }

    #[test]
    fn errors_split_along_and_across_the_reference() {
        let reference = around(1.0, 3.0);
        // 0.3 m ahead and 0.4 m to the left along the first side.
        let track: Vec<_> = reference
            .iter()
            .map(|&(t, [x, y, h])| (t, [x + 0.3, y + 0.4, h + 0.1]))
            .collect();
        let laps = Laps::new(&reference, &square()).unwrap();
        let errors = errors(&track, &reference, &laps);
        assert_eq!(errors.len(), track.len());
        for e in &errors {
            assert!((e.position_m - 0.5).abs() < 1e-9);
            assert!((e.along_m - 0.3).abs() < 1e-9);
            assert!((e.cross_m - 0.4).abs() < 1e-9);
            assert!((e.heading_rad - 0.1).abs() < 1e-9);
        }
    }

    #[test]
    fn a_constant_offset_doesnt_drift_but_a_growing_one_does() {
        let reference = around(1.0, 8.0);
        let laps = Laps::new(&reference, &square()).unwrap();
        let shifted: Vec<_> = reference
            .iter()
            .map(|&(t, [x, y, h])| (t, [x + 1.0, y, h]))
            .collect();
        let errors_shifted = errors(&shifted, &reference, &laps);
        let drift: Vec<_> = errors_shifted.iter().filter_map(|e| e.drift_m).collect();
        assert!(!drift.is_empty());
        assert!(drift.iter().all(|&d| d < 1e-9));
        assert!(errors_shifted[0].drift_m.is_none());

        // 10% too far: over 5 m driven, the motion is 0.5 m off.
        let stretched: Vec<_> = reference
            .iter()
            .map(|&(t, [x, y, h])| (t, [1.0 + (x - 1.0) * 1.1, y, h]))
            .collect();
        let errors_stretched = errors(&stretched, &reference, &laps);
        let last = errors_stretched.last().unwrap().drift_m.unwrap();
        assert!((last - 0.5).abs() < 0.02, "drift {last}");
    }

    #[test]
    fn scores_cover_the_drive_and_each_lap() {
        let reference = around(35.0, 40.0);
        let laps = Laps::new(&reference, &square()).unwrap();
        let track: Vec<_> = reference
            .iter()
            .map(|&(t, [x, y, h])| {
                // Lost by 1 m for the first second.
                let off = if t < 1_000_000 { 1.0 } else { 0.1 };
                (t, [x, y + off, h])
            })
            .collect();
        let errors = errors(&track, &reference, &laps);
        let scans = [
            ScanStep {
                t_us: 0,
                process_ms: 2.0,
                quality: None,
            },
            ScanStep {
                t_us: 6_000_000,
                process_ms: 30.0,
                quality: None,
            },
        ];
        let scores = score(&errors, &scans, &laps, Some(25.0));
        assert_eq!(scores.len(), 3);
        let whole = scores[0];
        assert_eq!(whole.lap, None);
        assert_eq!(whole.scans, 2);
        assert_eq!(whole.slow_scans, 1);
        assert!((whole.lost_s - 1.0).abs() < 1e-9);
        assert!((whole.max_m - 1.0).abs() < 1e-9);
        assert_eq!(whole.mean_ms, Some(16.0));
        assert_eq!(scores[1].lap, Some(0));
        assert_eq!(scores[1].scans, 1);
        assert_eq!(scores[2].lap, Some(1));
        assert!((scores[2].rms_m - 0.1).abs() < 1e-9);
    }

    #[test]
    fn poses_are_interpolated_with_the_heading_wrapped() {
        let poses = [(0, [0.0, 0.0, 3.0]), (10, [1.0, 2.0, -3.0])];
        let mid = pose_at(&poses, 5).unwrap();
        assert!((mid[0] - 0.5).abs() < 1e-12 && (mid[1] - 1.0).abs() < 1e-12);
        assert!(mid[2].abs() > 3.0, "{mid:?} took the short way round");
        assert_eq!(pose_at(&poses, 0), Some([0.0, 0.0, 3.0]));
        assert_eq!(pose_at(&poses, 11), None);
    }
}
