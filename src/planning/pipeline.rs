//! [`plan`]: the whole pipeline from a loaded map to its race line - the
//! track, a reference line (the map's centerline, or one computed from its
//! walls), the minimum-curvature line around it, a speed profile, and - for
//! [`PlanningObjective::MinTime`] - the minimum-time line from there.

use super::config::PlanningConfig;
use super::geometry::{
    Point2, loop_length, max_abs_curvature, orient_from, resample_even_spacing, smooth,
};
use super::min_curvature::{self, Iteration};
use super::min_time;
use super::speed_profile::{lap_time, speeds};
use super::track::TrackGrid;
use super::{PlanError, centerline};
use crate::environment::{Map, SpeedPoint};
use crate::topics::PlanningObjective;

/// What [`plan`] tells its caller along the way.
pub enum Progress<'a> {
    /// A new step started, e.g. `"Computing the centerline"`.
    Stage(String),
    /// An optimization iteration finished - see
    /// [`min_curvature::Iteration`].
    Iteration {
        total: usize,
        iteration: Iteration<'a>,
    },
    /// A minimum-time outer iteration finished - see
    /// [`min_time::Iteration`].
    MinTimeIteration {
        total: usize,
        iteration: min_time::Iteration<'a>,
    },
}

/// A minimum-time line and its numbers.
pub struct MinTimeLine {
    /// The line, starting at the start/finish line.
    pub race_line: Vec<SpeedPoint>,
    /// Length of one lap, in meters.
    pub lap_length_m: f64,
    /// Time of one lap, in seconds.
    pub lap_time_s: f64,
}

/// A planned race line.
pub struct PlannedLines {
    /// The race line, starting at the start/finish line.
    pub race_line: Vec<SpeedPoint>,
    /// The centerline computed from the walls, with its own speed profile
    /// - `None` when the map already had one.
    pub computed_centerline: Option<Vec<SpeedPoint>>,
    /// Length of one lap of the race line, in meters.
    pub lap_length_m: f64,
    /// Time of one lap at the race line's speed profile, in seconds.
    pub lap_time_s: f64,
    /// Largest curvature magnitude along the race line, in 1/m.
    pub max_curvature_per_m: f64,
    /// Largest curvature magnitude along the reference line, in 1/m.
    pub reference_max_curvature_per_m: f64,
    /// For [`PlanningObjective::MinTime`], the minimum-time line - or why
    /// there's none, the minimum-curvature line being planned anyway.
    pub min_time: Option<Result<MinTimeLine, PlanError>>,
}

/// Plans `map`'s race line for `objective` with `config`. `progress` is
/// told what's going on, and cancels the planning - with
/// [`PlanError::Cancelled`] - by returning `false`.
pub fn plan(
    map: &Map,
    config: &PlanningConfig,
    objective: PlanningObjective,
    progress: &mut dyn FnMut(Progress) -> bool,
) -> Result<PlannedLines, PlanError> {
    let stage = |name: &str, progress: &mut dyn FnMut(Progress) -> bool| {
        if progress(Progress::Stage(name.to_string())) {
            Ok(())
        } else {
            Err(PlanError::Cancelled)
        }
    };

    stage("Extracting the track", progress)?;
    let grid = TrackGrid::build(map)?;

    let has_centerline = map.centerline.len() >= 3;
    let mut reference: Vec<Point2> = if has_centerline {
        map.centerline
            .iter()
            .map(|point| Point2 {
                x: point.x,
                y: point.y,
            })
            .collect()
    } else {
        stage("Computing the centerline", progress)?;
        let extracted = centerline::extract(&grid)?;
        // Marching squares steps a fraction of a pixel at a time: even the
        // points out before smoothing, so the window spans a fixed length.
        let even = resample_even_spacing(&extracted, config.spacing_m);
        smooth(&even, config.centerline_smoothing_window)
    };
    let (x_m, y_m, heading_rad) = map.info.start_finish_line.start_pose();
    orient_from(&mut reference, x_m, y_m, heading_rad);
    let reference = resample_even_spacing(&reference, config.spacing_m);

    stage("Optimizing", progress)?;
    let optimizer_config = config.min_curvature();
    let total = optimizer_config.iterations.max(1);
    let line = min_curvature::optimize(&reference, &grid, &optimizer_config, &mut |iteration| {
        progress(Progress::Iteration { total, iteration })
    })?;

    stage("Computing the speed profile", progress)?;
    let limits = config.speed_limits();
    let race_speeds = speeds(&line, &limits);
    let computed_centerline =
        (!has_centerline).then(|| with_speeds(&reference, &speeds(&reference, &limits)));

    let min_time = match objective {
        PlanningObjective::MinCurvature => None,
        PlanningObjective::MinTime => {
            stage("Optimizing the lap time", progress)?;
            let min_time_config = config.min_time();
            let total = min_time_config.max_outer_iterations.max(1);
            let optimized = min_time::optimize(&line, &grid, &min_time_config, &mut |iteration| {
                progress(Progress::MinTimeIteration { total, iteration })
            });
            match optimized {
                Err(PlanError::Cancelled) => return Err(PlanError::Cancelled),
                Err(err) => Some(Err(err)),
                Ok(points) => {
                    // Speeds that respect the limits exactly - the ALM only does
                    // up to a tolerance. Kept at the optimizer's own spacing:
                    // resampling a polyline linearly puts kinks at its old
                    // points, which the speed profile would brake for.
                    let point_speeds = speeds(&points, &limits);
                    Some(Ok(MinTimeLine {
                        lap_length_m: loop_length(&points),
                        lap_time_s: lap_time(&points, &point_speeds),
                        race_line: with_speeds(&points, &point_speeds),
                    }))
                }
            }
        }
    };

    Ok(PlannedLines {
        lap_length_m: loop_length(&line),
        lap_time_s: lap_time(&line, &race_speeds),
        max_curvature_per_m: max_abs_curvature(&line),
        reference_max_curvature_per_m: max_abs_curvature(&reference),
        race_line: with_speeds(&line, &race_speeds),
        computed_centerline,
        min_time,
    })
}

fn with_speeds(points: &[Point2], speeds: &[f64]) -> Vec<SpeedPoint> {
    points
        .iter()
        .zip(speeds)
        .map(|(point, &speed_mps)| SpeedPoint {
            x: point.x,
            y: point.y,
            speed_mps,
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::environment::{GenerationConfig, generate};
    use crate::planning::track::tests::ring_map;

    #[test]
    fn a_ring_without_a_centerline_gets_one_and_a_race_line_on_its_outside() {
        let map = ring_map(1.0, 2.0, 0.02);
        let config = PlanningConfig {
            spacing_m: 0.05,
            ..PlanningConfig::default()
        };
        let planned = plan(&map, &config, PlanningObjective::MinCurvature, &mut |_| {
            true
        })
        .unwrap();

        let centerline = planned
            .computed_centerline
            .expect("the ring had no centerline");
        assert!(centerline.len() > 100);
        // Starts at the start line, heading counterclockwise (+y there).
        assert!((centerline[0].x - 1.5).abs() < 0.05 && centerline[0].y.abs() < 0.05);
        assert!(centerline[3].y > centerline[0].y);

        let margin = config.vehicle_width_m / 2.0 + config.safety_margin_m;
        for point in &planned.race_line {
            let radius = (point.x * point.x + point.y * point.y).sqrt();
            assert!((radius - (2.0 - margin)).abs() < 0.04, "radius {radius}");
        }
        assert!(planned.max_curvature_per_m < planned.reference_max_curvature_per_m);
        assert!(planned.lap_time_s.is_finite() && planned.lap_time_s > 0.0);
    }

    /// On a generated track, the race line stays `margin` inside the walls
    /// everywhere and is no curvier than the centerline it started from.
    #[test]
    fn a_generated_map_s_race_line_stays_on_the_track() {
        let root = std::env::temp_dir().join(format!("aurorus_plan_{}", std::process::id()));
        let generation = GenerationConfig {
            output_root: root.clone(),
            seed: 7,
            ..GenerationConfig::default()
        };
        let generated = generate(&generation, Some("track"), true).unwrap();
        let map = Map::load(&generated.folder).unwrap();
        let config = PlanningConfig::default();

        let planned = plan(&map, &config, PlanningObjective::MinCurvature, &mut |_| {
            true
        })
        .unwrap();
        std::fs::remove_dir_all(&root).ok();

        assert!(planned.computed_centerline.is_none());
        let grid = TrackGrid::build(&map).unwrap();
        // A pixel of slack for the rasterized walls.
        let margin =
            config.vehicle_width_m / 2.0 + config.safety_margin_m - map.info.resolution_m_per_px;
        for point in &planned.race_line {
            for angle in (0..16).map(|i| std::f64::consts::TAU * i as f64 / 16.0) {
                let (x, y) = (
                    point.x + margin * angle.cos(),
                    point.y + margin * angle.sin(),
                );
                assert!(
                    grid.is_track(x, y),
                    "({}, {}) is too close to a wall",
                    point.x,
                    point.y
                );
            }
        }
        assert!(planned.max_curvature_per_m <= planned.reference_max_curvature_per_m);
    }

    /// On a generated track, the minimum-time line beats the
    /// minimum-curvature one (both timed with the same speed profile) and
    /// stays `margin` inside the walls.
    #[test]
    fn a_generated_map_s_min_time_line_is_faster_and_on_the_track() {
        let root = std::env::temp_dir().join(format!("aurorus_min_time_{}", std::process::id()));
        let generation = GenerationConfig {
            output_root: root.clone(),
            seed: 7,
            ..GenerationConfig::default()
        };
        let generated = generate(&generation, Some("track"), true).unwrap();
        let map = Map::load(&generated.folder).unwrap();
        let config = PlanningConfig::default();

        let started = std::time::Instant::now();
        let planned = plan(&map, &config, PlanningObjective::MinTime, &mut |_| true).unwrap();
        std::fs::remove_dir_all(&root).ok();
        let min_time = planned
            .min_time
            .expect("asked for minimum time")
            .expect("minimum time converged");
        eprintln!(
            "min-curvature {:.2} s, min-time {:.2} s, in {:.1} s",
            planned.lap_time_s,
            min_time.lap_time_s,
            started.elapsed().as_secs_f64()
        );

        assert!(min_time.lap_time_s < planned.lap_time_s);
        let grid = TrackGrid::build(&map).unwrap();
        let margin =
            config.vehicle_width_m / 2.0 + config.safety_margin_m - map.info.resolution_m_per_px;
        for point in &min_time.race_line {
            for angle in (0..16).map(|i| std::f64::consts::TAU * i as f64 / 16.0) {
                let (x, y) = (
                    point.x + margin * angle.cos(),
                    point.y + margin * angle.sin(),
                );
                assert!(
                    grid.is_track(x, y),
                    "({}, {}) is too close to a wall",
                    point.x,
                    point.y
                );
            }
        }
    }
}
