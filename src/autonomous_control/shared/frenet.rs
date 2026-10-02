//! The Frenet overtaking planner, used by `frenet_overtaking`: samples
//! paths that leave the vehicle's current offset from the race line for a range of end
//! offsets and lengths, keeps those staying on the map's free space (shrunk
//! away from the walls, see [`FreeGrid`]) and clear of the obstacles, and
//! picks the cheapest.
//!
//! Design choices:
//! - a path starts with the vehicle's slope relative to the line, not
//!   flat, so replanning doesn't kink the path it's on (see [`LateralCubic`]);
//! - the path field-of-view limit applies to both sides;
//! - when no full-length path is clear of the obstacles, [`Plan::reach_m`]
//!   says how far the most open one gets, so the caller can keep the speed
//!   it can still stop from (see [`stopping_speed`]);
//! - the look-ahead target is the first path point at least the lookahead
//!   distance along it, and the curvature slowdown doesn't compound across
//!   ticks;
//! - when no path is free, the fallback keeps clear of the obstacles if any
//!   path does (only leaving the shrunk free space), else gets farthest
//!   before its first one.

use crate::autonomous_control::shared::race_line::{Line, wrap_to_pi};
use crate::planning::track::squared_distance_transform;
use crate::topics::SelectedMap;

/// The map's free space, eroded: white pixels farther than a clearance
/// from every non-white one - so a LIDAR hit on a wall is never mistaken for
/// an obstacle, and a path never hugs a wall.
pub(crate) struct FreeGrid {
    width_px: usize,
    height_px: usize,
    origin_x_m: f64,
    origin_y_m: f64,
    resolution_m_per_px: f64,
    /// Row-major, like [`SelectedMap::pixels`].
    free: Vec<bool>,
}

impl FreeGrid {
    /// `map`'s free space shrunk by `clearance_m`, or `None` if no map is
    /// loaded (or its pixels don't match its size).
    pub(crate) fn new(map: &SelectedMap, clearance_m: f64) -> Option<Self> {
        let info = map.info.as_ref()?;
        let (width_px, height_px) = (map.width_px as usize, map.height_px as usize);
        if width_px == 0 || height_px == 0 || map.pixels.len() != width_px * height_px {
            return None;
        }
        let wall: Vec<bool> = map.pixels.iter().map(|&pixel| pixel != 255).collect();
        let squared_px = squared_distance_transform(&wall, width_px, height_px);
        let resolution = info.resolution_m_per_px;
        let clearance_px2 = (clearance_m.max(0.0) / resolution).powi(2);
        let free = squared_px
            .iter()
            .zip(&wall)
            .map(|(&d2, &wall)| !wall && d2 > clearance_px2)
            .collect();
        Some(Self {
            width_px,
            height_px,
            origin_x_m: info.origin.x,
            origin_y_m: info.origin.y,
            resolution_m_per_px: resolution,
            free,
        })
    }

    /// Whether `(x_m, y_m)` is free - never outside the map. Indexed like
    /// the simulated LIDAR's ray casting.
    pub(crate) fn is_free(&self, x_m: f64, y_m: f64) -> bool {
        let col = ((x_m - self.origin_x_m) / self.resolution_m_per_px).floor();
        let row = ((y_m - self.origin_y_m) / self.resolution_m_per_px).floor();
        col >= 0.0
            && row >= 0.0
            && col < self.width_px as f64
            && row < self.height_px as f64
            && self.free[row as usize * self.width_px + col as usize]
    }
}

/// A lateral offset profile `d(u)` over `u` in `[0, length]` meters of
/// line: from `d0` with slope `slope0` to `d1` with slope 0 - a cubic
/// Hermite.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct LateralCubic {
    a: [f64; 4],
}

impl LateralCubic {
    pub(crate) fn new(d0: f64, slope0: f64, d1: f64, length_m: f64) -> Self {
        let l = length_m;
        let rise = d1 - d0 - slope0 * l;
        Self {
            a: [
                d0,
                slope0,
                slope0 / l + 3.0 * rise / (l * l),
                -(2.0 * rise + slope0 * l) / (l * l * l),
            ],
        }
    }

    pub(crate) fn at(&self, u: f64) -> f64 {
        let [a0, a1, a2, a3] = self.a;
        a0 + u * (a1 + u * (a2 + u * a3))
    }

    #[cfg(test)]
    pub(crate) fn slope(&self, u: f64) -> f64 {
        let [_, a1, a2, a3] = self.a;
        a1 + u * (2.0 * a2 + 3.0 * u * a3)
    }

    /// The (constant) third derivative - the jerk the cost penalizes.
    pub(crate) fn third_derivative(&self) -> f64 {
        6.0 * self.a[3]
    }
}

/// Everything [`Planner::plan`] is tuned by.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct FrenetParams {
    /// End offsets are sampled in `[-max_road_width_m, max_road_width_m]`...
    pub(crate) max_road_width_m: f64,
    /// ...every `delta_road_width_m` (> 0).
    pub(crate) delta_road_width_m: f64,
    /// Spacing of a path's points along the line (> 0).
    pub(crate) path_point_distance_m: f64,
    /// Path lengths are sampled from `min_path_length_m` (> 0) to
    /// `max_path_length_m`, every `delta_path_length_m` (> 0).
    pub(crate) min_path_length_m: f64,
    pub(crate) max_path_length_m: f64,
    pub(crate) delta_path_length_m: f64,
    /// A path whose end is at a steeper angle than this from its start is
    /// not sampled.
    pub(crate) path_fov_rad: f64,
    /// A path point closer than this to an obstacle collides.
    pub(crate) robot_radius_m: f64,
    pub(crate) k_jerk: f64,
    pub(crate) k_length: f64,
    pub(crate) k_distance: f64,
    /// Each tick every path is blocked, the speed gain is multiplied by this.
    pub(crate) speed_decay_factor: f64,
    /// The speed gain never drops below this.
    pub(crate) min_speed_reduction_gain: f64,
    /// The remembered end offset decays by this every tick...
    pub(crate) decay_last_d_factor: f64,
    /// ...and moves this far toward the chosen path's end offset.
    pub(crate) weight_last_d: f64,
    /// Exponent of the curvature slowdown (line curvature over path curvature).
    pub(crate) speed_curvature_exponent: f64,
}

/// One sampled path, along the line and in the world.
#[derive(Debug, Clone, Default, PartialEq)]
pub(crate) struct FrenetPath {
    /// Lateral offset from the line, positive toward increasing
    /// heading (see [`Line::lateral`]), per point.
    pub(crate) d: Vec<f64>,
    /// Arc length along the line, per point (not wrapped to the lap).
    pub(crate) s: Vec<f64>,
    pub(crate) x: Vec<f64>,
    pub(crate) y: Vec<f64>,
    /// Distance from each point to the next (the last repeats the one before).
    pub(crate) ds: Vec<f64>,
    /// Signed curvature at each point.
    pub(crate) curvature: Vec<f64>,
    pub(crate) cost: f64,
}

impl FrenetPath {
    /// Arc length along the path from its start to the first point that
    /// collides with an obstacle, if any. The start point is skipped: it's
    /// where the vehicle already is.
    fn first_collision_m(&self, obstacles: &[[f64; 2]], radius_m: f64) -> Option<f64> {
        let radius2 = radius_m * radius_m;
        let mut along_m = 0.0;
        for i in 1..self.x.len() {
            along_m += self.ds[i - 1];
            let hit = obstacles.iter().any(|&[ox, oy]| {
                let (dx, dy) = (self.x[i] - ox, self.y[i] - oy);
                dx * dx + dy * dy <= radius2
            });
            if hit {
                return Some(along_m);
            }
        }
        None
    }

    /// Arc length along the line from its first point to its last.
    fn length_along_line(&self) -> f64 {
        self.s[self.s.len() - 1] - self.s[0]
    }

    /// Arc length along the path itself.
    fn length_m(&self) -> f64 {
        self.ds[..self.ds.len() - 1].iter().sum()
    }

    /// Whether every point but the start is on `grid`'s free space.
    fn is_on(&self, grid: &FreeGrid) -> bool {
        (1..self.x.len()).all(|i| grid.is_free(self.x[i], self.y[i]))
    }
}

/// What [`Planner::plan`] decided.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Plan {
    /// Every sampled path, with its world points.
    pub(crate) candidates: Vec<FrenetPath>,
    /// Index into `candidates` of the path to follow.
    pub(crate) best: usize,
    /// Whether `best` is free - else it's the fallback: the cheapest clear
    /// of the obstacles (though too close to a wall), else the one getting
    /// farthest before its first obstacle.
    pub(crate) free: bool,
    /// The point on `best` to steer toward.
    pub(crate) target: [f64; 2],
    /// Multiplies the reference speed, in `[min_speed_reduction_gain, 1]`.
    pub(crate) speed_gain: f64,
    /// `None` if some path of the longest sampled length is clear of every
    /// obstacle; else how far along any path the vehicle gets before its
    /// first obstacle, at most (a clear shorter path counts as its length).
    pub(crate) reach_m: Option<f64>,
}

impl Plan {
    pub(crate) fn best(&self) -> &FrenetPath {
        &self.candidates[self.best]
    }
}

/// What the planner remembers between ticks - reset whenever it's taken
/// back into use.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Planner {
    /// A decaying average of the chosen end offsets, which
    /// the cost pulls the next choice toward - so the vehicle commits to
    /// one side of an obstacle.
    d_weight: f64,
    /// Decays each tick every path is blocked, back to 1 once one is free.
    blocked_gain: f64,
}

impl Default for Planner {
    fn default() -> Self {
        Self {
            d_weight: 0.0,
            blocked_gain: 1.0,
        }
    }
}

/// `start`, `start + step`, ... up to `end` (inclusive, with a little
/// tolerance) - at least `start`.
fn samples(start: f64, end: f64, step: f64) -> impl Iterator<Item = f64> {
    let count = if step > 0.0 && end > start {
        ((end - start) / step + 1e-6).floor() as usize
    } else {
        0
    };
    (0..=count).map(move |k| start + k as f64 * step)
}

impl Planner {
    /// Plans from arc length `s0` on `line`, at offset `d0` and slope
    /// `slope0` (see [`Line::lateral`]), around `obstacles` (world points),
    /// within `grid`'s free space, steering `lookahead_m` along the chosen
    /// path.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn plan(
        &mut self,
        params: &FrenetParams,
        line: &Line,
        grid: &FreeGrid,
        obstacles: &[[f64; 2]],
        s0: f64,
        d0: f64,
        slope0: f64,
        lookahead_m: f64,
    ) -> Plan {
        let mut candidates = self.candidates(params, s0, d0, slope0, true);
        if candidates.is_empty() {
            // Too far off to reach any end offset within the field of view:
            // better any path than none.
            candidates = self.candidates(params, s0, d0, slope0, false);
        }
        for path in &mut candidates {
            to_world(path, line);
        }

        let cheapest = |indices: &mut dyn Iterator<Item = usize>| {
            indices.min_by(|&a, &b| candidates[a].cost.total_cmp(&candidates[b].cost))
        };
        let collisions: Vec<Option<f64>> = candidates
            .iter()
            .map(|path| path.first_collision_m(obstacles, params.robot_radius_m))
            .collect();
        let free = cheapest(
            &mut (0..candidates.len())
                .filter(|&i| collisions[i].is_none() && candidates[i].is_on(grid)),
        );
        let best = match free {
            Some(best) => {
                self.blocked_gain = 1.0;
                best
            }
            None => {
                self.blocked_gain *= params.speed_decay_factor;
                // Better closer to a wall than into an obstacle; else the
                // path getting farthest before its first one.
                cheapest(&mut (0..candidates.len()).filter(|&i| collisions[i].is_none()))
                    .or_else(|| {
                        (0..candidates.len()).max_by(|&a, &b| {
                            let reached = |i: usize| collisions[i].unwrap_or(f64::INFINITY);
                            reached(a)
                                .total_cmp(&reached(b))
                                .then(candidates[b].cost.total_cmp(&candidates[a].cost))
                        })
                    })
                    .expect("at least one path is always sampled")
            }
        };
        let reach_m = reach(&candidates, obstacles, params.robot_radius_m);
        let path = &candidates[best];

        let end_d = *path.d.last().expect("a path has points");
        self.d_weight = params.decay_last_d_factor
            * ((1.0 - params.weight_last_d) * self.d_weight + params.weight_last_d * end_d);

        // The first point at least `lookahead_m` along the path, else its end.
        let mut along_m = 0.0;
        let mut target = path.x.len() - 1;
        for i in 1..path.x.len() {
            along_m += path.ds[i - 1];
            if along_m >= lookahead_m {
                target = i;
                break;
            }
        }

        // Slower where the path curves more than the line does there.
        let epsilon = f64::from(f32::EPSILON);
        let path_curvature = path.curvature[target].abs().max(epsilon);
        let line_curvature = line.curvature_at(path.s[target]).abs().max(epsilon);
        let curvature_factor = (line_curvature / path_curvature)
            .powf(params.speed_curvature_exponent)
            .min(1.0);
        let speed_gain = (self.blocked_gain * curvature_factor)
            .clamp(params.min_speed_reduction_gain.min(1.0), 1.0);

        Plan {
            target: [path.x[target], path.y[target]],
            best,
            free: free.is_some(),
            speed_gain,
            reach_m,
            candidates,
        }
    }

    /// The paths along the line (no world points yet): every end offset
    /// and length, those out of the field of view skipped if `within_fov`.
    fn candidates(
        &self,
        params: &FrenetParams,
        s0: f64,
        d0: f64,
        slope0: f64,
        within_fov: bool,
    ) -> Vec<FrenetPath> {
        let width = params.max_road_width_m.max(0.0);
        let mut paths = Vec::new();
        for d1 in samples(-width, width, params.delta_road_width_m) {
            for length_m in samples(
                params.min_path_length_m,
                params.max_path_length_m,
                params.delta_path_length_m,
            ) {
                if within_fov && (d1 - d0).atan2(length_m).abs() > params.path_fov_rad {
                    continue;
                }
                let cubic = LateralCubic::new(d0, slope0, d1, length_m);
                // At least 3 points, for the curvature.
                let step = params.path_point_distance_m.min(length_m / 2.0);
                let (s, d): (Vec<f64>, Vec<f64>) = samples(0.0, length_m, step)
                    .map(|u| (s0 + u, cubic.at(u)))
                    .unzip();
                let jerk = cubic.third_derivative().powi(2) * d.len() as f64;
                let cost = params.k_jerk * jerk
                    + params.k_length / length_m
                    + params.k_distance * (d[d.len() - 1] - self.d_weight).powi(2);
                paths.push(FrenetPath {
                    d,
                    s,
                    cost,
                    ..Default::default()
                });
            }
        }
        paths
    }
}

/// See [`Plan::reach_m`].
fn reach(candidates: &[FrenetPath], obstacles: &[[f64; 2]], radius_m: f64) -> Option<f64> {
    let longest_s = candidates
        .iter()
        .map(FrenetPath::length_along_line)
        .fold(0.0, f64::max);
    let mut reach_m: f64 = 0.0;
    for path in candidates {
        match path.first_collision_m(obstacles, radius_m) {
            Some(along_m) => reach_m = reach_m.max(along_m),
            None if path.length_along_line() >= longest_s - 1e-9 => return None,
            None => reach_m = reach_m.max(path.length_m()),
        }
    }
    Some(reach_m)
}

/// Fills in `path`'s world points, spacing and curvature, from its offsets
/// along `line`.
fn to_world(path: &mut FrenetPath, line: &Line) {
    let n = path.s.len();
    (path.x, path.y) = path
        .s
        .iter()
        .zip(&path.d)
        .map(|(&s, &d)| {
            let base = line.at(s);
            let (sin, cos) = line.heading_at(s).sin_cos();
            (base.x - d * sin, base.y + d * cos)
        })
        .unzip();
    let yaw: Vec<f64> = (0..n - 1)
        .map(|i| (path.y[i + 1] - path.y[i]).atan2(path.x[i + 1] - path.x[i]))
        .collect();
    path.ds = (0..n - 1)
        .map(|i| (path.x[i + 1] - path.x[i]).hypot(path.y[i + 1] - path.y[i]))
        .collect();
    path.ds.push(path.ds[n - 2]);
    // At each interior point, the turn between the segments either side of
    // it over their mean length; the ends repeat their neighbors'.
    path.curvature = vec![0.0; n];
    for i in 1..n - 1 {
        let mean_m = (path.ds[i - 1] + path.ds[i]) / 2.0;
        path.curvature[i] = if mean_m > 0.0 {
            wrap_to_pi(yaw[i] - yaw[i - 1]) / mean_m
        } else {
            0.0
        };
    }
    path.curvature[0] = path.curvature[1];
    path.curvature[n - 1] = path.curvature[n - 2];
}

/// The fastest speed from which the vehicle can still stop, braking at
/// `max_decel_mps2`, `margin_m` short of an obstacle `distance_m` ahead.
pub(crate) fn stopping_speed(distance_m: f64, margin_m: f64, max_decel_mps2: f64) -> f64 {
    (2.0 * max_decel_mps2.max(0.0) * (distance_m - margin_m).max(0.0)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::environment::SpeedPoint;
    use crate::planning::track::tests::ring_map;

    pub(crate) fn params() -> FrenetParams {
        FrenetParams {
            max_road_width_m: 1.0,
            delta_road_width_m: 0.1,
            path_point_distance_m: 0.2,
            min_path_length_m: 1.25,
            max_path_length_m: 4.35,
            delta_path_length_m: 1.5,
            path_fov_rad: 30f64.to_radians(),
            robot_radius_m: 0.35,
            k_jerk: 0.01,
            k_length: 0.1,
            k_distance: 0.01,
            speed_decay_factor: 0.999,
            min_speed_reduction_gain: 0.7,
            decay_last_d_factor: 0.99,
            weight_last_d: 0.4,
            speed_curvature_exponent: 0.3,
        }
    }

    /// A straight 40 m x 6 m loop along y = 0 (out) and y = 6 (back).
    fn loop_line() -> Line {
        let point = |x: f64, y: f64| SpeedPoint {
            x,
            y,
            speed_mps: 3.0,
        };
        let mut points: Vec<SpeedPoint> = (0..=200).map(|i| point(i as f64 * 0.2, 0.0)).collect();
        points.extend((0..=200).rev().map(|i| point(i as f64 * 0.2, 6.0)));
        Line::new(points).unwrap()
    }

    /// Free everywhere within 1.5 m of the straight y = 0 (and far from the
    /// other one).
    fn corridor_grid() -> FreeGrid {
        let resolution = 0.05;
        let (width, height) = (800usize, 60usize);
        let (ox, oy) = (0.0, -1.5);
        FreeGrid {
            width_px: width,
            height_px: height,
            origin_x_m: ox,
            origin_y_m: oy,
            resolution_m_per_px: resolution,
            free: vec![true; width * height],
        }
    }

    #[test]
    fn the_cubic_meets_its_end_conditions() {
        let cubic = LateralCubic::new(0.3, 0.2, -0.5, 2.5);
        assert!((cubic.at(0.0) - 0.3).abs() < 1e-12);
        assert!((cubic.slope(0.0) - 0.2).abs() < 1e-12);
        assert!((cubic.at(2.5) + 0.5).abs() < 1e-12);
        assert!(cubic.slope(2.5).abs() < 1e-12);
    }

    #[test]
    fn the_path_field_of_view_is_symmetric() {
        let planner = Planner::default();
        let paths = planner.candidates(&params(), 0.0, 0.0, 0.0, true);
        let ends: Vec<f64> = paths.iter().map(|p| *p.d.last().unwrap()).collect();
        let (lowest, highest) = ends
            .iter()
            .fold((f64::MAX, f64::MIN), |(lo, hi), &d| (lo.min(d), hi.max(d)));
        assert!((lowest + highest).abs() < 1e-9, "{lowest} vs {highest}");
        // The shortest path can't swerve a full meter within 30 degrees.
        assert!(paths.iter().all(|p| {
            let length = p.s.last().unwrap() - p.s[0];
            length > 2.0 || p.d.last().unwrap().abs() <= 1.25 * 30f64.to_radians().tan() + 1e-9
        }));
    }

    #[test]
    fn free_space_is_eroded_away_from_the_walls() {
        let map = ring_map(2.0, 4.0, 0.05);
        let selected = SelectedMap {
            path: None,
            width_px: map.info.width_px,
            height_px: map.info.height_px,
            pixels: map.raster.to_bytes().into(),
            info: Some(map.info.clone()),
        };
        let grid = FreeGrid::new(&selected, 0.3).unwrap();
        assert!(grid.is_free(3.0, 0.0));
        assert!(!grid.is_free(2.15, 0.0), "too close to the inner wall");
        assert!(!grid.is_free(3.85, 0.0), "too close to the outer wall");
        assert!(!grid.is_free(0.0, 0.0), "the island");
        assert!(!grid.is_free(100.0, 0.0), "off the map");
        let unshrunk = FreeGrid::new(&selected, 0.0).unwrap();
        assert!(unshrunk.is_free(2.15, 0.0));
    }

    #[test]
    fn with_nothing_in_the_way_it_stays_on_the_line() {
        let line = loop_line();
        let plan =
            Planner::default().plan(&params(), &line, &corridor_grid(), &[], 5.0, 0.0, 0.0, 1.0);
        assert!(plan.free);
        assert!(plan.best().d.iter().all(|d| d.abs() < 1e-9));
        assert!(
            (plan.target[0] - 6.0).abs() < 0.21 && plan.target[1].abs() < 1e-9,
            "{:?}",
            plan.target
        );
        assert_eq!(plan.speed_gain, 1.0);
    }

    #[test]
    fn it_swerves_around_an_obstacle_on_the_line() {
        let line = loop_line();
        let grid = corridor_grid();
        let obstacles = [[8.5, 0.0], [8.5, 0.1], [8.5, -0.1]];
        let mut planner = Planner::default();
        let plan = planner.plan(&params(), &line, &grid, &obstacles, 5.0, 0.0, 0.0, 1.0);
        assert!(plan.free);
        assert_eq!(plan.reach_m, None, "a full-length path gets around it");
        let best = plan.best();
        for (x, y) in best.x.iter().zip(&best.y) {
            assert!(grid.is_free(*x, *y));
            for [ox, oy] in obstacles {
                assert!((x - ox).hypot(y - oy) > params().robot_radius_m);
            }
        }
        assert!(best.d.last().unwrap().abs() > 0.3, "{:?}", best.d);
        assert!(plan.speed_gain < 1.0 && plan.speed_gain >= params().min_speed_reduction_gain);
    }

    #[test]
    fn a_wall_of_obstacles_limits_the_reach() {
        let line = loop_line();
        let obstacles: Vec<[f64; 2]> = (-30..=30).map(|i| [8.0, i as f64 * 0.05]).collect();
        let mut planner = Planner::default();
        let plan = planner.plan(
            &params(),
            &line,
            &corridor_grid(),
            &obstacles,
            5.0,
            0.0,
            0.0,
            1.0,
        );
        let blocked = plan
            .reach_m
            .expect("every full-length path runs into the wall");
        assert!(blocked > 2.0 && blocked < 3.5, "{blocked}");
        assert!(stopping_speed(blocked, 0.5, 4.0) < stopping_speed(blocked + 1.0, 0.5, 4.0));
        assert_eq!(stopping_speed(0.3, 0.5, 4.0), 0.0);
    }

    #[test]
    fn without_a_free_path_it_still_keeps_clear_of_the_obstacles() {
        let line = loop_line();
        // Free space only within 0.3 m of the line: every swerve leaves it.
        let mut grid = corridor_grid();
        for (i, free) in grid.free.iter_mut().enumerate() {
            let y = grid.origin_y_m + ((i / grid.width_px) as f64 + 0.5) * grid.resolution_m_per_px;
            *free = y.abs() < 0.3;
        }
        let obstacles = [[6.0, 0.0], [6.0, 0.1], [6.0, -0.1]];
        let plan = Planner::default().plan(&params(), &line, &grid, &obstacles, 5.0, 0.0, 0.0, 1.0);
        assert!(!plan.free);
        let best = plan.best();
        assert_eq!(
            best.first_collision_m(&obstacles, params().robot_radius_m),
            None
        );
    }

    #[test]
    fn a_path_starts_along_the_vehicle_slope() {
        let line = loop_line();
        let plan =
            Planner::default().plan(&params(), &line, &corridor_grid(), &[], 5.0, 0.2, 0.3, 1.0);
        let best = plan.best();
        let initial = (best.d[1] - best.d[0]) / (best.s[1] - best.s[0]);
        assert!((initial - 0.3).abs() < 0.1, "{initial}");
    }
}
