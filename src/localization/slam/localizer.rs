//! [`Localizer`]: tracks the vehicle on a known map without ever changing
//! the map, by matching every scan against the map's walls.
//!
//! slam_toolbox's localization mode works differently: it loads a saved pose
//! graph and matches against the scans stored in it (Karto's
//! `ProcessLocalization`). Here the map is just the binary raster every map
//! folder has, so any map can be localized in, generated or saved by SLAM.
//! The search is the same correlative scan matcher mapping uses, with the
//! map's wall pixels as reference points instead of recent scans.

use super::pose::Pose2;
use super::scan::LocalizedScan;
use super::scan_matcher::{MatchParams, ScanMatcher};
use crate::topics::SelectedMap;

/// Everything [`Localizer`] needs to know - the subset of
/// [`crate::localization::SlamConfig`] it uses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalizerParams {
    pub correlation_search_space_dimension_m: f64,
    pub correlation_search_space_resolution_m: f64,
    pub correlation_search_space_smear_deviation_m: f64,
    pub max_laser_range_m: f64,
    /// A match is only trusted at or above this response.
    pub minimum_response: f64,
    pub matching: MatchParams,
}

/// What [`Localizer::process`] did with a scan.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Localized {
    /// The pose the vehicle is now believed to be at, in the map frame.
    pub pose: Pose2,
    /// How well the scan matched the map, from `0` to `1`.
    pub response: f64,
    /// Whether the match was trusted. If not, `pose` is odometry's alone,
    /// carried on from the last trusted match.
    pub trusted: bool,
}

/// The localization state: the map's walls, and where the `odom` frame sits
/// on the map.
pub struct Localizer {
    params: LocalizerParams,
    /// Built on the first scan, once the range threshold (which depends on
    /// the sensor's maximum distance) is known - as in [`super::mapper`].
    matcher: Option<(f64, ScanMatcher)>,
    /// The map's wall points, in the map frame - see [`wall_points`].
    walls: Vec<(f64, f64)>,
    /// The `odom` frame's origin on the map: a pose odometry reports lands
    /// at `map_to_odom.compose(pose)`. Each trusted match corrects it.
    map_to_odom: Pose2,
    /// The latest pose, in the map frame - `None` before the first scan.
    pose: Option<Pose2>,
}

impl Localizer {
    /// Localizes against `walls` (see [`wall_points`]), starting with the
    /// `odom` frame's origin at `map_to_odom`, i.e. where odometry was last
    /// reset.
    pub fn new(params: LocalizerParams, walls: Vec<(f64, f64)>, map_to_odom: Pose2) -> Self {
        Self {
            params,
            matcher: None,
            walls,
            map_to_odom,
            pose: None,
        }
    }

    /// The range threshold a scan from a sensor reaching up to
    /// `max_distance_m` must be built with.
    pub fn range_threshold_m(&self, max_distance_m: f64) -> f64 {
        self.params.max_laser_range_m.min(max_distance_m)
    }

    /// The latest pose, in the map frame.
    pub fn pose(&self) -> Option<Pose2> {
        self.pose
    }

    /// Where the `odom` frame's origin currently sits on the map.
    pub fn map_to_odom(&self) -> Pose2 {
        self.map_to_odom
    }

    /// Places `scan` on the map: odometry's pose, moved by the current
    /// correction, then matched against the walls. A trusted match updates
    /// the correction.
    pub fn process(&mut self, mut scan: LocalizedScan) -> Localized {
        let prior = self.map_to_odom.compose(&scan.odometric_pose);
        scan.set_corrected_pose(prior);

        let threshold_m = scan.range_threshold_m();
        if self
            .matcher
            .as_ref()
            .is_none_or(|(threshold, _)| *threshold != threshold_m)
        {
            self.matcher = Some((
                threshold_m,
                ScanMatcher::new(
                    self.params.correlation_search_space_dimension_m,
                    self.params.correlation_search_space_resolution_m,
                    self.params.correlation_search_space_smear_deviation_m,
                    threshold_m,
                    self.params.matching,
                ),
            ));
        }
        let (_, matcher) = self.matcher.as_mut().expect("built just above");
        let result = matcher.match_points(&scan, &self.walls, true, true);

        let trusted = result.response >= self.params.minimum_response;
        let pose = if trusted {
            self.map_to_odom = result.pose.compose(&scan.odometric_pose.inverse());
            result.pose
        } else {
            prior
        };
        self.pose = Some(pose);
        Localized {
            pose,
            response: result.response,
            trusted,
        }
    }
}

/// The walls of `map`, as the centers of its black pixels next to a white
/// (drivable) one - the surfaces a lidar on the track can hit. Empty if no
/// map is loaded.
pub fn wall_points(map: &SelectedMap) -> Vec<(f64, f64)> {
    let Some(info) = &map.info else {
        return Vec::new();
    };
    let (width, height) = (map.width_px as usize, map.height_px as usize);
    let white = |col: usize, row: usize| map.pixels[row * width + col] == 255;
    let resolution = info.resolution_m_per_px;
    let mut points = Vec::new();
    for row in 0..height {
        for col in 0..width {
            if white(col, row) {
                continue;
            }
            let next_to_white = (col > 0 && white(col - 1, row))
                || (col + 1 < width && white(col + 1, row))
                || (row > 0 && white(col, row - 1))
                || (row + 1 < height && white(col, row + 1));
            if next_to_white {
                points.push((
                    info.origin.x + (col as f64 + 0.5) * resolution,
                    info.origin.y + (row as f64 + 0.5) * resolution,
                ));
            }
        }
    }
    points
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::environment::{ImageOrigin, MapInfo, MapSource, StartFinishLine, WorldPoint};
    use crate::localization::slam::scan_matcher::tests::{params, room_scan};
    use std::time::Instant;

    fn localizer_params() -> LocalizerParams {
        LocalizerParams {
            correlation_search_space_dimension_m: 0.5,
            correlation_search_space_resolution_m: 0.01,
            correlation_search_space_smear_deviation_m: 0.1,
            max_laser_range_m: 12.0,
            minimum_response: 0.3,
            matching: params(),
        }
    }

    /// The room of [`room_scan`] as a map: white inside the room, except
    /// in the pillar.
    fn room_map() -> SelectedMap {
        let resolution = 0.05;
        let (origin_x, origin_y) = (-4.0, -3.0);
        let (width, height) = (180u32, 180u32);
        let mut pixels = Vec::with_capacity((width * height) as usize);
        for row in 0..height {
            for col in 0..width {
                let x = origin_x + (f64::from(col) + 0.5) * resolution;
                let y = origin_y + (f64::from(row) + 0.5) * resolution;
                let inside_room = x > -3.0 && x < 4.0 && y > -2.0 && y < 5.0;
                let in_pillar = (x - 1.5).hypot(y - 2.0) < 0.5;
                pixels.push(if inside_room && !in_pillar { 255 } else { 0 });
            }
        }
        SelectedMap {
            path: None,
            width_px: width,
            height_px: height,
            pixels: pixels.into(),
            info: Some(MapInfo {
                resolution_m_per_px: resolution,
                width_px: width,
                height_px: height,
                origin: ImageOrigin {
                    x: origin_x,
                    y: origin_y,
                    theta_rad: 0.0,
                },
                start_finish_line: StartFinishLine {
                    a: WorldPoint { x: 0.0, y: 0.0 },
                    b: WorldPoint { x: 0.0, y: 0.0 },
                },
                generated_at: String::new(),
                source: MapSource::Real,
                generation: None,
            }),
        }
    }

    #[test]
    fn wall_points_line_the_drivable_area() {
        let walls = wall_points(&room_map());
        assert!(!walls.is_empty());
        for (x, y) in walls {
            // Within a pixel of a wall or of the pillar.
            let to_wall = [
                (x + 3.0).abs(),
                (x - 4.0).abs(),
                (y + 2.0).abs(),
                (y - 5.0).abs(),
            ]
            .into_iter()
            .fold(f64::INFINITY, f64::min);
            let to_pillar = ((x - 1.5).hypot(y - 2.0) - 0.5).abs();
            assert!(to_wall.min(to_pillar) < 0.08, "({x}, {y})");
        }
        assert!(wall_points(&SelectedMap::default()).is_empty());
    }

    #[test]
    fn a_drifting_odometry_is_corrected_onto_the_map() {
        let map = room_map();
        // Odometry was reset at (0.5, 0, 0.1) on the map.
        let start = Pose2::new(0.5, 0.0, 0.1);
        let mut localizer = Localizer::new(localizer_params(), wall_points(&map), start);

        // The vehicle drives a curve; odometry underestimates the distance
        // and the turn, drifting away from the truth.
        let mut truth = start;
        let mut odometry = Pose2::default();
        for _ in 0..30 {
            truth = truth.compose(&Pose2::new(0.1, 0.0, 0.03));
            odometry = odometry.compose(&Pose2::new(0.085, 0.01, 0.02));
            let scan = LocalizedScan::new(&room_scan(truth, 360), Instant::now(), odometry, 12.0);
            let localized = localizer.process(scan);
            assert!(localized.trusted, "response {}", localized.response);
        }

        let pose = localizer.pose().unwrap();
        let naive = start.compose(&odometry);
        assert!(
            naive.squared_distance(&truth).sqrt() > 0.3,
            "odometry must drift"
        );
        assert!(
            pose.squared_distance(&truth).sqrt() < 0.03,
            "localized at {pose:?}, truth {truth:?}"
        );
        assert!(
            Pose2::new(0.0, 0.0, pose.heading_rad - truth.heading_rad)
                .heading_rad
                .abs()
                < 0.01
        );
        let odom_on_map = localizer.map_to_odom().compose(&odometry);
        assert!(odom_on_map.squared_distance(&pose) < 1e-12);
    }

    #[test]
    fn an_untrusted_match_follows_odometry() {
        let mut localizer =
            Localizer::new(localizer_params(), Vec::new(), Pose2::new(1.0, 0.0, 0.0));
        let odometry = Pose2::new(0.5, 0.0, 0.0);
        let scan = LocalizedScan::new(&room_scan(odometry, 360), Instant::now(), odometry, 12.0);

        let localized = localizer.process(scan);

        assert!(!localized.trusted);
        assert!(localized.pose.squared_distance(&Pose2::new(1.5, 0.0, 0.0)) < 1e-12);
        assert_eq!(localizer.map_to_odom(), Pose2::new(1.0, 0.0, 0.0));
    }
}
