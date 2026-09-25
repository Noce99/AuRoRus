//! [`export`]: turns the map [`super::Mapper`] built into a binary map in
//! the repo's own format ([`MapInfo`] + [`Raster`]), for
//! [`crate::environment::save`] to write under `maps/`. It plays the role of
//! slam_toolbox's `map_saver`, which shells out to nav2's map saver instead.
//!
//! Two things happen on the way:
//!
//! - **The map is cleaned** ([`drivable_area`]). SLAM's free / occupied /
//!   unknown map becomes white (drivable) / black. Only free space the
//!   vehicle can reach from its trajectory is white. Free specks outside the
//!   walls, and areas beams leaked into through holes in a one-cell wall,
//!   aren't reachable, so they turn black.
//! - **The start/finish line is placed** ([`start_finish_line`]) through
//!   where mapping started, across the track, as perpendicular to both of its
//!   borders as possible. A SLAM map has no centerline to derive it from.

use super::pose::Pose2;
use crate::environment::{
    ImageOrigin, MapInfo, MapSource, Raster, StartFinishLine, WorldPoint, now_rfc3339,
};
use crate::topics::SlamMap;
use std::collections::VecDeque;
use std::f64::consts::PI;

/// Black border kept around the drivable area when cropping, in pixels, so
/// the track never touches the image's edge. A ray leaving the image hits
/// nothing (see `SimulatedLidar`), so a wall there would leak.
const CROP_MARGIN_PX: i64 = 10;
/// How many directions the start/finish line is tried in, over half a turn
/// (1 degree apart).
const START_LINE_DIRECTIONS: usize = 180;
/// Farthest a track border may be from the start position, in meters.
const START_LINE_MAX_HALF_LENGTH_M: f64 = 10.0;
/// Only lines at most this many times as long as the shortest one through
/// the start are candidates for the start/finish line.
const START_LINE_MAX_LENGTH_RATIO: f64 = 1.5;
/// Radius around each end of the start/finish line whose border pixels
/// give that border's direction, in meters.
const BORDER_FIT_RADIUS_M: f64 = 0.3;

/// Why [`export`] failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExportError {
    /// No scan was taken yet.
    Empty,
    /// No free cell is reachable from the trajectory.
    NoDrivableArea,
    /// Mapping started somewhere not drivable, or no line through it spans
    /// the track.
    NoStartFinishLine,
}

impl std::fmt::Display for ExportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Empty => write!(f, "the map is empty"),
            Self::NoDrivableArea => write!(f, "no drivable area around the trajectory"),
            Self::NoStartFinishLine => {
                write!(
                    f,
                    "no start/finish line across the track where mapping started"
                )
            }
        }
    }
}

impl std::error::Error for ExportError {}

/// `map` as a binary map whose start/finish line runs through `start` (the
/// pose mapping started from), in SLAM's own frame.
pub fn export(map: &SlamMap, start: Pose2) -> Result<(MapInfo, Raster), ExportError> {
    if map.width_px == 0 || map.trajectory.is_empty() {
        return Err(ExportError::Empty);
    }
    let grid = drivable_area(map).ok_or(ExportError::NoDrivableArea)?;
    let line = start_finish_line(&grid, start).ok_or(ExportError::NoStartFinishLine)?;
    let info = MapInfo {
        resolution_m_per_px: grid.resolution_m,
        width_px: grid.width,
        height_px: grid.height,
        origin: ImageOrigin {
            x: grid.origin_x_m,
            y: grid.origin_y_m,
            theta_rad: 0.0,
        },
        start_finish_line: line,
        generated_at: now_rfc3339(),
        source: MapSource::Real,
        generation: None,
    };
    let raster = Raster::new(grid.width, grid.height, grid.white);
    Ok((info, raster))
}

/// A binary map: white (drivable) or black cells, pixel `(0, 0)`'s corner
/// at the origin, rows growing along `+y`.
#[derive(Debug)]
struct BinaryGrid {
    resolution_m: f64,
    origin_x_m: f64,
    origin_y_m: f64,
    width: u32,
    height: u32,
    white: Vec<bool>,
}

impl BinaryGrid {
    /// The cell under `(x_m, y_m)`, if inside the grid.
    fn cell(&self, x_m: f64, y_m: f64) -> Option<(u32, u32)> {
        let col = ((x_m - self.origin_x_m) / self.resolution_m).floor();
        let row = ((y_m - self.origin_y_m) / self.resolution_m).floor();
        (col >= 0.0 && row >= 0.0 && col < f64::from(self.width) && row < f64::from(self.height))
            .then_some((col as u32, row as u32))
    }

    /// Whether `(x_m, y_m)` is drivable - never outside the grid.
    fn is_white(&self, x_m: f64, y_m: f64) -> bool {
        self.cell(x_m, y_m)
            .is_some_and(|(col, row)| self.white[self.index(col, row)])
    }

    fn index(&self, col: u32, row: u32) -> usize {
        row as usize * self.width as usize + col as usize
    }

    /// The center of cell `(col, row)`, in meters.
    fn center(&self, col: u32, row: u32) -> (f64, f64) {
        (
            self.origin_x_m + (f64::from(col) + 0.5) * self.resolution_m,
            self.origin_y_m + (f64::from(row) + 0.5) * self.resolution_m,
        )
    }

    /// Whether `(col, row)` is a black cell next to a white one - on a
    /// track border.
    fn is_border(&self, col: u32, row: u32) -> bool {
        if self.white[self.index(col, row)] {
            return false;
        }
        neighbors4(col, row, self.width, self.height).any(|(c, r)| self.white[self.index(c, r)])
    }
}

/// The free cells of `map` reachable from its trajectory, cropped to them
/// plus [`CROP_MARGIN_PX`] - `None` if there are none.
///
/// Walls are one cell thick, and beams slip through their diagonal gaps.
/// So the flood fill doesn't cross cells next to a wall (8-neighborhood),
/// which plugs any one-cell hole, and then grows back by one cell into free
/// cells, so the drivable area still reaches the walls.
fn drivable_area(map: &SlamMap) -> Option<BinaryGrid> {
    let (width, height) = (map.width_px, map.height_px);
    let index = |col: u32, row: u32| row as usize * width as usize + col as usize;
    let free = |col: u32, row: u32| map.pixels[index(col, row)] == SlamMap::FREE;
    let passable = |col: u32, row: u32| {
        free(col, row)
            && !neighbors8(col, row, width, height)
                .any(|(c, r)| map.pixels[index(c, r)] == SlamMap::OCCUPIED)
    };

    let mut reached = vec![false; map.pixels.len()];
    let mut queue = VecDeque::new();
    for &[x, y] in &map.trajectory {
        let col = ((f64::from(x) - map.origin_x_m) / map.resolution_m_per_px).floor();
        let row = ((f64::from(y) - map.origin_y_m) / map.resolution_m_per_px).floor();
        if col < 0.0 || row < 0.0 || col >= f64::from(width) || row >= f64::from(height) {
            continue;
        }
        let (col, row) = (col as u32, row as u32);
        if passable(col, row) && !reached[index(col, row)] {
            reached[index(col, row)] = true;
            queue.push_back((col, row));
        }
    }
    while let Some((col, row)) = queue.pop_front() {
        for (c, r) in neighbors4(col, row, width, height) {
            if !reached[index(c, r)] && passable(c, r) {
                reached[index(c, r)] = true;
                queue.push_back((c, r));
            }
        }
    }

    let white: Vec<bool> = (0..height)
        .flat_map(|row| (0..width).map(move |col| (col, row)))
        .map(|(col, row)| {
            reached[index(col, row)]
                || (free(col, row)
                    && neighbors8(col, row, width, height).any(|(c, r)| reached[index(c, r)]))
        })
        .collect();

    // Crop to the white cells, plus a black margin.
    let mut min = (i64::MAX, i64::MAX);
    let mut max = (i64::MIN, i64::MIN);
    for row in 0..height {
        for col in 0..width {
            if white[index(col, row)] {
                min = (min.0.min(i64::from(col)), min.1.min(i64::from(row)));
                max = (max.0.max(i64::from(col)), max.1.max(i64::from(row)));
            }
        }
    }
    if min.0 > max.0 {
        return None;
    }
    let (first_col, first_row) = (min.0 - CROP_MARGIN_PX, min.1 - CROP_MARGIN_PX);
    let cropped_width = (max.0 - min.0 + 1 + 2 * CROP_MARGIN_PX) as u32;
    let cropped_height = (max.1 - min.1 + 1 + 2 * CROP_MARGIN_PX) as u32;
    let mut cropped = vec![false; cropped_width as usize * cropped_height as usize];
    for row in 0..cropped_height {
        for col in 0..cropped_width {
            let (source_col, source_row) = (first_col + i64::from(col), first_row + i64::from(row));
            let inside = source_col >= 0
                && source_row >= 0
                && source_col < i64::from(width)
                && source_row < i64::from(height);
            cropped[row as usize * cropped_width as usize + col as usize] =
                inside && white[index(source_col as u32, source_row as u32)];
        }
    }
    Some(BinaryGrid {
        resolution_m: map.resolution_m_per_px,
        origin_x_m: map.origin_x_m + first_col as f64 * map.resolution_m_per_px,
        origin_y_m: map.origin_y_m + first_row as f64 * map.resolution_m_per_px,
        width: cropped_width,
        height: cropped_height,
        white: cropped,
    })
}

/// The start/finish line through `start`'s position: of the lines through
/// it spanning the track (from border to border, and not much longer than
/// the shortest such line), the one closest to perpendicular to both
/// borders. Ordered so the direction of travel it
/// carries (see [`StartFinishLine`]) is the side `start` heads towards.
/// `None` if `start` isn't drivable, or no line spans the track.
fn start_finish_line(grid: &BinaryGrid, start: Pose2) -> Option<StartFinishLine> {
    let (x, y) = (start.x_m, start.y_m);
    if !grid.is_white(x, y) {
        return None;
    }

    // (length, score, direction, one end, the other end) per direction.
    let mut candidates = Vec::new();
    for k in 0..START_LINE_DIRECTIONS {
        let angle = PI * k as f64 / START_LINE_DIRECTIONS as f64;
        let (uy, ux) = angle.sin_cos();
        let (Some(backward), Some(forward)) = (
            border_distance(grid, x, y, -ux, -uy),
            border_distance(grid, x, y, ux, uy),
        ) else {
            continue;
        };
        let end_a = (x - ux * backward, y - uy * backward);
        let end_b = (x + ux * forward, y + uy * forward);
        let (Some(border_a), Some(border_b)) =
            (border_direction(grid, end_a), border_direction(grid, end_b))
        else {
            continue;
        };
        // 0 when perpendicular to both borders.
        let score =
            (ux * border_a.0 + uy * border_a.1).abs() + (ux * border_b.0 + uy * border_b.1).abs();
        candidates.push((backward + forward, score, (ux, uy), end_a, end_b));
    }
    // A line along a straight can meet walls far ahead (the outside of the
    // next corner) square on too: only lines about as short as the
    // shortest one actually cross the track.
    let shortest = candidates
        .iter()
        .map(|candidate| candidate.0)
        .fold(f64::INFINITY, f64::min);
    let (_, _, (ux, uy), end_a, end_b) = candidates
        .into_iter()
        .filter(|candidate| candidate.0 <= START_LINE_MAX_LENGTH_RATIO * shortest)
        .min_by(|a, b| a.1.total_cmp(&b.1))?;

    // The direction of travel is `b - a` turned a quarter counterclockwise:
    // `(ux, uy)` turned so is `(-uy, ux)`. Swap the ends if that's
    // backwards.
    let (heading_y, heading_x) = start.heading_rad.sin_cos();
    let travel_along_u = (-uy * heading_x + ux * heading_y) >= 0.0;
    let (a, b) = if travel_along_u {
        (end_a, end_b)
    } else {
        (end_b, end_a)
    };
    Some(StartFinishLine {
        a: WorldPoint { x: a.0, y: a.1 },
        b: WorldPoint { x: b.0, y: b.1 },
    })
}

/// How far from `(x, y)`, along the unit direction `(dx, dy)`, the first
/// black point is - `None` beyond [`START_LINE_MAX_HALF_LENGTH_M`].
fn border_distance(grid: &BinaryGrid, x: f64, y: f64, dx: f64, dy: f64) -> Option<f64> {
    let step_m = 0.25 * grid.resolution_m;
    let mut traveled_m = 0.0;
    while traveled_m <= START_LINE_MAX_HALF_LENGTH_M {
        if !grid.is_white(x + dx * traveled_m, y + dy * traveled_m) {
            return Some(traveled_m);
        }
        traveled_m += step_m;
    }
    None
}

/// The direction of the track border around `at`, as a unit vector: the
/// principal axis of the border cells within [`BORDER_FIT_RADIUS_M`].
/// `None` with too few of them to tell.
fn border_direction(grid: &BinaryGrid, at: (f64, f64)) -> Option<(f64, f64)> {
    let radius_px = (BORDER_FIT_RADIUS_M / grid.resolution_m).ceil() as i64;
    let at_col = ((at.0 - grid.origin_x_m) / grid.resolution_m).floor() as i64;
    let at_row = ((at.1 - grid.origin_y_m) / grid.resolution_m).floor() as i64;
    let mut points = Vec::new();
    for row in (at_row - radius_px)..=(at_row + radius_px) {
        for col in (at_col - radius_px)..=(at_col + radius_px) {
            if col < 0 || row < 0 || col >= i64::from(grid.width) || row >= i64::from(grid.height) {
                continue;
            }
            let (col, row) = (col as u32, row as u32);
            let (cx, cy) = grid.center(col, row);
            if (cx - at.0).hypot(cy - at.1) <= BORDER_FIT_RADIUS_M && grid.is_border(col, row) {
                points.push((cx, cy));
            }
        }
    }
    if points.len() < 3 {
        return None;
    }
    let n = points.len() as f64;
    let mean_x = points.iter().map(|p| p.0).sum::<f64>() / n;
    let mean_y = points.iter().map(|p| p.1).sum::<f64>() / n;
    let (mut sxx, mut syy, mut sxy) = (0.0, 0.0, 0.0);
    for (px, py) in &points {
        let (dx, dy) = (px - mean_x, py - mean_y);
        sxx += dx * dx;
        syy += dy * dy;
        sxy += dx * dy;
    }
    let angle = 0.5 * (2.0 * sxy).atan2(sxx - syy);
    Some((angle.cos(), angle.sin()))
}

/// The 4-connected neighbors of `(col, row)` inside a `width` x `height`
/// grid.
fn neighbors4(col: u32, row: u32, width: u32, height: u32) -> impl Iterator<Item = (u32, u32)> {
    [(-1, 0), (1, 0), (0, -1), (0, 1)]
        .into_iter()
        .filter_map(move |offset| offset_cell(col, row, offset, width, height))
}

/// The 8-connected neighbors of `(col, row)` inside a `width` x `height`
/// grid.
fn neighbors8(col: u32, row: u32, width: u32, height: u32) -> impl Iterator<Item = (u32, u32)> {
    [
        (-1, -1),
        (0, -1),
        (1, -1),
        (-1, 0),
        (1, 0),
        (-1, 1),
        (0, 1),
        (1, 1),
    ]
    .into_iter()
    .filter_map(move |offset| offset_cell(col, row, offset, width, height))
}

fn offset_cell(
    col: u32,
    row: u32,
    (dc, dr): (i64, i64),
    width: u32,
    height: u32,
) -> Option<(u32, u32)> {
    let (c, r) = (i64::from(col) + dc, i64::from(row) + dr);
    (c >= 0 && r >= 0 && c < i64::from(width) && r < i64::from(height))
        .then_some((c as u32, r as u32))
}

#[cfg(test)]
mod tests {
    use super::*;

    const RESOLUTION_M: f64 = 0.05;

    /// A SLAM map of a straight corridor of half-width `half_width_m`
    /// through the origin, along `angle_rad`: free inside, a one-cell wall
    /// on both borders, unknown outside. Trajectory along its axis.
    fn corridor(angle_rad: f64, half_width_m: f64) -> SlamMap {
        let side = 200u32;
        let origin = -0.5 * f64::from(side) * RESOLUTION_M;
        let (sin, cos) = angle_rad.sin_cos();
        let mut pixels = Vec::with_capacity((side * side) as usize);
        for row in 0..side {
            for col in 0..side {
                let x = origin + (f64::from(col) + 0.5) * RESOLUTION_M;
                let y = origin + (f64::from(row) + 0.5) * RESOLUTION_M;
                let across = (-sin * x + cos * y).abs();
                pixels.push(if across < half_width_m {
                    SlamMap::FREE
                } else if across < half_width_m + RESOLUTION_M {
                    SlamMap::OCCUPIED
                } else {
                    SlamMap::UNKNOWN
                });
            }
        }
        let trajectory = (-20..=20)
            .map(|i| {
                let along = f64::from(i) * 0.1;
                [(along * cos) as f32, (along * sin) as f32]
            })
            .collect();
        SlamMap {
            resolution_m_per_px: RESOLUTION_M,
            origin_x_m: origin,
            origin_y_m: origin,
            width_px: side,
            height_px: side,
            pixels: pixels.into(),
            trajectory,
        }
    }

    fn set(map: &mut SlamMap, x: f64, y: f64, value: u8) {
        let col = ((x - map.origin_x_m) / map.resolution_m_per_px).floor() as usize;
        let row = ((y - map.origin_y_m) / map.resolution_m_per_px).floor() as usize;
        let mut pixels = map.pixels.to_vec();
        pixels[row * map.width_px as usize + col] = value;
        map.pixels = pixels.into();
    }

    /// Whether `(x, y)` is white in the exported raster - never outside it.
    fn is_white(info: &MapInfo, raster: &Raster, x: f64, y: f64) -> bool {
        let col = ((x - info.origin.x) / info.resolution_m_per_px).floor();
        let row = ((y - info.origin.y) / info.resolution_m_per_px).floor();
        col >= 0.0
            && row >= 0.0
            && col < f64::from(raster.width_px)
            && row < f64::from(raster.height_px)
            && raster.row(row as u32)[col as usize]
    }

    #[test]
    fn an_empty_map_isnt_exported() {
        assert_eq!(
            export(&SlamMap::default(), Pose2::default()).unwrap_err(),
            ExportError::Empty
        );
    }

    #[test]
    fn free_space_the_vehicle_cant_reach_is_black() {
        let mut map = corridor(0.0, 1.0);
        // A free speck outside the walls...
        set(&mut map, 0.0, 3.0, SlamMap::FREE);
        // ... and a one-cell hole in the upper wall with free space behind
        // it, where beams leaked through.
        set(&mut map, 2.0, 1.02, SlamMap::FREE);
        for dy in 0..10 {
            set(
                &mut map,
                2.0,
                1.08 + f64::from(dy) * RESOLUTION_M,
                SlamMap::FREE,
            );
        }

        let (info, raster) = export(&map, Pose2::default()).unwrap();

        assert!(is_white(&info, &raster, 0.0, 0.0));
        assert!(
            is_white(&info, &raster, 0.0, 0.97),
            "the track reaches its walls"
        );
        assert!(!is_white(&info, &raster, 0.0, 1.02), "walls are black");
        assert!(!is_white(&info, &raster, 0.0, 3.0));
        assert!(!is_white(&info, &raster, 2.0, 1.3));
    }

    #[test]
    fn the_map_is_cropped_to_the_track_with_a_black_margin() {
        let (info, raster) = export(&corridor(0.0, 1.0), Pose2::default()).unwrap();

        // 2 m of free space, plus the margin on both sides.
        let expected = 40 + 2 * CROP_MARGIN_PX as u32;
        assert_eq!(raster.height_px, expected);
        assert!(raster.row(0).iter().all(|&white| !white));
        assert!(raster.row(raster.height_px - 1).iter().all(|&white| !white));
        assert!((info.origin.y - (-1.0 - CROP_MARGIN_PX as f64 * RESOLUTION_M)).abs() < 1e-9);
    }

    #[test]
    fn the_start_line_is_across_the_track_and_heads_where_the_vehicle_did() {
        for angle in [0.0, 0.5, 2.0, -1.2] {
            let map = corridor(angle, 1.0);
            // Slightly off the axis and off the corridor's direction: the
            // line must still be across the track, not along the heading.
            let start = Pose2::new(0.1 * angle.sin(), -0.1 * angle.cos(), angle + 0.2);

            let (info, _) = export(&map, start).unwrap();
            let line = info.start_finish_line;
            let (_, _, heading) = line.start_pose();

            let length = (line.b.x - line.a.x).hypot(line.b.y - line.a.y);
            assert!((length - 2.0).abs() < 0.1, "spans the track: {length}");
            // Within 3 degrees: a diagonal border is a staircase of pixels.
            let error = Pose2::new(0.0, 0.0, heading - angle).heading_rad.abs();
            assert!(error < 0.05, "angle {angle}: heading {heading}");
        }
    }

    #[test]
    fn driving_the_other_way_swaps_the_ends() {
        let map = corridor(0.0, 1.0);
        let (info, _) = export(&map, Pose2::new(0.0, 0.0, PI - 0.1)).unwrap();
        let (_, _, heading) = info.start_finish_line.start_pose();
        assert!(Pose2::new(0.0, 0.0, heading - PI).heading_rad.abs() < 0.02);
    }

    #[test]
    fn a_start_off_the_track_has_no_start_line() {
        let map = corridor(0.0, 1.0);
        assert_eq!(
            export(&map, Pose2::new(0.0, 3.0, 0.0)).unwrap_err(),
            ExportError::NoStartFinishLine
        );
    }
}
