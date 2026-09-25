//! [`TrackGrid`]: a map's raster reduced to the one track a race line can
//! use - the drivable pixels connected to the start/finish line - and the
//! two walls around it, plus what's measured on it: the distance to either
//! wall ([`TrackGrid::wall_distances`]) and the free space on either side
//! of a point ([`TrackGrid::free_distance`]).

use super::PlanError;
use super::geometry::Point2;
use crate::environment::Map;
use std::collections::VecDeque;

/// Squared distance standing in for "infinitely far" in
/// [`squared_distance_transform`] - far beyond any real map.
const FAR: f64 = 1e20;

/// The track of a map: every drivable pixel 4-connected to the start pose,
/// and the two walls bounding it - the inner one (the island the track
/// loops around) and the outer one (everything else).
///
/// Padded with one non-drivable pixel on every side, so the outer wall
/// always surrounds the track even where the track reaches the raster's
/// edge.
pub struct TrackGrid {
    /// Size of the padded grid, in pixels.
    width: usize,
    height: usize,
    resolution_m_per_px: f64,
    /// World position of the padded grid's pixel `(0, 0)`'s corner.
    origin_x_m: f64,
    origin_y_m: f64,
    /// Row-major, `true` for a track pixel.
    track: Vec<bool>,
    /// Row-major, `true` for a pixel of the inner wall.
    inner: Vec<bool>,
}

impl TrackGrid {
    /// Extracts the track of `map` around its start pose.
    ///
    /// Fails if the start pose isn't on a drivable pixel, or if the track
    /// isn't a single loop: exactly two walls must touch it, one of them
    /// enclosed by the track.
    pub fn build(map: &Map) -> Result<TrackGrid, PlanError> {
        let raster = &map.raster;
        let width = raster.width_px as usize + 2;
        let height = raster.height_px as usize + 2;
        let mut drivable = vec![false; width * height];
        for row in 0..raster.height_px {
            let start = (row as usize + 1) * width + 1;
            drivable[start..start + raster.width_px as usize].copy_from_slice(raster.row(row));
        }

        let resolution_m_per_px = map.info.resolution_m_per_px;
        let mut grid = TrackGrid {
            width,
            height,
            resolution_m_per_px,
            origin_x_m: map.info.origin.x - resolution_m_per_px,
            origin_y_m: map.info.origin.y - resolution_m_per_px,
            track: vec![false; width * height],
            inner: vec![false; width * height],
        };

        let (x_m, y_m, _) = map.info.start_finish_line.start_pose();
        let start = grid
            .cell_at(x_m, y_m)
            .filter(|&cell| drivable[cell])
            .ok_or(PlanError::StartOffTrack { x_m, y_m })?;
        grid.track = flood_fill(&drivable, width, height, start);

        let (labels, _) = label_walls(&grid.track, width, height);
        // The padding is one wall component, the outer one by construction.
        let outer = labels[0];
        let mut touching: Vec<u32> = Vec::new();
        for (cell, _) in grid.track.iter().enumerate().filter(|(_, track)| **track) {
            for neighbor in neighbors4(cell, width, height) {
                let label = labels[neighbor];
                if label != 0 && !touching.contains(&label) {
                    touching.push(label);
                }
            }
        }
        if touching.len() != 2 {
            return Err(PlanError::NotALoop {
                walls: touching.len(),
            });
        }
        let Some(&inner) = touching.iter().find(|&&label| label != outer) else {
            return Err(PlanError::NotALoop { walls: 1 });
        };
        grid.inner = labels.iter().map(|&label| label == inner).collect();
        Ok(grid)
    }

    /// Size of the (padded) grid, in pixels.
    pub fn size(&self) -> (usize, usize) {
        (self.width, self.height)
    }

    /// World position of the center of pixel (`col`, `row`).
    pub fn cell_center(&self, col: usize, row: usize) -> Point2 {
        Point2 {
            x: self.origin_x_m + (col as f64 + 0.5) * self.resolution_m_per_px,
            y: self.origin_y_m + (row as f64 + 0.5) * self.resolution_m_per_px,
        }
    }

    /// Index of the pixel containing (`x_m`, `y_m`), or `None` off the grid.
    fn cell_at(&self, x_m: f64, y_m: f64) -> Option<usize> {
        let col = ((x_m - self.origin_x_m) / self.resolution_m_per_px).floor();
        let row = ((y_m - self.origin_y_m) / self.resolution_m_per_px).floor();
        if col < 0.0 || row < 0.0 || col >= self.width as f64 || row >= self.height as f64 {
            return None;
        }
        Some(row as usize * self.width + col as usize)
    }

    /// Whether (`x_m`, `y_m`) is on the track.
    pub fn is_track(&self, x_m: f64, y_m: f64) -> bool {
        self.cell_at(x_m, y_m).is_some_and(|cell| self.track[cell])
    }

    /// How far one can go from `from` along the unit vector `direction`
    /// before leaving the track, in meters - at most `max_m`. Marched in
    /// quarter-pixel steps, so accurate to about an eighth of a pixel.
    pub fn free_distance(&self, from: Point2, direction: Point2, max_m: f64) -> f64 {
        let step = self.resolution_m_per_px / 4.0;
        let mut distance = 0.0;
        while distance < max_m {
            let next = distance + step;
            if !self.is_track(from.x + direction.x * next, from.y + direction.y * next) {
                return distance + step / 2.0;
            }
            distance = next;
        }
        max_m
    }

    /// For every pixel, row-major, its distance to the nearest pixel of the
    /// inner wall and of the outer wall (every non-track pixel that isn't
    /// the inner wall), center to center, in meters.
    pub fn wall_distances(&self) -> (Vec<f64>, Vec<f64>) {
        let outer: Vec<bool> = self
            .track
            .iter()
            .zip(&self.inner)
            .map(|(&track, &inner)| !track && !inner)
            .collect();
        let to_meters = |squared: Vec<f64>| -> Vec<f64> {
            squared
                .into_iter()
                .map(|d2| d2.sqrt() * self.resolution_m_per_px)
                .collect()
        };
        (
            to_meters(squared_distance_transform(
                &self.inner,
                self.width,
                self.height,
            )),
            to_meters(squared_distance_transform(&outer, self.width, self.height)),
        )
    }
}

/// Every cell of `passable` 4-connected to `start`.
fn flood_fill(passable: &[bool], width: usize, height: usize, start: usize) -> Vec<bool> {
    let mut reached = vec![false; passable.len()];
    reached[start] = true;
    let mut queue = VecDeque::from([start]);
    while let Some(cell) = queue.pop_front() {
        for neighbor in neighbors4(cell, width, height) {
            if passable[neighbor] && !reached[neighbor] {
                reached[neighbor] = true;
                queue.push_back(neighbor);
            }
        }
    }
    reached
}

/// Labels the 8-connected components of the non-track cells, from `1` up -
/// `0` marks a track cell. Returns the labels and how many there are.
fn label_walls(track: &[bool], width: usize, height: usize) -> (Vec<u32>, u32) {
    let mut labels = vec![0u32; track.len()];
    let mut count = 0;
    for seed in 0..track.len() {
        if track[seed] || labels[seed] != 0 {
            continue;
        }
        count += 1;
        labels[seed] = count;
        let mut queue = VecDeque::from([seed]);
        while let Some(cell) = queue.pop_front() {
            for neighbor in neighbors8(cell, width, height) {
                if !track[neighbor] && labels[neighbor] == 0 {
                    labels[neighbor] = count;
                    queue.push_back(neighbor);
                }
            }
        }
    }
    (labels, count)
}

fn neighbors4(cell: usize, width: usize, height: usize) -> impl Iterator<Item = usize> {
    let (col, row) = ((cell % width) as i64, (cell / width) as i64);
    [(1, 0), (-1, 0), (0, 1), (0, -1)]
        .into_iter()
        .filter_map(move |(dc, dr)| offset(col + dc, row + dr, width, height))
}

fn neighbors8(cell: usize, width: usize, height: usize) -> impl Iterator<Item = usize> {
    let (col, row) = ((cell % width) as i64, (cell / width) as i64);
    (-1..=1)
        .flat_map(|dr| (-1..=1).map(move |dc| (dc, dr)))
        .filter(|&offsets| offsets != (0, 0))
        .filter_map(move |(dc, dr)| offset(col + dc, row + dr, width, height))
}

fn offset(col: i64, row: i64, width: usize, height: usize) -> Option<usize> {
    (col >= 0 && row >= 0 && (col as usize) < width && (row as usize) < height)
        .then(|| row as usize * width + col as usize)
}

/// Exact squared Euclidean distance, in pixels, from every cell to the
/// nearest `source` cell (row-major) - [`FAR`] everywhere if there's none.
/// Felzenszwalb and Huttenlocher's separable algorithm: a 1D lower-envelope
/// pass down every column, then along every row.
pub fn squared_distance_transform(source: &[bool], width: usize, height: usize) -> Vec<f64> {
    let mut grid: Vec<f64> = source
        .iter()
        .map(|&is_source| if is_source { 0.0 } else { FAR })
        .collect();
    let longest = width.max(height);
    let mut line = vec![0.0; longest];
    let mut out = vec![0.0; longest];
    let mut hull = vec![0usize; longest];
    let mut bounds = vec![0.0; longest + 1];

    for col in 0..width {
        for row in 0..height {
            line[row] = grid[row * width + col];
        }
        distance_transform_1d(&line[..height], &mut out, &mut hull, &mut bounds);
        for row in 0..height {
            grid[row * width + col] = out[row];
        }
    }
    for row in 0..height {
        let cells = &mut grid[row * width..(row + 1) * width];
        line[..width].copy_from_slice(cells);
        distance_transform_1d(&line[..width], &mut out, &mut hull, &mut bounds);
        cells.copy_from_slice(&out[..width]);
    }
    grid
}

/// One 1D pass of [`squared_distance_transform`]: `out[q] = min_p (q - p)^2
/// + f[p]`, via the lower envelope of the parabolas rooted at each `p`.
/// `hull` and `bounds` are scratch space at least `f.len()` (+1) long.
fn distance_transform_1d(f: &[f64], out: &mut [f64], hull: &mut [usize], bounds: &mut [f64]) {
    let n = f.len();
    if n == 0 {
        return;
    }
    let intersection = |p: usize, q: usize| -> f64 {
        let (p_f, q_f) = (p as f64, q as f64);
        ((f[q] + q_f * q_f) - (f[p] + p_f * p_f)) / (2.0 * (q_f - p_f))
    };
    let mut k = 0;
    hull[0] = 0;
    bounds[0] = f64::NEG_INFINITY;
    bounds[1] = f64::INFINITY;
    for q in 1..n {
        let mut s = intersection(hull[k], q);
        while s <= bounds[k] {
            k -= 1;
            s = intersection(hull[k], q);
        }
        k += 1;
        hull[k] = q;
        bounds[k] = s;
        bounds[k + 1] = f64::INFINITY;
    }
    k = 0;
    for (q, value) in out.iter_mut().enumerate().take(n) {
        while bounds[k + 1] < q as f64 {
            k += 1;
        }
        let d = q as f64 - hull[k] as f64;
        *value = d * d + f[hull[k]];
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::environment::{
        ImageOrigin, MapInfo, MapSource, Raster, StartFinishLine, WorldPoint,
    };
    use std::path::PathBuf;

    /// A map of a ring around `(0, 0)`: drivable between `inner_m` and
    /// `outer_m`, at `resolution` m/px, run counterclockwise from its start
    /// line on `+x`.
    pub fn ring_map(inner_m: f64, outer_m: f64, resolution: f64) -> Map {
        let half = outer_m + 0.5;
        let size = (2.0 * half / resolution).ceil() as u32;
        let mut white = Vec::with_capacity((size * size) as usize);
        for row in 0..size {
            for col in 0..size {
                let x = -half + (col as f64 + 0.5) * resolution;
                let y = -half + (row as f64 + 0.5) * resolution;
                let r = (x * x + y * y).sqrt();
                white.push(r > inner_m && r < outer_m);
            }
        }
        Map {
            folder: PathBuf::new(),
            info: MapInfo {
                resolution_m_per_px: resolution,
                width_px: size,
                height_px: size,
                origin: ImageOrigin {
                    x: -half,
                    y: -half,
                    theta_rad: 0.0,
                },
                // Counterclockwise at +x means heading +y: `a` (left) is
                // toward the center.
                start_finish_line: StartFinishLine {
                    a: WorldPoint { x: inner_m, y: 0.0 },
                    b: WorldPoint { x: outer_m, y: 0.0 },
                },
                generated_at: String::new(),
                source: MapSource::Real,
                generation: None,
            },
            raster: Raster::new(size, size, white),
            centerline: Vec::new(),
            race_line: Vec::new(),
        }
    }

    #[test]
    fn the_distance_transform_matches_brute_force() {
        let (width, height) = (13, 9);
        let source: Vec<bool> = (0..width * height)
            .map(|cell| [5, 17, 60, 100].contains(&cell))
            .collect();
        let fast = squared_distance_transform(&source, width, height);
        for (cell, &distance) in fast.iter().enumerate() {
            let (col, row) = ((cell % width) as f64, (cell / width) as f64);
            let brute = (0..width * height)
                .filter(|&other| source[other])
                .map(|other| {
                    let (c, r) = ((other % width) as f64, (other / width) as f64);
                    (c - col).powi(2) + (r - row).powi(2)
                })
                .fold(f64::INFINITY, f64::min);
            assert_eq!(distance, brute, "cell {cell}");
        }
    }

    #[test]
    fn a_ring_is_one_loop_with_the_island_as_inner_wall() {
        let map = ring_map(1.0, 2.0, 0.05);
        let grid = TrackGrid::build(&map).unwrap();
        assert!(grid.is_track(1.5, 0.0));
        assert!(!grid.is_track(0.0, 0.0));
        assert!(!grid.is_track(2.3, 0.0));

        // From the middle of the track, outward and inward: half a meter
        // each way.
        let middle = Point2 { x: 0.0, y: 1.5 };
        let out = grid.free_distance(middle, Point2 { x: 0.0, y: 1.0 }, 5.0);
        let inward = grid.free_distance(middle, Point2 { x: 0.0, y: -1.0 }, 5.0);
        assert!((out - 0.5).abs() < 0.05, "{out}");
        assert!((inward - 0.5).abs() < 0.05, "{inward}");
    }

    #[test]
    fn a_track_with_an_obstacle_is_not_a_single_loop() {
        let mut map = ring_map(1.0, 2.0, 0.05);
        // A pillar in the track, on -x.
        let size = map.raster.width_px;
        let mut white = map.raster.to_bytes();
        let half = 2.5;
        for row in 0..size {
            for col in 0..size {
                let x = -half + (col as f64 + 0.5) * 0.05;
                let y = -half + (row as f64 + 0.5) * 0.05;
                if ((x + 1.5).powi(2) + y * y).sqrt() < 0.1 {
                    white[(row * size + col) as usize] = 0;
                }
            }
        }
        map.raster = Raster::new(size, size, white.iter().map(|&p| p == 255).collect());
        assert!(matches!(
            TrackGrid::build(&map),
            Err(PlanError::NotALoop { walls: 3 })
        ));
    }

    #[test]
    fn a_start_pose_off_the_track_is_refused() {
        let mut map = ring_map(1.0, 2.0, 0.05);
        map.info.start_finish_line = StartFinishLine {
            a: WorldPoint { x: -0.1, y: 0.0 },
            b: WorldPoint { x: 0.1, y: 0.0 },
        };
        assert!(matches!(
            TrackGrid::build(&map),
            Err(PlanError::StartOffTrack { .. })
        ));
    }
}
