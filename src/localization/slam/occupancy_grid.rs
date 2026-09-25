//! [`OccupancyGrid`]: counts, per cell, how many beams passed through it and
//! how many ended in it, and turns those counts into a free / occupied /
//! unknown map - Karto's `OccupancyGrid` (`Karto.h`).
//!
//! Karto rebuilds its grid from every scan each time the map is asked for,
//! since loop closure can move any of them. This one is incremental
//! instead - each new scan is traced once, and the grid grows as the
//! vehicle explores - and only [`rebuild`](OccupancyGrid::rebuild)s from
//! every scan when a loop closure actually moved them.

use super::scan::LocalizedScan;
use crate::topics::SlamMap;
use std::sync::Arc;

/// How much room to add around the area actually needed whenever the grid
/// grows, in meters - so it doesn't reallocate on every scan at the edge
/// of the explored area.
const GROWTH_MARGIN_M: f64 = 5.0;

/// Pass/hit counters over a growable grid. Cell `(i, j)` is centered on
/// `offset + (i, j) * resolution` - world to grid rounds to the nearest
/// cell, as Karto does.
pub struct OccupancyGrid {
    resolution_m: f64,
    /// Beams must have passed through a cell more than this many times
    /// before it's anything but unknown - Karto's `MinPassThrough`.
    min_pass_through: u32,
    /// Hit/pass ratio above which a cell is occupied rather than free -
    /// Karto's `OccupancyThreshold`.
    occupancy_threshold: f64,
    offset: (f64, f64),
    width: i32,
    height: i32,
    pass_counts: Vec<u32>,
    hit_counts: Vec<u32>,
}

impl OccupancyGrid {
    pub fn new(resolution_m: f64, min_pass_through: u32, occupancy_threshold: f64) -> Self {
        Self {
            resolution_m,
            min_pass_through,
            occupancy_threshold,
            offset: (0.0, 0.0),
            width: 0,
            height: 0,
            pass_counts: Vec::new(),
            hit_counts: Vec::new(),
        }
    }

    /// Whether no scan has been added yet.
    pub fn is_empty(&self) -> bool {
        self.width == 0
    }

    /// Drops every scan added so far.
    pub fn clear(&mut self) {
        *self = Self::new(
            self.resolution_m,
            self.min_pass_through,
            self.occupancy_threshold,
        );
    }

    /// Throws every count away and traces every scan of `scans` again, from
    /// their current corrected poses - Karto's
    /// `OccupancyGrid::CreateFromScans`.
    pub fn rebuild(&mut self, scans: &[LocalizedScan]) {
        self.clear();
        for scan in scans {
            self.add_scan(scan);
        }
    }

    /// Traces every reading of `scan`, from its corrected pose - Karto's
    /// `OccupancyGrid::AddScan`. Readings past the scan's range threshold
    /// are traced up to it, but don't mark where they ended as occupied.
    pub fn add_scan(&mut self, scan: &LocalizedScan) {
        let origin = scan.corrected_pose();
        let threshold_m = scan.range_threshold_m();
        let rays: Vec<((f64, f64), bool)> = scan
            .readings()
            .iter()
            .map(|reading| {
                let ratio = (threshold_m / reading.range_m).min(1.0);
                let end = origin.transform_point(reading.x_m * ratio, reading.y_m * ratio);
                (end, reading.range_m < threshold_m)
            })
            .collect();

        let (mut min_x, mut min_y) = (origin.x_m, origin.y_m);
        let (mut max_x, mut max_y) = (origin.x_m, origin.y_m);
        for &((x, y), _) in &rays {
            min_x = min_x.min(x);
            min_y = min_y.min(y);
            max_x = max_x.max(x);
            max_y = max_y.max(y);
        }
        self.grow_to_contain(min_x, min_y, max_x, max_y);

        let from = self.world_to_grid(origin.x_m, origin.y_m);
        for (end, is_hit) in rays {
            let to = self.world_to_grid(end.0, end.1);
            self.trace_line(from, to);
            if is_hit && let Some(index) = self.index(to.0, to.1) {
                self.pass_counts[index] += 1;
                self.hit_counts[index] += 1;
            }
        }
    }

    /// The map as a [`SlamMap`] raster - Karto's `OccupancyGrid::Update`
    /// and `UpdateCell`. `trajectory` is passed through.
    pub fn to_map(&self, trajectory: Vec<[f32; 2]>) -> SlamMap {
        let pixels: Arc<[u8]> = self
            .pass_counts
            .iter()
            .zip(&self.hit_counts)
            .map(|(&passes, &hits)| {
                if passes <= self.min_pass_through {
                    SlamMap::UNKNOWN
                } else if f64::from(hits) / f64::from(passes) > self.occupancy_threshold {
                    SlamMap::OCCUPIED
                } else {
                    SlamMap::FREE
                }
            })
            .collect();
        SlamMap {
            resolution_m_per_px: self.resolution_m,
            // Pixel (0, 0)'s corner, half a cell before its center.
            origin_x_m: self.offset.0 - 0.5 * self.resolution_m,
            origin_y_m: self.offset.1 - 0.5 * self.resolution_m,
            width_px: self.width as u32,
            height_px: self.height as u32,
            pixels,
            trajectory,
        }
    }

    fn world_to_grid(&self, x_m: f64, y_m: f64) -> (i32, i32) {
        (
            ((x_m - self.offset.0) / self.resolution_m).round() as i32,
            ((y_m - self.offset.1) / self.resolution_m).round() as i32,
        )
    }

    fn index(&self, gx: i32, gy: i32) -> Option<usize> {
        ((0..self.width).contains(&gx) && (0..self.height).contains(&gy))
            .then(|| (gy * self.width + gx) as usize)
    }

    /// Makes sure the rectangle `min..=max` (world meters) is covered,
    /// reallocating with [`GROWTH_MARGIN_M`] to spare if it isn't. Cells
    /// stay on the same lattice, so counts are copied over unchanged.
    fn grow_to_contain(&mut self, min_x: f64, min_y: f64, max_x: f64, max_y: f64) {
        let margin = (GROWTH_MARGIN_M / self.resolution_m).ceil() as i32;
        if self.is_empty() {
            // Anchor the lattice on a multiple of the resolution, so maps
            // of the same place line up cell for cell.
            let snap = |v: f64| (v / self.resolution_m).round() * self.resolution_m;
            self.offset = (snap(min_x), snap(min_y));
            let (gx, gy) = self.world_to_grid(max_x, max_y);
            self.reallocate(-margin, -margin, gx + 1 + margin, gy + 1 + margin);
            return;
        }

        let (low_x, low_y) = self.world_to_grid(min_x, min_y);
        let (high_x, high_y) = self.world_to_grid(max_x, max_y);
        if low_x >= 0 && low_y >= 0 && high_x < self.width && high_y < self.height {
            return;
        }
        let grow_low = |low: i32| if low < 0 { low - margin } else { 0 };
        let grow_high = |high: i32, size: i32| {
            if high >= size {
                high + 1 + margin
            } else {
                size
            }
        };
        self.reallocate(
            grow_low(low_x),
            grow_low(low_y),
            grow_high(high_x, self.width),
            grow_high(high_y, self.height),
        );
    }

    /// Resizes the grid to cells `start..end` of the current lattice
    /// (`start <= 0`, `end >= current size`), keeping every count.
    fn reallocate(&mut self, start_x: i32, start_y: i32, end_x: i32, end_y: i32) {
        let width = end_x - start_x;
        let height = end_y - start_y;
        let mut pass_counts = vec![0; (width * height) as usize];
        let mut hit_counts = vec![0; (width * height) as usize];
        for row in 0..self.height {
            let old = (row * self.width) as usize..((row + 1) * self.width) as usize;
            let new_start = ((row - start_y) * width - start_x) as usize;
            let new = new_start..new_start + self.width as usize;
            pass_counts[new.clone()].copy_from_slice(&self.pass_counts[old.clone()]);
            hit_counts[new].copy_from_slice(&self.hit_counts[old]);
        }
        self.offset = (
            self.offset.0 + f64::from(start_x) * self.resolution_m,
            self.offset.1 + f64::from(start_y) * self.resolution_m,
        );
        self.width = width;
        self.height = height;
        self.pass_counts = pass_counts;
        self.hit_counts = hit_counts;
    }

    /// Bumps the pass count of every cell on the line from `from` to `to`,
    /// both ends included - Karto's `Grid::TraceLine` (Bresenham).
    fn trace_line(&mut self, from: (i32, i32), to: (i32, i32)) {
        let (mut x0, mut y0) = from;
        let (mut x1, mut y1) = to;
        let steep = (y1 - y0).abs() > (x1 - x0).abs();
        if steep {
            std::mem::swap(&mut x0, &mut y0);
            std::mem::swap(&mut x1, &mut y1);
        }
        if x0 > x1 {
            std::mem::swap(&mut x0, &mut x1);
            std::mem::swap(&mut y0, &mut y1);
        }
        let delta_x = x1 - x0;
        let delta_y = (y1 - y0).abs();
        let y_step = if y0 < y1 { 1 } else { -1 };
        let mut error = 0;
        let mut y = y0;
        for x in x0..=x1 {
            let (px, py) = if steep { (y, x) } else { (x, y) };
            error += delta_y;
            if 2 * error >= delta_x {
                y += y_step;
                error -= delta_x;
            }
            if let Some(index) = self.index(px, py) {
                self.pass_counts[index] += 1;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::localization::slam::pose::Pose2;
    use crate::topics::LidarScan;
    use std::time::Instant;

    /// A single reading straight ahead at `range`.
    fn one_ray(range: f32) -> LidarScan {
        LidarScan::new(vec![range], vec![1.0], 0.1, 10.0, 0.0)
    }

    fn pixel_at(map: &SlamMap, x_m: f64, y_m: f64) -> u8 {
        let col = ((x_m - map.origin_x_m) / map.resolution_m_per_px).floor() as usize;
        let row = ((y_m - map.origin_y_m) / map.resolution_m_per_px).floor() as usize;
        map.pixels[row * map.width_px as usize + col]
    }

    fn grid_with(scans: usize, scan: &LidarScan, pose: Pose2) -> OccupancyGrid {
        let mut grid = OccupancyGrid::new(0.1, 2, 0.1);
        let localized = LocalizedScan::new(scan, Instant::now(), pose, 8.0);
        for _ in 0..scans {
            grid.add_scan(&localized);
        }
        grid
    }

    #[test]
    fn a_ray_frees_the_cells_it_crosses_and_occupies_the_one_it_ends_in() {
        let map = grid_with(3, &one_ray(2.0), Pose2::default()).to_map(Vec::new());
        assert_eq!(pixel_at(&map, 1.0, 0.0), SlamMap::FREE);
        assert_eq!(pixel_at(&map, 2.0, 0.0), SlamMap::OCCUPIED);
        assert_eq!(pixel_at(&map, 1.0, 1.0), SlamMap::UNKNOWN);
    }

    #[test]
    fn cells_crossed_too_few_times_stay_unknown() {
        let map = grid_with(1, &one_ray(2.0), Pose2::default()).to_map(Vec::new());
        assert_eq!(pixel_at(&map, 1.0, 0.0), SlamMap::UNKNOWN);
    }

    #[test]
    fn a_reading_past_the_range_threshold_only_frees_up_to_it() {
        // Threshold 8 m, reading 9 m: free up to 8 m, and no hit anywhere.
        let map = grid_with(3, &one_ray(9.0), Pose2::default()).to_map(Vec::new());
        assert_eq!(pixel_at(&map, 7.5, 0.0), SlamMap::FREE);
        assert!(!map.pixels.contains(&SlamMap::OCCUPIED));
    }

    #[test]
    fn rebuilding_traces_scans_at_their_new_poses() {
        let ray = one_ray(2.0);
        let mut scans: Vec<LocalizedScan> = (0..3)
            .map(|_| LocalizedScan::new(&ray, Instant::now(), Pose2::default(), 8.0))
            .collect();
        let mut grid = OccupancyGrid::new(0.1, 2, 0.1);
        grid.rebuild(&scans);
        assert_eq!(
            pixel_at(&grid.to_map(Vec::new()), 2.0, 0.0),
            SlamMap::OCCUPIED
        );

        // Every scan moved 1 m up: the wall follows.
        for scan in &mut scans {
            scan.set_corrected_pose(Pose2::new(0.0, 1.0, 0.0));
        }
        grid.rebuild(&scans);
        let map = grid.to_map(Vec::new());
        assert_eq!(pixel_at(&map, 2.0, 1.0), SlamMap::OCCUPIED);
        assert_eq!(pixel_at(&map, 2.0, 0.0), SlamMap::UNKNOWN);
    }

    #[test]
    fn growing_keeps_every_count_in_place() {
        let mut grid = grid_with(3, &one_ray(2.0), Pose2::default());
        let before = grid.to_map(Vec::new());
        // A scan far away, behind the first one, forces growth on the low
        // side - which shifts the offset.
        let far = LocalizedScan::new(
            &one_ray(1.0),
            Instant::now(),
            Pose2::new(-20.0, -20.0, 0.0),
            8.0,
        );
        grid.add_scan(&far);
        let after = grid.to_map(Vec::new());

        assert!(after.width_px > before.width_px && after.origin_x_m < before.origin_x_m);
        assert_eq!(pixel_at(&after, 1.0, 0.0), SlamMap::FREE);
        assert_eq!(pixel_at(&after, 2.0, 0.0), SlamMap::OCCUPIED);
    }
}
