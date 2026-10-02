//! Rasterizes a closed centerline into a binary occupancy grid: white
//! (drivable) within `track_width_m / 2` of the centerline, black
//! otherwise. Uses a bucketed spatial grid (cell size = the query radius)
//! so most pixels - which are far from the thin track band - skip distance
//! computation entirely, instead of the `O(pixels x points)` a brute-force
//! nearest-point search would cost.

use crate::environment::raster::Raster;
use crate::geometry::Point2;

/// Maps between pixel coordinates and world coordinates. World coordinates
/// share the same axis directions as pixel columns/rows (x rightward, y
/// downward), so `origin_{x,y}_m` is simply the world position of pixel
/// `(0, 0)`'s corner and no axis flip is ever needed.
#[derive(Debug, Clone, Copy)]
pub struct ImageTransform {
    pub resolution_m_per_px: f64,
    pub width_px: u32,
    pub height_px: u32,
    pub origin_x_m: f64,
    pub origin_y_m: f64,
}

impl ImageTransform {
    fn pixel_center_world(&self, col: u32, row: u32) -> Point2 {
        Point2 {
            x: self.origin_x_m + (col as f64 + 0.5) * self.resolution_m_per_px,
            y: self.origin_y_m + (row as f64 + 0.5) * self.resolution_m_per_px,
        }
    }
}

/// Bucket size equals the query radius, so any point within the radius of a
/// query location is guaranteed to be in the query's own bucket or one of
/// its 8 neighbors (standard uniform-grid nearest-neighbor result) - no
/// correctness loss versus brute force, just skipped empty regions.
struct PointGrid {
    cell_size: f64,
    min_x: f64,
    min_y: f64,
    cols: usize,
    rows: usize,
    buckets: Vec<Vec<usize>>,
}

impl PointGrid {
    fn build(points: &[Point2], cell_size: f64) -> Self {
        let min_x = points.iter().map(|p| p.x).fold(f64::INFINITY, f64::min) - cell_size;
        let min_y = points.iter().map(|p| p.y).fold(f64::INFINITY, f64::min) - cell_size;
        let max_x = points.iter().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max) + cell_size;
        let max_y = points.iter().map(|p| p.y).fold(f64::NEG_INFINITY, f64::max) + cell_size;

        let cols = (((max_x - min_x) / cell_size).ceil() as usize).max(1) + 1;
        let rows = (((max_y - min_y) / cell_size).ceil() as usize).max(1) + 1;
        let mut buckets = vec![Vec::new(); cols * rows];

        for (idx, p) in points.iter().enumerate() {
            let (bx, by) = Self::bucket_of(p.x, p.y, min_x, min_y, cell_size);
            buckets[by * cols + bx].push(idx);
        }

        Self {
            cell_size,
            min_x,
            min_y,
            cols,
            rows,
            buckets,
        }
    }

    fn bucket_of(x: f64, y: f64, min_x: f64, min_y: f64, cell_size: f64) -> (usize, usize) {
        (
            ((x - min_x) / cell_size) as usize,
            ((y - min_y) / cell_size) as usize,
        )
    }

    /// Distance to the nearest point within the 3x3 bucket neighborhood
    /// around `query`, if any point fell in it. Any point genuinely within
    /// `cell_size` of `query` is guaranteed to be found here - see the
    /// struct-level doc comment.
    fn nearest_distance(&self, query: Point2, points: &[Point2]) -> Option<f64> {
        let (bx, by) = Self::bucket_of(query.x, query.y, self.min_x, self.min_y, self.cell_size);
        let mut best: Option<f64> = None;
        for dy in -1i64..=1 {
            for dx in -1i64..=1 {
                let nx = bx as i64 + dx;
                let ny = by as i64 + dy;
                if nx < 0 || ny < 0 || nx as usize >= self.cols || ny as usize >= self.rows {
                    continue;
                }
                for &idx in &self.buckets[ny as usize * self.cols + nx as usize] {
                    let d = query.distance(&points[idx]);
                    if best.is_none_or(|b| d < b) {
                        best = Some(d);
                    }
                }
            }
        }
        best
    }
}

/// Rasterizes `centerline` (closed) into a white/black grid described by
/// `transform`: white iff within `track_width_m / 2` of the nearest
/// centerline point.
pub fn rasterize(centerline: &[Point2], track_width_m: f64, transform: &ImageTransform) -> Raster {
    let radius = track_width_m / 2.0;
    let grid = PointGrid::build(centerline, radius);

    let width = transform.width_px as usize;
    let height = transform.height_px as usize;
    let mut white = vec![false; width * height];

    for row in 0..transform.height_px {
        for col in 0..transform.width_px {
            let world = transform.pixel_center_world(col, row);
            let is_white = grid
                .nearest_distance(world, centerline)
                .is_some_and(|d| d <= radius);
            white[row as usize * width + col as usize] = is_white;
        }
    }

    Raster::new(transform.width_px, transform.height_px, white)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn brute_force_rasterize(
        centerline: &[Point2],
        track_width_m: f64,
        transform: &ImageTransform,
    ) -> Vec<bool> {
        let radius = track_width_m / 2.0;
        let mut white = vec![false; (transform.width_px * transform.height_px) as usize];
        for row in 0..transform.height_px {
            for col in 0..transform.width_px {
                let world = transform.pixel_center_world(col, row);
                let min_dist = centerline
                    .iter()
                    .map(|p| world.distance(p))
                    .fold(f64::INFINITY, f64::min);
                white[(row * transform.width_px + col) as usize] = min_dist <= radius;
            }
        }
        white
    }

    #[test]
    fn bucketed_rasterization_matches_brute_force() {
        let centerline = vec![
            Point2 { x: -1.0, y: 0.0 },
            Point2 { x: 0.0, y: 1.0 },
            Point2 { x: 1.0, y: 0.0 },
            Point2 { x: 0.0, y: -1.0 },
        ];
        let transform = ImageTransform {
            resolution_m_per_px: 0.1,
            width_px: 40,
            height_px: 40,
            origin_x_m: -2.0,
            origin_y_m: -2.0,
        };
        let raster = rasterize(&centerline, 0.6, &transform);
        let expected = brute_force_rasterize(&centerline, 0.6, &transform);

        let mut actual = Vec::new();
        for row in 0..transform.height_px {
            actual.extend_from_slice(raster.row(row));
        }
        assert_eq!(actual, expected);
        assert!(actual.iter().any(|&w| w), "expected some white pixels");
        assert!(actual.iter().any(|&w| !w), "expected some black pixels");
    }
}
