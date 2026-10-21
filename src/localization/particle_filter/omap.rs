//! [`OMap`]: the occupancy grid the ray casters work on - range_libc's
//! `OMap` - and [`DistanceTransform`], the distance from every pixel to the
//! nearest occupied one, which [`super::ray_marching`] steps through.
//!
//! Everything here is in pixels, as in range_libc. Pixel `(x, y)` is column
//! `x` and row `y` of the map's raster, whose rows grow along +y (see
//! [`crate::topics::SelectedMap`]): pixel space is the world scaled by the
//! resolution and shifted by the origin, with no flip, so a heading means
//! the same in both.

use crate::topics::SelectedMap;

/// Which pixels of a map are occupied - range_libc's `OMap`, stored row by
/// row instead of as a vector of columns.
#[derive(Debug, Clone, PartialEq)]
pub struct OMap {
    pub width: usize,
    pub height: usize,
    occupied: Vec<bool>,
}

impl OMap {
    /// A `width` x `height` map with nothing occupied.
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            occupied: vec![false; width * height],
        }
    }

    /// The occupied pixels of `map`: everything that isn't drivable white,
    /// the same rule as [`crate::environment::raycast::cast_ray`].
    pub fn from_selected_map(map: &SelectedMap) -> Self {
        Self {
            width: map.width_px as usize,
            height: map.height_px as usize,
            occupied: map.pixels.iter().map(|&pixel| pixel != 255).collect(),
        }
    }

    /// Marks `(x, y)` occupied or not.
    pub fn set(&mut self, x: usize, y: usize, occupied: bool) {
        self.occupied[y * self.width + x] = occupied;
    }

    /// Whether `(x, y)` is occupied - range_libc's `isOccupied`: `false`
    /// outside the map.
    pub fn is_occupied(&self, x: i64, y: i64) -> bool {
        if x < 0 || y < 0 || x >= self.width as i64 || y >= self.height as i64 {
            return false;
        }
        self.occupied[y as usize * self.width + x as usize]
    }

    /// The occupied pixels that touch a free one inside the map - range_libc's
    /// `make_edge_map`, with `count_corners` adding the four diagonal
    /// neighbors. No ray can stop at an occupied pixel that isn't one of
    /// these.
    pub fn edge_map(&self, count_corners: bool) -> OMap {
        let mut edges = OMap::new(self.width, self.height);
        let straight = [(1, 0), (-1, 0), (0, 1), (0, -1)];
        let diagonal = [(1, 1), (-1, 1), (1, -1), (-1, -1)];
        for y in 0..self.height as i64 {
            for x in 0..self.width as i64 {
                if !self.is_occupied(x, y) {
                    continue;
                }
                let neighbors =
                    straight
                        .iter()
                        .chain(if count_corners { &diagonal[..] } else { &[] });
                let touches_free = neighbors.into_iter().any(|&(dx, dy)| {
                    let (cx, cy) = (x + dx, y + dy);
                    cx >= 0
                        && cy >= 0
                        && cx < self.width as i64
                        && cy < self.height as i64
                        && !self.is_occupied(cx, cy)
                });
                if touches_free {
                    edges.set(x as usize, y as usize, true);
                }
            }
        }
        edges
    }

    /// Every free pixel, as `(x, y)`.
    pub fn free_pixels(&self) -> Vec<(usize, usize)> {
        (0..self.height)
            .flat_map(|y| (0..self.width).map(move |x| (x, y)))
            .filter(|&(x, y)| !self.occupied[y * self.width + x])
            .collect()
    }
}

/// The Euclidean distance, in pixels, from every pixel to the nearest
/// occupied one (`0` on occupied pixels) - range_libc's
/// `DistanceTransform`. Computed exactly with Felzenszwalb and
/// Huttenlocher's two-pass lower envelope, as range_libc's vendored
/// `distance_transform.h` does.
#[derive(Debug, Clone, PartialEq)]
pub struct DistanceTransform {
    width: usize,
    distances: Vec<f32>,
}

impl DistanceTransform {
    pub fn new(map: &OMap) -> Self {
        let (width, height) = (map.width, map.height);
        // Squared distances, "infinite" where free.
        let far = (width * width + height * height + 1) as f64;
        let mut squared: Vec<f64> = (0..width * height)
            .map(|i| if map.occupied[i] { 0.0 } else { far })
            .collect();
        let mut line = Vec::new();
        let mut out = Vec::new();
        // Along columns, then along rows.
        for x in 0..width {
            line.clear();
            line.extend((0..height).map(|y| squared[y * width + x]));
            squared_distance_1d(&line, &mut out);
            for y in 0..height {
                squared[y * width + x] = out[y];
            }
        }
        for y in 0..height {
            line.clear();
            line.extend_from_slice(&squared[y * width..(y + 1) * width]);
            squared_distance_1d(&line, &mut out);
            squared[y * width..(y + 1) * width].copy_from_slice(&out);
        }
        Self {
            width,
            distances: squared.into_iter().map(|d| d.sqrt() as f32).collect(),
        }
    }

    /// The distance at `(x, y)`, which must be inside the map.
    pub fn get(&self, x: usize, y: usize) -> f32 {
        self.distances[y * self.width + x]
    }
}

/// The 1D squared distance transform of `f` into `out`: `out[q] = min_p
/// (q - p)^2 + f[p]`.
fn squared_distance_1d(f: &[f64], out: &mut Vec<f64>) {
    let n = f.len();
    out.clear();
    out.resize(n, 0.0);
    if n == 0 {
        return;
    }
    // Parabola vertices and the boundaries between them.
    let mut vertices = vec![0usize; n];
    let mut bounds = vec![0.0f64; n + 1];
    let mut k = 0;
    bounds[0] = f64::NEG_INFINITY;
    bounds[1] = f64::INFINITY;
    let intersection = |q: usize, p: usize| {
        let (q, p) = (q as f64, p as f64);
        ((f[q as usize] + q * q) - (f[p as usize] + p * p)) / (2.0 * q - 2.0 * p)
    };
    for q in 1..n {
        let mut s = intersection(q, vertices[k]);
        while s <= bounds[k] {
            k -= 1;
            s = intersection(q, vertices[k]);
        }
        k += 1;
        vertices[k] = q;
        bounds[k] = s;
        bounds[k + 1] = f64::INFINITY;
    }
    k = 0;
    for (q, value) in out.iter_mut().enumerate() {
        while bounds[k + 1] < q as f64 {
            k += 1;
        }
        let p = vertices[k];
        *value = (q as f64 - p as f64).powi(2) + f[p];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_edge_map_keeps_only_the_surface_of_a_block() {
        let mut map = OMap::new(7, 7);
        for y in 1..6 {
            for x in 1..6 {
                map.set(x, y, true);
            }
        }
        let edges = map.edge_map(true);
        assert!(edges.is_occupied(1, 1) && edges.is_occupied(5, 3));
        assert!(!edges.is_occupied(3, 3), "the inside can't be hit");
        assert!(!edges.is_occupied(0, 0), "free stays free");
    }

    #[test]
    fn the_distance_transform_is_euclidean() {
        let mut map = OMap::new(9, 6);
        map.set(2, 1, true);
        map.set(7, 4, true);
        let transform = DistanceTransform::new(&map);
        for y in 0..6 {
            for x in 0..9 {
                let expected = [(2.0f32, 1.0f32), (7.0, 4.0)]
                    .iter()
                    .map(|&(ox, oy)| (x as f32 - ox).hypot(y as f32 - oy))
                    .fold(f32::INFINITY, f32::min);
                assert!((transform.get(x, y) - expected).abs() < 1e-5, "({x}, {y})");
            }
        }
    }

    #[test]
    fn outside_the_map_is_free() {
        let mut map = OMap::new(2, 2);
        map.set(0, 0, true);
        assert!(map.is_occupied(0, 0));
        assert!(!map.is_occupied(-1, 0) && !map.is_occupied(0, 2));
        assert_eq!(map.free_pixels(), vec![(1, 0), (0, 1), (1, 1)]);
    }
}
