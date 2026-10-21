//! [`RayMarching`]: range_libc's `RayMarching` - steps along a ray by the
//! distance to the nearest wall, which can never overshoot one. Exact up
//! to a pixel, and the reference [`super::cddt`] is tested against.

use super::omap::{DistanceTransform, OMap};

/// range_libc's `RayMarching`, in pixels.
pub struct RayMarching {
    width: usize,
    height: usize,
    distances: DistanceTransform,
    max_range: f32,
}

/// A ray stops on a pixel this close to a wall - range_libc's
/// `distThreshold`.
const DIST_THRESHOLD: f32 = 0.0;
/// Fraction of the distance to the nearest wall stepped at once -
/// range_libc's `step_coeff`.
const STEP_COEFF: f32 = 0.999;

impl RayMarching {
    /// Casts on `map`, up to `max_range` pixels.
    pub fn new(map: &OMap, max_range: f32) -> Self {
        Self {
            width: map.width,
            height: map.height,
            distances: DistanceTransform::new(map),
            max_range,
        }
    }

    /// How far, in pixels, a ray from `(x, y)` along `heading` travels
    /// before it reaches a wall - `max_range` if it doesn't, or leaves the
    /// map. range_libc's `RayMarching::calc_range`.
    pub fn calc_range(&self, x: f32, y: f32, heading: f32) -> f32 {
        let (x0, y0) = (x, y);
        let (ray_direction_y, ray_direction_x) = heading.sin_cos();
        let mut t = 0.0f32;
        while t < self.max_range {
            // C++'s float-to-int conversion truncates toward zero, as `as`
            // does.
            let px = (x0 + ray_direction_x * t) as i64;
            let py = (y0 + ray_direction_y * t) as i64;
            if px >= self.width as i64 || px < 0 || py < 0 || py >= self.height as i64 {
                return self.max_range;
            }
            let d = self.distances.get(px as usize, py as usize);
            if d <= DIST_THRESHOLD {
                let xd = px as f32 - x0;
                let yd = py as f32 - y0;
                return (xd * xd + yd * yd).sqrt();
            }
            t += (d * STEP_COEFF).max(1.0);
        }
        self.max_range
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_ray_stops_at_the_wall_it_faces() {
        // A wall along column 20.
        let mut map = OMap::new(30, 30);
        for y in 0..30 {
            map.set(20, y, true);
        }
        let rm = RayMarching::new(&map, 100.0);
        let range = rm.calc_range(5.5, 10.5, 0.0);
        assert!((range - 14.5).abs() <= 1.0, "{range}");
        // Facing away: off the map.
        assert_eq!(rm.calc_range(5.5, 10.5, std::f32::consts::PI), 100.0);
    }
}
