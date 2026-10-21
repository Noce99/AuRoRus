//! [`Cddt`]: Compressed Directional Distance Transform ray casting - a
//! faithful port of range_libc's `CDDTCast` (`includes/RangeLib.h`, Corey
//! H. Walsh, Apache-2.0), the ray caster UBM's particle filter uses by
//! default.
//!
//! For each of `theta_discretization / 2` directions, the map is rotated so
//! rays along that direction run along the x axis of "LUT space", and
//! every wall pixel's x coordinate is stored in the row ("bin") of LUT
//! space it covers. A ray's range is then the gap between its own x and
//! the next wall entry in its bin, found by a binary or linear search.
//! The other half of the directions is the same bins searched backwards.
//!
//! Ported with range_libc's default compile-time flags:
//! `_USE_CACHED_CONSTANTS = 1`, `_USE_ALTERNATE_MOD = 1`,
//! `_USE_FAST_ROUND = 0`, `_USE_CACHED_TRIG = 0`, `_USE_LRU_CACHE = 0`, no
//! trace map, no collision index tracking. The comments name the C++ each
//! part comes from, so the two can be read side by side. Where the C++
//! reads outside its vectors (a pose outside the map), this returns
//! `max_range` or treats the pixel as free instead.

use super::omap::OMap;

/// range_libc's `_EPSILON`.
const EPSILON: f64 = 0.00001;
/// range_libc's `M_2PI` - a literal, not `2 * pi`, kept as is so the
/// discretization rounds exactly as the C++ does.
#[allow(clippy::approx_constant)]
const M_2PI: f64 = 6.28318530718;
/// Bins with more entries than this are searched with a binary search -
/// range_libc's `_BINARY_SEARCH_THRESHOLD`.
const BINARY_SEARCH_THRESHOLD: usize = 64;

/// range_libc's `CDDTCast`, in pixels.
pub struct Cddt {
    map: OMap,
    max_range: f32,
    theta_discretization: u32,
    /// `[angle][bin]`: the sorted LUT-space x of every wall pixel in that
    /// bin. Only the first half of the angles is ever filled.
    compressed_lut: Vec<Vec<Vec<f32>>>,
    lut_translations: Vec<f32>,
    theta_discretization_div_m_2pi: f32,
    m_2pi_div_theta_discretization: f32,
}

impl Cddt {
    /// Builds the LUT for `map`, casting up to `max_range` pixels, with
    /// `theta_discretization` directions over the full circle. range_libc's
    /// `CDDTCast::CDDTCast`.
    pub fn new(map: &OMap, max_range: f32, theta_discretization: u32) -> Self {
        let theta_discretization = theta_discretization.max(2);
        let theta_discretization_div_m_2pi = (f64::from(theta_discretization) / M_2PI) as f32;
        let m_2pi_div_theta_discretization = (M_2PI / f64::from(theta_discretization)) as f32;
        let (width, height) = (map.width as f32, map.height as f32);

        let mut lut_widths = Vec::new();
        let mut angles = Vec::new();
        let mut lut_translations = Vec::new();
        for i in 0..theta_discretization {
            let angle = i as f32 * m_2pi_div_theta_discretization;
            angles.push(angle);
            // The height of the rotated map's bounding box is the LUT's
            // width for this angle.
            let rotated_height = (width * angle.sin()).abs() + (height * angle.cos()).abs();
            lut_widths.push((f64::from(rotated_height) - EPSILON).ceil().max(0.0) as usize);
            // Shift LUT space so every corner's y is positive.
            let left_top_corner_y = height * angle.cos();
            let right_top_corner_y = width * angle.sin() + height * angle.cos();
            let right_bottom_corner_y = width * angle.sin();
            let min_corner_y = left_top_corner_y
                .min(right_top_corner_y)
                .min(right_bottom_corner_y);
            lut_translations.push((-f64::from(min_corner_y) - EPSILON).max(0.0) as f32);
        }

        let mut compressed_lut: Vec<Vec<Vec<f32>>> = lut_widths
            .iter()
            .map(|&lut_width| vec![Vec::new(); lut_width])
            .collect();

        // Only the surface of the walls can be hit.
        let edge_map = map.edge_map(true);
        let half = half_angles(theta_discretization);
        for x in 0..map.width {
            for y in 0..map.height {
                if !edge_map.is_occupied(x as i64, y as i64) {
                    continue;
                }
                let pixel_center = (x as f32 + 0.5, y as f32 + 0.5);
                for a in 0..half {
                    let (sinangle, cosangle) = angles[a].sin_cos();
                    let half_lut_space_width = (sinangle.abs() + cosangle.abs()) / 2.0;
                    let lut_space_center_x = pixel_center.0 * cosangle - pixel_center.1 * sinangle;
                    let lut_space_center_y = (pixel_center.0 * sinangle
                        + pixel_center.1 * cosangle)
                        + lut_translations[a];
                    let upper_bin =
                        (f64::from(lut_space_center_y + half_lut_space_width) - EPSILON) as i64;
                    let lower_bin =
                        (f64::from(lut_space_center_y - half_lut_space_width) + EPSILON) as i64;
                    let bins = &mut compressed_lut[a];
                    for i in lower_bin.max(0)..=upper_bin.min(bins.len() as i64 - 1) {
                        bins[i as usize].push(lut_space_center_x);
                    }
                }
            }
        }

        // Sorted for the searches, without duplicates: they can't change
        // the answer.
        for bins in &mut compressed_lut {
            for bin in bins {
                bin.sort_by(f32::total_cmp);
                bin.dedup();
            }
        }

        Self {
            map: map.clone(),
            max_range,
            theta_discretization,
            compressed_lut,
            lut_translations,
            theta_discretization_div_m_2pi,
            m_2pi_div_theta_discretization,
        }
    }

    /// Drops every LUT entry no ray from a free pixel's center within
    /// `max_range` can stop at - range_libc's `CDDTCast::prune`. Makes the
    /// bins shorter, and so the searches faster, without changing any range
    /// cast from a pixel center.
    pub fn prune(&mut self, max_range: f32) {
        let half = half_angles(self.theta_discretization);
        let mut used: Vec<Vec<Vec<bool>>> = self.compressed_lut[..half]
            .iter()
            .map(|bins| bins.iter().map(|bin| vec![false; bin.len()]).collect())
            .collect();

        for (angle_index, used) in used.iter_mut().enumerate() {
            let angle = angle_index as f32 * self.m_2pi_div_theta_discretization;
            let (sinangle, cosangle) = angle.sin_cos();
            let translation = self.lut_translations[angle_index];
            for x in 0..self.map.width {
                let x_center = 0.5 + x as f32;
                for y in 0..self.map.height {
                    let y_center = 0.5 + y as f32;
                    let lut_space_x = x_center * cosangle - y_center * sinangle;
                    let lut_space_y = (x_center * sinangle + y_center * cosangle) + translation;
                    let lut_index = lut_space_y as i64;
                    if lut_index < 0 || lut_index as usize >= used.len() {
                        continue;
                    }
                    let lut_index = lut_index as usize;
                    let lut_bin = &self.compressed_lut[angle_index][lut_index];
                    // There are no entries in this bin.
                    let Some(&furthest) = lut_bin.last() else {
                        continue;
                    };
                    if self.map.is_occupied(x as i64, y as i64) {
                        continue;
                    }
                    let marks = &mut used[lut_index];
                    if furthest < lut_space_x && lut_space_x - furthest < max_range {
                        marks[lut_bin.len() - 1] = true;
                        continue;
                    }
                    // The next entry at or past the query point (the C++'s
                    // `lower_bound`, or its linear search, which agree).
                    // Past every entry, only the last one is marked, as the
                    // C++'s out-of-range insert is never looked up.
                    let index = lut_bin.partition_point(|&entry| entry < lut_space_x);
                    if index < marks.len() {
                        marks[index] = true;
                    }
                    if index > 0 {
                        marks[index - 1] = true;
                    }
                }
            }
        }

        for (bins, used) in self.compressed_lut.iter_mut().zip(used) {
            for (bin, marks) in bins.iter_mut().zip(used) {
                let mut marks = marks.into_iter();
                bin.retain(|_| marks.next().unwrap_or(false));
            }
        }
    }

    /// The nearest discrete direction to `theta` - its index in the first
    /// half, the angle of that index, and whether the ray actually runs the
    /// opposite way. range_libc's `CDDTCast::discretize_theta`.
    fn discretize_theta(&self, mut theta: f32) -> (usize, f32, bool) {
        // _USE_ALTERNATE_MOD
        let m_2pi = M_2PI as f32;
        if theta < 0.0 {
            while theta < 0.0 {
                theta += m_2pi;
            }
        } else if theta > m_2pi {
            while theta > m_2pi {
                theta -= m_2pi;
            }
        }
        // Rotational symmetry: fold [pi, 2 pi) onto [0, pi).
        let mut is_flipped = false;
        if f64::from(theta) >= std::f64::consts::PI {
            is_flipped = true;
            theta = (f64::from(theta) - std::f64::consts::PI) as f32;
        }
        let mut rounded = (theta * self.theta_discretization_div_m_2pi).round() as i64;
        // Rounding up to pi wraps around to 0, flipped.
        if rounded == i64::from(self.theta_discretization >> 1) {
            rounded = 0;
            is_flipped = !is_flipped;
        }
        let binned = rounded.rem_euclid(i64::from(self.theta_discretization)) as usize;
        let discrete_angle = binned as f32 * self.m_2pi_div_theta_discretization;
        (binned, discrete_angle, is_flipped)
    }

    /// How far, in pixels, a ray from `(x, y)` along `heading` travels
    /// before it hits a wall - `max_range` if none is in its bin that way.
    /// Unlike [`super::ray_marching::RayMarching`], a wall farther than
    /// `max_range` is still reported at its distance. range_libc's
    /// `CDDTCast::calc_range`.
    pub fn calc_range(&self, x: f32, y: f32, heading: f32) -> f32 {
        let (angle_index, discrete_theta, is_flipped) = self.discretize_theta(-heading);
        let (sinangle, cosangle) = discrete_theta.sin_cos();
        let lut_space_x = x * cosangle - y * sinangle;
        let lut_space_y = (x * sinangle + y * cosangle) + self.lut_translations[angle_index];

        let lut_index = lut_space_y as i64;
        let bins = &self.compressed_lut[angle_index];
        if lut_index < 0 || lut_index as usize >= bins.len() {
            return self.max_range;
        }
        let lut_bin = &bins[lut_index as usize];
        let Some((&low, &high_value)) = lut_bin.first().zip(lut_bin.last()) else {
            // There are no entries in this bin.
            return self.max_range;
        };
        let high = lut_bin.len() - 1;
        let inside_wall = || self.map.is_occupied(x as i64, y as i64);

        if is_flipped {
            // The ray runs toward decreasing LUT-space x.
            if low > lut_space_x {
                return self.max_range;
            }
            if high_value < lut_space_x {
                return lut_space_x - high_value;
            }
            if inside_wall() {
                return 0.0;
            }
            if high > BINARY_SEARCH_THRESHOLD {
                // std::upper_bound
                let index = lut_bin.partition_point(|&entry| entry <= lut_space_x);
                lut_space_x - lut_bin[index - 1]
            } else {
                lut_bin
                    .iter()
                    .rev()
                    .find(|&&obstacle_x| obstacle_x <= lut_space_x)
                    .map_or(self.max_range, |&obstacle_x| lut_space_x - obstacle_x)
            }
        } else {
            if high_value < lut_space_x {
                return self.max_range;
            }
            if low > lut_space_x {
                return low - lut_space_x;
            }
            if inside_wall() {
                return 0.0;
            }
            if high > BINARY_SEARCH_THRESHOLD {
                // std::upper_bound: the first entry strictly past the query
                // point - unlike the linear search's "at or past" below,
                // exactly as in the C++.
                // Past the last entry when it's exactly at the query point,
                // where the C++ reads out of bounds: a zero range.
                let index = lut_bin.partition_point(|&entry| entry <= lut_space_x);
                lut_bin.get(index).unwrap_or(&high_value) - lut_space_x
            } else {
                lut_bin
                    .iter()
                    .find(|&&obstacle_x| obstacle_x >= lut_space_x)
                    .map_or(self.max_range, |&obstacle_x| obstacle_x - lut_space_x)
            }
        }
    }

    /// The number of LUT entries, across every direction and bin.
    #[cfg(test)]
    pub fn lut_entries(&self) -> usize {
        self.compressed_lut
            .iter()
            .flat_map(|bins| bins.iter().map(Vec::len))
            .sum()
    }
}

/// The directions the LUT is filled for: range_libc's `a <
/// theta_discretization / 2.0`.
fn half_angles(theta_discretization: u32) -> usize {
    theta_discretization.div_ceil(2) as usize
}

#[cfg(test)]
mod tests {
    use super::super::ray_marching::RayMarching;
    use super::*;
    use rand::rngs::StdRng;
    use rand::{RngExt, SeedableRng};

    /// A 120 x 90 box with walls two pixels thick, a pillar, and a slanted
    /// wall - every direction hits something.
    fn room() -> OMap {
        let (width, height) = (120, 90);
        let mut map = OMap::new(width, height);
        for y in 0..height {
            for x in 0..width {
                let border = x < 2 || y < 2 || x >= width - 2 || y >= height - 2;
                let pillar = (x as f32 - 40.0).hypot(y as f32 - 50.0) < 8.0;
                let slanted = (x as i64 - y as i64 - 50).abs() <= 1 && x > 70 && y < 40;
                if border || pillar || slanted {
                    map.set(x, y, true);
                }
            }
        }
        map
    }

    /// Random pixel centers in free space, and random headings.
    fn queries(map: &OMap, count: usize) -> Vec<(f32, f32, f32)> {
        let mut rng = StdRng::seed_from_u64(3);
        let mut queries = Vec::new();
        while queries.len() < count {
            let x = rng.random_range(0..map.width);
            let y = rng.random_range(0..map.height);
            if map.is_occupied(x as i64, y as i64) {
                continue;
            }
            let heading = rng.random_range(-std::f32::consts::PI..std::f32::consts::PI);
            queries.push((x as f32 + 0.5, y as f32 + 0.5, heading));
        }
        queries
    }

    /// CDDT never sees through a wall, and agrees with ray marching but for
    /// rays grazing a wall: a bin is about a pixel wide, so CDDT stops at
    /// a wall pixel up to a pixel and a half to the side of the ray, where
    /// ray marching slips past.
    #[test]
    fn cddt_agrees_with_ray_marching_but_for_grazing_rays() {
        let map = room();
        let theta_discretization = 360;
        let cddt = Cddt::new(&map, 500.0, theta_discretization);
        let rm = RayMarching::new(&map, 500.0);
        let step = std::f32::consts::TAU / theta_discretization as f32;
        let mut errors = Vec::new();
        for (x, y, heading) in queries(&map, 3000) {
            // Cast along CDDT's own discrete direction, so only the
            // discretization of space remains.
            let discrete = (heading / step).round() * step;
            let expected = rm.calc_range(x, y, discrete);
            let range = cddt.calc_range(x, y, discrete);
            assert!(
                range <= expected + 1.5,
                "({x}, {y}, {discrete}): CDDT {range} past ray marching's {expected}"
            );
            if expected - range > 2.0 {
                let (sin, cos) = discrete.sin_cos();
                let grazed = [-1.5f32, -0.75, 0.75, 1.5].iter().any(|&offset| {
                    rm.calc_range(x - offset * sin, y + offset * cos, discrete) <= range + 1.5
                });
                assert!(
                    grazed,
                    "({x}, {y}, {discrete}): CDDT {range}, ray marching {expected}"
                );
            }
            errors.push((range - expected).abs());
        }
        errors.sort_by(f32::total_cmp);
        let median = errors[errors.len() / 2];
        let p90 = errors[errors.len() * 9 / 10];
        assert!(median <= 1.0, "median error {median} px");
        assert!(p90 <= 2.0, "90th percentile error {p90} px");
    }

    #[test]
    fn a_heading_between_two_directions_lands_near_the_wall() {
        // A coarse discretization only moves the hit along the wall by the
        // heading error.
        let map = room();
        let cddt = Cddt::new(&map, 500.0, 112);
        // Facing +x above the pillar: the right wall's inner face is at 118.
        let range = cddt.calc_range(10.5, 70.5, 0.01);
        assert!((range - 107.5).abs() < 2.0, "{range}");
        // Facing -x: the left wall's inner face is at 2.
        let range = cddt.calc_range(10.5, 70.5, std::f32::consts::PI);
        assert!((range - 8.5).abs() < 1.5, "{range}");
        // Facing +y: the top wall at 88.
        let range = cddt.calc_range(10.5, 70.5, std::f32::consts::FRAC_PI_2);
        assert!((range - 17.5).abs() < 1.5, "{range}");
        // Facing -y: the bottom wall's inner face is at 2.
        let range = cddt.calc_range(10.5, 70.5, -std::f32::consts::FRAC_PI_2);
        assert!((range - 68.5).abs() < 1.5, "{range}");
    }

    #[test]
    fn pruning_keeps_every_range_from_a_pixel_center() {
        let map = room();
        let mut cddt = Cddt::new(&map, 500.0, 112);
        let queries = queries(&map, 2000);
        let before: Vec<f32> = queries
            .iter()
            .map(|&(x, y, heading)| cddt.calc_range(x, y, heading))
            .collect();
        let entries = cddt.lut_entries();
        cddt.prune(500.0);
        assert!(cddt.lut_entries() < entries, "nothing pruned");
        for (&(x, y, heading), &expected) in queries.iter().zip(&before) {
            let range = cddt.calc_range(x, y, heading);
            assert!(
                (range - expected).abs() < 1e-3,
                "({x}, {y}, {heading}): {range} != {expected}"
            );
        }
    }

    #[test]
    fn a_ray_from_inside_a_wall_has_zero_range() {
        let map = room();
        let cddt = Cddt::new(&map, 500.0, 112);
        assert_eq!(cddt.calc_range(40.5, 50.5, 0.3), 0.0);
    }

    #[test]
    fn a_ray_off_the_map_has_max_range() {
        let map = room();
        let cddt = Cddt::new(&map, 500.0, 112);
        assert_eq!(cddt.calc_range(-1000.0, -1000.0, 0.0), 500.0);
    }
}
