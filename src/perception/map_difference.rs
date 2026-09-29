//! The pure steps of ubm's `detector_py` (`detector_py.py`,
//! `kalman_filter.py`) used by [`super::UbmDetector`]: finding the stretch
//! of a scan that's shorter than the map predicts ([`find_plateau`]),
//! rejecting one on a wall ([`near_wall`]), smoothing it over time
//! ([`Kalman`]) - plus a rectangle fit around its points ([`fit_rectangle`],
//! the closeness-criterion L-shape fit of ubm's `detector_cpp`), which
//! `detector_py` doesn't have.

use crate::topics::BoundingBox;

/// A stretch of rays `start..end` of the scan where something not on the
/// map is, as [`find_plateau`] finds it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Plateau {
    pub(crate) start: usize,
    pub(crate) end: usize,
    /// Median of the real ranges across it, in meters.
    pub(crate) median_range_m: f64,
}

/// What [`find_plateau`] accepts as an object.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct PlateauLimits {
    pub(crate) median_kernel_size: usize,
    /// `0` picks the threshold from the scan itself.
    pub(crate) gradient_threshold: f64,
    pub(crate) min_length: usize,
    pub(crate) max_std_m: f64,
    pub(crate) min_mean_difference_m: f64,
}

/// `detect_object_in_difference`: the best stretch where `real` is
/// shorter than `expected` (both one range per ray, same length) by a
/// consistent amount, or `None`.
///
/// The positive difference is median-filtered, then split at every jump
/// larger than the gradient threshold; of the stretches between jumps at
/// least `min_length` rays long, whose real ranges spread less than
/// `max_std_m` and which are on average at least `min_mean_difference_m`
/// shorter than expected, the one scoring lowest - long, flat and far in
/// front of the map - wins.
pub(crate) fn find_plateau(
    expected: &[f64],
    real: &[f64],
    limits: &PlateauLimits,
) -> Option<Plateau> {
    let n = expected.len().min(real.len());
    let min_length = limits.min_length.max(1);
    let difference: Vec<f64> = (0..n).map(|i| (expected[i] - real[i]).max(0.0)).collect();
    let filtered = median_filter(&difference, limits.median_kernel_size);
    let gradient: Vec<f64> = filtered.windows(2).map(|w| (w[1] - w[0]).abs()).collect();
    if gradient.is_empty() {
        return None;
    }

    let threshold = if limits.gradient_threshold > 0.0 {
        limits.gradient_threshold
    } else {
        let mean = gradient.iter().sum::<f64>() / gradient.len() as f64;
        let deviations: Vec<f64> = gradient.iter().map(|g| (g - mean).abs()).collect();
        median(&gradient) + 2.0 * median(&deviations)
    };
    let changes: Vec<usize> = (0..gradient.len())
        .filter(|&i| gradient[i] > threshold)
        .collect();
    let (&first, &last) = (changes.first()?, changes.last()?);

    let mut candidates = Vec::new();
    if first > min_length {
        candidates.push((0, first));
    }
    for pair in changes.windows(2) {
        let (start, end) = (pair[0] + 1, pair[1]);
        if end >= start + min_length {
            candidates.push((start, end));
        }
    }
    if n - last > min_length {
        candidates.push((last + 1, n));
    }

    let mut best: Option<((usize, usize), f64)> = None;
    for (start, end) in candidates {
        let spread = std_dev(&real[start..end]);
        let mean_difference =
            filtered[start..end].iter().map(|d| d.abs()).sum::<f64>() / (end - start) as f64;
        let score = spread / (1.0 + (end - start) as f64 / min_length as f64) - mean_difference;
        if spread < limits.max_std_m
            && mean_difference >= limits.min_mean_difference_m
            && best.is_none_or(|(_, best_score)| score < best_score)
        {
            best = Some(((start, end), score));
        }
    }
    let ((start, end), _) = best?;
    Some(Plateau {
        start,
        end,
        median_range_m: median(&real[start..end]),
    })
}

/// `values` median-filtered over a window of `kernel_size` (at least 1),
/// reflecting them at either end - scipy's `median_filter` with its default
/// `mode="reflect"`, as `detector_py` calls it.
pub(crate) fn median_filter(values: &[f64], kernel_size: usize) -> Vec<f64> {
    let n = values.len() as isize;
    let kernel_size = kernel_size.max(1) as isize;
    let reflected = |i: isize| {
        let mut i = i;
        // A kernel wider than twice the input can reflect more than once.
        loop {
            if i < 0 {
                i = -i - 1;
            } else if i >= n {
                i = 2 * n - i - 1;
            } else {
                return values[i as usize];
            }
        }
    };
    let mut window = Vec::with_capacity(kernel_size as usize);
    (0..n)
        .map(|i| {
            window.clear();
            window.extend((0..kernel_size).map(|k| reflected(i - kernel_size / 2 + k)));
            window.sort_by(f64::total_cmp);
            window[window.len() / 2]
        })
        .collect()
}

/// numpy's `median`: the middle value, or the mean of the two middle ones.
/// `0` for no values.
fn median(values: &[f64]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(f64::total_cmp);
    let mid = sorted.len() / 2;
    if sorted.len().is_multiple_of(2) {
        (sorted[mid - 1] + sorted[mid]) / 2.0
    } else {
        sorted[mid]
    }
}

/// numpy's (population) `std`.
fn std_dev(values: &[f64]) -> f64 {
    let n = values.len() as f64;
    let mean = values.iter().sum::<f64>() / n;
    (values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n).sqrt()
}

/// `has_neighbor_below_threshold`: whether any pixel within `radius_px`
/// (a square, clipped to the map) of the one under `(x_m, y_m)` isn't
/// drivable. `pixels` is row-major, rows growing along +y, `255` drivable -
/// like [`crate::topics::SelectedMap::pixels`]. A point off the map entirely
/// has no neighbors, so it's never near a wall.
pub(crate) fn near_wall(
    map: &crate::topics::SelectedMap,
    info: &crate::environment::MapInfo,
    x_m: f64,
    y_m: f64,
    radius_px: usize,
) -> bool {
    let col = ((x_m - info.origin.x) / info.resolution_m_per_px).floor() as i64;
    let row = ((y_m - info.origin.y) / info.resolution_m_per_px).floor() as i64;
    let radius = radius_px as i64;
    let (width, height) = (i64::from(info.width_px), i64::from(info.height_px));
    let rows = (row - radius).max(0)..(row + radius + 1).min(height);
    let cols = (col - radius).max(0)..(col + radius + 1).min(width);
    rows.into_iter().any(|r| {
        cols.clone()
            .any(|c| map.pixels[(r * width + c) as usize] != 255)
    })
}

/// One axis of `KalmanFilter2D`: position and velocity under a constant
/// velocity model, measured in position only. `detector_py`'s 4-state
/// filter keeps x and y apart in every matrix (they start, move and get
/// measured independently, with the same noises), so two of these are
/// exactly it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Kalman {
    pub(crate) position: f64,
    pub(crate) velocity: f64,
    /// Covariance of (position, velocity).
    covariance: [[f64; 2]; 2],
}

impl Default for Kalman {
    /// At rest at `0`, with unit covariance - as `KalmanFilter2D` starts.
    fn default() -> Self {
        Self {
            position: 0.0,
            velocity: 0.0,
            covariance: [[1.0, 0.0], [0.0, 1.0]],
        }
    }
}

impl Kalman {
    /// Moves the state `dt_s` forward, the velocity disturbed by white
    /// acceleration noise of standard deviation `process_noise`.
    pub(crate) fn predict(&mut self, dt_s: f64, process_noise: f64) {
        let q = process_noise * process_noise;
        let [[p00, p01], [p10, p11]] = self.covariance;
        self.position += dt_s * self.velocity;
        // F P F^T + Q, with F = [[1, dt], [0, 1]].
        self.covariance = [
            [
                p00 + dt_s * (p01 + p10) + dt_s * dt_s * p11 + 0.25 * dt_s.powi(4) * q,
                p01 + dt_s * p11 + 0.5 * dt_s.powi(3) * q,
            ],
            [
                p10 + dt_s * p11 + 0.5 * dt_s.powi(3) * q,
                p11 + dt_s * dt_s * q,
            ],
        ];
    }

    /// Corrects the state with a position `measured`, off by `measurement_noise`
    /// (standard deviation).
    pub(crate) fn update(&mut self, measured: f64, measurement_noise: f64) {
        let [[p00, p01], [p10, p11]] = self.covariance;
        let s = p00 + measurement_noise * measurement_noise;
        let (k0, k1) = (p00 / s, p10 / s);
        let innovation = measured - self.position;
        self.position += k0 * innovation;
        self.velocity += k1 * innovation;
        // (I - K H) P, with H = [1, 0].
        self.covariance = [
            [p00 - k0 * p00, p01 - k0 * p01],
            [p10 - k1 * p00, p11 - k1 * p01],
        ];
    }
}

/// The oriented rectangle around `points` whose sides they hug closest -
/// the closeness-criterion search of `detector_cpp`'s
/// `obs_point_clouds_2_obs_array` (Zhang et al., "Efficient L-Shape
/// Fitting for Vehicle Detection Using Laser Scanners"), trying every whole
/// degree in `[0, 90)`. Unlike `detector_cpp`, which keeps one side and
/// makes a square of it, both sides are kept. `min_distance_m` caps how
/// much a single point lying on a side can weigh. `None` for fewer than two
/// points.
pub(crate) fn fit_rectangle(points: &[[f64; 2]], min_distance_m: f64) -> Option<BoundingBox> {
    if points.len() < 2 {
        return None;
    }
    let min_distance_m = min_distance_m.max(1e-6);
    let project = |theta: f64| {
        let (sin, cos) = theta.sin_cos();
        let along: Vec<f64> = points.iter().map(|p| p[0] * cos + p[1] * sin).collect();
        let across: Vec<f64> = points.iter().map(|p| -p[0] * sin + p[1] * cos).collect();
        (along, across)
    };
    // Each point's distance to the nearer of an axis' two edges - the edge
    // being whichever of them the points hug closer, as a whole.
    let edge_distances = |values: &[f64]| {
        let (min, max) = bounds(values);
        let to_max: Vec<f64> = values.iter().map(|v| max - v).collect();
        let to_min: Vec<f64> = values.iter().map(|v| v - min).collect();
        let norm = |d: &[f64]| d.iter().map(|x| x * x).sum::<f64>();
        if norm(&to_max) <= norm(&to_min) {
            to_max
        } else {
            to_min
        }
    };

    let (best_degrees, _) = (0..90)
        .map(|degrees| {
            let (along, across) = project(f64::from(degrees).to_radians());
            let (d1, d2) = (edge_distances(&along), edge_distances(&across));
            let closeness: f64 = d1
                .iter()
                .zip(&d2)
                .map(|(a, b)| 1.0 / a.min(*b).max(min_distance_m))
                .sum();
            (degrees, closeness)
        })
        .fold((0, f64::NEG_INFINITY), |best, candidate| {
            if candidate.1 > best.1 {
                candidate
            } else {
                best
            }
        });

    let heading_rad = f64::from(best_degrees).to_radians();
    let (along, across) = project(heading_rad);
    let ((min_along, max_along), (min_across, max_across)) = (bounds(&along), bounds(&across));
    let (sin, cos) = heading_rad.sin_cos();
    let to_world = |a: f64, c: f64| [a * cos - c * sin, a * sin + c * cos];
    Some(BoundingBox {
        center: to_world(
            (min_along + max_along) / 2.0,
            (min_across + max_across) / 2.0,
        ),
        heading_rad,
        length_m: max_along - min_along,
        width_m: max_across - min_across,
        corners: [
            to_world(min_along, min_across),
            to_world(max_along, min_across),
            to_world(max_along, max_across),
            to_world(min_along, max_across),
        ],
    })
}

/// The smallest and largest of `values`.
fn bounds(values: &[f64]) -> (f64, f64) {
    values
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), &v| {
            (min.min(v), max.max(v))
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn limits() -> PlateauLimits {
        PlateauLimits {
            median_kernel_size: 5,
            gradient_threshold: 0.6,
            min_length: 5,
            max_std_m: 1.0,
            min_mean_difference_m: 0.15,
        }
    }

    #[test]
    fn median_filter_reflects_at_the_ends_and_drops_spikes() {
        assert_eq!(
            median_filter(&[1.0, 9.0, 1.0, 1.0], 3),
            vec![1.0, 1.0, 1.0, 1.0]
        );
        // Reflected: [2, 1 | 1, 2, 3 | 3, 2].
        assert_eq!(median_filter(&[1.0, 2.0, 3.0], 5), vec![2.0, 2.0, 2.0]);
        assert_eq!(median_filter(&[4.0, 1.0], 1), vec![4.0, 1.0]);
    }

    #[test]
    fn finds_the_stretch_where_the_scan_is_shorter_than_the_map() {
        let expected = vec![5.0; 60];
        let mut real = expected.clone();
        for range in &mut real[20..30] {
            *range = 2.0;
        }
        let plateau = find_plateau(&expected, &real, &limits()).unwrap();
        // The median filter blurs the edges by at most half its kernel.
        assert!((19..=21).contains(&plateau.start), "{plateau:?}");
        assert!((29..=31).contains(&plateau.end), "{plateau:?}");
        assert_eq!(plateau.median_range_m, 2.0);
    }

    #[test]
    fn nothing_when_the_scan_matches_the_map_or_differs_in_a_single_ray() {
        let expected: Vec<f64> = (0..60).map(|i| 3.0 + f64::from(i) * 0.01).collect();
        assert_eq!(find_plateau(&expected, &expected, &limits()), None);

        let mut real = expected.clone();
        real[30] = 0.5;
        assert_eq!(find_plateau(&expected, &real, &limits()), None);
    }

    #[test]
    fn kalman_locks_onto_a_constant_velocity() {
        let (mut filter, dt) = (Kalman::default(), 0.05);
        for step in 0..200 {
            filter.predict(dt, 5.0);
            filter.update(1.0 + 2.0 * dt * f64::from(step + 1), 0.2);
        }
        assert!((filter.velocity - 2.0).abs() < 0.05, "{filter:?}");
        assert!(
            (filter.position - (1.0 + 2.0 * dt * 200.0)).abs() < 0.05,
            "{filter:?}"
        );
    }

    #[test]
    fn fits_a_rotated_rectangle_from_its_two_visible_sides() {
        let (heading, length, width, center) = (30f64.to_radians(), 0.6, 0.3, [2.0, -1.0]);
        let (sin, cos) = heading.sin_cos();
        let to_world =
            |a: f64, c: f64| [center[0] + a * cos - c * sin, center[1] + a * sin + c * cos];
        // The two sides facing a lidar sitting toward -along, -across.
        let points: Vec<[f64; 2]> = (0..=20)
            .map(|i| to_world(-length / 2.0 + length * f64::from(i) / 20.0, -width / 2.0))
            .chain(
                (1..=10)
                    .map(|i| to_world(-length / 2.0, -width / 2.0 + width * f64::from(i) / 10.0)),
            )
            .collect();

        let fitted = fit_rectangle(&points, 0.01).unwrap();
        assert!((fitted.heading_rad - heading).abs() < 1e-9, "{fitted:?}");
        assert!((fitted.length_m - length).abs() < 1e-9, "{fitted:?}");
        assert!((fitted.width_m - width).abs() < 1e-9, "{fitted:?}");
        assert!((fitted.center[0] - center[0]).abs() < 1e-9, "{fitted:?}");
        assert!((fitted.center[1] - center[1]).abs() < 1e-9, "{fitted:?}");
    }

    #[test]
    fn no_rectangle_around_a_single_point() {
        assert_eq!(fit_rectangle(&[[1.0, 1.0]], 0.01), None);
    }
}
