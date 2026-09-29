//! The pure steps of ubm's `detector_py` (`detector_py.py`,
//! `kalman_filter.py`) used by [`super::UbmDetector`]: finding the stretch
//! of a scan that's shorter than the map predicts ([`find_plateaus`]),
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

/// What [`find_plateaus`] accepts as an object.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct PlateauLimits {
    pub(crate) median_kernel_size: usize,
    /// `0` picks the threshold from the scan itself.
    pub(crate) gradient_threshold: f64,
    /// Narrowest a stretch may be, in meters across: its rays times
    /// `ray_step_rad` times its median real range. (ubm counted rays, which
    /// ties the threshold to one lidar's resolution.)
    pub(crate) min_width_m: f64,
    /// Angle between two consecutive rays of the scan.
    pub(crate) ray_step_rad: f64,
    pub(crate) max_std_m: f64,
    pub(crate) min_mean_difference_m: f64,
    /// Which acceptable stretch wins: `0` for the lowest score, or
    /// [`SELECTION_CLOSEST`].
    pub(crate) selection: u8,
}

/// A jump in the difference is an object's edge only if the (median-
/// filtered) real scan moves by at least this much there, in meters - see
/// [`find_plateaus`]. Well above the range noise, below the edges seen
/// between a car and what's behind it, 0.25 m and up.
const EDGE_MIN_REAL_STEP_M: f64 = 0.15;

/// [`PlateauLimits::selection`]: the closest, by median real range - the
/// opponent nearest the ego vehicle, e.g. for its MPC.
pub(crate) const SELECTION_CLOSEST: u8 = 1;

/// `detect_object_in_difference`: every stretch where `real` is shorter
/// than `expected` (both one range per ray, same length) by a consistent
/// amount, best first - so a caller rejecting one (e.g. on a wall) falls
/// back on the next rather than on nothing (ubm returned only the best).
///
/// The positive difference is median-filtered, then split at an object's
/// edges: every jump in it larger than the gradient threshold, either from a
/// level below that threshold (next to nothing in front of the map) or back,
/// or between two levels above it where the (median-filtered) real scan
/// moves by at least [`EDGE_MIN_REAL_STEP_M`] too (ubm split at every jump).
/// Of the stretches between them at least `min_width_m` wide, whose real
/// ranges spread less than `max_std_m` and which are on average at least
/// `min_mean_difference_m` shorter than expected, the ones scoring lowest - wide, flat and far in
/// front of the map - come first, or with [`SELECTION_CLOSEST`] the closest.
pub(crate) fn find_plateaus(
    expected: &[f64],
    real: &[f64],
    limits: &PlateauLimits,
) -> Vec<Plateau> {
    let n = expected.len().min(real.len());
    // Never 0: it divides the score.
    let min_width_m = limits.min_width_m.max(1e-3);
    let width_m = |start: usize, end: usize| {
        (end - start) as f64 * limits.ray_step_rad * median(&real[start..end])
    };
    let difference: Vec<f64> = (0..n).map(|i| (expected[i] - real[i]).max(0.0)).collect();
    let filtered = median_filter(&difference, limits.median_kernel_size);
    let gradient: Vec<f64> = filtered.windows(2).map(|w| (w[1] - w[0]).abs()).collect();
    // How much the real scan itself moves between two rays: a jump in the
    // difference between two levels of something in front of the map is
    // only an object's edge if the real scan moves too - not if it's only
    // the map jumping *behind* the object (a wall's corner), which ubm took
    // for an edge, cutting the object in pieces often too narrow to count.
    let smoothed_real = median_filter(&real[..n], limits.median_kernel_size);
    let real_step: Vec<f64> = smoothed_real
        .windows(2)
        .map(|w| (w[1] - w[0]).abs())
        .collect();
    if gradient.is_empty() {
        return Vec::new();
    }

    let threshold = if limits.gradient_threshold > 0.0 {
        limits.gradient_threshold
    } else {
        let mean = gradient.iter().sum::<f64>() / gradient.len() as f64;
        let deviations: Vec<f64> = gradient.iter().map(|g| (g - mean).abs()).collect();
        median(&gradient) + 2.0 * median(&deviations)
    };
    let changes: Vec<usize> = (0..gradient.len())
        .filter(|&i| {
            if gradient[i] <= threshold {
                return false;
            }
            // From (next to) nothing in front of the map to something, or
            // back - one side less than a jump's worth: an edge.
            if filtered[i].min(filtered[i + 1]) <= threshold {
                return true;
            }
            // Between two levels of something: only if the real scan steps
            // there too.
            real_step[i] >= EDGE_MIN_REAL_STEP_M
        })
        .collect();
    let (Some(&first), Some(&last)) = (changes.first(), changes.last()) else {
        return Vec::new();
    };

    // Before the first jump, between each two, and after the last.
    let candidates = std::iter::once((0, first))
        .chain(changes.windows(2).map(|pair| (pair[0] + 1, pair[1])))
        .chain(std::iter::once((last + 1, n)))
        .filter(|&(start, end)| end > start && width_m(start, end) >= min_width_m);

    let mut accepted: Vec<(Plateau, f64)> = candidates
        .filter_map(|(start, end)| {
            // Median-filtered, like the edges: a single ray that got no
            // return (or a stray one) doesn't make an object look spread.
            let spread = std_dev(&smoothed_real[start..end]);
            let mean_difference =
                filtered[start..end].iter().map(|d| d.abs()).sum::<f64>() / (end - start) as f64;
            let median_range_m = median(&smoothed_real[start..end]);
            let score = if limits.selection == SELECTION_CLOSEST {
                median_range_m
            } else {
                spread / (1.0 + width_m(start, end) / min_width_m) - mean_difference
            };
            (spread < limits.max_std_m && mean_difference >= limits.min_mean_difference_m)
                .then_some((
                    Plateau {
                        start,
                        end,
                        median_range_m,
                    },
                    score,
                ))
        })
        .collect();
    accepted.sort_by(|a, b| a.1.total_cmp(&b.1));
    accepted.into_iter().map(|(plateau, _)| plateau).collect()
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

    /// The best stretch, as ubm's `detect_object_in_difference` returned it.
    fn find_plateau(expected: &[f64], real: &[f64], limits: &PlateauLimits) -> Option<Plateau> {
        find_plateaus(expected, real, limits).into_iter().next()
    }

    fn limits() -> PlateauLimits {
        PlateauLimits {
            median_kernel_size: 5,
            gradient_threshold: 0.6,
            min_width_m: 0.15,
            // The simulated lidar: 360 rays over 4.2 rad.
            ray_step_rad: 4.2 / 359.0,
            max_std_m: 1.0,
            min_mean_difference_m: 0.15,
            selection: 0,
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
    fn the_closest_stretch_wins_when_selected() {
        // Down a long straight, an object 4 m ahead of a wall 10 m away;
        // to the side, one 1 m ahead of a wall 2 m away.
        let mut expected = vec![10.0; 90];
        expected[50..90].fill(2.0);
        let mut real = expected.clone();
        real[10..25].fill(4.0);
        real[60..75].fill(1.0);
        // ubm's score favors the larger difference: the far one.
        let best = find_plateau(&expected, &real, &limits()).unwrap();
        assert_eq!(best.median_range_m, 4.0, "{best:?}");
        let closest = PlateauLimits {
            selection: SELECTION_CLOSEST,
            ..limits()
        };
        let closest = find_plateau(&expected, &real, &closest).unwrap();
        assert_eq!(closest.median_range_m, 1.0, "{closest:?}");
    }

    /// A scan of the simulated lidar (360 rays over 4.2 rad, straight ahead
    /// in the middle) of a wall `wall_m` away, with a `width_m` wide object
    /// square to it `distance_m` straight ahead.
    fn object_ahead(distance_m: f64, width_m: f64, wall_m: f64) -> (Vec<f64>, Vec<f64>) {
        let expected = vec![wall_m; 360];
        let real = (0..360)
            .map(|i| {
                let angle = -2.1 + i as f64 * 4.2 / 359.0;
                let across = distance_m * angle.tan();
                if angle.abs() < 1.2 && across.abs() <= width_m / 2.0 {
                    distance_m / angle.cos()
                } else {
                    wall_m
                }
            })
            .collect();
        (expected, real)
    }

    #[test]
    fn a_car_is_found_however_few_rays_it_takes() {
        // A car's rear, 0.25 m wide: 4 rays of the simulated lidar from
        // 4.5 m on - ubm's 5-ray minimum lost it there.
        for distance_m in [1.0, 2.0, 3.0, 4.5, 5.0, 6.0, 7.0] {
            let (expected, real) = object_ahead(distance_m, 0.25, 10.0);
            let plateau = find_plateau(&expected, &real, &limits());
            let plateau = plateau.unwrap_or_else(|| panic!("nothing found at {distance_m} m"));
            assert!(
                (plateau.median_range_m - distance_m).abs() < 0.01,
                "{plateau:?}"
            );
        }
    }

    #[test]
    fn a_corner_behind_an_object_does_not_cut_it_in_two() {
        // A car's rear 1.4 m ahead, over a wall whose range jumps from 7 m to
        // 10 m right behind its middle: split on the difference, as ubm
        // did, it made two stretches narrower than min_width_m.
        let (mut expected, real) = object_ahead(1.4, 0.25, 10.0);
        expected[..180].fill(7.0);
        let plateau = find_plateau(&expected, &real, &limits()).expect("the car");
        assert!((plateau.median_range_m - 1.4).abs() < 0.01, "{plateau:?}");
        assert!(plateau.start < 178 && plateau.end > 182, "{plateau:?}");
    }

    #[test]
    fn an_object_at_the_range_of_the_wall_beside_it_is_found() {
        // A wall seen at a grazing angle, its range climbing to 6 m, then a
        // car at 5.2 m: the real scan barely steps at the car's edge (less
        // than the gradient threshold), the difference does.
        let mut expected: Vec<f64> = (0..360).map(|i| 3.0 + i as f64 * 0.02).collect();
        let mut real = expected.clone();
        for i in 200..205 {
            expected[i] = 7.0;
            real[i] = 5.2;
        }
        for i in 205..360 {
            expected[i] = 12.0;
            real[i] = 12.0;
        }
        let plateau = find_plateau(&expected, &real, &limits()).expect("the car");
        assert!((plateau.median_range_m - 5.2).abs() < 0.01, "{plateau:?}");
    }

    #[test]
    fn an_object_at_the_range_of_a_wall_ending_beside_it_is_found() {
        // A wall ending 4.3 m away, then a car at 4.3 m in front of open
        // space: the real scan doesn't step at all at the car's edge - only
        // the difference, from nothing to something.
        let expected: Vec<f64> = (0..360).map(|i| if i < 200 { 4.3 } else { 12.0 }).collect();
        let real: Vec<f64> = (0..360).map(|i| if i < 206 { 4.3 } else { 12.0 }).collect();
        let plateau = find_plateau(&expected, &real, &limits()).expect("the car");
        assert!((plateau.median_range_m - 4.3).abs() < 0.01, "{plateau:?}");
        assert!(plateau.start >= 199 && plateau.end <= 207, "{plateau:?}");
    }

    #[test]
    fn a_corner_behind_a_narrow_object_near_its_edge_does_not_cut_it() {
        // 6 rays of car 3.2 m ahead, over open space that becomes a wall
        // 6 m away two rays before the car's far edge.
        let (mut expected, real) = object_ahead(3.2, 0.25, 12.0);
        let rays: Vec<usize> = (0..360).filter(|&i| real[i] < 12.0).collect();
        let last = *rays.last().unwrap();
        expected[last - 1..].fill(6.0);
        let plateau = find_plateau(&expected, &real, &limits()).expect("the car");
        assert!((plateau.median_range_m - 3.2).abs() < 0.05, "{plateau:?}");
    }

    #[test]
    fn too_narrow_an_object_is_not_one() {
        // 0.08 m across at 1 m: 7 rays, plenty for ubm's 5-ray minimum.
        let (expected, real) = object_ahead(1.0, 0.08, 10.0);
        assert_eq!(find_plateau(&expected, &real, &limits()), None);
        let wide_enough = PlateauLimits {
            min_width_m: 0.05,
            ..limits()
        };
        assert!(find_plateau(&expected, &real, &wide_enough).is_some());
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
