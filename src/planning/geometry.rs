//! Helpers on closed loops of points - the shape every line here takes: the
//! last point's successor is the first, which is never repeated at the end.

pub use crate::environment::simulator::smoothing::{Point2, resample_even_spacing};

/// Length of the closed loop through `points`, including the closing
/// segment.
pub fn loop_length(points: &[Point2]) -> f64 {
    let n = points.len();
    (0..n)
        .map(|i| points[i].distance(&points[(i + 1) % n]))
        .sum()
}

/// Unit tangent at every point of the closed loop: the direction from its
/// predecessor to its successor (central difference). A degenerate
/// neighborhood (both neighbors on the point) gets `+x`.
pub fn tangents(points: &[Point2]) -> Vec<Point2> {
    let n = points.len();
    (0..n)
        .map(|i| {
            let prev = points[(i + n - 1) % n];
            let next = points[(i + 1) % n];
            normalized(next.x - prev.x, next.y - prev.y)
        })
        .collect()
}

/// `tangent` rotated a quarter turn counterclockwise: the unit normal
/// pointing to the left of the direction of travel - the same convention as
/// [`crate::environment::StartFinishLine`].
pub fn left_normal(tangent: Point2) -> Point2 {
    Point2 {
        x: -tangent.y,
        y: tangent.x,
    }
}

/// Signed curvature at every point of the closed loop, in 1/m: the Menger
/// curvature of the point and its two neighbors (the reciprocal of their
/// circumradius), positive when the loop turns left (counterclockwise).
pub fn curvatures(points: &[Point2]) -> Vec<f64> {
    let n = points.len();
    (0..n)
        .map(|i| menger_curvature(points[(i + n - 1) % n], points[i], points[(i + 1) % n]))
        .collect()
}

/// Largest curvature magnitude along the closed loop, in 1/m.
pub fn max_abs_curvature(points: &[Point2]) -> f64 {
    curvatures(points)
        .into_iter()
        .fold(0.0, |max, curvature| max.max(curvature.abs()))
}

/// Signed Menger curvature of `a`, `b`, `c`: `2 * cross(b - a, c - b) /
/// (|ab| * |bc| * |ca|)`. Zero for (near-)coincident points.
fn menger_curvature(a: Point2, b: Point2, c: Point2) -> f64 {
    let cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
    let denom = a.distance(&b) * b.distance(&c) * c.distance(&a);
    if denom < 1e-12 {
        0.0
    } else {
        2.0 * cross / denom
    }
}

/// The closed loop through `points` smoothed by a centered moving average
/// over `window` points (rounded up to an odd count), wrapping around the
/// seam. A window of 1 or less returns the points unchanged.
pub fn smooth(points: &[Point2], window: usize) -> Vec<Point2> {
    let n = points.len();
    // Never wider than the loop itself.
    let half = (window / 2).min(n.saturating_sub(1) / 2);
    if half == 0 {
        return points.to_vec();
    }
    let count = (2 * half + 1) as f64;
    (0..n)
        .map(|i| {
            let (mut x, mut y) = (0.0, 0.0);
            for offset in 0..=2 * half {
                let point = points[(i + n + offset - half) % n];
                x += point.x;
                y += point.y;
            }
            Point2 {
                x: x / count,
                y: y / count,
            }
        })
        .collect()
}

/// Makes the closed loop `points` run along `heading_rad` near
/// (`x_m`, `y_m`) and start at its point closest to there: reversed if it
/// runs the other way, then rotated. Used to start every line at the
/// start/finish line, in the direction of travel.
pub fn orient_from(points: &mut [Point2], x_m: f64, y_m: f64, heading_rad: f64) {
    if points.len() < 3 {
        return;
    }
    let here = Point2 { x: x_m, y: y_m };
    let closest = closest_index(points, here);
    let tangent = tangents(points)[closest];
    if tangent.x * heading_rad.cos() + tangent.y * heading_rad.sin() < 0.0 {
        points.reverse();
    }
    let closest = closest_index(points, here);
    points.rotate_left(closest);
}

/// Index of the point of `points` closest to `target`.
fn closest_index(points: &[Point2], target: Point2) -> usize {
    points
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| a.distance(&target).total_cmp(&b.distance(&target)))
        .map(|(index, _)| index)
        .unwrap_or(0)
}

/// `(x, y)` scaled to unit length, or `+x` if it's (near) zero.
fn normalized(x: f64, y: f64) -> Point2 {
    let length = (x * x + y * y).sqrt();
    if length < 1e-12 {
        Point2 { x: 1.0, y: 0.0 }
    } else {
        Point2 {
            x: x / length,
            y: y / length,
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// `n` points evenly spaced on the circle of `radius` around the origin,
    /// counterclockwise, starting on `+x`.
    pub fn circle(radius: f64, n: usize) -> Vec<Point2> {
        (0..n)
            .map(|i| {
                let angle = std::f64::consts::TAU * i as f64 / n as f64;
                Point2 {
                    x: radius * angle.cos(),
                    y: radius * angle.sin(),
                }
            })
            .collect()
    }

    #[test]
    fn a_counterclockwise_circle_has_positive_curvature_one_over_its_radius() {
        for curvature in curvatures(&circle(2.0, 200)) {
            assert!((curvature - 0.5).abs() < 1e-3, "{curvature}");
        }
        let mut clockwise = circle(2.0, 200);
        clockwise.reverse();
        assert!(
            curvatures(&clockwise)
                .iter()
                .all(|&k| (k + 0.5).abs() < 1e-3)
        );
    }

    #[test]
    fn a_circle_s_normals_point_to_its_center() {
        let points = circle(1.0, 100);
        for (point, tangent) in points.iter().zip(tangents(&points)) {
            let normal = left_normal(tangent);
            assert!((normal.x + point.x).abs() < 1e-3);
            assert!((normal.y + point.y).abs() < 1e-3);
        }
    }

    #[test]
    fn smoothing_keeps_a_straight_line_and_wraps_around() {
        let square: Vec<Point2> = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
            .iter()
            .map(|&(x, y)| Point2 { x, y })
            .collect();
        let smoothed = smooth(&square, 3);
        // Each corner averaged with its two neighbors (across the seam too).
        assert!((smoothed[0].x - 1.0 / 3.0).abs() < 1e-12);
        assert!((smoothed[0].y - 1.0 / 3.0).abs() < 1e-12);
        assert_eq!(smooth(&square, 1), square);
    }

    #[test]
    fn orienting_reverses_and_rotates_to_the_start() {
        let mut points = circle(1.0, 8);
        // At (0, -1), heading along -x: the circle runs +x there, so it's
        // reversed, then starts at (0, -1).
        orient_from(&mut points, 0.0, -1.0, std::f64::consts::PI);
        assert!(points[0].distance(&Point2 { x: 0.0, y: -1.0 }) < 1e-9);
        assert!(points[1].x < 0.0);
    }
}
