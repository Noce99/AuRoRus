//! Per-point speed profile from local curvature: sharper turns get a lower
//! target speed. See [`assign_speeds`].

use crate::environment::race_line::SpeedPoint;
use crate::geometry::Point2;

/// Assigns a target speed to every point of a closed, evenly-spaced
/// centerline, from local curvature: `v = min(max_speed_mps,
/// sqrt(max_lateral_accel_mps2 / curvature))`, or `max_speed_mps` when
/// curvature is ~0 (a straight). Curvature is the discrete Menger curvature
/// of each point and its two neighbors (the reciprocal of the circumradius
/// of the triangle they form).
///
/// Deliberate v1 simplification: this has no forward/backward
/// acceleration-limiting pass, so speed can in principle jump between
/// adjacent points faster than `max_lateral_accel_mps2` would really allow
/// under braking/acceleration - only the *lateral* (cornering) limit is
/// modeled.
pub fn assign_speeds(
    closed_points: &[Point2],
    max_speed_mps: f64,
    max_lateral_accel_mps2: f64,
) -> Vec<SpeedPoint> {
    let n = closed_points.len();
    (0..n)
        .map(|i| {
            let prev = closed_points[(i + n - 1) % n];
            let curr = closed_points[i];
            let next = closed_points[(i + 1) % n];
            let curvature = menger_curvature(prev, curr, next);
            let speed = if curvature > 1e-9 {
                (max_lateral_accel_mps2 / curvature)
                    .sqrt()
                    .min(max_speed_mps)
            } else {
                max_speed_mps
            };
            SpeedPoint {
                x: curr.x,
                y: curr.y,
                speed_mps: speed,
            }
        })
        .collect()
}

/// Menger curvature of the triangle formed by `a`, `b`, `c`: `4 * area /
/// (|ab| * |bc| * |ca|)`. Near-zero for near-collinear points.
fn menger_curvature(a: Point2, b: Point2, c: Point2) -> f64 {
    let area = ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)).abs() / 2.0;
    let denom = a.distance(&b) * b.distance(&c) * c.distance(&a);
    if denom < 1e-12 {
        0.0
    } else {
        4.0 * area / denom
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_circle(radius: f64, n: usize) -> Vec<Point2> {
        (0..n)
            .map(|i| {
                let angle = 2.0 * std::f64::consts::PI * i as f64 / n as f64;
                Point2 {
                    x: radius * angle.cos(),
                    y: radius * angle.sin(),
                }
            })
            .collect()
    }

    #[test]
    fn collinear_points_get_max_speed() {
        let points = vec![
            Point2 { x: 0.0, y: 0.0 },
            Point2 { x: 1.0, y: 0.0 },
            Point2 { x: 2.0, y: 0.0 },
        ];
        let speeds = assign_speeds(&points, 10.0, 5.0);
        assert!((speeds[1].speed_mps - 10.0).abs() < 1e-9);
    }

    #[test]
    fn circle_matches_closed_form_curvature_speed() {
        let radius = 4.0;
        let points = make_circle(radius, 60);
        let speeds = assign_speeds(&points, 100.0, 5.0);
        let expected = (5.0 * radius).sqrt();
        for sp in &speeds {
            assert!(
                (sp.speed_mps - expected).abs() < 0.05,
                "{} vs {expected}",
                sp.speed_mps
            );
        }
    }

    #[test]
    fn speeds_stay_within_bounds() {
        let points = make_circle(1.0, 30);
        let speeds = assign_speeds(&points, 8.0, 6.0);
        for sp in &speeds {
            assert!(sp.speed_mps > 0.0 && sp.speed_mps <= 8.0 + 1e-9);
        }
    }
}
