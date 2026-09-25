//! A speed for every point of a closed line: as fast as the lateral
//! acceleration limit allows in each corner, capped at a top speed, then
//! slowed wherever braking or accelerating between neighboring points
//! would take more than the longitudinal limits - the classic
//! forward/backward pass for a point mass (the lateral and longitudinal
//! limits are applied separately, not as a combined friction ellipse).

use super::geometry::{Point2, curvatures};

/// Every limit [`speeds`] respects.
#[derive(Debug, Clone, Copy)]
pub struct SpeedLimits {
    /// Top speed, in m/s.
    pub max_speed_mps: f64,
    /// Largest lateral (centripetal) acceleration, in m/s^2.
    pub max_lateral_accel_mps2: f64,
    /// Largest forward acceleration, in m/s^2.
    pub max_accel_mps2: f64,
    /// Largest braking deceleration, in m/s^2 (positive).
    pub max_decel_mps2: f64,
}

/// Most full backward+forward rounds [`speeds`] makes before giving up on
/// the profile settling - two always suffice on a closed loop, so this is
/// only a guard.
const MAX_ROUNDS: usize = 10;

/// The speed at every point of the closed loop `points`, in m/s - see the
/// module docs.
pub fn speeds(points: &[Point2], limits: &SpeedLimits) -> Vec<f64> {
    let n = points.len();
    let mut speeds: Vec<f64> = curvatures(points)
        .into_iter()
        .map(|curvature| {
            if curvature.abs() > 1e-9 {
                (limits.max_lateral_accel_mps2 / curvature.abs())
                    .sqrt()
                    .min(limits.max_speed_mps)
            } else {
                limits.max_speed_mps
            }
        })
        .collect();
    if n < 2 {
        return speeds;
    }
    let steps: Vec<f64> = (0..n)
        .map(|i| points[i].distance(&points[(i + 1) % n]))
        .collect();

    // v_next^2 <= v^2 + 2 a d, both ways, wrapping around the seam until
    // nothing changes.
    for _ in 0..MAX_ROUNDS {
        let mut changed = false;
        for i in (0..n).rev() {
            let reachable =
                (speeds[(i + 1) % n].powi(2) + 2.0 * limits.max_decel_mps2 * steps[i]).sqrt();
            if reachable < speeds[i] {
                speeds[i] = reachable;
                changed = true;
            }
        }
        for i in 0..n {
            let next = (i + 1) % n;
            let reachable = (speeds[i].powi(2) + 2.0 * limits.max_accel_mps2 * steps[i]).sqrt();
            if reachable < speeds[next] {
                speeds[next] = reachable;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    speeds
}

/// Time to drive one lap of the closed loop `points` at `speeds`, in
/// seconds: every segment at the mean of its two ends' speeds. Infinite if
/// a segment would be driven at zero speed.
pub fn lap_time(points: &[Point2], speeds: &[f64]) -> f64 {
    let n = points.len();
    (0..n)
        .map(|i| {
            let mean = (speeds[i] + speeds[(i + 1) % n]) / 2.0;
            let step = points[i].distance(&points[(i + 1) % n]);
            if mean > 0.0 {
                step / mean
            } else {
                f64::INFINITY
            }
        })
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::geometry::tests::circle;

    fn limits() -> SpeedLimits {
        SpeedLimits {
            max_speed_mps: 8.0,
            max_lateral_accel_mps2: 4.0,
            max_accel_mps2: 3.0,
            max_decel_mps2: 5.0,
        }
    }

    #[test]
    fn a_circle_is_driven_at_its_cornering_limit() {
        // v = sqrt(a * r) = sqrt(4 * 2) everywhere.
        for speed in speeds(&circle(2.0, 300), &limits()) {
            assert!((speed - 8f64.sqrt()).abs() < 1e-2, "{speed}");
        }
    }

    #[test]
    fn a_wide_circle_is_capped_at_top_speed() {
        for speed in speeds(&circle(100.0, 3000), &limits()) {
            assert!((speed - 8.0).abs() < 1e-9);
        }
    }

    /// A stadium: two straights joined by tight half circles.
    fn stadium() -> Vec<Point2> {
        let mut points = Vec::new();
        let (straight, radius, step) = (20.0, 1.0, 0.05);
        let along = (straight / step) as usize;
        let around = (std::f64::consts::PI * radius / step) as usize;
        for i in 0..along {
            points.push(Point2 {
                x: i as f64 * step,
                y: 0.0,
            });
        }
        for i in 0..around {
            let angle =
                -std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * i as f64 / around as f64;
            points.push(Point2 {
                x: straight + radius * angle.cos(),
                y: radius + radius * angle.sin(),
            });
        }
        for i in 0..along {
            points.push(Point2 {
                x: straight - i as f64 * step,
                y: 2.0 * radius,
            });
        }
        for i in 0..around {
            let angle =
                std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * i as f64 / around as f64;
            points.push(Point2 {
                x: radius * angle.cos(),
                y: radius + radius * angle.sin(),
            });
        }
        points
    }

    #[test]
    fn neighboring_speeds_respect_the_longitudinal_limits_across_the_seam() {
        let points = stadium();
        let limits = limits();
        let speeds = speeds(&points, &limits);
        let n = points.len();
        for i in 0..n {
            let next = (i + 1) % n;
            let step = points[i].distance(&points[next]);
            let delta = speeds[next].powi(2) - speeds[i].powi(2);
            assert!(
                delta <= 2.0 * limits.max_accel_mps2 * step + 1e-9,
                "accel at {i}"
            );
            assert!(
                -delta <= 2.0 * limits.max_decel_mps2 * step + 1e-9,
                "decel at {i}"
            );
        }
        // The straights get fast, the corners stay at their limit.
        assert!(speeds[n / 4] > 5.0);
        assert!(speeds.iter().cloned().fold(f64::INFINITY, f64::min) <= 2.0 + 1e-2);
    }

    #[test]
    fn a_lap_takes_its_length_over_its_speed() {
        let points = circle(2.0, 300);
        let speeds = vec![2.0; 300];
        let expected = super::super::geometry::loop_length(&points) / 2.0;
        assert!((lap_time(&points, &speeds) - expected).abs() < 1e-9);
    }
}
