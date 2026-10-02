//! A speed for every point of a closed line: as fast as the lateral
//! acceleration limit allows in each corner, capped at a top speed, then
//! slowed wherever braking or accelerating between neighboring points
//! would take more than the vehicle can - the classic forward/backward pass
//! for a point mass.
//!
//! The vehicle's grip is a **friction ellipse**: on the segment from point
//! `i` to `i + 1`, the longitudinal acceleration `a_lon = (v_{i+1}^2 -
//! v_i^2) / (2 d_i)` and the lateral one at its start, `a_lat = v_i^2 *
//! kappa_i`, must satisfy `(a_lon / max_decel)^2 + (a_lat / max_lateral)^2
//! <= 1` - braking or accelerating in a corner leaves less grip to turn.
//! Accelerating is further capped by the motor, `a_lon <= max_accel`. These
//! are exactly the constraints [`super::min_time`] optimizes under, so a
//! profile from here is a feasible starting point for it.

use crate::geometry::{Point2, curvatures};

/// Every limit [`speeds`] respects.
#[derive(Debug, Clone, Copy)]
pub struct SpeedLimits {
    /// Top speed, in m/s.
    pub max_speed_mps: f64,
    /// Largest lateral (centripetal) acceleration - the friction ellipse's
    /// lateral semi-axis - in m/s^2.
    pub max_lateral_accel_mps2: f64,
    /// Largest forward acceleration the motor gives, in m/s^2.
    pub max_accel_mps2: f64,
    /// Largest braking deceleration - the friction ellipse's longitudinal
    /// semi-axis, so the grip limit on accelerating too - in m/s^2
    /// (positive).
    pub max_decel_mps2: f64,
}

/// Most full backward+forward rounds [`speeds`] makes before giving up on
/// the profile settling - a couple always suffice on a closed loop, so
/// this is only a guard.
const MAX_ROUNDS: usize = 10;

/// The speed at every point of the closed loop `points`, in m/s - see the
/// module docs.
pub fn speeds(points: &[Point2], limits: &SpeedLimits) -> Vec<f64> {
    let n = points.len();
    let kappa = curvatures(points);
    // Worked in squared speeds, where every limit is a simple bound.
    let mut squared: Vec<f64> = kappa
        .iter()
        .map(|curvature| {
            let cornering = if curvature.abs() > 1e-9 {
                limits.max_lateral_accel_mps2 / curvature.abs()
            } else {
                f64::INFINITY
            };
            cornering.min(limits.max_speed_mps.powi(2))
        })
        .collect();
    if n < 2 {
        return squared.into_iter().map(f64::sqrt).collect();
    }
    let steps: Vec<f64> = (0..n)
        .map(|i| points[i].distance(&points[(i + 1) % n]))
        .collect();
    // Lateral grip used per unit of squared speed at each point.
    let lateral: Vec<f64> = kappa
        .iter()
        .map(|k| k.abs() / limits.max_lateral_accel_mps2)
        .collect();

    for _ in 0..MAX_ROUNDS {
        let mut changed = false;
        // Braking into i + 1: the fastest V_i from which V_{i+1} is reached
        // decelerating within the ellipse at i.
        for i in (0..n).rev() {
            let limit = braking_limit(
                squared[(i + 1) % n],
                2.0 * steps[i] * limits.max_decel_mps2,
                lateral[i],
            );
            if limit < squared[i] {
                squared[i] = limit;
                changed = true;
            }
        }
        // Accelerating out of i: grip left by the corner, capped by the motor.
        for i in 0..n {
            let next = (i + 1) % n;
            let grip = (1.0 - (squared[i] * lateral[i]).powi(2)).max(0.0).sqrt();
            let accel = limits.max_accel_mps2.min(limits.max_decel_mps2 * grip);
            let limit = squared[i] + 2.0 * steps[i] * accel;
            if limit < squared[next] {
                squared[next] = limit;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    squared.into_iter().map(f64::sqrt).collect()
}

/// The largest squared speed `x` at a point from which squared speed `next`
/// is reached braking within the ellipse: `x - next <= c * sqrt(1 - (k
/// x)^2)`, with `c = 2 d max_decel` and `k` the point's lateral grip per
/// squared speed. Squaring gives a quadratic in `x`; its larger root is the
/// limit. With no root at or above `next`, even holding speed uses all the
/// grip, so no braking is possible and `next` itself is the limit (a slower
/// point is the forward pass's business).
fn braking_limit(next: f64, c: f64, k: f64) -> f64 {
    let a = 1.0 + c * c * k * k;
    let discriminant = c * c * (a - k * k * next * next);
    if discriminant < 0.0 {
        return next;
    }
    ((next + discriminant.sqrt()) / a).max(next)
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
    use crate::geometry::closed_loop::tests::circle;

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
    fn neighboring_speeds_stay_inside_the_friction_ellipse() {
        let points = stadium();
        let limits = limits();
        let speeds = speeds(&points, &limits);
        let kappa = curvatures(&points);
        let n = points.len();
        for i in 0..n {
            let next = (i + 1) % n;
            let step = points[i].distance(&points[next]);
            let a_lon = (speeds[next].powi(2) - speeds[i].powi(2)) / (2.0 * step);
            let a_lat = speeds[i].powi(2) * kappa[i];
            let used = (a_lon / limits.max_decel_mps2).powi(2)
                + (a_lat / limits.max_lateral_accel_mps2).powi(2);
            assert!(used <= 1.0 + 1e-6, "ellipse at {i}: {used}");
            assert!(a_lon <= limits.max_accel_mps2 + 1e-9, "motor at {i}");
        }
    }

    #[test]
    fn the_braking_limit_solves_its_quadratic() {
        // Straight (k = 0): plain v^2 = next + c.
        assert!((braking_limit(4.0, 3.0, 0.0) - 7.0).abs() < 1e-12);
        // In a corner, the limit sits exactly on the ellipse.
        let (next, c, k) = (2.0, 1.5, 0.3);
        let x = braking_limit(next, c, k);
        assert!((x - next - c * (1.0 - (k * x).powi(2)).sqrt()).abs() < 1e-9);
        // A corner using all the grip already allows no braking.
        assert_eq!(braking_limit(10.0, 1.0, 0.2), 10.0);
    }

    #[test]
    fn a_lap_takes_its_length_over_its_speed() {
        let points = circle(2.0, 300);
        let speeds = vec![2.0; 300];
        let expected = crate::geometry::loop_length(&points) / 2.0;
        assert!((lap_time(&points, &speeds) - expected).abs() < 1e-9);
    }
}
