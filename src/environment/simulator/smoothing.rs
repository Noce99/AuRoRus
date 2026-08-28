//! Turns the raw, irregularly-spaced closed loop from
//! [`crate::environment::simulator::voronoi_loop`] into a smooth, evenly
//! arc-length-spaced closed curve: [`densify`] interpolates it with a
//! centripetal Catmull-Rom spline, then [`resample_even_spacing`] walks the
//! result at fixed arc-length steps.

/// A point in world coordinates (meters).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point2 {
    pub x: f64,
    pub y: f64,
}

impl Point2 {
    pub fn distance(&self, other: &Point2) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }
}

/// Centripetal parametrization exponent (see Yuksel et al., "On the
/// Parameterization of Catmull-Rom Curves"). Uniform (alpha = 0) parametrization
/// is known to produce cusps/local self-intersections on irregularly-spaced
/// control points - exactly the input this function gets - which would
/// silently undo the topological simplicity guarantee the loop was built
/// with. Centripetal (alpha = 0.5) avoids that.
const CENTRIPETAL_ALPHA: f64 = 0.5;

/// Densifies a closed polygon (`loop_points`, not repeating its first point)
/// into `loop_points.len() * samples_per_segment` points along a centripetal
/// Catmull-Rom spline through them. The output is likewise closed and
/// doesn't repeat its first point.
pub fn densify(loop_points: &[Point2], samples_per_segment: usize) -> Vec<Point2> {
    let n = loop_points.len();
    if n < 3 || samples_per_segment == 0 {
        return loop_points.to_vec();
    }

    let mut out = Vec::with_capacity(n * samples_per_segment);
    for i in 0..n {
        let p0 = loop_points[(i + n - 1) % n];
        let p1 = loop_points[i];
        let p2 = loop_points[(i + 1) % n];
        let p3 = loop_points[(i + 2) % n];
        for s in 0..samples_per_segment {
            let u = s as f64 / samples_per_segment as f64;
            out.push(catmull_rom_segment(p0, p1, p2, p3, u));
        }
    }
    out
}

/// Point at local parameter `u` (0.0 at `p1`, 1.0 at `p2`) along the
/// centripetal Catmull-Rom segment defined by control points `p0..p3`.
fn catmull_rom_segment(p0: Point2, p1: Point2, p2: Point2, p3: Point2, u: f64) -> Point2 {
    let t0 = 0.0;
    let t1 = t0 + knot_interval(p0, p1);
    let t2 = t1 + knot_interval(p1, p2);
    let t3 = t2 + knot_interval(p2, p3);
    let t = t1 + u * (t2 - t1);

    let a1 = lerp_point(p0, p1, t0, t1, t);
    let a2 = lerp_point(p1, p2, t1, t2, t);
    let a3 = lerp_point(p2, p3, t2, t3, t);
    let b1 = lerp_point(a1, a2, t0, t2, t);
    let b2 = lerp_point(a2, a3, t1, t3, t);
    lerp_point(b1, b2, t1, t2, t)
}

fn knot_interval(a: Point2, b: Point2) -> f64 {
    a.distance(&b).max(1e-9).powf(CENTRIPETAL_ALPHA)
}

fn lerp_point(a: Point2, b: Point2, ta: f64, tb: f64, t: f64) -> Point2 {
    if (tb - ta).abs() < 1e-12 {
        return a;
    }
    let f = (t - ta) / (tb - ta);
    Point2 {
        x: a.x + f * (b.x - a.x),
        y: a.y + f * (b.y - a.y),
    }
}

/// Resamples `dense` (treated as closed - the last point connects back to
/// the first) into `round(total_length / spacing_m)` points evenly spaced by
/// arc length, including the closing segment. This is the "fixed distance
/// between points" contract for the race-line CSV: dividing the *whole*
/// closed length evenly (rather than walking forward and stopping whenever
/// the loop happens to close) keeps the wrap-around gap the same size as
/// every other gap.
pub fn resample_even_spacing(dense: &[Point2], spacing_m: f64) -> Vec<Point2> {
    let n = dense.len();
    if n < 2 {
        return dense.to_vec();
    }

    let total_length: f64 = (0..n).map(|i| dense[i].distance(&dense[(i + 1) % n])).sum();
    let num_points = ((total_length / spacing_m).round() as usize).max(3);
    let actual_spacing = total_length / num_points as f64;

    let mut out = Vec::with_capacity(num_points);
    let mut segment_index = 0;
    let mut segment_start_length = 0.0;
    let mut segment_length = dense[0].distance(&dense[1 % n]);

    for k in 0..num_points {
        let target = k as f64 * actual_spacing;
        while segment_start_length + segment_length < target && segment_index < n - 1 {
            segment_start_length += segment_length;
            segment_index += 1;
            segment_length = dense[segment_index].distance(&dense[(segment_index + 1) % n]);
        }
        let f = if segment_length > 1e-12 {
            ((target - segment_start_length) / segment_length).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let a = dense[segment_index];
        let b = dense[(segment_index + 1) % n];
        out.push(Point2 {
            x: a.x + f * (b.x - a.x),
            y: a.y + f * (b.y - a.y),
        });
    }

    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square() -> Vec<Point2> {
        vec![
            Point2 { x: 0.0, y: 0.0 },
            Point2 { x: 4.0, y: 0.0 },
            Point2 { x: 4.0, y: 4.0 },
            Point2 { x: 0.0, y: 4.0 },
        ]
    }

    #[test]
    fn densify_scales_with_samples_per_segment() {
        let loop_points = square();
        let dense = densify(&loop_points, 10);
        assert_eq!(dense.len(), loop_points.len() * 10);
    }

    #[test]
    fn densify_stays_reasonably_close_to_the_input_shape() {
        let loop_points = square();
        let dense = densify(&loop_points, 20);
        for p in &dense {
            assert!(p.x >= -0.5 && p.x <= 4.5, "x out of expected range: {}", p.x);
            assert!(p.y >= -0.5 && p.y <= 4.5, "y out of expected range: {}", p.y);
        }
    }

    #[test]
    fn resample_even_spacing_gaps_are_close_to_requested() {
        let loop_points = square();
        let dense = densify(&loop_points, 30);
        let spacing = 0.5;
        let resampled = resample_even_spacing(&dense, spacing);

        let n = resampled.len();
        assert!(n > 10);
        for i in 0..n {
            let gap = resampled[i].distance(&resampled[(i + 1) % n]);
            assert!((gap - spacing).abs() < spacing * 0.1, "gap {gap} too far from {spacing}");
        }
    }
}
