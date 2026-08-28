//! The start/finish line: a segment perpendicular to the track direction at
//! the race line's first point, spanning the track width.

use crate::simulator::environment::smoothing::Point2;

/// The two endpoints of the start/finish line.
#[derive(Debug, Clone, Copy)]
pub struct StartFinishSegment {
    pub a: Point2,
    pub b: Point2,
}

/// Perpendicular to the direction from `closed_points[0]` to
/// `closed_points[1]`, centered on `closed_points[0]`, spanning
/// `track_width_m`.
///
/// # Panics
///
/// Panics if `closed_points` has fewer than 2 points.
pub fn compute(closed_points: &[Point2], track_width_m: f64) -> StartFinishSegment {
    let p0 = closed_points[0];
    let p1 = closed_points[1];
    let dx = p1.x - p0.x;
    let dy = p1.y - p0.y;
    let len = (dx * dx + dy * dy).sqrt();
    let (nx, ny) = if len > 1e-12 { (-dy / len, dx / len) } else { (1.0, 0.0) };
    let half = track_width_m / 2.0;
    StartFinishSegment {
        a: Point2 { x: p0.x + nx * half, y: p0.y + ny * half },
        b: Point2 { x: p0.x - nx * half, y: p0.y - ny * half },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn segment_is_centered_and_perpendicular() {
        let points = [Point2 { x: 0.0, y: 0.0 }, Point2 { x: 1.0, y: 0.0 }];
        let seg = compute(&points, 4.0);

        let mid = Point2 { x: (seg.a.x + seg.b.x) / 2.0, y: (seg.a.y + seg.b.y) / 2.0 };
        assert!(mid.x.abs() < 1e-9 && mid.y.abs() < 1e-9);

        let seg_len = seg.a.distance(&seg.b);
        assert!((seg_len - 4.0).abs() < 1e-9);

        let dir = (points[1].x - points[0].x, points[1].y - points[0].y);
        let seg_dir = (seg.a.x - seg.b.x, seg.a.y - seg.b.y);
        let dot = dir.0 * seg_dir.0 + dir.1 * seg_dir.1;
        assert!(dot.abs() < 1e-9);
    }
}
