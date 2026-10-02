//! The start/finish line: a segment perpendicular to the track direction at
//! the race line's first point, spanning the track width. [`rotate_to_straightest`]
//! picks which point that is; [`compute`] draws the segment there.

use crate::geometry::Point2;

/// The two endpoints of the start/finish line.
#[derive(Debug, Clone, Copy)]
pub struct StartFinishSegment {
    pub a: Point2,
    pub b: Point2,
}

/// Rotates `closed_points` (cyclically, in place) so index `0` lands on its
/// straightest point - the center of the run of points, at least
/// `track_width_m` of arc length in each direction, with the smallest worst-
/// case turn angle.
///
/// Leaving the start/finish gate at an arbitrary point (e.g. whatever
/// happened to land at index `0` before rotation) can place it in or near a
/// sharp corner. There, the rasterizer's nearest-centerline-point occupancy
/// test ([`crate::environment::generator::raster::rasterize`]) no longer
/// traces a clean, `track_width_m`-wide parallel band: at a tight enough
/// turn, pixels on the corner's inside can be nearest to a *different* part
/// of the loop across the bend, widening the apparent band there - and that
/// widening reaches out roughly `track_width_m` from the corner, not just
/// the corner's own point. [`compute`]'s fixed-width perpendicular chord
/// then visibly falls short of that widened edge. A point can measure a
/// near-zero turn angle against its immediate neighbors yet still sit this
/// close to a sharp bend - e.g. the inflection point of an S-bend, flanked
/// on both sides by tight turns just past it. Checking every point within
/// `track_width_m` of arc length, not just the candidate itself, catches
/// that case. Rotating to the point with the smallest such worst-case turn
/// angle avoids this, since [`compute`]'s straight-chord assumption only
/// holds where the track is locally straight (a large turn radius relative
/// to `track_width_m`) across that whole neighborhood.
///
/// # Panics
///
/// Panics if `closed_points` has fewer than 3 points (need a previous and a
/// next neighbor to measure a turn angle).
pub fn rotate_to_straightest(closed_points: &mut [Point2], track_width_m: f64) {
    let index = straightest_index(closed_points, track_width_m);
    closed_points.rotate_left(index);
}

fn straightest_index(closed_points: &[Point2], track_width_m: f64) -> usize {
    let n = closed_points.len();
    let half_window = neighborhood_half_window(closed_points, track_width_m);
    (0..n)
        .min_by(|&i, &j| {
            worst_turn_angle_nearby(closed_points, i, half_window)
                .total_cmp(&worst_turn_angle_nearby(closed_points, j, half_window))
        })
        .expect("closed_points is non-empty (checked by rotate_to_straightest's caller contract)")
}

/// How many neighbors on each side of a candidate point to check, so the
/// checked span covers `track_width_m` of arc length - estimated from the
/// spacing between the first two points, since [`closed_points`] comes from
/// [`crate::geometry::resample_even_spacing`] and
/// is therefore evenly spaced throughout.
fn neighborhood_half_window(closed_points: &[Point2], track_width_m: f64) -> usize {
    let n = closed_points.len();
    let spacing = closed_points[0].distance(&closed_points[1]).max(1e-9);
    let half_window = (track_width_m / spacing).ceil() as usize;
    half_window.clamp(1, (n - 1) / 2)
}

/// The largest [`turn_angle`] among `closed_points[i]` and its
/// `half_window` neighbors on each side.
fn worst_turn_angle_nearby(closed_points: &[Point2], i: usize, half_window: usize) -> f64 {
    let n = closed_points.len();
    (0..=2 * half_window)
        .map(|offset| turn_angle(closed_points, (i + n + offset - half_window) % n))
        .fold(0.0, f64::max)
}

/// Absolute turn angle (radians) at `closed_points[i]`, between the
/// incoming and outgoing segment directions - near `0` on a straight,
/// larger through a corner.
fn turn_angle(closed_points: &[Point2], i: usize) -> f64 {
    let n = closed_points.len();
    let prev = closed_points[(i + n - 1) % n];
    let curr = closed_points[i];
    let next = closed_points[(i + 1) % n];
    let (d1x, d1y) = (curr.x - prev.x, curr.y - prev.y);
    let (d2x, d2y) = (next.x - curr.x, next.y - curr.y);
    let cross = d1x * d2y - d1y * d2x;
    let dot = d1x * d2x + d1y * d2y;
    cross.atan2(dot).abs()
}

/// Perpendicular to the direction from `closed_points[0]` to
/// `closed_points[1]`, centered on `closed_points[0]`, spanning
/// `track_width_m`. Callers should [`rotate_to_straightest`] first, so
/// index `0` is somewhere this straight-chord assumption actually holds.
/// `a` is on the left of the direction of travel and `b` on the right, as
/// [`crate::environment::StartFinishLine`] requires.
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
    let (nx, ny) = if len > 1e-12 {
        (-dy / len, dx / len)
    } else {
        (1.0, 0.0)
    };
    let half = track_width_m / 2.0;
    StartFinishSegment {
        a: Point2 {
            x: p0.x + nx * half,
            y: p0.y + ny * half,
        },
        b: Point2 {
            x: p0.x - nx * half,
            y: p0.y - ny * half,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn segment_is_centered_and_perpendicular() {
        let points = [Point2 { x: 0.0, y: 0.0 }, Point2 { x: 1.0, y: 0.0 }];
        let seg = compute(&points, 4.0);

        let mid = Point2 {
            x: (seg.a.x + seg.b.x) / 2.0,
            y: (seg.a.y + seg.b.y) / 2.0,
        };
        assert!(mid.x.abs() < 1e-9 && mid.y.abs() < 1e-9);

        let seg_len = seg.a.distance(&seg.b);
        assert!((seg_len - 4.0).abs() < 1e-9);

        let dir = (points[1].x - points[0].x, points[1].y - points[0].y);
        let seg_dir = (seg.a.x - seg.b.x, seg.a.y - seg.b.y);
        let dot = dir.0 * seg_dir.0 + dir.1 * seg_dir.1;
        assert!(dot.abs() < 1e-9);
    }

    #[test]
    fn a_is_on_the_left_of_the_direction_of_travel() {
        use crate::environment::{StartFinishLine, WorldPoint};

        // Heading along +y.
        let points = [Point2 { x: 0.0, y: 0.0 }, Point2 { x: 0.0, y: 1.0 }];
        let seg = compute(&points, 2.0);
        assert!(seg.a.x < 0.0 && seg.b.x > 0.0);

        let line = StartFinishLine {
            a: WorldPoint {
                x: seg.a.x,
                y: seg.a.y,
            },
            b: WorldPoint {
                x: seg.b.x,
                y: seg.b.y,
            },
        };
        let (_, _, heading) = line.start_pose();
        assert!((heading - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
    }

    #[test]
    fn rotate_to_straightest_picks_the_zero_turn_angle_point() {
        // A sharp zigzag, except point 3 sits exactly between its two
        // collinear neighbors (points 2 and 4) - the only zero-turn-angle
        // point in this loop, and its neighborhood (points 2 and 4, spacing
        // 1.0) has no turn sharper than the tiny track_width_m below.
        let mut points = vec![
            Point2 { x: 0.0, y: 0.0 },
            Point2 { x: 1.0, y: 1.0 },
            Point2 { x: 1.0, y: 0.0 },
            Point2 { x: 2.0, y: 0.0 },
            Point2 { x: 3.0, y: 0.0 },
            Point2 { x: 3.0, y: 1.0 },
        ];
        let straight_point = points[3];

        rotate_to_straightest(&mut points, 0.5);

        assert_eq!(points[0], straight_point);
    }

    #[test]
    fn rotate_to_straightest_avoids_an_s_bend_inflection_near_sharp_turns() {
        // An S-bend: a zero-turn-angle inflection point (index 3) flanked
        // closely, on both sides, by a sharp turn - and a genuinely
        // straight, isolated run of points far away (indices 8-11). Even
        // though index 3 has the smallest single-point turn angle, its
        // neighborhood (within track_width_m) contains the sharp turns
        // right next to it, so it should be rejected in favor of the
        // isolated straight run.
        let mut points = vec![
            Point2 { x: 0.0, y: 0.0 },  // 0
            Point2 { x: 1.0, y: 1.0 },  // 1: sharp turn into the inflection
            Point2 { x: 2.0, y: 1.5 },  // 2
            Point2 { x: 3.0, y: 2.0 },  // 3: inflection - zero turn angle
            Point2 { x: 4.0, y: 2.5 },  // 4
            Point2 { x: 5.0, y: 3.0 },  // 5: sharp turn out of the inflection
            Point2 { x: 5.5, y: 4.0 },  // 6
            Point2 { x: 5.5, y: 5.0 },  // 7: turns toward the straight run
            Point2 { x: 5.5, y: 6.0 },  // 8
            Point2 { x: 5.5, y: 7.0 },  // 9: straight run, far from any turn
            Point2 { x: 5.5, y: 8.0 },  // 10
            Point2 { x: 5.5, y: 9.0 },  // 11
            Point2 { x: 5.0, y: 10.0 }, // 12: turns away again
            Point2 { x: 4.0, y: 10.0 }, // 13
        ];
        // Indices 7-10 all measure a zero single-point turn angle, but 8 and
        // 9 are the only ones whose whole track_width_m neighborhood stays
        // clear of the turns at both ends of the straight run; ties go to
        // the lower index.
        let far_straight_point = points[8];

        rotate_to_straightest(&mut points, 2.0);

        assert_eq!(points[0], far_straight_point);
    }
}
