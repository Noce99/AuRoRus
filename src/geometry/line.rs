//! [`Line`]: a closed race line with its arc length, and where a pose
//! projects onto it - what everything following or measuring against a race
//! line shares.

use super::pose::{Pose, wrap_to_pi};
use crate::environment::SpeedPoint;

/// A closed race line, with the arc length at each of its points.
pub(crate) struct Line {
    pub(crate) points: Vec<SpeedPoint>,
    /// `cumulative_m[i]` is the arc length from point 0 to point `i`;
    /// `cumulative_m[n]`, one past the last point, is the lap length.
    pub(crate) cumulative_m: Vec<f64>,
}

/// Where a pose projects onto a [`Line`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Nearest {
    /// Segment from point `segment` to the next one.
    pub(crate) segment: usize,
    /// Arc length of the projection.
    pub(crate) s_m: f64,
    pub(crate) x_m: f64,
    pub(crate) y_m: f64,
    /// Distance from the pose to the projection.
    pub(crate) distance_m: f64,
}

impl Line {
    /// `None` if `points` can't make a closed line.
    pub(crate) fn new(points: Vec<SpeedPoint>) -> Option<Self> {
        if points.len() < 3 {
            return None;
        }
        let n = points.len();
        let mut cumulative_m = Vec::with_capacity(n + 1);
        cumulative_m.push(0.0);
        for i in 0..n {
            let (a, b) = (points[i], points[(i + 1) % n]);
            cumulative_m.push(cumulative_m[i] + (b.x - a.x).hypot(b.y - a.y));
        }
        (cumulative_m[n] > 0.0).then_some(Self {
            points,
            cumulative_m,
        })
    }

    pub(crate) fn lap_m(&self) -> f64 {
        self.cumulative_m[self.points.len()]
    }

    pub(crate) fn segment_len_m(&self, segment: usize) -> f64 {
        self.cumulative_m[segment + 1] - self.cumulative_m[segment]
    }

    /// The projection of `(x_m, y_m)` onto the line: onto every segment if
    /// there's no `hint`, else only onto those from a couple before `hint` up
    /// to `window_m` of arc length past it - so the vehicle never jumps to
    /// another stretch of track that happens to run close by.
    pub(crate) fn nearest(
        &self,
        x_m: f64,
        y_m: f64,
        hint: Option<usize>,
        window_m: f64,
    ) -> Nearest {
        let n = self.points.len();
        let segments: Box<dyn Iterator<Item = usize>> = match hint {
            None => Box::new(0..n),
            Some(hint) => {
                let first = (hint % n + n - 2) % n;
                let mut covered_m = 0.0;
                Box::new(
                    (0..n)
                        .map(move |k| (first + k) % n)
                        .take_while(move |&segment| {
                            let inside = covered_m <= window_m;
                            covered_m += self.segment_len_m(segment);
                            inside
                        }),
                )
            }
        };
        segments
            .map(|segment| {
                let (a, b) = (self.points[segment], self.points[(segment + 1) % n]);
                let (dx, dy) = (b.x - a.x, b.y - a.y);
                let len2 = dx * dx + dy * dy;
                let t = if len2 > 0.0 {
                    (((x_m - a.x) * dx + (y_m - a.y) * dy) / len2).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let (px, py) = (a.x + t * dx, a.y + t * dy);
                Nearest {
                    segment,
                    s_m: self.cumulative_m[segment] + t * self.segment_len_m(segment),
                    x_m: px,
                    y_m: py,
                    distance_m: (x_m - px).hypot(y_m - py),
                }
            })
            .min_by(|a, b| a.distance_m.total_cmp(&b.distance_m))
            .expect("a line has at least 3 segments, and a window at least one")
    }

    /// The point at arc length `s_m`, wrapped around the lap, interpolated
    /// between its segment's ends - speed included.
    pub(crate) fn at(&self, s_m: f64) -> SpeedPoint {
        let n = self.points.len();
        let s_m = s_m.rem_euclid(self.lap_m());
        let segment = self.segment_at(s_m);
        let len_m = self.segment_len_m(segment);
        let t = if len_m > 0.0 {
            (s_m - self.cumulative_m[segment]) / len_m
        } else {
            0.0
        };
        let (a, b) = (self.points[segment], self.points[(segment + 1) % n]);
        SpeedPoint {
            x: a.x + t * (b.x - a.x),
            y: a.y + t * (b.y - a.y),
            speed_mps: a.speed_mps + t * (b.speed_mps - a.speed_mps),
        }
    }

    /// The segment holding arc length `s_m`, wrapped around the lap.
    pub(crate) fn segment_at(&self, s_m: f64) -> usize {
        let s_m = s_m.rem_euclid(self.lap_m());
        (self.cumulative_m.partition_point(|&c| c <= s_m) - 1).min(self.points.len() - 1)
    }

    /// Direction of `segment` (wrapped around the lap), same convention as a
    /// pose's heading.
    pub(crate) fn segment_heading(&self, segment: usize) -> f64 {
        let n = self.points.len();
        let (a, b) = (self.points[segment % n], self.points[(segment + 1) % n]);
        (b.y - a.y).atan2(b.x - a.x)
    }

    /// Direction of the line at arc length `s_m`, continuous along it:
    /// interpolated between the headings at the ends of its segment, each
    /// the mean of the two segments meeting there - so a line offset
    /// sideways doesn't kink at every point.
    pub(crate) fn heading_at(&self, s_m: f64) -> f64 {
        let n = self.points.len();
        let s_m = s_m.rem_euclid(self.lap_m());
        let segment = self.segment_at(s_m);
        let len_m = self.segment_len_m(segment);
        let t = if len_m > 0.0 {
            (s_m - self.cumulative_m[segment]) / len_m
        } else {
            0.0
        };
        let vertex = |i: usize| {
            let before = self.segment_heading((i + n - 1) % n);
            before + wrap_to_pi(self.segment_heading(i % n) - before) / 2.0
        };
        let (start, end) = (vertex(segment), vertex(segment + 1));
        wrap_to_pi(start + t * wrap_to_pi(end - start))
    }

    /// Where `pose` sits relative to the line at its projection `nearest`:
    /// its signed lateral offset (positive toward increasing heading - the
    /// side a positive steering angle turns toward) and how fast that offset grows per meter of line
    /// travelled - the tangent of the heading error, clamped to +-1.
    pub(crate) fn lateral(&self, pose: Pose, nearest: &Nearest) -> (f64, f64) {
        let heading = self.heading_at(nearest.s_m);
        let (sin, cos) = heading.sin_cos();
        let d = -sin * (pose.x_m - nearest.x_m) + cos * (pose.y_m - nearest.y_m);
        let slope = wrap_to_pi(pose.heading_rad - heading)
            .tan()
            .clamp(-1.0, 1.0);
        (d, slope)
    }

    /// Signed curvature at arc length `s_m`: that of the circle through the
    /// start of its segment and the points either side of it (Menger
    /// curvature), positive where the line turns toward increasing heading.
    pub(crate) fn curvature_at(&self, s_m: f64) -> f64 {
        let n = self.points.len();
        let i = self.segment_at(s_m);
        let (a, b, c) = (
            self.points[(i + n - 1) % n],
            self.points[i],
            self.points[(i + 1) % n],
        );
        let cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
        let sides = (b.x - a.x).hypot(b.y - a.y)
            * (c.x - b.x).hypot(c.y - b.y)
            * (c.x - a.x).hypot(c.y - a.y);
        if sides > 0.0 {
            2.0 * cross / sides
        } else {
            0.0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn circle(radius_m: f64, counterclockwise: bool) -> Line {
        let n = 400;
        let sign = if counterclockwise { 1.0 } else { -1.0 };
        Line::new(
            (0..n)
                .map(|i| {
                    let angle = sign * 2.0 * PI * i as f64 / n as f64;
                    SpeedPoint {
                        x: radius_m * angle.cos(),
                        y: radius_m * angle.sin(),
                        speed_mps: 2.0,
                    }
                })
                .collect(),
        )
        .unwrap()
    }

    #[test]
    fn curvature_is_the_inverse_radius_signed_by_turning_direction() {
        assert!((circle(4.0, true).curvature_at(1.0) - 0.25).abs() < 1e-3);
        assert!((circle(4.0, false).curvature_at(1.0) + 0.25).abs() < 1e-3);
    }

    #[test]
    fn a_segment_heads_from_its_start_to_its_end() {
        let line = Line::new(vec![
            SpeedPoint {
                x: 0.0,
                y: 0.0,
                speed_mps: 1.0,
            },
            SpeedPoint {
                x: 0.0,
                y: 2.0,
                speed_mps: 1.0,
            },
            SpeedPoint {
                x: -1.0,
                y: 1.0,
                speed_mps: 1.0,
            },
        ])
        .unwrap();
        assert!((line.segment_heading(0) - PI / 2.0).abs() < 1e-12);
        assert_eq!(line.segment_at(1.0), 0);
        assert_eq!(line.segment_at(2.5), 1);
    }

    #[test]
    fn the_heading_is_continuous_and_tangent_on_a_circle() {
        let line = circle(4.0, true);
        let step = line.lap_m() / 1000.0;
        for i in 0..1000 {
            let s = i as f64 * step;
            let jump = wrap_to_pi(line.heading_at(s + step) - line.heading_at(s)).abs();
            assert!(jump < 2.0 * step / 4.0, "jump {jump} at {s}");
            // Tangent to a counterclockwise circle: the position angle plus 90 degrees.
            let p = line.at(s);
            let expected = p.y.atan2(p.x) + PI / 2.0;
            assert!(
                wrap_to_pi(line.heading_at(s) - expected).abs() < 0.02,
                "at {s}"
            );
        }
    }

    #[test]
    fn lateral_is_positive_toward_increasing_heading() {
        let line = circle(4.0, true);
        // At (4, 0) the line heads +y; increasing heading is toward the center.
        let pose = Pose {
            x_m: 3.8,
            y_m: 0.0,
            heading_rad: PI / 2.0 + 0.1,
        };
        let nearest = line.nearest(pose.x_m, pose.y_m, None, 0.0);
        let (d, slope) = line.lateral(pose, &nearest);
        assert!((d - 0.2).abs() < 1e-2, "{d}");
        assert!((slope - 0.1f64.tan()).abs() < 2e-2, "{slope}");
    }
}
