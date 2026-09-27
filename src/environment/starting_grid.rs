//! The starting grid of a race: where each vehicle starts, pole position
//! first - see [`slots`].

use crate::environment::{SpeedPoint, StartFinishLine};
use crate::topics::{StartState, VEHICLE_BODY_LENGTH_M, VEHICLE_BODY_WIDTH_M};

/// How the grid is laid out.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridSpacing {
    /// Added to half a body length between one slot and the next, along the
    /// centerline - so two slots on the same side are a body length plus
    /// twice this apart.
    pub gap_m: f64,
    /// Between a vehicle's side and the centerline.
    pub margin_m: f64,
}

/// The poses of `count` vehicles lined up behind `line`, pole position
/// first, all at rest: the first with its nose on the line, on the driver's
/// left of `centerline` - as the map is shown, y down, so a clockwise quarter
/// turn from the direction of travel, the way positive steering turns - each next one half
/// a body length plus [`GridSpacing::gap_m`] further back - measured along
/// the centerline - on the other side of it, [`GridSpacing::margin_m`]
/// clear of it. Each heads along the centerline's direction of travel
/// where it stands.
///
/// `is_free(x_m, y_m)` says whether a world point is drivable: a slot whose
/// body isn't entirely drivable is refused, as is a grid that doesn't fit on
/// the track at all.
pub fn slots(
    centerline: &[SpeedPoint],
    line: &StartFinishLine,
    count: usize,
    spacing: GridSpacing,
    is_free: impl Fn(f64, f64) -> bool,
) -> Result<Vec<StartState>, String> {
    let path = Path::new(centerline)
        .ok_or("The map has no centerline - a race needs one to line the grid up on.")?;
    let track_width_m = (line.b.x - line.a.x).hypot(line.b.y - line.a.y);
    let lateral_m = VEHICLE_BODY_WIDTH_M / 2.0 + spacing.margin_m;
    if track_width_m > 0.0 && 2.0 * (lateral_m + VEHICLE_BODY_WIDTH_M / 2.0) > track_width_m {
        return Err(format!(
            "The track ({track_width_m:.2} m wide) is too narrow for two vehicles side by side."
        ));
    }
    let stagger_m = VEHICLE_BODY_LENGTH_M / 2.0 + spacing.gap_m;
    let grid_m = VEHICLE_BODY_LENGTH_M + stagger_m * count.saturating_sub(1) as f64;
    if grid_m >= path.lap_m {
        return Err(format!(
            "A grid of {count} vehicles ({grid_m:.1} m) doesn't fit on a {:.1} m lap.",
            path.lap_m
        ));
    }

    let (x_m, y_m, heading_rad) = line.start_pose();
    let s0 = path.project(x_m, y_m);
    let (tx, ty) = path.tangent(s0);
    // Whether the centerline runs along the direction of travel.
    let forward = if tx * heading_rad.cos() + ty * heading_rad.sin() >= 0.0 {
        1.0
    } else {
        -1.0
    };

    (0..count)
        .map(|i| {
            let back_m = VEHICLE_BODY_LENGTH_M / 2.0 + stagger_m * i as f64;
            let s = s0 - forward * back_m;
            let (cx, cy) = path.at(s);
            let (tx, ty) = path.tangent(s);
            let (dx, dy) = (forward * tx, forward * ty);
            // The driver's left - `(dx, dy)` turned clockwise on screen,
            // y pointing down - for even slots.
            let side = if i % 2 == 0 { 1.0 } else { -1.0 };
            let slot = StartState {
                x_m: cx + dy * side * lateral_m,
                y_m: cy - dx * side * lateral_m,
                heading_rad: dy.atan2(dx),
                speed_mps: 0.0,
            };
            if body_is_free(&slot, &is_free) {
                Ok(slot)
            } else {
                Err(format!(
                    "Grid slot {} runs into a wall - the track is too narrow there.",
                    i + 1
                ))
            }
        })
        .collect()
}

/// Whether every point of a body at `pose` is drivable, sampled every few
/// centimeters.
fn body_is_free(pose: &StartState, is_free: &impl Fn(f64, f64) -> bool) -> bool {
    const STEPS: usize = 8;
    let (cos, sin) = (pose.heading_rad.cos(), pose.heading_rad.sin());
    (0..=STEPS).all(|i| {
        (0..=STEPS).all(|j| {
            let along = VEHICLE_BODY_LENGTH_M * (i as f64 / STEPS as f64 - 0.5);
            let across = VEHICLE_BODY_WIDTH_M * (j as f64 / STEPS as f64 - 0.5);
            is_free(
                pose.x_m + along * cos - across * sin,
                pose.y_m + along * sin + across * cos,
            )
        })
    })
}

/// A closed polyline, walked by arc length.
struct Path {
    points: Vec<(f64, f64)>,
    /// `s` at each point; the loop closes back to the first at `lap_m`.
    s: Vec<f64>,
    lap_m: f64,
}

impl Path {
    /// `None` with fewer than 3 distinct points.
    fn new(centerline: &[SpeedPoint]) -> Option<Self> {
        let mut points: Vec<(f64, f64)> = centerline.iter().map(|p| (p.x, p.y)).collect();
        points.dedup();
        // A closing point repeating the first.
        if points.len() > 1 && points.first() == points.last() {
            points.pop();
        }
        if points.len() < 3 {
            return None;
        }
        let mut s = Vec::with_capacity(points.len());
        let mut total = 0.0;
        for (i, &(x, y)) in points.iter().enumerate() {
            s.push(total);
            let (nx, ny) = points[(i + 1) % points.len()];
            total += (nx - x).hypot(ny - y);
        }
        Some(Self {
            points,
            s,
            lap_m: total,
        })
    }

    fn wrap(&self, s: f64) -> f64 {
        s.rem_euclid(self.lap_m)
    }

    /// The segment `s` is on, and how far along it.
    fn locate(&self, s: f64) -> (usize, f64) {
        let s = self.wrap(s);
        let i = self.s.partition_point(|&start| start <= s).max(1) - 1;
        (i, s - self.s[i])
    }

    fn at(&self, s: f64) -> (f64, f64) {
        let (i, along) = self.locate(s);
        let (x, y) = self.points[i];
        let (nx, ny) = self.points[(i + 1) % self.points.len()];
        let length = (nx - x).hypot(ny - y).max(f64::MIN_POSITIVE);
        let t = along / length;
        (x + (nx - x) * t, y + (ny - y) * t)
    }

    /// The unit direction of increasing `s` at `s` - a central difference
    /// over a body length, so one short segment doesn't skew it.
    fn tangent(&self, s: f64) -> (f64, f64) {
        let h = VEHICLE_BODY_LENGTH_M / 2.0;
        let (ax, ay) = self.at(s - h);
        let (bx, by) = self.at(s + h);
        let length = (bx - ax).hypot(by - ay).max(f64::MIN_POSITIVE);
        ((bx - ax) / length, (by - ay) / length)
    }

    /// `s` of the point of the path nearest `(x, y)`.
    fn project(&self, x: f64, y: f64) -> f64 {
        let n = self.points.len();
        (0..n)
            .map(|i| {
                let (ax, ay) = self.points[i];
                let (bx, by) = self.points[(i + 1) % n];
                let (dx, dy) = (bx - ax, by - ay);
                let length2 = (dx * dx + dy * dy).max(f64::MIN_POSITIVE);
                let t = (((x - ax) * dx + (y - ay) * dy) / length2).clamp(0.0, 1.0);
                let (px, py) = (ax + dx * t, ay + dy * t);
                ((px - x).hypot(py - y), self.s[i] + t * length2.sqrt())
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map_or(0.0, |(_, s)| s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::environment::WorldPoint;
    use std::f64::consts::FRAC_PI_2;

    const SPACING: GridSpacing = GridSpacing {
        gap_m: 0.2,
        margin_m: 0.1,
    };

    fn point(x: f64, y: f64) -> SpeedPoint {
        SpeedPoint {
            x,
            y,
            speed_mps: 1.0,
        }
    }

    /// A 40 m x 20 m rectangle, counterclockwise from the origin.
    fn rectangle() -> Vec<SpeedPoint> {
        vec![
            point(0.0, 0.0),
            point(40.0, 0.0),
            point(40.0, 20.0),
            point(0.0, 20.0),
        ]
    }

    /// Across the bottom straight at `x`, 2 m wide, heading along +x.
    fn line_at(x: f64) -> StartFinishLine {
        // `a` on the left of +x is +y.
        StartFinishLine {
            a: WorldPoint { x, y: 1.0 },
            b: WorldPoint { x, y: -1.0 },
        }
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn a_straight_grid_staggers_left_and_right() {
        let slots = slots(&rectangle(), &line_at(20.0), 3, SPACING, |_, _| true).unwrap();
        let lateral = VEHICLE_BODY_WIDTH_M / 2.0 + SPACING.margin_m;
        let stagger = VEHICLE_BODY_LENGTH_M / 2.0 + SPACING.gap_m;
        // Pole: nose on the line, on the driver's left - up the screen,
        // heading along +x with y down.
        assert!(close(slots[0].x_m + VEHICLE_BODY_LENGTH_M / 2.0, 20.0));
        assert!(close(slots[0].y_m, -lateral));
        assert!(close(slots[1].x_m, slots[0].x_m - stagger));
        assert!(close(slots[1].y_m, lateral));
        assert!(close(slots[2].x_m, slots[0].x_m - 2.0 * stagger));
        assert!(close(slots[2].y_m, -lateral));
        assert!(slots.iter().all(|slot| close(slot.heading_rad, 0.0)));
        assert!(slots.iter().all(|slot| slot.speed_mps == 0.0));
    }

    #[test]
    fn a_grid_follows_the_centerline_round_a_corner_and_past_its_end() {
        // Just past the first point: the grid reaches back up the left side,
        // heading down it (-y), past the end of the point list.
        let slots = slots(&rectangle(), &line_at(1.0), 6, SPACING, |_, _| true).unwrap();
        let last = slots.last().unwrap();
        assert!(close(
            last.x_m.abs(),
            VEHICLE_BODY_WIDTH_M / 2.0 + SPACING.margin_m
        ));
        assert!(last.y_m > 0.0);
        assert!(close(last.heading_rad, -FRAC_PI_2));
    }

    #[test]
    fn a_centerline_listed_against_the_direction_of_travel_makes_the_same_grid() {
        let mut reversed = rectangle();
        reversed.reverse();
        let along = slots(&rectangle(), &line_at(20.0), 4, SPACING, |_, _| true).unwrap();
        let against = slots(&reversed, &line_at(20.0), 4, SPACING, |_, _| true).unwrap();
        for (along, against) in along.iter().zip(&against) {
            assert!(close(along.x_m, against.x_m));
            assert!(close(along.y_m, against.y_m));
            assert!(close(along.heading_rad, against.heading_rad));
        }
    }

    #[test]
    fn a_grid_is_refused_without_a_centerline_or_room() {
        assert!(slots(&[], &line_at(20.0), 2, SPACING, |_, _| true).is_err());
        let narrow = StartFinishLine {
            a: WorldPoint { x: 20.0, y: 0.3 },
            b: WorldPoint { x: 20.0, y: -0.3 },
        };
        assert!(slots(&rectangle(), &narrow, 2, SPACING, |_, _| true).is_err());
        assert!(slots(&rectangle(), &line_at(20.0), 1000, SPACING, |_, _| true).is_err());
        let error = slots(&rectangle(), &line_at(20.0), 3, SPACING, |x, _| x > 19.0).unwrap_err();
        assert!(error.contains("slot 3"), "{error}");
    }
}
