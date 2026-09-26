//! Where the vehicle is, and where the race line is: the pose sources and
//! the closed-line geometry shared by the algorithms that follow the
//! selected map's race line (`pure_pursuit`, `ubm_potential_pursuit`).

use crate::environment::SpeedPoint;
use crate::topics::{
    ODOMETRY_TOPIC_NAME, Odometry, SLAM_STATUS_TOPIC_NAME, SlamState, SlamStatus,
    VEHICLE_STATUS_TOPIC_NAME, VehicleStatus,
};
use crate::Captain;
use std::f64::consts::PI;
use std::time::Duration;

/// How old the pose may get before it's no longer trusted.
pub(crate) const POSE_TIMEOUT: Duration = Duration::from_millis(300);

/// A `pose_source` value: odometry composed onto SLAM's `map_to_odom`.
pub(crate) const POSE_LOCALIZATION: u8 = 0;
/// A `pose_source` value: the simulator's `vehicle_status`.
pub(crate) const POSE_GROUND_TRUTH: u8 = 1;

/// The pose from `source` ([`POSE_LOCALIZATION`] or [`POSE_GROUND_TRUTH`]),
/// or why there's none.
pub(crate) fn pose(captain: &Captain, source: u8) -> Result<Pose, String> {
    match source {
        POSE_LOCALIZATION => localization_pose(captain),
        POSE_GROUND_TRUTH => ground_truth_pose(captain),
        // Unreachable when tuned: the tuner keeps it in range.
        _ => Err(format!("Unknown pose source {source}.")),
    }
}

/// A pose in the map frame, heading wrapped to `(-pi, pi]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Pose {
    pub(crate) x_m: f64,
    pub(crate) y_m: f64,
    pub(crate) heading_rad: f64,
}

impl Pose {
    /// `other`, given in this pose's own frame, expressed in the frame this
    /// pose is expressed in.
    pub(crate) fn compose(&self, other: &Pose) -> Pose {
        let (sin, cos) = self.heading_rad.sin_cos();
        Pose {
            x_m: self.x_m + other.x_m * cos - other.y_m * sin,
            y_m: self.y_m + other.x_m * sin + other.y_m * cos,
            heading_rad: wrap_to_pi(self.heading_rad + other.heading_rad),
        }
    }

    /// This pose moved `distance_m` backward along its heading.
    pub(crate) fn moved_back(&self, distance_m: f64) -> Pose {
        let (sin, cos) = self.heading_rad.sin_cos();
        Pose {
            x_m: self.x_m - distance_m * cos,
            y_m: self.y_m - distance_m * sin,
            ..*self
        }
    }
}

/// The simulator's ground truth, if fresh - else why not.
pub(crate) fn ground_truth_pose(captain: &Captain) -> Result<Pose, String> {
    let status = captain
        .try_topic::<VehicleStatus>(VEHICLE_STATUS_TOPIC_NAME)
        .ok_or("No ground truth pose (vehicle_status) in this binary.")?
        .read();
    if status.age().is_none_or(|age| age > POSE_TIMEOUT) {
        return Err("The ground truth pose (vehicle_status) is stale.".into());
    }
    Ok(Pose {
        x_m: status.x_m,
        y_m: status.y_m,
        heading_rad: status.heading_rad,
    })
}

/// Odometry's latest pose composed onto SLAM's `map_to_odom` - the pose on
/// the map at odometry's rate - while SLAM is localizing (not paused, where
/// the pose would only be dead-reckoned), odometry is fresh, and both agree
/// on odometry's frame - else why not.
pub(crate) fn localization_pose(captain: &Captain) -> Result<Pose, String> {
    let slam = captain
        .try_topic::<SlamStatus>(SLAM_STATUS_TOPIC_NAME)
        .ok_or("No localization (slam_status) in this binary.")?
        .read()
        .into_value();
    if slam.state != SlamState::Localizing {
        return Err("Localization isn't running - start it in the Localization panel.".into());
    }
    let [x_m, y_m, heading_rad] = slam
        .map_to_odom
        .ok_or("Localization has no pose yet.")?;
    let odometry = captain
        .try_topic::<Odometry>(ODOMETRY_TOPIC_NAME)
        .ok_or("No odometry in this binary.")?
        .read();
    if odometry.age().is_none_or(|age| age > POSE_TIMEOUT) {
        return Err("Odometry is stale.".into());
    }
    if odometry.reset_count != slam.odometry_reset_count {
        return Err("Odometry was reset - waiting for localization to catch up.".into());
    }
    let map_to_odom = Pose { x_m, y_m, heading_rad };
    Ok(map_to_odom.compose(&Pose {
        x_m: odometry.x_m,
        y_m: odometry.y_m,
        heading_rad: odometry.heading_rad,
    }))
}

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
    pub(crate) fn nearest(&self, x_m: f64, y_m: f64, hint: Option<usize>, window_m: f64) -> Nearest {
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
        let segment = (self.cumulative_m.partition_point(|&c| c <= s_m) - 1).min(n - 1);
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
}


pub(crate) fn wrap_to_pi(angle_rad: f64) -> f64 {
    let wrapped = (angle_rad + PI).rem_euclid(2.0 * PI) - PI;
    if wrapped <= -PI { wrapped + 2.0 * PI } else { wrapped }
}
