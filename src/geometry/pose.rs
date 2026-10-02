//! [`Pose`]: a position and heading in the plane, and the angle wrapping
//! that keeps headings comparable.

use std::f64::consts::PI;

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

/// `angle_rad` wrapped to `(-pi, pi]`.
pub(crate) fn wrap_to_pi(angle_rad: f64) -> f64 {
    let wrapped = (angle_rad + PI).rem_euclid(2.0 * PI) - PI;
    if wrapped <= -PI {
        wrapped + 2.0 * PI
    } else {
        wrapped
    }
}
