//! [`Pose2`]: a 2D pose, and the handful of frame operations SLAM needs on
//! it - Karto's `Pose2` and `Transform`.

use crate::topics::Odometry;

/// A position and heading in some 2D frame, heading wrapped to `(-pi, pi]`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pose2 {
    pub x_m: f64,
    pub y_m: f64,
    pub heading_rad: f64,
}

impl Pose2 {
    pub fn new(x_m: f64, y_m: f64, heading_rad: f64) -> Self {
        Self {
            x_m,
            y_m,
            heading_rad: wrap_to_pi(heading_rad),
        }
    }

    /// Where the point `(x_m, y_m)`, given in this pose's own frame, lands
    /// in the frame this pose is expressed in.
    pub fn transform_point(&self, x_m: f64, y_m: f64) -> (f64, f64) {
        let (sin, cos) = self.heading_rad.sin_cos();
        (
            self.x_m + x_m * cos - y_m * sin,
            self.y_m + x_m * sin + y_m * cos,
        )
    }

    /// `other`, given in this pose's own frame, expressed in the frame this
    /// pose is expressed in.
    pub fn compose(&self, other: &Pose2) -> Pose2 {
        let (x_m, y_m) = self.transform_point(other.x_m, other.y_m);
        Pose2::new(x_m, y_m, self.heading_rad + other.heading_rad)
    }

    /// The pose that [`compose`](Self::compose)s with this one into the
    /// identity.
    pub fn inverse(&self) -> Pose2 {
        let (sin, cos) = self.heading_rad.sin_cos();
        Pose2::new(
            -self.x_m * cos - self.y_m * sin,
            self.x_m * sin - self.y_m * cos,
            -self.heading_rad,
        )
    }

    /// Squared distance between the two positions, in square meters.
    pub fn squared_distance(&self, other: &Pose2) -> f64 {
        (self.x_m - other.x_m).powi(2) + (self.y_m - other.y_m).powi(2)
    }

    /// The pose `odometry` reports, in the `odom` frame.
    pub fn from_odometry(odometry: &Odometry) -> Self {
        Pose2::new(odometry.x_m, odometry.y_m, odometry.heading_rad)
    }
}

/// Karto's `Transform(from, to)`: re-expresses poses given relative to
/// `from` relative to `to` instead. [`crate::localization::Slam`] uses it to
/// carry the correction the last scan match made (odometric pose → corrected
/// pose) over to the next scan's odometric pose.
pub fn transform_pose(from: &Pose2, to: &Pose2, pose: &Pose2) -> Pose2 {
    to.compose(&from.inverse().compose(pose))
}

/// Wraps an angle in radians to `(-pi, pi]`.
pub fn wrap_to_pi(angle_rad: f64) -> f64 {
    angle_rad.sin().atan2(angle_rad.cos())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn assert_close(a: Pose2, b: Pose2) {
        assert!(
            (a.x_m - b.x_m).abs() < 1e-12
                && (a.y_m - b.y_m).abs() < 1e-12
                && wrap_to_pi(a.heading_rad - b.heading_rad).abs() < 1e-12,
            "{a:?} != {b:?}"
        );
    }

    #[test]
    fn composing_with_the_inverse_gives_the_identity() {
        let pose = Pose2::new(1.5, -2.0, 0.7);
        assert_close(pose.compose(&pose.inverse()), Pose2::default());
        assert_close(pose.inverse().compose(&pose), Pose2::default());
    }

    #[test]
    fn composing_moves_along_the_first_pose_s_heading() {
        // Facing +y at (1, 1): 2 m forward lands at (1, 3), still facing +y.
        let pose = Pose2::new(1.0, 1.0, FRAC_PI_2);
        assert_close(
            pose.compose(&Pose2::new(2.0, 0.0, 0.0)),
            Pose2::new(1.0, 3.0, FRAC_PI_2),
        );
    }

    #[test]
    fn transform_pose_carries_a_correction_over_to_the_next_pose() {
        // Odometry said (1, 0, 0) but matching corrected it to (1, 0.2, 0.1):
        // the next odometric pose 1 m further ahead must land 1 m ahead of
        // the *corrected* pose, along its corrected heading.
        let odometric = Pose2::new(1.0, 0.0, 0.0);
        let corrected = Pose2::new(1.0, 0.2, 0.1);
        let next = transform_pose(&odometric, &corrected, &Pose2::new(2.0, 0.0, 0.0));
        assert_close(next, corrected.compose(&Pose2::new(1.0, 0.0, 0.0)));
    }

    #[test]
    fn transform_pose_with_equal_frames_changes_nothing() {
        let frame = Pose2::new(3.0, -1.0, 2.0);
        let pose = Pose2::new(0.5, 0.25, -1.0);
        assert_close(transform_pose(&frame, &frame, &pose), pose);
    }
}
