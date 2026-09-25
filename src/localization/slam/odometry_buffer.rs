//! [`OdometryBuffer`]: the last few [`Odometry`] samples, so the pose at
//! the instant a scan was taken can be interpolated between the two samples
//! around it - the job ROS's TF buffer does for slam_toolbox.

use super::pose::{Pose2, wrap_to_pi};
use crate::topics::Odometry;
use std::collections::VecDeque;
use std::time::Instant;

/// What [`OdometryBuffer::pose_at`] found for an instant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PoseAt {
    /// The pose at that instant, in the `odom` frame of
    /// [`OdometryBuffer::reset_count`].
    Pose(Pose2),
    /// The instant is newer than the newest sample: ask again once
    /// odometry has caught up.
    NotYet,
    /// The instant is older than the oldest sample (or the buffer is
    /// empty and never will cover it): it can't be answered anymore.
    Unavailable,
}

/// A ring of the latest `capacity` odometry samples, stamped with when
/// they were written, all from the same `odom` frame: a sample with a
/// different [`Odometry::reset_count`] than the ones already held throws
/// them all away first.
pub struct OdometryBuffer {
    capacity: usize,
    samples: VecDeque<(Instant, Odometry)>,
}

impl OdometryBuffer {
    /// # Panics
    ///
    /// Panics if `capacity` is below `2` - interpolating needs two samples.
    pub fn new(capacity: usize) -> Self {
        assert!(
            capacity >= 2,
            "OdometryBuffer: capacity must be at least 2, got {capacity}"
        );
        Self {
            capacity,
            samples: VecDeque::with_capacity(capacity),
        }
    }

    /// Adds `odometry`, written at `at`. Samples must arrive in write
    /// order; one older than the newest held is ignored.
    pub fn push(&mut self, at: Instant, odometry: Odometry) {
        if let Some((newest_at, newest)) = self.samples.back() {
            if newest.reset_count != odometry.reset_count {
                self.samples.clear();
            } else if at < *newest_at {
                return;
            }
        }
        if self.samples.len() == self.capacity {
            self.samples.pop_front();
        }
        self.samples.push_back((at, odometry));
    }

    /// The [`Odometry::reset_count`] of the samples held, if any.
    pub fn reset_count(&self) -> Option<u64> {
        self.samples
            .back()
            .map(|(_, odometry)| odometry.reset_count)
    }

    /// The pose at `at`, linearly interpolated between the two samples
    /// around it (heading along the shorter arc).
    pub fn pose_at(&self, at: Instant) -> PoseAt {
        let (Some((oldest_at, _)), Some((newest_at, newest))) =
            (self.samples.front(), self.samples.back())
        else {
            return PoseAt::NotYet;
        };
        if at > *newest_at {
            return PoseAt::NotYet;
        }
        if at == *newest_at {
            return PoseAt::Pose(Pose2::from_odometry(newest));
        }
        if at < *oldest_at {
            return PoseAt::Unavailable;
        }

        // The first sample written after `at` - there is one, since `at` is
        // before the newest - and the one just before it.
        let after = self
            .samples
            .partition_point(|(sample_at, _)| *sample_at <= at);
        let (before_at, before) = &self.samples[after - 1];
        let (after_at, after) = &self.samples[after];
        let span_s = after_at.duration_since(*before_at).as_secs_f64();
        let fraction = if span_s > 0.0 {
            at.duration_since(*before_at).as_secs_f64() / span_s
        } else {
            0.0
        };
        let heading_change = wrap_to_pi(after.heading_rad - before.heading_rad);
        PoseAt::Pose(Pose2::new(
            before.x_m + fraction * (after.x_m - before.x_m),
            before.y_m + fraction * (after.y_m - before.y_m),
            before.heading_rad + fraction * heading_change,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;
    use std::time::Duration;

    fn odometry(x_m: f64, heading_rad: f64, reset_count: u64) -> Odometry {
        Odometry {
            x_m,
            heading_rad,
            reset_count,
            ..Odometry::default()
        }
    }

    fn ms(start: Instant, ms: u64) -> Instant {
        start + Duration::from_millis(ms)
    }

    #[test]
    fn interpolates_between_the_two_samples_around_the_instant() {
        let t0 = Instant::now();
        let mut buffer = OdometryBuffer::new(8);
        buffer.push(ms(t0, 0), odometry(0.0, 0.0, 0));
        buffer.push(ms(t0, 10), odometry(1.0, 0.2, 0));
        buffer.push(ms(t0, 20), odometry(3.0, 0.4, 0));

        let PoseAt::Pose(pose) = buffer.pose_at(ms(t0, 15)) else {
            panic!("15 ms is inside the buffer");
        };
        assert!((pose.x_m - 2.0).abs() < 1e-9);
        assert!((pose.heading_rad - 0.3).abs() < 1e-9);
    }

    #[test]
    fn the_heading_is_interpolated_across_plus_minus_pi() {
        let t0 = Instant::now();
        let mut buffer = OdometryBuffer::new(8);
        buffer.push(ms(t0, 0), odometry(0.0, PI - 0.1, 0));
        buffer.push(ms(t0, 10), odometry(0.0, -PI + 0.1, 0));

        let PoseAt::Pose(pose) = buffer.pose_at(ms(t0, 5)) else {
            panic!("5 ms is inside the buffer");
        };
        // Halfway along the short 0.2 rad arc is pi, not 0.
        assert!(wrap_to_pi(pose.heading_rad - PI).abs() < 1e-9);
    }

    #[test]
    fn instants_outside_the_buffer_are_not_yet_or_unavailable() {
        let t0 = Instant::now();
        let mut buffer = OdometryBuffer::new(8);
        assert_eq!(buffer.pose_at(t0), PoseAt::NotYet);
        buffer.push(ms(t0, 10), odometry(0.0, 0.0, 0));
        buffer.push(ms(t0, 20), odometry(1.0, 0.0, 0));
        assert_eq!(buffer.pose_at(ms(t0, 25)), PoseAt::NotYet);
        assert_eq!(buffer.pose_at(ms(t0, 5)), PoseAt::Unavailable);
        assert!(matches!(buffer.pose_at(ms(t0, 20)), PoseAt::Pose(_)));
    }

    #[test]
    fn a_new_reset_count_drops_every_older_sample() {
        let t0 = Instant::now();
        let mut buffer = OdometryBuffer::new(8);
        buffer.push(ms(t0, 0), odometry(5.0, 0.0, 0));
        buffer.push(ms(t0, 10), odometry(6.0, 0.0, 0));
        buffer.push(ms(t0, 20), odometry(0.0, 0.0, 1));

        assert_eq!(buffer.reset_count(), Some(1));
        assert_eq!(buffer.pose_at(ms(t0, 15)), PoseAt::Unavailable);
    }

    #[test]
    fn the_oldest_sample_is_dropped_past_capacity() {
        let t0 = Instant::now();
        let mut buffer = OdometryBuffer::new(2);
        buffer.push(ms(t0, 0), odometry(0.0, 0.0, 0));
        buffer.push(ms(t0, 10), odometry(1.0, 0.0, 0));
        buffer.push(ms(t0, 20), odometry(2.0, 0.0, 0));
        assert_eq!(buffer.pose_at(ms(t0, 5)), PoseAt::Unavailable);
        assert!(matches!(buffer.pose_at(ms(t0, 15)), PoseAt::Pose(_)));
    }
}
