//! [`LocalizedScan`]: one lidar scan, with the pose it was taken from -
//! Karto's `LocalizedRangeScan`.

use super::pose::Pose2;
use crate::topics::LidarScan;
use std::time::Instant;

/// One usable reading, in the sensor's own frame (x forward, y left).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Reading {
    pub x_m: f64,
    pub y_m: f64,
    pub range_m: f64,
}

/// A scan and the pose it was taken from. The sensor sits on the
/// vehicle's reference point, facing forward (as
/// [`crate::sensors::SimulatedLidar`] raycasts from it), so the vehicle's
/// pose and the sensor's are the same.
#[derive(Debug, Clone)]
pub struct LocalizedScan {
    /// When the scan was written.
    pub time: Instant,
    /// Where odometry says the scan was taken from, in the `odom` frame.
    pub odometric_pose: Pose2,
    corrected_pose: Pose2,
    /// Every reading strictly between the sensor's minimum and maximum
    /// distance. A reading at the maximum is the sensor saying "nothing
    /// hit", and carries no information about where anything is - Karto
    /// drops those too.
    readings: Vec<Reading>,
    /// Readings at or beyond this distance are too noisy to match against,
    /// and only mark the cells they pass through as free, up to it -
    /// Karto's `RangeThreshold`.
    range_threshold_m: f64,
    /// Every reading closer than `range_threshold_m`, in the SLAM frame at
    /// `corrected_pose` - kept in reading order, which
    /// [`super::scan_matcher`] relies on.
    world_points: Vec<(f64, f64)>,
}

impl LocalizedScan {
    /// `scan`, taken from `odometric_pose` at `time`. Its corrected pose
    /// starts out equal to `odometric_pose`.
    pub fn new(
        scan: &LidarScan,
        time: Instant,
        odometric_pose: Pose2,
        range_threshold_m: f64,
    ) -> Self {
        let min_m = f64::from(scan.min_distance);
        let max_m = f64::from(scan.max_distance);
        let readings = scan
            .points
            .iter()
            .enumerate()
            .filter_map(|(i, &range)| {
                let range_m = f64::from(range);
                if !range_m.is_finite() || range_m <= min_m || range_m >= max_m {
                    return None;
                }
                let (sin, cos) = f64::from(scan.angle_rad(i)).sin_cos();
                Some(Reading {
                    x_m: range_m * cos,
                    y_m: range_m * sin,
                    range_m,
                })
            })
            .collect();
        let mut localized = Self {
            time,
            odometric_pose,
            corrected_pose: odometric_pose,
            readings,
            range_threshold_m,
            world_points: Vec::new(),
        };
        localized.set_corrected_pose(odometric_pose);
        localized
    }

    /// Where the scan was taken from, in the SLAM frame.
    pub fn corrected_pose(&self) -> Pose2 {
        self.corrected_pose
    }

    /// Moves the scan to `pose`, in the SLAM frame.
    pub fn set_corrected_pose(&mut self, pose: Pose2) {
        self.corrected_pose = pose;
        self.world_points = self
            .matchable_readings()
            .map(|reading| pose.transform_point(reading.x_m, reading.y_m))
            .collect();
    }

    /// Every reading, in the sensor's own frame.
    pub fn readings(&self) -> &[Reading] {
        &self.readings
    }

    /// The readings close enough to match against - see
    /// [`range_threshold_m`](Self::range_threshold_m).
    pub fn matchable_readings(&self) -> impl Iterator<Item = &Reading> {
        self.readings
            .iter()
            .filter(|reading| reading.range_m < self.range_threshold_m)
    }

    /// [`matchable_readings`](Self::matchable_readings), in the SLAM frame.
    pub fn world_points(&self) -> &[(f64, f64)] {
        &self.world_points
    }

    pub fn range_threshold_m(&self) -> f64 {
        self.range_threshold_m
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn lidar_scan(points: Vec<f32>) -> LidarScan {
        let n = points.len();
        LidarScan::new(points, vec![1.0; n], 0.1, 10.0, std::f32::consts::PI)
    }

    #[test]
    fn readings_at_the_distance_limits_are_dropped() {
        let scan = LocalizedScan::new(
            &lidar_scan(vec![0.05, 2.0, 10.0]),
            Instant::now(),
            Pose2::default(),
            8.0,
        );
        assert_eq!(scan.readings().len(), 1);
        // The middle of three rays across pi points straight ahead.
        let reading = scan.readings()[0];
        assert!((reading.x_m - 2.0).abs() < 1e-6 && reading.y_m.abs() < 1e-6);
    }

    #[test]
    fn world_points_follow_the_corrected_pose_and_skip_far_readings() {
        let mut scan = LocalizedScan::new(
            &lidar_scan(vec![1.0, 2.0, 9.0]),
            Instant::now(),
            Pose2::default(),
            8.0,
        );
        assert_eq!(scan.world_points().len(), 2);

        // Facing +y from (1, 1): the reading 2 m straight ahead is at (1, 3).
        scan.set_corrected_pose(Pose2::new(1.0, 1.0, FRAC_PI_2));
        let (x, y) = scan.world_points()[1];
        assert!((x - 1.0).abs() < 1e-6 && (y - 3.0).abs() < 1e-6);
    }
}
