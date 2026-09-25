//! The [`LidarScan`] topic: one full sweep from a 2D LIDAR sensor.

/// Name of the topic a [`LidarScan`] is published on.
pub const LIDAR_SCAN_TOPIC_NAME: &str = "lidar_scan";

/// One full sweep from a 2D LIDAR sensor: a set of distance and intensity
/// readings spread evenly across the sensor's field of view.
///
/// `points` and `intensities` are parallel vectors - `intensities[i]` is the
/// signal strength of the reading at `points[i]` - and both are expected to
/// have exactly `num_lidar_points` entries, evenly spaced across `fov`
/// radians.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct LidarScan {
    /// Number of readings in this scan; equal to both `points.len()` and
    /// `intensities.len()`.
    pub num_lidar_points: usize,
    /// Distance reading for each point, in meters.
    pub points: Vec<f32>,
    /// Smallest distance the sensor can reliably report, in meters.
    pub min_distance: f32,
    /// Largest distance the sensor can reliably report, in meters.
    pub max_distance: f32,
    /// The sensor's field of view, in radians.
    pub fov: f32,
    /// Signal intensity for each point, parallel to `points`.
    pub intensities: Vec<f32>,
}

impl LidarScan {
    /// Builds a scan from `points`/`intensities` readings. `num_lidar_points` is
    /// derived from their shared length. When it was written is tracked by the
    /// topic itself - see [`crate::WriteMeta`].
    ///
    /// # Panics
    ///
    /// Panics if `points` and `intensities` don't have the same length - they
    /// must be parallel, one entry per point.
    pub fn new(
        points: Vec<f32>,
        intensities: Vec<f32>,
        min_distance: f32,
        max_distance: f32,
        fov: f32,
    ) -> Self {
        assert_eq!(
            points.len(),
            intensities.len(),
            "LidarScan: points and intensities must have the same length"
        );
        let num_lidar_points = points.len();
        Self {
            num_lidar_points,
            points,
            min_distance,
            max_distance,
            fov,
            intensities,
        }
    }

    /// The angle of reading `index` (of `num_points` total), relative to the
    /// sensor's forward direction: readings are spread evenly across `fov`,
    /// with the first and last landing exactly on the FOV's edges. Shared by
    /// whoever produces a scan and whoever interprets one, so the two can't
    /// disagree on where a reading points.
    pub fn ray_angle_rad(fov: f32, num_points: usize, index: usize) -> f32 {
        if num_points <= 1 {
            return 0.0;
        }
        -fov / 2.0 + index as f32 * fov / (num_points - 1) as f32
    }

    /// The angle of `points[index]`, relative to the sensor's forward
    /// direction - see [`Self::ray_angle_rad`].
    pub fn angle_rad(&self, index: usize) -> f32 {
        Self::ray_angle_rad(self.fov, self.num_lidar_points, index)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_derives_num_lidar_points_from_the_vectors() {
        let scan = LidarScan::new(
            vec![1.0, 2.0, 3.0],
            vec![0.1, 0.2, 0.3],
            0.1,
            12.0,
            std::f32::consts::PI,
        );
        assert_eq!(scan.num_lidar_points, 3);
        assert_eq!(scan.points.len(), scan.intensities.len());
    }

    #[test]
    #[should_panic(expected = "must have the same length")]
    fn new_panics_on_mismatched_lengths() {
        LidarScan::new(vec![1.0, 2.0], vec![0.1], 0.1, 12.0, std::f32::consts::PI);
    }
}
