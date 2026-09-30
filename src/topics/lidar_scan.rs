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
    /// Where the sensor sits on its vehicle, in meters, in the vehicle's frame
    /// (x forward, y left, from the point its pose describes): every reading
    /// starts here, not at the vehicle's pose - see [`Self::origin_m`]. The
    /// sensor faces the vehicle's forward direction.
    pub mount_x_m: f32,
    /// See [`Self::mount_x_m`].
    pub mount_y_m: f32,
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
            mount_x_m: 0.0,
            mount_y_m: 0.0,
        }
    }

    /// This scan, taken by a sensor mounted at `(x_m, y_m)` on its vehicle -
    /// see [`Self::mount_x_m`]. Unmounted scans start at the vehicle's pose.
    pub fn mounted_at(self, x_m: f32, y_m: f32) -> Self {
        Self {
            mount_x_m: x_m,
            mount_y_m: y_m,
            ..self
        }
    }

    /// Where the sensor was, in the world frame, when its vehicle was at
    /// `(x_m, y_m)` facing `heading_rad`: the point every reading starts from.
    /// The sensor faces `heading_rad` too.
    pub fn origin_m(&self, x_m: f64, y_m: f64, heading_rad: f64) -> (f64, f64) {
        Self::sensor_origin_m(self.mount_x_m, self.mount_y_m, x_m, y_m, heading_rad)
    }

    /// [`Self::origin_m`] of a sensor mounted at `(mount_x_m, mount_y_m)`,
    /// for whoever produces a scan before having one.
    pub fn sensor_origin_m(
        mount_x_m: f32,
        mount_y_m: f32,
        x_m: f64,
        y_m: f64,
        heading_rad: f64,
    ) -> (f64, f64) {
        let (sin, cos) = heading_rad.sin_cos();
        let (mx, my) = (f64::from(mount_x_m), f64::from(mount_y_m));
        (x_m + mx * cos - my * sin, y_m + mx * sin + my * cos)
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
    fn the_origin_follows_the_mount_rotated_by_the_heading() {
        let scan = LidarScan::new(vec![1.0], vec![1.0], 0.1, 12.0, 1.0).mounted_at(0.3, 0.1);
        let (x, y) = scan.origin_m(2.0, 1.0, std::f64::consts::FRAC_PI_2);
        // Facing +y: forward is +y, left is -x.
        assert!((x - 1.9).abs() < 1e-6 && (y - 1.3).abs() < 1e-6, "({x}, {y})");
    }

    #[test]
    #[should_panic(expected = "must have the same length")]
    fn new_panics_on_mismatched_lengths() {
        LidarScan::new(vec![1.0, 2.0], vec![0.1], 0.1, 12.0, std::f32::consts::PI);
    }
}
