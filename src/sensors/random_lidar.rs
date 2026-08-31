//! [`RandomLidar`]: a synthetic LIDAR sensor that publishes random
//! [`LidarScan`]s, for exercising algorithms without real hardware.

use crate::topics::{LIDAR_SCAN_TOPIC_NAME, LidarScan};
use crate::{Captain, Executor};
use rand::RngExt;
use std::any::Any;
use std::thread;
use std::time::Duration;

/// Every tunable parameter [`RandomLidar`] needs - loaded from
/// `config/sensors/random_lidar.toml` (see [`Default`]) or from an arbitrary
/// path via [`crate::config::load`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Deserialize)]
pub struct RandomLidarConfig {
    /// Rate at which [`RandomLidar`] publishes a new scan, in Hz.
    pub rate_hz: f64,
    /// How many points [`RandomLidar`] puts in every scan.
    pub num_points: usize,
    /// [`RandomLidar`]'s reported minimum distance, in meters.
    pub min_distance_m: f32,
    /// [`RandomLidar`]'s reported maximum distance, in meters.
    pub max_distance_m: f32,
    /// [`RandomLidar`]'s reported field of view, in radians.
    pub fov_rad: f32,
    /// Lower bound of the intensity values [`RandomLidar`] reports per
    /// point.
    pub intensity_min: f32,
    /// Upper bound of the intensity values [`RandomLidar`] reports per
    /// point.
    pub intensity_max: f32,
}

impl Default for RandomLidarConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/sensors/random_lidar.toml"))
            .expect("config/sensors/random_lidar.toml must deserialize into RandomLidarConfig")
    }
}

/// A synthetic LIDAR sensor: claims [`LIDAR_SCAN_TOPIC_NAME`] and publishes a
/// [`LidarScan`] of uniformly random distances and intensities at
/// [`RandomLidarConfig::rate_hz`], useful for exercising downstream
/// algorithms without real hardware.
pub struct RandomLidar {
    id: u8,
    name: String,
    config: RandomLidarConfig,
}

impl RandomLidar {
    pub fn new(name: impl Into<String>, config: RandomLidarConfig) -> Self {
        Self {
            id: 0,
            name: name.into(),
            config,
        }
    }
}

impl Executor for RandomLidar {
    fn init(&mut self, id: u8) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        let config = self.config;
        captain.claim_writer::<LidarScan>(LIDAR_SCAN_TOPIC_NAME, self.id, move || {
            LidarScan::new(Vec::new(), Vec::new(), config.min_distance_m, config.max_distance_m, config.fov_rad)
        });
    }

    fn run(&mut self, captain: &Captain) {
        let topic = captain.topic::<LidarScan>(LIDAR_SCAN_TOPIC_NAME);
        let interval = Duration::from_secs_f64(1.0 / self.config.rate_hz);
        let mut rng = rand::rng();
        let intensity_range = self.config.intensity_min..=self.config.intensity_max;

        while captain.is_running(self.id) {
            let points = (0..self.config.num_points)
                .map(|_| rng.random_range(self.config.min_distance_m..=self.config.max_distance_m))
                .collect();
            let intensities = (0..self.config.num_points)
                .map(|_| rng.random_range(intensity_range.clone()))
                .collect();

            topic
                .write(
                    self.id,
                    LidarScan::new(points, intensities, self.config.min_distance_m, self.config.max_distance_m, self.config.fov_rad),
                )
                .expect("lost writer authorization for the lidar_scan topic");

            thread::sleep(interval);
        }
    }

    fn name(&self) -> String {
        self.name.clone()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn fresh(&self) -> Box<dyn Executor> {
        Box::new(RandomLidar::new(self.name.clone(), self.config))
    }
}
