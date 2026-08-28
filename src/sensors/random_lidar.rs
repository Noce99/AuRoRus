//! [`RandomLidar`]: a synthetic LIDAR sensor that publishes random
//! [`LidarScan`]s, for exercising algorithms without real hardware.

use crate::topics::{LIDAR_SCAN_TOPIC_NAME, LidarScan};
use crate::{Captain, Executor};
use rand::RngExt;
use std::any::Any;
use std::f32::consts::TAU;
use std::thread;
use std::time::Duration;

/// Rate at which [`RandomLidar`] publishes a new scan.
const RATE_HZ: f64 = 50.0;
/// How many points [`RandomLidar`] puts in every scan.
const NUM_POINTS: usize = 360;
/// [`RandomLidar`]'s reported minimum distance, in meters.
const MIN_DISTANCE: f32 = 0.15;
/// [`RandomLidar`]'s reported maximum distance, in meters.
const MAX_DISTANCE: f32 = 12.0;
/// [`RandomLidar`]'s reported field of view: a full rotation, in radians.
const FOV: f32 = TAU;
/// Range of intensity values [`RandomLidar`] reports per point.
const INTENSITY_RANGE: std::ops::RangeInclusive<f32> = 0.0..=255.0;

/// A synthetic LIDAR sensor: claims [`LIDAR_SCAN_TOPIC_NAME`] and publishes a
/// [`LidarScan`] of uniformly random distances and intensities at
/// [`RATE_HZ`], useful for exercising downstream algorithms without real
/// hardware.
pub struct RandomLidar {
    id: u8,
    name: String,
}

impl RandomLidar {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            id: 0,
            name: name.into(),
        }
    }
}

impl Executor for RandomLidar {
    fn init(&mut self, id: u8) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_writer::<LidarScan>(LIDAR_SCAN_TOPIC_NAME, self.id, || {
            LidarScan::new(Vec::new(), Vec::new(), MIN_DISTANCE, MAX_DISTANCE, FOV)
        });
    }

    fn run(&mut self, captain: &Captain) {
        let topic = captain.topic::<LidarScan>(LIDAR_SCAN_TOPIC_NAME);
        let interval = Duration::from_secs_f64(1.0 / RATE_HZ);
        let mut rng = rand::rng();

        while captain.is_running(self.id) {
            let points = (0..NUM_POINTS)
                .map(|_| rng.random_range(MIN_DISTANCE..=MAX_DISTANCE))
                .collect();
            let intensities = (0..NUM_POINTS)
                .map(|_| rng.random_range(INTENSITY_RANGE))
                .collect();

            topic
                .write(
                    self.id,
                    LidarScan::new(points, intensities, MIN_DISTANCE, MAX_DISTANCE, FOV),
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
}
