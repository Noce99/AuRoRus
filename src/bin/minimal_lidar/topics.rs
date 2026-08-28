use std::time::{SystemTime, UNIX_EPOCH};

// Lidar Topic
pub const LIDAR_POINTS_NUMBER: usize = 1200;
#[derive(Clone)]
pub struct Scan {
    pub _timestamp: u128,
    pub _distances: [f32; LIDAR_POINTS_NUMBER],
}
impl Scan {
    pub fn new() -> Self {
        let _timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_micros();
        let _distances = [0.0f32; LIDAR_POINTS_NUMBER];
        Scan {
            _timestamp,
            _distances,
        }
    }
}
pub const SCAN_TOPIC_NAME: &str = "scan";
