use std::time::{SystemTime, UNIX_EPOCH};

// Lidar Topic
pub const LIDAR_POINTS_NUMBER: usize = 1200;
#[derive(Clone)]
pub struct Scan {
    pub timestamp: u128,
    pub distances: [f32; LIDAR_POINTS_NUMBER]
}
impl Scan {
    pub fn new() -> Self{
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_micros();
        let distances = [0.0f32; LIDAR_POINTS_NUMBER];
        Scan {
            timestamp,
            distances
        }
    }
}
pub const SCAN_TOPIC_NAME: &str = "scan";