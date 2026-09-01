use std::time::{SystemTime, UNIX_EPOCH};

// Lidar Topic
pub const LIDAR_POINTS_NUMBER: usize = 1200;
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct Scan {
    pub _timestamp: u128,
    pub _distances: Vec<f32>,
}
impl Scan {
    pub fn new() -> Self {
        let _timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_micros();
        let _distances = vec![0.0f32; LIDAR_POINTS_NUMBER];
        Scan {
            _timestamp,
            _distances,
        }
    }
}
pub const SCAN_TOPIC_NAME: &str = "scan";
