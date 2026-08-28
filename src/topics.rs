//! Shared topic definitions: the concrete types published/read on named
//! [`crate::RwLockTopic`]s, so multiple binaries can agree on the same shape
//! without redefining it.

mod lidar_scan;

pub use lidar_scan::{LIDAR_SCAN_TOPIC_NAME, LidarScan};
