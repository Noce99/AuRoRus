//! Shared topic definitions: the concrete types published/read on named
//! [`crate::RwLockTopic`]s, so multiple binaries can agree on the same shape
//! without redefining it.

mod lidar_scan;
mod map;
mod vehicle_status;
mod vesc_command;

pub use lidar_scan::{LIDAR_SCAN_TOPIC_NAME, LidarScan};
pub use map::{MAP_SELECTION_TOPIC_NAME, MAP_TOPIC_NAME, MapSelection, SelectedMap};
pub use vehicle_status::{VEHICLE_STATUS_TOPIC_NAME, VehicleStatus};
pub use vesc_command::{HUMAN_VESC_COMMAND_TOPIC_NAME, VESC_COMMAND_TOPIC_NAME, VescCommand};
