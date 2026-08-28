//! Sensor drivers: [`crate::Executor`]s that publish real-world readings onto
//! a topic.

mod random_lidar;

pub use random_lidar::RandomLidar;
