//! Perception: finding what's around the ego vehicle in its sensor data.
//! [`UbmDetector`] finds one opponent, where the lidar scan is shorter than
//! the map predicts.

mod config;
mod map_difference;
mod ubm_detector;

pub use config::{
    UbmDetectorConfig, config_path, save_parameters, saved_values, tunable_parameters,
};
pub use ubm_detector::UbmDetector;
