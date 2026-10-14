//! The topics tying the simulated sensors ([`crate::simulation::SimulatedImu`]
//! and [`crate::simulation::SimulatedLidar`]) to whoever tunes them (e.g.
//! `web_gui`'s Simulation panel): [`SimulatedSensorParameters`] asks for new
//! values, [`SimulatedSensorStatus`] reports the ones in effect - one pair
//! of topics per sensor, mirroring
//! [`crate::topics::DetectorParameters`]/[`crate::topics::DetectorStatus`].

use crate::topics::AlgorithmParameter;
use std::collections::BTreeMap;

/// Name of the topic the IMU's [`SimulatedSensorParameters`] are published on.
pub const SIMULATED_IMU_PARAMETERS_TOPIC_NAME: &str = "simulated_imu_parameters";
/// Name of the topic the IMU's [`SimulatedSensorStatus`] is published on.
pub const SIMULATED_IMU_STATUS_TOPIC_NAME: &str = "simulated_imu_status";
/// Name of the topic the lidars' [`SimulatedSensorParameters`] are published
/// on - every simulated lidar, the opponents' included, applies them.
pub const SIMULATED_LIDAR_PARAMETERS_TOPIC_NAME: &str = "simulated_lidar_parameters";
/// Name of the topic the ego lidar's [`SimulatedSensorStatus`] is published on.
pub const SIMULATED_LIDAR_STATUS_TOPIC_NAME: &str = "simulated_lidar_status";

/// The parameter values a driver (e.g. `web_gui`) wants a simulated sensor
/// to run with, by name. Always the *whole* wanted state. Applied
/// (sanitized, see [`crate::topics::ParameterKind::sanitize`]) from the next
/// reading on, and reported back in [`SimulatedSensorStatus::parameters`].
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SimulatedSensorParameters {
    pub values: BTreeMap<String, f64>,
}

/// What a simulated sensor runs with, so a driver can reflect it.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SimulatedSensorStatus {
    /// Every tunable parameter, with the value currently in effect.
    pub parameters: Vec<AlgorithmParameter>,
}
