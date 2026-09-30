//! The [`VescParametersStatus`]/[`VescParameters`] topic pair: how the real
//! car is driven (`config/actuators/vesc.toml`) tuned live, like a
//! vehicle model's (see [`crate::topics::VehicleModelStatus`]).
//! [`crate::actuators::Vesc`] lists every numeric value of its config with
//! the value in effect, and applies whatever a driver (e.g. `web_gui`) asks
//! for. Nothing publishes either in simulation.

use super::AlgorithmParameter;
use std::collections::BTreeMap;

/// Name of the topic [`VescParametersStatus`] is published on.
pub const VESC_PARAMETERS_STATUS_TOPIC_NAME: &str = "vesc_parameters_status";
/// Name of the topic [`VescParameters`] is published on.
pub const VESC_PARAMETERS_TOPIC_NAME: &str = "vesc_parameters";

/// The calibration [`crate::actuators::Vesc`] currently drives with.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct VescParametersStatus {
    /// Every tunable top-level value of `vesc.toml`, with the value in effect.
    pub parameters: Vec<AlgorithmParameter>,
    /// Every actuator limit but the steering angle, which is the car's (its
    /// `[limits]` table, see [`crate::actuators::VescLimits`]), with the
    /// value in effect.
    pub limits: Vec<AlgorithmParameter>,
}

/// The values a driver (e.g. `web_gui`) wants [`crate::actuators::Vesc`] to
/// drive with, by name. Always the *whole* wanted state, like
/// [`crate::topics::VehicleModelParameters`]. Applied sanitized, and only if
/// the whole resulting calibration is still valid - see
/// [`crate::actuators::VescConfig::validate`].
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct VescParameters {
    pub values: BTreeMap<String, f64>,
    pub limits: BTreeMap<String, f64>,
}
