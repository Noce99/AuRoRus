//! The topics tying autonomous algorithms together - see
//! [`crate::autonomous_control`] for how they're used.
//!
//! Every algorithm publishes its own [`crate::topics::VescCommand`] on
//! [`AUTONOMOUS_CONTROL_TOPIC_PREFIX`] followed by its name, plus a static
//! [`AutonomousAlgorithmInfo`] on [`AUTONOMOUS_CONTROL_INFO_TOPIC_PREFIX`]
//! followed by the same name - both claimed via
//! [`crate::Captain::claim_autonomous_control`].
//! [`crate::autonomous_control::AutonomousControlsHandler`] finds every
//! algorithm by that info prefix, forwards whichever one
//! [`AutonomousAlgorithmSelection`] names, and reports what it did on
//! [`AutonomousAlgorithmStatus`] - mirroring
//! [`crate::topics::VehicleModelSelection`]/[`crate::topics::VehicleModelStatus`].

/// Prefix of every algorithm's own command topic, e.g.
/// `autonomous_control/always_left`.
pub const AUTONOMOUS_CONTROL_TOPIC_PREFIX: &str = "autonomous_control/";
/// Prefix of every algorithm's [`AutonomousAlgorithmInfo`] topic, e.g.
/// `autonomous_control_info/always_left`.
pub const AUTONOMOUS_CONTROL_INFO_TOPIC_PREFIX: &str = "autonomous_control_info/";
/// Name of the topic [`AutonomousAlgorithmSelection`] is published on.
pub const AUTONOMOUS_ALGORITHM_SELECTION_TOPIC_NAME: &str = "autonomous_algorithm_selection";
/// Name of the topic [`AutonomousAlgorithmStatus`] is published on.
pub const AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME: &str = "autonomous_algorithm_status";

/// What an algorithm says about itself, for a picker in a UI. Written once,
/// when the algorithm claims its topics - it never changes.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AutonomousAlgorithmInfo {
    /// Short, human-readable name, e.g. `"Always left"`.
    pub label: String,
    /// One or two sentences on what the algorithm does.
    pub description: String,
}

impl AutonomousAlgorithmInfo {
    pub fn new(label: impl Into<String>, description: impl Into<String>) -> Self {
        Self {
            label: label.into(),
            description: description.into(),
        }
    }
}

/// Which algorithm a driver of the selection (e.g. `web_gui`) wants in
/// control, by its name (the part of its topics' names after the prefix) -
/// or `None` for no autonomous control at all, only a human driver.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AutonomousAlgorithmSelection {
    pub name: Option<String>,
}

/// One algorithm [`crate::autonomous_control::AutonomousControlsHandler`]
/// found, as listed in [`AutonomousAlgorithmStatus::available`].
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AvailableAlgorithm {
    pub name: String,
    pub label: String,
    pub description: String,
}

/// What [`crate::autonomous_control::AutonomousControlsHandler`] is
/// currently doing, so a driver of the selection can reflect it.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AutonomousAlgorithmStatus {
    /// The algorithm whose command is being forwarded, or `None` if nothing
    /// is selected (or the selection names no known algorithm).
    pub active: Option<String>,
    /// Every algorithm found, sorted by name.
    pub available: Vec<AvailableAlgorithm>,
    /// Whether `active`'s latest command is recent enough to be forwarded
    /// (see [`crate::topics::VESC_COMMAND_TIMEOUT`]) - if not, a stationary,
    /// centered command is forwarded instead. `false` when nothing is active.
    pub command_fresh: bool,
}
