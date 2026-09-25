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
//!
//! Algorithms can also be tuned live: each lists its tunable
//! [`AlgorithmParameter`]s, with their current values, in its
//! [`AutonomousAlgorithmInfo`], and applies whatever [`AutonomousParameters`]
//! asks for - see [`crate::autonomous_control::ParameterTuner`].

use std::collections::BTreeMap;

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
/// Name of the topic [`AutonomousParameters`] is published on.
pub const AUTONOMOUS_PARAMETERS_TOPIC_NAME: &str = "autonomous_parameters";

/// What an algorithm says about itself, for a picker in a UI. Written when
/// the algorithm claims its topics, and rewritten only when one of its
/// `parameters`' values changes.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AutonomousAlgorithmInfo {
    /// Short, human-readable name, e.g. `"Always left"`.
    pub label: String,
    /// One or two sentences on what the algorithm does.
    pub description: String,
    /// Every parameter that can be tuned while the algorithm runs, with the
    /// value it's currently running with. Empty if none.
    pub parameters: Vec<AlgorithmParameter>,
    /// What the algorithm wants its driver to know right now, e.g. why it's
    /// holding the vehicle stopped - `None` if nothing. Set with
    /// [`crate::autonomous_control::report_message`].
    pub message: Option<String>,
}

impl AutonomousAlgorithmInfo {
    pub fn new(label: impl Into<String>, description: impl Into<String>) -> Self {
        Self {
            label: label.into(),
            description: description.into(),
            parameters: Vec::new(),
            message: None,
        }
    }

    /// Declares `parameters` as tunable, taking each one's current value from
    /// the field of the same name in `config` - see
    /// [`crate::autonomous_control::ParameterTuner`].
    ///
    /// # Panics
    ///
    /// Panics if `config` has no numeric field named after one of
    /// `parameters` - a typo in the declaration, caught at startup.
    pub fn with_parameters<C: serde::Serialize>(
        mut self,
        config: &C,
        parameters: impl IntoIterator<Item = AlgorithmParameter>,
    ) -> Self {
        self.parameters = parameters.into_iter().collect();
        self.refresh_values(config);
        self
    }

    /// Re-reads every parameter's `value` from the field of the same name in
    /// `config`. Panics like [`with_parameters`](Self::with_parameters).
    pub fn refresh_values<C: serde::Serialize>(&mut self, config: &C) {
        crate::config::refresh_parameter_values(&mut self.parameters, config);
    }
}

/// One parameter of an algorithm that can be tuned while it runs, e.g. from
/// a slider in `web_gui`. Named after the config field it sets.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AlgorithmParameter {
    /// The config field it sets, e.g. `"t_m"`.
    pub name: String,
    /// What it does, for a tooltip or a line under the slider.
    pub description: String,
    /// Unit shown next to the value, e.g. `"m"` - empty for a pure number.
    pub unit: String,
    /// Range and granularity of the values it accepts.
    pub kind: ParameterKind,
    /// The value the algorithm is currently running with. Always `f64`, even
    /// for an [`ParameterKind::Int`] (then a whole number), so a UI handles
    /// every parameter alike.
    pub value: f64,
}

impl AlgorithmParameter {
    /// A real-valued parameter in `min..=max`, moved in steps of `step`.
    pub fn float(name: impl Into<String>, min: f64, max: f64, step: f64) -> Self {
        Self::new(name, ParameterKind::Float { min, max, step })
    }

    /// A whole-number parameter in `min..=max`, moved in steps of `step`.
    pub fn int(name: impl Into<String>, min: i64, max: i64, step: i64) -> Self {
        Self::new(name, ParameterKind::Int { min, max, step })
    }

    fn new(name: impl Into<String>, kind: ParameterKind) -> Self {
        Self {
            name: name.into(),
            description: String::new(),
            unit: String::new(),
            kind,
            value: 0.0,
        }
    }

    pub fn description(mut self, description: impl Into<String>) -> Self {
        self.description = description.into();
        self
    }

    pub fn unit(mut self, unit: impl Into<String>) -> Self {
        self.unit = unit.into();
        self
    }
}

/// The values an [`AlgorithmParameter`] accepts.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ParameterKind {
    Float { min: f64, max: f64, step: f64 },
    Int { min: i64, max: i64, step: i64 },
}

impl ParameterKind {
    /// `value` brought into range - rounded to a whole number too, for an
    /// [`Int`](Self::Int) - or `None` if it isn't a number at all (NaN or
    /// infinite).
    pub fn sanitize(&self, value: f64) -> Option<f64> {
        if !value.is_finite() {
            return None;
        }
        Some(match *self {
            Self::Float { min, max, .. } => value.clamp(min, max),
            Self::Int { min, max, .. } => value.round().clamp(min as f64, max as f64),
        })
    }

    /// An already [`sanitize`](Self::sanitize)d `value` as the JSON a config
    /// field of this kind deserializes from - an integer for an
    /// [`Int`](Self::Int), so it fits a `usize`/`u32` field too.
    pub fn to_json(&self, value: f64) -> serde_json::Value {
        match self {
            Self::Float { .. } => serde_json::Value::from(value),
            Self::Int { .. } => serde_json::Value::from(value as i64),
        }
    }
}

/// The parameter values a driver of the selection (e.g. `web_gui`) wants
/// each algorithm to run with: algorithm name -> parameter name -> value.
/// Always the *whole* wanted state rather than one change at a time, so two
/// changes landing between two of an algorithm's reads can't overwrite one
/// another - topics only ever hold their latest value. An algorithm applies
/// its own entry (sanitized, see [`ParameterKind::sanitize`]) and reports
/// the result in its [`AutonomousAlgorithmInfo`].
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AutonomousParameters {
    pub values: BTreeMap<String, BTreeMap<String, f64>>,
}

/// Which algorithm a driver of the selection (e.g. `web_gui`) has picked,
/// by its name (the part of its topics' names after the prefix), and
/// whether it should actually drive - paused, or with nothing picked,
/// there's no autonomous control at all, only a human driver.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AutonomousAlgorithmSelection {
    /// The algorithm picked, or `None` if none is yet.
    pub name: Option<String>,
    /// Whether `name` is in control (started) rather than paused.
    pub running: bool,
}

/// One algorithm [`crate::autonomous_control::AutonomousControlsHandler`]
/// found, as listed in [`AutonomousAlgorithmStatus::available`].
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AvailableAlgorithm {
    pub name: String,
    pub label: String,
    pub description: String,
    /// See [`AutonomousAlgorithmInfo::parameters`].
    pub parameters: Vec<AlgorithmParameter>,
    /// See [`AutonomousAlgorithmInfo::message`].
    pub message: Option<String>,
}

/// What [`crate::autonomous_control::AutonomousControlsHandler`] is
/// currently doing, so a driver of the selection can reflect it.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AutonomousAlgorithmStatus {
    /// The algorithm picked, running or paused, or `None` if nothing is
    /// picked (or the selection names no known algorithm).
    pub selected: Option<String>,
    /// The algorithm whose command is being forwarded: `selected` while it's
    /// running, `None` while it's paused.
    pub active: Option<String>,
    /// Every algorithm found, sorted by name.
    pub available: Vec<AvailableAlgorithm>,
    /// Whether `active`'s latest command is recent enough to be forwarded
    /// (see [`crate::topics::VESC_COMMAND_TIMEOUT`]) - if not, a stationary,
    /// centered command is forwarded instead. `false` when nothing is active.
    pub command_fresh: bool,
}
