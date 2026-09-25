//! The [`VehicleModelStatus`]/[`VehicleModelSelection`] topic pair: which
//! vehicle physics model is currently wanted, and which one is currently
//! running - together, how [`crate::actuators::SimulatedVehicle`] and a
//! driver of the wanted selection (e.g. `web_gui`) agree on the live model
//! without either one needing to know how the other decides its defaults.
//! Mirrors [`crate::topics::MapSelection`]/[`crate::topics::SelectedMap`].
//!
//! The running model can also be tuned live, like an autonomous algorithm:
//! [`VehicleModelStatus`] lists its tunable [`AlgorithmParameter`]s with the
//! values in effect, and [`crate::actuators::SimulatedVehicle`] applies
//! whatever [`VehicleModelParameters`] asks for.

use super::AlgorithmParameter;
use std::collections::BTreeMap;

/// Name of the topic [`VehicleModelSelection`] is published on.
pub const VEHICLE_MODEL_SELECTION_TOPIC_NAME: &str = "vehicle_model_selection";
/// Name of the topic [`VehicleModelStatus`] is published on.
pub const VEHICLE_MODEL_STATUS_TOPIC_NAME: &str = "vehicle_model_status";
/// Name of the topic [`VehicleModelParameters`] is published on.
pub const VEHICLE_MODEL_PARAMETERS_TOPIC_NAME: &str = "vehicle_model_parameters";

/// Which vehicle physics model is wanted/running. The concrete model
/// parameters and actuator limits for each kind live in
/// [`crate::actuators::simulated_vehicle`], not here - this enum is just the
/// stable, serializable identity that flows over the selection/status
/// topics and the web API.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum VehicleModelKind {
    /// A CG-referenced kinematic bicycle model - see
    /// [`crate::environment::simulator::vehicle::bicycle`].
    #[default]
    Bicycle,
    /// A dynamic bicycle model with lateral tire forces - see
    /// [`crate::environment::simulator::vehicle::dynamic_bicycle`].
    DynamicBicycle,
    /// A dynamic bicycle model with tire saturation, load transfer, and
    /// combined slip - see
    /// [`crate::environment::simulator::vehicle::nonlinear_bicycle`].
    NonlinearBicycle,
    /// A dynamic bicycle model using the full Pacejka Magic Formula for
    /// lateral tire force - see
    /// [`crate::environment::simulator::vehicle::pacejka_bicycle`].
    PacejkaBicycle,
    /// A two-track (four-wheel) model with lateral load transfer and
    /// per-wheel asymmetry - see
    /// [`crate::environment::simulator::vehicle::two_track`].
    TwoTrack,
}

impl VehicleModelKind {
    /// Every kind, in the order a picker should offer them, each paired
    /// with the stable string the web APIs use for it, a human-readable
    /// label, and a brief description.
    ///
    /// The single source of truth for all four: `web_gui` serves it from
    /// `GET /api/vehicle_models`, and its frontend renders the
    /// labels/descriptions it's given rather than keeping its own copy.
    pub const ALL: &'static [(Self, &'static str, &'static str, &'static str)] = &[
        (
            Self::Bicycle,
            "bicycle",
            "Kinematic bicycle",
            "A CG-referenced kinematic bicycle model.",
        ),
        (
            Self::DynamicBicycle,
            "dynamic_bicycle",
            "Dynamic bicycle (tire forces)",
            "A dynamic bicycle model with lateral tire forces.",
        ),
        (
            Self::NonlinearBicycle,
            "nonlinear_bicycle",
            "Nonlinear bicycle (tire saturation + load transfer)",
            "A dynamic bicycle model with tire saturation, load transfer, and combined slip.",
        ),
        (
            Self::PacejkaBicycle,
            "pacejka_bicycle",
            "Pacejka bicycle (full Magic Formula)",
            "A dynamic bicycle model using the full Pacejka Magic Formula for lateral tire force.",
        ),
        (
            Self::TwoTrack,
            "two_track",
            "Two-track (four-wheel, lateral load transfer)",
            "A two-track (four-wheel) model with lateral load transfer and per-wheel asymmetry.",
        ),
    ];

    /// This kind's stable API string - the inverse of [`from_api_str`](Self::from_api_str).
    pub fn api_str(self) -> &'static str {
        Self::ALL
            .iter()
            .find(|(kind, ..)| *kind == self)
            .map(|(_, s, ..)| *s)
            .expect("ALL lists every VehicleModelKind")
    }

    /// This kind's human-readable label, for a picker or a status line.
    pub fn label(self) -> &'static str {
        Self::ALL
            .iter()
            .find(|(kind, ..)| *kind == self)
            .map(|(_, _, label, _)| *label)
            .expect("ALL lists every VehicleModelKind")
    }

    /// This kind's brief description, for a picker to show alongside its label.
    pub fn description(self) -> &'static str {
        Self::ALL
            .iter()
            .find(|(kind, ..)| *kind == self)
            .map(|(.., description)| *description)
            .expect("ALL lists every VehicleModelKind")
    }

    /// Parses an API string back into a kind, or `None` if it names no
    /// known model.
    pub fn from_api_str(s: &str) -> Option<Self> {
        Self::ALL
            .iter()
            .find(|(_, api, ..)| *api == s)
            .map(|(kind, ..)| *kind)
    }
}

/// The vehicle model kind a driver of the selection (e.g. `web_gui`)
/// currently wants running. [`crate::actuators::SimulatedVehicle`] polls
/// this and switches models whenever it no longer matches the currently
/// running one, publishing the change on [`VehicleModelStatus`].
#[derive(Debug, Clone, Copy, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct VehicleModelSelection {
    pub kind: VehicleModelKind,
}

/// The vehicle model kind currently running, published by
/// [`crate::actuators::SimulatedVehicle`] so a driver of the selection can
/// tell what's actually active - e.g. to reflect it in a dropdown on
/// startup, or after another client changes it.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct VehicleModelStatus {
    pub kind: VehicleModelKind,
    /// Every parameter of `kind` that can be tuned while it runs, with the
    /// value it's currently running with.
    pub parameters: Vec<AlgorithmParameter>,
    /// Every actuator limit (see [`crate::topics::ActuatorLimits`]), shared
    /// by all kinds, with the value currently in effect.
    pub limits: Vec<AlgorithmParameter>,
}

/// The parameter values a driver of the selection (e.g. `web_gui`) wants
/// each vehicle model kind to run with: kind (its
/// [`VehicleModelKind::api_str`]) -> parameter name -> value. Always the
/// *whole* wanted state, like [`crate::topics::AutonomousParameters`], so
/// two changes landing between two reads can't overwrite one another.
/// [`crate::actuators::SimulatedVehicle`] applies the running kind's entry
/// (sanitized, see [`crate::topics::ParameterKind::sanitize`]) and reports
/// the result in [`VehicleModelStatus`].
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct VehicleModelParameters {
    pub values: BTreeMap<String, BTreeMap<String, f64>>,
    /// Wanted actuator limits, by name - shared by every kind, so applied
    /// whichever one is running.
    pub limits: BTreeMap<String, f64>,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The three copies of this mapping that used to exist (web_gui's
    /// options table, the replay UI's (then `debug_web_interface`) `kind_str`, and a hardcoded
    /// label object in its JavaScript) could drift apart silently. Now
    /// there's one, and this checks it's complete.
    #[test]
    fn every_kind_round_trips_through_its_api_string() {
        for (kind, api, label, description) in VehicleModelKind::ALL {
            assert_eq!(kind.api_str(), *api);
            assert_eq!(kind.label(), *label);
            assert_eq!(kind.description(), *description);
            assert_eq!(VehicleModelKind::from_api_str(api), Some(*kind));
            assert!(!label.is_empty());
            assert!(!description.is_empty());
        }
    }

    #[test]
    fn an_unknown_api_string_is_rejected() {
        assert_eq!(VehicleModelKind::from_api_str("hovercraft"), None);
    }

    /// `api_str`/`label` panic on a kind missing from `ALL`, so a new
    /// variant must be added there too - this is what catches that.
    #[test]
    fn all_lists_every_variant() {
        let kinds = [
            VehicleModelKind::Bicycle,
            VehicleModelKind::DynamicBicycle,
            VehicleModelKind::NonlinearBicycle,
            VehicleModelKind::PacejkaBicycle,
            VehicleModelKind::TwoTrack,
        ];
        assert_eq!(VehicleModelKind::ALL.len(), kinds.len());
        for kind in kinds {
            assert!(
                VehicleModelKind::ALL.iter().any(|(k, ..)| *k == kind),
                "{kind:?} missing from ALL"
            );
        }
    }
}
