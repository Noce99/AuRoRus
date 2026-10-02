//! [`SimulatedVehicleConfig`]: every model's physical parameters and the
//! actuator limits - loading them, tuning them live, and saving them back
//! to their file.

use super::model::VehicleModel;
use crate::calibration::CarCalibration;
use crate::simulation::vehicle_models::{
    BicycleParams, DynamicParams, NonlinearTireParams, PacejkaTireParams, TwoTrackParams,
};
use crate::topics::{
    ActuatorLimits, AlgorithmParameter, VehicleGeometry, VehicleModelKind, VehicleModelStatus,
};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// Every tunable parameter [`SimulatedVehicle`](super::SimulatedVehicle) needs: how often it ticks,
/// the [`ActuatorLimits`] shared by every model kind, and each kind's own
/// physical parameters - loaded from `config/simulation/vehicle.toml`
/// (see [`Default`]) or from an arbitrary path via [`crate::config::load`].
/// What the simulated car itself is - its size, mass and largest steering
/// angle - comes from a car's calibration instead: see [`Self::for_car`],
/// which a loaded config needs before it's used.
#[derive(Debug, Clone, PartialEq, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SimulatedVehicleConfig {
    /// How often [`SimulatedVehicle`](super::SimulatedVehicle) advances the model and republishes
    /// [`VehicleStatus`](crate::topics::VehicleStatus), and how often it checks
    /// [`VEHICLE_MODEL_SELECTION_TOPIC_NAME`](crate::topics::VEHICLE_MODEL_SELECTION_TOPIC_NAME) for a wanted model switch, in
    /// Hz.
    pub tick_rate_hz: f64,
    /// Shared by every [`VehicleModelKind`] - see [`ActuatorLimits`].
    pub limits: ActuatorLimits,
    pub bicycle: BicycleParams,
    pub dynamic_bicycle: DynamicParams,
    pub nonlinear_bicycle: NonlinearTireParams,
    pub pacejka_bicycle: PacejkaTireParams,
    pub two_track: TwoTrackParams,
    /// The simulated car's size, published on
    /// [`VehicleTopics::vehicle_geometry`](crate::topics::VehicleTopics::vehicle_geometry) - see [`Self::for_car`].
    #[serde(skip)]
    pub geometry: VehicleGeometry,
}

impl Default for SimulatedVehicleConfig {
    /// RC-car-scale geometry and actuator limits for a small (roughly
    /// 1/10-scale) RC racecar, matching the kind of track the map generator
    /// produces, from the checked-in `config/simulation/vehicle.toml`.
    /// `dynamic_bicycle`'s mass/inertia/cornering-stiffness values are
    /// placeholder estimates for that same scale of vehicle, not measured -
    /// tune them once a real (or more carefully modeled) vehicle is
    /// available. This is a deliberate exception to
    /// [`DynamicParams`]/[`BicycleParams`] having no [`Default`]: a live,
    /// web-selectable model needs *some* starting parameters for a kind the
    /// caller only names, not configures. Simulates the template car (see
    /// [`CarCalibration::template`]).
    fn default() -> Self {
        let config: Self = toml::from_str(include_str!("../../../config/simulation/vehicle.toml"))
            .expect("config/simulation/vehicle.toml must deserialize into SimulatedVehicleConfig");
        config.for_car(&CarCalibration::template("template"))
    }
}

impl SimulatedVehicleConfig {
    /// This config simulating `car`: every model kind with its mass and axle
    /// distances (and the two-track's track width), steering at most as far
    /// as it does both ways, and its size published.
    pub fn for_car(mut self, car: &CarCalibration) -> Self {
        let geometry = car.vehicle_geometry();
        let (lf_m, lr_m, mass_kg) = (geometry.lf_m(), geometry.lr_m(), car.geometry.mass_kg);
        (self.bicycle.lf_m, self.bicycle.lr_m) = (lf_m, lr_m);
        let dynamic = &mut self.dynamic_bicycle;
        (dynamic.lf_m, dynamic.lr_m, dynamic.mass_kg) = (lf_m, lr_m, mass_kg);
        let nonlinear = &mut self.nonlinear_bicycle;
        (nonlinear.lf_m, nonlinear.lr_m, nonlinear.mass_kg) = (lf_m, lr_m, mass_kg);
        let pacejka = &mut self.pacejka_bicycle;
        (pacejka.lf_m, pacejka.lr_m, pacejka.mass_kg) = (lf_m, lr_m, mass_kg);
        let two_track = &mut self.two_track;
        (two_track.lf_m, two_track.lr_m, two_track.mass_kg) = (lf_m, lr_m, mass_kg);
        two_track.track_width_m = geometry.track_width_m;
        self.limits.max_steering_angle_rad = car.steering.max_angle_rad();
        self.geometry = geometry;
        self
    }
}

/// The default [`VehicleModel`] for `kind`, built from `config`.
pub fn default_model(kind: VehicleModelKind, config: &SimulatedVehicleConfig) -> VehicleModel {
    match kind {
        VehicleModelKind::Bicycle => VehicleModel::Bicycle {
            params: config.bicycle,
            limits: config.limits,
        },
        VehicleModelKind::DynamicBicycle => VehicleModel::DynamicBicycle {
            params: config.dynamic_bicycle,
            limits: config.limits,
        },
        VehicleModelKind::NonlinearBicycle => VehicleModel::NonlinearBicycle {
            params: config.nonlinear_bicycle,
            limits: config.limits,
        },
        VehicleModelKind::PacejkaBicycle => VehicleModel::PacejkaBicycle {
            params: config.pacejka_bicycle,
            limits: config.limits,
        },
        VehicleModelKind::TwoTrack => VehicleModel::TwoTrack {
            params: config.two_track,
            limits: config.limits,
        },
    }
}

/// Where [`SimulatedVehicle`](super::SimulatedVehicle)'s config lives:
/// `config/simulation/vehicle.toml`, relative to the working
/// directory - rewritten by [`save_parameters`], and reread on a restart.
pub fn config_path() -> PathBuf {
    Path::new(crate::config::DEFAULT_CONFIG_ROOT)
        .join("simulation")
        .join("vehicle.toml")
}

/// `kind`'s live-tunable parameters, with the values `config` holds for it.
pub(super) fn tunable_parameters(
    kind: VehicleModelKind,
    config: &SimulatedVehicleConfig,
) -> Vec<AlgorithmParameter> {
    fn with_values(
        mut parameters: Vec<AlgorithmParameter>,
        params: &impl serde::Serialize,
    ) -> Vec<AlgorithmParameter> {
        crate::config::refresh_parameter_values(&mut parameters, params);
        parameters
    }
    match kind {
        VehicleModelKind::Bicycle => {
            with_values(BicycleParams::tunable_parameters(), &config.bicycle)
        }
        VehicleModelKind::DynamicBicycle => {
            with_values(DynamicParams::tunable_parameters(), &config.dynamic_bicycle)
        }
        VehicleModelKind::NonlinearBicycle => with_values(
            NonlinearTireParams::tunable_parameters(),
            &config.nonlinear_bicycle,
        ),
        VehicleModelKind::PacejkaBicycle => with_values(
            PacejkaTireParams::tunable_parameters(),
            &config.pacejka_bicycle,
        ),
        VehicleModelKind::TwoTrack => {
            with_values(TwoTrackParams::tunable_parameters(), &config.two_track)
        }
    }
}

/// Applies `wanted` to `kind`'s parameters in `config`, each sanitized (see
/// [`crate::config::apply_parameters`]). Returns whether anything changed.
pub(super) fn apply_wanted(
    kind: VehicleModelKind,
    config: &mut SimulatedVehicleConfig,
    wanted: &BTreeMap<String, f64>,
) -> bool {
    use crate::config::apply_parameters as apply;
    let parameters = tunable_parameters(kind, config);
    match kind {
        VehicleModelKind::Bicycle => apply(&mut config.bicycle, &parameters, wanted),
        VehicleModelKind::DynamicBicycle => apply(&mut config.dynamic_bicycle, &parameters, wanted),
        VehicleModelKind::NonlinearBicycle => {
            apply(&mut config.nonlinear_bicycle, &parameters, wanted)
        }
        VehicleModelKind::PacejkaBicycle => apply(&mut config.pacejka_bicycle, &parameters, wanted),
        VehicleModelKind::TwoTrack => apply(&mut config.two_track, &parameters, wanted),
    }
}

/// What [`SimulatedVehicle`](super::SimulatedVehicle) publishes on [`VEHICLE_MODEL_STATUS_TOPIC_NAME`](crate::topics::VEHICLE_MODEL_STATUS_TOPIC_NAME)
/// while running `kind` with `config`.
pub(super) fn model_status(
    kind: VehicleModelKind,
    config: &SimulatedVehicleConfig,
) -> VehicleModelStatus {
    let mut limits = ActuatorLimits::tunable_parameters_but_steering_angle();
    crate::config::refresh_parameter_values(&mut limits, &config.limits);
    VehicleModelStatus {
        kind,
        parameters: tunable_parameters(kind, config),
        limits,
    }
}

/// Writes `parameters`' values into `kind`'s `[<kind>]` table of
/// [`config_path`], leaving everything else in the file - comments, other
/// tables, layout - untouched. Returns the file's path.
pub fn save_parameters(
    kind: VehicleModelKind,
    parameters: &[AlgorithmParameter],
) -> Result<PathBuf, String> {
    save_table(kind.api_str(), parameters)
}

/// Like [`save_parameters`], for the actuator limits' `[limits]` table.
pub fn save_limits(parameters: &[AlgorithmParameter]) -> Result<PathBuf, String> {
    save_table("limits", parameters)
}

/// The values `kind`'s `[<kind>]` table of [`config_path`] holds for
/// `parameters`, by name, and the file's path - e.g. to go back to what was
/// last saved.
pub fn saved_values(
    kind: VehicleModelKind,
    parameters: &[AlgorithmParameter],
) -> Result<(BTreeMap<String, f64>, PathBuf), String> {
    saved_table_values(kind.api_str(), parameters)
}

/// Like [`saved_values`], for the actuator limits' `[limits]` table.
pub fn saved_limits(
    parameters: &[AlgorithmParameter],
) -> Result<(BTreeMap<String, f64>, PathBuf), String> {
    saved_table_values("limits", parameters)
}

pub(super) fn saved_table_values(
    table: &str,
    parameters: &[AlgorithmParameter],
) -> Result<(BTreeMap<String, f64>, PathBuf), String> {
    let path = config_path();
    let names: Vec<&str> = parameters
        .iter()
        .map(|parameter| parameter.name.as_str())
        .collect();
    Ok((
        crate::config::load_toml_values(&path, Some(table), &names)?,
        path,
    ))
}

pub(super) fn save_table(
    table: &str,
    parameters: &[AlgorithmParameter],
) -> Result<PathBuf, String> {
    let path = config_path();
    let values: Vec<(&str, String)> = parameters
        .iter()
        .map(|parameter| {
            (
                parameter.name.as_str(),
                crate::config::parameter_toml_value(parameter),
            )
        })
        .collect();
    crate::config::save_toml_values(&path, Some(table), &values)?;
    Ok(path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::vehicle::model::tests::*;
    use crate::simulation::vehicle::model::*;

    /// The params' fields that are the car's, set by `for_car`.
    const CAR_OWNED: [&str; 4] = ["lf_m", "lr_m", "mass_kg", "track_width_m"];

    #[test]
    fn default_limits_validate() {
        assert!(
            matches!(test_model(), VehicleModel::Bicycle { limits, .. } if limits.validate().is_ok())
        );
    }

    #[test]
    fn default_model_validates_for_every_kind() {
        let config = SimulatedVehicleConfig::default();
        for kind in [
            VehicleModelKind::Bicycle,
            VehicleModelKind::DynamicBicycle,
            VehicleModelKind::NonlinearBicycle,
            VehicleModelKind::PacejkaBicycle,
            VehicleModelKind::TwoTrack,
        ] {
            match default_model(kind, &config) {
                VehicleModel::Bicycle { params, limits } => {
                    assert!(params.validate().is_ok());
                    assert!(limits.validate().is_ok());
                }
                VehicleModel::DynamicBicycle { params, limits } => {
                    assert!(params.validate().is_ok());
                    assert!(limits.validate().is_ok());
                }
                VehicleModel::NonlinearBicycle { params, limits } => {
                    assert!(params.validate().is_ok());
                    assert!(limits.validate().is_ok());
                }
                VehicleModel::PacejkaBicycle { params, limits } => {
                    assert!(params.validate().is_ok());
                    assert!(limits.validate().is_ok());
                }
                VehicleModel::TwoTrack { params, limits } => {
                    assert!(params.validate().is_ok());
                    assert!(limits.validate().is_ok());
                }
            }
        }
    }

    /// Every kind declares one tunable parameter per field of its params
    /// but the car's own (a mismatched name panics in
    /// `tunable_parameters`), each starting
    /// from the checked-in config's value and within its own range - so
    /// saving untouched values never changes the file.
    #[test]
    fn every_kind_declares_a_parameter_per_field_within_range() {
        let config = SimulatedVehicleConfig::default();
        for (kind, ..) in VehicleModelKind::ALL {
            let parameters = tunable_parameters(*kind, &config);
            let json = match kind {
                VehicleModelKind::Bicycle => serde_json::to_value(config.bicycle),
                VehicleModelKind::DynamicBicycle => serde_json::to_value(config.dynamic_bicycle),
                VehicleModelKind::NonlinearBicycle => {
                    serde_json::to_value(config.nonlinear_bicycle)
                }
                VehicleModelKind::PacejkaBicycle => serde_json::to_value(config.pacejka_bicycle),
                VehicleModelKind::TwoTrack => serde_json::to_value(config.two_track),
            }
            .unwrap();
            let fields = json.as_object().unwrap();
            let own = fields
                .keys()
                .filter(|name| !CAR_OWNED.contains(&name.as_str()));
            assert_eq!(
                parameters.len(),
                own.count(),
                "{kind:?} doesn't declare every field"
            );
            assert!(
                parameters
                    .iter()
                    .all(|parameter| !CAR_OWNED.contains(&parameter.name.as_str())),
                "{kind:?} tunes the car's own geometry"
            );
            for parameter in &parameters {
                assert_eq!(
                    parameter.kind.sanitize(parameter.value),
                    Some(parameter.value),
                    "{kind:?}'s {} default is outside its range",
                    parameter.name
                );
            }
        }
    }

    #[test]
    fn limits_declare_a_parameter_per_field_within_range() {
        let config = SimulatedVehicleConfig::default();
        let limits = model_status(VehicleModelKind::Bicycle, &config).limits;
        let json = serde_json::to_value(config.limits).unwrap();
        // All but the steering angle, which is the car's.
        assert_eq!(limits.len(), json.as_object().unwrap().len() - 1);
        for parameter in &limits {
            assert_eq!(
                parameter.kind.sanitize(parameter.value),
                Some(parameter.value),
                "{}'s default is outside its range",
                parameter.name
            );
        }
    }

    #[test]
    fn wanted_values_apply_to_the_running_kind_only() {
        let mut config = SimulatedVehicleConfig::default();
        let wanted = BTreeMap::from([
            ("yaw_inertia_kgm2".to_string(), 0.3),
            ("cf_n_per_rad".to_string(), 999.0),
            // The car's own: ignored.
            ("lf_m".to_string(), 0.3),
        ]);
        let lf_m = config.dynamic_bicycle.lf_m;
        assert!(apply_wanted(
            VehicleModelKind::DynamicBicycle,
            &mut config,
            &wanted
        ));
        assert_eq!(config.dynamic_bicycle.yaw_inertia_kgm2, 0.3);
        // Clamped to its range.
        assert_eq!(config.dynamic_bicycle.cf_n_per_rad, 300.0);
        assert_eq!(config.dynamic_bicycle.lf_m, lf_m);
        assert_eq!(config.bicycle, SimulatedVehicleConfig::default().bicycle);
        assert!(!apply_wanted(
            VehicleModelKind::DynamicBicycle,
            &mut config,
            &wanted
        ));

        let status = model_status(VehicleModelKind::DynamicBicycle, &config);
        let inertia = status
            .parameters
            .iter()
            .find(|p| p.name == "yaw_inertia_kgm2")
            .unwrap();
        assert_eq!(inertia.value, 0.3);
    }

    #[test]
    fn every_kind_simulates_the_cars_geometry() {
        let mut car = CarCalibration::template("test");
        car.geometry.wheelbase_m = 0.4;
        car.geometry.rear_axle_to_cg_m = 0.15;
        car.geometry.mass_kg = 2.0;
        car.geometry.track_width_m = 0.25;
        car.steering.points[0].angle_rad = -0.3;
        let config = SimulatedVehicleConfig::default().for_car(&car);
        for (kind, ..) in VehicleModelKind::ALL {
            let (lf_m, lr_m) = axles_of(&default_model(*kind, &config));
            assert!((lf_m - 0.25).abs() < 1e-12 && lr_m == 0.15, "{kind:?}");
        }
        assert_eq!(config.dynamic_bicycle.mass_kg, 2.0);
        assert_eq!(config.two_track.mass_kg, 2.0);
        assert_eq!(config.two_track.track_width_m, 0.25);
        // The smaller side.
        assert_eq!(config.limits.max_steering_angle_rad, 0.3);
        assert_eq!(config.geometry, car.vehicle_geometry());
    }
}
