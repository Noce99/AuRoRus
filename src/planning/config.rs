//! [`PlanningConfig`]: every tunable of [`super::Planner`], read from
//! `config/planning/race_line.toml` and tunable live from `web_gui`'s
//! Planning panel (see [`tunable_parameters`]), which can also save the
//! values back ([`save_parameters`]) or reload them ([`saved_values`]).

use super::min_curvature::MinCurvatureConfig;
use super::speed_profile::SpeedLimits;
use crate::topics::AlgorithmParameter;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// Every tunable parameter of [`super::Planner`] - loaded from
/// `config/planning/race_line.toml` (see [`Default`]) or from an arbitrary
/// path via [`crate::config::load`]. Every field but `poll_interval_ms` and
/// `solver_tolerance` is live-tunable, see [`tunable_parameters`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PlanningConfig {
    /// How often the planner checks for a new request or parameter change,
    /// in milliseconds.
    pub poll_interval_ms: u64,
    /// Spacing of the race line's (and a computed centerline's) points, in
    /// meters.
    pub spacing_m: f64,
    /// Moving-average window, in points, smoothing a centerline computed
    /// from the walls - `1` for none. Unused when the map has a centerline.
    pub centerline_smoothing_window: usize,
    /// The vehicle's width, in meters.
    pub vehicle_width_m: f64,
    /// Extra distance kept from either wall on top of half the vehicle's
    /// width, in meters.
    pub safety_margin_m: f64,
    /// Weight of the penalty on neighboring points' sideways offsets
    /// differing, in 1/m^4 - `0` for pure minimum curvature.
    pub smoothness_weight: f64,
    /// Farthest any point may move in one optimization iteration, in
    /// meters.
    pub max_step_m: f64,
    /// Most times the optimization is solved again around its latest
    /// solution.
    pub iterations: usize,
    /// The race line has converged once no point moved further than this
    /// in an iteration, in meters.
    pub tolerance_m: f64,
    /// PANOC's tolerance on its fixed-point residual.
    pub solver_tolerance: f64,
    /// Most PANOC iterations per solve.
    pub solver_max_iterations: usize,
    /// Top speed of the speed profile, in m/s.
    pub max_speed_mps: f64,
    /// Largest lateral acceleration of the speed profile, in m/s^2.
    pub max_lateral_accel_mps2: f64,
    /// Largest forward acceleration of the speed profile, in m/s^2.
    pub max_accel_mps2: f64,
    /// Largest braking deceleration of the speed profile, in m/s^2.
    pub max_decel_mps2: f64,
}

impl Default for PlanningConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/planning/race_line.toml"))
            .expect("config/planning/race_line.toml must deserialize into PlanningConfig")
    }
}

impl PlanningConfig {
    /// The optimizer's share of this config.
    pub fn min_curvature(&self) -> MinCurvatureConfig {
        MinCurvatureConfig {
            margin_m: self.vehicle_width_m / 2.0 + self.safety_margin_m,
            spacing_m: self.spacing_m,
            smoothness_weight: self.smoothness_weight,
            max_step_m: self.max_step_m,
            iterations: self.iterations,
            tolerance_m: self.tolerance_m,
            solver_tolerance: self.solver_tolerance,
            solver_max_iterations: self.solver_max_iterations,
        }
    }

    /// The speed profile's share of this config.
    pub fn speed_limits(&self) -> SpeedLimits {
        SpeedLimits {
            max_speed_mps: self.max_speed_mps,
            max_lateral_accel_mps2: self.max_lateral_accel_mps2,
            max_accel_mps2: self.max_accel_mps2,
            max_decel_mps2: self.max_decel_mps2,
        }
    }
}

/// Every live-tunable parameter, each named after the [`PlanningConfig`]
/// field it sets, with its `value` left at `0` - see
/// [`crate::config::refresh_parameter_values`].
pub fn tunable_parameters() -> Vec<AlgorithmParameter> {
    vec![
        AlgorithmParameter::float("spacing_m", 0.02, 0.5, 0.01)
            .unit("m")
            .description("Distance between the race line's points."),
        AlgorithmParameter::int("centerline_smoothing_window", 1, 51, 2).description(
            "Points averaged to smooth a centerline computed from the walls (maps without one).",
        ),
        AlgorithmParameter::float("vehicle_width_m", 0.1, 1.0, 0.01)
            .unit("m")
            .description("The vehicle's width: the line keeps half of it from either wall."),
        AlgorithmParameter::float("safety_margin_m", 0.0, 1.0, 0.01)
            .unit("m")
            .description("Extra distance kept from either wall."),
        AlgorithmParameter::float("smoothness_weight", 0.0, 10.0, 0.01).description(
            "Penalty on neighboring points' offsets differing - 0 for pure minimum curvature.",
        ),
        AlgorithmParameter::float("max_step_m", 0.01, 2.0, 0.01)
            .unit("m")
            .description(
                "Farthest any point may move in one iteration - smaller is slower but safer.",
            ),
        AlgorithmParameter::int("iterations", 1, 100, 1)
            .description("Most times the optimization is solved again around its latest solution."),
        AlgorithmParameter::float("tolerance_m", 0.001, 0.1, 0.001)
            .unit("m")
            .description("Stops once no point moves further than this in an iteration."),
        AlgorithmParameter::int("solver_max_iterations", 1000, 1_000_000, 1000)
            .description("Most PANOC iterations per solve."),
        AlgorithmParameter::float("max_speed_mps", 0.5, 20.0, 0.1)
            .unit("m/s")
            .description("Top speed of the speed profile."),
        AlgorithmParameter::float("max_lateral_accel_mps2", 0.5, 20.0, 0.1)
            .unit("m/s²")
            .description("Largest cornering acceleration."),
        AlgorithmParameter::float("max_accel_mps2", 0.5, 20.0, 0.1)
            .unit("m/s²")
            .description("Largest forward acceleration."),
        AlgorithmParameter::float("max_decel_mps2", 0.5, 20.0, 0.1)
            .unit("m/s²")
            .description("Largest braking deceleration."),
    ]
}

/// Where the planner keeps its config: `config/planning/race_line.toml`,
/// relative to the working directory - rewritten by [`save_parameters`].
pub fn config_path() -> PathBuf {
    Path::new(crate::config::DEFAULT_CONFIG_ROOT)
        .join("planning")
        .join("race_line.toml")
}

/// Writes `parameters`' values into [`config_path`], leaving everything
/// else in it - comments, other keys, layout - untouched. Returns the
/// file's path.
pub fn save_parameters(parameters: &[AlgorithmParameter]) -> Result<PathBuf, String> {
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
    crate::config::save_toml_values(&path, None, &values)?;
    Ok(path)
}

/// The values [`config_path`] holds for `parameters`, by name, and the
/// file's path - e.g. to go back to what was last saved.
pub fn saved_values(
    parameters: &[AlgorithmParameter],
) -> Result<(BTreeMap<String, f64>, PathBuf), String> {
    let path = config_path();
    let names: Vec<&str> = parameters
        .iter()
        .map(|parameter| parameter.name.as_str())
        .collect();
    Ok((crate::config::load_toml_values(&path, None, &names)?, path))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every declared parameter names a numeric config field - this panics
    /// otherwise - and the shipped defaults sit inside their slider ranges.
    #[test]
    fn every_parameter_names_a_config_field_within_its_range() {
        let mut parameters = tunable_parameters();
        crate::config::refresh_parameter_values(&mut parameters, &PlanningConfig::default());
        for parameter in parameters {
            assert_eq!(
                parameter.kind.sanitize(parameter.value),
                Some(parameter.value),
                "{} = {} is outside its range",
                parameter.name,
                parameter.value
            );
        }
    }
}
