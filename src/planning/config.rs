//! [`PlanningConfig`]: every tunable of [`super::Planner`], read from
//! `config/planning/race_line.toml` and tunable live from `web_gui`'s
//! Planning panel (see [`tunable_parameters`]), which can also save the
//! values back ([`save_parameters`]) or reload them ([`saved_values`]).

use super::min_curvature::MinCurvatureConfig;
use super::min_time::MinTimeConfig;
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
    /// Extra distance kept from either wall on top of half the vehicle's
    /// width (its [`crate::topics::VehicleGeometry`]'s), in meters.
    pub safety_margin_m: f64,
    /// Share of the vehicle's steering lock the race line may use - see
    /// [`Self::curvature_limit_per_m`].
    pub max_steering_fraction: f64,
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
    /// Largest braking deceleration, in m/s^2 - the friction ellipse's
    /// longitudinal semi-axis, so the grip limit on accelerating too.
    pub max_decel_mps2: f64,
    /// Spacing of the minimum-time optimization's points, in meters.
    pub min_time_spacing_m: f64,
    /// Spacing of the B-spline control values the minimum-time line's
    /// offsets follow, in meters.
    pub min_time_control_spacing_m: f64,
    /// Lowest speed the minimum-time line may slow down to, in m/s.
    pub min_speed_mps: f64,
    /// Most outer (augmented Lagrangian) iterations of the minimum-time
    /// optimization.
    pub min_time_max_outer_iterations: usize,
    /// Time budget of the minimum-time optimization, in seconds.
    pub min_time_max_duration_s: f64,
    /// The minimum-time line has converged once no limit is exceeded by
    /// more than this fraction of it.
    pub min_time_tolerance: f64,
}

/// What planning needs to know about the vehicle the line is for.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanningVehicle {
    /// Its body's width, in meters - see [`PlanningConfig::wall_margin_m`].
    pub body_width_m: f64,
    /// Between its axles, in meters.
    pub wheelbase_m: f64,
    /// Its steering lock, in radians: the largest front-wheel angle, either
    /// way (the smaller of its two sides).
    pub max_steering_angle_rad: f64,
}

impl Default for PlanningVehicle {
    /// The template car's (`config/car_template.toml`).
    fn default() -> Self {
        let car = crate::hardware::CarCalibration::template("template");
        let geometry = car.vehicle_geometry();
        Self {
            body_width_m: geometry.body_width_m,
            wheelbase_m: geometry.wheelbase_m,
            max_steering_angle_rad: car.steering.max_angle_rad(),
        }
    }
}

impl Default for PlanningConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/planning/race_line.toml"))
            .expect("config/planning/race_line.toml must deserialize into PlanningConfig")
    }
}

impl PlanningConfig {
    /// How far the race line keeps from either wall for a vehicle
    /// `body_width_m` wide: half of it, plus `safety_margin_m`.
    pub fn wall_margin_m(&self, body_width_m: f64) -> f64 {
        body_width_m / 2.0 + self.safety_margin_m
    }

    /// Tightest the race line may turn for `vehicle`, in 1/m: a kinematic
    /// bicycle's curvature, `tan(steering) / wheelbase`, at
    /// `max_steering_fraction` of its steering lock.
    pub fn curvature_limit_per_m(&self, vehicle: &PlanningVehicle) -> f64 {
        let steering_rad = self.max_steering_fraction * vehicle.max_steering_angle_rad;
        steering_rad.tan() / vehicle.wheelbase_m.max(1e-3)
    }

    /// The optimizer's share of this config, for `vehicle`.
    pub fn min_curvature(&self, vehicle: &PlanningVehicle) -> MinCurvatureConfig {
        MinCurvatureConfig {
            margin_m: self.wall_margin_m(vehicle.body_width_m),
            max_curvature_per_m: self.curvature_limit_per_m(vehicle),
            spacing_m: self.spacing_m,
            smoothness_weight: self.smoothness_weight,
            max_step_m: self.max_step_m,
            iterations: self.iterations,
            tolerance_m: self.tolerance_m,
            solver_tolerance: self.solver_tolerance,
            solver_max_iterations: self.solver_max_iterations,
        }
    }

    /// The minimum-time optimizer's share of this config, for `vehicle`.
    pub fn min_time(&self, vehicle: &PlanningVehicle) -> MinTimeConfig {
        MinTimeConfig {
            margin_m: self.wall_margin_m(vehicle.body_width_m),
            max_curvature_per_m: self.curvature_limit_per_m(vehicle),
            spacing_m: self.min_time_spacing_m,
            control_spacing_m: self.min_time_control_spacing_m,
            min_speed_mps: self.min_speed_mps,
            limits: self.speed_limits(),
            max_outer_iterations: self.min_time_max_outer_iterations,
            max_inner_iterations: self.solver_max_iterations,
            max_duration: std::time::Duration::from_secs_f64(self.min_time_max_duration_s),
            tolerance: self.min_time_tolerance,
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
        AlgorithmParameter::float("safety_margin_m", 0.0, 1.0, 0.005)
            .unit("m")
            .description("Extra distance kept from either wall, on top of half the vehicle's width."),
        AlgorithmParameter::float("max_steering_fraction", 0.1, 1.0, 0.01).description(
            "Share of the vehicle's steering lock the race line may use - it never turns tighter.",
        ),
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
            .description("Largest cornering acceleration - the friction ellipse's lateral axis."),
        AlgorithmParameter::float("max_accel_mps2", 0.5, 20.0, 0.1)
            .unit("m/s²")
            .description("Largest forward acceleration the motor gives."),
        AlgorithmParameter::float("max_decel_mps2", 0.5, 20.0, 0.1)
            .unit("m/s²")
            .description("Largest braking deceleration - the friction ellipse's longitudinal axis."),
        AlgorithmParameter::float("min_time_spacing_m", 0.05, 1.0, 0.01)
            .unit("m")
            .description("Minimum time: distance between the optimized points."),
        AlgorithmParameter::float("min_time_control_spacing_m", 0.2, 5.0, 0.1)
            .unit("m")
            .description("Minimum time: the line bends through control points this far apart - smaller follows the track closer, but is slower to solve."),
        AlgorithmParameter::float("min_speed_mps", 0.1, 5.0, 0.1)
            .unit("m/s")
            .description("Minimum time: lowest speed allowed anywhere."),
        AlgorithmParameter::int("min_time_max_outer_iterations", 1, 200, 1)
            .description("Minimum time: most augmented Lagrangian iterations."),
        AlgorithmParameter::float("min_time_max_duration_s", 5.0, 600.0, 5.0)
            .unit("s")
            .description("Minimum time: time budget of the optimization."),
        AlgorithmParameter::float("min_time_tolerance", 0.001, 0.1, 0.001)
            .description("Minimum time: converged once no limit is exceeded by more than this fraction."),
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
