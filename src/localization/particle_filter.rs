//! The particle filter: Monte Carlo localization on a known map, a Rust
//! port of UBM's `particle_filter_cpp` ROS 2 node and of the ray caster it
//! uses, range_libc's CDDT. [`crate::localization::Slam`] runs it as its
//! second localization backend, next to scan matching, picked by
//! [`crate::topics::SlamCommand::localization_algorithm`]. See
//! `documentation/slam.md#particle-filter`.
//!
//! - [`omap`]: the occupancy grid and its distance transform;
//! - [`cddt`]: range_libc's `CDDTCast`, ported faithfully;
//! - [`ray_marching`]: range_libc's `RayMarching`, the exact reference;
//! - [`sensor_model`]: the beam model table;
//! - [`filter`]: the filter itself.

mod cddt;
mod filter;
mod omap;
mod ray_marching;
mod sensor_model;

pub(crate) use filter::{Estimate, ParticleFilter, Start};

use crate::topics::AlgorithmParameter;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// Every parameter of the particle filter - loaded from
/// `config/localization/particle_filter.toml` (see [`Default`]). Names
/// follow UBM's node, with units added. Every numeric field but `seed` is
/// live-tunable, see [`tunable_parameters`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ParticleFilterConfig {
    /// Particles while tracking.
    pub num_particles: usize,
    /// Field of view of the beams used, centered on the sensor's forward
    /// direction, in radians - clamped to the sensor's own.
    pub considered_lidar_fov_rad: f64,
    /// Every this many beams one is used.
    pub range_step: usize,
    /// Readings and ray casts are clamped to this, in meters.
    pub max_range_m: f64,
    /// `0`: CDDT; `1`: ray marching.
    pub ray_cast_algo: u8,
    /// Directions CDDT casts along, over the full circle.
    pub theta_discretization: usize,
    /// Beam model mixture weights: a short reading...
    pub ray_short_weight: f64,
    /// ... a max-range reading...
    pub ray_max_weight: f64,
    /// ... uniform noise...
    pub ray_rand_weight: f64,
    /// ... and a hit near the predicted range.
    pub ray_hit_weight: f64,
    /// Standard deviation of a hit, in map pixels.
    pub ray_hit_sigma_px: f64,
    /// A scan's likelihood is raised to `1 / squash_factor`, so one scan
    /// can't collapse the cloud.
    pub squash_factor: f64,
    /// Spread of a (re)started cloud around its pose.
    pub initial_pose_stddev_x_m: f64,
    pub initial_pose_stddev_y_m: f64,
    pub initial_pose_stddev_heading_rad: f64,
    /// Motion noise added on every scan the vehicle moved, in its frame...
    pub motion_dispersion_stddev_x_m: f64,
    pub motion_dispersion_stddev_y_m: f64,
    pub motion_dispersion_stddev_heading_rad: f64,
    /// ... plus this much per meter driven (on x and y)...
    pub motion_noise_per_m: f64,
    /// ... and this much per radian turned (on the heading).
    pub motion_noise_per_rad: f64,
    /// Resample once the effective sample size drops below this fraction
    /// of the particles - `1` resamples on every scan, as UBM's node does.
    pub resample_threshold: f64,
    /// Particles of a global (re)localization.
    pub global_num_particles: usize,
    /// A global localization has converged, and drops to `num_particles`,
    /// once the cloud's spread is below this, in meters...
    pub global_converged_spread_m: f64,
    /// ... for this many scans in a row.
    pub global_converged_scans: usize,
    /// [`Self::squash_factor`] while a global localization converges -
    /// larger, so one scan can't settle it on a stretch of track that only
    /// looks like the right one.
    pub global_squash_factor: f64,
    /// Particles drawn on the map.
    pub draw_particles: usize,
    /// Threads casting rays - capped at the cores available.
    pub threads: usize,
    /// Seed of the noise - fixed, so a run can be repeated.
    pub seed: u64,
}

impl Default for ParticleFilterConfig {
    fn default() -> Self {
        toml::from_str(include_str!(
            "../../config/localization/particle_filter.toml"
        ))
        .expect(
            "config/localization/particle_filter.toml must deserialize into ParticleFilterConfig",
        )
    }
}

impl ParticleFilterConfig {
    /// This config with `values` - tunable parameters by name, see
    /// [`tunable_parameters`] - applied, each brought into its range.
    pub fn with_parameters(&self, values: &BTreeMap<String, f64>) -> Self {
        let mut tuned = *self;
        crate::config::apply_parameters(&mut tuned, &tunable_parameters(), values);
        tuned
    }
}

/// Every live-tunable parameter, each named after the
/// [`ParticleFilterConfig`] field it sets, with its `value` left at `0` -
/// see [`crate::config::refresh_parameter_values`].
pub fn tunable_parameters() -> Vec<AlgorithmParameter> {
    vec![
        AlgorithmParameter::int("num_particles", 10, 20_000, 10)
            .description("Particles while tracking - more is steadier, and slower."),
        AlgorithmParameter::float("considered_lidar_fov_rad", 0.1, std::f64::consts::TAU, 0.01)
            .unit("rad")
            .description(
                "Field of view of the beams used, centered ahead - clamped to the sensor's.",
            ),
        AlgorithmParameter::int("range_step", 1, 100, 1)
            .description("One beam in this many is used."),
        AlgorithmParameter::float("max_range_m", 1.0, 30.0, 0.5)
            .unit("m")
            .description("Readings and casts are clamped to this - the map model is rebuilt."),
        AlgorithmParameter::int("ray_cast_algo", 0, 1, 1)
            .description("0: CDDT (fast, approximate). 1: ray marching (exact, slower)."),
        AlgorithmParameter::int("theta_discretization", 8, 720, 2).description(
            "Directions CDDT casts along, over the full circle - the map model is rebuilt.",
        ),
        AlgorithmParameter::float("ray_short_weight", 0.0, 1.0, 0.01)
            .description("Beam model: weight of a short reading, off something not on the map."),
        AlgorithmParameter::float("ray_max_weight", 0.0, 1.0, 0.01)
            .description("Beam model: weight of a max-range reading."),
        AlgorithmParameter::float("ray_rand_weight", 0.0, 1.0, 0.01)
            .description("Beam model: weight of uniform noise."),
        AlgorithmParameter::float("ray_hit_weight", 0.0, 1.0, 0.01)
            .description("Beam model: weight of a hit near the predicted range."),
        AlgorithmParameter::float("ray_hit_sigma_px", 0.5, 50.0, 0.5)
            .unit("px")
            .description("Standard deviation of a hit, in map pixels."),
        AlgorithmParameter::float("squash_factor", 1.0, 20.0, 0.1).description(
            "A scan's likelihood is raised to 1 / this - larger keeps the cloud from \
             collapsing onto one scan.",
        ),
        AlgorithmParameter::float("initial_pose_stddev_x_m", 0.0, 3.0, 0.01)
            .unit("m")
            .description("Spread of a (re)started cloud, along x..."),
        AlgorithmParameter::float("initial_pose_stddev_y_m", 0.0, 3.0, 0.01)
            .unit("m")
            .description("... along y..."),
        AlgorithmParameter::float(
            "initial_pose_stddev_heading_rad",
            0.0,
            std::f64::consts::PI,
            0.01,
        )
        .unit("rad")
        .description("... and in heading."),
        AlgorithmParameter::float("motion_dispersion_stddev_x_m", 0.0, 0.5, 0.001)
            .unit("m")
            .description("Motion noise on every scan the vehicle moved: forward..."),
        AlgorithmParameter::float("motion_dispersion_stddev_y_m", 0.0, 0.5, 0.001)
            .unit("m")
            .description("... sideways..."),
        AlgorithmParameter::float("motion_dispersion_stddev_heading_rad", 0.0, 0.5, 0.001)
            .unit("rad")
            .description("... and in heading."),
        AlgorithmParameter::float("motion_noise_per_m", 0.0, 2.0, 0.01)
            .description("Extra position noise per meter driven."),
        AlgorithmParameter::float("motion_noise_per_rad", 0.0, 2.0, 0.01)
            .description("Extra heading noise per radian turned."),
        AlgorithmParameter::float("resample_threshold", 0.0, 1.0, 0.01).description(
            "Resample once the effective sample size drops below this fraction of the \
             particles - 1 resamples on every scan.",
        ),
        AlgorithmParameter::int("global_num_particles", 100, 200_000, 100)
            .description("Particles of a global localization (Relocalize)."),
        AlgorithmParameter::float("global_converged_spread_m", 0.01, 5.0, 0.01)
            .unit("m")
            .description(
                "A global localization has converged once the cloud is tighter than this...",
            ),
        AlgorithmParameter::int("global_converged_scans", 1, 1000, 1)
            .description("... for this many scans in a row."),
        AlgorithmParameter::float("global_squash_factor", 1.0, 50.0, 0.1).description(
            "The squash factor while a global localization converges - larger, so one scan \
             can't settle it on a stretch of track that only looks like the right one.",
        ),
        AlgorithmParameter::int("draw_particles", 0, 5000, 10)
            .description("Particles drawn on the map."),
        AlgorithmParameter::int("threads", 1, 64, 1)
            .description("Threads casting rays - capped at the cores available."),
    ]
}

/// Where the particle filter keeps its config:
/// `config/localization/particle_filter.toml`, relative to the working
/// directory - rewritten by [`save_parameters`].
pub fn config_path() -> PathBuf {
    Path::new(crate::config::DEFAULT_CONFIG_ROOT)
        .join("localization")
        .join("particle_filter.toml")
}

/// Writes `parameters`' values into [`config_path`], keeping the rest of
/// the file. Returns the file's path.
pub fn save_parameters(parameters: &[AlgorithmParameter]) -> Result<PathBuf, String> {
    let path = config_path();
    crate::config::save_parameter_values(&path, parameters)?;
    Ok(path)
}

/// The values [`config_path`] holds for `parameters`, by name, and the
/// file's path.
pub fn saved_values(
    parameters: &[AlgorithmParameter],
) -> Result<(BTreeMap<String, f64>, PathBuf), String> {
    let path = config_path();
    Ok((
        crate::config::saved_parameter_values(&path, parameters)?,
        path,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_checked_in_config_deserializes() {
        let config = ParticleFilterConfig::default();
        assert!(config.num_particles > 0);
        let on_disk: ParticleFilterConfig =
            crate::config::load(Path::new("config/localization/particle_filter.toml")).unwrap();
        assert_eq!(on_disk, config);
    }

    #[test]
    fn every_parameter_names_a_config_field_within_its_range() {
        let mut parameters = tunable_parameters();
        crate::config::refresh_parameter_values(&mut parameters, &ParticleFilterConfig::default());
        for parameter in &parameters {
            assert_eq!(
                parameter.kind.sanitize(parameter.value),
                Some(parameter.value),
                "{} = {} is out of its range",
                parameter.name,
                parameter.value
            );
        }
    }

    #[test]
    fn a_parameter_is_applied_and_clamped() {
        let values = BTreeMap::from([
            ("num_particles".to_string(), 500.0),
            ("squash_factor".to_string(), 1e9),
        ]);
        let tuned = ParticleFilterConfig::default().with_parameters(&values);
        assert_eq!(tuned.num_particles, 500);
        assert_eq!(tuned.squash_factor, 20.0);
    }
}
