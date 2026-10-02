//! [`UbmDetectorConfig`]: every tunable of [`super::UbmDetector`], read from
//! `config/perception/ubm_detector.toml` and tunable live from `web_gui`'s
//! Detector panel (see [`tunable_parameters`]), which can also save the
//! values back ([`save_parameters`]) or reload them ([`saved_values`]).

use crate::topics::AlgorithmParameter;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// Every tunable parameter of [`super::UbmDetector`] - loaded from
/// `config/perception/ubm_detector.toml` (see [`Default`]) or from an
/// arbitrary path via [`crate::config::load`]. The TOML file documents each
/// one. Every field but `poll_interval_ms` is live-tunable, see
/// [`tunable_parameters`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct UbmDetectorConfig {
    pub poll_interval_ms: u64,
    /// [`POSE_LOCALIZATION`](crate::localization::pose_source::POSE_LOCALIZATION)
    /// or [`POSE_GROUND_TRUTH`](crate::localization::pose_source::POSE_GROUND_TRUTH).
    pub pose_source: u8,
    pub max_detection_range_m: f64,
    pub median_filter_kernel_size: usize,
    pub gradient_threshold: f64,
    pub min_object_width_m: f64,
    pub object_std_threshold_m: f64,
    pub distance_from_walls_threshold_m: f64,
    pub robot_radius_m: f64,
    /// `0` or `1`.
    pub ignore_walls: u8,
    pub ignore_walls_radius_px: usize,
    pub kf_process_noise: f64,
    pub kf_measurement_noise: f64,
    pub prediction_count: usize,
    pub prediction_dt_s: f64,
    pub min_2_points_dist_m: f64,
    /// Which object wins when there are several: `0` = the best score,
    /// `1` = the closest.
    pub selection: u8,
}

impl Default for UbmDetectorConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/perception/ubm_detector.toml"))
            .expect("config/perception/ubm_detector.toml must deserialize into UbmDetectorConfig")
    }
}

/// Every live-tunable parameter, each named after the [`UbmDetectorConfig`]
/// field it sets, with its `value` left at `0` - see
/// [`crate::config::refresh_parameter_values`].
pub fn tunable_parameters() -> Vec<AlgorithmParameter> {
    vec![
        AlgorithmParameter::int("pose_source", 0, 1, 1)
            .description("0 = localization (SLAM must be localizing), 1 = ground truth (simulation only)."),
        AlgorithmParameter::float("max_detection_range_m", 1.0, 30.0, 0.5)
            .unit("m")
            .description("Ranges beyond this are clamped to it, in both scans."),
        AlgorithmParameter::int("median_filter_kernel_size", 1, 15, 2)
            .description("Rays the median filter smoothing the scans' difference spans - 1 for none."),
        AlgorithmParameter::float("gradient_threshold", 0.0, 5.0, 0.05)
            .unit("m")
            .description("A jump in the difference larger than this starts or ends a stretch - 0 picks it from the scan."),
        AlgorithmParameter::float("min_object_width_m", 0.0, 1.0, 0.01)
            .unit("m")
            .description("Narrowest an object may be, across the rays that see it."),
        AlgorithmParameter::float("object_std_threshold_m", 0.01, 5.0, 0.01)
            .unit("m")
            .description("Most an object's ranges may spread (standard deviation)."),
        AlgorithmParameter::float("distance_from_walls_threshold_m", 0.0, 2.0, 0.01)
            .unit("m")
            .description("Least the real scan must be shorter than the map's, on average, across an object."),
        AlgorithmParameter::float("robot_radius_m", 0.0, 1.0, 0.01)
            .unit("m")
            .description("How far behind the surface seen the opponent's center is."),
        AlgorithmParameter::int("ignore_walls", 0, 1, 1)
            .description("1 drops a detection close to a wall, 0 keeps it."),
        AlgorithmParameter::int("ignore_walls_radius_px", 0, 20, 1)
            .unit("px")
            .description("How close to a wall a detection may be before it's dropped."),
        AlgorithmParameter::float("kf_process_noise", 0.01, 20.0, 0.01)
            .unit("m/s²")
            .description("Kalman filter: how much the opponent accelerates - higher follows it faster, but noisier."),
        AlgorithmParameter::float("kf_measurement_noise", 0.01, 5.0, 0.01)
            .unit("m")
            .description("Kalman filter: how far off a detection is - higher is smoother, but lags."),
        AlgorithmParameter::int("prediction_count", 0, 50, 1)
            .description("How many predicted positions to publish."),
        AlgorithmParameter::float("prediction_dt_s", 0.01, 1.0, 0.01)
            .unit("s")
            .description("Time between two predicted positions."),
        AlgorithmParameter::float("min_2_points_dist_m", 0.001, 0.2, 0.001)
            .unit("m")
            .description("Bounding box fit: points closer than this to a side count as this close."),
        AlgorithmParameter::int("selection", 0, 1, 1)
            .description("Which object wins when there are several: 0 = the best score, 1 = the closest."),
    ]
}

/// Where the detector keeps its config: `config/perception/ubm_detector.toml`,
/// relative to the working directory - rewritten by [`save_parameters`].
pub fn config_path() -> PathBuf {
    Path::new(crate::config::DEFAULT_CONFIG_ROOT)
        .join("perception")
        .join("ubm_detector.toml")
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
        crate::config::refresh_parameter_values(&mut parameters, &UbmDetectorConfig::default());
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
