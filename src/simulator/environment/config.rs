//! [`GenerationConfig`]: the tunable parameters for one map generation run.

use std::path::PathBuf;

/// Default output root for generated maps, relative to the current working
/// directory - a gitignored folder at the repo root.
const DEFAULT_OUTPUT_ROOT: &str = "maps";

/// Every tunable parameter for [`crate::simulator::environment::generate`].
/// Construct via [`Default`] and override only the fields that matter, e.g.
/// `GenerationConfig { seed: 42, ..Default::default() }`.
#[derive(Debug, Clone)]
pub struct GenerationConfig {
    /// Width of the bounded area Voronoi sites are scattered in, in meters.
    pub area_width_m: f64,
    /// Height of the bounded area Voronoi sites are scattered in, in meters.
    pub area_height_m: f64,
    /// Size of one raster pixel, in meters.
    pub resolution_m_per_px: f64,
    /// Number of random Voronoi seed points to scatter.
    pub num_sites: usize,
    /// Minimum allowed distance between any two seed points, in meters.
    pub min_site_spacing_m: f64,
    /// Fraction (0.0-1.0, exclusive) of the diagram's total interior area
    /// the BFS region growth aims to cover before stopping.
    pub target_area_fraction: f64,
    /// Width of the drivable track band, in meters.
    pub track_width_m: f64,
    /// Target arc-length spacing between consecutive race-line points, in
    /// meters.
    pub point_spacing_m: f64,
    /// How many points to interpolate per raw loop segment before
    /// resampling to even spacing - higher gives a smoother curve.
    pub smoothing_samples_per_segment: usize,
    /// Speed cap on straights, in meters/second.
    pub max_speed_mps: f64,
    /// Maximum lateral acceleration used to derive cornering speed, in
    /// meters/second^2.
    pub max_lateral_accel_mps2: f64,
    /// Seed for the deterministic RNG driving the whole generation - the
    /// same seed and config reproduce the same map exactly.
    pub seed: u64,
    /// Folder every generated map's own folder is created under.
    pub output_root: PathBuf,
}

impl Default for GenerationConfig {
    fn default() -> Self {
        Self {
            area_width_m: 60.0,
            area_height_m: 60.0,
            resolution_m_per_px: 0.05,
            num_sites: 60,
            min_site_spacing_m: 3.0,
            target_area_fraction: 0.35,
            track_width_m: 2.5,
            point_spacing_m: 0.25,
            smoothing_samples_per_segment: 20,
            max_speed_mps: 8.0,
            max_lateral_accel_mps2: 6.0,
            seed: 0,
            output_root: PathBuf::from(DEFAULT_OUTPUT_ROOT),
        }
    }
}

impl GenerationConfig {
    /// Basic sanity checks on the parameter values, independent of any
    /// particular random draw - catches misconfiguration (e.g. a zero or
    /// negative dimension) before spending time on generation.
    pub fn validate(&self) -> Result<(), String> {
        if self.area_width_m <= 0.0 || self.area_height_m <= 0.0 {
            return Err("area_width_m and area_height_m must be positive".to_string());
        }
        if self.resolution_m_per_px <= 0.0 {
            return Err("resolution_m_per_px must be positive".to_string());
        }
        if self.num_sites < 8 {
            return Err("num_sites must be at least 8 to plausibly form a loop".to_string());
        }
        if self.min_site_spacing_m <= 0.0 {
            return Err("min_site_spacing_m must be positive".to_string());
        }
        if self.target_area_fraction <= 0.0 || self.target_area_fraction >= 1.0 {
            return Err("target_area_fraction must be in (0.0, 1.0)".to_string());
        }
        if self.track_width_m <= 0.0 {
            return Err("track_width_m must be positive".to_string());
        }
        if self.point_spacing_m <= 0.0 {
            return Err("point_spacing_m must be positive".to_string());
        }
        if self.smoothing_samples_per_segment == 0 {
            return Err("smoothing_samples_per_segment must be at least 1".to_string());
        }
        if self.max_speed_mps <= 0.0 {
            return Err("max_speed_mps must be positive".to_string());
        }
        if self.max_lateral_accel_mps2 <= 0.0 {
            return Err("max_lateral_accel_mps2 must be positive".to_string());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_validates() {
        assert!(GenerationConfig::default().validate().is_ok());
    }

    #[test]
    fn invalid_track_width_is_rejected() {
        let config = GenerationConfig {
            track_width_m: 0.0,
            ..Default::default()
        };
        assert!(config.validate().is_err());
    }

    #[test]
    fn invalid_target_area_fraction_is_rejected() {
        let config = GenerationConfig {
            target_area_fraction: 1.5,
            ..Default::default()
        };
        assert!(config.validate().is_err());
    }
}
