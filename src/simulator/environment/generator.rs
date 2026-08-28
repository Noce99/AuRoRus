//! Orchestrates one full map generation run: sites -> loop -> smoothed race
//! line -> speeds -> raster -> writes `map.tiff`,
//! `race_lines/centerline.csv`, and `info.json` into a new folder under
//! [`GenerationConfig::output_root`].

use crate::simulator::environment::config::GenerationConfig;
use crate::simulator::environment::dynamics::{self, SpeedPoint};
use crate::simulator::environment::info::{self, ImageOrigin, MapInfo, MapSource, StartFinishLine, WorldPoint};
use crate::simulator::environment::points::{self, PointSamplingError};
use crate::simulator::environment::race_line;
use crate::simulator::environment::raster::{self, ImageTransform};
use crate::simulator::environment::smoothing;
use crate::simulator::environment::start_finish;
use crate::simulator::environment::tiff;
use crate::simulator::environment::voronoi_loop::{self, LoopConstructionError};
use rand::SeedableRng;
use rand::rngs::StdRng;
use std::path::PathBuf;

/// Name of the raster file inside a generated map's folder.
pub const MAP_TIFF_FILE_NAME: &str = "map.tiff";
/// Name of the race lines folder inside a generated map's folder.
pub const RACE_LINES_DIR_NAME: &str = "race_lines";
/// Name of the (only, for now) race line file - the folder is designed so
/// more can be added later (e.g. an optimized racing line alongside the
/// centerline) without a format change.
pub const CENTERLINE_FILE_NAME: &str = "centerline.csv";
/// Name of the metadata file inside a generated map's folder.
pub const INFO_FILE_NAME: &str = "info.json";

/// Error returned by [`generate`].
#[derive(Debug)]
pub enum MapGenerationError {
    InvalidConfig(String),
    PointSampling(PointSamplingError),
    LoopConstruction(LoopConstructionError),
    Tiff(tiff::TiffWriteError),
    RaceLine(race_line::RaceLineWriteError),
    Info(info::InfoWriteError),
    Io(std::io::Error),
}

impl std::fmt::Display for MapGenerationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidConfig(msg) => write!(f, "invalid map generation config: {msg}"),
            Self::PointSampling(err) => write!(f, "{err}"),
            Self::LoopConstruction(err) => write!(f, "{err}"),
            Self::Tiff(err) => write!(f, "{err}"),
            Self::RaceLine(err) => write!(f, "{err}"),
            Self::Info(err) => write!(f, "{err}"),
            Self::Io(err) => write!(f, "{err}"),
        }
    }
}

impl std::error::Error for MapGenerationError {}

impl From<std::io::Error> for MapGenerationError {
    fn from(err: std::io::Error) -> Self {
        Self::Io(err)
    }
}

/// The result of a successful [`generate`] call.
#[derive(Debug)]
pub struct GeneratedMap {
    pub folder: PathBuf,
    pub num_race_line_points: usize,
    pub width_px: u32,
    pub height_px: u32,
}

/// Runs one full map generation and writes it to a new folder under
/// `config.output_root`, named `folder_name` if given, or a
/// timestamp+seed-derived name otherwise (see [`default_folder_name`]).
pub fn generate(config: &GenerationConfig, folder_name: Option<&str>) -> Result<GeneratedMap, MapGenerationError> {
    config.validate().map_err(MapGenerationError::InvalidConfig)?;

    let mut rng = StdRng::seed_from_u64(config.seed);

    let sites = points::sample(
        &mut rng,
        config.num_sites,
        config.area_width_m,
        config.area_height_m,
        config.min_site_spacing_m,
    )
    .map_err(MapGenerationError::PointSampling)?;

    let loop_points = voronoi_loop::build_loop(
        sites,
        config.area_width_m,
        config.area_height_m,
        config.target_area_fraction,
    )
    .map_err(MapGenerationError::LoopConstruction)?;

    let dense = smoothing::densify(&loop_points, config.smoothing_samples_per_segment);
    let closed_points = smoothing::resample_even_spacing(&dense, config.point_spacing_m);

    let speeds: Vec<SpeedPoint> =
        dynamics::assign_speeds(&closed_points, config.max_speed_mps, config.max_lateral_accel_mps2);
    let start_finish_segment = start_finish::compute(&closed_points, config.track_width_m);

    let width_px = (config.area_width_m / config.resolution_m_per_px).ceil() as u32;
    let height_px = (config.area_height_m / config.resolution_m_per_px).ceil() as u32;
    let transform = ImageTransform {
        resolution_m_per_px: config.resolution_m_per_px,
        width_px,
        height_px,
        origin_x_m: -config.area_width_m / 2.0,
        origin_y_m: -config.area_height_m / 2.0,
    };
    let map_raster = raster::rasterize(&closed_points, config.track_width_m, &transform);

    let folder_name = folder_name
        .map(String::from)
        .unwrap_or_else(|| default_folder_name(config.seed));
    let folder = config.output_root.join(folder_name);
    let race_lines_dir = folder.join(RACE_LINES_DIR_NAME);
    std::fs::create_dir_all(&race_lines_dir)?;

    tiff::write(&map_raster, &folder.join(MAP_TIFF_FILE_NAME)).map_err(MapGenerationError::Tiff)?;
    race_line::write(&speeds, &race_lines_dir.join(CENTERLINE_FILE_NAME)).map_err(MapGenerationError::RaceLine)?;

    let map_info = MapInfo {
        resolution_m_per_px: config.resolution_m_per_px,
        width_px,
        height_px,
        origin: ImageOrigin { x: transform.origin_x_m, y: transform.origin_y_m, theta_rad: 0.0 },
        start_finish_line: StartFinishLine {
            a: WorldPoint { x: start_finish_segment.a.x, y: start_finish_segment.a.y },
            b: WorldPoint { x: start_finish_segment.b.x, y: start_finish_segment.b.y },
        },
        generated_at: info::now_rfc3339(),
        source: MapSource::Random,
        track_width_m: config.track_width_m,
        point_spacing_m: config.point_spacing_m,
        seed: config.seed,
    };
    info::write(&map_info, &folder.join(INFO_FILE_NAME)).map_err(MapGenerationError::Info)?;

    Ok(GeneratedMap { folder, num_race_line_points: speeds.len(), width_px, height_px })
}

/// `<compact-UTC-timestamp>_seed<seed>`, e.g. `20260828T153000Z_seed42` -
/// sortable, and the seed is visible without opening `info.json`.
fn default_folder_name(seed: u64) -> String {
    let now = time::OffsetDateTime::now_utc();
    format!(
        "{:04}{:02}{:02}T{:02}{:02}{:02}Z_seed{seed}",
        now.year(),
        u8::from(now.month()),
        now.day(),
        now.hour(),
        now.minute(),
        now.second(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch_config(suffix: &str) -> GenerationConfig {
        let output_root =
            std::env::temp_dir().join(format!("aurorus_generator_test_{suffix}_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&output_root);
        GenerationConfig {
            area_width_m: 20.0,
            area_height_m: 20.0,
            resolution_m_per_px: 0.2,
            num_sites: 24,
            min_site_spacing_m: 2.0,
            smoothing_samples_per_segment: 6,
            point_spacing_m: 0.5,
            seed: 7,
            output_root,
            ..Default::default()
        }
    }

    #[test]
    fn generate_writes_all_three_outputs() {
        let config = scratch_config("outputs");
        let result = generate(&config, Some("run")).expect("generation should succeed with these params");

        assert!(result.folder.join(MAP_TIFF_FILE_NAME).exists());
        assert!(result.folder.join(RACE_LINES_DIR_NAME).join(CENTERLINE_FILE_NAME).exists());
        assert!(result.folder.join(INFO_FILE_NAME).exists());
        assert!(result.num_race_line_points > 0);

        std::fs::remove_dir_all(&config.output_root).ok();
    }

    #[test]
    fn same_seed_is_reproducible() {
        let config = scratch_config("repro");
        let first = generate(&config, Some("a")).expect("first generation should succeed");
        let second = generate(&config, Some("b")).expect("second generation should succeed");

        let csv_a = std::fs::read(first.folder.join(RACE_LINES_DIR_NAME).join(CENTERLINE_FILE_NAME)).unwrap();
        let csv_b = std::fs::read(second.folder.join(RACE_LINES_DIR_NAME).join(CENTERLINE_FILE_NAME)).unwrap();
        assert_eq!(csv_a, csv_b);

        let tiff_a = std::fs::read(first.folder.join(MAP_TIFF_FILE_NAME)).unwrap();
        let tiff_b = std::fs::read(second.folder.join(MAP_TIFF_FILE_NAME)).unwrap();
        assert_eq!(tiff_a, tiff_b);

        std::fs::remove_dir_all(&config.output_root).ok();
    }
}
