//! Orchestrates one full map generation run: sites -> loop -> smoothed race
//! line -> speeds -> raster -> writes `map.tiff`,
//! `race_lines/centerline.csv`, and `info.json` into a new folder under
//! [`GenerationConfig::output_root`].

use crate::environment::info::{self, ImageOrigin, MapInfo, MapSource, StartFinishLine, WorldPoint};
use crate::environment::map::{CENTERLINE_FILE_NAME, INFO_FILE_NAME, MAP_TIFF_FILE_NAME, RACE_LINES_DIR_NAME};
use crate::environment::race_line::{self, SpeedPoint};
use crate::environment::simulator::config::GenerationConfig;
use crate::environment::simulator::dynamics;
use crate::environment::simulator::points::{self, PointSamplingError};
use crate::environment::simulator::raster::{self, ImageTransform};
use crate::environment::simulator::smoothing;
use crate::environment::simulator::start_finish;
use crate::environment::simulator::voronoi_loop::{self, LoopConstructionError};
use crate::environment::tiff;
use rand::SeedableRng;
use rand::rngs::StdRng;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

/// Error returned by [`generate`].
#[derive(Debug)]
pub enum MapGenerationError {
    InvalidConfig(String),
    PointSampling(PointSamplingError),
    LoopConstruction(LoopConstructionError),
    Tiff(tiff::TiffWriteError),
    RaceLine(race_line::RaceLineWriteError),
    Info(info::InfoWriteError),
    /// The target folder already holds a map and `overwrite` was `false` -
    /// see [`generate`]. Generating over it would silently destroy that map,
    /// so the caller has to say explicitly that it wants that.
    FolderExists(PathBuf),
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
            Self::FolderExists(path) => write!(
                f,
                "a map already exists at {path:?} - pick another name, delete it first, \
                 or generate with overwrite enabled"
            ),
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
///
/// Refuses - with [`MapGenerationError::FolderExists`] - to write into a
/// folder that already holds a map, unless `overwrite` is `true`. Writing
/// the three output files into an existing map's folder would replace that
/// map in place with no way to get it back, so a caller that genuinely
/// wants that has to ask for it.
pub fn generate(
    config: &GenerationConfig,
    folder_name: Option<&str>,
    overwrite: bool,
) -> Result<GeneratedMap, MapGenerationError> {
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
    let mut closed_points = smoothing::resample_even_spacing(&dense, config.point_spacing_m);
    start_finish::rotate_to_straightest(&mut closed_points, config.track_width_m);

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
    if !overwrite && holds_a_map(&folder) {
        return Err(MapGenerationError::FolderExists(folder));
    }
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

/// Whether `folder` already holds a generated map, i.e. has the `info.json`
/// every map is identified by (see [`crate::environment::Map::load`]). An
/// empty or unrelated folder doesn't count - only a real map is worth
/// refusing to clobber.
fn holds_a_map(folder: &Path) -> bool {
    folder.join(INFO_FILE_NAME).exists()
}

/// A fresh seed for one generation run, for callers that don't have an
/// explicit one to use (`web_gui`'s generate popup, `generate_map` without
/// `--seed`).
///
/// Derived from the wall clock at *nanosecond* resolution and mixed with a
/// per-process counter, so two runs started in the same second - two quick
/// clicks of "Generate Map", say - can't come out with the same seed. A
/// second-resolution seed (what this used to be) made them produce not just
/// the same map but the same [`default_folder_name`], so the second run
/// silently overwrote the first.
pub fn random_seed() -> u64 {
    /// Distinguishes two calls that land in the same nanosecond. Scaled by
    /// the golden-ratio constant so consecutive counter values land far
    /// apart rather than in adjacent seeds.
    static COUNTER: AtomicU64 = AtomicU64::new(0);

    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_nanos() as u64);
    let counter = COUNTER.fetch_add(1, Ordering::Relaxed);
    nanos.wrapping_add(counter.wrapping_mul(0x9E37_79B9_7F4A_7C15))
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
        let result = generate(&config, Some("run"), false).expect("generation should succeed with these params");

        assert!(result.folder.join(MAP_TIFF_FILE_NAME).exists());
        assert!(result.folder.join(RACE_LINES_DIR_NAME).join(CENTERLINE_FILE_NAME).exists());
        assert!(result.folder.join(INFO_FILE_NAME).exists());
        assert!(result.num_race_line_points > 0);

        std::fs::remove_dir_all(&config.output_root).ok();
    }

    #[test]
    fn same_seed_is_reproducible() {
        let config = scratch_config("repro");
        let first = generate(&config, Some("a"), false).expect("first generation should succeed");
        let second = generate(&config, Some("b"), false).expect("second generation should succeed");

        let csv_a = std::fs::read(first.folder.join(RACE_LINES_DIR_NAME).join(CENTERLINE_FILE_NAME)).unwrap();
        let csv_b = std::fs::read(second.folder.join(RACE_LINES_DIR_NAME).join(CENTERLINE_FILE_NAME)).unwrap();
        assert_eq!(csv_a, csv_b);

        let tiff_a = std::fs::read(first.folder.join(MAP_TIFF_FILE_NAME)).unwrap();
        let tiff_b = std::fs::read(second.folder.join(MAP_TIFF_FILE_NAME)).unwrap();
        assert_eq!(tiff_a, tiff_b);

        std::fs::remove_dir_all(&config.output_root).ok();
    }

    #[test]
    fn generating_over_an_existing_map_is_refused_unless_overwrite_is_set() {
        let config = scratch_config("clobber");
        let first = generate(&config, Some("run"), false).expect("first generation should succeed");
        let original = std::fs::read(first.folder.join(MAP_TIFF_FILE_NAME)).unwrap();

        let different = GenerationConfig { seed: config.seed + 1, ..config.clone() };
        let err = generate(&different, Some("run"), false)
            .expect_err("generating over an existing map must be refused");
        assert!(matches!(err, MapGenerationError::FolderExists(_)));
        // ...and the original map is still exactly as it was.
        assert_eq!(std::fs::read(first.folder.join(MAP_TIFF_FILE_NAME)).unwrap(), original);

        // With overwrite it goes through, and really does replace the map.
        generate(&different, Some("run"), true).expect("overwrite: true should succeed");
        assert_ne!(std::fs::read(first.folder.join(MAP_TIFF_FILE_NAME)).unwrap(), original);

        std::fs::remove_dir_all(&config.output_root).ok();
    }

    #[test]
    fn freshly_rolled_seeds_differ_within_the_same_second() {
        // The bug this guards: a seed at one-second resolution handed two
        // back-to-back generations the same seed *and* the same default
        // folder name, so the second silently overwrote the first.
        let seeds: Vec<u64> = (0..100).map(|_| random_seed()).collect();
        let unique: std::collections::HashSet<u64> = seeds.iter().copied().collect();
        assert_eq!(unique.len(), seeds.len(), "every freshly rolled seed must be distinct");

        let names: std::collections::HashSet<String> =
            seeds.iter().map(|&s| default_folder_name(s)).collect();
        assert_eq!(names.len(), seeds.len(), "distinct seeds must give distinct folder names");
    }

    #[test]
    fn generated_map_loads_back_correctly() {
        let config = scratch_config("load");
        let generated = generate(&config, Some("run"), false).expect("generation should succeed with these params");

        let map = crate::environment::Map::load(&generated.folder).expect("loading a freshly generated map should succeed");

        assert_eq!(map.info.width_px, generated.width_px);
        assert_eq!(map.info.height_px, generated.height_px);
        assert_eq!(map.raster.width_px, generated.width_px);
        assert_eq!(map.raster.height_px, generated.height_px);
        assert_eq!(map.race_line.len(), generated.num_race_line_points);

        std::fs::remove_dir_all(&config.output_root).ok();
    }
}
