//! The environment: map metadata/format ([`info`], [`raster`], [`tiff`],
//! [`race_line`]), the catalog of a map's race lines ([`race_lines`]), the
//! loadable [`Map`] type ([`map`]), and the vehicle
//! simulator ([`simulator`]) - one way to *produce* a map
//! ([`MapSource::Random`]) plus vehicle dynamics models, for exercising
//! algorithms without real hardware. [`crate::localization::Slam`] saves
//! the maps it builds ([`MapSource::Real`]) through [`save`], in the same
//! [`Map::load`]-compatible file format.

mod info;
mod map;
pub mod race_line;
pub mod race_lines;
mod raster;
pub mod simulator;
pub mod starting_grid;
mod tiff;

pub use info::{
    GenerationInfo, ImageOrigin, MapInfo, MapSource, StartFinishLine, WorldPoint, now_rfc3339,
};
pub use map::{
    CENTERLINE_FILE_NAME, INFO_FILE_NAME, MAP_TIFF_FILE_NAME, Map, MapLoadError, MapSaveError,
    RACE_LINES_DIR_NAME, map_folder, read_info, save, write_line,
};
pub use race_line::{RaceLineWriteError, SpeedPoint};
pub use race_lines::{RaceLineEntry, RaceLineMethod};
pub use raster::Raster;
pub use simulator::{GeneratedMap, GenerationConfig, MapGenerationError, generate, random_seed};
