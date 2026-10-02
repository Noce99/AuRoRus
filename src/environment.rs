//! The environment: map metadata/format ([`info`], [`raster`], [`tiff`],
//! [`race_line`]), the catalog of a map's race lines ([`race_lines`]), the
//! loadable [`Map`] type ([`map`]), and the track generator
//! ([`generator`]) - one way to *produce* a map ([`MapSource::Random`]), for
//! exercising algorithms without a real track. [`crate::localization::Slam`] saves
//! the maps it builds ([`MapSource::Real`]) through [`save`], in the same
//! [`Map::load`]-compatible file format.
//!
//! Also the executors that put a map on its topics: [`MapServer`] keeps the
//! selected map and race line published, and [`RaceLinePublisher`]
//! publishes one fixed race line (e.g. an opponent's).

pub mod generator;
mod info;
mod map;
mod map_server;
pub mod race_line;
mod race_line_publisher;
pub mod race_lines;
mod raster;
mod raycast;
pub mod starting_grid;
mod tiff;

pub use generator::{GeneratedMap, GenerationConfig, MapGenerationError, generate, random_seed};
pub use info::{
    GenerationInfo, ImageOrigin, MapInfo, MapSource, StartFinishLine, WorldPoint, now_rfc3339,
};
pub use map::{
    CENTERLINE_FILE_NAME, INFO_FILE_NAME, MAP_TIFF_FILE_NAME, Map, MapLoadError, MapSaveError,
    ORIGINAL_MAP_TIFF_FILE_NAME, RACE_LINES_DIR_NAME, RasterReplaceError, map_folder, read_info,
    read_original_raster, read_raster, replace_raster, save, write_info, write_line,
};
pub use map_server::{MapServer, MapServerConfig};
pub use race_line::{RaceLineWriteError, SpeedPoint};
pub use race_line_publisher::RaceLinePublisher;
pub use race_lines::{RaceLineEntry, RaceLineMethod};
pub use raster::Raster;
pub(crate) use raycast::cast_ray;
pub use tiff::decode_grayscale as decode_tiff_grayscale;
