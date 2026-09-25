//! The environment: map metadata/format ([`info`], [`raster`], [`tiff`],
//! [`race_line`]), the loadable [`Map`] type ([`map`]), and the vehicle
//! simulator ([`simulator`]) - one way to *produce* a map
//! ([`MapSource::Random`]) plus vehicle dynamics models, for exercising
//! algorithms without real hardware. [`crate::localization::Slam`] saves
//! the maps it builds ([`MapSource::Real`]) through [`save`], in the same
//! [`Map::load`]-compatible file format.

mod info;
mod map;
mod race_line;
mod raster;
pub mod simulator;
mod tiff;

pub use info::{
    GenerationInfo, ImageOrigin, MapInfo, MapSource, StartFinishLine, WorldPoint, now_rfc3339,
};
pub use map::{Map, MapLoadError, MapSaveError, map_folder, read_info, save};
pub use race_line::SpeedPoint;
pub use raster::Raster;
pub use simulator::{GeneratedMap, GenerationConfig, MapGenerationError, generate, random_seed};
