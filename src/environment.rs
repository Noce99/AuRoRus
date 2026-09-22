//! The environment: map metadata/format ([`info`], [`raster`], [`tiff`],
//! [`race_line`]), the loadable [`Map`] type ([`map`]), and the vehicle
//! simulator ([`simulator`]) - one way to *produce* a map
//! ([`MapSource::Random`]) plus vehicle dynamics models, for exercising
//! algorithms without real hardware. A future real-car mapping session
//! would live alongside `simulator` here, producing [`MapSource::Real`]
//! maps through the same [`Map::load`]-compatible file format.

mod info;
mod map;
mod race_line;
mod raster;
pub mod simulator;
mod tiff;

pub use info::{ImageOrigin, MapInfo, MapSource, StartFinishLine, WorldPoint};
pub use map::{Map, MapLoadError, read_info};
pub use race_line::SpeedPoint;
pub use raster::Raster;
pub use simulator::{GeneratedMap, GenerationConfig, MapGenerationError, generate, random_seed};
