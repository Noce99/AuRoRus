//! One way to produce a [`crate::environment::Map`]: procedurally generating
//! a synthetic race track (a [`crate::environment::MapSource::Random`] map),
//! plus vehicle dynamics models ([`vehicle`]) so algorithms can be exercised
//! against it without a real track or hardware. See [`generate`] for the
//! generation entry point and [`GenerationConfig`] for its tunable
//! parameters; see [`crate::environment::Map::load`] to read a generated map
//! back into memory.

mod config;
mod dynamics;
mod generator;
mod points;
pub(crate) mod raster;
pub(crate) mod smoothing;
mod start_finish;
mod voronoi_loop;
pub mod vehicle;

pub use config::GenerationConfig;
pub use generator::{GeneratedMap, MapGenerationError, generate, random_seed};
