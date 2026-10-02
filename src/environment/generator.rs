//! One way to produce a [`crate::environment::Map`]: procedurally generating
//! a synthetic race track (a [`crate::environment::MapSource::Random`] map),
//! so algorithms can be exercised without a real track - against the
//! vehicle and sensors of [`crate::simulation`]. See [`generate`] for the
//! generation entry point and [`GenerationConfig`] for its tunable
//! parameters; see [`crate::environment::Map::load`] to read a generated map
//! back into memory.

mod config;
mod dynamics;
mod generate;
mod points;
pub(crate) mod raster;
mod start_finish;
mod voronoi_loop;

pub use config::GenerationConfig;
pub use generate::{GeneratedMap, MapGenerationError, generate, random_seed};
