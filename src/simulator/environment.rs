//! Random environment/map generation for the vehicle simulator: builds a
//! drivable-area raster and race line(s) for a synthetic race track, so
//! [`crate::simulator::vehicle`] algorithms can be exercised without a real
//! track or hardware. See [`generate`] for the entry point and
//! [`GenerationConfig`] for the tunable parameters.
//!
//! A generated map is a folder containing `map.tiff` (a CCITT Group 4
//! compressed binary raster - white is drivable track, black is outside),
//! `race_lines/centerline.csv` (a closed, evenly arc-length-spaced
//! `x,y,speed` path), and `info.json` (resolution, origin, start/finish
//! line, and other metadata).

mod config;
mod dynamics;
mod generator;
mod info;
mod points;
mod race_line;
mod raster;
mod smoothing;
mod start_finish;
mod tiff;
mod voronoi_loop;

pub use config::GenerationConfig;
pub use generator::{GeneratedMap, MapGenerationError, generate};
pub use info::{ImageOrigin, MapInfo, MapSource, StartFinishLine, WorldPoint};
