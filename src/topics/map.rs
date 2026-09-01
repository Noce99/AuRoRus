//! The [`SelectedMap`]/[`MapSelection`] topic pair: which map folder is
//! currently wanted, and the currently-loaded map's path and pixels -
//! together, how [`crate::sensors::MapServer`] and a driver of the wanted
//! selection (e.g. `web_gui`) agree on the live map without either one
//! touching the other's disk I/O directly.

use crate::environment::MapInfo;
use std::path::PathBuf;

/// Name of the topic [`SelectedMap`] is published on.
pub const MAP_TOPIC_NAME: &str = "map";
/// Name of the topic [`MapSelection`] is published on.
pub const MAP_SELECTION_TOPIC_NAME: &str = "map_selection";

/// The currently loaded map: which folder it came from, its occupancy raster
/// pixels - one byte per pixel, row-major, `255` for drivable (white) and `0`
/// otherwise, matching [`crate::environment::Raster::to_bytes`] - and its full
/// [`MapInfo`] (resolution, origin, start/finish line, ...). Published by
/// [`crate::sensors::MapServer`] once it has loaded whatever [`MapSelection`]
/// currently asks for.
///
/// `info` duplicates what a live client could otherwise fetch straight from
/// disk (as `web_gui`'s frontend does, via `GET /api/maps/{name}/info`) - it's
/// carried here too so this topic alone is enough to render the map, which
/// matters for a recorded `.debug` session played back with no access to the
/// original `maps/` folder.
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct SelectedMap {
    /// Folder the currently loaded map was read from, or `None` if no map
    /// has been loaded yet.
    pub path: Option<PathBuf>,
    /// Width of `pixels`, in pixels.
    pub width_px: u32,
    /// Height of `pixels`, in pixels.
    pub height_px: u32,
    /// The raster pixels themselves - `width_px * height_px` bytes,
    /// row-major.
    pub pixels: Vec<u8>,
    /// This map's full metadata, or `None` if no map has been loaded yet.
    pub info: Option<MapInfo>,
}

/// The map folder a driver of the selection (e.g. `web_gui`) currently
/// wants loaded. [`crate::sensors::MapServer`] polls this and republishes
/// [`SelectedMap`] whenever it no longer matches
/// [`SelectedMap::path`].
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct MapSelection {
    /// Wanted map folder, or `None` for no map selected.
    pub path: Option<PathBuf>,
}
