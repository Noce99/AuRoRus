//! The [`SelectedMap`]/[`MapSelection`] topic pair: which map folder is
//! currently wanted, and the currently-loaded map's path and pixels -
//! together, how [`crate::sensors::MapServer`] and a driver of the wanted
//! selection (e.g. `web_gui`) agree on the live map without either one
//! touching the other's disk I/O directly.

use crate::environment::MapInfo;
use std::path::PathBuf;
use std::sync::Arc;

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
    ///
    /// Behind an [`Arc`] because [`crate::RwLockTopic::read`] hands every
    /// reader an owned clone of the whole topic, and for a 1200x1200 map
    /// that's 1.44 MB copied per read - paid by every `GET /api/map` (which
    /// only wants the name and dimensions) and by `MapServer`'s own poll
    /// loop. Sharing the buffer makes those clones a refcount bump; the
    /// pixels are never mutated in place, only replaced wholesale when a
    /// different map is loaded.
    pub pixels: Arc<[u8]>,
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
    /// Bumped to have the same folder read from disk again, after its files
    /// changed (e.g. `web_gui` moving its start/finish line) - any change
    /// reloads it, even with `path` unchanged.
    #[serde(default)]
    pub revision: u64,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `RwLockTopic::read` clones the whole value, so this is what keeps a
    /// `GET /api/map` (name and dimensions only) and `MapServer`'s poll loop
    /// from copying a 1.44 MB raster each time.
    #[test]
    fn cloning_a_selected_map_shares_its_pixels_rather_than_copying_them() {
        let selected = SelectedMap {
            path: Some("maps/track".into()),
            width_px: 2,
            height_px: 2,
            pixels: vec![255u8, 0, 255, 0].into(),
            info: None,
        };

        let copy = selected.clone();

        assert!(
            Arc::ptr_eq(&selected.pixels, &copy.pixels),
            "cloning SelectedMap must share the raster, not duplicate it"
        );
        assert_eq!(&*copy.pixels, &[255u8, 0, 255, 0]);
    }

    /// The raster still has to survive a recording round trip - the debug
    /// format serializes every topic, and `Arc` is only cheap in-process.
    #[test]
    fn pixels_survive_a_serde_round_trip() {
        let selected = SelectedMap {
            path: None,
            width_px: 2,
            height_px: 1,
            pixels: vec![7u8, 9].into(),
            info: None,
        };

        let encoded =
            bincode::serde::encode_to_vec(&selected, bincode::config::standard()).unwrap();
        let (decoded, _): (SelectedMap, _) =
            bincode::serde::decode_from_slice(&encoded, bincode::config::standard()).unwrap();

        assert_eq!(&*decoded.pixels, &[7u8, 9]);
        assert_eq!(decoded.width_px, 2);
    }
}
