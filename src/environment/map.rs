//! [`Map`]: an in-memory, loaded view of a map folder (`map.tiff` +
//! `info.json`, plus `race_lines/centerline.csv` when the centerline is
//! known - the planned race lines next to it are catalogued by
//! [`crate::environment::race_lines`]), independent of how that folder was produced - by
//! [`crate::environment::generator::generate`] (a
//! [`crate::environment::MapSource::Random`] map) or by
//! [`crate::localization::Slam`] (a [`crate::environment::MapSource::Real`]
//! map, with no centerline). See [`Map::load`].

use crate::environment::MapInfo;
use crate::environment::info::{self, InfoReadError, InfoWriteError};
use crate::environment::race_line::{self, RaceLineReadError, RaceLineWriteError, SpeedPoint};
use crate::environment::raster::Raster;
use crate::environment::tiff::{self, TiffReadError, TiffWriteError};
use std::path::{Path, PathBuf};

/// Name of the raster file inside a map folder.
pub const MAP_TIFF_FILE_NAME: &str = "map.tiff";
/// Name of the untouched copy of `map.tiff` kept inside a map folder once
/// its pixels are edited by hand - see [`replace_raster`].
pub const ORIGINAL_MAP_TIFF_FILE_NAME: &str = "map.orig.tiff";
/// Name of the race lines folder inside a map folder.
pub const RACE_LINES_DIR_NAME: &str = "race_lines";
/// Name of the centerline file inside `race_lines/`.
pub const CENTERLINE_FILE_NAME: &str = "centerline.csv";
/// Name of the metadata file inside a map folder.
pub const INFO_FILE_NAME: &str = "info.json";

/// An in-memory, loaded map.
#[derive(Debug)]
pub struct Map {
    pub folder: PathBuf,
    pub info: MapInfo,
    pub raster: Raster,
    /// The centerline - empty when the map has none (e.g. a map saved by
    /// [`crate::localization::Slam`], until [`crate::planning`] computes
    /// one).
    pub centerline: Vec<SpeedPoint>,
}

/// Error returned by [`Map::load`].
#[derive(Debug)]
pub enum MapLoadError {
    Info(InfoReadError),
    Tiff(TiffReadError),
    RaceLine(RaceLineReadError),
}

impl std::fmt::Display for MapLoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Info(err) => write!(f, "{err}"),
            Self::Tiff(err) => write!(f, "{err}"),
            Self::RaceLine(err) => write!(f, "{err}"),
        }
    }
}

impl std::error::Error for MapLoadError {}

impl From<InfoReadError> for MapLoadError {
    fn from(err: InfoReadError) -> Self {
        Self::Info(err)
    }
}

impl From<TiffReadError> for MapLoadError {
    fn from(err: TiffReadError) -> Self {
        Self::Tiff(err)
    }
}

impl From<RaceLineReadError> for MapLoadError {
    fn from(err: RaceLineReadError) -> Self {
        Self::RaceLine(err)
    }
}

impl Map {
    /// Reads back a map folder written by any [`crate::environment::MapSource`].
    /// A missing centerline file loads as an empty [`Map::centerline`]; a
    /// malformed one is still an error. The planned race lines aren't
    /// loaded - see [`crate::environment::race_lines`].
    pub fn load(folder: &Path) -> Result<Map, MapLoadError> {
        let info = info::read(&folder.join(INFO_FILE_NAME))?;
        let raster = tiff::read(&folder.join(MAP_TIFF_FILE_NAME))?;
        Ok(Map {
            folder: folder.to_path_buf(),
            info,
            raster,
            centerline: read_line_if_present(folder, CENTERLINE_FILE_NAME)?,
        })
    }
}

/// The line in `folder`'s `race_lines/file_name`, or an empty one if there's
/// no such file.
fn read_line_if_present(folder: &Path, file_name: &str) -> Result<Vec<SpeedPoint>, MapLoadError> {
    let path = folder.join(RACE_LINES_DIR_NAME).join(file_name);
    if path.exists() {
        Ok(race_line::read(&path)?)
    } else {
        Ok(Vec::new())
    }
}

/// Writes `points` as `folder`'s `race_lines/file_name` (e.g.
/// [`CENTERLINE_FILE_NAME`]), creating `race_lines/` if needed and replacing
/// any previous file of that name. Returns the file's path.
pub fn write_line(
    folder: &Path,
    file_name: &str,
    points: &[SpeedPoint],
) -> Result<PathBuf, RaceLineWriteError> {
    let dir = folder.join(RACE_LINES_DIR_NAME);
    std::fs::create_dir_all(&dir)?;
    let path = dir.join(file_name);
    race_line::write(points, &path)?;
    Ok(path)
}

/// Reads back just a map folder's `info.json`, without touching its raster
/// or race line - cheaper than [`Map::load`] when only the metadata is
/// needed, e.g. to list many maps.
pub fn read_info(folder: &Path) -> Result<MapInfo, InfoReadError> {
    info::read(&folder.join(INFO_FILE_NAME))
}

/// Replaces a map folder's `info.json` with `info`, leaving its raster and
/// race lines alone - e.g. to move its start/finish line.
pub fn write_info(folder: &Path, info: &MapInfo) -> Result<(), InfoWriteError> {
    info::write(info, &folder.join(INFO_FILE_NAME))
}

/// Reads a map folder's `map.tiff` alone, without its `info.json` or race
/// lines.
pub fn read_raster(folder: &Path) -> Result<Raster, TiffReadError> {
    tiff::read(&folder.join(MAP_TIFF_FILE_NAME))
}

/// Reads the untouched raster [`replace_raster`] kept aside, or `None` if
/// the map's pixels were never edited.
pub fn read_original_raster(folder: &Path) -> Result<Option<Raster>, TiffReadError> {
    let path = folder.join(ORIGINAL_MAP_TIFF_FILE_NAME);
    if !path.exists() {
        return Ok(None);
    }
    tiff::read(&path).map(Some)
}

/// Error returned by [`replace_raster`].
#[derive(Debug)]
pub enum RasterReplaceError {
    Io(std::io::Error),
    Tiff(TiffWriteError),
}

impl std::fmt::Display for RasterReplaceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(err) => write!(f, "failed to replace map.tiff: {err}"),
            Self::Tiff(err) => write!(f, "{err}"),
        }
    }
}

impl std::error::Error for RasterReplaceError {}

/// Replaces a map folder's `map.tiff` with `raster`, e.g. after editing its
/// pixels by hand. The first time, the current `map.tiff` is copied to
/// [`ORIGINAL_MAP_TIFF_FILE_NAME`] - never afterwards, so it always holds
/// the map as it was made (generated, imported or mapped), whatever edits
/// came since. The new file is written aside and renamed over the old one,
/// so a failed write never leaves the map without a raster.
pub fn replace_raster(folder: &Path, raster: &Raster) -> Result<(), RasterReplaceError> {
    let current = folder.join(MAP_TIFF_FILE_NAME);
    let original = folder.join(ORIGINAL_MAP_TIFF_FILE_NAME);
    if !original.exists() {
        std::fs::copy(&current, &original).map_err(RasterReplaceError::Io)?;
    }
    let staged = folder.join(format!("{MAP_TIFF_FILE_NAME}.tmp"));
    tiff::write(raster, &staged).map_err(RasterReplaceError::Tiff)?;
    std::fs::rename(&staged, &current).map_err(RasterReplaceError::Io)
}

/// Where the map named `name` lives under `maps_root` - `None` unless `name`
/// is a plain folder name: non-empty, and free of any path separator or `..`
/// component, so it can never escape `maps_root` (names come from arbitrary
/// HTTP clients).
pub fn map_folder(maps_root: &Path, name: &str) -> Option<PathBuf> {
    if name.is_empty() || name == "." || name == ".." || name.contains('/') || name.contains('\\') {
        return None;
    }
    Some(maps_root.join(name))
}

/// Error returned by [`save`].
#[derive(Debug)]
pub enum MapSaveError {
    /// The folder already holds a map - saving would silently destroy it.
    FolderExists(PathBuf),
    Io(std::io::Error),
    Tiff(TiffWriteError),
    Info(InfoWriteError),
}

impl std::fmt::Display for MapSaveError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::FolderExists(path) => write!(f, "a map already exists at {path:?}"),
            Self::Io(err) => write!(f, "failed to create the map folder: {err}"),
            Self::Tiff(err) => write!(f, "{err}"),
            Self::Info(err) => write!(f, "{err}"),
        }
    }
}

impl std::error::Error for MapSaveError {}

/// Writes a map with no centerline (`map.tiff` + `info.json`) into a new
/// `folder`, for [`Map::load`] to read back. Refuses to write into a folder
/// that already holds a map.
pub fn save(folder: &Path, info: &MapInfo, raster: &Raster) -> Result<(), MapSaveError> {
    if folder.join(INFO_FILE_NAME).exists() {
        return Err(MapSaveError::FolderExists(folder.to_path_buf()));
    }
    std::fs::create_dir_all(folder).map_err(MapSaveError::Io)?;
    tiff::write(raster, &folder.join(MAP_TIFF_FILE_NAME)).map_err(MapSaveError::Tiff)?;
    info::write(info, &folder.join(INFO_FILE_NAME)).map_err(MapSaveError::Info)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::environment::{ImageOrigin, MapSource, StartFinishLine, WorldPoint, now_rfc3339};

    fn raster(white: [bool; 4]) -> Raster {
        Raster::new(2, 2, white.to_vec())
    }

    #[test]
    fn replacing_the_raster_keeps_the_first_one_as_the_original() {
        let folder =
            std::env::temp_dir().join(format!("aurorus_replace_raster_{}", std::process::id()));
        std::fs::remove_dir_all(&folder).ok();
        let info = MapInfo {
            resolution_m_per_px: 0.05,
            width_px: 2,
            height_px: 2,
            origin: ImageOrigin {
                x: 0.0,
                y: 0.0,
                theta_rad: 0.0,
            },
            start_finish_line: StartFinishLine {
                a: WorldPoint { x: 0.0, y: 0.0 },
                b: WorldPoint { x: 0.1, y: 0.0 },
            },
            generated_at: now_rfc3339(),
            source: MapSource::Real,
            generation: None,
        };
        let mapped = [true, false, false, true];
        save(&folder, &info, &raster(mapped)).unwrap();
        assert!(read_original_raster(&folder).unwrap().is_none());

        let first_edit = [true, true, false, true];
        replace_raster(&folder, &raster(first_edit)).unwrap();
        let second_edit = [false, false, false, true];
        replace_raster(&folder, &raster(second_edit)).unwrap();

        let current = read_raster(&folder).unwrap();
        let original = read_original_raster(&folder).unwrap().unwrap();
        std::fs::remove_dir_all(&folder).ok();
        assert_eq!(current.to_bytes(), raster(second_edit).to_bytes());
        assert_eq!(original.to_bytes(), raster(mapped).to_bytes());
    }
}
