//! [`Map`]: an in-memory, loaded view of a map folder (`map.tiff` +
//! `info.json`, plus `race_lines/centerline.csv` when the centerline is
//! known), independent of how that folder was produced - by
//! [`crate::environment::simulator::generate`] (a
//! [`crate::environment::MapSource::Random`] map) or by
//! [`crate::localization::Slam`] (a [`crate::environment::MapSource::Real`]
//! map, with no centerline). See [`Map::load`].

use crate::environment::MapInfo;
use crate::environment::info::{self, InfoReadError, InfoWriteError};
use crate::environment::race_line::{self, RaceLineReadError, SpeedPoint};
use crate::environment::raster::Raster;
use crate::environment::tiff::{self, TiffReadError, TiffWriteError};
use std::path::{Path, PathBuf};

/// Name of the raster file inside a map folder.
pub const MAP_TIFF_FILE_NAME: &str = "map.tiff";
/// Name of the race lines folder inside a map folder.
pub const RACE_LINES_DIR_NAME: &str = "race_lines";
/// Name of the (only, for now) race line file inside `race_lines/`.
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
    /// [`crate::localization::Slam`], whose centerline isn't known).
    pub race_line: Vec<SpeedPoint>,
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
    /// A missing centerline file loads as an empty [`Map::race_line`]; a
    /// malformed one is still an error.
    pub fn load(folder: &Path) -> Result<Map, MapLoadError> {
        let info = info::read(&folder.join(INFO_FILE_NAME))?;
        let raster = tiff::read(&folder.join(MAP_TIFF_FILE_NAME))?;
        let centerline = folder.join(RACE_LINES_DIR_NAME).join(CENTERLINE_FILE_NAME);
        let race_line = if centerline.exists() {
            race_line::read(&centerline)?
        } else {
            Vec::new()
        };
        Ok(Map {
            folder: folder.to_path_buf(),
            info,
            raster,
            race_line,
        })
    }
}

/// Reads back just a map folder's `info.json`, without touching its raster
/// or race line - cheaper than [`Map::load`] when only the metadata is
/// needed, e.g. to list many maps.
pub fn read_info(folder: &Path) -> Result<MapInfo, InfoReadError> {
    info::read(&folder.join(INFO_FILE_NAME))
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
