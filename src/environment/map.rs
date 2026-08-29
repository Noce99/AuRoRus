//! [`Map`]: an in-memory, loaded view of a map folder (`map.tiff` +
//! `race_lines/centerline.csv` + `info.json`), independent of how that
//! folder was produced - by [`crate::environment::simulator::generate`]
//! (a [`crate::environment::MapSource::Random`] map) today, or eventually a
//! recorded real-car mapping session (a [`crate::environment::MapSource::Real`]
//! map). See [`Map::load`].

use crate::environment::MapInfo;
use crate::environment::info::{self, InfoReadError};
use crate::environment::race_line::{self, RaceLineReadError, SpeedPoint};
use crate::environment::raster::Raster;
use crate::environment::tiff::{self, TiffReadError};
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
    pub fn load(folder: &Path) -> Result<Map, MapLoadError> {
        let info = info::read(&folder.join(INFO_FILE_NAME))?;
        let raster = tiff::read(&folder.join(MAP_TIFF_FILE_NAME))?;
        let race_line = race_line::read(&folder.join(RACE_LINES_DIR_NAME).join(CENTERLINE_FILE_NAME))?;
        Ok(Map { folder: folder.to_path_buf(), info, raster, race_line })
    }
}

/// Reads back just a map folder's `info.json`, without touching its raster
/// or race line - cheaper than [`Map::load`] when only the metadata is
/// needed, e.g. to list many maps.
pub fn read_info(folder: &Path) -> Result<MapInfo, InfoReadError> {
    info::read(&folder.join(INFO_FILE_NAME))
}
