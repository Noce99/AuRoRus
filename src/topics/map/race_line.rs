//! The [`SelectedRaceLine`]/[`RaceLineSelection`] topic pair: the line a
//! vehicle should follow on the currently selected map, and which of the
//! map's lines (see [`crate::environment::race_lines`]) a driver (e.g.
//! `web_gui`) picked. Both handled by [`crate::environment::MapServer`]
//! alongside [`crate::topics::SelectedMap`]: it publishes the newest
//! planned line of a freshly loaded map (or its centerline, if it has no
//! planned one), then whatever a new [`RaceLineSelection`] asks for.

use crate::environment::race_lines::{self, RaceLineFileError};
use crate::environment::{RaceLineMethod, SpeedPoint};
use std::path::{Path, PathBuf};

/// Name of the topic [`SelectedRaceLine`] is published on.
pub const RACE_LINE_TOPIC_NAME: &str = "race_line";
/// Name of the topic [`RaceLineSelection`] is published on.
pub const RACE_LINE_SELECTION_TOPIC_NAME: &str = "race_line_selection";

/// The line to follow on the selected map: closed, one point per row of its
/// CSV file, the last point's successor being the first.
#[derive(Debug, Clone, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct SelectedRaceLine {
    /// Folder of the map this line belongs to - `None` with no map loaded.
    pub map: Option<PathBuf>,
    /// The line's file inside the map's `race_lines/` - `None` if the map
    /// has no line at all.
    pub file: Option<String>,
    pub method: RaceLineMethod,
    /// The line's points - empty without a `file`.
    pub points: Vec<SpeedPoint>,
}

impl SelectedRaceLine {
    /// The map in `folder`'s race line `file` (inside its `race_lines/`).
    pub fn load(folder: &Path, file: &str) -> Result<Self, RaceLineFileError> {
        // First, so `file` is known to be a valid name before anything else
        // reads the folder with it.
        let points = race_lines::read(folder, file)?;
        Ok(Self {
            map: Some(folder.to_path_buf()),
            file: Some(file.to_string()),
            method: race_lines::meta(folder, file).method,
            points,
        })
    }
}

/// Which of a map's race lines a driver (e.g. `web_gui`) wants followed.
/// [`crate::environment::MapServer`] acts on every new write of it - not on its
/// value, so picking the same line again after a newer one was planned
/// still switches back - as long as `map` is the map it has loaded.
#[derive(Debug, Clone, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct RaceLineSelection {
    /// The map folder the choice was made for.
    pub map: Option<PathBuf>,
    /// File name inside that map's `race_lines/`.
    pub file: String,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_race_line_survives_a_bincode_round_trip() {
        let line = SelectedRaceLine {
            map: Some(PathBuf::from("maps/track")),
            file: Some("2026_09_25__15_30_12.csv".to_string()),
            method: RaceLineMethod::MinTime,
            points: vec![SpeedPoint {
                x: 1.0,
                y: -2.0,
                speed_mps: 3.5,
            }],
        };

        let encoded = bincode::serde::encode_to_vec(&line, bincode::config::standard()).unwrap();
        let (decoded, _): (SelectedRaceLine, _) =
            bincode::serde::decode_from_slice(&encoded, bincode::config::standard()).unwrap();

        assert_eq!(decoded, line);
    }
}
