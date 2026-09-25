//! The [`SelectedRaceLine`] topic: the line a vehicle should follow on the
//! currently selected map - its planned race line if it has one (see
//! [`crate::planning`]), its centerline otherwise. Published by
//! [`crate::sensors::MapServer`] alongside [`crate::topics::SelectedMap`].

use crate::environment::SpeedPoint;
use std::path::PathBuf;

/// Name of the topic [`SelectedRaceLine`] is published on.
pub const RACE_LINE_TOPIC_NAME: &str = "race_line";

/// Which of a map's lines [`SelectedRaceLine`] holds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RaceLineKind {
    /// The map has neither line.
    #[default]
    None,
    /// The map's centerline - it has no planned race line yet.
    Centerline,
    /// The map's planned race line.
    RaceLine,
}

/// The line to follow on the selected map: closed, one point per row of its
/// CSV file, the last point's successor being the first.
#[derive(Debug, Clone, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct SelectedRaceLine {
    /// Folder of the map this line belongs to - `None` with no map loaded.
    pub map: Option<PathBuf>,
    pub kind: RaceLineKind,
    /// The line's points - empty for [`RaceLineKind::None`].
    pub points: Vec<SpeedPoint>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_race_line_survives_a_bincode_round_trip() {
        let line = SelectedRaceLine {
            map: Some(PathBuf::from("maps/track")),
            kind: RaceLineKind::RaceLine,
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
