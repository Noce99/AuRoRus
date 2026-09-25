//! The topics tying [`crate::localization::Slam`] to whoever drives it (e.g.
//! `web_gui`'s Mapping panel): [`SlamCommand`] says what SLAM should be
//! doing, [`SlamSaveRequest`] asks it to save its map, [`SlamStatus`]
//! reports what it's actually doing, and [`SlamMap`] carries the map built
//! so far - mirroring the
//! [`crate::topics::VehicleModelSelection`]/[`crate::topics::VehicleModelStatus`]
//! pair.

use std::sync::Arc;

/// Name of the topic [`SlamCommand`] is published on.
pub const SLAM_COMMAND_TOPIC_NAME: &str = "slam_command";
/// Name of the topic [`SlamStatus`] is published on.
pub const SLAM_STATUS_TOPIC_NAME: &str = "slam_status";
/// Name of the topic [`SlamMap`] is published on.
pub const SLAM_MAP_TOPIC_NAME: &str = "slam_map";
/// Name of the topic [`SlamSaveRequest`] is published on.
pub const SLAM_SAVE_TOPIC_NAME: &str = "slam_save";

/// What SLAM is (or should be) doing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SlamState {
    /// Nothing in memory, and no scan taken - entering this state throws
    /// away the whole map.
    #[default]
    Off,
    /// Paused: the map built so far is kept, but new scans are ignored.
    Waiting,
    /// Mapping: every new scan is matched and added to the map.
    Running,
    /// Localizing on the selected map ([`crate::topics::SelectedMap`]):
    /// every new scan is matched against it, and the map never changes.
    /// Refused - SLAM stays in [`SlamState::Waiting`] - while a map built by
    /// SLAM is in memory.
    Localizing,
    /// Localization paused: the pose is kept, but new scans are ignored.
    LocalizationPaused,
}

impl SlamState {
    /// The name the web API uses for this state - the same as its serde
    /// name.
    pub fn api_str(self) -> &'static str {
        match self {
            SlamState::Off => "off",
            SlamState::Waiting => "waiting",
            SlamState::Running => "running",
            SlamState::Localizing => "localizing",
            SlamState::LocalizationPaused => "localization_paused",
        }
    }

    /// The state named `name` in the web API, if any - see
    /// [`Self::api_str`].
    pub fn from_api_str(name: &str) -> Option<Self> {
        SlamState::ALL
            .into_iter()
            .find(|state| state.api_str() == name)
    }

    /// Every state.
    pub const ALL: [SlamState; 5] = [
        SlamState::Off,
        SlamState::Waiting,
        SlamState::Running,
        SlamState::Localizing,
        SlamState::LocalizationPaused,
    ];

    /// Whether this is one of the localization states.
    pub fn is_localization(self) -> bool {
        matches!(self, SlamState::Localizing | SlamState::LocalizationPaused)
    }
}

/// What a driver of SLAM (e.g. `web_gui`) wants it to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub struct SlamCommand {
    pub state: SlamState,
    /// Bumped on every write of [`SlamState::Off`]. SLAM clears its memory
    /// whenever this no longer matches what it last applied - so a quick
    /// Clear then Play, both landing between two of SLAM's ticks, still
    /// clears. A counter rather than a bool for the same reason as
    /// [`crate::topics::PlaceAtStart`].
    pub clear_requested: u64,
}

/// Asks SLAM to save the map built so far as a new map folder named `name`
/// under its maps root - see [`crate::localization::Slam`].
#[derive(Debug, Clone, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub struct SlamSaveRequest {
    /// The new map folder's name.
    pub name: String,
    /// Bumped on every request: SLAM saves whenever this no longer matches
    /// the last one it handled, as [`SlamCommand::clear_requested`].
    pub requested: u64,
}

/// How SLAM handled a [`SlamSaveRequest`]: exactly one of `saved_to` and
/// `error` is set.
#[derive(Debug, Clone, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub struct SlamSaveOutcome {
    /// The [`SlamSaveRequest::requested`] this answers.
    pub requested: u64,
    /// The folder the map was saved to.
    pub saved_to: Option<String>,
    /// Why the map wasn't saved.
    pub error: Option<String>,
}

/// What [`crate::localization::Slam`] is currently doing, so a driver of
/// [`SlamCommand`] can reflect it.
#[derive(Debug, Clone, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct SlamStatus {
    /// The state SLAM is actually in.
    pub state: SlamState,
    /// How many scans the map is built from so far.
    pub scans: usize,
    /// Latest corrected pose `(x_m, y_m, heading_rad)` - `None` before the
    /// first scan. While mapping, in SLAM's own frame (see [`SlamMap`]);
    /// while localizing, in the selected map's frame.
    pub pose: Option<[f64; 3]>,
    /// While localizing, where odometry's frame sits on the map as
    /// `(x_m, y_m, heading_rad)`: odometry's pose composed onto this is the
    /// vehicle's pose on the map, between two scans too. `None` otherwise.
    pub map_to_odom: Option<[f64; 3]>,
    /// The [`crate::topics::Odometry::reset_count`] the map is being built
    /// against.
    pub odometry_reset_count: u64,
    /// How well the latest scan matched the map, from `0` (no overlap at
    /// all) to `1` (perfect) - `None` before the second scan, the first
    /// one having nothing to match against.
    pub last_match_response: Option<f64>,
    /// How long processing the latest scan took, in milliseconds -
    /// including any loop closure it triggered.
    pub last_process_ms: Option<f64>,
    /// How many loops have been closed since the map was last cleared.
    pub loop_closures: usize,
    /// How long the latest pose graph optimization took, in milliseconds -
    /// `None` until a loop is closed.
    pub last_optimization_ms: Option<f64>,
    /// How the latest [`SlamSaveRequest`] went - `None` before the first.
    pub last_save: Option<SlamSaveOutcome>,
}

/// The occupancy map built so far, in SLAM's own frame: the `odom` frame of
/// the [`crate::topics::Odometry`] it was built from, i.e. its origin is
/// where dead reckoning was last reset and its x axis the vehicle's heading
/// there. Grows as the vehicle explores.
#[derive(Debug, Clone, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct SlamMap {
    /// Side of one pixel, in meters.
    pub resolution_m_per_px: f64,
    /// Where pixel `(0, 0)`'s corner sits, in meters. Rows grow along +y.
    pub origin_x_m: f64,
    pub origin_y_m: f64,
    /// Width of `pixels`, in pixels.
    pub width_px: u32,
    /// Height of `pixels`, in pixels.
    pub height_px: u32,
    /// One byte per pixel, row-major: [`SlamMap::FREE`],
    /// [`SlamMap::OCCUPIED`] or [`SlamMap::UNKNOWN`]. Behind an [`Arc`] for
    /// the same reason as [`crate::topics::SelectedMap::pixels`].
    pub pixels: Arc<[u8]>,
    /// Every corrected pose a scan was taken from, oldest first.
    pub trajectory: Vec<[f32; 2]>,
}

impl SlamMap {
    /// A cell beams pass through - drawn like a map's drivable pixels.
    pub const FREE: u8 = 255;
    /// A cell beams end in - drawn like a map's walls.
    pub const OCCUPIED: u8 = 0;
    /// A cell too few beams have crossed to tell - drawn mid-gray.
    pub const UNKNOWN: u8 = 128;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_state_round_trips_through_its_api_name() {
        for state in SlamState::ALL {
            assert_eq!(SlamState::from_api_str(state.api_str()), Some(state));
            assert_eq!(
                serde_json::to_string(&state).unwrap(),
                format!("\"{}\"", state.api_str())
            );
        }
        assert_eq!(SlamState::from_api_str("paused"), None);
    }

    /// The debug recorder serializes every topic, so the map must survive a
    /// bincode round trip.
    #[test]
    fn a_map_survives_a_bincode_round_trip() {
        let map = SlamMap {
            resolution_m_per_px: 0.05,
            origin_x_m: -1.0,
            origin_y_m: 2.0,
            width_px: 2,
            height_px: 1,
            pixels: vec![SlamMap::FREE, SlamMap::OCCUPIED].into(),
            trajectory: vec![[0.0, 0.0], [0.5, 0.1]],
        };

        let encoded = bincode::serde::encode_to_vec(&map, bincode::config::standard()).unwrap();
        let (decoded, _): (SlamMap, _) =
            bincode::serde::decode_from_slice(&encoded, bincode::config::standard()).unwrap();

        assert_eq!(decoded, map);
    }
}
