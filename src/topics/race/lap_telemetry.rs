//! The [`LapTelemetry`] topic: how the ego vehicle is doing against the race
//! line it follows - its Frenet lateral offset and speed error along the
//! line, for the lap in progress and the one before, and every completed
//! lap's time. Published by [`crate::telemetry::LapTelemetryRecorder`],
//! shown by `web_gui`'s (and `replay_web_gui`'s) bottom panel.

/// Name of the topic [`LapTelemetry`] is published on.
pub const LAP_TELEMETRY_TOPIC_NAME: &str = "lap_telemetry";

/// The ego vehicle's laps on the current race line - all of it starts over
/// whenever the line (or the map) changes.
#[derive(Debug, Clone, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct LapTelemetry {
    /// The race line's file inside its map's `race_lines/` - `None` without
    /// a line, when there's nothing else here either.
    pub race_line: Option<String>,
    /// Length of the race line, in meters: the `s` coordinate runs from `0`
    /// (the start/finish line) to this.
    pub lap_length_m: f64,
    /// Where the vehicle projects onto the line, in meters of `s` - `None`
    /// without a (fresh) pose.
    pub s_m: Option<f64>,
    /// Why nothing is being recorded right now, if so - e.g. no race line,
    /// or a stale pose.
    pub status: Option<String>,
    /// Seconds into the lap in progress - `None` while it isn't timed
    /// (e.g. an out lap, after the vehicle was placed away from the line).
    pub current_lap_time_s: Option<f64>,
    /// The lap in progress.
    pub current: LapTrace,
    /// The last completed lap.
    pub previous: Option<LapTrace>,
    /// Every completed lap, oldest first.
    pub laps: Vec<LapRecord>,
}

/// One lap's errors along the line, binned by `s`: bin `i` covers
/// `[i, i + 1) * lap_length_m / bins` - `None` where the vehicle hasn't been
/// (yet) on this lap.
#[derive(Debug, Clone, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct LapTrace {
    /// The lap's number, as in [`LapRecord::number`] - `0` for an untimed lap.
    pub number: u32,
    /// Signed lateral offset from the line, in meters - positive to its
    /// left (seen along the direction of travel).
    pub lateral_m: Vec<Option<f32>>,
    /// Speed minus the line's speed there, in meters/second - positive when
    /// faster than planned.
    pub speed_error_mps: Vec<Option<f32>>,
}

impl LapTrace {
    /// An empty trace of `bins` bins.
    pub fn empty(number: u32, bins: usize) -> Self {
        Self {
            number,
            lateral_m: vec![None; bins],
            speed_error_mps: vec![None; bins],
        }
    }
}

/// One completed lap.
#[derive(Debug, Clone, Copy, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct LapRecord {
    /// Counted from `1`, the first timed lap on this line.
    pub number: u32,
    pub time_s: f64,
    /// Length of the path actually driven, in meters - not the line's.
    pub distance_m: f64,
    /// `distance_m / time_s`.
    pub average_speed_mps: f64,
}
