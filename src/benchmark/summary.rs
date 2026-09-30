//! [`BenchmarkSummary`]: everything about one benchmark run worth reading or
//! filtering on - what drove, where, with which parameters, and how it
//! went - as the run folder's `summary.toml`. Written and read through this
//! one type, so the writer and any viewer can't drift apart.

use crate::environment::RaceLineMethod;
use crate::telemetry::TelemetryPoseSource;
use std::collections::BTreeMap;
use std::path::Path;

/// [`BenchmarkSummary::format_version`] of the summaries written now - bumped
/// whenever the format changes in a way a reader has to know about.
pub const FORMAT_VERSION: u32 = 1;

/// How a run ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RunStatus {
    /// Every requested lap was driven.
    Completed,
    /// A lap took longer than [`MapRecord::lap_timeout_s`].
    Timeout,
    /// Stopped before the end: the Abort button, a WASD key, or the
    /// program shutting down. Also what a run still in progress (or cut
    /// short by a crash) says, since it's written before the run starts.
    AbortedByUser,
}

/// One run's `summary.toml`. Fields a viewer filters on sit at fixed
/// places; parameters are name -> value tables, so any of them can be.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BenchmarkSummary {
    /// See [`FORMAT_VERSION`].
    pub format_version: u32,
    pub status: RunStatus,
    /// When the run started (before the countdown), RFC 3339 in local time.
    pub started_at: String,
    /// How long the countdown lasted, in seconds - the trajectory's `t_s`
    /// counts from its end, the "Go!".
    pub countdown_s: f64,
    pub laps_requested: u32,
    /// Where lap timing took the vehicle's pose from.
    pub pose_source: TelemetryPoseSource,
    pub code: CodeVersion,
    pub map: MapRecord,
    pub race_line: RaceLineRecord,
    pub algorithm: AlgorithmRecord,
    pub vehicle: VehicleRecord,
    /// `None` until at least one lap was completed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub results: Option<Results>,
    /// Every completed lap, in order.
    #[serde(default)]
    pub laps: Vec<LapResult>,
}

/// The code that ran, as far as git can tell.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CodeVersion {
    /// `git rev-parse HEAD` - `None` outside a git checkout.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub commit: Option<String>,
    /// Whether the working tree had uncommitted changes.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dirty: Option<bool>,
}

/// The map driven - copied into the run folder's `map/`.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct MapRecord {
    /// The map's folder name under `maps/`.
    pub name: String,
    /// SHA-256 of its `info.json`.
    pub info_sha256: String,
    /// SHA-256 of its `map.tiff`.
    pub tiff_sha256: String,
    /// SHA-256 of its centerline, copied into `map/race_lines/` - `None` if
    /// it has none (or the run predates the copy).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub centerline_sha256: Option<String>,
    /// Length of one lap of its centerline, in meters.
    pub centerline_length_m: f64,
    /// Longest a lap may take before the run ends as [`RunStatus::Timeout`],
    /// in seconds.
    pub lap_timeout_s: f64,
}

/// The race line laps were timed against - copied into the run folder as
/// `race_line.csv`.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RaceLineRecord {
    /// File name inside the map's `race_lines/`.
    pub file: String,
    pub method: RaceLineMethod,
    /// Whether the algorithm follows it - if not, it's the centerline, only
    /// there to time the laps against.
    pub used_by_algorithm: bool,
    /// SHA-256 of the CSV.
    pub sha256: String,
    /// Length of one lap of it, in meters.
    pub length_m: f64,
}

/// The algorithm that drove.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AlgorithmRecord {
    /// Its file stem, e.g. `path_follower`.
    pub name: String,
    /// Whether `parameters` all match its config file - if not, they were
    /// tuned in the UI and not saved.
    pub saved_to_config: bool,
    /// The values it ran with.
    pub parameters: BTreeMap<String, f64>,
}

/// The simulated vehicle.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct VehicleRecord {
    /// The model's API string, e.g. `dynamic_bicycle`.
    pub model: String,
    pub body_length_m: f64,
    pub body_width_m: f64,
    /// Whether `parameters` and `limits` all match the vehicle's config file.
    pub saved_to_config: bool,
    /// The model's parameter values it ran with.
    pub parameters: BTreeMap<String, f64>,
    /// The actuator limits it ran with.
    pub limits: BTreeMap<String, f64>,
    /// The simulated car's size - `None` in runs from before it was a
    /// car's (see [`crate::hardware::CarCalibration`]), whose `parameters`
    /// held the model's own axle distances instead.
    #[serde(default)]
    pub geometry: Option<crate::topics::VehicleGeometry>,
}

/// Lap time statistics over the completed laps.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Results {
    /// Every lap's time added up - only when the run was
    /// [`RunStatus::Completed`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub total_time_s: Option<f64>,
    pub best_lap_s: f64,
    pub mean_lap_s: f64,
    /// Population standard deviation of the lap times.
    pub std_lap_s: f64,
}

impl Results {
    /// The statistics of `laps`, `None` if there are none. `total_time_s` is
    /// only filled in when `completed`.
    pub fn of(laps: &[LapResult], completed: bool) -> Option<Self> {
        if laps.is_empty() {
            return None;
        }
        let times: Vec<f64> = laps.iter().map(|lap| lap.time_s).collect();
        let total: f64 = times.iter().sum();
        let mean = total / times.len() as f64;
        let variance =
            times.iter().map(|time| (time - mean).powi(2)).sum::<f64>() / times.len() as f64;
        Some(Self {
            total_time_s: completed.then_some(total),
            best_lap_s: times.iter().copied().fold(f64::INFINITY, f64::min),
            mean_lap_s: mean,
            std_lap_s: variance.sqrt(),
        })
    }
}

/// One completed lap.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LapResult {
    /// Counted from `1`.
    pub number: u32,
    pub time_s: f64,
    /// Length of the path actually driven, in meters.
    pub distance_m: f64,
    pub average_speed_mps: f64,
    /// Largest distance from the race line over the lap, in meters.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_abs_lateral_m: Option<f64>,
    /// Mean distance from the race line over the lap, in meters.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mean_abs_lateral_m: Option<f64>,
}

impl BenchmarkSummary {
    /// Reads a `summary.toml`.
    pub fn read(path: &Path) -> Result<Self, String> {
        let text = std::fs::read_to_string(path)
            .map_err(|err| format!("failed to read {}: {err}", path.display()))?;
        toml::from_str(&text).map_err(|err| format!("failed to parse {}: {err}", path.display()))
    }

    /// Writes this summary to `path`, replacing it.
    pub fn write(&self, path: &Path) -> Result<(), String> {
        let text = toml::to_string_pretty(self)
            .map_err(|err| format!("failed to serialize the benchmark summary: {err}"))?;
        std::fs::write(path, text)
            .map_err(|err| format!("failed to write {}: {err}", path.display()))
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    pub(crate) fn summary() -> BenchmarkSummary {
        let laps = vec![
            LapResult {
                number: 1,
                time_s: 12.0,
                distance_m: 40.0,
                average_speed_mps: 40.0 / 12.0,
                max_abs_lateral_m: Some(0.3),
                mean_abs_lateral_m: Some(0.1),
            },
            LapResult {
                number: 2,
                time_s: 10.0,
                distance_m: 40.5,
                average_speed_mps: 4.05,
                max_abs_lateral_m: None,
                mean_abs_lateral_m: None,
            },
        ];
        BenchmarkSummary {
            format_version: FORMAT_VERSION,
            status: RunStatus::Completed,
            started_at: "2026-09-29T14:03:12+02:00".to_string(),
            countdown_s: 3.0,
            laps_requested: 2,
            pose_source: TelemetryPoseSource::GroundTruth,
            code: CodeVersion {
                commit: Some("813c951".to_string()),
                dirty: Some(false),
            },
            map: MapRecord {
                name: "map".to_string(),
                info_sha256: "aa".to_string(),
                tiff_sha256: "bb".to_string(),
                centerline_sha256: Some("dd".to_string()),
                centerline_length_m: 42.0,
                lap_timeout_s: 42.0,
            },
            race_line: RaceLineRecord {
                file: "centerline.csv".to_string(),
                method: RaceLineMethod::Centerline,
                used_by_algorithm: false,
                sha256: "cc".to_string(),
                length_m: 42.0,
            },
            algorithm: AlgorithmRecord {
                name: "gap_follower".to_string(),
                saved_to_config: true,
                parameters: BTreeMap::from([("max_speed_mps".to_string(), 4.5)]),
            },
            vehicle: VehicleRecord {
                model: "bicycle".to_string(),
                body_length_m: 0.45,
                body_width_m: 0.25,
                saved_to_config: false,
                parameters: BTreeMap::from([("wheelbase_m".to_string(), 0.33)]),
                limits: BTreeMap::from([("max_speed_mps".to_string(), 8.0)]),
                geometry: Some(crate::topics::VehicleGeometry::default()),
            },
            results: Results::of(&laps, true),
            laps,
        }
    }

    #[test]
    fn a_summary_round_trips_through_toml() {
        let summary = summary();
        let text = toml::to_string_pretty(&summary).unwrap();
        assert_eq!(toml::from_str::<BenchmarkSummary>(&text).unwrap(), summary);
    }

    #[test]
    fn results_summarize_the_lap_times() {
        let results = summary().results.unwrap();
        assert_eq!(results.total_time_s, Some(22.0));
        assert_eq!(results.best_lap_s, 10.0);
        assert_eq!(results.mean_lap_s, 11.0);
        assert_eq!(results.std_lap_s, 1.0);
        assert_eq!(
            Results::of(&summary().laps, false).unwrap().total_time_s,
            None
        );
        assert_eq!(Results::of(&[], true), None);
    }
}
