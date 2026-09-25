//! [`MapInfo`]: the `info.json` schema - everything a consumer needs to
//! interpret `map.tiff` and the race lines without recomputing anything.

use std::path::Path;
use time::OffsetDateTime;
use time::format_description::well_known::Rfc3339;

/// A point in world coordinates (meters).
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct WorldPoint {
    pub x: f64,
    pub y: f64,
}

/// The world-space transform of the image: `x`/`y` is the world position of
/// pixel `(0, 0)`'s corner, and world coordinates increase in the same
/// direction as pixel columns/rows (x rightward, y downward) - so going
/// between pixel and world space only needs a scale by
/// [`MapInfo::resolution_m_per_px`], never an axis flip. `theta_rad` is
/// always `0.0` today; kept for schema parity with a possible future
/// rotated origin.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct ImageOrigin {
    pub x: f64,
    pub y: f64,
    pub theta_rad: f64,
}

/// The two endpoints of the start/finish line, in world coordinates.
///
/// Their order carries the direction of travel: it's `b - a` rotated a
/// quarter turn counterclockwise (`a` on the driver's left, `b` on the
/// right) - so the line alone places the vehicle, see
/// [`StartFinishLine::start_pose`].
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct StartFinishLine {
    pub a: WorldPoint,
    pub b: WorldPoint,
}

impl StartFinishLine {
    /// Where a vehicle starts on this line: its midpoint, heading along the
    /// direction of travel (see [`StartFinishLine`]), as `(x_m, y_m,
    /// heading_rad)`. A degenerate line (both ends equal) heads along `+x`.
    pub fn start_pose(&self) -> (f64, f64, f64) {
        let x_m = (self.a.x + self.b.x) / 2.0;
        let y_m = (self.a.y + self.b.y) / 2.0;
        let (dx, dy) = (self.b.x - self.a.x, self.b.y - self.a.y);
        let heading_rad = if dx == 0.0 && dy == 0.0 {
            0.0
        } else {
            // (dx, dy) rotated a quarter turn counterclockwise.
            dx.atan2(-dy)
        };
        (x_m, y_m, heading_rad)
    }
}

/// Whether a map was procedurally generated ([`MapSource::Random`], see
/// [`MapInfo::generation`]) or recorded from a track by
/// [`crate::localization::Slam`] ([`MapSource::Real`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MapSource {
    Random,
    Real,
}

/// How a [`MapSource::Random`] map was generated.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct GenerationInfo {
    pub track_width_m: f64,
    pub point_spacing_m: f64,
    /// RNG seed used to generate this map - re-running with the same seed
    /// and config reproduces it exactly.
    pub seed: u64,
}

/// The full contents of a map's `info.json`.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct MapInfo {
    pub resolution_m_per_px: f64,
    pub width_px: u32,
    pub height_px: u32,
    pub origin: ImageOrigin,
    pub start_finish_line: StartFinishLine,
    /// RFC3339 timestamp of when this map was generated or saved.
    pub generated_at: String,
    pub source: MapSource,
    /// How the map was generated - only for [`MapSource::Random`] maps.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub generation: Option<GenerationInfo>,
}

/// The current UTC time, formatted as RFC3339. Deliberately UTC (not local,
/// unlike [`crate::core`]'s logging) - map metadata has no reason to prefer
/// local time, and this sidesteps `OffsetDateTime::now_local`'s documented
/// multi-threaded failure mode.
pub fn now_rfc3339() -> String {
    OffsetDateTime::now_utc()
        .format(&Rfc3339)
        .expect("formatting the current time as RFC3339 never fails")
}

/// Error returned by [`write`].
#[derive(Debug)]
pub enum InfoWriteError {
    Io(std::io::Error),
    Json(serde_json::Error),
}

impl std::fmt::Display for InfoWriteError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(err) => write!(f, "failed to write info.json: {err}"),
            Self::Json(err) => write!(f, "failed to serialize info.json: {err}"),
        }
    }
}

impl std::error::Error for InfoWriteError {}

impl From<std::io::Error> for InfoWriteError {
    fn from(err: std::io::Error) -> Self {
        Self::Io(err)
    }
}

impl From<serde_json::Error> for InfoWriteError {
    fn from(err: serde_json::Error) -> Self {
        Self::Json(err)
    }
}

/// Writes `info` to `path` as pretty-printed JSON.
pub fn write(info: &MapInfo, path: &Path) -> Result<(), InfoWriteError> {
    let json = serde_json::to_string_pretty(info)?;
    std::fs::write(path, json)?;
    Ok(())
}

/// Error returned by [`read`].
#[derive(Debug)]
pub enum InfoReadError {
    Io(std::io::Error),
    Json(serde_json::Error),
}

impl std::fmt::Display for InfoReadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(err) => write!(f, "failed to read info.json: {err}"),
            Self::Json(err) => write!(f, "failed to parse info.json: {err}"),
        }
    }
}

impl std::error::Error for InfoReadError {}

impl From<std::io::Error> for InfoReadError {
    fn from(err: std::io::Error) -> Self {
        Self::Io(err)
    }
}

impl From<serde_json::Error> for InfoReadError {
    fn from(err: serde_json::Error) -> Self {
        Self::Json(err)
    }
}

/// Reads back `info.json` written by [`write`].
pub fn read(path: &Path) -> Result<MapInfo, InfoReadError> {
    let text = std::fs::read_to_string(path)?;
    Ok(serde_json::from_str(&text)?)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_info() -> MapInfo {
        MapInfo {
            resolution_m_per_px: 0.05,
            width_px: 100,
            height_px: 100,
            origin: ImageOrigin {
                x: -2.5,
                y: -2.5,
                theta_rad: 0.0,
            },
            start_finish_line: StartFinishLine {
                a: WorldPoint { x: 0.0, y: 1.0 },
                b: WorldPoint { x: 0.0, y: -1.0 },
            },
            generated_at: now_rfc3339(),
            source: MapSource::Random,
            generation: Some(GenerationInfo {
                track_width_m: 2.5,
                point_spacing_m: 0.25,
                seed: 42,
            }),
        }
    }

    #[test]
    fn written_json_round_trips_expected_fields() {
        let info = sample_info();
        let path =
            std::env::temp_dir().join(format!("aurorus_info_test_{}.json", std::process::id()));
        write(&info, &path).unwrap();

        let text = std::fs::read_to_string(&path).unwrap();
        std::fs::remove_file(&path).ok();
        let value: serde_json::Value = serde_json::from_str(&text).unwrap();

        assert_eq!(value["source"], "random");
        assert_eq!(value["generation"]["seed"], 42);
        assert_eq!(value["width_px"], 100);
        assert!(value["generated_at"].as_str().unwrap().contains('T'));
    }

    #[test]
    fn a_map_without_generation_info_omits_the_block() {
        let info = MapInfo {
            source: MapSource::Real,
            generation: None,
            ..sample_info()
        };
        let value = serde_json::to_value(&info).unwrap();
        assert!(value.get("generation").is_none());

        let read: MapInfo = serde_json::from_value(value).unwrap();
        assert!(read.generation.is_none());
    }

    #[test]
    fn the_start_pose_is_the_midpoint_heading_along_the_direction_of_travel() {
        // `a` on the left of a vehicle heading along +y.
        let line = StartFinishLine {
            a: WorldPoint { x: -1.0, y: 3.0 },
            b: WorldPoint { x: 1.0, y: 3.0 },
        };
        let (x, y, heading) = line.start_pose();
        assert_eq!((x, y), (0.0, 3.0));
        assert!((heading - std::f64::consts::FRAC_PI_2).abs() < 1e-12);

        let degenerate = StartFinishLine {
            a: WorldPoint { x: 2.0, y: 2.0 },
            b: WorldPoint { x: 2.0, y: 2.0 },
        };
        assert_eq!(degenerate.start_pose(), (2.0, 2.0, 0.0));
    }
}
