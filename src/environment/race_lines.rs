//! The catalog of a map folder's race lines: every `*.csv` in its
//! `race_lines/` folder (see [`super::race_line`] for the file format).
//!
//! A planned line is saved by [`save_new`] under the local date and time it
//! was written, `YYYY_MM_DD__HH_mm_ss.csv` - so planning again never
//! overwrites an earlier line - next to a `.json` sidecar of the same name
//! holding its [`RaceLineMeta`]: how it was computed and when. [`list`]
//! describes each line - method, lap length and time - newest first, and
//! [`default_line`] picks the one to follow when nobody chose one.
//!
//! Files without a sidecar still list: `centerline.csv` as the
//! [`RaceLineMethod::Centerline`], the pre-catalog `race_line.csv` and
//! `race_line_min_time.csv` by their names, anything else as
//! [`RaceLineMethod::Unknown`] - and they're dated by their modification
//! time.

use super::map::{CENTERLINE_FILE_NAME, RACE_LINES_DIR_NAME};
use super::race_line::{self, RaceLineReadError, RaceLineWriteError, SpeedPoint};
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};
use time::OffsetDateTime;

/// Name the minimum-curvature line was saved under before the catalog.
const LEGACY_MIN_CURVATURE_FILE_NAME: &str = "race_line.csv";
/// Name the minimum-time line was saved under before the catalog.
const LEGACY_MIN_TIME_FILE_NAME: &str = "race_line_min_time.csv";

/// How a race line was computed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RaceLineMethod {
    /// Not recorded - a CSV put there by hand, or no line at all.
    #[default]
    Unknown,
    /// The map's centerline: generated with a random map, or computed from
    /// the walls by [`crate::planning`].
    Centerline,
    /// [`crate::planning`]'s minimum-curvature optimization.
    MinCurvature,
    /// [`crate::planning`]'s minimum-time optimization.
    MinTime,
}

/// What a race line's `.json` sidecar holds.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RaceLineMeta {
    pub method: RaceLineMethod,
    /// When the line was saved, in milliseconds since the Unix epoch - what
    /// [`list`] orders by, since it survives copying the folder around,
    /// unlike a modification time.
    pub created_unix_ms: u64,
}

/// One race line of a map, as [`list`] describes it.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RaceLineEntry {
    /// File name inside `race_lines/`, e.g. `2026_09_25__15_30_12.csv`.
    pub file: String,
    pub method: RaceLineMethod,
    /// When it was saved, in milliseconds since the Unix epoch - see
    /// [`RaceLineMeta::created_unix_ms`].
    pub created_unix_ms: u64,
    pub num_points: usize,
    /// Length of one lap, in meters.
    pub lap_length_m: f64,
    /// Time of one lap at the line's speed profile, in seconds - see
    /// [`lap_time_s`]. Infinite if some point's speed is zero.
    pub lap_time_s: f64,
}

/// Error returned by [`read`].
#[derive(Debug)]
pub enum RaceLineFileError {
    /// Not a plain `*.csv` file name - names come from HTTP clients, so
    /// anything that could leave `race_lines/` is refused.
    InvalidName(String),
    Read(RaceLineReadError),
}

impl std::fmt::Display for RaceLineFileError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidName(name) => write!(f, "invalid race line file name {name:?}"),
            Self::Read(err) => write!(f, "{err}"),
        }
    }
}

impl std::error::Error for RaceLineFileError {}

/// Whether `file` is a plain `*.csv` name inside `race_lines/`.
fn is_valid_file_name(file: &str) -> bool {
    file.ends_with(".csv")
        && file.len() > ".csv".len()
        && !file.contains('/')
        && !file.contains('\\')
        && !file.starts_with('.')
}

fn race_lines_dir(map_folder: &Path) -> PathBuf {
    map_folder.join(RACE_LINES_DIR_NAME)
}

/// The sidecar of `race_lines/<file>`: the same name with `.json` in place
/// of `.csv`.
fn sidecar_path(map_folder: &Path, file: &str) -> PathBuf {
    race_lines_dir(map_folder).join(file).with_extension("json")
}

/// Reads `map_folder`'s `race_lines/<file>`.
pub fn read(map_folder: &Path, file: &str) -> Result<Vec<SpeedPoint>, RaceLineFileError> {
    if !is_valid_file_name(file) {
        return Err(RaceLineFileError::InvalidName(file.to_string()));
    }
    race_line::read(&race_lines_dir(map_folder).join(file)).map_err(RaceLineFileError::Read)
}

/// Saves `points` as a new line of `map_folder`, computed by `method`,
/// named after the current local date and time (`YYYY_MM_DD__HH_mm_ss.csv`,
/// UTC if the local offset can't be determined - see `crate::core::log`),
/// with its sidecar. A name already taken gets a `_2`, `_3`, ... suffix, so
/// two lines saved within the same second both keep theirs - and still
/// list in the order they were saved. Returns the file name.
pub fn save_new(
    map_folder: &Path,
    method: RaceLineMethod,
    points: &[SpeedPoint],
) -> Result<String, RaceLineWriteError> {
    let dir = race_lines_dir(map_folder);
    std::fs::create_dir_all(&dir)?;
    let now = OffsetDateTime::now_local().unwrap_or_else(|_| OffsetDateTime::now_utc());
    let stem = format!(
        "{:04}_{:02}_{:02}__{:02}_{:02}_{:02}",
        now.year(),
        u8::from(now.month()),
        now.day(),
        now.hour(),
        now.minute(),
        now.second(),
    );
    let file = (1..)
        .map(|n| match n {
            1 => format!("{stem}.csv"),
            n => format!("{stem}_{n}.csv"),
        })
        .find(|file| !dir.join(file).exists())
        .expect("some suffix is free");

    race_line::write(points, &dir.join(&file))?;
    let meta = RaceLineMeta {
        method,
        created_unix_ms: unix_ms(SystemTime::now()),
    };
    let json = serde_json::to_string_pretty(&meta).expect("RaceLineMeta always serializes");
    std::fs::write(sidecar_path(map_folder, &file), json)?;
    Ok(file)
}

fn unix_ms(time: SystemTime) -> u64 {
    time.duration_since(UNIX_EPOCH)
        .map(|d| u64::try_from(d.as_millis()).unwrap_or(u64::MAX))
        .unwrap_or(0)
}

/// `file`'s sidecar, or what can be told without one (see the module docs).
pub fn meta(map_folder: &Path, file: &str) -> RaceLineMeta {
    if let Some(meta) = std::fs::read_to_string(sidecar_path(map_folder, file))
        .ok()
        .and_then(|json| serde_json::from_str::<RaceLineMeta>(&json).ok())
    {
        return meta;
    }
    let method = match file {
        CENTERLINE_FILE_NAME => RaceLineMethod::Centerline,
        LEGACY_MIN_CURVATURE_FILE_NAME => RaceLineMethod::MinCurvature,
        LEGACY_MIN_TIME_FILE_NAME => RaceLineMethod::MinTime,
        _ => RaceLineMethod::Unknown,
    };
    let created_unix_ms = std::fs::metadata(race_lines_dir(map_folder).join(file))
        .and_then(|metadata| metadata.modified())
        .map(unix_ms)
        .unwrap_or(0);
    RaceLineMeta {
        method,
        created_unix_ms,
    }
}

/// Every readable race line of `map_folder`, newest first (ties broken by
/// name, later first). Empty if it has no `race_lines/` folder; a CSV that
/// can't be read is left out, with a warning.
pub fn list(map_folder: &Path) -> Vec<RaceLineEntry> {
    let Ok(dir) = std::fs::read_dir(race_lines_dir(map_folder)) else {
        return Vec::new();
    };
    let mut entries: Vec<RaceLineEntry> = dir
        .filter_map(|entry| entry.ok()?.file_name().into_string().ok())
        .filter(|file| is_valid_file_name(file))
        .filter_map(|file| {
            let points = read(map_folder, &file)
                .inspect_err(|err| eprintln!("race lines: skipping {file:?}: {err}"))
                .ok()?;
            let meta = meta(map_folder, &file);
            Some(RaceLineEntry {
                method: meta.method,
                created_unix_ms: meta.created_unix_ms,
                num_points: points.len(),
                lap_length_m: lap_length_m(&points),
                lap_time_s: lap_time_s(&points),
                file,
            })
        })
        .collect();
    entries.sort_by(|a, b| {
        b.created_unix_ms
            .cmp(&a.created_unix_ms)
            .then_with(|| b.file.cmp(&a.file))
    });
    entries
}

/// The line to follow when nobody chose one, out of `entries` as [`list`]
/// returns them: the newest planned line, else the centerline.
pub fn default_line(entries: &[RaceLineEntry]) -> Option<&RaceLineEntry> {
    entries
        .iter()
        .find(|entry| entry.method != RaceLineMethod::Centerline)
        .or_else(|| entries.first())
}

/// Length of one lap around the closed line `points`, in meters.
pub fn lap_length_m(points: &[SpeedPoint]) -> f64 {
    let n = points.len();
    if n < 2 {
        return 0.0;
    }
    (0..n)
        .map(|i| segment_length_m(&points[i], &points[(i + 1) % n]))
        .sum()
}

/// Time of one lap around the closed line `points` at their speeds, in
/// seconds: every segment at the mean of its two ends' speeds, as
/// `crate::planning`'s speed profile does. Infinite if a segment would be
/// driven at zero speed.
pub fn lap_time_s(points: &[SpeedPoint]) -> f64 {
    let n = points.len();
    if n < 2 {
        return 0.0;
    }
    (0..n)
        .map(|i| {
            let (a, b) = (&points[i], &points[(i + 1) % n]);
            let mean = (a.speed_mps + b.speed_mps) / 2.0;
            if mean > 0.0 {
                segment_length_m(a, b) / mean
            } else {
                f64::INFINITY
            }
        })
        .sum()
}

fn segment_length_m(a: &SpeedPoint, b: &SpeedPoint) -> f64 {
    (b.x - a.x).hypot(b.y - a.y)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_map(tag: &str) -> PathBuf {
        let folder =
            std::env::temp_dir().join(format!("aurorus_race_lines_{tag}_{}", std::process::id()));
        std::fs::remove_dir_all(&folder).ok();
        folder
    }

    /// A square of side 1 m driven at 2 m/s.
    fn square() -> Vec<SpeedPoint> {
        [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
            .map(|(x, y)| SpeedPoint {
                x,
                y,
                speed_mps: 2.0,
            })
            .to_vec()
    }

    #[test]
    fn lap_length_and_time_close_the_loop() {
        assert!((lap_length_m(&square()) - 4.0).abs() < 1e-12);
        assert!((lap_time_s(&square()) - 2.0).abs() < 1e-12);
    }

    #[test]
    fn saved_lines_list_newest_first_with_their_method() {
        let folder = temp_map("list");
        race_line::write(&square(), &{
            std::fs::create_dir_all(race_lines_dir(&folder)).unwrap();
            race_lines_dir(&folder).join(CENTERLINE_FILE_NAME)
        })
        .unwrap();
        let first = save_new(&folder, RaceLineMethod::MinCurvature, &square()).unwrap();
        let second = save_new(&folder, RaceLineMethod::MinTime, &square()).unwrap();

        let entries = list(&folder);
        std::fs::remove_dir_all(&folder).ok();

        assert_ne!(first, second, "same-second saves must not collide");
        let listed: Vec<_> = entries
            .iter()
            .map(|e| (e.file.as_str(), e.method))
            .collect();
        assert_eq!(listed[0], (second.as_str(), RaceLineMethod::MinTime));
        assert_eq!(listed[1], (first.as_str(), RaceLineMethod::MinCurvature));
        assert!(listed.contains(&(CENTERLINE_FILE_NAME, RaceLineMethod::Centerline)));
        assert_eq!(default_line(&entries).unwrap().file, second);
        assert_eq!(entries[0].num_points, 4);
    }

    #[test]
    fn the_centerline_is_the_default_only_when_alone() {
        let folder = temp_map("centerline");
        std::fs::create_dir_all(race_lines_dir(&folder)).unwrap();
        race_line::write(
            &square(),
            &race_lines_dir(&folder).join(CENTERLINE_FILE_NAME),
        )
        .unwrap();

        let entries = list(&folder);
        std::fs::remove_dir_all(&folder).ok();

        assert_eq!(default_line(&entries).unwrap().file, CENTERLINE_FILE_NAME);
        assert!(default_line(&[]).is_none());
    }

    #[test]
    fn names_that_could_leave_the_folder_are_refused() {
        for name in ["../x.csv", "a/b.csv", ".csv", "x.json", "..\\x.csv"] {
            assert!(!is_valid_file_name(name), "{name}");
        }
        assert!(is_valid_file_name("2026_09_25__15_30_12.csv"));
    }
}
