//! Where a benchmark's files go:
//!
//! ```text
//! <benchmarks root>/<map name>/<YYYY_MM_DD__HH_MM_SS>_<algorithm>_<model>/
//!     summary.toml       see BenchmarkSummary
//!     trajectory.csv     see TrajectoryRow
//!     map/info.json      copies of the map driven
//!     map/map.tiff
//!     race_line.csv      copy of the race line laps were timed against
//! ```
//!
//! Every run folder is self-contained, so it still replays after the map
//! it was driven on is regenerated, renamed or deleted.

use super::BenchmarkSummary;
use super::hash::sha256_file;
use crate::environment::{
    CENTERLINE_FILE_NAME, INFO_FILE_NAME, MAP_TIFF_FILE_NAME, RACE_LINES_DIR_NAME,
};
use std::path::{Path, PathBuf};
use time::OffsetDateTime;

pub const SUMMARY_FILE_NAME: &str = "summary.toml";
pub const TRAJECTORY_FILE_NAME: &str = "trajectory.csv";
/// The copy of the map's files, inside a run folder.
pub const MAP_DIR_NAME: &str = "map";
/// The copy of the race line, inside a run folder.
pub const RACE_LINE_FILE_NAME: &str = "race_line.csv";

/// Creates a new, empty run folder for a run started `at`:
/// `<root>/<map>/<YYYY_MM_DD__HH_MM_SS>_<algorithm>_<model>`, with a `_2`,
/// `_3`, ... suffix if that's already taken.
pub fn create_run_folder(
    root: &Path,
    map: &str,
    at: OffsetDateTime,
    algorithm: &str,
    model: &str,
) -> std::io::Result<PathBuf> {
    let parent = root.join(map);
    std::fs::create_dir_all(&parent)?;
    let stem = format!(
        "{:04}_{:02}_{:02}__{:02}_{:02}_{:02}_{algorithm}_{model}",
        at.year(),
        u8::from(at.month()),
        at.day(),
        at.hour(),
        at.minute(),
        at.second(),
    );
    for n in 1.. {
        let folder = match n {
            1 => parent.join(&stem),
            n => parent.join(format!("{stem}_{n}")),
        };
        match std::fs::create_dir(&folder) {
            Ok(()) => return Ok(folder),
            Err(err) if err.kind() == std::io::ErrorKind::AlreadyExists => continue,
            Err(err) => return Err(err),
        }
    }
    unreachable!("some suffix is free")
}

/// The SHA-256 of each file [`copy_track`] copied.
#[derive(Debug, Clone, PartialEq)]
pub struct TrackHashes {
    pub info_sha256: String,
    pub tiff_sha256: String,
    pub race_line_sha256: String,
}

/// Copies the map in `map_folder` (its `info.json` and `map.tiff`) and its
/// race line `race_line` (a file of its `race_lines/`) into `run_folder`,
/// and hashes the copies.
pub fn copy_track(
    map_folder: &Path,
    race_line: &str,
    run_folder: &Path,
) -> std::io::Result<TrackHashes> {
    let map_copy = run_folder.join(MAP_DIR_NAME);
    std::fs::create_dir_all(&map_copy)?;
    let copy = |from: PathBuf, to: PathBuf| -> std::io::Result<String> {
        std::fs::copy(&from, &to)?;
        sha256_file(&to)
    };
    Ok(TrackHashes {
        info_sha256: copy(
            map_folder.join(INFO_FILE_NAME),
            map_copy.join(INFO_FILE_NAME),
        )?,
        tiff_sha256: copy(
            map_folder.join(MAP_TIFF_FILE_NAME),
            map_copy.join(MAP_TIFF_FILE_NAME),
        )?,
        race_line_sha256: copy(
            map_folder.join(RACE_LINES_DIR_NAME).join(race_line),
            run_folder.join(RACE_LINE_FILE_NAME),
        )?,
    })
}

/// The race line an algorithm that follows none is timed against.
pub const TIMING_LINE_FILE_NAME: &str = CENTERLINE_FILE_NAME;

/// One run found by [`scan`].
#[derive(Debug, Clone, PartialEq)]
pub struct ScannedRun {
    pub folder: PathBuf,
    pub summary: BenchmarkSummary,
}

/// Every run under `root` (`<root>/*/*/summary.toml`), sorted by folder.
/// A summary that can't be read is left out, with a warning.
pub fn scan(root: &Path) -> Vec<ScannedRun> {
    let subfolders = |dir: &Path| -> Vec<PathBuf> {
        std::fs::read_dir(dir)
            .map(|entries| {
                entries
                    .flatten()
                    .map(|entry| entry.path())
                    .filter(|path| path.is_dir())
                    .collect()
            })
            .unwrap_or_default()
    };
    let mut runs: Vec<ScannedRun> = subfolders(root)
        .iter()
        .flat_map(|map| subfolders(map))
        .filter_map(|folder| {
            let path = folder.join(SUMMARY_FILE_NAME);
            if !path.is_file() {
                return None;
            }
            match BenchmarkSummary::read(&path) {
                Ok(summary) => Some(ScannedRun { folder, summary }),
                Err(err) => {
                    eprintln!("benchmarks: skipping {}: {err}", folder.display());
                    None
                }
            }
        })
        .collect();
    runs.sort_by(|a, b| a.folder.cmp(&b.folder));
    runs
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::benchmark::summary::tests::summary;

    fn temp_root(name: &str) -> PathBuf {
        let root =
            std::env::temp_dir().join(format!("aurorus_benchmark_{name}_{}", std::process::id()));
        std::fs::remove_dir_all(&root).ok();
        root
    }

    #[test]
    fn a_taken_run_folder_gets_a_suffix() {
        let root = temp_root("layout");
        let at = OffsetDateTime::UNIX_EPOCH;
        let first = create_run_folder(&root, "map", at, "algo", "bicycle").unwrap();
        let second = create_run_folder(&root, "map", at, "algo", "bicycle").unwrap();
        std::fs::remove_dir_all(&root).ok();
        assert_eq!(
            first,
            root.join("map").join("1970_01_01__00_00_00_algo_bicycle")
        );
        assert_eq!(
            second,
            root.join("map").join("1970_01_01__00_00_00_algo_bicycle_2")
        );
    }

    #[test]
    fn scan_finds_every_readable_run() {
        let root = temp_root("scan");
        let at = OffsetDateTime::UNIX_EPOCH;
        let a = create_run_folder(&root, "map_a", at, "algo", "bicycle").unwrap();
        let b = create_run_folder(&root, "map_b", at, "algo", "two_track").unwrap();
        let broken = create_run_folder(&root, "map_b", at, "other", "bicycle").unwrap();
        let empty = create_run_folder(&root, "map_b", at, "empty", "bicycle").unwrap();
        summary().write(&a.join(SUMMARY_FILE_NAME)).unwrap();
        summary().write(&b.join(SUMMARY_FILE_NAME)).unwrap();
        std::fs::write(broken.join(SUMMARY_FILE_NAME), "not = [toml").unwrap();

        let runs = scan(&root);
        std::fs::remove_dir_all(&root).ok();
        let folders: Vec<&PathBuf> = runs.iter().map(|run| &run.folder).collect();
        assert_eq!(folders, [&a, &b]);
        assert!(!folders.contains(&&empty));
        assert_eq!(runs[0].summary, summary());
    }
}
