//! Where a benchmark's files go:
//!
//! ```text
//! <benchmarks root>/<map name>/<YYYY_MM_DD__HH_MM_SS>_<algorithm>_<model>/
//!     summary.toml       see BenchmarkSummary
//!     trajectory.csv     see TrajectoryRow
//!     map/info.json      copies of the map driven
//!     map/map.tiff
//!     map/race_lines/centerline.csv
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
    /// `None` if the map has no centerline.
    pub centerline_sha256: Option<String>,
}

/// Copies the map in `map_folder` (its `info.json`, `map.tiff` and, if it
/// has one, its centerline - so `map/` is a map folder
/// [`crate::environment::Map::load`] reads as is) and its race line
/// `race_line` (a file of its `race_lines/`) into `run_folder`, and hashes
/// the copies.
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
        centerline_sha256: {
            let centerline = map_folder
                .join(RACE_LINES_DIR_NAME)
                .join(CENTERLINE_FILE_NAME);
            if centerline.is_file() {
                let lines_copy = map_copy.join(RACE_LINES_DIR_NAME);
                std::fs::create_dir_all(&lines_copy)?;
                Some(copy(centerline, lines_copy.join(CENTERLINE_FILE_NAME))?)
            } else {
                None
            }
        },
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

/// Every run folder under `root` (`<root>/*/*/` holding a `summary.toml`),
/// split into those whose summary reads and those whose doesn't.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ScanOutcome {
    /// Sorted by folder.
    pub runs: Vec<ScannedRun>,
    /// Each folder with why its summary can't be read, sorted by folder.
    pub unreadable: Vec<(PathBuf, String)>,
}

/// Every run under `root` (`<root>/*/*/summary.toml`), sorted by folder.
/// A summary that can't be read is left out, with a warning - see
/// [`scan_all`] to get those too.
pub fn scan(root: &Path) -> Vec<ScannedRun> {
    let outcome = scan_all(root);
    for (folder, err) in &outcome.unreadable {
        eprintln!("benchmarks: skipping {}: {err}", folder.display());
    }
    outcome.runs
}

/// Every run under `root`, readable or not - see [`ScanOutcome`].
pub fn scan_all(root: &Path) -> ScanOutcome {
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
    let mut outcome = ScanOutcome::default();
    for folder in subfolders(root).iter().flat_map(|map| subfolders(map)) {
        let path = folder.join(SUMMARY_FILE_NAME);
        if !path.is_file() {
            continue;
        }
        match BenchmarkSummary::read(&path) {
            Ok(summary) => outcome.runs.push(ScannedRun { folder, summary }),
            Err(err) => outcome.unreadable.push((folder, err)),
        }
    }
    outcome.runs.sort_by(|a, b| a.folder.cmp(&b.folder));
    outcome.unreadable.sort();
    outcome
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
    fn copy_track_makes_a_loadable_map_folder() {
        let root = temp_root("copy_track");
        let map = root.join("source");
        let lines = map.join(RACE_LINES_DIR_NAME);
        std::fs::create_dir_all(&lines).unwrap();
        std::fs::write(map.join(INFO_FILE_NAME), "{}").unwrap();
        std::fs::write(map.join(MAP_TIFF_FILE_NAME), "tiff").unwrap();
        std::fs::write(lines.join(CENTERLINE_FILE_NAME), "x,y,speed\n").unwrap();
        std::fs::write(lines.join("fast.csv"), "x,y,speed\n1,2,3\n").unwrap();
        let run = root.join("run");
        std::fs::create_dir_all(&run).unwrap();

        let hashes = copy_track(&map, "fast.csv", &run).unwrap();
        let copied = |path: PathBuf| std::fs::read_to_string(path).unwrap();
        assert_eq!(copied(run.join(RACE_LINE_FILE_NAME)), "x,y,speed\n1,2,3\n");
        assert_eq!(
            copied(
                run.join(MAP_DIR_NAME)
                    .join(RACE_LINES_DIR_NAME)
                    .join(CENTERLINE_FILE_NAME)
            ),
            "x,y,speed\n"
        );
        assert!(hashes.centerline_sha256.is_some());

        std::fs::remove_file(lines.join(CENTERLINE_FILE_NAME)).unwrap();
        let run = root.join("run_without_centerline");
        std::fs::create_dir_all(&run).unwrap();
        assert_eq!(
            copy_track(&map, "fast.csv", &run)
                .unwrap()
                .centerline_sha256,
            None
        );
        std::fs::remove_dir_all(&root).ok();
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
        let outcome = scan_all(&root);
        std::fs::remove_dir_all(&root).ok();
        assert_eq!(outcome.runs, runs);
        assert_eq!(
            outcome
                .unreadable
                .iter()
                .map(|(folder, _)| folder)
                .collect::<Vec<_>>(),
            [&broken]
        );
        let folders: Vec<&PathBuf> = runs.iter().map(|run| &run.folder).collect();
        assert_eq!(folders, [&a, &b]);
        assert!(!folders.contains(&&empty));
        assert_eq!(runs[0].summary, summary());
    }
}
