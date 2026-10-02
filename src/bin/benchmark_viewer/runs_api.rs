//! The viewer's API: every benchmark run under the root, and each one's
//! trajectory, map and lines - read straight off its self-contained run
//! folder (see `aurorus::benchmark::layout`), nothing cached.
//!
//! A run is named by its id: its folder relative to the root,
//! `<map>/<run folder>`. An id is only ever resolved by [`run_folder`],
//! which refuses anything but two plain folder names, so no request can
//! reach outside the root.

use aurorus::benchmark::layout::{
    MAP_DIR_NAME, RACE_LINE_FILE_NAME, SUMMARY_FILE_NAME, TRAJECTORY_FILE_NAME,
};
use aurorus::benchmark::{BenchmarkSummary, read_trajectory, scan_all};
use aurorus::environment::{CENTERLINE_FILE_NAME, Map, RACE_LINES_DIR_NAME, race_line, read_info};
use aurorus::web::{error_response, header, json_response, not_found};
use std::path::{Component, Path, PathBuf};
use tiny_http::{Response, ResponseBox};

#[derive(serde::Serialize)]
struct Run {
    id: String,
    summary: BenchmarkSummary,
}

#[derive(serde::Serialize)]
struct Unreadable {
    id: String,
    error: String,
}

#[derive(serde::Serialize)]
struct Runs {
    root: PathBuf,
    runs: Vec<Run>,
    /// Run folders whose summary can't be read.
    unreadable: Vec<Unreadable>,
}

/// A run's id: its folder relative to `root`, with `/` separators.
fn id_of(root: &Path, folder: &Path) -> String {
    folder
        .strip_prefix(root)
        .unwrap_or(folder)
        .components()
        .map(|component| component.as_os_str().to_string_lossy())
        .collect::<Vec<_>>()
        .join("/")
}

/// The folder of the run `id` under `root`, if `id` is two plain folder
/// names (`<map>/<run>`) naming a folder with a summary.
fn run_folder(root: &Path, id: &str) -> Option<PathBuf> {
    let parts: Vec<&str> = id.split('/').collect();
    let plain = |part: &&str| {
        let mut components = Path::new(part).components();
        matches!(components.next(), Some(Component::Normal(_))) && components.next().is_none()
    };
    if parts.len() != 2 || !parts.iter().all(plain) {
        return None;
    }
    let folder = root.join(parts[0]).join(parts[1]);
    folder.join(SUMMARY_FILE_NAME).is_file().then_some(folder)
}

/// `GET /api/runs` - every run under `root`, rescanned on every call.
pub fn list(root: &Path) -> ResponseBox {
    let outcome = scan_all(root);
    json_response(
        &Runs {
            root: root.to_path_buf(),
            runs: outcome
                .runs
                .into_iter()
                .map(|run| Run {
                    id: id_of(root, &run.folder),
                    summary: run.summary,
                })
                .collect(),
            unreadable: outcome
                .unreadable
                .into_iter()
                .map(|(folder, error)| Unreadable {
                    id: id_of(root, &folder),
                    error,
                })
                .collect(),
        },
        200,
    )
}

/// Routes `GET /api/runs/<map>/<run>/<what>`.
pub fn route(root: &Path, rest: &str) -> ResponseBox {
    let Some((id, what)) = rest.rsplit_once('/') else {
        return not_found();
    };
    let Some(folder) = run_folder(root, id) else {
        return not_found();
    };
    match what {
        "trajectory" => trajectory(&folder),
        "map_info" => map_info(&folder),
        "map_raster" => map_raster(&folder),
        "lines" => lines(&folder),
        _ => not_found(),
    }
}

/// A trajectory column by column - about half the size of one object per
/// row. The localization estimate's columns are left out when it never ran.
#[derive(Debug, Default, PartialEq, serde::Serialize)]
struct Columns {
    t_s: Vec<f64>,
    lap: Vec<u32>,
    x_m: Vec<f64>,
    y_m: Vec<f64>,
    heading_rad: Vec<f64>,
    speed_mps: Vec<f64>,
    steering_cmd_rad: Vec<f64>,
    speed_cmd_mps: Vec<f64>,
    s_m: Vec<Option<f64>>,
    lateral_m: Vec<Option<f64>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    est_x_m: Option<Vec<Option<f64>>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    est_y_m: Option<Vec<Option<f64>>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    est_heading_rad: Option<Vec<Option<f64>>>,
}

fn columns(rows: &[aurorus::benchmark::TrajectoryRow]) -> Columns {
    let mut columns = Columns::default();
    for row in rows {
        columns.t_s.push(row.t_s);
        columns.lap.push(row.lap);
        columns.x_m.push(row.x_m);
        columns.y_m.push(row.y_m);
        columns.heading_rad.push(row.heading_rad);
        columns.speed_mps.push(row.speed_mps);
        columns.steering_cmd_rad.push(row.steering_cmd_rad);
        columns.speed_cmd_mps.push(row.speed_cmd_mps);
        columns.s_m.push(row.s_m);
        columns.lateral_m.push(row.lateral_m);
    }
    if rows.iter().any(|row| row.est_x_m.is_some()) {
        columns.est_x_m = Some(rows.iter().map(|row| row.est_x_m).collect());
        columns.est_y_m = Some(rows.iter().map(|row| row.est_y_m).collect());
        columns.est_heading_rad = Some(rows.iter().map(|row| row.est_heading_rad).collect());
    }
    columns
}

/// `GET /api/runs/<id>/trajectory` - the run's trajectory, as [`Columns`].
fn trajectory(folder: &Path) -> ResponseBox {
    match read_trajectory(&folder.join(TRAJECTORY_FILE_NAME)) {
        Ok(rows) => json_response(&columns(&rows), 200),
        Err(err) => error_response(500, &format!("couldn't read the trajectory: {err}")),
    }
}

/// `GET /api/runs/<id>/map_info` - the copied map's `MapInfo`.
fn map_info(folder: &Path) -> ResponseBox {
    match read_info(&folder.join(MAP_DIR_NAME)) {
        Ok(info) => json_response(&info, 200),
        Err(err) => error_response(500, &format!("couldn't read the map: {err}")),
    }
}

/// `GET /api/runs/<id>/map_raster` - the copied map's occupancy grid as raw
/// bytes, one per pixel (`0` or `255`), row-major - as `web_gui` serves a
/// map's.
fn map_raster(folder: &Path) -> ResponseBox {
    match Map::load(&folder.join(MAP_DIR_NAME)) {
        Ok(map) => Response::from_data(map.raster.to_bytes())
            .with_header(header("Content-Type", "application/octet-stream"))
            .boxed(),
        Err(err) => error_response(500, &format!("couldn't read the map: {err}")),
    }
}

#[derive(serde::Serialize)]
struct Lines {
    /// `[x, y, speed]` per point, closed.
    race_line: Vec<[f64; 3]>,
    /// The map's centerline - `None` for a run that predates copying it.
    centerline: Option<Vec<[f64; 3]>>,
}

fn points(path: &Path) -> Result<Vec<[f64; 3]>, String> {
    race_line::read(path)
        .map(|points| {
            points
                .iter()
                .map(|point| [point.x, point.y, point.speed_mps])
                .collect()
        })
        .map_err(|err| err.to_string())
}

/// `GET /api/runs/<id>/lines` - the race line driven and the map's
/// centerline, as [`Lines`].
fn lines(folder: &Path) -> ResponseBox {
    let race_line = match points(&folder.join(RACE_LINE_FILE_NAME)) {
        Ok(points) => points,
        Err(err) => return error_response(500, &format!("couldn't read the race line: {err}")),
    };
    let centerline_path = folder
        .join(MAP_DIR_NAME)
        .join(RACE_LINES_DIR_NAME)
        .join(CENTERLINE_FILE_NAME);
    let centerline = centerline_path
        .is_file()
        .then(|| points(&centerline_path).ok())
        .flatten();
    json_response(
        &Lines {
            race_line,
            centerline,
        },
        200,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use aurorus::benchmark::layout::create_run_folder;
    use aurorus::benchmark::{
        AlgorithmRecord, CodeVersion, FORMAT_VERSION, MapRecord, RaceLineRecord, RunStatus,
        TrajectoryRow, TrajectoryWriter, VehicleRecord,
    };
    use aurorus::environment::RaceLineMethod;
    use aurorus::telemetry::TelemetryPoseSource;
    use std::collections::BTreeMap;
    use time::OffsetDateTime;

    fn summary() -> BenchmarkSummary {
        BenchmarkSummary {
            format_version: FORMAT_VERSION,
            status: RunStatus::Completed,
            started_at: "2026-09-29T14:03:12+02:00".to_string(),
            countdown_s: 3.0,
            laps_requested: 1,
            pose_source: TelemetryPoseSource::GroundTruth,
            code: CodeVersion::default(),
            map: MapRecord {
                name: "map".to_string(),
                info_sha256: "aa".to_string(),
                tiff_sha256: "bb".to_string(),
                centerline_sha256: None,
                centerline_length_m: 10.0,
                lap_timeout_s: 10.0,
            },
            race_line: RaceLineRecord {
                file: "centerline.csv".to_string(),
                method: RaceLineMethod::Centerline,
                used_by_algorithm: false,
                sha256: "cc".to_string(),
                length_m: 10.0,
            },
            algorithm: AlgorithmRecord {
                name: "algo".to_string(),
                saved_to_config: true,
                parameters: BTreeMap::new(),
            },
            vehicle: VehicleRecord {
                model: "bicycle".to_string(),
                body_length_m: 0.45,
                body_width_m: 0.25,
                saved_to_config: true,
                parameters: BTreeMap::new(),
                limits: BTreeMap::new(),
                geometry: None,
            },
            results: None,
            laps: Vec::new(),
        }
    }

    fn row(t_s: f64, estimate: Option<f64>) -> TrajectoryRow {
        TrajectoryRow {
            t_s,
            lap: 1,
            x_m: t_s,
            y_m: 0.0,
            heading_rad: 0.0,
            speed_mps: 1.0,
            steering_cmd_rad: 0.0,
            speed_cmd_mps: 1.0,
            s_m: Some(t_s),
            lateral_m: None,
            est_x_m: estimate,
            est_y_m: estimate,
            est_heading_rad: estimate,
        }
    }

    #[test]
    fn only_two_plain_folder_names_resolve_to_a_run() {
        let root = std::env::temp_dir().join(format!(
            "aurorus_benchmark_viewer_runs_{}",
            std::process::id()
        ));
        std::fs::remove_dir_all(&root).ok();
        let folder =
            create_run_folder(&root, "map", OffsetDateTime::UNIX_EPOCH, "algo", "bicycle").unwrap();
        summary().write(&folder.join(SUMMARY_FILE_NAME)).unwrap();
        let empty =
            create_run_folder(&root, "map", OffsetDateTime::UNIX_EPOCH, "none", "bicycle").unwrap();

        let id = id_of(&root, &folder);
        assert_eq!(id, "map/1970_01_01__00_00_00_algo_bicycle");
        assert_eq!(run_folder(&root, &id), Some(folder));
        assert_eq!(run_folder(&root, &id_of(&root, &empty)), None);
        for bad in [
            "map",
            "map/../map/x",
            "../map/x",
            "/etc/passwd",
            "map/./x",
            "map//x",
        ] {
            assert_eq!(run_folder(&root, bad), None, "{bad}");
        }
        std::fs::remove_dir_all(&root).ok();
    }

    #[test]
    fn estimate_columns_only_appear_when_localization_ran() {
        let without = columns(&[row(0.0, None), row(0.025, None)]);
        assert_eq!(without.t_s, [0.0, 0.025]);
        assert_eq!(without.est_x_m, None);
        let json = serde_json::to_value(&without).unwrap();
        assert!(json.get("est_x_m").is_none());

        let with = columns(&[row(0.0, None), row(0.025, Some(1.0))]);
        assert_eq!(with.est_x_m, Some(vec![None, Some(1.0)]));
    }

    #[test]
    fn a_written_trajectory_reads_back_as_columns() {
        let path = std::env::temp_dir().join(format!(
            "aurorus_benchmark_viewer_trajectory_{}.csv",
            std::process::id()
        ));
        let mut writer = TrajectoryWriter::create(&path).unwrap();
        writer.write(&row(0.0, None)).unwrap();
        writer.write(&row(0.025, None)).unwrap();
        writer.flush().unwrap();
        let rows = read_trajectory(&path).unwrap();
        std::fs::remove_file(&path).ok();
        assert_eq!(columns(&rows).x_m, [0.0, 0.025]);
    }
}
