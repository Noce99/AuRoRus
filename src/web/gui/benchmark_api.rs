//! The Benchmark panel: an autonomous algorithm driving the ego vehicle
//! alone for [`WebGuiConfig::benchmark_laps`] laps, on every combination of
//! the maps, vehicle models and race lines picked, each run recorded under
//! the benchmarks folder (see [`crate::benchmark`]).
//!
//! The runs are driven by [`run_orchestrator`](super::benchmark_orchestrator::run_orchestrator), a thread of [`super::WebGui`]
//! writing the same selection topics the other panels do, as `WebGui`. Each
//! run starts with a soft reset (opponents removed, any debug recording
//! stopped, the map, model and race line selected), lines the vehicle up
//! like a race start with a countdown, and ends after the laps, when a lap
//! times out, or when aborted (the Abort button, any WASD key, a shutdown).
//! While a benchmark runs, [`blocks`] tells [`super::handlers`] to refuse
//! every request that would change the setup.

use super::benchmark_orchestrator::{algorithm_status, lap_timeout_s};
use super::{BenchmarkSetup, WebGuiConfig};
use crate::Captain;
use crate::benchmark::RunStatus;
use crate::benchmark::layout::TIMING_LINE_FILE_NAME;
use crate::environment::{CENTERLINE_FILE_NAME, map_folder, race_lines, read_info};
use crate::topics::{
    VEHICLE_MODEL_STATUS_TOPIC_NAME, VehicleModelKind, VehicleModelStatus, now_ms,
};
use crate::web::{bad_request, error_response, json_response, read_json};
use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use tiny_http::{Method, Request, ResponseBox};

/// The routes still served while a benchmark runs: aborting it, drawing the
/// map (a POST, but read-only), and the WASD command - which aborts it.
const ALLOWED_WHILE_RUNNING: &[&str] = &[
    "/api/benchmark/abort",
    "/api/draw",
    "/api/human_vesc_command",
];

/// One run of a benchmark: where, on which model, against which line.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct RunSpec {
    pub(super) map: String,
    pub(super) model: VehicleModelKind,
    /// File inside the map's `race_lines/` - the centerline when the
    /// algorithm follows no line.
    pub(super) race_line: String,
}

/// A benchmark: every run, in order.
#[derive(Debug, Clone)]
pub(super) struct Plan {
    pub(super) algorithm: String,
    /// Whether the algorithm follows the race line.
    pub(super) uses_race_line: bool,
    pub(super) runs: Vec<RunSpec>,
}

/// What a run is doing, for the progress display.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub(super) enum Stage {
    /// The soft reset: selecting the map, model and line.
    Preparing,
    /// On the grid, waiting for the "Go!".
    Countdown,
    Driving,
}

/// The run in progress.
#[derive(Debug, Clone, serde::Serialize)]
pub(super) struct Progress {
    /// Counted from `1`.
    pub(super) run: usize,
    pub(super) map: String,
    pub(super) model: String,
    pub(super) race_line: String,
    pub(super) stage: Stage,
    /// The lap in progress, counted from `1` - `0` before the first line
    /// crossing.
    pub(super) lap: u32,
    /// When the vehicle is released, in [`now_ms`]'s clock.
    pub(super) go_at_ms: Option<u64>,
    pub(super) lap_timeout_s: Option<f64>,
    /// Seconds into the lap in progress, by the benchmark's own clock (the
    /// one the timeout is measured with).
    pub(super) lap_elapsed_s: f64,
    /// Seconds since the "Go!".
    pub(super) elapsed_s: f64,
}

/// How one run went.
#[derive(Debug, Clone, serde::Serialize)]
pub(super) struct RunOutcome {
    pub(super) map: String,
    pub(super) model: String,
    pub(super) race_line: String,
    /// `None` when the run couldn't start - see `error`.
    pub(super) status: Option<RunStatus>,
    pub(super) error: Option<String>,
    /// The run folder, if one was written.
    pub(super) folder: Option<PathBuf>,
    pub(super) laps: u32,
    pub(super) total_time_s: Option<f64>,
    pub(super) best_lap_s: Option<f64>,
}

#[derive(Debug, Default)]
pub(super) struct State {
    /// A benchmark requested but not picked up by the orchestrator yet.
    pub(super) pending: Option<Plan>,
    /// The benchmark running, or the last one.
    pub(super) plan: Option<Plan>,
    pub(super) running: bool,
    pub(super) abort: bool,
    pub(super) progress: Option<Progress>,
    /// Every run of `plan` finished so far.
    pub(super) outcomes: Vec<RunOutcome>,
}

/// The benchmark shared by `WebGui`'s request handlers and its orchestrator
/// thread. Cheap to clone - every clone is the same benchmark - and carried
/// across restarts (see [`super::WebGui`]'s `fresh`), so the last results
/// still show.
#[derive(Debug, Clone, Default)]
pub struct Benchmark(pub(super) Arc<Mutex<State>>);

impl Benchmark {
    pub(super) fn state(&self) -> std::sync::MutexGuard<'_, State> {
        self.0.lock().unwrap()
    }

    pub(super) fn aborted(&self) -> bool {
        self.state().abort
    }

    pub(super) fn set_progress(&self, update: impl FnOnce(&mut Progress)) {
        if let Some(progress) = &mut self.state().progress {
            update(progress);
        }
    }
}

/// Whether `method` on `path` is refused because a benchmark is running.
pub fn blocks(benchmark: &Benchmark, method: &Method, path: &str) -> bool {
    *method == Method::Post && !ALLOWED_WHILE_RUNNING.contains(&path) && benchmark.state().running
}

/// The response to a request [`blocks`] refused.
pub fn refused() -> ResponseBox {
    error_response(409, "A benchmark is running - abort it first.")
}

// ---------------------------------------------------------------------
// Endpoints
// ---------------------------------------------------------------------

#[derive(serde::Serialize)]
struct AlgorithmOption {
    name: String,
    label: String,
    uses_race_line: bool,
}

#[derive(serde::Serialize)]
struct RaceLineOption {
    file: String,
    method: crate::environment::RaceLineMethod,
    lap_length_m: f64,
}

#[derive(serde::Serialize)]
struct MapOption {
    name: String,
    /// Newest first, as the Race Lines panel lists them.
    race_lines: Vec<RaceLineOption>,
    /// Longest a lap may take on this map, in seconds - `None` without a
    /// centerline, when it can't be benchmarked.
    lap_timeout_s: Option<f64>,
}

#[derive(serde::Serialize)]
struct ModelOption {
    kind: &'static str,
    label: &'static str,
}

#[derive(serde::Serialize)]
struct Options {
    algorithms: Vec<AlgorithmOption>,
    /// The algorithm picked in the Autonomous Algos panel.
    selected_algorithm: Option<String>,
    maps: Vec<MapOption>,
    models: Vec<ModelOption>,
    /// The vehicle model running now.
    current_model: &'static str,
    laps: u32,
    countdown_s: f64,
}

/// `GET /api/benchmark/options` - everything the panel offers: the
/// algorithms, every map with its race lines, and the vehicle models.
pub fn options(captain: &Captain, maps_root: &Path, config: &WebGuiConfig) -> ResponseBox {
    let status = algorithm_status(captain);
    let algorithms = status
        .available
        .iter()
        .map(|algorithm| AlgorithmOption {
            name: algorithm.name.clone(),
            label: algorithm.label.clone(),
            uses_race_line: algorithm.requires.race_line,
        })
        .collect();

    let mut maps: Vec<MapOption> = std::fs::read_dir(maps_root)
        .map(|entries| {
            entries
                .flatten()
                .filter_map(|entry| {
                    let folder = entry.path();
                    let name = folder.file_name()?.to_str()?.to_string();
                    read_info(&folder).ok()?;
                    let lines = race_lines::list(&folder);
                    let lap_timeout_s = lines
                        .iter()
                        .find(|line| line.file == CENTERLINE_FILE_NAME)
                        .map(|line| lap_timeout_s(line.lap_length_m, config));
                    Some(MapOption {
                        name,
                        race_lines: lines
                            .into_iter()
                            .map(|line| RaceLineOption {
                                file: line.file,
                                method: line.method,
                                lap_length_m: line.lap_length_m,
                            })
                            .collect(),
                        lap_timeout_s,
                    })
                })
                .collect()
        })
        .unwrap_or_default();
    maps.sort_by(|a, b| a.name.cmp(&b.name));

    let current_model = captain
        .try_topic::<VehicleModelStatus>(VEHICLE_MODEL_STATUS_TOPIC_NAME)
        .map(|topic| topic.read().kind)
        .unwrap_or_default()
        .api_str();
    json_response(
        &Options {
            algorithms,
            selected_algorithm: status.selected,
            maps,
            models: VehicleModelKind::ALL
                .iter()
                .map(|(_, kind, label, _)| ModelOption { kind, label })
                .collect(),
            current_model,
            laps: config.benchmark_laps,
            countdown_s: config.race_countdown_ms as f64 / 1000.0,
        },
        200,
    )
}

#[derive(serde::Deserialize)]
struct StartBody {
    algorithm: String,
    maps: Vec<String>,
    /// Vehicle model API strings.
    models: Vec<String>,
    /// Map name -> the race line files picked on it. Ignored for an
    /// algorithm that follows no race line.
    #[serde(default)]
    race_lines: HashMap<String, Vec<String>>,
}

#[derive(serde::Serialize)]
struct Started {
    runs: usize,
}

/// `POST /api/benchmark/start` - body `{"algorithm": "...", "maps": [...],
/// "models": [...], "race_lines": {"<map>": ["<file>", ...]}}` - checks the
/// picks and queues every combination for [`run_orchestrator`](super::benchmark_orchestrator::run_orchestrator).
pub fn start(
    request: &mut Request,
    captain: &Captain,
    maps_root: &Path,
    benchmark: &Benchmark,
) -> ResponseBox {
    let body: StartBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let Some(algorithm) = algorithm_status(captain)
        .available
        .into_iter()
        .find(|algorithm| algorithm.name == body.algorithm)
    else {
        return bad_request(&format!("Unknown algorithm {:?}.", body.algorithm));
    };
    let mut models = Vec::new();
    for model in &body.models {
        match VehicleModelKind::from_api_str(model) {
            Some(kind) => models.push(kind),
            None => return bad_request(&format!("Unknown vehicle model {model:?}.")),
        }
    }
    let mut available = BTreeMap::new();
    for map in &body.maps {
        let Some(folder) = map_folder(maps_root, map).filter(|folder| read_info(folder).is_ok())
        else {
            return bad_request(&format!("There's no map {map:?}."));
        };
        let files: Vec<String> = race_lines::list(&folder)
            .into_iter()
            .map(|line| line.file)
            .collect();
        available.insert(map.clone(), files);
    }
    let runs = match expand(
        &body.maps,
        &models,
        &body.race_lines,
        &available,
        algorithm.requires.race_line,
    ) {
        Ok(runs) => runs,
        Err(err) => return bad_request(&err),
    };

    let mut state = benchmark.state();
    if state.running {
        return refused();
    }
    let count = runs.len();
    let plan = Plan {
        algorithm: algorithm.name,
        uses_race_line: algorithm.requires.race_line,
        runs,
    };
    state.pending = Some(plan.clone());
    state.plan = Some(plan);
    state.running = true;
    state.abort = false;
    state.progress = None;
    state.outcomes.clear();
    json_response(&Started { runs: count }, 200)
}

/// Every run of a benchmark, maps outermost so each map is loaded once:
/// every picked model on every picked line of every picked map - or, for an
/// algorithm that follows no line (`uses_race_line` false), every model on
/// every map's centerline, only to time the laps against. `available` is
/// each map's race line files.
fn expand(
    maps: &[String],
    models: &[VehicleModelKind],
    picked_lines: &HashMap<String, Vec<String>>,
    available: &BTreeMap<String, Vec<String>>,
    uses_race_line: bool,
) -> Result<Vec<RunSpec>, String> {
    if maps.is_empty() {
        return Err("Pick at least one map.".to_string());
    }
    if models.is_empty() {
        return Err("Pick at least one vehicle model.".to_string());
    }
    let mut runs = Vec::new();
    for map in maps {
        let files = available.get(map).map(Vec::as_slice).unwrap_or_default();
        if !files.iter().any(|file| file == TIMING_LINE_FILE_NAME) {
            return Err(format!(
                "The map {map:?} has no centerline - laps can't be timed on it."
            ));
        }
        let lines: Vec<String> = if uses_race_line {
            let picked = picked_lines.get(map).map(Vec::as_slice).unwrap_or_default();
            if let Some(unknown) = picked.iter().find(|file| !files.contains(file)) {
                return Err(format!("The map {map:?} has no race line {unknown:?}."));
            }
            picked.to_vec()
        } else {
            vec![TIMING_LINE_FILE_NAME.to_string()]
        };
        for &model in models {
            for race_line in &lines {
                runs.push(RunSpec {
                    map: map.clone(),
                    model,
                    race_line: race_line.clone(),
                });
            }
        }
    }
    if runs.is_empty() {
        return Err("Pick at least one race line.".to_string());
    }
    Ok(runs)
}

/// `POST /api/benchmark/abort` - stops the benchmark: the run in progress
/// ends as aborted, and no other one starts.
pub fn abort(benchmark: &Benchmark) -> ResponseBox {
    let mut state = benchmark.state();
    if !state.running {
        return bad_request("No benchmark is running.");
    }
    state.abort = true;
    json_response(&(), 200)
}

#[derive(serde::Serialize)]
struct Status<'a> {
    running: bool,
    aborting: bool,
    algorithm: Option<&'a str>,
    runs: usize,
    laps: u32,
    progress: Option<&'a Progress>,
    /// Milliseconds until the "Go!" of the run in progress, if it's still
    /// to come - by the server's clock, so the page's countdown doesn't
    /// depend on its own.
    go_in_ms: Option<u64>,
    outcomes: &'a [RunOutcome],
    root: &'a Path,
}

/// `GET /api/benchmark` - the benchmark running (or the last one): its
/// progress and every run finished so far.
pub fn status(benchmark: &Benchmark, setup: &BenchmarkSetup, config: &WebGuiConfig) -> ResponseBox {
    let state = benchmark.state();
    let now = now_ms();
    json_response(
        &Status {
            running: state.running,
            aborting: state.abort,
            algorithm: state.plan.as_ref().map(|plan| plan.algorithm.as_str()),
            runs: state.plan.as_ref().map_or(0, |plan| plan.runs.len()),
            laps: config.benchmark_laps,
            progress: state.progress.as_ref(),
            go_in_ms: state
                .progress
                .as_ref()
                .and_then(|progress| progress.go_at_ms)
                .filter(|&go_at_ms| go_at_ms > now)
                .map(|go_at_ms| go_at_ms - now),
            outcomes: &state.outcomes,
            root: &setup.root,
        },
        200,
    )
}

// ---------------------------------------------------------------------
// The orchestrator
// ---------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn available() -> BTreeMap<String, Vec<String>> {
        BTreeMap::from([
            (
                "a".to_string(),
                vec!["centerline.csv".to_string(), "fast.csv".to_string()],
            ),
            ("b".to_string(), vec!["centerline.csv".to_string()]),
            ("no_centerline".to_string(), vec!["fast.csv".to_string()]),
        ])
    }

    fn maps(names: &[&str]) -> Vec<String> {
        names.iter().map(|name| name.to_string()).collect()
    }

    #[test]
    fn a_race_line_algorithm_runs_every_picked_line_on_every_model() {
        let picked = HashMap::from([(
            "a".to_string(),
            vec!["fast.csv".to_string(), "centerline.csv".to_string()],
        )]);
        let models = [VehicleModelKind::Bicycle, VehicleModelKind::TwoTrack];
        let runs = expand(&maps(&["a", "b"]), &models, &picked, &available(), true).unwrap();
        let described: Vec<(&str, VehicleModelKind, &str)> = runs
            .iter()
            .map(|run| (run.map.as_str(), run.model, run.race_line.as_str()))
            .collect();
        assert_eq!(
            described,
            [
                ("a", VehicleModelKind::Bicycle, "fast.csv"),
                ("a", VehicleModelKind::Bicycle, "centerline.csv"),
                ("a", VehicleModelKind::TwoTrack, "fast.csv"),
                ("a", VehicleModelKind::TwoTrack, "centerline.csv"),
            ]
        );
    }

    #[test]
    fn an_algorithm_without_a_race_line_runs_once_per_map_and_model_on_the_centerline() {
        let picked = HashMap::from([("a".to_string(), vec!["fast.csv".to_string()])]);
        let runs = expand(
            &maps(&["a", "b"]),
            &[VehicleModelKind::Bicycle],
            &picked,
            &available(),
            false,
        )
        .unwrap();
        assert_eq!(runs.len(), 2);
        assert!(runs.iter().all(|run| run.race_line == "centerline.csv"));
    }

    #[test]
    fn empty_or_invalid_picks_are_refused() {
        let models = [VehicleModelKind::Bicycle];
        let none = HashMap::new();
        assert!(expand(&[], &models, &none, &available(), false).is_err());
        assert!(expand(&maps(&["a"]), &[], &none, &available(), false).is_err());
        assert!(expand(&maps(&["a"]), &models, &none, &available(), true).is_err());
        assert!(
            expand(
                &maps(&["no_centerline"]),
                &models,
                &none,
                &available(),
                false
            )
            .is_err()
        );
        let unknown = HashMap::from([("a".to_string(), vec!["nope.csv".to_string()])]);
        assert!(expand(&maps(&["a"]), &models, &unknown, &available(), true).is_err());
    }

    #[test]
    fn only_setup_changing_requests_are_blocked_while_running() {
        let benchmark = Benchmark::default();
        assert!(!blocks(&benchmark, &Method::Post, "/api/restart"));
        benchmark.state().running = true;
        assert!(blocks(&benchmark, &Method::Post, "/api/restart"));
        assert!(blocks(&benchmark, &Method::Post, "/api/map_selection"));
        assert!(blocks(&benchmark, &Method::Post, "/api/benchmark/start"));
        assert!(!blocks(&benchmark, &Method::Post, "/api/benchmark/abort"));
        assert!(!blocks(
            &benchmark,
            &Method::Post,
            "/api/human_vesc_command"
        ));
        assert!(!blocks(&benchmark, &Method::Post, "/api/draw"));
        assert!(!blocks(&benchmark, &Method::Get, "/api/maps"));
    }
}
