//! The Benchmark panel: an autonomous algorithm driving the ego vehicle
//! alone for [`WebGuiConfig::benchmark_laps`] laps, on every combination of
//! the maps, vehicle models and race lines picked, each run recorded under
//! the benchmarks folder (see [`crate::benchmark`]).
//!
//! The runs are driven by [`run_orchestrator`], a thread of [`super::WebGui`]
//! writing the same selection topics the other panels do, as `WebGui`. Each
//! run starts with a soft reset (opponents removed, any debug recording
//! stopped, the map, model and race line selected), lines the vehicle up
//! like a race start with a countdown, and ends after the laps, when a lap
//! times out, or when aborted (the Abort button, any WASD key, a shutdown).
//! While a benchmark runs, [`blocks`] tells [`super::handlers`] to refuse
//! every request that would change the setup.

use super::live_api::stop_mapping;
use super::opponents_api::{self, line_up};
use super::{BenchmarkSetup, WebGuiConfig};
use crate::autonomous_control::shared::race_line::{Line, Pose, localization_pose};
use crate::benchmark::layout::{
    SUMMARY_FILE_NAME, TIMING_LINE_FILE_NAME, TRAJECTORY_FILE_NAME, copy_track, create_run_folder,
};
use crate::benchmark::{
    AlgorithmRecord, BenchmarkSummary, CodeVersion, FORMAT_VERSION, LapResult, MapRecord,
    RaceLineRecord, Results, RunStatus, TrajectoryRow, TrajectoryWriter, VehicleRecord,
};
use crate::environment::{CENTERLINE_FILE_NAME, map_folder, race_lines, read_info};
use crate::topics::{
    AUTONOMOUS_ALGORITHM_SELECTION_TOPIC_NAME, AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME,
    AUTONOMOUS_VESC_COMMAND_TOPIC_NAME, AlgorithmParameter, AutonomousAlgorithmSelection,
    AutonomousAlgorithmStatus, HUMAN_VESC_COMMAND_TOPIC_NAME, LAP_TELEMETRY_TOPIC_NAME,
    LapTelemetry, MAP_SELECTION_TOPIC_NAME, MAP_TOPIC_NAME, MapSelection, OpponentRequest,
    RACE_LINE_SELECTION_TOPIC_NAME, RACE_LINE_TOPIC_NAME, RaceLineSelection, Racer, SelectedMap,
    SelectedRaceLine, VEHICLE_BODY_LENGTH_M, VEHICLE_BODY_WIDTH_M,
    VEHICLE_MODEL_SELECTION_TOPIC_NAME, VEHICLE_MODEL_STATUS_TOPIC_NAME, VEHICLE_STATUS_TOPIC_NAME,
    VehicleModelKind, VehicleModelSelection, VehicleModelStatus, VehicleStatus, VehicleTopics,
    VescCommand, now_ms,
};
use crate::web::{bad_request, error_response, json_response, read_json};
use crate::{Captain, DebugRecorder, DebugState, Stamped, Ticker, actuators, autonomous_control};
use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use time::OffsetDateTime;
use time::format_description::well_known::Rfc3339;
use tiny_http::{Method, Request, ResponseBox};

/// How long each step of a run's soft reset may take to show on its topic
/// before the run is skipped.
const STEP_TIMEOUT: Duration = Duration::from_secs(10);
/// How often a waiting step, or an idle orchestrator, looks again.
const POLL_INTERVAL: Duration = Duration::from_millis(50);
/// How far along the race line, in meters, a sample's projection is searched
/// from the previous one's - as `crate::telemetry`'s `search_window_m`.
const LINE_SEARCH_WINDOW_M: f64 = 3.0;
/// A human command no older than this counts as someone driving - as
/// `crate::topics::VESC_COMMAND_TIMEOUT`.
const HUMAN_COMMAND_FRESHNESS: Duration = Duration::from_millis(500);

/// The routes still served while a benchmark runs: aborting it, drawing the
/// map (a POST, but read-only), and the WASD command - which aborts it.
const ALLOWED_WHILE_RUNNING: &[&str] = &[
    "/api/benchmark/abort",
    "/api/draw",
    "/api/human_vesc_command",
];

/// One run of a benchmark: where, on which model, against which line.
#[derive(Debug, Clone, PartialEq)]
struct RunSpec {
    map: String,
    model: VehicleModelKind,
    /// File inside the map's `race_lines/` - the centerline when the
    /// algorithm follows no line.
    race_line: String,
}

/// A benchmark: every run, in order.
#[derive(Debug, Clone)]
struct Plan {
    algorithm: String,
    /// Whether the algorithm follows the race line.
    uses_race_line: bool,
    runs: Vec<RunSpec>,
}

/// What a run is doing, for the progress display.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
enum Stage {
    /// The soft reset: selecting the map, model and line.
    Preparing,
    /// On the grid, waiting for the "Go!".
    Countdown,
    Driving,
}

/// The run in progress.
#[derive(Debug, Clone, serde::Serialize)]
struct Progress {
    /// Counted from `1`.
    run: usize,
    map: String,
    model: String,
    race_line: String,
    stage: Stage,
    /// The lap in progress, counted from `1` - `0` before the first line
    /// crossing.
    lap: u32,
    /// When the vehicle is released, in [`now_ms`]'s clock.
    go_at_ms: Option<u64>,
    lap_timeout_s: Option<f64>,
    /// Seconds into the lap in progress, by the benchmark's own clock (the
    /// one the timeout is measured with).
    lap_elapsed_s: f64,
    /// Seconds since the "Go!".
    elapsed_s: f64,
}

/// How one run went.
#[derive(Debug, Clone, serde::Serialize)]
struct RunOutcome {
    map: String,
    model: String,
    race_line: String,
    /// `None` when the run couldn't start - see `error`.
    status: Option<RunStatus>,
    error: Option<String>,
    /// The run folder, if one was written.
    folder: Option<PathBuf>,
    laps: u32,
    total_time_s: Option<f64>,
    best_lap_s: Option<f64>,
}

#[derive(Debug, Default)]
struct State {
    /// A benchmark requested but not picked up by the orchestrator yet.
    pending: Option<Plan>,
    /// The benchmark running, or the last one.
    plan: Option<Plan>,
    running: bool,
    abort: bool,
    progress: Option<Progress>,
    /// Every run of `plan` finished so far.
    outcomes: Vec<RunOutcome>,
}

/// The benchmark shared by `WebGui`'s request handlers and its orchestrator
/// thread. Cheap to clone - every clone is the same benchmark - and carried
/// across restarts (see [`super::WebGui`]'s `fresh`), so the last results
/// still show.
#[derive(Debug, Clone, Default)]
pub struct Benchmark(Arc<Mutex<State>>);

impl Benchmark {
    fn state(&self) -> std::sync::MutexGuard<'_, State> {
        self.0.lock().unwrap()
    }

    fn aborted(&self) -> bool {
        self.state().abort
    }

    fn set_progress(&self, update: impl FnOnce(&mut Progress)) {
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
/// picks and queues every combination for [`run_orchestrator`].
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

/// Everything [`run_orchestrator`] needs from `WebGui`.
pub struct Orchestrator<'a> {
    pub captain: &'a Captain,
    /// `WebGui`'s executor id, to write the topics it claimed.
    pub id: u16,
    pub config: &'a WebGuiConfig,
    pub maps_root: &'a Path,
    pub setup: &'a BenchmarkSetup,
    pub recorder: &'a DebugRecorder,
    pub benchmark: &'a Benchmark,
}

/// Why a run stopped early.
enum Halt {
    /// Aborted, or shutting down: the whole benchmark stops.
    Abort,
    /// This run can't go on; the next one still starts.
    Failed(String),
}

/// Runs every benchmark [`start`] queues, one run after the other, until
/// `WebGui` stops.
pub fn run_orchestrator(ctx: &Orchestrator) {
    {
        // A benchmark cut short by a restart doesn't run on in the next one.
        let mut state = ctx.benchmark.state();
        state.pending = None;
        state.running = false;
        state.progress = None;
    }
    while ctx.captain.is_running(ctx.id) {
        let pending = ctx.benchmark.state().pending.take();
        let Some(plan) = pending else {
            std::thread::sleep(POLL_INTERVAL);
            continue;
        };
        run_plan(ctx, &plan);
        let mut state = ctx.benchmark.state();
        state.running = false;
        state.progress = None;
    }
}

fn run_plan(ctx: &Orchestrator, plan: &Plan) {
    let code = code_version();
    for (index, spec) in plan.runs.iter().enumerate() {
        if ctx.benchmark.aborted() || !ctx.captain.is_running(ctx.id) {
            break;
        }
        ctx.benchmark.state().progress = Some(Progress {
            run: index + 1,
            map: spec.map.clone(),
            model: spec.model.api_str().to_string(),
            race_line: spec.race_line.clone(),
            stage: Stage::Preparing,
            lap: 0,
            go_at_ms: None,
            lap_timeout_s: None,
            lap_elapsed_s: 0.0,
            elapsed_s: 0.0,
        });
        let mut outcome = RunOutcome {
            map: spec.map.clone(),
            model: spec.model.api_str().to_string(),
            race_line: spec.race_line.clone(),
            status: None,
            error: None,
            folder: None,
            laps: 0,
            total_time_s: None,
            best_lap_s: None,
        };
        let stop = match run_one(ctx, plan, spec, &code, &mut outcome) {
            Ok(()) => false,
            Err(Halt::Abort) => true,
            Err(Halt::Failed(err)) => {
                eprintln!("benchmark: run {} skipped: {err}", index + 1);
                outcome.error = Some(err);
                false
            }
        };
        // Whatever happened, the vehicle stops.
        select_algorithm(ctx, &plan.algorithm, false);
        ctx.benchmark.state().outcomes.push(outcome);
        if stop {
            break;
        }
    }
}

/// One run, from the soft reset to its files, filling in `outcome`. An
/// error before its folder was created means it never started.
fn run_one(
    ctx: &Orchestrator,
    plan: &Plan,
    spec: &RunSpec,
    code: &CodeVersion,
    outcome: &mut RunOutcome,
) -> Result<(), Halt> {
    let folder = map_folder(ctx.maps_root, &spec.map)
        .ok_or_else(|| Halt::Failed(format!("invalid map name {:?}", spec.map)))?;
    soft_reset(ctx, plan, spec, &folder)?;

    // What's about to run, exactly.
    let centerline = race_lines::read(&folder, CENTERLINE_FILE_NAME)
        .map_err(|err| Halt::Failed(format!("the map's centerline: {err}")))?;
    let centerline_length_m = race_lines::lap_length_m(&centerline);
    let lap_timeout_s = lap_timeout_s(centerline_length_m, ctx.config);
    let line_points = race_lines::read(&folder, &spec.race_line)
        .map_err(|err| Halt::Failed(format!("the race line: {err}")))?;
    let line_length_m = race_lines::lap_length_m(&line_points);
    let line = Line::new(line_points)
        .ok_or_else(|| Halt::Failed("the race line has too few points".to_string()))?;
    let algorithm = algorithm_record(ctx, &plan.algorithm)?;
    let vehicle = vehicle_record(ctx)?;

    let started = OffsetDateTime::now_local().unwrap_or_else(|_| OffsetDateTime::now_utc());
    let run_folder = create_run_folder(
        &ctx.setup.root,
        &spec.map,
        started,
        &plan.algorithm,
        spec.model.api_str(),
    )
    .map_err(|err| Halt::Failed(format!("couldn't create the run folder: {err}")))?;
    outcome.folder = Some(run_folder.clone());
    let hashes = copy_track(&folder, &spec.race_line, &run_folder)
        .map_err(|err| Halt::Failed(format!("couldn't copy the map: {err}")))?;
    let mut summary = BenchmarkSummary {
        format_version: FORMAT_VERSION,
        // Until the run ends: what a crash leaves behind.
        status: RunStatus::AbortedByUser,
        started_at: started
            .format(&Rfc3339)
            .expect("formatting a time as RFC3339 never fails"),
        countdown_s: ctx.config.race_countdown_ms as f64 / 1000.0,
        laps_requested: ctx.config.benchmark_laps,
        pose_source: ctx.setup.pose_source,
        code: code.clone(),
        map: MapRecord {
            name: spec.map.clone(),
            info_sha256: hashes.info_sha256,
            tiff_sha256: hashes.tiff_sha256,
            centerline_length_m,
            lap_timeout_s,
        },
        race_line: RaceLineRecord {
            file: spec.race_line.clone(),
            method: race_lines::meta(&folder, &spec.race_line).method,
            used_by_algorithm: plan.uses_race_line,
            sha256: hashes.race_line_sha256,
            length_m: line_length_m,
        },
        algorithm,
        vehicle,
        results: None,
        laps: Vec::new(),
    };
    let summary_path = run_folder.join(SUMMARY_FILE_NAME);
    summary.write(&summary_path).map_err(Halt::Failed)?;
    let mut trajectory = TrajectoryWriter::create(&run_folder.join(TRAJECTORY_FILE_NAME))
        .map_err(|err| Halt::Failed(format!("couldn't create the trajectory: {err}")))?;

    let drive = drive(ctx, plan, &line, lap_timeout_s, &mut trajectory);
    select_algorithm(ctx, &plan.algorithm, false);
    if let Err(err) = trajectory.flush() {
        eprintln!("benchmark: couldn't write the trajectory: {err}");
    }
    let (status, laps) = drive?;

    summary.status = status;
    summary.results = Results::of(&laps, status == RunStatus::Completed);
    summary.laps = laps;
    summary.write(&summary_path).map_err(Halt::Failed)?;
    outcome.status = Some(status);
    outcome.laps = summary.laps.len() as u32;
    outcome.total_time_s = summary.results.and_then(|results| results.total_time_s);
    outcome.best_lap_s = summary.results.map(|results| results.best_lap_s);
    if status == RunStatus::AbortedByUser {
        return Err(Halt::Abort);
    }
    Ok(())
}

/// Brings everything to the run's starting point: no opponents, no debug
/// recording, the algorithm picked but paused, and the run's map, model and
/// line loaded - each confirmed on its topic before going on.
fn soft_reset(ctx: &Orchestrator, plan: &Plan, spec: &RunSpec, folder: &Path) -> Result<(), Halt> {
    let captain = ctx.captain;

    for opponent in opponents_api::opponents(captain).list {
        opponents_api::queue(captain, ctx.id, OpponentRequest::Delete(opponent.id));
    }
    wait_until(ctx, "the opponents to be removed", || {
        opponents_api::opponents(captain).list.is_empty()
    })?;

    if ctx.recorder.status().state == DebugState::Recording
        && let Err(err) = ctx.recorder.stop(captain)
    {
        eprintln!("benchmark: couldn't stop the debug recording: {err}");
    }

    select_algorithm(ctx, &plan.algorithm, false);
    wait_until(ctx, "the algorithm to be selected", || {
        let status = algorithm_status(captain);
        status.selected.as_deref() == Some(plan.algorithm.as_str()) && status.active.is_none()
    })?;

    let map_topic = captain.topic::<SelectedMap>(MAP_TOPIC_NAME);
    if map_topic.read().path.as_deref() != Some(folder) {
        captain
            .topic::<MapSelection>(MAP_SELECTION_TOPIC_NAME)
            .write(
                ctx.id,
                MapSelection {
                    path: Some(folder.to_path_buf()),
                },
            )
            .expect("lost writer authorization for the map_selection topic");
        // A map built on the old track means nothing on the new one.
        stop_mapping(captain, ctx.id);
    }
    wait_until(ctx, "the map to load", || {
        map_topic.read().path.as_deref() == Some(folder)
    })?;

    captain
        .topic::<VehicleModelSelection>(VEHICLE_MODEL_SELECTION_TOPIC_NAME)
        .write(ctx.id, VehicleModelSelection { kind: spec.model })
        .expect("lost writer authorization for the vehicle_model_selection topic");
    wait_until(ctx, "the vehicle model to switch", || {
        captain
            .try_topic::<VehicleModelStatus>(VEHICLE_MODEL_STATUS_TOPIC_NAME)
            .is_some_and(|topic| topic.read().kind == spec.model)
    })?;

    // Selected even when it's already the line followed, so the lap
    // telemetry starts over on it.
    let line_topic = captain.topic::<SelectedRaceLine>(RACE_LINE_TOPIC_NAME);
    let writes_before = line_topic.meta().write_count;
    captain
        .topic::<RaceLineSelection>(RACE_LINE_SELECTION_TOPIC_NAME)
        .write(
            ctx.id,
            RaceLineSelection {
                map: Some(folder.to_path_buf()),
                file: spec.race_line.clone(),
            },
        )
        .expect("lost writer authorization for the race_line_selection topic");
    wait_until(ctx, "the race line to load", || {
        let line = line_topic.read();
        line.meta.write_count > writes_before
            && line.map.as_deref() == Some(folder)
            && line.file.as_deref() == Some(spec.race_line.as_str())
    })?;
    let line_written_at = line_topic.meta().written_at;
    wait_until(ctx, "the lap telemetry to follow the race line", || {
        let telemetry = lap_telemetry(captain);
        telemetry.meta.written_at > line_written_at
            && telemetry.race_line.as_deref() == Some(spec.race_line.as_str())
    })
}

/// Lines the vehicle up, hands it to the algorithm at the "Go!", and records
/// it until the laps are done, a lap times out, or the benchmark is
/// aborted. Returns how it ended and every lap completed.
fn drive(
    ctx: &Orchestrator,
    plan: &Plan,
    line: &Line,
    lap_timeout_s: f64,
    trajectory: &mut TrajectoryWriter,
) -> Result<(RunStatus, Vec<LapResult>), Halt> {
    let captain = ctx.captain;
    let laps_requested = ctx.config.benchmark_laps as usize;
    let go_at_ms = line_up(captain, ctx.id, &[Racer::Ego], ctx.config).map_err(Halt::Failed)?;
    let go_at = Instant::now() + Duration::from_millis(go_at_ms.saturating_sub(now_ms()));
    // Held on the grid until the "Go!" whatever it commands.
    select_algorithm(ctx, &plan.algorithm, true);
    ctx.benchmark.set_progress(|progress| {
        progress.stage = Stage::Countdown;
        progress.go_at_ms = Some(go_at_ms);
        progress.lap_timeout_s = Some(lap_timeout_s);
    });

    let ego = VehicleTopics::ego();
    let status_topic = captain.topic::<VehicleStatus>(VEHICLE_STATUS_TOPIC_NAME);
    let command_topic = captain.try_topic::<VescCommand>(AUTONOMOUS_VESC_COMMAND_TOPIC_NAME);
    let human_topic = captain.topic::<VescCommand>(HUMAN_VESC_COMMAND_TOPIC_NAME);
    // Laps already on this line before the run - a placement keeps them.
    let baseline = lap_telemetry(captain).laps.len();
    let mut laps: Vec<LapResult> = Vec::new();
    let mut lap_started = go_at;
    let mut hint = None;
    let mut ticker = Ticker::new(ctx.config.benchmark_sample_rate_hz);

    loop {
        ticker.wait();
        if ctx.benchmark.aborted()
            || !captain.is_running(ctx.id)
            || human_drives(&human_topic.read())
        {
            return Ok((RunStatus::AbortedByUser, laps));
        }
        let now = Instant::now();
        if now < go_at {
            continue;
        }

        let telemetry = lap_telemetry(captain).into_value();
        for record in telemetry.laps.iter().skip(baseline + laps.len()) {
            // The lap just completed is `previous` - unless another one
            // completed since, too fast to see it.
            let trace = telemetry
                .previous
                .as_ref()
                .filter(|trace| trace.number == record.number);
            let lateral: Vec<f64> = trace
                .map(|trace| {
                    trace
                        .lateral_m
                        .iter()
                        .flatten()
                        .map(|&d| f64::from(d).abs())
                        .collect()
                })
                .unwrap_or_default();
            laps.push(LapResult {
                number: laps.len() as u32 + 1,
                time_s: record.time_s,
                distance_m: record.distance_m,
                average_speed_mps: record.average_speed_mps,
                max_abs_lateral_m: lateral.iter().copied().reduce(f64::max),
                mean_abs_lateral_m: (!lateral.is_empty())
                    .then(|| lateral.iter().sum::<f64>() / lateral.len() as f64),
            });
            lap_started = now;
        }
        if laps.len() >= laps_requested {
            laps.truncate(laps_requested);
            return Ok((RunStatus::Completed, laps));
        }
        if now.duration_since(lap_started).as_secs_f64() > lap_timeout_s {
            return Ok((RunStatus::Timeout, laps));
        }

        let lap = if telemetry.current_lap_time_s.is_some() {
            laps.len() as u32 + 1
        } else {
            0
        };
        let elapsed_s = now.duration_since(go_at).as_secs_f64();
        ctx.benchmark.set_progress(|progress| {
            progress.stage = Stage::Driving;
            progress.lap = lap;
            progress.elapsed_s = elapsed_s;
            progress.lap_elapsed_s = now.duration_since(lap_started).as_secs_f64();
        });

        let vehicle = status_topic.read();
        if vehicle.meta.write_count == 0 {
            continue;
        }
        let nearest = line.nearest(vehicle.x_m, vehicle.y_m, hint, LINE_SEARCH_WINDOW_M);
        hint = Some(nearest.segment);
        let pose = Pose {
            x_m: vehicle.x_m,
            y_m: vehicle.y_m,
            heading_rad: vehicle.heading_rad,
        };
        let (lateral_m, _) = line.lateral(pose, &nearest);
        let command = command_topic
            .as_ref()
            .map(|topic| topic.read().into_value())
            .unwrap_or_default();
        let estimate = localization_pose(captain, &ego).ok();
        let row = TrajectoryRow {
            t_s: elapsed_s,
            lap,
            x_m: vehicle.x_m,
            y_m: vehicle.y_m,
            heading_rad: vehicle.heading_rad,
            speed_mps: vehicle.speed_mps,
            steering_cmd_rad: command.servo_position_rad,
            speed_cmd_mps: command.speed_mps,
            s_m: Some(nearest.s_m),
            lateral_m: Some(lateral_m),
            est_x_m: estimate.map(|pose| pose.x_m),
            est_y_m: estimate.map(|pose| pose.y_m),
            est_heading_rad: estimate.map(|pose| pose.heading_rad),
        };
        if let Err(err) = trajectory.write(&row) {
            return Err(Halt::Failed(format!(
                "couldn't write the trajectory: {err}"
            )));
        }
    }
}

/// Waits until `done`, checking every [`POLL_INTERVAL`] for up to
/// [`STEP_TIMEOUT`] - failing the run past that, and stopping the benchmark
/// if it's aborted meanwhile.
fn wait_until(ctx: &Orchestrator, what: &str, mut done: impl FnMut() -> bool) -> Result<(), Halt> {
    let deadline = Instant::now() + STEP_TIMEOUT;
    loop {
        if ctx.benchmark.aborted() || !ctx.captain.is_running(ctx.id) {
            return Err(Halt::Abort);
        }
        if done() {
            return Ok(());
        }
        if Instant::now() >= deadline {
            return Err(Halt::Failed(format!("timed out waiting for {what}")));
        }
        std::thread::sleep(POLL_INTERVAL);
    }
}

fn select_algorithm(ctx: &Orchestrator, name: &str, running: bool) {
    ctx.captain
        .topic::<AutonomousAlgorithmSelection>(AUTONOMOUS_ALGORITHM_SELECTION_TOPIC_NAME)
        .write(
            ctx.id,
            AutonomousAlgorithmSelection {
                name: Some(name.to_string()),
                running,
            },
        )
        .expect("lost writer authorization for the autonomous_algorithm_selection topic");
}

/// Whether someone is driving with WASD: a fresh command that asks for
/// anything - `web_gui` keeps re-sending a stationary one while no key is
/// held.
fn human_drives(command: &Stamped<VescCommand>) -> bool {
    command.value != VescCommand::default()
        && command
            .age()
            .is_some_and(|age| age <= HUMAN_COMMAND_FRESHNESS)
}

/// Longest a lap of a map whose centerline is `centerline_length_m` long may
/// take, in seconds.
fn lap_timeout_s(centerline_length_m: f64, config: &WebGuiConfig) -> f64 {
    centerline_length_m / config.benchmark_timeout_speed_mps.max(f64::MIN_POSITIVE)
}

fn algorithm_status(captain: &Captain) -> AutonomousAlgorithmStatus {
    captain
        .try_topic::<AutonomousAlgorithmStatus>(AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME)
        .map(|topic| topic.read().into_value())
        .unwrap_or_default()
}

/// The lap telemetry - never written if this binary has no
/// `LapTelemetryRecorder`, when every run times out its first lap.
fn lap_telemetry(captain: &Captain) -> Stamped<LapTelemetry> {
    captain
        .try_topic::<LapTelemetry>(LAP_TELEMETRY_TOPIC_NAME)
        .map(|topic| topic.read())
        .unwrap_or_else(|| Stamped {
            value: LapTelemetry::default(),
            meta: crate::WriteMeta::default(),
        })
}

fn values(parameters: &[AlgorithmParameter]) -> BTreeMap<String, f64> {
    parameters
        .iter()
        .map(|parameter| (parameter.name.clone(), parameter.value))
        .collect()
}

/// Whether every value in `running` is what `saved` holds for it - within a
/// few parts per million, since a parameter stored as an `f32` runs with the
/// saved `f64` rounded to it.
fn matches_saved(
    running: &BTreeMap<String, f64>,
    saved: Result<(BTreeMap<String, f64>, PathBuf), String>,
) -> bool {
    saved.is_ok_and(|(saved, _)| {
        running.iter().all(|(name, value)| {
            saved
                .get(name)
                .is_some_and(|saved| (saved - value).abs() <= 1e-6 * value.abs().max(1.0))
        })
    })
}

/// The algorithm's parameters it's running with now.
fn algorithm_record(ctx: &Orchestrator, name: &str) -> Result<AlgorithmRecord, Halt> {
    let status = algorithm_status(ctx.captain);
    let algorithm = status
        .available
        .iter()
        .find(|algorithm| algorithm.name == name)
        .ok_or_else(|| Halt::Failed(format!("the algorithm {name:?} is gone")))?;
    let parameters = values(&algorithm.parameters);
    Ok(AlgorithmRecord {
        name: name.to_string(),
        saved_to_config: matches_saved(
            &parameters,
            autonomous_control::saved_values(name, &algorithm.parameters),
        ),
        parameters,
    })
}

/// The vehicle model's parameters and limits it's running with now.
fn vehicle_record(ctx: &Orchestrator) -> Result<VehicleRecord, Halt> {
    let status = ctx
        .captain
        .try_topic::<VehicleModelStatus>(VEHICLE_MODEL_STATUS_TOPIC_NAME)
        .ok_or_else(|| Halt::Failed("no simulated vehicle".to_string()))?
        .read()
        .into_value();
    let parameters = values(&status.parameters);
    let limits = values(&status.limits);
    let saved_to_config =
        matches_saved(
            &parameters,
            actuators::saved_vehicle_model_values(status.kind, &status.parameters),
        ) && matches_saved(&limits, actuators::saved_vehicle_limits(&status.limits));
    Ok(VehicleRecord {
        model: status.kind.api_str().to_string(),
        body_length_m: VEHICLE_BODY_LENGTH_M,
        body_width_m: VEHICLE_BODY_WIDTH_M,
        saved_to_config,
        parameters,
        limits,
    })
}

/// The git commit checked out, and whether the working tree has changes -
/// both `None` if git can't tell.
fn code_version() -> CodeVersion {
    let git = |args: &[&str]| {
        std::process::Command::new("git")
            .args(args)
            .output()
            .ok()
            .filter(|output| output.status.success())
            .map(|output| String::from_utf8_lossy(&output.stdout).trim().to_string())
    };
    CodeVersion {
        commit: git(&["rev-parse", "HEAD"]),
        dirty: git(&["status", "--porcelain"]).map(|changes| !changes.is_empty()),
    }
}

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

    #[test]
    fn a_lap_times_out_at_the_centerline_driven_at_the_timeout_speed() {
        let config = WebGuiConfig {
            benchmark_timeout_speed_mps: 2.0,
            ..WebGuiConfig::default()
        };
        assert_eq!(lap_timeout_s(40.0, &config), 20.0);
    }

    #[test]
    fn saved_values_match_within_rounding() {
        let running = BTreeMap::from([("a".to_string(), 0.1 + 0.2)]);
        let saved = |value: f64| Ok((BTreeMap::from([("a".to_string(), value)]), PathBuf::new()));
        assert!(matches_saved(&running, saved(0.3)));
        let rounded = BTreeMap::from([("a".to_string(), f64::from(2.2f32))]);
        assert!(matches_saved(&rounded, saved(2.2)));
        assert!(!matches_saved(&running, saved(0.31)));
        assert!(!matches_saved(&running, Err("no file".to_string())));
    }
}
