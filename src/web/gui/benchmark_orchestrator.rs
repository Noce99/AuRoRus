//! What drives the Benchmark panel's runs: [`run_orchestrator`], a thread of
//! [`super::WebGui`] that takes each run of a plan through its soft reset,
//! line-up, countdown and laps, recording it under the benchmarks folder -
//! see [`super::benchmark_api`] for the panel's endpoints and the state the
//! two share.

use super::benchmark_api::{Benchmark, Plan, Progress, RunOutcome, RunSpec, Stage};
use super::opponents_api::{self, line_up};
use super::slam_api::stop_mapping;
use super::{BenchmarkSetup, WebGuiConfig};
use crate::benchmark::layout::{
    SUMMARY_FILE_NAME, TRAJECTORY_FILE_NAME, copy_track, create_run_folder,
};
use crate::benchmark::{
    AlgorithmRecord, BenchmarkSummary, CodeVersion, FORMAT_VERSION, LapResult, MapRecord,
    RaceLineRecord, Results, RunStatus, TrajectoryRow, TrajectoryWriter, VehicleRecord,
};
use crate::environment::{CENTERLINE_FILE_NAME, map_folder, race_lines};
use crate::geometry::{Line, Pose};
use crate::localization::pose_source::localization_pose;
use crate::topics::{
    AUTONOMOUS_ALGORITHM_SELECTION_TOPIC_NAME, AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME,
    AUTONOMOUS_VESC_COMMAND_TOPIC_NAME, ActuatorLimits, AlgorithmParameter,
    AutonomousAlgorithmSelection, AutonomousAlgorithmStatus, HUMAN_VESC_COMMAND_TOPIC_NAME,
    JOYSTICK_VESC_COMMAND_TOPIC_NAME, LAP_TELEMETRY_TOPIC_NAME, LapTelemetry,
    MAP_SELECTION_TOPIC_NAME, MAP_TOPIC_NAME, MapSelection, OpponentRequest,
    RACE_LINE_SELECTION_TOPIC_NAME, RACE_LINE_TOPIC_NAME, RaceLineSelection, Racer, SelectedMap,
    SelectedRaceLine, VEHICLE_MODEL_SELECTION_TOPIC_NAME, VEHICLE_MODEL_STATUS_TOPIC_NAME,
    VEHICLE_STATUS_TOPIC_NAME, VehicleGeometry, VehicleModelSelection, VehicleModelStatus,
    VehicleStatus, VehicleTopics, VescCommand, now_ms,
};
use crate::{Captain, DebugRecorder, DebugState, Stamped, Ticker, autonomous_control, simulation};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};
use time::OffsetDateTime;
use time::format_description::well_known::Rfc3339;

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

/// Runs every benchmark [`start`](super::benchmark_api::start) queues, one run after the other, until
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
            centerline_sha256: hashes.centerline_sha256,
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
                    ..Default::default()
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
    // Laps are scored against the simulator's ground truth, which a real car
    // (`web_gui` on a car) doesn't have.
    let status_topic = captain
        .try_topic::<VehicleStatus>(VEHICLE_STATUS_TOPIC_NAME)
        .ok_or_else(|| Halt::Failed("benchmarks need the simulated vehicle".into()))?;
    let command_topic = captain.try_topic::<VescCommand>(AUTONOMOUS_VESC_COMMAND_TOPIC_NAME);
    let human_topic = captain.topic::<VescCommand>(HUMAN_VESC_COMMAND_TOPIC_NAME);
    let joystick_topic = captain.try_topic::<VescCommand>(JOYSTICK_VESC_COMMAND_TOPIC_NAME);
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
            || joystick_topic
                .as_ref()
                .is_some_and(|topic| human_drives(&topic.read()))
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

/// Whether someone is driving with WASD or the joystick: a fresh command
/// that asks for anything - both keep re-sending a stationary one while no
/// control is held.
fn human_drives(command: &Stamped<VescCommand>) -> bool {
    command.value != VescCommand::default()
        && command
            .age()
            .is_some_and(|age| age <= HUMAN_COMMAND_FRESHNESS)
}

/// Longest a lap of a map whose centerline is `centerline_length_m` long may
/// take, in seconds.
pub(super) fn lap_timeout_s(centerline_length_m: f64, config: &WebGuiConfig) -> f64 {
    centerline_length_m / config.benchmark_timeout_speed_mps.max(f64::MIN_POSITIVE)
}

pub(super) fn algorithm_status(captain: &Captain) -> AutonomousAlgorithmStatus {
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
    let mut limits = values(&status.limits);
    let saved_to_config =
        matches_saved(
            &parameters,
            simulation::saved_vehicle_model_values(status.kind, &status.parameters),
        ) && matches_saved(&limits, simulation::saved_vehicle_limits(&status.limits));
    // The car's own - not in the config file, nor tuned.
    let ego = VehicleTopics::ego();
    if let Some(topic) = ctx
        .captain
        .try_topic::<ActuatorLimits>(&ego.vehicle_limits())
    {
        limits.insert(
            "max_steering_angle_rad".to_string(),
            topic.read().max_steering_angle_rad,
        );
    }
    let geometry = ctx
        .captain
        .try_topic::<VehicleGeometry>(&ego.vehicle_geometry())
        .map_or_else(VehicleGeometry::default, |topic| topic.read().into_value());
    Ok(VehicleRecord {
        model: status.kind.api_str().to_string(),
        body_length_m: geometry.body_length_m,
        body_width_m: geometry.body_width_m,
        saved_to_config,
        parameters,
        limits,
        geometry: Some(geometry),
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
