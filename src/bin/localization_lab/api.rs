//! The lab's HTTP API: the session list and the loaded session, the race
//! line and reference runs are scored against, the draft pipeline (its
//! parameters with the same `set`/`save`/`load` shape as `web_gui`'s
//! parameter panels, so the page reuses `/parameter_panel.js`), the runs
//! with their scores, the series the charts plot, and the drawing protocol
//! (see `aurorus::web::draw`) the map canvas is fed by.
//!
//! The canvas has four layers: the map, the reference trajectory
//! (the ground truth, in pink), every shown run's trajectory in its color,
//! and where each was at the scrubbed time. Which runs are shown, the lap
//! and the time are the page's, sent with every `POST /api/draw`.

use crate::lab::{
    Context, Finished, Group, Lab, Loaded, OdometryChoice, ReferenceChoice, Run, RunState, Spec,
    View,
};
use aurorus::localization::evaluation::{self, LapScore, TimedPose};
use aurorus::topics::{AlgorithmParameter, Color, Drawing, LocalizationAlgorithm, Shape};
use aurorus::web::draw::{
    DrawLayer, DrawRequest, DrawResponse, RasterQuery, raster_response, stale_raster,
    without_raster_pixels,
};
use aurorus::web::{bad_request, error_response, json_response, not_found, query_param, read_json};
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use std::path::Path;
use std::sync::Arc;
use tiny_http::{Request, ResponseBox};

/// What the reference is drawn in.
const REFERENCE_COLOR: Color = Color::PINK;
/// The layer the map is drawn on.
const MAP_LAYER: &str = "Map";
/// Every run's color, by its position in the list of shown runs - the page
/// uses the same.
pub const RUN_COLORS: [Color; 8] = [
    Color::rgb(0x3b, 0x8e, 0xff),
    Color::rgb(0xff, 0xb0, 0x20),
    Color::rgb(0x3b, 0xd1, 0x6f),
    Color::rgb(0xc0, 0x7b, 0xff),
    Color::rgb(0x2e, 0xd8, 0xe6),
    Color::rgb(0xff, 0x5c, 0x5c),
    Color::rgb(0xe8, 0xe8, 0x50),
    Color::rgb(0xff, 0xff, 0xff),
];
/// At most this many points of one run's series are sent for a chart.
const MAX_SERIES_POINTS: usize = 2500;

#[derive(serde::Serialize)]
struct SessionFile {
    name: String,
    size_bytes: u64,
}

/// `GET /api/sessions` - the session files to pick from, newest first.
pub fn sessions(lab: &Lab) -> ResponseBox {
    let files: Vec<_> = lab
        .session_files()
        .into_iter()
        .map(|(name, size_bytes)| SessionFile { name, size_bytes })
        .collect();
    json_response(&files, 200)
}

#[derive(serde::Deserialize)]
struct LoadBody {
    name: String,
}

/// `POST /api/session` - body `{"name": "..."}` - loads that session file,
/// forgetting every run of the current one and reading back its own.
pub fn load_session(request: &mut Request, lab: &Lab) -> ResponseBox {
    let body: LoadBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    match lab.load(&body.name, None) {
        Ok(()) => json_response(&(), 200),
        Err(err) => bad_request(&err),
    }
}

#[derive(serde::Serialize)]
struct SessionSummary<'a> {
    name: &'a str,
    duration_s: f64,
    scans: usize,
    odometry: usize,
    imu: usize,
    ground_truth: bool,
    car: Option<&'a str>,
    map: Option<&'a str>,
    driver: Option<&'a str>,
    driven_on_localization: bool,
    race_lines: &'a [String],
    scan_period_ms: Option<f64>,
}

#[derive(serde::Serialize)]
struct ContextSummary<'a> {
    reference: ReferenceChoice,
    race_line: &'a str,
    lap_m: f64,
    /// When each lap started, in seconds - lap `0` at the session's start.
    laps: Vec<LapSpan>,
}

#[derive(serde::Serialize)]
struct LapSpan {
    lap: u32,
    start_s: f64,
    end_s: f64,
}

#[derive(serde::Serialize)]
struct RunSummary<'a> {
    id: u64,
    label: String,
    /// `queued`, `running`, `done`, `failed` or `cancelled`.
    state: &'static str,
    progress: f64,
    error: Option<&'a str>,
    spec: &'a Spec,
    /// The algorithm's parameters that differ from the session's.
    changes: Vec<Change<'a>>,
    stats: Option<RunStats>,
}

#[derive(serde::Serialize)]
struct Change<'a> {
    name: &'a str,
    value: f64,
    baseline: f64,
}

#[derive(serde::Serialize)]
struct RunStats {
    elapsed_s: f64,
    scans: usize,
    dropped_scans: usize,
    odometry_us_per_sample: Option<f64>,
    /// The commit the run was computed with, and whether the tree had
    /// changes.
    commit: Option<String>,
    dirty: Option<bool>,
    /// The whole drive first, then each lap - empty without a reference.
    scores: Vec<LapScore>,
}

#[derive(serde::Serialize)]
struct DraftSummary {
    odometry: OdometryChoice,
    algorithm: Option<LocalizationAlgorithm>,
    label: String,
    scan_matching: Vec<AlgorithmParameter>,
    particle_filter: Vec<AlgorithmParameter>,
}

#[derive(serde::Serialize)]
struct LabBody<'a> {
    sessions_folder: String,
    session: Option<SessionSummary<'a>>,
    load_error: Option<&'a str>,
    context: Option<ContextSummary<'a>>,
    context_error: Option<&'a str>,
    draft: Option<DraftSummary>,
    /// Oldest first.
    runs: Vec<RunSummary<'a>>,
    workers: usize,
}

/// `GET /api/lab` - the loaded session, how runs are scored, the draft and
/// every run with its scores.
pub fn lab(lab: &Lab) -> ResponseBox {
    lab.with(|view| {
        let View {
            session,
            draft,
            runs,
            load_error,
            context,
            context_error,
        } = view;
        let body = LabBody {
            sessions_folder: lab.sessions_root.display().to_string(),
            session: session.map(|loaded| summary(loaded)),
            load_error,
            context: context.map(|context| context_summary(context, session)),
            context_error: session.and(context_error),
            draft: draft.map(|spec| DraftSummary {
                odometry: spec.odometry,
                algorithm: spec.algorithm,
                label: spec.label(),
                scan_matching: Group::ScanMatching.with_values(spec),
                particle_filter: Group::ParticleFilter.with_values(spec),
            }),
            runs: match session {
                Some(loaded) => runs
                    .iter()
                    .map(|run| run_summary(run, loaded, context))
                    .collect(),
                None => Vec::new(),
            },
            workers: lab.workers,
        };
        json_response(&body, 200)
    })
}

fn summary(loaded: &Loaded) -> SessionSummary<'_> {
    let session = &loaded.session;
    let info = session.info.localization.as_ref();
    SessionSummary {
        name: &loaded.name,
        duration_s: session.duration_us as f64 / 1e6,
        scans: session.scans.len(),
        odometry: session.odometry.len(),
        imu: session.imu.len(),
        ground_truth: !loaded.ground_truth.is_empty(),
        car: session.info.car.as_deref(),
        map: session.info.map.as_deref(),
        driver: info.and_then(|info| info.driver.as_deref()),
        driven_on_localization: info.is_some_and(|info| info.driven_on_localization),
        race_lines: &loaded.race_lines,
        scan_period_ms: loaded.scan_period_ms,
    }
}

fn context_summary<'a>(context: &'a Context, session: Option<&Arc<Loaded>>) -> ContextSummary<'a> {
    let end_s = session.map_or(0.0, |s| s.session.duration_us as f64 / 1e6);
    let starts: Vec<f64> = std::iter::once(0.0)
        .chain(context.laps.lap_starts_us.iter().map(|&t| t as f64 / 1e6))
        .collect();
    let laps = starts
        .iter()
        .enumerate()
        .map(|(lap, &start_s)| LapSpan {
            lap: lap as u32,
            start_s,
            end_s: starts.get(lap + 1).copied().unwrap_or(end_s),
        })
        .collect();
    ContextSummary {
        reference: context.choice,
        race_line: &context.race_line,
        lap_m: context.laps.lap_m,
        laps,
    }
}

fn run_summary<'a>(
    run: &'a Run,
    loaded: &Loaded,
    context: Option<&Arc<Context>>,
) -> RunSummary<'a> {
    let baseline = loaded.baseline.algorithm_values();
    let changes = run
        .spec
        .algorithm_values()
        .iter()
        .filter_map(|(name, &value)| {
            let baseline = baseline.get(name).copied()?;
            (value != baseline).then_some(Change {
                name,
                value,
                baseline,
            })
        })
        .collect();
    let (progress, error, stats) = match &run.state {
        RunState::Queued | RunState::Cancelled => (0.0, None, None),
        RunState::Running { progress } => (*progress, None, None),
        RunState::Done(finished) => (1.0, None, Some(stats(finished, loaded, context))),
        RunState::Failed(err) => (0.0, Some(err.as_str()), None),
    };
    RunSummary {
        id: run.id,
        label: run.spec.label(),
        state: run.state.name(),
        progress,
        error,
        spec: &run.spec,
        changes,
        stats,
    }
}

fn stats(finished: &Finished, loaded: &Loaded, context: Option<&Arc<Context>>) -> RunStats {
    let evaluation = &finished.evaluation;
    RunStats {
        elapsed_s: evaluation.elapsed.as_secs_f64(),
        scans: evaluation.scans.len(),
        dropped_scans: evaluation.dropped_scans,
        odometry_us_per_sample: evaluation.odometry_us_per_sample,
        commit: finished.code.commit.clone(),
        dirty: finished.code.dirty,
        scores: context.map_or_else(Vec::new, |context| {
            finished
                .scored(context, loaded.scan_period_ms)
                .scores
                .clone()
        }),
    }
}

#[derive(serde::Deserialize)]
struct ReferenceBody {
    reference: ReferenceChoice,
}

/// `POST /api/reference` - body `{"reference": {"kind": "ground_truth"}}`
/// or `{"reference": {"kind": "run", "run": 3}}` - what runs are scored
/// against.
pub fn set_reference(request: &mut Request, lab: &Lab) -> ResponseBox {
    let body: ReferenceBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    result(lab.set_reference(body.reference))
}

#[derive(serde::Deserialize)]
struct RaceLineBody {
    file: String,
}

/// `POST /api/race_line` - body `{"file": "..."}` - the race line laps are
/// counted along.
pub fn set_race_line(request: &mut Request, lab: &Lab) -> ResponseBox {
    let body: RaceLineBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    result(lab.set_race_line(&body.file))
}

fn result(result: Result<(), String>) -> ResponseBox {
    match result {
        Ok(()) => json_response(&(), 200),
        Err(err) => bad_request(&err),
    }
}

#[derive(serde::Deserialize)]
struct PipelineBody {
    odometry: OdometryChoice,
    algorithm: Option<LocalizationAlgorithm>,
}

/// `POST /api/draft` - body `{"odometry": "recorded" | "euler" | "midpoint"
/// | "rk4", "algorithm": "scan_matching" | "particle_filter" | null}`.
pub fn set_draft(request: &mut Request, lab: &Lab) -> ResponseBox {
    let body: PipelineBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    result(lab.set_draft_pipeline(body.odometry, body.algorithm))
}

#[derive(serde::Deserialize)]
struct ParameterBody {
    name: String,
    value: f64,
}

/// `POST /api/draft/<group>_parameter` - body `{"name", "value"}`.
pub fn set_draft_parameter(request: &mut Request, lab: &Lab, group: Group) -> ResponseBox {
    let body: ParameterBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    result(lab.set_draft_parameter(group, &body.name, body.value))
}

#[derive(serde::Serialize)]
struct PathBody {
    path: String,
}

fn path_response(path: &Path) -> ResponseBox {
    json_response(
        &PathBody {
            path: path.display().to_string(),
        },
        200,
    )
}

fn group_path(lab: &Lab, group: Group) -> std::path::PathBuf {
    match group {
        Group::ScanMatching => lab.slam_config_path(),
        Group::ParticleFilter => lab.particle_filter_config_path(),
    }
}

fn draft_parameters(lab: &Lab, group: Group) -> Option<Vec<AlgorithmParameter>> {
    lab.with(|view| view.draft.map(|spec| group.with_values(spec)))
}

/// `POST /api/draft/<group>_parameters_save` - writes the draft's values
/// of that group into its config file, keeping its comments. Responds with
/// its path.
pub fn save_draft(lab: &Lab, group: Group) -> ResponseBox {
    let Some(parameters) = draft_parameters(lab, group) else {
        return bad_request("no session is loaded");
    };
    let path = group_path(lab, group);
    match aurorus::config::save_parameter_values(&path, &parameters) {
        Ok(()) => path_response(&path),
        Err(err) => error_response(500, &err),
    }
}

/// `POST /api/draft/<group>_parameters_load` - sets that group of the draft
/// to the values in its config file. Responds with its path.
pub fn load_draft(lab: &Lab, group: Group) -> ResponseBox {
    let Some(parameters) = draft_parameters(lab, group) else {
        return bad_request("no session is loaded");
    };
    let path = group_path(lab, group);
    match aurorus::config::saved_parameter_values(&path, &parameters) {
        Ok(values) => {
            lab.set_draft_values(group, &values);
            path_response(&path)
        }
        Err(err) => error_response(500, &err),
    }
}

/// `POST /api/draft_reset` - sets the draft back to what the session was
/// recorded with.
pub fn reset_draft(lab: &Lab) -> ResponseBox {
    result(lab.reset_draft())
}

#[derive(serde::Serialize)]
struct QueuedBody {
    id: u64,
}

/// `POST /api/runs` - queues a run with the draft. Responds with its id.
pub fn queue_run(lab: &Lab) -> ResponseBox {
    match lab.queue_run() {
        Ok(id) => json_response(&QueuedBody { id }, 200),
        Err(err) => bad_request(&err),
    }
}

#[derive(serde::Deserialize)]
struct RunBody {
    id: u64,
}

/// `POST /api/runs/<action>` - body `{"id": ...}`.
pub fn run_action(
    request: &mut Request,
    action: impl FnOnce(u64) -> Result<(), String>,
) -> ResponseBox {
    let body: RunBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    result(action(body.id))
}

/// The ids in `url`'s `runs` query parameter, comma-separated.
fn run_ids(url: &str) -> Vec<u64> {
    query_param(url, "runs")
        .map(|ids| ids.split(',').filter_map(|id| id.parse().ok()).collect())
        .unwrap_or_default()
}

/// One run's series for the charts.
#[derive(serde::Serialize)]
struct Series {
    id: u64,
    /// `[t_s, s_m, position_m, along_m, cross_m, heading_rad, drift_m]` -
    /// drift `null` until the window is driven.
    errors: Vec<[Option<f64>; 7]>,
    /// `[t_s, s_m, process_ms]`.
    scans: Vec<[f64; 3]>,
}

/// `GET /api/series?runs=1,2&lap=N` - the shown runs' errors and scan
/// costs over lap `N` (every lap without it), at most
/// [`MAX_SERIES_POINTS`] each.
pub fn series(url: &str, lab: &Lab) -> ResponseBox {
    let (Some(session), Some(context)) = (lab.session(), lab.context()) else {
        return json_response(&Vec::<Series>::new(), 200);
    };
    let lap: Option<u32> = query_param(url, "lap").and_then(|lap| lap.parse().ok());
    let body: Vec<Series> = run_ids(url)
        .into_iter()
        .filter_map(|id| Some((id, lab.finished(id)?)))
        .map(|(id, finished)| {
            let scored = finished.scored(&context, session.scan_period_ms);
            let errors: Vec<_> = scored
                .errors
                .iter()
                .filter(|e| lap.is_none_or(|lap| e.lap == lap))
                .collect();
            let step = errors.len().div_ceil(MAX_SERIES_POINTS).max(1);
            let errors = errors
                .into_iter()
                .step_by(step)
                .map(|e| {
                    [
                        Some(e.t_us as f64 / 1e6),
                        Some(e.s_m),
                        Some(e.position_m),
                        Some(e.along_m),
                        Some(e.cross_m),
                        Some(e.heading_rad),
                        e.drift_m,
                    ]
                })
                .collect();
            let scans = finished
                .evaluation
                .scans
                .iter()
                .filter_map(|scan| {
                    let (scan_lap, s_m, _) = context.laps.at(scan.t_us)?;
                    lap.is_none_or(|lap| lap == scan_lap).then_some([
                        scan.t_us as f64 / 1e6,
                        s_m,
                        scan.process_ms,
                    ])
                })
                .collect();
            Series { id, errors, scans }
        })
        .collect();
    json_response(&body, 200)
}

/// `POST /api/draw`'s body: the drawing protocol's (its `t_us` the scrubbed
/// time), plus which runs the page shows and over which lap.
#[derive(serde::Deserialize)]
struct LabDrawRequest {
    #[serde(flatten)]
    request: DrawRequest,
    #[serde(default)]
    runs: Vec<u64>,
    #[serde(default)]
    lap: Option<u32>,
}

/// A layer's version from whatever it's drawn from.
fn version(parts: impl Hash) -> u64 {
    let mut hasher = DefaultHasher::new();
    parts.hash(&mut hasher);
    // Within what a JavaScript number holds exactly, as the page sends it
    // back; `0` means "nothing drawn yet" to it.
    (hasher.finish() & ((1 << 53) - 1)).max(1)
}

/// `POST /api/draw` - the four layers (see the module docs).
pub fn draw(request: &mut Request, lab: &Lab, epoch: u64) -> ResponseBox {
    let body: LabDrawRequest = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let Some(session) = lab.session() else {
        return json_response(
            &DrawResponse {
                epoch,
                layers: Vec::new(),
            },
            200,
        );
    };
    let context = lab.context();
    let span = context
        .as_ref()
        .zip(body.lap)
        .map(|(context, lap)| lap_span_us(context, lap, session.session.duration_us));
    let in_span = |t: u64| span.is_none_or(|(start, end)| t >= start && t < end);
    let runs: Vec<(usize, u64, Arc<Finished>, String)> = body
        .runs
        .iter()
        .enumerate()
        .filter_map(|(index, &id)| {
            let label = lab.with(|view| {
                view.runs
                    .iter()
                    .find(|run| run.id == id)
                    .map(|run| run.spec.label())
            })?;
            Some((index, id, lab.finished(id)?, label))
        })
        .collect();
    let geometry = session.session.geometry.unwrap_or_default();
    let context_key = context.as_ref().map_or(0, |context| context.key);
    let t_us = body.request.t_us;

    let layer = |topic: &str, version: u64, drawing: &dyn Fn() -> Drawing| DrawLayer {
        topic: topic.to_string(),
        writer: None,
        write_count: version,
        age_ms: Some(0.0),
        drawing: (!body.request.holds(epoch, topic, version))
            .then(|| without_raster_pixels(drawing())),
    };
    let ids: Vec<u64> = runs.iter().map(|(_, id, _, _)| *id).collect();
    let layers = vec![
        layer(MAP_LAYER, session.generation, &|| map_drawing(&session)),
        layer(
            "Reference",
            version((session.generation, context_key, body.lap)),
            &|| match &context {
                Some(context) => Drawing::default()
                    .element(
                        reference_name(context),
                        [polyline(&context.reference, &in_span, REFERENCE_COLOR)],
                        true,
                    )
                    .z_index(-30),
                None => Drawing::default(),
            },
        ),
        layer(
            "Runs",
            version((session.generation, &ids, body.lap)),
            &|| {
                runs.iter()
                    .fold(
                        Drawing::default(),
                        |drawing, (index, id, finished, label)| {
                            drawing.element(
                                format!("#{id} {label}"),
                                [polyline(
                                    &finished.evaluation.track,
                                    &in_span,
                                    RUN_COLORS[index % RUN_COLORS.len()],
                                )],
                                true,
                            )
                        },
                    )
                    .z_index(-20)
            },
        ),
        layer(
            "Vehicles",
            version((session.generation, &ids, t_us, context_key)),
            &|| {
                let Some(t_us) = t_us else {
                    return Drawing::default();
                };
                let vehicle = |pose: [f64; 3], color: Color| Shape::Vehicle {
                    x_m: pose[0],
                    y_m: pose[1],
                    heading_rad: pose[2],
                    speed_mps: 0.0,
                    steering_rad: 0.0,
                    length_m: geometry.body_length_m,
                    width_m: geometry.body_width_m,
                    front_axle_m: geometry.lf_m(),
                    rear_axle_m: geometry.lr_m(),
                    color,
                };
                let mut drawing = Drawing::default();
                if let Some(context) = &context
                    && let Some(pose) = evaluation::pose_at(&context.reference, t_us)
                {
                    drawing = drawing.element(
                        reference_name(context),
                        [vehicle(pose, REFERENCE_COLOR)],
                        true,
                    );
                }
                for (index, id, finished, _) in &runs {
                    if let Some(pose) = evaluation::pose_at(&finished.evaluation.track, t_us) {
                        drawing = drawing.element(
                            format!("#{id}"),
                            [vehicle(pose, RUN_COLORS[index % RUN_COLORS.len()])],
                            true,
                        );
                    }
                }
                drawing.z_index(10)
            },
        ),
    ];
    json_response(&DrawResponse { epoch, layers }, 200)
}

/// The session's map, as `web_gui` draws the selected one.
fn map_drawing(session: &Loaded) -> Drawing {
    let Some(info) = &session.map.info else {
        return Drawing::default();
    };
    Drawing::default()
        .element(
            "Map",
            [Shape::Raster {
                origin_x_m: info.origin.x,
                origin_y_m: info.origin.y,
                resolution_m_per_px: info.resolution_m_per_px,
                width_px: session.map.width_px,
                height_px: session.map.height_px,
                pixels: session.map.pixels.clone(),
            }],
            true,
        )
        .z_index(-100)
}

/// `GET /api/draw/raster?...` - the map's raster: its layer's version is
/// the session's generation.
pub fn draw_raster(url: &str, lab: &Lab, epoch: u64) -> ResponseBox {
    let query = match RasterQuery::parse(url) {
        Ok(query) => query,
        Err(response) => return response,
    };
    if query.epoch != epoch {
        return stale_raster();
    }
    match lab.session() {
        Some(session) if query.topic == MAP_LAYER && query.write_count == session.generation => {
            raster_response(&map_drawing(&session), query.shape)
        }
        Some(_) => stale_raster(),
        None => not_found(),
    }
}

fn reference_name(context: &Context) -> String {
    match context.choice {
        ReferenceChoice::GroundTruth => "Ground truth".to_string(),
        ReferenceChoice::Run(id) => format!("Reference: run #{id}"),
    }
}

/// Lap `lap`'s start and end, in microseconds.
fn lap_span_us(context: &Context, lap: u32, duration_us: u64) -> (u64, u64) {
    let starts = &context.laps.lap_starts_us;
    let start = match lap {
        0 => 0,
        lap => starts.get(lap as usize - 1).copied().unwrap_or(u64::MAX),
    };
    let end = starts.get(lap as usize).copied().unwrap_or(duration_us + 1);
    (start, end)
}

/// The poses of `poses` whose time passes `keep`, as a line - one in two.
fn polyline(poses: &[TimedPose], keep: &dyn Fn(u64) -> bool, color: Color) -> Shape {
    Shape::Polyline {
        points: poses
            .iter()
            .filter(|(t, _)| keep(*t))
            .step_by(2)
            .map(|(_, pose)| [pose[0] as f32, pose[1] as f32])
            .collect(),
        closed: false,
        width_px: 2.0,
        color,
    }
}

/// Every run's color as `#rrggbb`, for the page.
pub fn colors() -> ResponseBox {
    let colors: Vec<String> = RUN_COLORS
        .iter()
        .map(|c| format!("#{:02x}{:02x}{:02x}", c.r, c.g, c.b))
        .collect();
    json_response(&colors, 200)
}
