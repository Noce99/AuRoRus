//! The lab's HTTP API: the session list and the loaded session, the draft
//! parameters (the same `set`/`save`/`load` shape as `web_gui`'s parameter
//! panels, so the page reuses `/parameter_panel.js`), the runs, the sweeps,
//! and the drawing protocol (see `aurorus::web::draw`) the map canvas is fed by.
//!
//! The canvas has three layers: the selected run's map as live SLAM draws
//! it, a compared run's walls and trajectory laid over it, and - in
//! simulation - the ground truth: the true map's walls and the true
//! trajectory. Which runs are selected and compared is the page's, sent
//! with every `POST /api/draw`.

use crate::lab::{Finished, Lab, Loaded, Run, RunState, Sweep, View, parameters_with};
use crate::sweep::{Axis, AxisSpec};
use aurorus::localization::Scores;
use aurorus::topics::{Color, Drawing, Shape};
use aurorus::web::draw::{
    DrawLayer, DrawRequest, DrawResponse, RasterQuery, raster_response, stale_raster,
    without_raster_pixels,
};
use aurorus::web::{
    bad_request, error_response, header, json_response, not_found, query_param, read_json,
};
use std::collections::BTreeMap;
use std::path::Path;
use std::sync::Arc;
use tiny_http::{Request, Response, ResponseBox};

/// The layer the selected run's map is drawn on.
const SELECTED_LAYER: &str = "Selected run";
/// The layer a compared run's walls and trajectory are drawn on.
const COMPARED_LAYER: &str = "Compared run";
/// The layer the ground truth is drawn on.
const TRUTH_LAYER: &str = "Ground truth";
/// What the ground truth is drawn in.
const TRUTH_COLOR: Color = Color::PINK;
/// Above the runs.
const TRUTH_Z_INDEX: i32 = -30;
/// Every how many recorded true poses one is drawn.
const TRUTH_EVERY: usize = 5;

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
/// forgetting every run of the current one.
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
    /// Whether the true pose was recorded (in simulation).
    ground_truth: bool,
    car: Option<&'a str>,
    map: Option<&'a str>,
    true_map_error: Option<&'a str>,
}

#[derive(serde::Serialize)]
struct Change<'a> {
    name: &'a str,
    value: f64,
    baseline: f64,
}

#[derive(serde::Serialize)]
struct RunStats<'a> {
    kept: usize,
    processed: usize,
    loop_closures: usize,
    dropped: usize,
    paused: usize,
    elapsed_s: f64,
    map_width_m: f64,
    map_height_m: f64,
    scores: &'a Scores,
}

#[derive(serde::Serialize)]
struct RunSummary<'a> {
    id: u64,
    /// The sweep that queued it, if one did.
    sweep: Option<u64>,
    /// `queued`, `running`, `done`, `failed` or `cancelled`.
    state: &'static str,
    progress: f64,
    error: Option<&'a str>,
    /// The parameters that differ from the session's baseline.
    changes: Vec<Change<'a>>,
    stats: Option<RunStats<'a>>,
}

#[derive(serde::Serialize)]
struct LabBody<'a> {
    sessions_folder: String,
    session: Option<SessionSummary<'a>>,
    load_error: Option<&'a str>,
    /// Every tunable parameter, with the draft's value.
    parameters: Vec<aurorus::topics::AlgorithmParameter>,
    /// Newest first.
    runs: Vec<RunSummary<'a>>,
    /// Newest first.
    sweeps: Vec<SweepSummary<'a>>,
    /// How many runs are built at once.
    workers: usize,
}

#[derive(serde::Serialize)]
struct CellSummary<'a> {
    /// An index into each axis' values.
    cell: &'a [usize],
    /// Its run - `None` if it was skipped.
    run: Option<u64>,
    /// Why SLAM can't run with it, if it was skipped.
    skipped: Option<&'a str>,
}

#[derive(serde::Serialize)]
struct SweepSummary<'a> {
    id: u64,
    axes: &'a [Axis],
    cells: Vec<CellSummary<'a>>,
    /// How many of its runs are over, of how many.
    over: usize,
    runs: usize,
    /// Where its results were written, once every run is over.
    saved_to: Option<String>,
    save_error: Option<&'a str>,
}

/// `GET /api/lab` - the loaded session, the draft, every run and every
/// sweep.
pub fn lab(lab: &Lab) -> ResponseBox {
    lab.with(|view| {
        let View {
            session,
            draft,
            runs,
            sweeps,
            load_error,
        } = view;
        let body = LabBody {
            sessions_folder: lab.sessions_root.display().to_string(),
            session: session.map(|loaded| summary(loaded)),
            load_error,
            parameters: parameters_with(draft),
            runs: match session {
                Some(loaded) => runs
                    .iter()
                    .rev()
                    .map(|run| run_summary(run, &loaded.baseline))
                    .collect(),
                None => Vec::new(),
            },
            sweeps: sweeps
                .iter()
                .rev()
                .map(|sweep| sweep_summary(sweep, runs))
                .collect(),
            workers: lab.workers,
        };
        json_response(&body, 200)
    })
}

fn sweep_summary<'a>(sweep: &'a Sweep, runs: &[Run]) -> SweepSummary<'a> {
    let run_ids: Vec<u64> = sweep
        .cells
        .iter()
        .filter_map(|c| c.run.as_ref().ok().copied())
        .collect();
    let over = run_ids
        .iter()
        .filter(|&&id| {
            runs.iter()
                .find(|run| run.id == id)
                .is_none_or(|run| run.state.is_over())
        })
        .count();
    let (saved_to, save_error) = match &sweep.saved {
        Some(Ok(path)) => (Some(path.display().to_string()), None),
        Some(Err(err)) => (None, Some(err.as_str())),
        None => (None, None),
    };
    SweepSummary {
        id: sweep.id,
        axes: &sweep.axes,
        cells: sweep
            .cells
            .iter()
            .map(|cell| CellSummary {
                cell: &cell.cell,
                run: cell.run.as_ref().ok().copied(),
                skipped: cell.run.as_ref().err().map(String::as_str),
            })
            .collect(),
        over,
        runs: run_ids.len(),
        saved_to,
        save_error,
    }
}

fn summary(loaded: &Loaded) -> SessionSummary<'_> {
    let session = &loaded.session;
    SessionSummary {
        name: &loaded.name,
        duration_s: session.duration_us as f64 / 1e6,
        scans: session.scans.len(),
        odometry: session.odometry.len(),
        imu: session.imu.len(),
        ground_truth: !session.ground_truth.is_empty(),
        car: session.info.car.as_deref(),
        map: session.info.map.as_deref(),
        true_map_error: loaded.true_map_error.as_deref(),
    }
}

fn run_summary<'a>(run: &'a Run, baseline: &BTreeMap<String, f64>) -> RunSummary<'a> {
    let changes = run
        .parameters
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
        RunState::Done(finished) => (1.0, None, Some(stats(finished))),
        RunState::Failed(err) => (0.0, Some(err.as_str()), None),
    };
    RunSummary {
        id: run.id,
        sweep: run.sweep,
        state: run.state.name(),
        progress,
        error,
        changes,
        stats,
    }
}

fn stats(finished: &Finished) -> RunStats<'_> {
    let outcome = &finished.outcome;
    let map = &outcome.map;
    RunStats {
        kept: outcome.kept.len(),
        processed: outcome.processed.len(),
        loop_closures: outcome.loop_closures,
        dropped: outcome.dropped_scans,
        paused: outcome.paused_scans,
        elapsed_s: outcome.elapsed.as_secs_f64(),
        map_width_m: f64::from(map.width_px) * map.resolution_m_per_px,
        map_height_m: f64::from(map.height_px) * map.resolution_m_per_px,
        scores: &finished.scores,
    }
}

#[derive(serde::Deserialize)]
struct DraftParameterBody {
    name: String,
    value: f64,
}

/// `POST /api/draft_parameter` - body `{"name": "...", "value": ...}` - sets
/// one of the draft's parameters, for the next run.
pub fn set_draft_parameter(request: &mut Request, lab: &Lab) -> ResponseBox {
    let body: DraftParameterBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    match lab.set_draft(&body.name, body.value) {
        Ok(()) => json_response(&(), 200),
        Err(err) => bad_request(&err),
    }
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

/// `POST /api/draft_parameters_save` - writes the draft into
/// `localization/slam.toml`, keeping its comments. Responds with its path.
pub fn save_draft(lab: &Lab) -> ResponseBox {
    let path = lab.slam_config_path();
    let parameters = lab.with(|view| parameters_with(view.draft));
    match aurorus::config::save_parameter_values(&path, &parameters) {
        Ok(()) => path_response(&path),
        Err(err) => error_response(500, &err),
    }
}

/// `POST /api/draft_parameters_load` - sets the draft to the values in
/// `localization/slam.toml`. Responds with its path.
pub fn load_draft(lab: &Lab) -> ResponseBox {
    let path = lab.slam_config_path();
    let parameters = lab.with(|view| parameters_with(view.draft));
    match aurorus::config::saved_parameter_values(&path, &parameters) {
        Ok(values) => {
            lab.set_draft_values(&values);
            path_response(&path)
        }
        Err(err) => error_response(500, &err),
    }
}

/// `POST /api/draft_reset` - sets the draft back to the session's baseline:
/// what it was recorded with.
pub fn reset_draft(lab: &Lab) -> ResponseBox {
    let Some(baseline) = lab.with(|view| view.session.map(|s| s.baseline.clone())) else {
        return bad_request("no session is loaded");
    };
    lab.set_draft_values(&baseline);
    json_response(&(), 200)
}

#[derive(serde::Serialize)]
struct QueuedBody {
    id: u64,
}

/// `POST /api/runs` - queues a run of the loaded session with the draft.
/// Responds with its id.
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

/// `POST /api/runs/cancel` - body `{"id": ...}`.
pub fn cancel_run(request: &mut Request, lab: &Lab) -> ResponseBox {
    run_action(request, |id| lab.cancel_run(id))
}

/// `POST /api/runs/delete` - body `{"id": ...}`.
pub fn delete_run(request: &mut Request, lab: &Lab) -> ResponseBox {
    run_action(request, |id| lab.delete_run(id))
}

/// `POST /api/runs/use` - body `{"id": ...}` - sets the draft to that run's
/// parameters.
pub fn use_run(request: &mut Request, lab: &Lab) -> ResponseBox {
    run_action(request, |id| {
        lab.set_draft_values(&lab.run_parameters(id)?);
        Ok(())
    })
}

fn run_action(
    request: &mut Request,
    action: impl FnOnce(u64) -> Result<(), String>,
) -> ResponseBox {
    let body: RunBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    match action(body.id) {
        Ok(()) => json_response(&(), 200),
        Err(err) => bad_request(&err),
    }
}

#[derive(serde::Deserialize)]
struct SweepBody {
    axes: Vec<AxisSpec>,
}

/// `POST /api/sweeps/plan` - body `{"axes": [{"name", "from", "to",
/// "count"}, ...]}` - what that sweep would queue: each axis' values, and
/// how many combinations would run and be skipped.
pub fn plan_sweep(request: &mut Request, lab: &Lab) -> ResponseBox {
    let body: SweepBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    match lab.plan_sweep(&body.axes) {
        Ok(plan) => json_response(&plan, 200),
        Err(err) => bad_request(&err),
    }
}

/// `POST /api/sweeps` - the same body - queues a run for every combination
/// SLAM can run with. Responds with the sweep's id.
pub fn queue_sweep(request: &mut Request, lab: &Lab) -> ResponseBox {
    let body: SweepBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    match lab.queue_sweep(&body.axes) {
        Ok(id) => json_response(&QueuedBody { id }, 200),
        Err(err) => bad_request(&err),
    }
}

/// `POST /api/sweeps/cancel` - body `{"id": ...}`.
pub fn cancel_sweep(request: &mut Request, lab: &Lab) -> ResponseBox {
    run_action(request, |id| lab.cancel_sweep(id))
}

/// `POST /api/sweeps/delete` - body `{"id": ...}` - forgets the sweep and
/// every run of it.
pub fn delete_sweep(request: &mut Request, lab: &Lab) -> ResponseBox {
    run_action(request, |id| lab.delete_sweep(id))
}

/// `GET /api/sweeps/csv?id=...` - the sweep's results as CSV, to download.
pub fn sweep_csv(url: &str, lab: &Lab) -> ResponseBox {
    let Some(id) = query_param(url, "id").and_then(|id| id.parse::<u64>().ok()) else {
        return bad_request("id must be a sweep's id");
    };
    match lab.sweep_csv(id) {
        Ok(csv) => Response::from_string(csv)
            .with_header(header("Content-Type", "text/csv; charset=utf-8"))
            .with_header(header(
                "Content-Disposition",
                &format!("attachment; filename=\"sweep{id}.csv\""),
            ))
            .boxed(),
        Err(err) => bad_request(&err),
    }
}

#[derive(serde::Deserialize)]
struct SaveMapBody {
    id: u64,
    name: String,
}

/// `POST /api/runs/save_map` - body `{"id": ..., "name": "..."}` - saves
/// that run's map as a new map folder under the maps folder, as **Save
/// map** does live. Responds with the folder.
pub fn save_map(request: &mut Request, lab: &Lab) -> ResponseBox {
    let body: SaveMapBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let Some(finished) = lab.finished(body.id) else {
        return bad_request(&format!("run {} isn't done", body.id));
    };
    match finished.outcome.save(&body.name, &lab.maps_root) {
        Ok(folder) => path_response(&folder),
        Err(err) => bad_request(&err),
    }
}

/// `POST /api/draw`'s body: the drawing protocol's, plus which runs the page
/// shows.
#[derive(serde::Deserialize)]
struct LabDrawRequest {
    #[serde(flatten)]
    request: DrawRequest,
    #[serde(default)]
    selected: Option<u64>,
    #[serde(default)]
    compared: Option<u64>,
}

/// `POST /api/draw` - the three layers (see the module docs). A run's
/// layer's version is the run's id - `0` for none - and the ground truth's
/// the session's generation.
pub fn draw(request: &mut Request, lab: &Lab, epoch: u64) -> ResponseBox {
    let body: LabDrawRequest = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let finished = |id: Option<u64>| id.and_then(|id| lab.finished(id).map(|f| (id, f)));
    let selected = finished(body.selected);
    let compared = finished(body.compared);
    let session = lab.with(|view| view.session.cloned());
    let anchor = session.as_ref().map(|s| s.anchor).unwrap_or_default();

    let layer = |topic: &str, version: u64, drawing: &dyn Fn() -> Drawing| DrawLayer {
        topic: topic.to_string(),
        writer: None,
        write_count: version,
        age_ms: Some(0.0),
        drawing: (!body.request.holds(epoch, topic, version))
            .then(|| without_raster_pixels(drawing())),
    };
    let mut layers = vec![
        layer(
            COMPARED_LAYER,
            compared.as_ref().map_or(0, |(id, _)| *id),
            &|| {
                compared
                    .as_ref()
                    .map(|(_, f)| f.comparison(&anchor).clone())
                    .unwrap_or_default()
            },
        ),
        layer(
            SELECTED_LAYER,
            selected.as_ref().map_or(0, |(id, _)| *id),
            &|| {
                selected
                    .as_ref()
                    .map(|(_, f)| f.drawing(&anchor).clone())
                    .unwrap_or_default()
            },
        ),
    ];
    if let Some(session) = &session
        && (session.true_walls.is_some() || !session.session.ground_truth.is_empty())
    {
        layers.push(layer(TRUTH_LAYER, session.generation, &|| {
            truth_drawing(session)
        }));
    }
    json_response(&DrawResponse { epoch, layers }, 200)
}

/// The true map's walls and the true trajectory of `session`.
fn truth_drawing(session: &Arc<Loaded>) -> Drawing {
    let trajectory = session
        .session
        .ground_truth
        .iter()
        .step_by(TRUTH_EVERY)
        .map(|(_, truth)| [truth.x_m as f32, truth.y_m as f32])
        .collect();
    Drawing::default()
        .element(
            "True walls",
            [Shape::Points {
                points: session.true_walls.clone().unwrap_or_default(),
                radius_px: 1.0,
                color: TRUTH_COLOR,
            }],
            true,
        )
        .element(
            "True trajectory",
            [Shape::Polyline {
                points: trajectory,
                closed: false,
                width_px: 1.5,
                color: TRUTH_COLOR,
            }],
            true,
        )
        .z_index(TRUTH_Z_INDEX)
}

/// `GET /api/draw/raster?...` - the selected run's map raster: its layer's
/// version is the run's id.
pub fn draw_raster(url: &str, lab: &Lab, epoch: u64) -> ResponseBox {
    let query = match RasterQuery::parse(url) {
        Ok(query) => query,
        Err(response) => return response,
    };
    if query.epoch != epoch {
        return stale_raster();
    }
    if query.topic != SELECTED_LAYER {
        return not_found();
    }
    let session = lab.with(|view| view.session.cloned());
    match (session, lab.finished(query.write_count)) {
        (Some(session), Some(finished)) => {
            raster_response(finished.drawing(&session.anchor), query.shape)
        }
        _ => not_found(),
    }
}
