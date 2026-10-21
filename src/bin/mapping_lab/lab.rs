//! The lab's state: the mapping session loaded, the draft parameters the
//! next run gets, every run of that session - queued, running on one of
//! the worker threads (see [`run_workers`]), or done - and the sweeps that
//! queued many of them at once (see [`crate::sweep`]).

use crate::sweep::{self, Axis, AxisSpec, CsvRow};
use aurorus::localization::session::{self, Session, SessionKind};
use aurorus::localization::{
    RebuildOutcome, Scores, SlamConfig, map_walls, rebuild, score, slam_tunable_parameters,
};
use aurorus::topics::{AlgorithmParameter, Color, Drawing, StartState};
use std::collections::BTreeMap;
use std::ops::ControlFlow;
use std::panic::AssertUnwindSafe;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, OnceLock};
use std::time::Instant;

/// The color a compared run's walls and trajectory are drawn in.
const COMPARED_COLOR: Color = Color::AMBER;

/// A session loaded into the lab.
pub struct Loaded {
    /// Its file name, under the sessions folder.
    pub name: String,
    /// Where it was read from.
    pub path: PathBuf,
    pub session: Session,
    /// `config/localization/slam.toml` with the session's recorded parameter
    /// values applied: what every run starts from.
    pub base: SlamConfig,
    /// The value of every tunable parameter in `base`.
    pub baseline: BTreeMap<String, f64>,
    /// Where SLAM's frame sat in the world - see [`Session::anchor`].
    pub anchor: StartState,
    /// The walls of the map the session was recorded on, if it names one
    /// that loads - in simulation, the true map.
    pub true_walls: Option<Vec<[f32; 2]>>,
    /// Why the recorded map didn't load, if it names one.
    pub true_map_error: Option<String>,
    /// Bumped on every load, so drawings of different sessions never share
    /// a version.
    pub generation: u64,
}

/// What a run is at.
pub enum RunState {
    Queued,
    Running { progress: f64 },
    Done(Arc<Finished>),
    Failed(String),
    Cancelled,
}

impl RunState {
    /// Whether the run won't change any more.
    pub fn is_over(&self) -> bool {
        !matches!(self, Self::Queued | Self::Running { .. })
    }

    /// `queued`, `running`, `done`, `failed` or `cancelled`.
    pub fn name(&self) -> &'static str {
        match self {
            Self::Queued => "queued",
            Self::Running { .. } => "running",
            Self::Done(_) => "done",
            Self::Failed(_) => "failed",
            Self::Cancelled => "cancelled",
        }
    }
}

/// A finished run's map and its scores, computed on the worker - and its
/// drawings, built the first time the run is shown: a sweep's runs mostly
/// never are.
pub struct Finished {
    pub outcome: RebuildOutcome,
    pub scores: Scores,
    drawing: OnceLock<Drawing>,
    comparison: OnceLock<Drawing>,
}

impl Finished {
    pub fn new(outcome: RebuildOutcome, scores: Scores) -> Self {
        Self {
            outcome,
            scores,
            drawing: OnceLock::new(),
            comparison: OnceLock::new(),
        }
    }

    /// The map as live SLAM draws it - see [`RebuildOutcome::drawing`].
    pub fn drawing(&self, anchor: &StartState) -> &Drawing {
        self.drawing.get_or_init(|| self.outcome.drawing(anchor))
    }

    /// The walls and trajectory laid over another run's map.
    pub fn comparison(&self, anchor: &StartState) -> &Drawing {
        self.comparison
            .get_or_init(|| self.outcome.comparison_drawing(anchor, COMPARED_COLOR))
    }
}

/// One rebuild of the loaded session.
pub struct Run {
    /// Unique across sessions, from `1`.
    pub id: u64,
    /// Every tunable parameter's value.
    pub parameters: BTreeMap<String, f64>,
    pub state: RunState,
    /// The sweep that queued it, if one did.
    pub sweep: Option<u64>,
    cancel: Arc<AtomicBool>,
}

/// Many runs queued at once: one per combination of the swept parameters'
/// values, the rest of the parameters as in the draft when it was queued.
pub struct Sweep {
    /// Unique across sessions, from `1`.
    pub id: u64,
    pub axes: Vec<Axis>,
    /// Every combination, the last axis changing fastest.
    pub cells: Vec<SweepCell>,
    /// Where its results were written once every run was over - or why
    /// they couldn't be. `None` until then.
    pub saved: Option<Result<PathBuf, String>>,
}

/// One combination of a sweep.
pub struct SweepCell {
    /// An index into each axis' values.
    pub cell: Vec<usize>,
    /// The run built with it - or why SLAM can't run with it.
    pub run: Result<u64, String>,
}

impl Sweep {
    /// The swept values of `cell`.
    pub fn values(&self, cell: &[usize]) -> Vec<f64> {
        self.axes
            .iter()
            .zip(cell)
            .map(|(axis, &i)| axis.values[i])
            .collect()
    }

    /// Its results as CSV - see [`sweep::csv`].
    pub fn csv(&self, runs: &[Run]) -> String {
        let rows: Vec<_> = self
            .cells
            .iter()
            .map(|cell| {
                let values = self.values(&cell.cell);
                match &cell.run {
                    Err(reason) => CsvRow {
                        run: None,
                        values,
                        state: "skipped",
                        scores: None,
                        note: Some(reason),
                    },
                    Ok(id) => {
                        let run = runs.iter().find(|run| run.id == *id);
                        CsvRow {
                            run: Some(*id),
                            values,
                            state: run.map_or("deleted", |run| run.state.name()),
                            scores: run.and_then(|run| match &run.state {
                                RunState::Done(finished) => Some(&finished.scores),
                                _ => None,
                            }),
                            note: run.and_then(|run| match &run.state {
                                RunState::Failed(err) => Some(err.as_str()),
                                _ => None,
                            }),
                        }
                    }
                }
            })
            .collect();
        sweep::csv(&self.axes, &rows)
    }
}

/// A sweep about to be queued: its axes, and how many of its combinations
/// would run and how many SLAM can't run with.
#[derive(serde::Serialize)]
pub struct SweepPlan {
    pub axes: Vec<Axis>,
    pub runs: usize,
    pub skipped: usize,
}

/// What [`Lab::with`] shows.
pub struct View<'a> {
    pub session: Option<&'a Arc<Loaded>>,
    pub draft: &'a BTreeMap<String, f64>,
    /// Oldest first.
    pub runs: &'a [Run],
    /// Oldest first.
    pub sweeps: &'a [Sweep],
    /// Why the last load failed.
    pub load_error: Option<&'a str>,
}

#[derive(Default)]
struct Inner {
    session: Option<Arc<Loaded>>,
    /// Why the last session couldn't be loaded.
    load_error: Option<String>,
    /// The parameter values the next run gets.
    draft: BTreeMap<String, f64>,
    /// Newest last.
    runs: Vec<Run>,
    next_run_id: u64,
    /// Newest last.
    sweeps: Vec<Sweep>,
    next_sweep_id: u64,
    generation: u64,
}

/// Everything the lab knows, shared between the HTTP workers and the run
/// workers.
pub struct Lab {
    pub sessions_root: PathBuf,
    pub maps_root: PathBuf,
    /// Where `localization/slam.toml` is read from and saved to.
    pub config_dir: PathBuf,
    /// How many runs are built at once - see [`run_workers`].
    pub workers: usize,
    inner: Mutex<Inner>,
    /// Signalled whenever a run is queued.
    work: Condvar,
}

impl Lab {
    pub fn new(
        sessions_root: PathBuf,
        maps_root: PathBuf,
        config_dir: PathBuf,
        workers: usize,
    ) -> Self {
        Self {
            sessions_root,
            maps_root,
            config_dir,
            workers,
            inner: Mutex::new(Inner {
                next_run_id: 1,
                next_sweep_id: 1,
                ..Inner::default()
            }),
            work: Condvar::new(),
        }
    }

    fn inner(&self) -> MutexGuard<'_, Inner> {
        self.inner
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    /// `config/localization/slam.toml`, under [`Self::config_dir`].
    pub fn slam_config_path(&self) -> PathBuf {
        self.config_dir.join("localization").join("slam.toml")
    }

    /// The `.mapping` files under [`Self::sessions_root`], by name, with
    /// their sizes in bytes - newest first.
    pub fn session_files(&self) -> Vec<(String, u64)> {
        session::list(&self.sessions_root, SessionKind::Mapping)
    }

    /// Loads the session file `name` under [`Self::sessions_root`] - or the
    /// file at `path` itself, given one - in place of the current one:
    /// every run of the current one is cancelled and forgotten, and the
    /// draft goes back to the new one's baseline.
    pub fn load(&self, name: &str, path: Option<&Path>) -> Result<(), String> {
        match self.read_session(name, path) {
            Ok(loaded) => {
                self.install(loaded);
                Ok(())
            }
            Err(err) => {
                self.inner().load_error = Some(err.clone());
                Err(err)
            }
        }
    }

    /// Makes `loaded` the current session - see [`Self::load`].
    fn install(&self, loaded: Loaded) {
        let mut inner = self.inner();
        for run in &inner.runs {
            run.cancel.store(true, Ordering::Relaxed);
        }
        inner.runs.clear();
        inner.sweeps.clear();
        inner.generation += 1;
        inner.draft = loaded.baseline.clone();
        inner.load_error = None;
        inner.session = Some(Arc::new(Loaded {
            generation: inner.generation,
            ..loaded
        }));
    }

    fn read_session(&self, name: &str, path: Option<&Path>) -> Result<Loaded, String> {
        let path = match path {
            Some(path) => path.to_path_buf(),
            None => {
                if name.is_empty() || name.starts_with('.') || name.contains(['/', '\\']) {
                    return Err(format!("{name:?} isn't a session file name"));
                }
                self.sessions_root.join(name)
            }
        };
        let session = Session::load(&path)?;
        let file_config = aurorus::config::load::<SlamConfig>(&self.slam_config_path())
            .unwrap_or_else(|_| SlamConfig::default());
        let base = file_config
            .with_parameters(&session.info.slam_parameters)
            .map_err(|err| format!("the session's parameters can't be run: {err}"))?;
        // The map embedded in the session, or the one it names if it
        // predates embedding.
        let (true_walls, true_map_error) = match session.map_folder() {
            Ok(Some(folder)) => match map_walls(&folder) {
                Ok(walls) => (Some(walls), None),
                Err(err) => (None, Some(err)),
            },
            Ok(None) if session.info.map.is_some() => (
                None,
                Some("not embedded in the session, and no longer in maps/".to_string()),
            ),
            Ok(None) => (None, None),
            Err(err) => (None, Some(err)),
        };
        Ok(Loaded {
            name: path
                .file_name()
                .map(|name| name.to_string_lossy().into_owned())
                .unwrap_or_default(),
            path,
            anchor: session.anchor(),
            baseline: values_of(&base),
            base,
            session,
            true_walls,
            true_map_error,
            generation: 0,
        })
    }

    /// Runs `f` on what the lab holds - under the lock, so keep it short.
    pub fn with<T>(&self, f: impl FnOnce(View) -> T) -> T {
        let inner = self.inner();
        f(View {
            session: inner.session.as_ref(),
            draft: &inner.draft,
            runs: &inner.runs,
            sweeps: &inner.sweeps,
            load_error: inner.load_error.as_deref(),
        })
    }

    /// Sets the draft's `name` to `value`, brought into its range.
    pub fn set_draft(&self, name: &str, value: f64) -> Result<(), String> {
        let parameter = slam_tunable_parameters()
            .into_iter()
            .find(|parameter| parameter.name == name)
            .ok_or_else(|| format!("SLAM has no tunable parameter {name:?}"))?;
        let value = parameter
            .kind
            .sanitize(value)
            .ok_or("value must be a finite number")?;
        self.inner().draft.insert(name.to_string(), value);
        Ok(())
    }

    /// Replaces the draft's values with `values` - only the ones of tunable
    /// parameters, each brought into its range.
    pub fn set_draft_values(&self, values: &BTreeMap<String, f64>) {
        let mut inner = self.inner();
        for parameter in slam_tunable_parameters() {
            if let Some(value) = values
                .get(&parameter.name)
                .and_then(|&value| parameter.kind.sanitize(value))
            {
                inner.draft.insert(parameter.name, value);
            }
        }
    }

    /// Queues a run of the loaded session with the draft - or says why it
    /// can't be run. Returns its id.
    pub fn queue_run(&self) -> Result<u64, String> {
        let mut inner = self.inner();
        let session = inner.session.clone().ok_or("no session is loaded")?;
        session.base.with_parameters(&inner.draft)?;
        let id = inner.next_run_id;
        inner.next_run_id += 1;
        let parameters = inner.draft.clone();
        inner.runs.push(Run {
            id,
            parameters,
            state: RunState::Queued,
            sweep: None,
            cancel: Arc::default(),
        });
        drop(inner);
        self.work.notify_one();
        Ok(id)
    }

    /// The axes of a sweep over `specs` and every combination of their
    /// values: each one's parameters (the draft's, with the swept values),
    /// or why SLAM can't run with them.
    #[allow(clippy::type_complexity)]
    fn sweep_cells(
        inner: &Inner,
        specs: &[AxisSpec],
    ) -> Result<
        (
            Vec<Axis>,
            Vec<(Vec<usize>, Result<BTreeMap<String, f64>, String>)>,
        ),
        String,
    > {
        let session = inner.session.as_ref().ok_or("no session is loaded")?;
        let axes = sweep::axes(specs, &slam_tunable_parameters())?;
        let cells = sweep::grid(&axes)
            .into_iter()
            .map(|cell| {
                let mut parameters = inner.draft.clone();
                for (axis, &i) in axes.iter().zip(&cell) {
                    parameters.insert(axis.spec.name.clone(), axis.values[i]);
                }
                let parameters = session
                    .base
                    .with_parameters(&parameters)
                    .map(|_| parameters);
                (cell, parameters)
            })
            .collect();
        Ok((axes, cells))
    }

    /// What a sweep over `specs` would queue - without queueing it.
    pub fn plan_sweep(&self, specs: &[AxisSpec]) -> Result<SweepPlan, String> {
        let (axes, cells) = Self::sweep_cells(&self.inner(), specs)?;
        let runs = cells.iter().filter(|(_, p)| p.is_ok()).count();
        Ok(SweepPlan {
            axes,
            runs,
            skipped: cells.len() - runs,
        })
    }

    /// Queues a run for every combination of `specs`' values SLAM can run
    /// with - behind any run queued by hand, which goes first. Returns the
    /// sweep's id.
    pub fn queue_sweep(&self, specs: &[AxisSpec]) -> Result<u64, String> {
        let mut inner = self.inner();
        let (axes, cells) = Self::sweep_cells(&inner, specs)?;
        if cells.iter().all(|(_, parameters)| parameters.is_err()) {
            let reason = cells
                .into_iter()
                .find_map(|(_, parameters)| parameters.err())
                .unwrap_or_default();
            return Err(format!("SLAM can't run with any combination: {reason}"));
        }
        let id = inner.next_sweep_id;
        inner.next_sweep_id += 1;
        let mut sweep_cells = Vec::with_capacity(cells.len());
        for (cell, parameters) in cells {
            let run = parameters.map(|parameters| {
                let run_id = inner.next_run_id;
                inner.next_run_id += 1;
                inner.runs.push(Run {
                    id: run_id,
                    parameters,
                    state: RunState::Queued,
                    sweep: Some(id),
                    cancel: Arc::default(),
                });
                run_id
            });
            sweep_cells.push(SweepCell { cell, run });
        }
        inner.sweeps.push(Sweep {
            id,
            axes,
            cells: sweep_cells,
            saved: None,
        });
        drop(inner);
        self.work.notify_all();
        Ok(id)
    }

    /// Stops every run of sweep `id` still queued or running.
    pub fn cancel_sweep(&self, id: u64) -> Result<(), String> {
        let mut inner = self.inner();
        find_sweep(&inner.sweeps, id)?;
        for run in inner.runs.iter_mut().filter(|run| run.sweep == Some(id)) {
            run.cancel.store(true, Ordering::Relaxed);
            if !run.state.is_over() {
                run.state = RunState::Cancelled;
            }
        }
        drop(inner);
        self.save_finished_sweeps();
        Ok(())
    }

    /// Forgets sweep `id` and every run of it, cancelling them first.
    pub fn delete_sweep(&self, id: u64) -> Result<(), String> {
        let mut inner = self.inner();
        find_sweep(&inner.sweeps, id)?;
        for run in inner.runs.iter().filter(|run| run.sweep == Some(id)) {
            run.cancel.store(true, Ordering::Relaxed);
        }
        inner.runs.retain(|run| run.sweep != Some(id));
        inner.sweeps.retain(|sweep| sweep.id != id);
        Ok(())
    }

    /// Sweep `id`'s results as CSV.
    pub fn sweep_csv(&self, id: u64) -> Result<String, String> {
        let inner = self.inner();
        Ok(find_sweep(&inner.sweeps, id)?.csv(&inner.runs))
    }

    /// Writes the results of every sweep whose runs are all over, and that
    /// wasn't written yet, next to the session: `<session>.sweeps/` beside
    /// `<session>.mapping`.
    fn save_finished_sweeps(&self) {
        let mut writes = Vec::new();
        {
            let mut inner = self.inner();
            let Some(session) = inner.session.clone() else {
                return;
            };
            let Inner { sweeps, runs, .. } = &mut *inner;
            for sweep in sweeps.iter_mut().filter(|sweep| sweep.saved.is_none()) {
                let over = sweep.cells.iter().all(|cell| match cell.run {
                    Ok(id) => runs
                        .iter()
                        .find(|run| run.id == id)
                        .is_none_or(|run| run.state.is_over()),
                    Err(_) => true,
                });
                if over {
                    // Marked now, so two workers don't both write it.
                    sweep.saved = Some(Err("being written".to_string()));
                    writes.push((
                        sweep.id,
                        sweep_csv_path(&session.path, sweep.id),
                        sweep.csv(runs),
                    ));
                }
            }
        }
        for (id, path, csv) in writes {
            let saved = path
                .parent()
                .map_or(Ok(()), std::fs::create_dir_all)
                .and_then(|()| std::fs::write(&path, csv))
                .map(|()| path.clone())
                .map_err(|err| format!("couldn't write {}: {err}", path.display()));
            match &saved {
                Ok(path) => println!("mapping_lab: sweep {id}'s results written to {path:?}"),
                Err(err) => eprintln!("mapping_lab: {err}"),
            }
            if let Some(sweep) = self.inner().sweeps.iter_mut().find(|sweep| sweep.id == id) {
                sweep.saved = Some(saved);
            }
        }
    }

    /// Stops run `id` if it's queued or running.
    pub fn cancel_run(&self, id: u64) -> Result<(), String> {
        let mut inner = self.inner();
        let run = find(&mut inner.runs, id)?;
        run.cancel.store(true, Ordering::Relaxed);
        if !run.state.is_over() {
            run.state = RunState::Cancelled;
        }
        drop(inner);
        self.save_finished_sweeps();
        Ok(())
    }

    /// Forgets run `id`, cancelling it first.
    pub fn delete_run(&self, id: u64) -> Result<(), String> {
        let mut inner = self.inner();
        find(&mut inner.runs, id)?
            .cancel
            .store(true, Ordering::Relaxed);
        inner.runs.retain(|run| run.id != id);
        drop(inner);
        self.save_finished_sweeps();
        Ok(())
    }

    /// Run `id`'s parameter values.
    pub fn run_parameters(&self, id: u64) -> Result<BTreeMap<String, f64>, String> {
        let mut inner = self.inner();
        Ok(find(&mut inner.runs, id)?.parameters.clone())
    }

    /// Run `id`'s map and drawings, once it's done.
    pub fn finished(&self, id: u64) -> Option<Arc<Finished>> {
        let inner = self.inner();
        inner
            .runs
            .iter()
            .find(|run| run.id == id)
            .and_then(|run| match &run.state {
                RunState::Done(finished) => Some(finished.clone()),
                _ => None,
            })
    }

    /// The next queued run, marked running: its id, the session, its
    /// config and its cancel flag. Runs queued by hand go before a sweep's.
    /// Waits for one.
    fn next_job(&self) -> (u64, Arc<Loaded>, SlamConfig, Arc<AtomicBool>) {
        let mut inner = self.inner();
        loop {
            let session = inner.session.clone();
            let queued = |run: &Run| matches!(run.state, RunState::Queued);
            let next = inner
                .runs
                .iter()
                .position(|run| queued(run) && run.sweep.is_none())
                .or_else(|| inner.runs.iter().position(queued));
            if let Some(session) = session
                && let Some(next) = next
            {
                let run = &mut inner.runs[next];
                run.state = RunState::Running { progress: 0.0 };
                let config = session
                    .base
                    .with_parameters(&run.parameters)
                    .expect("checked when queued");
                return (run.id, session, config, run.cancel.clone());
            }
            inner = self
                .work
                .wait(inner)
                .unwrap_or_else(|poisoned| poisoned.into_inner());
        }
    }

    /// Sets run `id`'s state, unless it was cancelled or forgotten since.
    fn update(&self, id: u64, state: RunState) {
        let mut inner = self.inner();
        if let Some(run) = inner.runs.iter_mut().find(|run| run.id == id)
            && !run.state.is_over()
        {
            run.state = state;
        }
    }
}

/// Sweep `id` among `sweeps`, or an error naming it.
fn find_sweep(sweeps: &[Sweep], id: u64) -> Result<&Sweep, String> {
    sweeps
        .iter()
        .find(|sweep| sweep.id == id)
        .ok_or_else(|| format!("there's no sweep {id}"))
}

/// Where sweep `id`'s results are written: `lap.sweeps/<when>_sweep<id>.csv`
/// beside the session file `lap.mapping`.
fn sweep_csv_path(session: &Path, id: u64) -> PathBuf {
    let stem = session
        .file_stem()
        .map(|stem| stem.to_string_lossy().into_owned())
        .unwrap_or_default();
    let generated = aurorus::debug_format::DebugFileReader::generated_filename();
    let when = generated.strip_suffix(".debug").unwrap_or(&generated);
    session
        .with_file_name(format!("{stem}.sweeps"))
        .join(format!("{when}_sweep{id}.csv"))
}

/// Run `id` among `runs`, or an error naming it.
fn find(runs: &mut [Run], id: u64) -> Result<&mut Run, String> {
    runs.iter_mut()
        .find(|run| run.id == id)
        .ok_or_else(|| format!("there's no run {id}"))
}

/// Every tunable parameter's value in `config`.
fn values_of(config: &SlamConfig) -> BTreeMap<String, f64> {
    let mut parameters = slam_tunable_parameters();
    aurorus::config::refresh_parameter_values(&mut parameters, config);
    parameters
        .into_iter()
        .map(|parameter| (parameter.name, parameter.value))
        .collect()
}

/// Every tunable parameter, with its value in `values`.
pub fn parameters_with(values: &BTreeMap<String, f64>) -> Vec<AlgorithmParameter> {
    let mut parameters = slam_tunable_parameters();
    for parameter in &mut parameters {
        parameter.value = values.get(&parameter.name).copied().unwrap_or_default();
    }
    parameters
}

/// Runs `count` worker threads, each building one queued run after the
/// other - forever.
pub fn run_workers(lab: &Arc<Lab>, count: usize) {
    for _ in 0..count {
        let lab = lab.clone();
        std::thread::spawn(move || {
            loop {
                let (id, session, config, cancel) = lab.next_job();
                let started = Instant::now();
                // A panic fails this run only, not the worker.
                let outcome = std::panic::catch_unwind(AssertUnwindSafe(|| {
                    rebuild(&session.session, &config, |progress| {
                        if cancel.load(Ordering::Relaxed) {
                            return ControlFlow::Break(());
                        }
                        lab.update(id, RunState::Running { progress });
                        ControlFlow::Continue(())
                    })
                }));
                match outcome {
                    Err(panic) => {
                        let message = panic
                            .downcast_ref::<String>()
                            .cloned()
                            .or_else(|| panic.downcast_ref::<&str>().map(|s| s.to_string()))
                            .unwrap_or_else(|| "the rebuild panicked".to_string());
                        lab.update(id, RunState::Failed(message));
                    }
                    Ok(Some(outcome)) => {
                        let scores =
                            score(&session.session, &outcome, session.true_walls.as_deref());
                        println!(
                            "mapping_lab: run {id} done in {:.1} s",
                            started.elapsed().as_secs_f64()
                        );
                        lab.update(id, RunState::Done(Arc::new(Finished::new(outcome, scores))));
                    }
                    Ok(None) => lab.update(id, RunState::Cancelled),
                }
                lab.save_finished_sweeps();
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_baseline_value_is_a_tunable_parameter() {
        let values = values_of(&SlamConfig::default());
        assert_eq!(values.len(), slam_tunable_parameters().len());
        let parameters = parameters_with(&values);
        assert!(parameters.iter().all(|p| values[&p.name] == p.value));
    }

    #[test]
    fn a_run_needs_a_session() {
        let lab = Lab::new(PathBuf::new(), PathBuf::new(), PathBuf::new(), 1);
        assert!(lab.queue_run().is_err());
    }

    #[test]
    fn the_draft_only_takes_tunable_parameters_in_range() {
        let lab = Lab::new(PathBuf::new(), PathBuf::new(), PathBuf::new(), 1);
        assert!(lab.set_draft("nope", 1.0).is_err());
        lab.set_draft("resolution_m", 99.0).unwrap();
        lab.with(|view| assert_eq!(view.draft["resolution_m"], 0.5));
    }

    /// A lab with an empty session loaded, recorded with SLAM's defaults.
    fn lab_with_session(root: &Path) -> Lab {
        let lab = Lab::new(root.to_path_buf(), PathBuf::new(), PathBuf::new(), 1);
        let base = SlamConfig::default();
        lab.install(Loaded {
            name: "lap.mapping".to_string(),
            path: root.join("lap.mapping"),
            session: Session::default(),
            baseline: values_of(&base),
            base,
            anchor: StartState::default(),
            true_walls: None,
            true_map_error: None,
            generation: 0,
        });
        lab
    }

    fn spec(name: &str, from: f64, to: f64, count: usize) -> AxisSpec {
        AxisSpec {
            name: name.to_string(),
            from,
            to,
            count,
        }
    }

    #[test]
    fn a_sweep_queues_a_run_per_combination_after_runs_by_hand() {
        let lab = lab_with_session(Path::new("unused"));
        let id = lab
            .queue_sweep(&[
                spec("loop_search_maximum_distance_m", 2.0, 4.0, 2),
                spec("minimum_travel_distance_m", 0.1, 0.3, 3),
            ])
            .unwrap();
        let manual = lab.queue_run().unwrap();
        lab.with(|view| {
            let sweep = &view.sweeps[0];
            assert_eq!(sweep.id, id);
            assert_eq!(sweep.cells.len(), 6);
            let runs: Vec<_> = view.runs.iter().filter(|r| r.sweep == Some(id)).collect();
            assert_eq!(runs.len(), 6);
            // The last axis changes fastest; the rest is the draft's.
            assert_eq!(runs[1].parameters["loop_search_maximum_distance_m"], 2.0);
            assert_eq!(runs[1].parameters["minimum_travel_distance_m"], 0.2);
            assert_eq!(
                runs[1].parameters["resolution_m"],
                view.draft["resolution_m"]
            );
        });
        assert_eq!(lab.next_job().0, manual);
    }

    #[test]
    fn combinations_slam_cant_run_are_skipped() {
        let lab = lab_with_session(Path::new("unused"));
        // A smear deviation past ten times its search's resolution fails.
        let r = SlamConfig::default().correlation_search_space_resolution_m;
        let name = "correlation_search_space_smear_deviation_m";
        let plan = lab.plan_sweep(&[spec(name, 8.0 * r, 12.0 * r, 5)]).unwrap();
        assert_eq!((plan.runs, plan.skipped), (3, 2));
        let id = lab
            .queue_sweep(&[spec(name, 8.0 * r, 12.0 * r, 5)])
            .unwrap();
        lab.with(|view| {
            let cells = &view.sweeps[0].cells;
            assert!(cells[..3].iter().all(|cell| cell.run.is_ok()));
            assert!(cells[3..].iter().all(|cell| cell.run.is_err()));
        });
        assert!(
            lab.queue_sweep(&[spec(name, 11.0 * r, 12.0 * r, 2)])
                .is_err()
        );
        lab.delete_sweep(id).unwrap();
        lab.with(|view| assert!(view.runs.is_empty() && view.sweeps.is_empty()));
    }

    #[test]
    fn a_sweep_is_written_next_to_the_session_once_over() {
        let root = std::env::temp_dir().join(format!("mapping_lab_sweep_{}", std::process::id()));
        let lab = lab_with_session(&root);
        let id = lab
            .queue_sweep(&[spec("minimum_travel_distance_m", 0.1, 0.2, 2)])
            .unwrap();
        lab.with(|view| assert!(view.sweeps[0].saved.is_none()));
        lab.cancel_sweep(id).unwrap();
        let path = lab
            .with(|view| view.sweeps[0].saved.clone())
            .unwrap()
            .unwrap();
        assert!(path.starts_with(root.join("lap.sweeps")));
        let csv = std::fs::read_to_string(&path).unwrap();
        assert_eq!(csv.lines().count(), 3);
        assert!(csv.lines().nth(1).unwrap().contains(",cancelled,"));
        let _ = std::fs::remove_dir_all(&root);
    }
}
