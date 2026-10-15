//! The lab's state: the mapping session loaded, the draft parameters the
//! next run gets, and every run of that session - queued, running on one of
//! the worker threads (see [`run_workers`]), or done.

use aurorus::localization::mapping_session::{MAPPING_SESSION_EXTENSION, MappingSession};
use aurorus::localization::{
    RebuildOutcome, SlamConfig, map_walls, rebuild, slam_tunable_parameters,
};
use aurorus::topics::{AlgorithmParameter, Color, Drawing, StartState};
use std::collections::BTreeMap;
use std::ops::ControlFlow;
use std::panic::AssertUnwindSafe;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::time::Instant;

/// The color a compared run's walls and trajectory are drawn in.
const COMPARED_COLOR: Color = Color::AMBER;

/// A session loaded into the lab.
pub struct Loaded {
    /// Its file name, under the sessions folder.
    pub name: String,
    pub session: MappingSession,
    /// `config/localization/slam.toml` with the session's recorded parameter
    /// values applied: what every run starts from.
    pub base: SlamConfig,
    /// The value of every tunable parameter in `base`.
    pub baseline: BTreeMap<String, f64>,
    /// Where SLAM's frame sat in the world - see [`MappingSession::anchor`].
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

/// A finished run's map, and its drawings - computed once, on the worker.
pub struct Finished {
    pub outcome: RebuildOutcome,
    pub drawing: Drawing,
    pub comparison: Drawing,
}

/// One rebuild of the loaded session.
pub struct Run {
    /// Unique across sessions, from `1`.
    pub id: u64,
    /// Every tunable parameter's value.
    pub parameters: BTreeMap<String, f64>,
    pub state: RunState,
    cancel: Arc<AtomicBool>,
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
    generation: u64,
}

/// Everything the lab knows, shared between the HTTP workers and the run
/// workers.
pub struct Lab {
    pub sessions_root: PathBuf,
    pub maps_root: PathBuf,
    /// Where `localization/slam.toml` is read from and saved to.
    pub config_dir: PathBuf,
    inner: Mutex<Inner>,
    /// Signalled whenever a run is queued.
    work: Condvar,
}

impl Lab {
    pub fn new(sessions_root: PathBuf, maps_root: PathBuf, config_dir: PathBuf) -> Self {
        Self {
            sessions_root,
            maps_root,
            config_dir,
            inner: Mutex::new(Inner {
                next_run_id: 1,
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
        let Ok(entries) = std::fs::read_dir(&self.sessions_root) else {
            return Vec::new();
        };
        let mut files: Vec<_> = entries
            .filter_map(Result::ok)
            .filter_map(|entry| {
                let name = entry.file_name().to_string_lossy().into_owned();
                let metadata = entry.metadata().ok()?;
                (metadata.is_file() && name.ends_with(MAPPING_SESSION_EXTENSION))
                    .then(|| (name, metadata.len(), metadata.modified().ok()))
            })
            .collect();
        files.sort_by(|a, b| b.2.cmp(&a.2).then_with(|| a.0.cmp(&b.0)));
        files
            .into_iter()
            .map(|(name, size, _)| (name, size))
            .collect()
    }

    /// Loads the session file `name` under [`Self::sessions_root`] - or the
    /// file at `path` itself, given one - in place of the current one:
    /// every run of the current one is cancelled and forgotten, and the
    /// draft goes back to the new one's baseline.
    pub fn load(&self, name: &str, path: Option<&Path>) -> Result<(), String> {
        let loaded = self.read_session(name, path);
        let mut inner = self.inner();
        let loaded = match loaded {
            Ok(loaded) => loaded,
            Err(err) => {
                inner.load_error = Some(err.clone());
                return Err(err);
            }
        };
        for run in &inner.runs {
            run.cancel.store(true, Ordering::Relaxed);
        }
        inner.runs.clear();
        inner.generation += 1;
        inner.draft = loaded.baseline.clone();
        inner.load_error = None;
        inner.session = Some(Arc::new(Loaded {
            generation: inner.generation,
            ..loaded
        }));
        Ok(())
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
        let session = MappingSession::load(&path)?;
        let file_config = aurorus::config::load::<SlamConfig>(&self.slam_config_path())
            .unwrap_or_else(|_| SlamConfig::default());
        let base = file_config
            .with_parameters(&session.info.slam_parameters)
            .map_err(|err| format!("the session's parameters can't be run: {err}"))?;
        let (true_walls, true_map_error) = match &session.info.map {
            Some(map) => match map_walls(Path::new(map)) {
                Ok(walls) => (Some(walls), None),
                Err(err) => (None, Some(err)),
            },
            None => (None, None),
        };
        Ok(Loaded {
            name: path
                .file_name()
                .map(|name| name.to_string_lossy().into_owned())
                .unwrap_or_default(),
            anchor: session.anchor(),
            baseline: values_of(&base),
            base,
            session,
            true_walls,
            true_map_error,
            generation: 0,
        })
    }

    /// Runs `f` on the current session (if any), the draft, the runs, and
    /// why the last load failed - under the lock, so keep it short.
    pub fn with<T>(
        &self,
        f: impl FnOnce(Option<&Arc<Loaded>>, &BTreeMap<String, f64>, &[Run], Option<&str>) -> T,
    ) -> T {
        let inner = self.inner();
        f(
            inner.session.as_ref(),
            &inner.draft,
            &inner.runs,
            inner.load_error.as_deref(),
        )
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
            cancel: Arc::default(),
        });
        drop(inner);
        self.work.notify_one();
        Ok(id)
    }

    /// Stops run `id` if it's queued or running.
    pub fn cancel_run(&self, id: u64) -> Result<(), String> {
        let mut inner = self.inner();
        let run = find(&mut inner.runs, id)?;
        run.cancel.store(true, Ordering::Relaxed);
        if matches!(run.state, RunState::Queued | RunState::Running { .. }) {
            run.state = RunState::Cancelled;
        }
        Ok(())
    }

    /// Forgets run `id`, cancelling it first.
    pub fn delete_run(&self, id: u64) -> Result<(), String> {
        let mut inner = self.inner();
        find(&mut inner.runs, id)?
            .cancel
            .store(true, Ordering::Relaxed);
        inner.runs.retain(|run| run.id != id);
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
    /// config and its cancel flag. Waits for one.
    fn next_job(&self) -> (u64, Arc<Loaded>, SlamConfig, Arc<AtomicBool>) {
        let mut inner = self.inner();
        loop {
            let session = inner.session.clone();
            if let Some(session) = session
                && let Some(run) = inner
                    .runs
                    .iter_mut()
                    .find(|run| matches!(run.state, RunState::Queued))
            {
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
            && matches!(run.state, RunState::Queued | RunState::Running { .. })
        {
            run.state = state;
        }
    }
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
                        let drawing = outcome.drawing(&session.anchor);
                        let comparison =
                            outcome.comparison_drawing(&session.anchor, COMPARED_COLOR);
                        println!(
                            "mapping_lab: run {id} done in {:.1} s",
                            started.elapsed().as_secs_f64()
                        );
                        lab.update(
                            id,
                            RunState::Done(Arc::new(Finished {
                                outcome,
                                drawing,
                                comparison,
                            })),
                        );
                    }
                    Ok(None) => lab.update(id, RunState::Cancelled),
                }
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
        let lab = Lab::new(PathBuf::new(), PathBuf::new(), PathBuf::new());
        assert!(lab.queue_run().is_err());
    }

    #[test]
    fn the_draft_only_takes_tunable_parameters_in_range() {
        let lab = Lab::new(PathBuf::new(), PathBuf::new(), PathBuf::new());
        assert!(lab.set_draft("nope", 1.0).is_err());
        lab.set_draft("resolution_m", 99.0).unwrap();
        lab.with(|_, draft, _, _| assert_eq!(draft["resolution_m"], 0.5));
    }
}
