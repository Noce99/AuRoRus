//! The lab's state: the localization session loaded, the draft pipeline the
//! next run localizes it with, every run of that session - queued, running
//! on one of the worker threads (see [`run_workers`]), or done - and what
//! runs are scored against: a reference trajectory (the ground truth, or a
//! run) placed on a race line to count laps.
//!
//! A finished run is written next to the session
//! (`<session>.runs/run_<id>.json`), and read back with it, so comparisons
//! survive the lab stopping.

use aurorus::benchmark::CodeVersion;
use aurorus::environment::race_lines;
use aurorus::localization::evaluation::{
    self, ErrorSample, Evaluation, LapScore, Laps, OdometrySource, Pipeline, ScanStep, TimedPose,
};
use aurorus::localization::session::{self, Session, SessionKind};
use aurorus::localization::{
    DeadReckoningConfig, Integration, ParticleFilterConfig, SlamConfig,
    particle_filter_tunable_parameters, slam_tunable_parameters,
};
use aurorus::topics::{AlgorithmParameter, LocalizationAlgorithm, SelectedMap};
use std::collections::BTreeMap;
use std::ops::ControlFlow;
use std::panic::AssertUnwindSafe;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, OnceLock};

/// The SLAM parameters scan matching localizes with - the rest only matter
/// to mapping.
pub const SCAN_MATCHING_PARAMETERS: [&str; 13] = [
    "odometry_buffer_len",
    "max_laser_range_m",
    "localization_minimum_response",
    "correlation_search_space_dimension_m",
    "correlation_search_space_resolution_m",
    "correlation_search_space_smear_deviation_m",
    "coarse_search_angle_offset_rad",
    "coarse_angle_resolution_rad",
    "fine_search_angle_offset_rad",
    "distance_variance_penalty",
    "angle_variance_penalty",
    "minimum_distance_penalty",
    "minimum_angle_penalty",
];

/// Where a pipeline's odometry comes from, as the page names it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OdometryChoice {
    /// As recorded.
    Recorded,
    /// The recorded IMU integrated again, with each method.
    Euler,
    Midpoint,
    Rk4,
}

impl OdometryChoice {
    pub fn label(self) -> &'static str {
        match self {
            Self::Recorded => "recorded odometry",
            Self::Euler => "Euler odometry",
            Self::Midpoint => "midpoint odometry",
            Self::Rk4 => "RK4 odometry",
        }
    }

    fn source(self, base: &DeadReckoningConfig) -> OdometrySource {
        let integration = match self {
            Self::Recorded => return OdometrySource::Recorded,
            Self::Euler => Integration::Euler,
            Self::Midpoint => Integration::Midpoint,
            Self::Rk4 => Integration::Rk4,
        };
        OdometrySource::Integrated(DeadReckoningConfig {
            integration,
            ..*base
        })
    }
}

/// What a run localizes with, as the page edits it.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Spec {
    pub odometry: OdometryChoice,
    /// `None` scores odometry alone.
    pub algorithm: Option<LocalizationAlgorithm>,
    /// Scan matching's parameters - [`SCAN_MATCHING_PARAMETERS`].
    pub scan_matching: BTreeMap<String, f64>,
    /// The particle filter's tunable parameters.
    pub particle_filter: BTreeMap<String, f64>,
}

impl Spec {
    /// What it says, in a few words.
    pub fn label(&self) -> String {
        let algorithm = match self.algorithm {
            None => "Odometry alone",
            Some(LocalizationAlgorithm::ScanMatching) => "Scan matching",
            Some(LocalizationAlgorithm::ParticleFilter) => "Particle filter",
        };
        format!("{algorithm}, {}", self.odometry.label())
    }

    /// The parameters the algorithm uses - none for odometry alone.
    pub fn algorithm_values(&self) -> &BTreeMap<String, f64> {
        static NONE: BTreeMap<String, f64> = BTreeMap::new();
        match self.algorithm {
            None => &NONE,
            Some(LocalizationAlgorithm::ScanMatching) => &self.scan_matching,
            Some(LocalizationAlgorithm::ParticleFilter) => &self.particle_filter,
        }
    }
}

/// The configs every run starts from - the config files, with the
/// session's recorded values applied.
#[derive(Debug, Clone, Copy)]
pub struct Bases {
    pub slam: SlamConfig,
    pub particle_filter: ParticleFilterConfig,
    pub dead_reckoning: DeadReckoningConfig,
}

impl Bases {
    /// `spec`'s pipeline - or why it can't run.
    pub fn pipeline(&self, spec: &Spec) -> Result<Pipeline, String> {
        Ok(Pipeline {
            odometry: spec.odometry.source(&self.dead_reckoning),
            algorithm: spec.algorithm,
            slam: self.slam.with_parameters(&spec.scan_matching)?,
            particle_filter: self.particle_filter.with_parameters(&spec.particle_filter),
        })
    }
}

/// A session loaded into the lab.
pub struct Loaded {
    /// Its file name, under the sessions folder.
    pub name: String,
    pub path: PathBuf,
    pub session: Session,
    /// The map it was recorded on, as localization takes it.
    pub map: SelectedMap,
    /// Where the map's files are.
    pub map_folder: PathBuf,
    /// The race lines the map has.
    pub race_lines: Vec<String>,
    /// The true pose - empty unless recorded in simulation.
    pub ground_truth: Vec<TimedPose>,
    pub scan_period_ms: Option<f64>,
    pub bases: Bases,
    /// The draft the session was recorded with: the live algorithm, with
    /// its parameters and the recorded odometry.
    pub baseline: Spec,
    /// Bumped on every load, so drawings of different sessions never share
    /// a version.
    pub generation: u64,
}

/// What runs are scored against.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case", tag = "kind", content = "run")]
pub enum ReferenceChoice {
    GroundTruth,
    Run(u64),
}

/// The reference trajectory, placed on the race line.
pub struct Context {
    /// Changes whenever the reference or the race line does - scores are
    /// cached against it.
    pub key: u64,
    pub choice: ReferenceChoice,
    pub race_line: String,
    pub reference: Vec<TimedPose>,
    pub laps: Laps,
}

/// A finished run's evaluation, and its scores against the current
/// [`Context`] - computed the first time they're asked for.
pub struct Finished {
    pub evaluation: Evaluation,
    pub code: CodeVersion,
    scored: Mutex<Option<(u64, Arc<Scored>)>>,
}

/// A run's errors at every pose, and its scores per lap.
pub struct Scored {
    pub errors: Vec<ErrorSample>,
    /// The whole drive first, then each lap.
    pub scores: Vec<LapScore>,
}

impl Finished {
    fn new(evaluation: Evaluation, code: CodeVersion) -> Self {
        Self {
            evaluation,
            code,
            scored: Mutex::new(None),
        }
    }

    /// Its scores against `context`.
    pub fn scored(&self, context: &Context, scan_period_ms: Option<f64>) -> Arc<Scored> {
        let mut cached = self.scored.lock().unwrap_or_else(|p| p.into_inner());
        if let Some((key, scored)) = cached.as_ref()
            && *key == context.key
        {
            return scored.clone();
        }
        let errors = evaluation::errors(&self.evaluation.track, &context.reference, &context.laps);
        let scores = evaluation::score(
            &errors,
            &self.evaluation.scans,
            &context.laps,
            scan_period_ms,
        );
        let scored = Arc::new(Scored { errors, scores });
        *cached = Some((context.key, scored.clone()));
        scored
    }
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

/// One localization of the loaded session.
pub struct Run {
    /// Unique within the session, from `1`.
    pub id: u64,
    pub spec: Spec,
    pub state: RunState,
    cancel: Arc<AtomicBool>,
}

/// What a finished run is saved as, next to its session.
#[derive(serde::Serialize, serde::Deserialize)]
struct SavedRun {
    id: u64,
    spec: Spec,
    #[serde(default)]
    code: CodeVersion,
    elapsed_s: f64,
    dropped_scans: usize,
    odometry_us_per_sample: Option<f64>,
    track: Vec<TimedPose>,
    scans: Vec<ScanStep>,
}

#[derive(Default)]
struct Inner {
    session: Option<Arc<Loaded>>,
    draft: Option<Spec>,
    runs: Vec<Run>,
    next_run_id: u64,
    load_error: Option<String>,
    generation: u64,
    reference: Option<ReferenceChoice>,
    race_line: Option<String>,
    context: Option<Arc<Context>>,
    /// Why there's no context, when there's a session.
    context_error: Option<String>,
    next_context_key: u64,
}

/// The lab - shared by the HTTP threads and the workers.
pub struct Lab {
    pub sessions_root: PathBuf,
    pub config_dir: PathBuf,
    pub workers: usize,
    inner: Mutex<Inner>,
    /// Signalled whenever a run is queued.
    work: Condvar,
}

/// What the HTTP handlers read of the lab, under its lock.
pub struct View<'a> {
    pub session: Option<&'a Arc<Loaded>>,
    pub draft: Option<&'a Spec>,
    pub runs: &'a [Run],
    pub load_error: Option<&'a str>,
    pub context: Option<&'a Arc<Context>>,
    pub context_error: Option<&'a str>,
}

impl Lab {
    pub fn new(sessions_root: PathBuf, config_dir: PathBuf, workers: usize) -> Self {
        Self {
            sessions_root,
            config_dir,
            workers,
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

    pub fn slam_config_path(&self) -> PathBuf {
        self.config_dir.join("localization/slam.toml")
    }

    pub fn particle_filter_config_path(&self) -> PathBuf {
        self.config_dir.join("localization/particle_filter.toml")
    }

    fn dead_reckoning_config_path(&self) -> PathBuf {
        self.config_dir.join("localization/dead_reckoning.toml")
    }

    /// The session files under [`Self::sessions_root`], newest first, with
    /// their sizes.
    pub fn session_files(&self) -> Vec<(String, u64)> {
        session::list(&self.sessions_root, SessionKind::Localization)
    }

    /// Loads the session file `name` under [`Self::sessions_root`] - or the
    /// file at `path` itself, given one - in place of the current one: every
    /// run of the current one is cancelled and forgotten, and the runs
    /// saved with the new one are read back.
    pub fn load(&self, name: &str, path: Option<&Path>) -> Result<(), String> {
        match self.read_session(name, path) {
            Ok((loaded, runs)) => {
                self.install(loaded, runs);
                Ok(())
            }
            Err(err) => {
                self.inner().load_error = Some(err.clone());
                Err(err)
            }
        }
    }

    fn install(&self, loaded: Loaded, saved: Vec<Run>) {
        let mut inner = self.inner();
        for run in &inner.runs {
            run.cancel.store(true, Ordering::Relaxed);
        }
        inner.generation += 1;
        inner.next_run_id = saved.iter().map(|run| run.id).max().unwrap_or(0) + 1;
        inner.runs = saved;
        inner.draft = Some(loaded.baseline.clone());
        inner.load_error = None;
        inner.reference = Some(if loaded.ground_truth.is_empty() {
            match inner.runs.first() {
                Some(run) => ReferenceChoice::Run(run.id),
                None => ReferenceChoice::GroundTruth,
            }
        } else {
            ReferenceChoice::GroundTruth
        });
        inner.race_line = loaded
            .session
            .info
            .localization
            .as_ref()
            .and_then(|info| info.race_line.clone())
            .filter(|line| loaded.race_lines.contains(line))
            .or_else(|| loaded.race_lines.first().cloned());
        inner.session = Some(Arc::new(Loaded {
            generation: inner.generation,
            ..loaded
        }));
        Self::refresh_context(&mut inner);
    }

    fn read_session(&self, name: &str, path: Option<&Path>) -> Result<(Loaded, Vec<Run>), String> {
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
        if session.info.kind != SessionKind::Localization {
            return Err(format!(
                "{path:?} is a mapping session, not a localization one"
            ));
        }
        let map_folder = session
            .map_folder()?
            .ok_or("the session embeds no map to localize on")?;
        let map = evaluation::load_map(&map_folder)?;
        let race_lines = race_lines::list(&map_folder)
            .into_iter()
            .map(|entry| entry.file)
            .collect();

        let slam_file =
            aurorus::config::load::<SlamConfig>(&self.slam_config_path()).unwrap_or_default();
        let particle_filter_file =
            aurorus::config::load::<ParticleFilterConfig>(&self.particle_filter_config_path())
                .unwrap_or_default();
        let dead_reckoning =
            aurorus::config::load::<DeadReckoningConfig>(&self.dead_reckoning_config_path())
                .unwrap_or_default();
        let info = session.info.localization.clone().unwrap_or_default();
        let bases = Bases {
            slam: slam_file
                .with_parameters(&session.info.slam_parameters)
                .unwrap_or(slam_file),
            particle_filter: particle_filter_file.with_parameters(&info.particle_filter_parameters),
            dead_reckoning,
        };
        let baseline = Spec {
            odometry: OdometryChoice::Recorded,
            algorithm: Some(info.algorithm),
            scan_matching: scan_matching_values(&bases.slam),
            particle_filter: particle_filter_values(&bases.particle_filter),
        };
        let runs = read_runs(&path);
        let loaded = Loaded {
            name: path
                .file_name()
                .map(|name| name.to_string_lossy().into_owned())
                .unwrap_or_default(),
            ground_truth: evaluation::ground_truth(&session),
            scan_period_ms: evaluation::scan_period_ms(&session),
            path,
            session,
            map,
            map_folder,
            race_lines,
            bases,
            baseline,
            generation: 0,
        };
        Ok((loaded, runs))
    }

    /// Places the reference on the race line again, after either changed.
    fn refresh_context(inner: &mut Inner) {
        inner.context = None;
        inner.context_error = None;
        let Some(session) = inner.session.clone() else {
            return;
        };
        let result = (|| {
            let choice = inner.reference.ok_or("no reference picked")?;
            let race_line = inner
                .race_line
                .clone()
                .ok_or("the map has no race line to count laps along")?;
            let reference = match choice {
                ReferenceChoice::GroundTruth if session.ground_truth.is_empty() => {
                    return Err(
                        "recorded on the car: no ground truth - pick a run as the reference"
                            .to_string(),
                    );
                }
                ReferenceChoice::GroundTruth => session.ground_truth.clone(),
                ReferenceChoice::Run(id) => match inner.runs.iter().find(|run| run.id == id) {
                    Some(Run {
                        state: RunState::Done(finished),
                        ..
                    }) => finished.evaluation.track.clone(),
                    Some(_) => return Err(format!("run #{id}, the reference, isn't done")),
                    None => return Err(format!("run #{id}, the reference, is gone")),
                },
            };
            let points = race_lines::read(&session.map_folder, &race_line)
                .map_err(|err| format!("{race_line}: {err}"))?;
            let laps = Laps::new(&reference, &points)
                .ok_or_else(|| format!("{race_line} isn't a closed line"))?;
            Ok((choice, race_line, reference, laps))
        })();
        match result {
            Ok((choice, race_line, reference, laps)) => {
                inner.next_context_key += 1;
                inner.context = Some(Arc::new(Context {
                    key: inner.next_context_key,
                    choice,
                    race_line,
                    reference,
                    laps,
                }));
            }
            Err(err) => inner.context_error = Some(err),
        }
    }

    /// Runs `f` on what the lab holds - under the lock, so keep it short.
    pub fn with<T>(&self, f: impl FnOnce(View) -> T) -> T {
        let inner = self.inner();
        f(View {
            session: inner.session.as_ref(),
            draft: inner.draft.as_ref(),
            runs: &inner.runs,
            load_error: inner.load_error.as_deref(),
            context: inner.context.as_ref(),
            context_error: inner.context_error.as_deref(),
        })
    }

    pub fn session(&self) -> Option<Arc<Loaded>> {
        self.inner().session.clone()
    }

    pub fn context(&self) -> Option<Arc<Context>> {
        self.inner().context.clone()
    }

    pub fn set_reference(&self, choice: ReferenceChoice) -> Result<(), String> {
        let mut inner = self.inner();
        inner.session.as_ref().ok_or("no session is loaded")?;
        inner.reference = Some(choice);
        Self::refresh_context(&mut inner);
        Ok(())
    }

    pub fn set_race_line(&self, file: &str) -> Result<(), String> {
        let mut inner = self.inner();
        let session = inner.session.clone().ok_or("no session is loaded")?;
        if !session.race_lines.iter().any(|line| line == file) {
            return Err(format!("the map has no race line {file:?}"));
        }
        inner.race_line = Some(file.to_string());
        Self::refresh_context(&mut inner);
        Ok(())
    }

    /// Sets the draft's odometry and algorithm.
    pub fn set_draft_pipeline(
        &self,
        odometry: OdometryChoice,
        algorithm: Option<LocalizationAlgorithm>,
    ) -> Result<(), String> {
        let mut inner = self.inner();
        let draft = inner.draft.as_mut().ok_or("no session is loaded")?;
        draft.odometry = odometry;
        draft.algorithm = algorithm;
        Ok(())
    }

    /// Sets one of the draft's `group` parameters (`scan_matching` or
    /// `particle_filter`) to `value`, brought into its range.
    pub fn set_draft_parameter(&self, group: Group, name: &str, value: f64) -> Result<(), String> {
        let parameter = group
            .parameters()
            .into_iter()
            .find(|parameter| parameter.name == name)
            .ok_or_else(|| format!("no tunable parameter {name:?}"))?;
        let value = parameter
            .kind
            .sanitize(value)
            .ok_or("value must be a finite number")?;
        let mut inner = self.inner();
        let draft = inner.draft.as_mut().ok_or("no session is loaded")?;
        group.values_mut(draft).insert(name.to_string(), value);
        Ok(())
    }

    /// Replaces the draft's `group` values with `values` - only those of its
    /// parameters, each brought into its range.
    pub fn set_draft_values(&self, group: Group, values: &BTreeMap<String, f64>) {
        let mut inner = self.inner();
        let Some(draft) = inner.draft.as_mut() else {
            return;
        };
        for parameter in group.parameters() {
            if let Some(value) = values
                .get(&parameter.name)
                .and_then(|&value| parameter.kind.sanitize(value))
            {
                group.values_mut(draft).insert(parameter.name, value);
            }
        }
    }

    /// Sets the draft back to what the session was recorded with.
    pub fn reset_draft(&self) -> Result<(), String> {
        let mut inner = self.inner();
        let session = inner.session.clone().ok_or("no session is loaded")?;
        inner.draft = Some(session.baseline.clone());
        Ok(())
    }

    /// Queues a run of the loaded session with the draft - or says why it
    /// can't be run. Returns its id.
    pub fn queue_run(&self) -> Result<u64, String> {
        let mut inner = self.inner();
        let session = inner.session.clone().ok_or("no session is loaded")?;
        let spec = inner.draft.clone().ok_or("no session is loaded")?;
        session.bases.pipeline(&spec)?;
        let id = inner.next_run_id;
        inner.next_run_id += 1;
        inner.runs.push(Run {
            id,
            spec,
            state: RunState::Queued,
            cancel: Arc::default(),
        });
        drop(inner);
        self.work.notify_all();
        Ok(id)
    }

    pub fn cancel_run(&self, id: u64) -> Result<(), String> {
        let mut inner = self.inner();
        let run = find(&mut inner.runs, id)?;
        run.cancel.store(true, Ordering::Relaxed);
        if !run.state.is_over() {
            run.state = RunState::Cancelled;
        }
        Ok(())
    }

    /// Forgets run `id`, and deletes its saved file.
    pub fn delete_run(&self, id: u64) -> Result<(), String> {
        let mut inner = self.inner();
        find(&mut inner.runs, id)?
            .cancel
            .store(true, Ordering::Relaxed);
        inner.runs.retain(|run| run.id != id);
        if let Some(session) = &inner.session {
            let _ = std::fs::remove_file(run_path(&session.path, id));
        }
        if inner.reference == Some(ReferenceChoice::Run(id)) {
            Self::refresh_context(&mut inner);
        }
        Ok(())
    }

    /// Sets the draft to run `id`'s spec.
    pub fn use_run(&self, id: u64) -> Result<(), String> {
        let mut inner = self.inner();
        let spec = find(&mut inner.runs, id)?.spec.clone();
        inner.draft = Some(spec);
        Ok(())
    }

    pub fn finished(&self, id: u64) -> Option<Arc<Finished>> {
        self.inner()
            .runs
            .iter()
            .find(|run| run.id == id)
            .and_then(|run| match &run.state {
                RunState::Done(finished) => Some(finished.clone()),
                _ => None,
            })
    }

    fn next_job(&self) -> (u64, Arc<Loaded>, Pipeline, Spec, Arc<AtomicBool>) {
        let mut inner = self.inner();
        loop {
            let session = inner.session.clone();
            let next = inner
                .runs
                .iter()
                .position(|run| matches!(run.state, RunState::Queued));
            if let Some(session) = session
                && let Some(next) = next
            {
                let run = &mut inner.runs[next];
                let pipeline = session.bases.pipeline(&run.spec);
                match pipeline {
                    Ok(pipeline) => {
                        run.state = RunState::Running { progress: 0.0 };
                        return (
                            run.id,
                            session,
                            pipeline,
                            run.spec.clone(),
                            run.cancel.clone(),
                        );
                    }
                    Err(err) => {
                        run.state = RunState::Failed(err);
                        continue;
                    }
                }
            }
            inner = self
                .work
                .wait(inner)
                .unwrap_or_else(|poisoned| poisoned.into_inner());
        }
    }

    fn update(&self, id: u64, generation: u64, state: RunState) {
        let mut inner = self.inner();
        if inner.generation != generation {
            return;
        }
        let done = matches!(state, RunState::Done(_));
        if let Some(run) = inner.runs.iter_mut().find(|run| run.id == id)
            && !run.state.is_over()
        {
            run.state = state;
        }
        // A run picked as the reference before it finished.
        if done && inner.reference == Some(ReferenceChoice::Run(id)) {
            Self::refresh_context(&mut inner);
        }
    }
}

/// Which of the draft's parameter groups.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Group {
    ScanMatching,
    ParticleFilter,
}

impl Group {
    /// Its tunable parameters.
    pub fn parameters(self) -> Vec<AlgorithmParameter> {
        match self {
            Self::ScanMatching => slam_tunable_parameters()
                .into_iter()
                .filter(|parameter| SCAN_MATCHING_PARAMETERS.contains(&parameter.name.as_str()))
                .collect(),
            Self::ParticleFilter => particle_filter_tunable_parameters(),
        }
    }

    fn values_mut(self, spec: &mut Spec) -> &mut BTreeMap<String, f64> {
        match self {
            Self::ScanMatching => &mut spec.scan_matching,
            Self::ParticleFilter => &mut spec.particle_filter,
        }
    }

    /// Its parameters, with `spec`'s values.
    pub fn with_values(self, spec: &Spec) -> Vec<AlgorithmParameter> {
        let values = match self {
            Self::ScanMatching => &spec.scan_matching,
            Self::ParticleFilter => &spec.particle_filter,
        };
        let mut parameters = self.parameters();
        for parameter in &mut parameters {
            if let Some(&value) = values.get(&parameter.name) {
                parameter.value = value;
            }
        }
        parameters
    }
}

fn scan_matching_values(config: &SlamConfig) -> BTreeMap<String, f64> {
    let mut parameters = Group::ScanMatching.parameters();
    aurorus::config::refresh_parameter_values(&mut parameters, config);
    values(&parameters)
}

fn particle_filter_values(config: &ParticleFilterConfig) -> BTreeMap<String, f64> {
    let mut parameters = particle_filter_tunable_parameters();
    aurorus::config::refresh_parameter_values(&mut parameters, config);
    values(&parameters)
}

fn values(parameters: &[AlgorithmParameter]) -> BTreeMap<String, f64> {
    parameters
        .iter()
        .map(|parameter| (parameter.name.clone(), parameter.value))
        .collect()
}

fn find(runs: &mut [Run], id: u64) -> Result<&mut Run, String> {
    runs.iter_mut()
        .find(|run| run.id == id)
        .ok_or_else(|| format!("there's no run {id}"))
}

/// The folder a session's runs are saved in: `<session>.runs/` beside it.
fn runs_folder(session: &Path) -> PathBuf {
    let name = session
        .file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_default();
    session.with_file_name(format!("{name}.runs"))
}

fn run_path(session: &Path, id: u64) -> PathBuf {
    runs_folder(session).join(format!("run_{id}.json"))
}

/// The runs saved beside `session`, oldest first - a file that doesn't
/// read is skipped, with a warning.
fn read_runs(session: &Path) -> Vec<Run> {
    let Ok(entries) = std::fs::read_dir(runs_folder(session)) else {
        return Vec::new();
    };
    let mut runs: Vec<Run> = entries
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| path.extension().is_some_and(|ext| ext == "json"))
        .filter_map(|path| {
            let read = std::fs::read(&path)
                .map_err(|err| err.to_string())
                .and_then(|bytes| {
                    serde_json::from_slice::<SavedRun>(&bytes).map_err(|err| err.to_string())
                });
            match read {
                Ok(saved) => Some(Run {
                    id: saved.id,
                    spec: saved.spec,
                    state: RunState::Done(Arc::new(Finished::new(
                        Evaluation {
                            track: saved.track,
                            scans: saved.scans,
                            dropped_scans: saved.dropped_scans,
                            odometry_us_per_sample: saved.odometry_us_per_sample,
                            elapsed: std::time::Duration::from_secs_f64(saved.elapsed_s),
                        },
                        saved.code,
                    ))),
                    cancel: Arc::default(),
                }),
                Err(err) => {
                    eprintln!("localization_lab: skipping {path:?}: {err}");
                    None
                }
            }
        })
        .collect();
    runs.sort_by_key(|run| run.id);
    runs
}

/// Saves a finished run beside its session.
fn save_run(session: &Path, id: u64, spec: &Spec, finished: &Finished) -> Result<PathBuf, String> {
    let evaluation = &finished.evaluation;
    let saved = SavedRun {
        id,
        spec: spec.clone(),
        code: finished.code.clone(),
        elapsed_s: evaluation.elapsed.as_secs_f64(),
        dropped_scans: evaluation.dropped_scans,
        odometry_us_per_sample: evaluation.odometry_us_per_sample,
        track: evaluation.track.clone(),
        scans: evaluation.scans.clone(),
    };
    let path = run_path(session, id);
    let write = || -> std::io::Result<()> {
        std::fs::create_dir_all(runs_folder(session))?;
        std::fs::write(&path, serde_json::to_vec(&saved)?)
    };
    write().map_err(|err| format!("couldn't write {path:?}: {err}"))?;
    Ok(path)
}

/// The git commit checked out, and whether the working tree has changes -
/// both `None` if git can't tell.
fn code_version() -> CodeVersion {
    static VERSION: OnceLock<CodeVersion> = OnceLock::new();
    VERSION
        .get_or_init(|| {
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
        })
        .clone()
}

/// Starts `count` threads that localize queued runs, one each at a time.
pub fn run_workers(lab: &Arc<Lab>, count: usize) {
    for _ in 0..count {
        let lab = lab.clone();
        std::thread::spawn(move || {
            loop {
                let (id, session, pipeline, spec, cancel) = lab.next_job();
                let generation = session.generation;
                // A panic fails this run only, not the worker.
                let outcome = std::panic::catch_unwind(AssertUnwindSafe(|| {
                    evaluation::evaluate(&session.session, &session.map, &pipeline, |progress| {
                        if cancel.load(Ordering::Relaxed) {
                            return ControlFlow::Break(());
                        }
                        lab.update(id, generation, RunState::Running { progress });
                        ControlFlow::Continue(())
                    })
                }));
                let state = match outcome {
                    Err(panic) => RunState::Failed(
                        panic
                            .downcast_ref::<String>()
                            .cloned()
                            .or_else(|| panic.downcast_ref::<&str>().map(|s| s.to_string()))
                            .unwrap_or_else(|| "the localization panicked".to_string()),
                    ),
                    Ok(Err(err)) => RunState::Failed(err),
                    Ok(Ok(None)) => RunState::Cancelled,
                    Ok(Ok(Some(evaluation))) => {
                        let finished = Arc::new(Finished::new(evaluation, code_version()));
                        match save_run(&session.path, id, &spec, &finished) {
                            Ok(path) => println!("localization_lab: run #{id} saved to {path:?}"),
                            Err(err) => eprintln!("localization_lab: {err}"),
                        }
                        RunState::Done(finished)
                    }
                };
                lab.update(id, generation, state);
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec() -> Spec {
        Spec {
            odometry: OdometryChoice::Rk4,
            algorithm: Some(LocalizationAlgorithm::ParticleFilter),
            scan_matching: BTreeMap::new(),
            particle_filter: BTreeMap::from([("num_particles".to_string(), 500.0)]),
        }
    }

    #[test]
    fn a_saved_run_reads_back_beside_its_session() {
        let folder = std::env::temp_dir().join(format!(
            "aurorus_localization_lab_tests_{}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&folder);
        std::fs::create_dir_all(&folder).unwrap();
        let session = folder.join("lap.localization");
        let evaluation = Evaluation {
            track: vec![(0, [1.0, 2.0, 0.5]), (10, [1.1, 2.0, 0.5])],
            scans: vec![ScanStep {
                t_us: 5,
                process_ms: 2.5,
                quality: Some(0.1),
            }],
            dropped_scans: 1,
            odometry_us_per_sample: Some(0.2),
            elapsed: std::time::Duration::from_millis(1500),
        };
        let finished = Finished::new(evaluation.clone(), CodeVersion::default());
        let path = save_run(&session, 7, &spec(), &finished).unwrap();
        assert_eq!(path, folder.join("lap.localization.runs/run_7.json"));
        // Something else in the folder is skipped.
        std::fs::write(folder.join("lap.localization.runs/notes.json"), b"{}").unwrap();

        let runs = read_runs(&session);
        std::fs::remove_dir_all(&folder).unwrap();
        assert_eq!(runs.len(), 1);
        assert_eq!(runs[0].id, 7);
        assert_eq!(runs[0].spec, spec());
        let RunState::Done(read) = &runs[0].state else {
            panic!("a saved run reads back done");
        };
        assert_eq!(read.evaluation.track, evaluation.track);
        assert_eq!(read.evaluation.scans, evaluation.scans);
        assert_eq!(read.evaluation.dropped_scans, 1);
        assert_eq!(read.evaluation.elapsed, evaluation.elapsed);
    }

    #[test]
    fn a_spec_names_its_pipeline() {
        assert_eq!(spec().label(), "Particle filter, RK4 odometry");
        let alone = Spec {
            algorithm: None,
            odometry: OdometryChoice::Recorded,
            ..spec()
        };
        assert_eq!(alone.label(), "Odometry alone, recorded odometry");
        assert!(alone.algorithm_values().is_empty());
    }

    #[test]
    fn scan_matching_is_tuned_with_its_own_parameters_only() {
        let names: Vec<_> = Group::ScanMatching
            .parameters()
            .into_iter()
            .map(|parameter| parameter.name)
            .collect();
        assert_eq!(names.len(), SCAN_MATCHING_PARAMETERS.len());
        assert!(!names.contains(&"loop_search_maximum_distance_m".to_string()));
    }
}
