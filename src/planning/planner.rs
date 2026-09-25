//! [`Planner`]: the [`Executor`] running [`super::plan`] for the selected
//! map whenever [`PLANNING_REQUEST_TOPIC_NAME`] asks it to, with its
//! parameters tuned live through [`PLANNING_PARAMETERS_TOPIC_NAME`].

use super::config::{PlanningConfig, tunable_parameters};
use super::geometry::Point2;
use super::pipeline::{PlannedLines, Progress, plan};
use crate::environment::{CENTERLINE_FILE_NAME, Map, RACE_LINE_FILE_NAME, write_line};
use crate::topics::{
    Color, Drawing, MAP_TOPIC_NAME, PLANNING_PARAMETERS_TOPIC_NAME, PLANNING_REQUEST_TOPIC_NAME,
    PLANNING_STATUS_TOPIC_NAME, PlanningOutcome, PlanningParameters, PlanningRequest,
    PlanningState, PlanningStatus, SelectedMap, Shape,
};
use crate::{Captain, Executor, Ticker};
use std::any::Any;
use std::path::Path;
use std::time::{Duration, Instant};

/// Paint order of the progress drawing: above the map (and the race line
/// [`crate::sensors::MapServer`] draws with it), below the vehicle.
const PROGRESS_Z_INDEX: i32 = -90;

/// Plans a race line for the map on [`MAP_TOPIC_NAME`] whenever
/// [`PlanningRequest::requested`] changes, saving it in the map's folder,
/// and reports on [`PLANNING_STATUS_TOPIC_NAME`]. While idle, applies
/// whatever [`PlanningParameters`] asks for to its config - so parameters
/// can be tuned before starting - and reports the values in effect. Draws
/// the optimization's progress while planning.
pub struct Planner {
    id: u8,
    name: String,
    config: PlanningConfig,
}

impl Planner {
    pub fn new(name: impl Into<String>, config: PlanningConfig) -> Self {
        Self {
            id: 0,
            name: name.into(),
            config,
        }
    }

    /// Applies any new [`PlanningParameters`] to the config. Returns
    /// whether the config changed.
    fn apply_parameters(&mut self, captain: &Captain, seen_write_count: &mut u64) -> bool {
        // Nothing may publish parameters at all (e.g. a binary without
        // `web_gui`) - then the config stays as loaded.
        let Some(requests) =
            captain.try_topic::<PlanningParameters>(PLANNING_PARAMETERS_TOPIC_NAME)
        else {
            return false;
        };
        let write_count = requests.meta().write_count;
        if write_count == *seen_write_count {
            return false;
        }
        *seen_write_count = write_count;
        let wanted = requests.read().into_value().values;
        crate::config::apply_parameters(&mut self.config, &tunable_parameters(), &wanted)
    }

    /// The tunable parameters with the values in effect.
    fn parameters(&self) -> Vec<crate::topics::AlgorithmParameter> {
        let mut parameters = tunable_parameters();
        crate::config::refresh_parameter_values(&mut parameters, &self.config);
        parameters
    }

    /// Plans and saves the race line for `map_path` (the selected map, if
    /// any), answering request number `requested`. Publishes its progress
    /// through `status` and the drawing topic.
    fn handle_request(
        &self,
        captain: &Captain,
        requested: u64,
        map_path: Option<&Path>,
        status: &mut PlanningStatus,
    ) -> PlanningOutcome {
        let started = Instant::now();
        let mut outcome = PlanningOutcome {
            requested,
            map: map_path.map(|path| path.display().to_string()),
            ..PlanningOutcome::default()
        };
        let Some(map_path) = map_path else {
            outcome.error = Some("no map is selected".to_string());
            return outcome;
        };

        let status_topic = captain.topic::<PlanningStatus>(PLANNING_STATUS_TOPIC_NAME);
        let drawing_topic = captain.drawing(self.id);
        let publish = |status: &PlanningStatus| {
            status_topic
                .write(self.id, status.clone())
                .expect("lost writer authorization for the planning_status topic");
        };
        status.state = PlanningState::Computing;
        status.stage = "Loading the map".to_string();
        publish(status);

        let result = Map::load(map_path)
            .map_err(|err| format!("failed to load the map: {err}"))
            .and_then(|map| {
                let planned = plan(&map, &self.config, &mut |progress| {
                    match progress {
                        Progress::Stage(stage) => status.stage = stage,
                        Progress::Iteration { total, iteration } => {
                            status.stage = format!(
                                "Optimizing - iteration {}/{total}, moved up to {:.3} m",
                                iteration.number, iteration.max_move_m
                            );
                            drawing_topic
                                .write(
                                    self.id,
                                    progress_drawing(iteration.reference, iteration.solution),
                                )
                                .expect(
                                    "lost writer authorization for the planner's drawing topic",
                                );
                        }
                    }
                    publish(status);
                    captain.is_running(self.id)
                })
                .map_err(|err| err.to_string())?;
                save(&map, &planned).map(|saved_to| (planned, saved_to))
            });

        drawing_topic
            .write(self.id, Drawing::default().z_index(PROGRESS_Z_INDEX))
            .expect("lost writer authorization for the planner's drawing topic");
        match result {
            Ok((planned, saved_to)) => {
                outcome.saved_to = Some(saved_to);
                outcome.computed_centerline = planned.computed_centerline.is_some();
                outcome.lap_length_m = planned.lap_length_m;
                outcome.lap_time_s = planned.lap_time_s;
                outcome.max_curvature_per_m = planned.max_curvature_per_m;
                outcome.reference_max_curvature_per_m = planned.reference_max_curvature_per_m;
            }
            Err(err) => outcome.error = Some(err),
        }
        outcome.elapsed_ms = started.elapsed().as_secs_f64() * 1000.0;
        outcome
    }
}

/// Writes `planned`'s race line - and its computed centerline, if the map
/// had none - into `map`'s folder. Returns the race line file's path.
fn save(map: &Map, planned: &PlannedLines) -> Result<String, String> {
    if let Some(centerline) = &planned.computed_centerline {
        write_line(&map.folder, CENTERLINE_FILE_NAME, centerline)
            .map_err(|err| format!("failed to save the centerline: {err}"))?;
    }
    write_line(&map.folder, RACE_LINE_FILE_NAME, &planned.race_line)
        .map(|path| path.display().to_string())
        .map_err(|err| format!("failed to save the race line: {err}"))
}

/// The optimization's progress: the line it solved around, thin and
/// grey, and its solution on top.
fn progress_drawing(reference: &[Point2], solution: &[Point2]) -> Drawing {
    let polyline = |points: &[Point2], width_px, color| Shape::Polyline {
        points: points.iter().map(|p| [p.x as f32, p.y as f32]).collect(),
        closed: true,
        width_px,
        color,
    };
    Drawing::new(vec![
        polyline(reference, 1.0, Color::WHITE.with_alpha(120)),
        polyline(solution, 2.0, Color::AMBER),
    ])
    .z_index(PROGRESS_Z_INDEX)
}

impl Executor for Planner {
    fn init(&mut self, id: u8) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        let parameters = self.parameters();
        captain.claim_writer::<PlanningStatus>(PLANNING_STATUS_TOPIC_NAME, self.id, move || {
            PlanningStatus {
                parameters: parameters.clone(),
                ..PlanningStatus::default()
            }
        });
        captain.claim_drawing(self.id);
    }

    fn run(&mut self, captain: &Captain) {
        let status_topic = captain.topic::<PlanningStatus>(PLANNING_STATUS_TOPIC_NAME);
        let map_topic = captain.topic::<SelectedMap>(MAP_TOPIC_NAME);
        let mut ticker = Ticker::from_interval(Duration::from_millis(self.config.poll_interval_ms));
        let mut status = PlanningStatus {
            parameters: self.parameters(),
            ..status_topic.read().into_value()
        };
        status.state = PlanningState::Idle;
        status.stage.clear();
        status_topic
            .write(self.id, status.clone())
            .expect("lost writer authorization for the planning_status topic");

        let request_topic = || captain.try_topic::<PlanningRequest>(PLANNING_REQUEST_TOPIC_NAME);
        let mut seen_parameters = 0;
        // Seeded from the topic, so a restart of just this executor doesn't
        // replay a request it already handled.
        let mut handled = request_topic().map_or(0, |topic| topic.read().requested);

        while captain.is_running(self.id) {
            if self.apply_parameters(captain, &mut seen_parameters) {
                status.parameters = self.parameters();
                status_topic
                    .write(self.id, status.clone())
                    .expect("lost writer authorization for the planning_status topic");
            }

            let requested = request_topic().map_or(handled, |topic| topic.read().requested);
            if requested != handled {
                handled = requested;
                let map_path = map_topic.read().into_value().path;
                let outcome =
                    self.handle_request(captain, requested, map_path.as_deref(), &mut status);
                status.state = PlanningState::Idle;
                status.stage.clear();
                status.last_outcome = Some(outcome);
                status_topic
                    .write(self.id, status.clone())
                    .expect("lost writer authorization for the planning_status topic");
            }

            ticker.wait();
        }
    }

    fn name(&self) -> String {
        self.name.clone()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn fresh(&self) -> Box<dyn Executor> {
        Box::new(Planner::new(self.name.clone(), self.config))
    }
}
