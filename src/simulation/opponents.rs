//! Opponents: other autonomous vehicles sharing the track with the ego
//! vehicle, added and deleted while everything runs - see
//! [`OpponentsManager`].
//!
//! Each opponent is a small group of executors (see
//! [`crate::Captain::spawn_group`]) publishing on its own topics
//! ([`VehicleTopics::opponent`]):
//! - a [`SimulatedVehicle::opponent`], with a copy of the ego vehicle's model
//!   and its own actuator limits, scaling every commanded speed;
//! - its own copy of an autonomous algorithm ([`Instance::opponent`]), which
//!   always drives and always knows the opponent's exact pose - opponents have
//!   no localization;
//! - a [`SimulatedLidar::opponent`], if the algorithm reads lidar scans;
//! - a [`RaceLinePublisher`], if it was given a race line.
//!
//! There are no collisions, but lidars see the other vehicles (see
//! [`SimulatedLidarConfig::see_vehicles`]).

use crate::autonomous_control::{self, Instance};
use crate::environment::{RaceLinePublisher, race_lines};
use crate::simulation::{
    OpponentVehicle, SimulatedVehicle, SimulatedVehicleConfig, opponent_model,
};
use crate::simulation::{SimulatedLidar, SimulatedLidarConfig};
use crate::topics::{
    AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME, ActuatorLimits, AutonomousAlgorithmStatus,
    AvailableAlgorithm, MAP_TOPIC_NAME, OPPONENT_REQUESTS_TOPIC_NAME, OPPONENT_TOPIC_PREFIX,
    OPPONENTS_TOPIC_NAME, Opponent, OpponentOutcome, OpponentRequest, OpponentRequests,
    OpponentSpec, Opponents, ParameterKind, SelectedMap, SelectedRaceLine,
    VEHICLE_MODEL_STATUS_TOPIC_NAME, VehicleModelStatus, VehicleTopics,
};
use crate::{Captain, Executor, Ticker};
use std::any::Any;
use std::path::{Path, PathBuf};

/// How often [`OpponentsManager`] looks for new requests and a map change, in
/// Hz.
const RATE_HZ: f64 = 20.0;

/// Why `spec` can't be an opponent, given the autonomous `algorithms`
/// available and the `race_lines` (file names) of the current map - else the
/// algorithm it runs.
pub fn validate<'a>(
    spec: &OpponentSpec,
    algorithms: &'a [AvailableAlgorithm],
    race_lines: &[String],
) -> Result<&'a AvailableAlgorithm, String> {
    let algorithm = algorithms
        .iter()
        .find(|algorithm| algorithm.name == spec.algorithm)
        .ok_or_else(|| format!("There's no autonomous algorithm {:?}.", spec.algorithm))?;
    if !(0.0..=1.0).contains(&spec.speed_scale) {
        return Err(format!(
            "The speed multiplier must be between 0 and 1, not {}.",
            spec.speed_scale
        ));
    }
    limits_within_range(&spec.limits)?;
    match &spec.race_line {
        None if algorithm.requires.race_line => Err(format!(
            "{} follows a race line: pick one.",
            algorithm.label
        )),
        Some(file) if !race_lines.contains(file) => Err(format!(
            "{file:?} isn't one of the current map's race lines."
        )),
        _ => Ok(algorithm),
    }
}

/// Whether every one of `limits` is within its tunable range (see
/// [`ActuatorLimits::tunable_parameters_but_steering_angle`]) - else why not.
/// The steering angle is the car's, not the opponent's to pick (see
/// [`opponent_model`]).
fn limits_within_range(limits: &ActuatorLimits) -> Result<(), String> {
    let mut parameters = ActuatorLimits::tunable_parameters_but_steering_angle();
    crate::config::refresh_parameter_values(&mut parameters, limits);
    for parameter in parameters {
        if parameter.kind.sanitize(parameter.value) != Some(parameter.value) {
            let range = match parameter.kind {
                ParameterKind::Float { min, max, .. } => format!("{min} and {max}"),
                ParameterKind::Int { min, max, .. } => format!("{min} and {max}"),
            };
            return Err(format!(
                "{} must be between {range}, not {}.",
                parameter.name, parameter.value
            ));
        }
    }
    Ok(())
}

/// The name of opponent `id`'s group of executors (see
/// [`crate::Captain::spawn_group`]): `opponent/<id>`.
fn group_name(id: u32) -> String {
    format!("{OPPONENT_TOPIC_PREFIX}{id}")
}

/// What an opponent is built from, besides its [`OpponentSpec`].
struct Ingredients<'a> {
    requires_lidar: bool,
    /// The race line it follows, already loaded.
    race_line: Option<SelectedRaceLine>,
    /// The ego vehicle's model, as it publishes it.
    ego_model: &'a VehicleModelStatus,
    vehicle_config: &'a SimulatedVehicleConfig,
    lidar_config: SimulatedLidarConfig,
}

/// Every executor opponent `id` runs - see the [module docs](self).
fn executors(
    id: u32,
    spec: &OpponentSpec,
    ingredients: Ingredients,
) -> Result<Vec<Box<dyn Executor>>, String> {
    let vehicle = VehicleTopics::opponent(id);
    let prefix = vehicle.prefix().to_string();
    let instance = Instance::opponent(id, &spec.algorithm);
    let command_topic = instance.algorithm_topics().command;
    let algorithm = autonomous_control::build(&spec.algorithm, instance)
        .ok_or_else(|| format!("There's no autonomous algorithm {:?}.", spec.algorithm))?;

    let (model, config) = opponent_model(
        ingredients.vehicle_config.clone(),
        ingredients.ego_model,
        spec.limits,
    );
    let mut executors: Vec<Box<dyn Executor>> = vec![
        SimulatedVehicle::opponent(
            format!("{prefix}vehicle"),
            model,
            config,
            vehicle.clone(),
            OpponentVehicle {
                command_topic,
                speed_scale: spec.speed_scale,
                color: spec.color.color(),
            },
        )
        .boxed(),
        algorithm,
    ];
    if ingredients.requires_lidar {
        executors.push(
            SimulatedLidar::opponent(
                format!("{prefix}lidar"),
                ingredients.lidar_config,
                vehicle.clone(),
            )
            .boxed(),
        );
    }
    if let Some(line) = ingredients.race_line {
        executors.push(
            RaceLinePublisher::new(
                format!("{prefix}race_line"),
                vehicle,
                line,
                spec.color.color(),
            )
            .boxed(),
        );
    }
    Ok(executors)
}

/// Adds and deletes opponents as [`OPPONENT_REQUESTS_TOPIC_NAME`] asks,
/// publishing the ones running - and how the latest request went - on
/// [`OPPONENTS_TOPIC_NAME`]. Deletes every opponent when the map changes,
/// since their race lines belong to the old one. Starts with none, after a
/// restart too - a restart never brings a group of executors back.
///
/// An opponent copies the model the ego vehicle runs at the moment it's
/// added (from [`VEHICLE_MODEL_STATUS_TOPIC_NAME`]), on top of
/// `vehicle_config`; its lidar, if any, is configured by `lidar_config`.
pub struct OpponentsManager {
    id: u16,
    name: String,
    vehicle_config: SimulatedVehicleConfig,
    lidar_config: SimulatedLidarConfig,
}

impl OpponentsManager {
    pub fn new(
        name: impl Into<String>,
        vehicle_config: SimulatedVehicleConfig,
        lidar_config: SimulatedLidarConfig,
    ) -> Self {
        Self {
            id: 0,
            name: name.into(),
            vehicle_config,
            lidar_config,
        }
    }

    /// Spawns opponent `id` as `spec` asks, on the map in `map` - else why
    /// not.
    fn add(
        &self,
        captain: &Captain,
        id: u32,
        spec: &OpponentSpec,
        map: Option<&Path>,
    ) -> Result<Opponent, String> {
        // Nothing may publish these at all (e.g. a binary without
        // `AutonomousControlsHandler`) - then nothing validates.
        let algorithms = captain
            .try_topic::<AutonomousAlgorithmStatus>(AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME)
            .map(|topic| topic.read().into_value().available)
            .unwrap_or_default();
        let race_line_files: Vec<String> = map
            .map(|folder| {
                race_lines::list(folder)
                    .into_iter()
                    .map(|entry| entry.file)
                    .collect()
            })
            .unwrap_or_default();
        let algorithm = validate(spec, &algorithms, &race_line_files)?;
        let race_line = match (&spec.race_line, map) {
            (Some(file), Some(folder)) => Some(
                SelectedRaceLine::load(folder, file)
                    .map_err(|err| format!("Couldn't load race line {file:?}: {err}"))?,
            ),
            _ => None,
        };
        let ego_model = captain
            .try_topic::<VehicleModelStatus>(VEHICLE_MODEL_STATUS_TOPIC_NAME)
            .ok_or("There's no ego vehicle to copy the model of.")?
            .read()
            .into_value();

        let executors = executors(
            id,
            spec,
            Ingredients {
                requires_lidar: algorithm.requires.lidar,
                race_line,
                ego_model: &ego_model,
                vehicle_config: &self.vehicle_config,
                lidar_config: self.lidar_config,
            },
        )?;
        captain.spawn_group(group_name(id), executors);
        Ok(Opponent {
            id,
            spec: spec.clone(),
            algorithm_label: algorithm.label.clone(),
        })
    }
}

impl Executor for OpponentsManager {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_writer::<Opponents>(OPPONENTS_TOPIC_NAME, self.id, Opponents::default);
    }

    fn run(&mut self, captain: &Captain) {
        let opponents_topic = captain.topic::<Opponents>(OPPONENTS_TOPIC_NAME);
        // Nothing may publish these at all (e.g. a binary without `web_gui`,
        // or without `MapServer`) - then nothing is ever asked, or the map
        // never changes.
        let requests_topic = || captain.try_topic::<OpponentRequests>(OPPONENT_REQUESTS_TOPIC_NAME);
        let map_topic = captain.try_topic::<SelectedMap>(MAP_TOPIC_NAME);

        let mut opponents = Opponents::default();
        let mut next_id: u32 = 1;
        // Every request on the topic is this run's to handle - even one made
        // before this thread got here: a restart starts every topic over.
        let mut handled = 0;
        // Requests are only reread when their topic is rewritten.
        let mut seen_request_writes = 0;
        // The map is only reread when its topic is rewritten - it's big.
        let mut seen_map_writes = map_topic.as_ref().map(|topic| topic.meta().write_count);
        let mut map: Option<PathBuf> = map_topic
            .as_ref()
            .and_then(|topic| topic.read().path.clone());
        let mut ticker = Ticker::new(RATE_HZ);

        while captain.is_running(self.id) {
            let mut changed = false;

            if let Some(topic) = &map_topic
                && seen_map_writes != Some(topic.meta().write_count)
            {
                let current = topic.read();
                seen_map_writes = Some(current.meta.write_count);
                if current.path != map {
                    map = current.path.clone();
                    for opponent in opponents.list.drain(..) {
                        captain.stop_group(group_name(opponent.id));
                        changed = true;
                    }
                }
            }

            if let Some(topic) = requests_topic()
                && topic.meta().write_count != seen_request_writes
            {
                let requests = topic.read();
                seen_request_writes = requests.meta.write_count;
                let already_handled = handled;
                for numbered in requests
                    .requests
                    .iter()
                    .filter(|r| r.number > already_handled)
                {
                    handled = numbered.number;
                    let result = match &numbered.request {
                        OpponentRequest::Add(spec) => self
                            .add(captain, next_id, spec, map.as_deref())
                            .map(|opponent| {
                                next_id += 1;
                                opponents.list.push(opponent);
                            }),
                        OpponentRequest::Delete(id) => {
                            match opponents.list.iter().position(|o| o.id == *id) {
                                Some(index) => {
                                    opponents.list.remove(index);
                                    captain.stop_group(group_name(*id));
                                    Ok(())
                                }
                                None => Err(format!("There's no opponent {id}.")),
                            }
                        }
                    };
                    if let Err(err) = &result {
                        eprintln!("{}: {err}", self.name);
                    }
                    opponents.last_outcome = Some(OpponentOutcome {
                        request: numbered.number,
                        error: result.err(),
                    });
                    changed = true;
                }
            }

            if changed {
                opponents_topic
                    .write(self.id, opponents.clone())
                    .expect("lost writer authorization for the opponents topic");
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
        Box::new(Self::new(
            self.name.clone(),
            self.vehicle_config.clone(),
            self.lidar_config,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Runner;
    use crate::topics::{
        AlgorithmRequirements, PLACE_AT_START_TOPIC_NAME, PlaceAtStart, START_STATE_TOPIC_NAME,
        StartState, VehicleModelKind, VehicleStatus,
    };
    use std::sync::{Arc, Mutex};
    use std::thread;
    use std::time::Duration;

    fn algorithm(name: &str, race_line: bool, lidar: bool) -> AvailableAlgorithm {
        AvailableAlgorithm {
            name: name.to_string(),
            label: name.to_string(),
            requires: AlgorithmRequirements { race_line, lidar },
            ..AvailableAlgorithm::default()
        }
    }

    fn algorithms() -> Vec<AvailableAlgorithm> {
        vec![
            algorithm("always_left", false, false),
            algorithm("gap_follower", false, true),
            algorithm("pure_pursuit", true, false),
        ]
    }

    fn spec(algorithm: &str, race_line: Option<&str>) -> OpponentSpec {
        OpponentSpec {
            color: Default::default(),
            race_line: race_line.map(str::to_string),
            algorithm: algorithm.to_string(),
            speed_scale: 0.5,
            limits: SimulatedVehicleConfig::default().limits,
        }
    }

    #[test]
    fn a_valid_spec_is_accepted() {
        let lines = vec!["line.csv".to_string()];
        assert_eq!(
            validate(&spec("gap_follower", None), &algorithms(), &lines)
                .unwrap()
                .name,
            "gap_follower"
        );
        assert!(
            validate(
                &spec("pure_pursuit", Some("line.csv")),
                &algorithms(),
                &lines
            )
            .is_ok()
        );
        // A race line an algorithm doesn't need is still fine.
        assert!(
            validate(
                &spec("gap_follower", Some("line.csv")),
                &algorithms(),
                &lines
            )
            .is_ok()
        );
    }

    #[test]
    fn a_spec_without_a_steering_angle_is_accepted() {
        // What the web GUI's form sends: every limit but the steering angle,
        // which is the car's.
        let mut limits = serde_json::to_value(SimulatedVehicleConfig::default().limits).unwrap();
        limits
            .as_object_mut()
            .unwrap()
            .remove("max_steering_angle_rad");
        let spec = OpponentSpec {
            limits: serde_json::from_value(limits).unwrap(),
            ..spec("gap_follower", None)
        };
        assert_eq!(spec.limits.max_steering_angle_rad, 0.0);
        assert!(validate(&spec, &algorithms(), &[]).is_ok());
    }

    #[test]
    fn an_algorithm_following_a_race_line_needs_one() {
        let err = validate(&spec("pure_pursuit", None), &algorithms(), &[]).unwrap_err();
        assert!(err.contains("race line"), "{err}");
    }

    #[test]
    fn invalid_specs_are_refused() {
        let lines = vec!["line.csv".to_string()];
        assert!(validate(&spec("nope", None), &algorithms(), &lines).is_err());
        assert!(
            validate(
                &spec("pure_pursuit", Some("other.csv")),
                &algorithms(),
                &lines
            )
            .is_err()
        );
        for speed_scale in [-0.1, 1.1, f64::NAN] {
            let spec = OpponentSpec {
                speed_scale,
                ..spec("gap_follower", None)
            };
            assert!(
                validate(&spec, &algorithms(), &lines).is_err(),
                "{speed_scale}"
            );
        }
        let mut too_fast = spec("gap_follower", None);
        too_fast.limits.max_speed_mps = 1000.0;
        let err = validate(&too_fast, &algorithms(), &lines).unwrap_err();
        assert!(err.contains("max_speed_mps"), "{err}");
    }

    fn ego_model() -> VehicleModelStatus {
        VehicleModelStatus {
            kind: VehicleModelKind::Bicycle,
            ..VehicleModelStatus::default()
        }
    }

    fn executor_names(algorithm: &str, requires_lidar: bool, race_line: bool) -> Vec<String> {
        let vehicle_config = SimulatedVehicleConfig::default();
        let ego_model = ego_model();
        executors(
            7,
            &spec(algorithm, None),
            Ingredients {
                requires_lidar,
                race_line: race_line.then(SelectedRaceLine::default),
                ego_model: &ego_model,
                vehicle_config: &vehicle_config,
                lidar_config: SimulatedLidarConfig::default(),
            },
        )
        .unwrap()
        .iter()
        .map(|executor| executor.name())
        .collect()
    }

    #[test]
    fn an_opponent_runs_only_the_executors_its_algorithm_needs() {
        assert_eq!(
            executor_names("always_left", false, false),
            ["opponent/7/vehicle", "opponent/7/always_left"]
        );
        assert_eq!(
            executor_names("gap_follower", true, false),
            [
                "opponent/7/vehicle",
                "opponent/7/gap_follower",
                "opponent/7/lidar"
            ]
        );
        assert_eq!(
            executor_names("pure_pursuit", false, true),
            [
                "opponent/7/vehicle",
                "opponent/7/pure_pursuit",
                "opponent/7/race_line"
            ]
        );
    }

    /// Stops everything if dropped while panicking - so a failed check in a
    /// test executor fails the test instead of leaving it running forever.
    struct StopOnPanic<'a>(&'a Captain);

    impl Drop for StopOnPanic<'_> {
        fn drop(&mut self) {
            if thread::panicking() {
                self.0.stop();
            }
        }
    }

    /// Plays a script of requests against an [`OpponentsManager`], writing
    /// down what it saw after each step.
    struct Driver {
        id: u16,
        log: Arc<Mutex<Vec<String>>>,
    }

    impl Driver {
        /// Polls `condition` for up to two seconds; whether it became true.
        fn eventually(condition: impl Fn() -> bool) -> bool {
            (0..2000).any(|_| {
                thread::sleep(Duration::from_millis(1));
                condition()
            })
        }
    }

    impl Executor for Driver {
        fn init(&mut self, id: u16) {
            self.id = id;
        }

        fn claim_writing_topics(&mut self, captain: &Captain) {
            captain.claim_writer::<OpponentRequests>(
                OPPONENT_REQUESTS_TOPIC_NAME,
                self.id,
                OpponentRequests::default,
            );
            captain.claim_writer::<SelectedMap>(MAP_TOPIC_NAME, self.id, SelectedMap::default);
        }

        fn run(&mut self, captain: &Captain) {
            let _stop_on_panic = StopOnPanic(captain);
            let requests = captain.topic::<OpponentRequests>(OPPONENT_REQUESTS_TOPIC_NAME);
            let opponents = captain.topic::<Opponents>(OPPONENTS_TOPIC_NAME);
            let request = |request: OpponentRequest| {
                let mut all = requests.read().into_value();
                let number = all.push(request);
                requests.write(self.id, all).unwrap();
                Self::eventually(|| {
                    opponents
                        .read()
                        .last_outcome
                        .as_ref()
                        .is_some_and(|outcome| outcome.request == number)
                });
                opponents.read().into_value()
            };
            let driving = |id: u32| {
                captain
                    .try_topic::<VehicleStatus>(&VehicleTopics::opponent(id).vehicle_status())
                    .is_some_and(|topic| topic.meta().write_count > 0)
            };
            let registered = |id: u32| {
                captain
                    .try_topic::<VehicleStatus>(&VehicleTopics::opponent(id).vehicle_status())
                    .is_some()
            };
            let log = |line: String| self.log.lock().unwrap().push(line);

            let added = request(OpponentRequest::Add(spec("always_left", None)));
            log(format!(
                "added {:?} driving {}",
                added.last_outcome.unwrap().error,
                Self::eventually(|| driving(1))
            ));

            let refused = request(OpponentRequest::Add(spec("pure_pursuit", None)));
            log(format!(
                "refused {} listed {}",
                refused.last_outcome.unwrap().error.is_some(),
                refused.list.len()
            ));

            let deleted = request(OpponentRequest::Delete(1));
            log(format!(
                "deleted {:?} gone {}",
                deleted.last_outcome.unwrap().error,
                Self::eventually(|| !registered(1))
            ));

            // Numbers aren't reused: the next opponent is 2.
            request(OpponentRequest::Add(spec("always_left", None)));
            let second = Self::eventually(|| driving(2));
            captain
                .topic::<SelectedMap>(MAP_TOPIC_NAME)
                .write(
                    self.id,
                    SelectedMap {
                        path: Some("/another/map".into()),
                        ..SelectedMap::default()
                    },
                )
                .unwrap();
            let cleared = Self::eventually(|| !registered(2) && opponents.read().list.is_empty());
            log(format!("second {second} cleared by a map change {cleared}"));

            captain.request_restart();
        }

        fn name(&self) -> String {
            "Driver".to_string()
        }

        fn as_any(&self) -> &dyn Any {
            self
        }

        fn fresh(&self) -> Box<dyn Executor> {
            Box::new(Stopper)
        }
    }

    /// After the restart: checks nothing came back, then stops everything.
    struct Stopper;

    impl Executor for Stopper {
        fn init(&mut self, _id: u16) {}

        fn run(&mut self, captain: &Captain) {
            let _stop_on_panic = StopOnPanic(captain);
            thread::sleep(Duration::from_millis(100));
            assert!(
                captain
                    .topic::<Opponents>(OPPONENTS_TOPIC_NAME)
                    .read()
                    .list
                    .is_empty()
            );
            captain.stop();
        }

        fn name(&self) -> String {
            "Stopper".to_string()
        }

        fn as_any(&self) -> &dyn Any {
            self
        }

        fn fresh(&self) -> Box<dyn Executor> {
            Box::new(Stopper)
        }
    }

    #[test]
    fn opponents_are_added_refused_deleted_and_cleared_while_everything_runs() {
        let mut runner = Runner::new();
        runner.register_topic(START_STATE_TOPIC_NAME, StartState::default);
        runner.register_topic(PLACE_AT_START_TOPIC_NAME, PlaceAtStart::default);
        runner.register_topic(VEHICLE_MODEL_STATUS_TOPIC_NAME, ego_model);
        runner.register_topic(AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME, || {
            AutonomousAlgorithmStatus {
                available: algorithms(),
                ..AutonomousAlgorithmStatus::default()
            }
        });
        let log = Arc::new(Mutex::new(Vec::new()));
        runner.add_executor(Box::new(Driver {
            id: 0,
            log: log.clone(),
        }));
        runner.add_executor(
            OpponentsManager::new(
                "OpponentsManager",
                SimulatedVehicleConfig::default(),
                SimulatedLidarConfig::default(),
            )
            .boxed(),
        );

        runner.run_until_stopped();

        assert_eq!(
            *log.lock().unwrap(),
            [
                "added None driving true",
                "refused true listed 1",
                "deleted None gone true",
                "second true cleared by a map change true",
            ]
        );
    }
}
