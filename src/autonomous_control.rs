//! Autonomous driving algorithms, and [`AutonomousControlsHandler`], which
//! picks the one in control.
//!
//! Every algorithm is its own [`Executor`], running on its own thread at its
//! own rate, publishing the [`VescCommand`] it would like the vehicle to
//! follow on its own topic (see [`crate::topics::AUTONOMOUS_CONTROL_TOPIC_PREFIX`]).
//! [`AutonomousControlsHandler`] forwards whichever one
//! [`AUTONOMOUS_ALGORITHM_SELECTION_TOPIC_NAME`] names (e.g. picked in
//! `web_gui`) to [`AUTONOMOUS_VESC_COMMAND_TOPIC_NAME`], the only autonomous
//! command the vehicle acts on - so switching algorithms is instant, and a
//! `--debug` recording captures what *every* algorithm wanted, not only the
//! one driving.
//!
//! # Adding an algorithm
//!
//! Add one file, `src/autonomous_control/<name>.rs`, and rebuild - nothing
//! else. `build.rs` declares it as a module of this one and adds it to
//! [`all`], which the binaries run, and to [`build`]. The file must define
//!
//! ```ignore
//! pub fn new(instance: crate::autonomous_control::Instance) -> Box<dyn crate::Executor>
//! ```
//!
//! returning an executor whose [`Executor::name`] is `instance.name`, which:
//! - loads its config with [`load_config`]`(&instance.config_name)`;
//! - claims its topics in [`Executor::claim_writing_topics`] via
//!   [`Captain::claim_autonomous_control`] with [`Instance::algorithm_topics`],
//!   describing itself with an [`crate::topics::AutonomousAlgorithmInfo`] for
//!   the picker - including what it [requires](crate::topics::AlgorithmRequirements);
//! - publishes its commands in [`Executor::run`] on
//!   [`Captain::autonomous_control`] - more often than
//!   [`crate::topics::VESC_COMMAND_TIMEOUT`], or the vehicle stops;
//! - reads every topic of the vehicle it drives through
//!   [`Instance::vehicle`] (e.g. the actuator limits on
//!   [`crate::topics::VehicleTopics::vehicle_limits`]) - the same algorithm
//!   also drives opponents (see [`Instance::opponent`]), each through its own
//!   topics;
//! - if it's expensive to run, can idle while
//!   [`Captain::is_selected_algorithm`] says it isn't selected - but only
//!   when [`Instance::is_opponent`] is `false`: an opponent is never
//!   "selected", it always drives;
//! - optionally, lets its parameters be tuned live - see [`ParameterTuner`];
//! - optionally, tells the driver what's going on (e.g. why it holds the
//!   vehicle stopped) - see [`report_message`].
//!
//! See `always_left.rs` for the smallest possible example, and
//! `gap_follower.rs` for one with tunable parameters.
//!
//! Code several algorithms share goes in [`shared`] - a directory, since
//! every `.rs` file directly in `src/autonomous_control/` is an algorithm.

use crate::topics::{
    AUTONOMOUS_ALGORITHM_SELECTION_TOPIC_NAME, AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME,
    AUTONOMOUS_CONTROL_INFO_TOPIC_PREFIX, AUTONOMOUS_CONTROL_TOPIC_PREFIX,
    AUTONOMOUS_PARAMETERS_TOPIC_NAME, AUTONOMOUS_VESC_COMMAND_TOPIC_NAME, AlgorithmParameter,
    AlgorithmTopics, AutonomousAlgorithmInfo, AutonomousAlgorithmSelection,
    AutonomousAlgorithmStatus, AutonomousParameters, AvailableAlgorithm, VESC_COMMAND_TIMEOUT,
    VehicleTopics, VescCommand,
};
use crate::{Captain, Executor, Stamped, Ticker};
use serde::Serialize;
use serde::de::DeserializeOwned;
use std::any::Any;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

include!(concat!(env!("OUT_DIR"), "/autonomous_algorithms.rs"));

pub(crate) mod shared;

/// Which vehicle one running copy of an algorithm drives, and under what
/// name - what every algorithm file's `new` is built from (see the
/// [module docs](self)).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Instance {
    /// The executor's name - e.g. `gap_follower` for the ego vehicle's copy,
    /// `opponent/1/gap_follower` for opponent 1's.
    pub name: String,
    /// The algorithm's file stem, e.g. `gap_follower`: which config file it
    /// loads (see [`config_path`]) and what its topics are named after.
    pub config_name: String,
    /// The topics of the vehicle it drives.
    pub vehicle: VehicleTopics,
}

impl Instance {
    /// The copy driving the ego vehicle, named after its file stem.
    pub fn ego(stem: &str) -> Self {
        Self {
            name: stem.to_string(),
            config_name: stem.to_string(),
            vehicle: VehicleTopics::ego(),
        }
    }

    /// The copy driving opponent `n`. Same config file as the ego's copy,
    /// but it always drives - it's never "selected" in the picker, nor tuned
    /// live, and it uses the simulator's ground truth pose, never
    /// localization.
    pub fn opponent(n: u32, stem: &str) -> Self {
        let vehicle = VehicleTopics::opponent(n);
        Self {
            name: format!("{}{stem}", vehicle.prefix()),
            config_name: stem.to_string(),
            vehicle,
        }
    }

    pub fn is_opponent(&self) -> bool {
        !self.vehicle.is_ego()
    }

    /// This copy's own command and info topics.
    pub fn algorithm_topics(&self) -> AlgorithmTopics {
        self.vehicle.algorithm(&self.config_name)
    }
}

/// How often [`AutonomousControlsHandler`] forwards the selected command, in
/// Hz - matched to `SimulatedVehicle`'s default tick rate, so forwarding adds
/// at most one vehicle tick of latency.
const HANDLER_RATE_HZ: f64 = 100.0;

/// The single writer of [`AUTONOMOUS_VESC_COMMAND_TOPIC_NAME`]: every tick,
/// finds every algorithm (by its info topic), reads which one
/// [`AUTONOMOUS_ALGORITHM_SELECTION_TOPIC_NAME`] wants, and forwards that
/// algorithm's latest command - or a stationary, centered one if nothing is
/// selected, or its command is missing or older than
/// [`VESC_COMMAND_TIMEOUT`] (see [`resolve_command`]). Reports what it did on
/// [`AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME`].
pub struct AutonomousControlsHandler {
    id: u16,
    name: String,
}

impl AutonomousControlsHandler {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            id: 0,
            name: name.into(),
        }
    }
}

/// Every algorithm whose info topic is currently registered, sorted by name.
fn discover(captain: &Captain) -> Vec<AvailableAlgorithm> {
    let mut available: Vec<AvailableAlgorithm> = captain
        .debug_topics_snapshot()
        .into_iter()
        .filter_map(|(topic_name, _)| {
            let name = topic_name
                .strip_prefix(AUTONOMOUS_CONTROL_INFO_TOPIC_PREFIX)?
                .to_string();
            // An info-prefixed topic of another type is someone's mistake -
            // skip it rather than take the whole process down over it.
            let info = captain
                .try_topic::<AutonomousAlgorithmInfo>(&topic_name)?
                .read()
                .into_value();
            Some(AvailableAlgorithm {
                name,
                label: info.label,
                description: info.description,
                parameters: info.parameters,
                message: info.message,
                requires: info.requires,
            })
        })
        .collect();
    available.sort_by(|a, b| a.name.cmp(&b.name));
    available
}

/// Applies live parameter changes to an algorithm's config: whatever
/// [`AUTONOMOUS_PARAMETERS_TOPIC_NAME`] (written by e.g. `web_gui`) wants
/// for it, reported back - as the values actually applied - in its
/// [`AutonomousAlgorithmInfo`], which is where the UI reads them from.
///
/// An algorithm declares its tunable parameters once, in
/// [`Executor::claim_writing_topics`], each named after the field of its
/// config it sets:
///
/// ```ignore
/// captain.claim_autonomous_control(
///     self.id,
///     &self.instance.algorithm_topics(),
///     AutonomousAlgorithmInfo::new("Gap follower", "...").with_parameters(
///         &self.config,
///         [AlgorithmParameter::float("t_m", 0.5, 12.0, 0.1).unit("m").description("...")],
///     ),
/// );
/// ```
///
/// then calls [`update`](Self::update) once per tick in [`Executor::run`].
/// An opponent's copy (see [`Instance::opponent`]) is never tuned live:
/// `update` leaves its config as loaded.
/// The config must be [`Serialize`] + [`DeserializeOwned`] - a value is set
/// by patching its JSON form - so no per-parameter code is needed. Anything
/// the algorithm derives from its config (e.g. a [`Ticker`] built from a
/// rate) must be rebuilt whenever `update` returns `true`.
pub struct ParameterTuner {
    executor_id: u16,
    /// What [`AutonomousParameters`] knows the algorithm as - its executor
    /// name.
    name: String,
    topics: AlgorithmTopics,
    opponent: bool,
    /// [`crate::WriteMeta::write_count`] of the last [`AutonomousParameters`]
    /// looked at, so an unchanged request costs one counter read per tick.
    seen_write_count: u64,
}

impl ParameterTuner {
    /// A tuner for `instance`, running as `executor_id`, which must have
    /// claimed its topics via [`Captain::claim_autonomous_control`].
    pub fn new(executor_id: u16, instance: &Instance) -> Self {
        Self {
            executor_id,
            name: instance.name.clone(),
            topics: instance.algorithm_topics(),
            opponent: instance.is_opponent(),
            seen_write_count: 0,
        }
    }

    /// Applies any newly requested parameter values to `config`, each
    /// sanitized (see [`crate::topics::ParameterKind::sanitize`]), and
    /// republishes the algorithm's info with the values now in effect.
    /// Returns whether `config` changed. Requests for parameters the
    /// algorithm didn't declare are ignored.
    ///
    /// # Panics
    ///
    /// Panics if a patched config no longer deserializes - a parameter
    /// declared with a kind its field can't hold, e.g. a float for a `usize`.
    pub fn update<C: Serialize + DeserializeOwned>(
        &mut self,
        captain: &Captain,
        config: &mut C,
    ) -> bool {
        if self.opponent {
            return false;
        }
        // Nothing may publish requests at all (e.g. a binary without
        // `web_gui`) - then the config stays as loaded.
        let Some(requests) =
            captain.try_topic::<AutonomousParameters>(AUTONOMOUS_PARAMETERS_TOPIC_NAME)
        else {
            return false;
        };
        let write_count = requests.meta().write_count;
        if write_count == self.seen_write_count {
            return false;
        }
        self.seen_write_count = write_count;
        let Some(wanted) = requests.read().into_value().values.remove(&self.name) else {
            return false;
        };

        let info_topic = captain.autonomous_control_info(&self.topics);
        let mut info = info_topic.read().into_value();
        if !crate::config::apply_parameters(config, &info.parameters, &wanted) {
            return false;
        }
        info.refresh_values(config);
        info_topic
            .write(self.executor_id, info)
            .expect("lost writer authorization for this algorithm's info topic");
        true
    }
}

/// Sets `instance`'s (running as `executor_id`)
/// [`AutonomousAlgorithmInfo::message`] - shown in `web_gui`'s autonomous
/// algorithms panel - rewriting its info only if the message changed, so it
/// can be called every tick.
pub fn report_message(
    captain: &Captain,
    executor_id: u16,
    instance: &Instance,
    message: Option<String>,
) {
    let info_topic = captain.autonomous_control_info(&instance.algorithm_topics());
    let mut info = info_topic.read().into_value();
    if info.message == message {
        return;
    }
    info.message = message;
    info_topic
        .write(executor_id, info)
        .expect("lost writer authorization for this algorithm's info topic");
}

/// Where the algorithm called `name` keeps its config:
/// `config/autonomous_control/<name>.toml`, relative to the working
/// directory - read by [`load_config`], rewritten by [`save_parameters`].
pub fn config_path(name: &str) -> PathBuf {
    Path::new(crate::config::DEFAULT_CONFIG_ROOT)
        .join("autonomous_control")
        .join(format!("{name}.toml"))
}

/// The algorithm called `name`'s config, read from [`config_path`] when the
/// algorithm is built - so values saved from the UI apply on the next
/// restart - or `C::default()` if it can't be read (e.g. when run from
/// another directory).
pub fn load_config<C: DeserializeOwned + Default>(name: &str) -> C {
    crate::config::load(&config_path(name)).unwrap_or_else(|err| {
        eprintln!("{name}: {err} - using the built-in defaults");
        C::default()
    })
}

/// Writes `parameters`' values into the algorithm called `name`'s config
/// file ([`config_path`]), leaving everything else in it - comments, other
/// keys, layout - untouched. Returns the file's path.
pub fn save_parameters(name: &str, parameters: &[AlgorithmParameter]) -> Result<PathBuf, String> {
    let path = config_path(name);
    let values: Vec<(&str, String)> = parameters
        .iter()
        .map(|parameter| {
            (
                parameter.name.as_str(),
                crate::config::parameter_toml_value(parameter),
            )
        })
        .collect();
    crate::config::save_toml_values(&path, None, &values)?;
    Ok(path)
}

/// The values the algorithm called `name`'s config file ([`config_path`])
/// holds for `parameters`, by name, and the file's path - e.g. to go back
/// to what was last saved.
pub fn saved_values(
    name: &str,
    parameters: &[AlgorithmParameter],
) -> Result<(BTreeMap<String, f64>, PathBuf), String> {
    let path = config_path(name);
    let names: Vec<&str> = parameters
        .iter()
        .map(|parameter| parameter.name.as_str())
        .collect();
    Ok((crate::config::load_toml_values(&path, None, &names)?, path))
}

/// The command to forward, given the `selected` algorithm's latest
/// `command` (`None` if nothing is selected, or it has no command topic),
/// and whether that command was fresh: written, and no older than
/// [`VESC_COMMAND_TIMEOUT`]. Anything else resolves to a stationary,
/// centered command.
fn resolve_command(command: Option<Stamped<VescCommand>>) -> (VescCommand, bool) {
    match command {
        Some(command) if command.age().is_some_and(|age| age <= VESC_COMMAND_TIMEOUT) => {
            (command.value, true)
        }
        _ => (VescCommand::default(), false),
    }
}

impl Executor for AutonomousControlsHandler {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_writer::<VescCommand>(
            AUTONOMOUS_VESC_COMMAND_TOPIC_NAME,
            self.id,
            VescCommand::default,
        );
        captain.claim_writer::<AutonomousAlgorithmStatus>(
            AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME,
            self.id,
            AutonomousAlgorithmStatus::default,
        );
    }

    fn run(&mut self, captain: &Captain) {
        let command_topic = captain.topic::<VescCommand>(AUTONOMOUS_VESC_COMMAND_TOPIC_NAME);
        let status_topic =
            captain.topic::<AutonomousAlgorithmStatus>(AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME);
        let mut last_status = None;
        let mut ticker = Ticker::new(HANDLER_RATE_HZ);

        while captain.is_running(self.id) {
            let available = discover(captain);
            // Nothing may publish a selection at all (e.g. a binary without
            // `web_gui`) - then nothing is ever selected.
            let selection = captain
                .try_topic::<AutonomousAlgorithmSelection>(
                    AUTONOMOUS_ALGORITHM_SELECTION_TOPIC_NAME,
                )
                .map(|topic| topic.read().into_value())
                .unwrap_or_default();
            let selected = selection
                .name
                .filter(|name| available.iter().any(|algorithm| &algorithm.name == name));
            let active = selected.clone().filter(|_| selection.running);
            let command = active.as_ref().and_then(|name| {
                captain
                    .try_topic::<VescCommand>(&format!("{AUTONOMOUS_CONTROL_TOPIC_PREFIX}{name}"))
                    .map(|topic| topic.read())
            });
            let (command, command_fresh) = resolve_command(command);

            command_topic
                .write(self.id, command)
                .expect("lost writer authorization for the autonomous_vesc_command topic");

            let status = AutonomousAlgorithmStatus {
                selected,
                active,
                available,
                command_fresh,
            };
            if last_status.as_ref() != Some(&status) {
                status_topic
                    .write(self.id, status.clone())
                    .expect("lost writer authorization for the autonomous_algorithm_status topic");
                last_status = Some(status);
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
        Box::new(Self::new(self.name.clone()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::WriteMeta;
    use std::time::{Duration, Instant};

    #[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    struct Config {
        rate_hz: f32,
        radius: usize,
        untouched: f32,
    }

    fn config() -> Config {
        Config {
            rate_hz: 50.0,
            radius: 10,
            untouched: 1.0,
        }
    }

    fn parameters() -> Vec<AlgorithmParameter> {
        vec![
            AlgorithmParameter::float("rate_hz", 5.0, 200.0, 1.0),
            AlgorithmParameter::int("radius", 0, 100, 1),
        ]
    }

    #[test]
    fn info_reports_the_values_in_effect() {
        let info =
            AutonomousAlgorithmInfo::new("Test", "").with_parameters(&config(), parameters());
        let values: Vec<f64> = info.parameters.iter().map(|p| p.value).collect();
        assert_eq!(values, vec![50.0, 10.0]);
    }

    fn written_at(at: Instant) -> Stamped<VescCommand> {
        Stamped {
            value: VescCommand::new(-0.4, 4.0),
            meta: WriteMeta {
                write_count: 1,
                written_at: Some(at),
                written_at_unix_us: 1,
            },
        }
    }

    #[test]
    fn a_fresh_command_is_forwarded() {
        assert_eq!(
            resolve_command(Some(written_at(Instant::now()))),
            (VescCommand::new(-0.4, 4.0), true)
        );
    }

    #[test]
    fn a_stale_command_stops_the_vehicle() {
        let stale = written_at(Instant::now() - VESC_COMMAND_TIMEOUT - Duration::from_millis(10));
        assert_eq!(
            resolve_command(Some(stale)),
            (VescCommand::default(), false)
        );
    }

    #[test]
    fn an_unwritten_seed_or_no_selection_stops_the_vehicle() {
        let seed = Stamped {
            value: VescCommand::new(-0.4, 4.0),
            meta: WriteMeta::default(),
        };
        assert_eq!(resolve_command(Some(seed)), (VescCommand::default(), false));
        assert_eq!(resolve_command(None), (VescCommand::default(), false));
    }
}
