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
//! [`all`], which the binaries run. The file must define
//!
//! ```ignore
//! pub fn new(name: &str) -> Box<dyn crate::Executor>
//! ```
//!
//! returning an executor whose [`Executor::name`] is `name` (the file stem),
//! which:
//! - claims its topics in [`Executor::claim_writing_topics`] via
//!   [`Captain::claim_autonomous_control`], describing itself with an
//!   [`crate::topics::AutonomousAlgorithmInfo`] for the picker;
//! - publishes its commands in [`Executor::run`] on
//!   [`Captain::autonomous_control`] - more often than
//!   [`crate::topics::VESC_COMMAND_TIMEOUT`], or the vehicle stops;
//! - reads whatever else it needs, e.g. the actuator limits on
//!   [`crate::topics::VEHICLE_LIMITS_TOPIC_NAME`];
//! - if it's expensive to run, can idle while
//!   [`Captain::is_selected_algorithm`] says it isn't selected;
//! - optionally, lets its parameters be tuned live - see [`ParameterTuner`].
//!
//! See `always_left.rs` for the smallest possible example, and
//! `gap_follower.rs` for one with tunable parameters.

use crate::topics::{
    AUTONOMOUS_ALGORITHM_SELECTION_TOPIC_NAME, AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME,
    AUTONOMOUS_CONTROL_INFO_TOPIC_PREFIX, AUTONOMOUS_CONTROL_TOPIC_PREFIX,
    AUTONOMOUS_PARAMETERS_TOPIC_NAME, AUTONOMOUS_VESC_COMMAND_TOPIC_NAME, AlgorithmParameter,
    AutonomousAlgorithmInfo, AutonomousAlgorithmSelection, AutonomousAlgorithmStatus,
    AutonomousParameters, AvailableAlgorithm, ParameterKind, VESC_COMMAND_TIMEOUT, VescCommand,
};
use crate::{Captain, Executor, Stamped, Ticker};
use serde::Serialize;
use serde::de::DeserializeOwned;
use std::any::Any;
use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

include!(concat!(env!("OUT_DIR"), "/autonomous_algorithms.rs"));

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
    id: u8,
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
///     AutonomousAlgorithmInfo::new("Gap follower", "...").with_parameters(
///         &self.config,
///         [AlgorithmParameter::float("t_m", 0.5, 12.0, 0.1).unit("m").description("...")],
///     ),
/// );
/// ```
///
/// then calls [`update`](Self::update) once per tick in [`Executor::run`].
/// The config must be [`Serialize`] + [`DeserializeOwned`] - a value is set
/// by patching its JSON form - so no per-parameter code is needed. Anything
/// the algorithm derives from its config (e.g. a [`Ticker`] built from a
/// rate) must be rebuilt whenever `update` returns `true`.
pub struct ParameterTuner {
    executor_id: u8,
    name: String,
    /// [`crate::WriteMeta::write_count`] of the last [`AutonomousParameters`]
    /// looked at, so an unchanged request costs one counter read per tick.
    seen_write_count: u64,
}

impl ParameterTuner {
    /// A tuner for the algorithm running as `executor_id`, which must have
    /// claimed its topics via [`Captain::claim_autonomous_control`].
    pub fn new(captain: &Captain, executor_id: u8) -> Self {
        Self {
            executor_id,
            name: captain.name_of(executor_id),
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

        let info_topic = captain.autonomous_control_info(self.executor_id);
        let mut info = info_topic.read().into_value();
        if !apply(config, &info.parameters, &wanted) {
            return false;
        }
        info.refresh_values(config);
        info_topic
            .write(self.executor_id, info)
            .expect("lost writer authorization for this algorithm's info topic");
        true
    }
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
    let text =
        fs::read_to_string(&path).map_err(|err| format!("failed to read {path:?}: {err}"))?;
    let values: Vec<(&str, String)> = parameters
        .iter()
        .map(|parameter| (parameter.name.as_str(), toml_value(parameter)))
        .collect();
    let updated = set_toml_values(&text, &values).map_err(|err| format!("{path:?}: {err}"))?;
    updated
        .parse::<toml::Table>()
        .map_err(|err| format!("{path:?} would no longer parse, not saved: {err}"))?;
    // Written aside then renamed over, so a failure never leaves it half-written.
    let temporary = path.with_extension("toml.tmp");
    fs::write(&temporary, updated)
        .and_then(|()| fs::rename(&temporary, &path))
        .map_err(|err| format!("failed to write {path:?}: {err}"))?;
    Ok(path)
}

/// `parameter`'s value as TOML: an integer for an [`ParameterKind::Int`],
/// otherwise a float with as many decimals as its step has (at least one,
/// so it stays a TOML float) - `3.3`, not the `3.299999952316284` an `f32`
/// field reads back as.
fn toml_value(parameter: &AlgorithmParameter) -> String {
    match parameter.kind {
        ParameterKind::Int { .. } => format!("{}", parameter.value.round() as i64),
        ParameterKind::Float { step, .. } => {
            let decimals = (0..9)
                .find(|&decimals| {
                    let scaled = step * 10f64.powi(decimals);
                    (scaled - scaled.round()).abs() < 1e-6
                })
                .unwrap_or(9)
                .max(1) as usize;
            format!("{:.*}", decimals, parameter.value)
        }
    }
}

/// `text` (a TOML file) with the value of each top-level `key = value` line
/// named in `values` replaced, keeping its indentation, trailing comment,
/// and line ending. Only flat numeric values are expected - every
/// algorithm config is a flat list of them. Fails if a key isn't found.
fn set_toml_values(text: &str, values: &[(&str, String)]) -> Result<String, String> {
    let mut missing: Vec<&str> = values.iter().map(|&(key, _)| key).collect();
    let mut in_table = false;
    let mut out = String::with_capacity(text.len());
    for line in text.split_inclusive('\n') {
        in_table |= line.trim_start().starts_with('[');
        match replace_value(line, values).filter(|_| !in_table) {
            Some((key, replaced)) => {
                missing.retain(|&missing_key| missing_key != key);
                out.push_str(&replaced);
            }
            None => out.push_str(line),
        }
    }
    if !missing.is_empty() {
        return Err(format!("no top-level `{}` to set", missing.join("`, `")));
    }
    Ok(out)
}

/// `line` with its value replaced, if it's a `key = value` line for one of
/// `values` - see [`set_toml_values`].
fn replace_value<'a>(line: &str, values: &[(&'a str, String)]) -> Option<(&'a str, String)> {
    let (lhs, rhs) = line.split_once('=')?;
    let (key, value) = values.iter().find(|(key, _)| *key == lhs.trim())?;
    // A number holds no '#', so one after the '=' starts a trailing comment.
    let value_end = rhs
        .find('#')
        .unwrap_or_else(|| rhs.trim_end_matches(['\n', '\r']).len());
    let (old, rest) = rhs.split_at(value_end);
    let spacing = &old[old.trim_end().len()..];
    Some((key, format!("{lhs}= {value}{spacing}{rest}")))
}

/// Sets every field of `config` named in both `parameters` and `wanted` to
/// its wanted value, sanitized - see [`ParameterTuner::update`]. Returns
/// whether anything changed.
fn apply<C: Serialize + DeserializeOwned>(
    config: &mut C,
    parameters: &[AlgorithmParameter],
    wanted: &BTreeMap<String, f64>,
) -> bool {
    let before = serde_json::to_value(&*config)
        .expect("an algorithm's config must serialize to JSON to be tunable");
    let mut after = before.clone();
    let fields = after
        .as_object_mut()
        .expect("a tunable algorithm's config must be a struct");
    for parameter in parameters {
        if let Some(value) = wanted
            .get(&parameter.name)
            .and_then(|&value| parameter.kind.sanitize(value))
        {
            fields.insert(parameter.name.clone(), parameter.kind.to_json(value));
        }
    }
    if after == before {
        return false;
    }
    *config = serde_json::from_value(after).unwrap_or_else(|err| {
        panic!("a tunable parameter's value doesn't fit its config field: {err}")
    });
    true
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
    fn init(&mut self, id: u8) {
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

    fn wanted(values: &[(&str, f64)]) -> BTreeMap<String, f64> {
        values
            .iter()
            .map(|&(name, value)| (name.to_string(), value))
            .collect()
    }

    #[test]
    fn wanted_values_are_applied_to_their_fields() {
        let mut config = config();
        assert!(apply(
            &mut config,
            &parameters(),
            &wanted(&[("rate_hz", 20.0), ("radius", 30.0)])
        ));
        assert_eq!(
            config,
            Config {
                rate_hz: 20.0,
                radius: 30,
                untouched: 1.0
            }
        );
    }

    #[test]
    fn wanted_values_are_sanitized() {
        let mut config = config();
        assert!(apply(
            &mut config,
            &parameters(),
            &wanted(&[("rate_hz", 0.0), ("radius", 12.6)])
        ));
        assert_eq!(
            config,
            Config {
                rate_hz: 5.0,
                radius: 13,
                untouched: 1.0
            }
        );
        assert!(apply(
            &mut config,
            &parameters(),
            &wanted(&[("radius", -4.0)])
        ));
        assert_eq!(config.radius, 0);
        assert!(!apply(
            &mut config,
            &parameters(),
            &wanted(&[("rate_hz", f64::NAN)])
        ));
    }

    #[test]
    fn undeclared_or_unchanged_values_change_nothing() {
        let mut config = config();
        assert!(!apply(
            &mut config,
            &parameters(),
            &wanted(&[("untouched", 7.0), ("nope", 1.0)])
        ));
        assert!(!apply(
            &mut config,
            &parameters(),
            &wanted(&[("rate_hz", 50.0), ("radius", 10.0)])
        ));
        assert_eq!(config, self::config());
    }

    #[test]
    fn saved_values_keep_the_rest_of_the_file() {
        let text =
            "# Header\n\n# Rate, in Hz.\nrate_hz = 50.0\nradius=100  # points\r\nother = 1\n";
        let values = [
            ("rate_hz", "20.0".to_string()),
            ("radius", "30".to_string()),
        ];
        assert_eq!(
            set_toml_values(text, &values).unwrap(),
            "# Header\n\n# Rate, in Hz.\nrate_hz = 20.0\nradius= 30  # points\r\nother = 1\n"
        );
    }

    #[test]
    fn saving_fails_on_a_missing_or_nested_key() {
        let text = "rate_hz = 50.0\n[table]\nradius = 3\n";
        let values = [("rate_hz", "1.0".to_string()), ("radius", "4".to_string())];
        assert_eq!(
            set_toml_values(text, &values),
            Err("no top-level `radius` to set".to_string())
        );
    }

    #[test]
    fn saved_values_are_formatted_by_kind() {
        let value =
            |parameter: AlgorithmParameter, value| AlgorithmParameter { value, ..parameter };
        let float = AlgorithmParameter::float("t_m", 0.5, 12.0, 0.1);
        assert_eq!(toml_value(&value(float, 3.299999952316284)), "3.3");
        let whole = AlgorithmParameter::float("rate_hz", 5.0, 200.0, 1.0);
        assert_eq!(toml_value(&value(whole, 50.0)), "50.0");
        let fine = AlgorithmParameter::float("k", 0.0, 1.0, 0.025);
        assert_eq!(toml_value(&value(fine, 0.125)), "0.125");
        let int = AlgorithmParameter::int("radius", 0, 100, 1);
        assert_eq!(toml_value(&value(int, 43.0)), "43");
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
