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
//!   [`Captain::is_selected_algorithm`] says it isn't selected.
//!
//! See `always_left.rs` for the smallest possible example.

use crate::topics::{
    AUTONOMOUS_ALGORITHM_SELECTION_TOPIC_NAME, AUTONOMOUS_ALGORITHM_STATUS_TOPIC_NAME,
    AUTONOMOUS_CONTROL_INFO_TOPIC_PREFIX, AUTONOMOUS_CONTROL_TOPIC_PREFIX,
    AUTONOMOUS_VESC_COMMAND_TOPIC_NAME, AutonomousAlgorithmInfo, AutonomousAlgorithmSelection,
    AutonomousAlgorithmStatus, AvailableAlgorithm, VESC_COMMAND_TIMEOUT, VescCommand,
};
use crate::{Captain, Executor, Stamped, Ticker};
use std::any::Any;

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
            })
        })
        .collect();
    available.sort_by(|a, b| a.name.cmp(&b.name));
    available
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
            let selected = captain
                .try_topic::<AutonomousAlgorithmSelection>(
                    AUTONOMOUS_ALGORITHM_SELECTION_TOPIC_NAME,
                )
                .and_then(|topic| topic.read().into_value().name);
            let active =
                selected.filter(|name| available.iter().any(|algorithm| &algorithm.name == name));
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
