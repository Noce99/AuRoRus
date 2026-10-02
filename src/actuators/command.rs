//! Which [`VescCommand`] a vehicle acts on: the arbitration between the
//! autonomous command and the human ones, shared by the real car's
//! [`super::Vesc`] and [`crate::simulation::SimulatedVehicle`].

use crate::Stamped;
use crate::topics::{VESC_COMMAND_TIMEOUT, VescCommand};
use std::time::Duration;

/// Whether `command` was written, and recently enough to act on - see
/// [`VESC_COMMAND_TIMEOUT`].
pub(crate) fn is_fresh(command: &Stamped<VescCommand>) -> bool {
    is_fresh_within(command, VESC_COMMAND_TIMEOUT)
}

/// Whether `command` was written at most `timeout` ago.
fn is_fresh_within(command: &Stamped<VescCommand>, timeout: Duration) -> bool {
    command.age().is_some_and(|age| age <= timeout)
}

/// Picks the command to act on this tick. A human one always overrides: the
/// first of `humans` (in priority order - the joystick's before `web_gui`'s)
/// that's fresh and asks for anything at all wins - `web_gui` and the
/// joystick re-send a stationary, centered command while no control is held,
/// so "fresh" alone can't mean "the human is driving". Otherwise the
/// autonomous one is used if fresh, and a stationary, centered command if
/// neither is - so a writer that stopped publishing never leaves its last
/// setpoint latched.
pub(crate) fn select_command(
    autonomous: Stamped<VescCommand>,
    humans: impl IntoIterator<Item = Stamped<VescCommand>>,
) -> VescCommand {
    select_command_within(autonomous, humans, VESC_COMMAND_TIMEOUT)
}

/// [`select_command`], with commands older than `timeout` counting as stale
/// - the real car's (see [`super::Vesc`]) is shorter than the simulator's.
pub(crate) fn select_command_within(
    autonomous: Stamped<VescCommand>,
    humans: impl IntoIterator<Item = Stamped<VescCommand>>,
    timeout: Duration,
) -> VescCommand {
    if let Some(human) = humans
        .into_iter()
        .find(|human| is_fresh_within(human, timeout) && human.value != VescCommand::default())
    {
        human.value
    } else if is_fresh_within(&autonomous, timeout) {
        autonomous.value
    } else {
        VescCommand::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::WriteMeta;

    fn written(
        servo_position_rad: f64,
        speed_mps: f64,
        written_at: std::time::Instant,
    ) -> Stamped<VescCommand> {
        Stamped {
            value: VescCommand::new(servo_position_rad, speed_mps),
            meta: WriteMeta {
                write_count: 1,
                written_at: Some(written_at),
                written_at_unix_us: 1,
            },
        }
    }

    #[test]
    fn select_command_lets_an_active_human_override_the_autonomous_command() {
        let now = std::time::Instant::now();
        let human = written(0.0, 1.0, now);
        let autonomous = written(-0.4, 4.0, now + std::time::Duration::from_millis(1));
        assert_eq!(
            select_command(autonomous, [human]),
            VescCommand::new(0.0, 1.0)
        );
    }

    #[test]
    fn select_command_ignores_an_idle_human_heartbeat() {
        let now = std::time::Instant::now();
        let human = written(0.0, 0.0, now + std::time::Duration::from_millis(1));
        let autonomous = written(-0.4, 4.0, now);
        assert_eq!(
            select_command(autonomous, [human]),
            VescCommand::new(-0.4, 4.0)
        );
    }

    #[test]
    fn select_command_stops_on_stale_or_unwritten_commands() {
        let stale_at =
            std::time::Instant::now() - VESC_COMMAND_TIMEOUT - std::time::Duration::from_millis(10);
        let seed = Stamped {
            value: VescCommand::new(0.3, 5.0),
            meta: WriteMeta::default(),
        };
        assert_eq!(
            select_command(written(-0.4, 4.0, stale_at), [written(0.1, 1.0, stale_at)]),
            VescCommand::default()
        );
        assert_eq!(select_command(seed.clone(), [seed]), VescCommand::default());
    }

    #[test]
    fn select_command_prefers_the_first_active_human() {
        let now = std::time::Instant::now();
        let autonomous = written(-0.4, 4.0, now);
        assert_eq!(
            select_command(
                autonomous.clone(),
                [written(0.2, 2.0, now), written(0.1, 1.0, now)]
            ),
            VescCommand::new(0.2, 2.0)
        );
        // An idle joystick hands over to WASD, not straight to autonomy.
        assert_eq!(
            select_command(autonomous, [written(0.0, 0.0, now), written(0.1, 1.0, now)]),
            VescCommand::new(0.1, 1.0)
        );
    }
}
