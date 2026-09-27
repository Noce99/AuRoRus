//! The [`RaceStart`] topic: `web_gui`'s "Start race" - every vehicle placed
//! on its own starting grid slot (see
//! [`crate::environment::starting_grid`]), held there until
//! [`RaceStart::go_at_ms`], then released all at once. Each
//! [`crate::actuators::SimulatedVehicle`] finds its own slot by its
//! [`VehicleTopics`]; see [`crate::topics::Placement`] for how a race start
//! folds into everything else that re-anchors on a placement.

use super::{StartState, VehicleTopics};
use std::time::{SystemTime, UNIX_EPOCH};

/// Name of the topic a [`RaceStart`] is published on.
pub const RACE_START_TOPIC_NAME: &str = "race_start";

/// One vehicle of a race.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, serde::Serialize, serde::Deserialize,
)]
#[serde(rename_all = "snake_case")]
pub enum Racer {
    Ego,
    /// The opponent with this [`crate::topics::Opponent::id`].
    Opponent(u32),
}

impl Racer {
    /// Where this vehicle's topics live.
    pub fn topics(self) -> VehicleTopics {
        match self {
            Self::Ego => VehicleTopics::ego(),
            Self::Opponent(id) => VehicleTopics::opponent(id),
        }
    }
}

/// Where one [`Racer`] starts.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct GridSlot {
    pub racer: Racer,
    pub pose: StartState,
}

/// The latest race start. `sequence` is a counter like
/// [`crate::topics::PlaceAtStart::requested`]: every vehicle with a slot is
/// placed there whenever it no longer matches what it last applied.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RaceStart {
    pub sequence: u64,
    /// Pole position first.
    pub slots: Vec<GridSlot>,
    /// When the vehicles are released, in [`now_ms`]'s clock - until then,
    /// every vehicle with a slot stands still whatever it's commanded.
    pub go_at_ms: u64,
}

impl RaceStart {
    /// Where `vehicle` starts, if it's racing.
    pub fn slot(&self, vehicle: &VehicleTopics) -> Option<StartState> {
        self.slots
            .iter()
            .find(|slot| slot.racer.topics() == *vehicle)
            .map(|slot| slot.pose)
    }

    /// Whether `vehicle` is still held on the grid at `now_ms`.
    pub fn holds(&self, vehicle: &VehicleTopics, now_ms: u64) -> bool {
        now_ms < self.go_at_ms && self.slot(vehicle).is_some()
    }
}

/// Milliseconds since the Unix epoch - a clock every executor, and the
/// browser counting down, agrees on.
pub fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |elapsed| elapsed.as_millis() as u64)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn race(go_at_ms: u64) -> RaceStart {
        RaceStart {
            sequence: 1,
            slots: vec![
                GridSlot {
                    racer: Racer::Opponent(2),
                    pose: StartState {
                        x_m: 1.0,
                        ..StartState::default()
                    },
                },
                GridSlot {
                    racer: Racer::Ego,
                    pose: StartState {
                        x_m: 2.0,
                        ..StartState::default()
                    },
                },
            ],
            go_at_ms,
        }
    }

    #[test]
    fn each_racer_finds_its_own_slot() {
        let race = race(0);
        assert_eq!(
            race.slot(&VehicleTopics::ego()).map(|pose| pose.x_m),
            Some(2.0)
        );
        assert_eq!(
            race.slot(&VehicleTopics::opponent(2)).map(|pose| pose.x_m),
            Some(1.0)
        );
        assert_eq!(race.slot(&VehicleTopics::opponent(3)), None);
    }

    #[test]
    fn racers_are_held_until_the_go() {
        let race = race(1_000);
        assert!(race.holds(&VehicleTopics::ego(), 999));
        assert!(!race.holds(&VehicleTopics::ego(), 1_000));
        assert!(!race.holds(&VehicleTopics::opponent(3), 999));
    }
}
