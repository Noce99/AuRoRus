//! The [`PlaceAtStart`] topic: an explicit request that
//! [`crate::actuators::SimulatedVehicle`] place the vehicle at whatever
//! [`crate::topics::StartState`] currently holds - even when that value
//! hasn't changed, e.g. the "P" key in `web_gui`'s UI wanting to snap the
//! vehicle back to the start line after it's driven away on the same map -
//! or, carrying a [`PlaceAtStart::pose`], at a pose of its own, e.g.
//! `web_gui`'s place-vehicle tool. [`crate::topics::StartState`] changing on
//! its own already triggers a placement (see its doc comment), so this only
//! needs to cover the cases a plain value comparison can't.
//!
//! [`Placement`] folds both topics - and [`crate::topics::RaceStart`] - into
//! the one pose the vehicle was last placed at, for everything that
//! re-anchors on a placement; [`PlacementTopics`] reads all three.

use super::{RACE_START_TOPIC_NAME, RaceStart, START_STATE_TOPIC_NAME, StartState, VehicleTopics};
use crate::{Captain, RwLockTopic};
use std::sync::Arc;

/// Name of the topic a [`PlaceAtStart`] is published on.
pub const PLACE_AT_START_TOPIC_NAME: &str = "place_at_start";

/// A monotonically increasing counter - `web_gui` bumps it on every
/// placement request; [`crate::actuators::SimulatedVehicle`] places the
/// vehicle whenever this no longer matches what it last applied. A counter
/// rather than a bool so two rapid requests can't cancel each other out via
/// toggle parity.
#[derive(Debug, Clone, Copy, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct PlaceAtStart {
    pub requested: u64,
    /// Where to place the ego vehicle, or `None` for the
    /// [`crate::topics::StartState`]. Opponents only ever go back to the
    /// start state: a request carrying a pose leaves them where they are.
    pub pose: Option<StartState>,
}

/// Everything a placement can come from, as read at one instant.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PlacementInputs {
    pub start: StartState,
    pub place: PlaceAtStart,
    pub race: RaceStart,
}

/// The topics a [`Placement`] follows. [`RACE_START_TOPIC_NAME`] is
/// optional - only `web_gui` ever starts a race.
pub struct PlacementTopics {
    start: Arc<RwLockTopic<StartState>>,
    place: Arc<RwLockTopic<PlaceAtStart>>,
    race: Option<Arc<RwLockTopic<RaceStart>>>,
}

impl PlacementTopics {
    /// # Panics
    ///
    /// Like [`Captain::topic`], if [`START_STATE_TOPIC_NAME`] or
    /// [`PLACE_AT_START_TOPIC_NAME`] was never registered.
    pub fn new(captain: &Captain) -> Self {
        Self {
            start: captain.topic(START_STATE_TOPIC_NAME),
            place: captain.topic(PLACE_AT_START_TOPIC_NAME),
            race: captain.try_topic(RACE_START_TOPIC_NAME),
        }
    }

    /// `None` if [`START_STATE_TOPIC_NAME`] or [`PLACE_AT_START_TOPIC_NAME`]
    /// was never registered.
    pub fn try_new(captain: &Captain) -> Option<Self> {
        Some(Self {
            start: captain.try_topic(START_STATE_TOPIC_NAME)?,
            place: captain.try_topic(PLACE_AT_START_TOPIC_NAME)?,
            race: captain.try_topic(RACE_START_TOPIC_NAME),
        })
    }

    pub fn read(&self) -> PlacementInputs {
        PlacementInputs {
            start: self.start.read().into_value(),
            place: self.place.read().into_value(),
            race: self
                .race
                .as_ref()
                .map(|race| race.read().into_value())
                .unwrap_or_default(),
        }
    }
}

/// Tracks the pose a vehicle was last placed at: the
/// [`crate::topics::StartState`] whenever that changes, a
/// [`PlaceAtStart`] request's pose (the start state when it carries none)
/// whenever that's bumped, or its [`RaceStart`] grid slot whenever a race
/// starts. Tracked as events rather than recomputed from the topics, so
/// whichever happened last wins - e.g. a map change after a custom placement
/// goes back to the new start line.
#[derive(Debug, Clone, Copy)]
pub struct Placement {
    start: StartState,
    requested: u64,
    race_sequence: u64,
    anchor: StartState,
}

impl Placement {
    /// Starts at `inputs.start`, taking its requests as already applied.
    pub fn new(inputs: &PlacementInputs) -> Self {
        Self {
            start: inputs.start,
            requested: inputs.place.requested,
            race_sequence: inputs.race.sequence,
            anchor: inputs.start,
        }
    }

    /// The pose `vehicle` was placed at if a placement happened since the
    /// last call, else `None`. An opponent consumes but ignores a
    /// [`PlaceAtStart`] carrying a pose, and any vehicle a race start with
    /// no slot for it.
    pub fn update(
        &mut self,
        inputs: &PlacementInputs,
        vehicle: &VehicleTopics,
    ) -> Option<StartState> {
        let start_changed = inputs.start != self.start;
        let bumped = inputs.place.requested != self.requested;
        let raced = inputs.race.sequence != self.race_sequence;
        self.start = inputs.start;
        self.requested = inputs.place.requested;
        self.race_sequence = inputs.race.sequence;
        let slot = raced.then(|| inputs.race.slot(vehicle)).flatten();
        let anchor = match (slot, bumped, inputs.place.pose) {
            (Some(slot), _, _) => slot,
            (None, true, Some(pose)) if vehicle.is_ego() => pose,
            (None, true, None) => inputs.start,
            _ if start_changed => inputs.start,
            _ => return None,
        };
        self.anchor = anchor;
        Some(anchor)
    }

    /// The pose the vehicle was last placed at.
    pub fn anchor(&self) -> StartState {
        self.anchor
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topics::{GridSlot, Racer};

    fn pose(x_m: f64) -> StartState {
        StartState {
            x_m,
            ..StartState::default()
        }
    }

    fn inputs(start: f64, requested: u64, place: Option<f64>) -> PlacementInputs {
        PlacementInputs {
            start: pose(start),
            place: PlaceAtStart {
                requested,
                pose: place.map(pose),
            },
            race: RaceStart::default(),
        }
    }

    fn raced(
        mut inputs: PlacementInputs,
        sequence: u64,
        slots: &[(Racer, f64)],
    ) -> PlacementInputs {
        inputs.race = RaceStart {
            sequence,
            slots: slots
                .iter()
                .map(|&(racer, x_m)| GridSlot {
                    racer,
                    pose: pose(x_m),
                })
                .collect(),
            go_at_ms: 0,
        };
        inputs
    }

    fn ego() -> VehicleTopics {
        VehicleTopics::ego()
    }

    #[test]
    fn nothing_changed_places_nothing() {
        let mut placement = Placement::new(&inputs(1.0, 3, None));
        assert_eq!(placement.update(&inputs(1.0, 3, None), &ego()), None);
        assert_eq!(placement.anchor(), pose(1.0));
    }

    #[test]
    fn a_start_change_places_at_the_new_start() {
        let mut placement = Placement::new(&inputs(1.0, 0, None));
        assert_eq!(
            placement.update(&inputs(2.0, 0, None), &ego()),
            Some(pose(2.0))
        );
        assert_eq!(placement.anchor(), pose(2.0));
    }

    #[test]
    fn a_bump_without_a_pose_places_at_the_start() {
        let mut placement = Placement::new(&inputs(1.0, 0, None));
        assert_eq!(
            placement.update(&inputs(1.0, 1, None), &ego()),
            Some(pose(1.0))
        );
    }

    #[test]
    fn a_bump_with_a_pose_places_there_until_the_start_changes() {
        let mut placement = Placement::new(&inputs(1.0, 0, None));
        assert_eq!(
            placement.update(&inputs(1.0, 1, Some(5.0)), &ego()),
            Some(pose(5.0))
        );
        assert_eq!(placement.update(&inputs(1.0, 1, Some(5.0)), &ego()), None);
        assert_eq!(placement.anchor(), pose(5.0));
        assert_eq!(
            placement.update(&inputs(2.0, 1, Some(5.0)), &ego()),
            Some(pose(2.0))
        );
        assert_eq!(placement.anchor(), pose(2.0));
    }

    #[test]
    fn an_opponent_ignores_a_pose() {
        let opponent = VehicleTopics::opponent(1);
        let mut placement = Placement::new(&inputs(1.0, 0, None));
        assert_eq!(
            placement.update(&inputs(1.0, 1, Some(5.0)), &opponent),
            None
        );
        assert_eq!(placement.anchor(), pose(1.0));
        assert_eq!(
            placement.update(&inputs(1.0, 2, None), &opponent),
            Some(pose(1.0))
        );
    }

    #[test]
    fn a_race_start_places_each_racer_on_its_own_slot() {
        let slots = [(Racer::Opponent(1), 7.0), (Racer::Ego, 6.0)];
        let mut ego_placement = Placement::new(&inputs(1.0, 0, None));
        let mut opponent_placement = Placement::new(&inputs(1.0, 0, None));
        let mut outsider_placement = Placement::new(&inputs(1.0, 0, None));
        let race = raced(inputs(1.0, 0, None), 1, &slots);
        assert_eq!(ego_placement.update(&race, &ego()), Some(pose(6.0)));
        assert_eq!(
            opponent_placement.update(&race, &VehicleTopics::opponent(1)),
            Some(pose(7.0))
        );
        assert_eq!(
            outsider_placement.update(&race, &VehicleTopics::opponent(2)),
            None
        );
        assert_eq!(ego_placement.update(&race, &ego()), None);
        assert_eq!(ego_placement.anchor(), pose(6.0));
    }

    #[test]
    fn a_map_change_after_a_race_start_goes_back_to_the_start_line() {
        let slots = [(Racer::Ego, 6.0)];
        let mut placement = Placement::new(&inputs(1.0, 0, None));
        placement.update(&raced(inputs(1.0, 0, None), 1, &slots), &ego());
        assert_eq!(
            placement.update(&raced(inputs(2.0, 0, None), 1, &slots), &ego()),
            Some(pose(2.0))
        );
    }
}
