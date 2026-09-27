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
//! [`Placement`] folds both topics into the one pose the vehicle was last
//! placed at, for everything that re-anchors on a placement.

use super::StartState;

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

/// Tracks the pose a vehicle was last placed at: the
/// [`crate::topics::StartState`] whenever that changes, or a
/// [`PlaceAtStart`] request's pose (the start state when it carries none)
/// whenever that's bumped. Tracked as events rather than recomputed from the
/// two topics, so whichever happened last wins - e.g. a map change after a
/// custom placement goes back to the new start line.
#[derive(Debug, Clone, Copy)]
pub struct Placement {
    start: StartState,
    requested: u64,
    anchor: StartState,
}

impl Placement {
    /// Starts at `start`, taking `place` as already applied.
    pub fn new(start: StartState, place: PlaceAtStart) -> Self {
        Self {
            start,
            requested: place.requested,
            anchor: start,
        }
    }

    /// The pose the vehicle was placed at if a placement happened since the
    /// last call, else `None`. With `follows_poses` false (an opponent), a
    /// request carrying a pose is consumed but ignored.
    pub fn update(
        &mut self,
        start: StartState,
        place: PlaceAtStart,
        follows_poses: bool,
    ) -> Option<StartState> {
        let start_changed = start != self.start;
        let bumped = place.requested != self.requested;
        self.start = start;
        self.requested = place.requested;
        let anchor = match (bumped, place.pose) {
            (true, Some(pose)) if follows_poses => pose,
            (true, None) => start,
            _ if start_changed => start,
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

    fn pose(x_m: f64) -> StartState {
        StartState {
            x_m,
            ..StartState::default()
        }
    }

    fn request(requested: u64, pose: Option<StartState>) -> PlaceAtStart {
        PlaceAtStart { requested, pose }
    }

    #[test]
    fn nothing_changed_places_nothing() {
        let mut placement = Placement::new(pose(1.0), request(3, None));
        assert_eq!(placement.update(pose(1.0), request(3, None), true), None);
        assert_eq!(placement.anchor(), pose(1.0));
    }

    #[test]
    fn a_start_change_places_at_the_new_start() {
        let mut placement = Placement::new(pose(1.0), PlaceAtStart::default());
        assert_eq!(
            placement.update(pose(2.0), PlaceAtStart::default(), true),
            Some(pose(2.0))
        );
        assert_eq!(placement.anchor(), pose(2.0));
    }

    #[test]
    fn a_bump_without_a_pose_places_at_the_start() {
        let mut placement = Placement::new(pose(1.0), PlaceAtStart::default());
        assert_eq!(
            placement.update(pose(1.0), request(1, None), true),
            Some(pose(1.0))
        );
    }

    #[test]
    fn a_bump_with_a_pose_places_there_until_the_start_changes() {
        let mut placement = Placement::new(pose(1.0), PlaceAtStart::default());
        assert_eq!(
            placement.update(pose(1.0), request(1, Some(pose(5.0))), true),
            Some(pose(5.0))
        );
        assert_eq!(
            placement.update(pose(1.0), request(1, Some(pose(5.0))), true),
            None
        );
        assert_eq!(placement.anchor(), pose(5.0));
        assert_eq!(
            placement.update(pose(2.0), request(1, Some(pose(5.0))), true),
            Some(pose(2.0))
        );
        assert_eq!(placement.anchor(), pose(2.0));
    }

    #[test]
    fn an_opponent_ignores_a_pose() {
        let mut placement = Placement::new(pose(1.0), PlaceAtStart::default());
        assert_eq!(
            placement.update(pose(1.0), request(1, Some(pose(5.0))), false),
            None
        );
        assert_eq!(placement.anchor(), pose(1.0));
        assert_eq!(
            placement.update(pose(1.0), request(2, None), false),
            Some(pose(1.0))
        );
    }
}
