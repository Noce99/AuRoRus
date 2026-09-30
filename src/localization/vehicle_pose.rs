//! [`VehiclePose`]: where a vehicle is in the world, as well as anything
//! running knows - for drawing what the vehicle sees where it is (e.g.
//! [`crate::sensors::HokuyoLidar`]'s hits), in simulation and on the real
//! car alike.

use crate::topics::{
    Odometry, PLACE_AT_START_TOPIC_NAME, PlaceAtStart, Placement, PlacementTopics,
    SLAM_STATUS_TOPIC_NAME, START_STATE_TOPIC_NAME, SlamStatus, StartState, VehicleStatus,
    VehicleTopics,
};
use crate::{Captain, RwLockTopic};
use std::sync::Arc;
use std::time::Duration;

/// How old a pose source may get before it's ignored.
const POSE_TIMEOUT: Duration = Duration::from_millis(300);

/// A pose `[x_m, y_m, heading_rad]` in the world frame.
pub type WorldPose = [f64; 3];

/// Tracks where a vehicle is in the world, from the best source available:
///
/// 1. the simulator's ground truth ([`VehicleTopics::vehicle_status`]), in
///    simulation;
/// 2. its odometry composed onto SLAM's correction
///    ([`SlamStatus::map_to_odom`]) - on the selected map while localizing,
///    on SLAM's own map (placed where dead reckoning started, as SLAM draws
///    it) while mapping - so it stays on the map SLAM draws, loop closures
///    included;
/// 3. its odometry alone, placed where dead reckoning started, as dead
///    reckoning draws it.
///
/// Unlike the race-line algorithms' localization pose, it takes whatever is
/// there - it's for showing, not for driving.
pub struct VehiclePose {
    vehicle: VehicleTopics,
    truth: Option<Arc<RwLockTopic<VehicleStatus>>>,
    odometry: Option<Arc<RwLockTopic<Odometry>>>,
    /// There's only the ego vehicle's SLAM.
    slam: Option<Arc<RwLockTopic<SlamStatus>>>,
    /// Where dead reckoning's frame sits in the world - `None` without the
    /// topics saying so, placing it at the world's origin.
    placement: Option<(PlacementTopics, Placement)>,
}

impl VehiclePose {
    pub fn new(captain: &Captain, vehicle: VehicleTopics) -> Self {
        let placement = (captain.try_topic::<StartState>(START_STATE_TOPIC_NAME).is_some()
            && captain
                .try_topic::<PlaceAtStart>(PLACE_AT_START_TOPIC_NAME)
                .is_some())
        .then(|| {
            let topics = PlacementTopics::new(captain);
            let placement = Placement::new(&topics.read());
            (topics, placement)
        });
        Self {
            truth: captain.try_topic(&vehicle.vehicle_status()),
            odometry: captain.try_topic(&vehicle.odometry()),
            slam: vehicle
                .is_ego()
                .then(|| captain.try_topic(SLAM_STATUS_TOPIC_NAME))
                .flatten(),
            placement,
            vehicle,
        }
    }

    /// The vehicle's pose now - `None` while nothing fresh says.
    pub fn current(&mut self) -> Option<WorldPose> {
        if let Some(truth) = &self.truth {
            let status = truth.read();
            if is_fresh(status.age()) {
                return Some([status.x_m, status.y_m, status.heading_rad]);
            }
        }

        let anchor = self.anchor();
        let odometry = self.odometry.as_ref()?.read();
        if !is_fresh(odometry.age()) {
            return None;
        }
        let odom = [odometry.x_m, odometry.y_m, odometry.heading_rad];
        if let Some(slam) = &self.slam {
            let slam = slam.read().into_value();
            if let Some(map_to_odom) = slam.map_to_odom
                && slam.odometry_reset_count == odometry.reset_count
            {
                let on_map = compose(map_to_odom, odom);
                return Some(if slam.state.is_localization() {
                    on_map
                } else {
                    compose(anchor, on_map)
                });
            }
        }
        Some(compose(anchor, odom))
    }

    /// Where dead reckoning's frame sits in the world now.
    fn anchor(&mut self) -> WorldPose {
        match &mut self.placement {
            Some((topics, placement)) => {
                placement.update(&topics.read(), &self.vehicle);
                let anchor = placement.anchor();
                [anchor.x_m, anchor.y_m, anchor.heading_rad]
            }
            None => [0.0; 3],
        }
    }
}

fn is_fresh(age: Option<Duration>) -> bool {
    age.is_some_and(|age| age <= POSE_TIMEOUT)
}

/// `other`, given in `frame`'s own frame, in the frame `frame` is in.
fn compose(frame: WorldPose, other: WorldPose) -> WorldPose {
    let (sin, cos) = frame[2].sin_cos();
    [
        frame[0] + other[0] * cos - other[1] * sin,
        frame[1] + other[0] * sin + other[1] * cos,
        frame[2] + other[2],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    #[test]
    fn composing_turns_the_other_pose_into_the_frames() {
        let [x, y, heading] = compose([1.0, 2.0, FRAC_PI_2], [3.0, 0.0, 0.5]);
        assert!((x - 1.0).abs() < 1e-12 && (y - 5.0).abs() < 1e-12);
        assert!((heading - (FRAC_PI_2 + 0.5)).abs() < 1e-12);
    }
}
