//! Where a vehicle is, for whoever needs its pose on the map: the pose
//! sources an algorithm can pick between - localization ([`POSE_LOCALIZATION`])
//! or the simulator's ground truth ([`POSE_GROUND_TRUTH`]).

use crate::Captain;
use crate::geometry::Pose;
use crate::topics::{
    Odometry, SLAM_STATUS_TOPIC_NAME, SlamState, SlamStatus, VehicleStatus, VehicleTopics,
};
use std::time::Duration;

/// How old the pose may get before it's no longer trusted.
pub(crate) const POSE_TIMEOUT: Duration = Duration::from_millis(300);

/// A `pose_source` value: odometry composed onto SLAM's `map_to_odom`.
pub(crate) const POSE_LOCALIZATION: u8 = 0;
/// A `pose_source` value: the simulator's `vehicle_status`.
pub(crate) const POSE_GROUND_TRUTH: u8 = 1;

/// The pose of the vehicle whose topics are `vehicle` from `source`
/// ([`POSE_LOCALIZATION`] or [`POSE_GROUND_TRUTH`]), or why there's none.
pub(crate) fn pose(captain: &Captain, vehicle: &VehicleTopics, source: u8) -> Result<Pose, String> {
    match source {
        POSE_LOCALIZATION => localization_pose(captain, vehicle),
        POSE_GROUND_TRUTH => ground_truth_pose(captain, vehicle),
        // Unreachable when tuned: the tuner keeps it in range.
        _ => Err(format!("Unknown pose source {source}.")),
    }
}

/// The simulator's ground truth, if fresh - else why not.
pub(crate) fn ground_truth_pose(
    captain: &Captain,
    vehicle: &VehicleTopics,
) -> Result<Pose, String> {
    let status = captain
        .try_topic::<VehicleStatus>(&vehicle.vehicle_status())
        .ok_or("No ground truth pose (vehicle_status) in this binary.")?
        .read();
    if status.age().is_none_or(|age| age > POSE_TIMEOUT) {
        return Err("The ground truth pose (vehicle_status) is stale.".into());
    }
    Ok(Pose {
        x_m: status.x_m,
        y_m: status.y_m,
        heading_rad: status.heading_rad,
    })
}

/// Odometry's latest pose composed onto SLAM's `map_to_odom` - the pose on
/// the map at odometry's rate - while SLAM is localizing (not paused, where
/// the pose would only be dead-reckoned), odometry is fresh, and both agree
/// on odometry's frame - else why not.
/// There's only the ego vehicle's localization (`slam_status`): an opponent's
/// algorithm uses [`POSE_GROUND_TRUTH`] instead.
pub(crate) fn localization_pose(
    captain: &Captain,
    vehicle: &VehicleTopics,
) -> Result<Pose, String> {
    let slam = captain
        .try_topic::<SlamStatus>(SLAM_STATUS_TOPIC_NAME)
        .ok_or("No localization (slam_status) in this binary.")?
        .read()
        .into_value();
    if slam.state != SlamState::Localizing {
        return Err("Localization isn't running - start it in the Localization panel.".into());
    }
    let [x_m, y_m, heading_rad] = slam.map_to_odom.ok_or("Localization has no pose yet.")?;
    let odometry = captain
        .try_topic::<Odometry>(&vehicle.odometry())
        .ok_or("No odometry in this binary.")?
        .read();
    if odometry.age().is_none_or(|age| age > POSE_TIMEOUT) {
        return Err("Odometry is stale.".into());
    }
    if odometry.reset_count != slam.odometry_reset_count {
        return Err("Odometry was reset - waiting for localization to catch up.".into());
    }
    let map_to_odom = Pose {
        x_m,
        y_m,
        heading_rad,
    };
    Ok(map_to_odom.compose(&Pose {
        x_m: odometry.x_m,
        y_m: odometry.y_m,
        heading_rad: odometry.heading_rad,
    }))
}

/// The speed of the vehicle whose topics are `vehicle` from `source`
/// ([`POSE_LOCALIZATION`]: odometry's, [`POSE_GROUND_TRUTH`]: the
/// simulator's), or why there's none.
pub(crate) fn speed(captain: &Captain, vehicle: &VehicleTopics, source: u8) -> Result<f64, String> {
    match source {
        POSE_LOCALIZATION => Ok(captain
            .try_topic::<Odometry>(&vehicle.odometry())
            .ok_or("No odometry in this binary.")?
            .read()
            .speed_mps),
        POSE_GROUND_TRUTH => Ok(captain
            .try_topic::<VehicleStatus>(&vehicle.vehicle_status())
            .ok_or("No ground truth pose (vehicle_status) in this binary.")?
            .read()
            .speed_mps),
        _ => Err(format!("Unknown pose source {source}.")),
    }
}
