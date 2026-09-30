//! Potential pursuit: the [`potential_field`](super::potential_field), but
//! attracted toward a blend of the longest LIDAR reading and a pursuit point
//! a lookahead distance ahead on the selected map's race line - so it
//! follows the line while the field steers it around what's in the way.
//! Ported from ubm's `potential_pursuit_node.cpp`.
//!
//! Needs a pose, like `pure_pursuit` - see [`PotentialPursuitConfig::pose_source`].
//! Without a trustworthy pose, a race line, or while too far from the line,
//! the vehicle is held stopped, and why is reported in the autonomous
//! algorithms panel (see [`report_message`]). See
//! `documentation/autonomous_algorithms.md`.

use super::potential_field::{
    PotentialFieldConfig, command, drawing_stale_after, field_drawing, field_parameters,
};
use crate::autonomous_control::shared::race_line::{
    Line, Nearest, POSE_GROUND_TRUTH, Pose, pose, wrap_to_pi,
};
use crate::autonomous_control::shared::reactive::{Field, potential_field};
use crate::autonomous_control::{Instance, ParameterTuner, load_config, report_message};
use crate::environment::SpeedPoint;
use crate::topics::{
    ActuatorLimits, AlgorithmParameter, AutonomousAlgorithmInfo, Color, Drawing, LidarScan,
    SelectedRaceLine, Shape, VehicleTopics, VescCommand,
};
use crate::{Captain, Executor, Ticker};
use std::any::Any;

/// Entry point build.rs calls - required, with exactly this signature.
pub fn new(instance: Instance) -> Box<dyn Executor> {
    let mut config: PotentialPursuitConfig = load_config(&instance.config_name);
    // An opponent has no localization of its own - see `Instance::opponent`.
    if instance.is_opponent() {
        config.pose_source = POSE_GROUND_TRUTH;
    }
    Box::new(PotentialPursuit {
        id: 0,
        instance,
        config,
    })
}

/// Every tunable parameter [`PotentialPursuit`] needs - loaded from
/// `config/autonomous_control/potential_pursuit.toml` at runtime (see [`load_config`]), falling
/// back to the copy compiled in (see [`Default`]). Every field can also be tuned live - see
/// [`parameters`]. The fields from `desired_fov_deg` on mean what they do in
/// [`PotentialFieldConfig`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PotentialPursuitConfig {
    /// Rate at which a new control is published, in Hz.
    pub rate_hz: f32,
    /// Where the pose comes from: [`POSE_LOCALIZATION`](crate::autonomous_control::shared::race_line::POSE_LOCALIZATION)
    /// or [`POSE_GROUND_TRUTH`](crate::autonomous_control::shared::race_line::POSE_GROUND_TRUTH).
    pub pose_source: u8,
    /// Shortest lookahead distance, in meters.
    pub min_look_ahead_m: f64,
    /// Lookahead growth with the race line's reference speed, in seconds.
    pub look_ahead_gain_s: f64,
    /// Blend of the attractive direction: 0 = the pursuit point, 1 = the
    /// longest reading, pure number.
    pub max_distance_weight: f32,
    /// Farther than this from the line, the vehicle stops, in meters.
    pub max_cross_track_m: f64,
    pub desired_fov_deg: f32,
    pub field_resolution_deg: f32,
    pub obstacle_threshold_gain: f32,
    pub hysteresis_m: f32,
    pub attractive_power: f32,
    pub car_width_m: f32,
    pub steering_gain: f32,
    pub include_global_minima: u8,
    pub use_minima_near_attractive: u8,
    pub use_speed_distance_gains: u8,
    pub front_fov_deg: f32,
    pub brake_gain: f32,
    pub speed_distance_gain: f32,
    pub max_speed: f32,
    pub min_speed: f32,
}

impl Default for PotentialPursuitConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/autonomous_control/potential_pursuit.toml"))
            .expect("config/autonomous_control/potential_pursuit.toml must deserialize into PotentialPursuitConfig")
    }
}

impl PotentialPursuitConfig {
    /// The potential field part of this config.
    fn potential_field(&self) -> PotentialFieldConfig {
        PotentialFieldConfig {
            rate_hz: self.rate_hz,
            desired_fov_deg: self.desired_fov_deg,
            field_resolution_deg: self.field_resolution_deg,
            obstacle_threshold_gain: self.obstacle_threshold_gain,
            hysteresis_m: self.hysteresis_m,
            attractive_power: self.attractive_power,
            car_width_m: self.car_width_m,
            steering_gain: self.steering_gain,
            include_global_minima: self.include_global_minima,
            use_minima_near_attractive: self.use_minima_near_attractive,
            use_speed_distance_gains: self.use_speed_distance_gains,
            front_fov_deg: self.front_fov_deg,
            brake_gain: self.brake_gain,
            speed_distance_gain: self.speed_distance_gain,
            max_speed: self.max_speed,
            min_speed: self.min_speed,
        }
    }
}

/// The live-tunable parameters, one per [`PotentialPursuitConfig`] field -
/// see [`ParameterTuner`].
fn parameters() -> Vec<AlgorithmParameter> {
    let mut parameters = vec![
        // At least a few Hz: below 1 Hz every command would be stale on arrival
        // (see `VESC_COMMAND_TIMEOUT`), holding the vehicle stopped.
        AlgorithmParameter::float("rate_hz", 5.0, 200.0, 1.0)
            .unit("Hz")
            .description("Rate at which a new control is published."),
        AlgorithmParameter::int("pose_source", 0, 1, 1).description(
            "0 = localization (only while localizing), 1 = ground truth (simulation only).",
        ),
        AlgorithmParameter::float("min_look_ahead_m", 0.1, 5.0, 0.05)
            .unit("m")
            .description("Shortest lookahead distance."),
        AlgorithmParameter::float("look_ahead_gain_s", 0.0, 2.0, 0.01)
            .unit("s")
            .description("Lookahead growth with the race line's reference speed."),
        AlgorithmParameter::float("max_distance_weight", 0.0, 1.0, 0.05)
            .description("Attractive direction: 0 = the pursuit point, 1 = the longest reading."),
        AlgorithmParameter::float("max_cross_track_m", 0.2, 5.0, 0.1)
            .unit("m")
            .description("Farther than this from the line, the vehicle stops."),
    ];
    parameters.extend(field_parameters());
    parameters
}

struct PotentialPursuit {
    id: u16,
    instance: Instance,
    config: PotentialPursuitConfig,
}

impl Executor for PotentialPursuit {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_autonomous_control(
            self.id,
            &self.instance.algorithm_topics(),
            AutonomousAlgorithmInfo::new(
                "Potential pursuit",
                "A potential field attracted toward a point ahead on the race line",
            )
            .requires_race_line()
            .requires_lidar()
            .with_parameters(&self.config, parameters()),
        );
        captain.claim_drawing(self.id);
    }

    fn run(&mut self, captain: &Captain) {
        let command_topic = captain.autonomous_control(&self.instance.algorithm_topics());
        let limits_topic = captain.topic::<ActuatorLimits>(&self.instance.vehicle.vehicle_limits());
        let scan_topic = captain.topic::<LidarScan>(&self.instance.vehicle.lidar_scan());
        let drawing_topic = captain.drawing(self.id);
        let mut tuner = ParameterTuner::new(self.id, &self.instance);

        // Both derive from `rate_hz`, so are rebuilt whenever it's tuned.
        let mut ticker = Ticker::new(self.config.rate_hz as f64);
        let mut stale_after = drawing_stale_after(self.config.rate_hz);

        // The race line in use, and the `write_count` it was read at.
        let mut line: Option<(u64, Option<Line>)> = None;
        // Segment the vehicle was nearest last tick - `None` searches the whole line.
        let mut hint: Option<usize> = None;

        while captain.is_running(self.id) {
            if tuner.update(captain, &mut self.config) {
                ticker = Ticker::new(self.config.rate_hz as f64);
                stale_after = drawing_stale_after(self.config.rate_hz);
            }

            // Nothing may publish a race line at all (e.g. a binary without `MapServer`).
            if let Some(topic) =
                captain.try_topic::<SelectedRaceLine>(&self.instance.vehicle.race_line())
            {
                let write_count = topic.meta().write_count;
                if line.as_ref().is_none_or(|(seen, _)| *seen != write_count) {
                    line = Some((write_count, Line::new(topic.read().into_value().points)));
                    hint = None;
                }
            }
            let line = line.as_ref().and_then(|(_, line)| line.as_ref());
            let pose = pose(captain, &self.instance.vehicle, self.config.pose_source);
            let scan = scan_topic.read().into_value();

            // A stationary command, and why, unless driving.
            let stopped = |why: String| (VescCommand::new(0.0, 0.0), Drawing::default(), Some(why));
            let (command, drawing, message) = match (line, pose) {
                (None, _) => {
                    stopped("No race line on the selected map - vehicle held stopped.".into())
                }
                (Some(_), Err(why)) => stopped(format!("{why} Vehicle held stopped.")),
                (Some(line), Ok(pose)) => match control(&self.config, line, pose, hint, &scan) {
                    Err(Lost::OffLine(nearest)) => {
                        // Lost: next tick searches the whole line again.
                        hint = None;
                        stopped(format!(
                            "{:.2} m off the race line (more than max_cross_track_m) - vehicle held stopped.",
                            nearest.distance_m
                        ))
                    }
                    Err(Lost::NoScan) => {
                        stopped("No LIDAR scan yet - vehicle held stopped.".into())
                    }
                    Ok(control) => {
                        hint = Some(control.nearest.segment);
                        let (steering_rad, speed_mps) = command(
                            &self.config.potential_field(),
                            &scan,
                            &control.field,
                            &limits_topic.read(),
                        );
                        (
                            VescCommand::new(steering_rad as f64, speed_mps as f64),
                            control.drawing(captain, &self.instance.vehicle, &scan),
                            None,
                        )
                    }
                },
            };

            report_message(captain, self.id, &self.instance, message);
            command_topic
                .write(self.id, command)
                .expect("lost writer authorization for this algorithm's command topic");
            drawing_topic
                .write(self.id, drawing.stale_after(stale_after))
                .expect("lost writer authorization for the potential pursuit drawing topic");
            ticker.wait();
        }
    }

    fn name(&self) -> String {
        self.instance.name.clone()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn fresh(&self) -> Box<dyn Executor> {
        new(self.instance.clone())
    }
}

/// How far past the last tick's nearest segment the next one is searched
/// for, in meters of arc length: far more than a tick's travel, short
/// enough not to jump to another stretch of track that runs close by.
const SEARCH_WINDOW_M: f64 = 3.0;

/// Why a tick can't drive.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Lost {
    /// Farther than `max_cross_track_m` from the line - where it projects.
    OffLine(Nearest),
    /// The scan has too few readings for a field.
    NoScan,
}

/// What one tick decided.
#[derive(Debug, Clone, PartialEq)]
struct Control {
    nearest: Nearest,
    target: SpeedPoint,
    /// Direction of `target`, in the sensor frame.
    pursuit_rad: f32,
    field: Field,
}

/// The field for `pose` on `line` and `scan`, attracted toward the blend
/// of the pursuit point's direction and the longest reading's.
fn control(
    config: &PotentialPursuitConfig,
    line: &Line,
    pose: Pose,
    hint: Option<usize>,
    scan: &LidarScan,
) -> Result<Control, Lost> {
    let nearest = line.nearest(pose.x_m, pose.y_m, hint, SEARCH_WINDOW_M);
    if nearest.distance_m > config.max_cross_track_m {
        return Err(Lost::OffLine(nearest));
    }
    let reference_mps = line.at(nearest.s_m).speed_mps.max(0.0);
    let lookahead_m = config
        .min_look_ahead_m
        .max(config.look_ahead_gain_s * reference_mps);
    let target = line.at(nearest.s_m + lookahead_m);
    let pursuit_rad =
        wrap_to_pi((target.y - pose.y_m).atan2(target.x - pose.x_m) - pose.heading_rad) as f32;

    let weight = config.max_distance_weight.clamp(0.0, 1.0);
    let field = potential_field(scan, &config.potential_field().field(), |longest_rad| {
        (1.0 - weight) * pursuit_rad + weight * longest_rad
    })
    .ok_or(Lost::NoScan)?;
    Ok(Control {
        nearest,
        target,
        pursuit_rad,
        field,
    })
}

impl Control {
    /// The field's drawing, plus the nearest and pursuit points, for the
    /// vehicle whose topics are `vehicle`, seen from where `scan` was taken.
    fn drawing(&self, captain: &Captain, vehicle: &VehicleTopics, scan: &LidarScan) -> Drawing {
        let nearest = Shape::Circle {
            x_m: self.nearest.x_m,
            y_m: self.nearest.y_m,
            radius_m: 0.06,
            filled: true,
            color: Color::BLUE,
        };
        let target = Shape::Circle {
            x_m: self.target.x,
            y_m: self.target.y,
            radius_m: 0.08,
            filled: true,
            color: Color::GREEN,
        };
        field_drawing(captain, vehicle, scan, &self.field)
            .element("Nearest point", [nearest], false)
            .element("Pursuit point", [target], false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autonomous_control::shared::reactive::tests::scan;
    use std::f32::consts::PI;

    fn point(x: f64, y: f64) -> SpeedPoint {
        SpeedPoint {
            x,
            y,
            speed_mps: 2.0,
        }
    }

    /// A dense circle of radius `radius_m` around the origin, driven in
    /// increasing angle.
    fn circle(radius_m: f64) -> Line {
        let n = 2000;
        Line::new(
            (0..n)
                .map(|i| {
                    let angle = 2.0 * std::f64::consts::PI * i as f64 / n as f64;
                    point(radius_m * angle.cos(), radius_m * angle.sin())
                })
                .collect(),
        )
        .unwrap()
    }

    fn config() -> PotentialPursuitConfig {
        PotentialPursuitConfig {
            max_distance_weight: 0.0,
            desired_fov_deg: 180.0,
            ..Default::default()
        }
    }

    #[test]
    fn without_obstacles_it_is_attracted_toward_the_pursuit_point() {
        // On a circle, heading along it: the pursuit point is on the
        // increasing-heading (positive) side.
        let line = circle(3.0);
        let pose = Pose {
            x_m: 3.0,
            y_m: 0.0,
            heading_rad: std::f64::consts::PI / 2.0,
        };
        let control = control(&config(), &line, pose, None, &scan(181, PI, 5.0)).unwrap();
        assert!(control.pursuit_rad > 0.0);
        let attractive = control.field.angle_rad(control.field.attractive_cell);
        assert!(
            (attractive - control.pursuit_rad).abs() < 0.01,
            "{attractive} vs {}",
            control.pursuit_rad
        );
        // Nothing repels, so there's no minimum but the attraction's.
        assert_eq!(control.field.chosen_cell, control.field.attractive_cell);
    }

    #[test]
    fn too_far_from_the_line_is_lost() {
        let line = circle(3.0);
        let pose = Pose {
            x_m: 6.0,
            y_m: 0.0,
            heading_rad: 0.0,
        };
        let lost = control(&config(), &line, pose, None, &scan(181, PI, 5.0)).unwrap_err();
        assert!(matches!(lost, Lost::OffLine(nearest) if (nearest.distance_m - 3.0).abs() < 1e-6));
    }

    #[test]
    fn every_config_field_is_tunable() {
        let config = PotentialPursuitConfig::default();
        let info = AutonomousAlgorithmInfo::new("Potential pursuit", "")
            .with_parameters(&config, parameters());
        let serde_json::Value::Object(fields) = serde_json::to_value(config).unwrap() else {
            panic!("the config serializes to an object");
        };
        let mut declared: Vec<&str> = info.parameters.iter().map(|p| p.name.as_str()).collect();
        let mut fields: Vec<&str> = fields.keys().map(String::as_str).collect();
        declared.sort_unstable();
        fields.sort_unstable();
        assert_eq!(declared, fields);
    }
}
