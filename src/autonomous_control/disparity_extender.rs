//! The Disparity Extender, a reactive algorithm: wherever two neighbouring
//! LIDAR readings differ a lot (a "disparity", e.g. the edge of an obstacle),
//! the nearer one is extended over enough readings on the farther side for
//! the vehicle to fit, then the vehicle steers toward the farthest reading
//! left. Based on
//! <https://www.nathanotterness.com/2019/04/the-disparity-extender-algorithm-and.html>,
//! ported from ubm's `disparity_extender.cpp`. See
//! `documentation/autonomous_algorithms.md`.

use crate::autonomous_control::shared::reactive::{
    fov_window, ray, ray_step_rad, scan_origin, speed_proportional_steering, world_point,
};
use crate::autonomous_control::{Instance, ParameterTuner, load_config};
use crate::topics::{
    ActuatorLimits, AlgorithmParameter, AutonomousAlgorithmInfo, Color, Drawing, LidarScan,
    Shape, VescCommand,
};
use crate::{Captain, Executor, Ticker};
use std::any::Any;
use std::ops::Range;
use std::time::Duration;

/// Entry point build.rs calls - required, with exactly this signature.
pub fn new(instance: Instance) -> Box<dyn Executor> {
    Box::new(DisparityExtender {
        id: 0,
        config: load_config(&instance.config_name),
        instance,
    })
}

/// Every tunable parameter [`DisparityExtender`] needs - loaded from
/// `config/autonomous_control/disparity_extender.toml` at runtime (see [`load_config`]), falling
/// back to the copy compiled in (see [`Default`]). Every field can also be tuned live - see
/// [`parameters`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DisparityExtenderConfig {
    /// Rate at which a new control is published, in Hz.
    pub rate_hz: f32,
    /// Part of the scan looked at, centred straight ahead, in degrees.
    pub desired_fov_deg: f32,
    /// Readings are clipped to this, in meters.
    pub max_range_m: f32,
    /// Vehicle width, in meters.
    pub car_width_m: f32,
    /// Two neighbouring readings further apart than this are a disparity, in meters.
    pub disparity_threshold_m: f32,
    /// Multiplies the number of readings a disparity is extended over, pure number.
    pub r_multiplier: f32,
    /// Readings within this of the farthest are equally good, and the one
    /// nearest to straight ahead wins; 0 picks the farthest, in meters.
    pub ray_eq_thr_m: f32,
    /// Among equally good readings, those within this of the one nearest to
    /// straight ahead are equal too, and `angle_priority` picks, in radians.
    pub angle_eq_thr_rad: f32,
    /// Which way a tie goes: 0 = toward negative angles (left), 1 = toward
    /// positive ones (right).
    pub angle_priority: u8,
    /// Speed when driving straight, in m/s.
    pub max_speed: f32,
    /// Speed at full steering lock, in m/s.
    pub min_speed: f32,
}

impl Default for DisparityExtenderConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/autonomous_control/disparity_extender.toml"))
            .expect("config/autonomous_control/disparity_extender.toml must deserialize into DisparityExtenderConfig")
    }
}

/// The live-tunable parameters, one per [`DisparityExtenderConfig`] field -
/// see [`ParameterTuner`].
fn parameters() -> [AlgorithmParameter; 11] {
    [
        // At least a few Hz: below 1 Hz every command would be stale on arrival
        // (see `VESC_COMMAND_TIMEOUT`), holding the vehicle stopped.
        AlgorithmParameter::float("rate_hz", 5.0, 200.0, 1.0)
            .unit("Hz")
            .description("Rate at which a new control is published."),
        AlgorithmParameter::float("desired_fov_deg", 30.0, 360.0, 5.0)
            .unit("deg")
            .description("Part of the scan looked at, centred straight ahead."),
        AlgorithmParameter::float("max_range_m", 1.0, 30.0, 0.5)
            .unit("m")
            .description("Readings are clipped to this."),
        AlgorithmParameter::float("car_width_m", 0.05, 1.0, 0.01)
            .unit("m")
            .description("Vehicle width: how far a disparity is extended."),
        AlgorithmParameter::float("disparity_threshold_m", 0.05, 5.0, 0.05)
            .unit("m")
            .description("Two neighbouring readings further apart than this are a disparity."),
        AlgorithmParameter::float("r_multiplier", 0.0, 5.0, 0.1)
            .description("Multiplies the number of readings a disparity is extended over."),
        AlgorithmParameter::float("ray_eq_thr_m", 0.0, 5.0, 0.05)
            .unit("m")
            .description("Readings within this of the farthest are equally good; 0 picks the farthest."),
        AlgorithmParameter::float("angle_eq_thr_rad", 0.0, 1.0, 0.01)
            .unit("rad")
            .description("Equally good directions within this of the straightest one tie."),
        AlgorithmParameter::int("angle_priority", 0, 1, 1)
            .description("Which way a tie goes: 0 = left (negative angles), 1 = right."),
        AlgorithmParameter::float("max_speed", 0.0, 10.0, 0.1)
            .unit("m/s")
            .description("Speed when driving straight."),
        AlgorithmParameter::float("min_speed", 0.0, 10.0, 0.1)
            .unit("m/s")
            .description("Speed at full steering lock."),
    ]
}

struct DisparityExtender {
    id: u16,
    instance: Instance,
    config: DisparityExtenderConfig,
}

impl Executor for DisparityExtender {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_autonomous_control(
            self.id,
            &self.instance.algorithm_topics(),
            AutonomousAlgorithmInfo::new(
                "Disparity extender",
                "Extends the edges of obstacles by the car's width, then steers toward the farthest reading",
            )
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
        let mut stale_after = drawing_stale_after(&self.config);

        while captain.is_running(self.id) {
            if tuner.update(captain, &mut self.config) {
                ticker = Ticker::new(self.config.rate_hz as f64);
                stale_after = drawing_stale_after(&self.config);
            }

            let scan = scan_topic.read().into_value();
            let limits = limits_topic.read();
            let Some(control) = control(&self.config, &scan, &limits) else {
                ticker.wait();
                continue;
            };

            command_topic
                .write(self.id, VescCommand::new(control.steering_rad as f64, control.speed_mps as f64))
                .expect("lost writer authorization for this algorithm's command topic");

            let mut drawing = Drawing::default();
            if let Some(origin) = scan_origin(captain, &self.instance.vehicle) {
                let processed = control
                    .window
                    .clone()
                    .map(|i| world_point(origin, scan.angle_rad(i), control.processed[i]))
                    .collect();
                drawing = drawing
                    .element(
                        "Extended ranges",
                        [Shape::Points { points: processed, radius_px: 2.0, color: Color::GREEN }],
                        false,
                    )
                    .element(
                        "Chosen direction",
                        [ray(origin, control.target_rad, control.processed[control.target], Color::PURPLE)],
                        false,
                    );
            }
            drawing_topic
                .write(self.id, drawing.stale_after(stale_after))
                .expect("lost writer authorization for the disparity extender drawing topic");
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

/// How long the drawing stays valid: a few publishing periods, but never
/// less than the default.
fn drawing_stale_after(config: &DisparityExtenderConfig) -> Duration {
    Drawing::DEFAULT_STALE_AFTER.max(Duration::from_secs_f64(3.0 / config.rate_hz as f64))
}

/// What one tick decided.
#[derive(Debug, Clone, PartialEq)]
struct Control {
    steering_rad: f32,
    speed_mps: f32,
    /// The readings looked at.
    window: Range<usize>,
    /// Every reading, clipped and with the disparities extended.
    processed: Vec<f32>,
    /// The reading steered toward, and its direction.
    target: usize,
    target_rad: f32,
}

/// The disparity extender on `scan` - `None` if it has too few readings.
fn control(config: &DisparityExtenderConfig, scan: &LidarScan, limits: &ActuatorLimits) -> Option<Control> {
    let window = fov_window(scan, config.desired_fov_deg.to_radians());
    if window.len() < 2 {
        return None;
    }
    let processed = extend_disparities(config, scan, window.clone());
    let target = choose(config, scan, &processed, window.clone());
    let target_rad = scan.angle_rad(target);
    let max_steering = limits.max_steering_angle_rad as f32;
    let steering_rad = target_rad.clamp(-max_steering, max_steering);
    let speed_mps = speed_proportional_steering(steering_rad, max_steering, config.max_speed, config.min_speed)
        .clamp(0.0, limits.max_speed_mps as f32);
    Some(Control { steering_rad, speed_mps, window, processed, target, target_rad })
}

/// `scan`'s readings clipped to `max_range_m`, with every disparity in
/// `window` extended: the nearer reading of the pair overwrites (where it's
/// nearer) enough readings on the farther side, starting at the farther
/// one, to cover half the car's width at its distance - times
/// `r_multiplier`.
fn extend_disparities(config: &DisparityExtenderConfig, scan: &LidarScan, window: Range<usize>) -> Vec<f32> {
    let clipped: Vec<f32> = scan.points.iter().map(|&r| r.min(config.max_range_m)).collect();
    let mut processed = clipped.clone();
    let step = ray_step_rad(scan);
    for i in window.start + 1..window.end {
        let (before, after) = (clipped[i - 1], clipped[i]);
        if (after - before).abs() <= config.disparity_threshold_m {
            continue;
        }
        let near = before.min(after);
        let theta = (config.car_width_m / 2.0 / near.max(1e-3)).atan();
        let count = ((theta / step).round() * config.r_multiplier).round() as usize;
        // The farther side, walking away from the nearer reading.
        let far_side: Box<dyn Iterator<Item = usize>> = if after > before {
            Box::new((i..window.end).take(count))
        } else {
            Box::new((window.start..i).rev().take(count))
        };
        for j in far_side {
            processed[j] = processed[j].min(near);
        }
    }
    processed
}

/// The reading in `window` to steer toward - see
/// [`DisparityExtenderConfig::ray_eq_thr_m`] and the fields after it.
fn choose(config: &DisparityExtenderConfig, scan: &LidarScan, processed: &[f32], window: Range<usize>) -> usize {
    let toward_positive = config.angle_priority == 1;
    let farthest = window.clone().map(|i| processed[i]).fold(f32::MIN, f32::max);
    // Lower indices are the more negative angles.
    let pick = |candidates: Vec<usize>| {
        if toward_positive { *candidates.last().unwrap() } else { candidates[0] }
    };

    if config.ray_eq_thr_m <= 0.0 {
        return pick(window.filter(|&i| processed[i] >= farthest).collect());
    }
    let good: Vec<usize> = window.filter(|&i| processed[i] >= farthest - config.ray_eq_thr_m).collect();
    let straightest = good
        .iter()
        .copied()
        .min_by(|&a, &b| scan.angle_rad(a).abs().total_cmp(&scan.angle_rad(b).abs()))
        .expect("the farthest reading is always good");
    let straightest_rad = scan.angle_rad(straightest);
    pick(
        good.into_iter()
            .filter(|&i| (scan.angle_rad(i) - straightest_rad).abs() <= config.angle_eq_thr_rad)
            .collect(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autonomous_control::shared::reactive::tests::scan;
    use std::f32::consts::PI;

    fn limits() -> ActuatorLimits {
        ActuatorLimits {
            max_steering_angle_rad: 0.4,
            max_steering_rate_rad_s: 4.0,
            max_speed_mps: 10.0,
            max_accel_mps2: 4.0,
            max_decel_mps2: 8.0,
        }
    }

    fn config() -> DisparityExtenderConfig {
        DisparityExtenderConfig {
            desired_fov_deg: 180.0,
            car_width_m: 0.30,
            disparity_threshold_m: 0.5,
            r_multiplier: 2.0,
            ray_eq_thr_m: 0.0,
            angle_priority: 0,
            ..Default::default()
        }
    }

    #[test]
    fn a_disparity_is_extended_over_the_farther_side() {
        // 1 degree per reading: near (2 m) up to 89, far (5 m) from 90.
        let mut scan = scan(181, PI, 5.0);
        scan.points[..90].fill(2.0);
        let processed = extend_disparities(&config(), &scan, 0..181);
        // atan(0.15 / 2) = 4.3 degrees -> 4 readings, times 2.
        assert!(processed[90..98].iter().all(|&r| r == 2.0), "{:?}", &processed[88..100]);
        assert_eq!(processed[98], 5.0);
        assert!(processed[..90].iter().all(|&r| r == 2.0));
    }

    #[test]
    fn ties_follow_the_angle_priority() {
        let scan = scan(181, PI, 5.0);
        let processed = scan.points.clone();
        assert_eq!(choose(&config(), &scan, &processed, 0..181), 0);
        let right = DisparityExtenderConfig { angle_priority: 1, ..config() };
        assert_eq!(choose(&right, &scan, &processed, 0..181), 180);
    }

    #[test]
    fn equally_good_readings_prefer_straight_ahead() {
        let mut scan = scan(181, PI, 5.0);
        scan.points[20] = 5.05;
        let processed = scan.points.clone();
        let tolerant = DisparityExtenderConfig { ray_eq_thr_m: 0.1, angle_eq_thr_rad: 0.0, ..config() };
        assert_eq!(choose(&tolerant, &scan, &processed, 0..181), 90);
        // Without the tolerance, the farthest wins.
        assert_eq!(choose(&config(), &scan, &processed, 0..181), 20);
    }

    #[test]
    fn a_wall_on_the_positive_side_steers_negative() {
        // Near readings on the positive (right) half, open on the negative one.
        let mut scan = scan(181, PI, 1.0);
        for i in 0..70 {
            scan.points[i] = 8.0;
        }
        let control = control(&config(), &scan, &limits()).unwrap();
        assert!(control.steering_rad < 0.0);
        assert!(control.steering_rad >= -0.4);
        assert!(control.speed_mps <= config().max_speed);
    }

    #[test]
    fn every_config_field_is_tunable() {
        let config = DisparityExtenderConfig::default();
        let info = AutonomousAlgorithmInfo::new("Disparity extender", "").with_parameters(&config, parameters());
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
