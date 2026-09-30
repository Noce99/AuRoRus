//! Potential field, a reactive algorithm: every obstacle in the LIDAR scan
//! repels with a Gaussian potential as wide as the obstacle plus the car,
//! the longest reading attracts, and the vehicle steers toward a minimum of
//! the sum. Based on "A Real-Time Obstacle Avoidance Method for Autonomous
//! Vehicles Using an Obstacle-Dependent Gaussian Potential Field"
//! (<https://doi.org/10.1155/2018/5041401>), ported from ubm's
//! `potential_field.cpp`. See `documentation/autonomous_algorithms.md`.

use crate::autonomous_control::shared::reactive::{
    Field, FieldConfig, field_shapes, fov_window, potential_field, scan_origin, speed_proportional_steering,
    speed_steer_and_fov,
};
use crate::autonomous_control::{Instance, ParameterTuner, load_config};
use crate::topics::{
    ActuatorLimits, AlgorithmParameter, AutonomousAlgorithmInfo, Drawing, LidarScan,
    VehicleTopics, VescCommand,
};
use crate::{Captain, Executor, Ticker};
use std::any::Any;
use std::time::Duration;

/// Entry point build.rs calls - required, with exactly this signature.
pub fn new(instance: Instance) -> Box<dyn Executor> {
    Box::new(PotentialField {
        id: 0,
        config: load_config(&instance.config_name),
        instance,
    })
}

/// Every tunable parameter [`PotentialField`] needs - loaded from
/// `config/autonomous_control/potential_field.toml` at runtime (see [`load_config`]), falling
/// back to the copy compiled in (see [`Default`]). Every field can also be tuned live - see
/// [`parameters`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PotentialFieldConfig {
    /// Rate at which a new control is published, in Hz.
    pub rate_hz: f32,
    /// Part of the scan looked at, centred straight ahead, in degrees.
    pub desired_fov_deg: f32,
    /// Angle between two cells of the field, in degrees.
    pub field_resolution_deg: f32,
    /// Readings nearer than this times the mean reading are obstacles, pure number.
    pub obstacle_threshold_gain: f32,
    /// Hysteresis around the obstacle threshold, in meters.
    pub hysteresis_m: f32,
    /// Weight of the pull toward the longest reading, pure number.
    pub attractive_power: f32,
    /// Vehicle width, which widens every obstacle's potential, in meters.
    pub car_width_m: f32,
    /// Multiplies the chosen direction to get the steering angle, pure number.
    pub steering_gain: f32,
    /// 1 = also consider the field's global minimum, not only local ones.
    pub include_global_minima: u8,
    /// 1 = pick the minimum nearest to the attractive direction, 0 = the lowest.
    pub use_minima_near_attractive: u8,
    /// 1 = the speed also grows with the room ahead and drops near a wall.
    pub use_speed_distance_gains: u8,
    /// Part of the scan "the room ahead" is measured over, in degrees.
    pub front_fov_deg: f32,
    /// Speed lost per 1/m of the room ahead, in m²/s.
    pub brake_gain: f32,
    /// Speed gained per meter of room ahead, in 1/s.
    pub speed_distance_gain: f32,
    /// Speed when driving straight, in m/s.
    pub max_speed: f32,
    /// Speed at full steering lock, in m/s.
    pub min_speed: f32,
}

impl Default for PotentialFieldConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/autonomous_control/potential_field.toml"))
            .expect("config/autonomous_control/potential_field.toml must deserialize into PotentialFieldConfig")
    }
}

/// The parameters of the field itself and of the speed law - shared with
/// `potential_pursuit`, whose config has the same fields.
pub(crate) fn field_parameters() -> Vec<AlgorithmParameter> {
    vec![
        AlgorithmParameter::float("desired_fov_deg", 30.0, 360.0, 5.0)
            .unit("deg")
            .description("Part of the scan looked at, centred straight ahead."),
        AlgorithmParameter::float("field_resolution_deg", 0.1, 5.0, 0.1)
            .unit("deg")
            .description("Angle between two cells of the field."),
        AlgorithmParameter::float("obstacle_threshold_gain", 0.1, 2.0, 0.05)
            .description("Readings nearer than this times the mean reading are obstacles."),
        AlgorithmParameter::float("hysteresis_m", 0.0, 2.0, 0.05)
            .unit("m")
            .description("Hysteresis around the obstacle threshold."),
        AlgorithmParameter::float("attractive_power", 0.0, 1.0, 0.005)
            .description("Weight of the pull toward the attractive direction."),
        AlgorithmParameter::float("car_width_m", 0.05, 1.0, 0.01)
            .unit("m")
            .description("Vehicle width, which widens every obstacle's potential."),
        AlgorithmParameter::float("steering_gain", 0.0, 2.0, 0.05)
            .description("Multiplies the chosen direction to get the steering angle."),
        AlgorithmParameter::int("include_global_minima", 0, 1, 1)
            .description("1 = also consider the field's global minimum, not only local ones."),
        AlgorithmParameter::int("use_minima_near_attractive", 0, 1, 1)
            .description("1 = pick the minimum nearest to the attractive direction, 0 = the lowest."),
        AlgorithmParameter::int("use_speed_distance_gains", 0, 1, 1)
            .description("1 = the speed also grows with the room ahead and drops near a wall."),
        AlgorithmParameter::float("front_fov_deg", 1.0, 90.0, 1.0)
            .unit("deg")
            .description("Part of the scan the room ahead is measured over."),
        AlgorithmParameter::float("brake_gain", 0.0, 10.0, 0.1)
            .unit("m²/s")
            .description("Speed lost per 1/m of room ahead."),
        AlgorithmParameter::float("speed_distance_gain", 0.0, 5.0, 0.05)
            .unit("1/s")
            .description("Speed gained per meter of room ahead."),
        AlgorithmParameter::float("max_speed", 0.0, 10.0, 0.1)
            .unit("m/s")
            .description("Speed when driving straight."),
        AlgorithmParameter::float("min_speed", 0.0, 10.0, 0.1)
            .unit("m/s")
            .description("Speed at full steering lock."),
    ]
}

/// The live-tunable parameters, one per [`PotentialFieldConfig`] field -
/// see [`ParameterTuner`].
fn parameters() -> Vec<AlgorithmParameter> {
    // At least a few Hz: below 1 Hz every command would be stale on arrival
    // (see `VESC_COMMAND_TIMEOUT`), holding the vehicle stopped.
    let mut parameters = vec![
        AlgorithmParameter::float("rate_hz", 5.0, 200.0, 1.0)
            .unit("Hz")
            .description("Rate at which a new control is published."),
    ];
    parameters.extend(field_parameters());
    parameters
}

impl PotentialFieldConfig {
    pub(crate) fn field(&self) -> FieldConfig {
        FieldConfig {
            fov_rad: self.desired_fov_deg.to_radians(),
            resolution_rad: self.field_resolution_deg.to_radians(),
            obstacle_threshold_gain: self.obstacle_threshold_gain,
            hysteresis_m: self.hysteresis_m,
            attractive_power: self.attractive_power,
            car_width_m: self.car_width_m,
            include_global_minima: self.include_global_minima == 1,
            use_minima_near_attractive: self.use_minima_near_attractive == 1,
        }
    }
}

/// The command for `field`: [`PotentialFieldConfig::steering_gain`] times
/// its chosen direction, and a speed from the steering (and the room ahead,
/// if enabled), both within `limits`.
pub(crate) fn command(
    config: &PotentialFieldConfig,
    scan: &LidarScan,
    field: &Field,
    limits: &ActuatorLimits,
) -> (f32, f32) {
    let max_steering = limits.max_steering_angle_rad as f32;
    let steering_rad = (config.steering_gain * field.chosen_rad()).clamp(-max_steering, max_steering);
    let mut speed = speed_proportional_steering(steering_rad, max_steering, config.max_speed, config.min_speed);
    if config.use_speed_distance_gains == 1 {
        let front = fov_window(scan, config.front_fov_deg.to_radians());
        speed = speed_steer_and_fov(speed, &scan.points[front], config.speed_distance_gain, config.brake_gain);
    }
    (steering_rad, speed.clamp(0.0, limits.max_speed_mps as f32))
}

/// The drawing of `field`, seen from where `scan` was taken on the vehicle
/// whose topics are `vehicle`, if there's a pose to draw it from.
pub(crate) fn field_drawing(
    captain: &Captain,
    vehicle: &VehicleTopics,
    scan: &LidarScan,
    field: &Field,
) -> Drawing {
    let Some(origin) = scan_origin(captain, vehicle, scan) else {
        return Drawing::default();
    };
    let (obstacles, potential, chosen) = field_shapes(origin, field);
    Drawing::default()
        .element("Obstacles", obstacles, false)
        .element("Potential", potential, false)
        .element("Chosen direction", chosen, false)
}

struct PotentialField {
    id: u16,
    instance: Instance,
    config: PotentialFieldConfig,
}

impl Executor for PotentialField {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_autonomous_control(
            self.id,
            &self.instance.algorithm_topics(),
            AutonomousAlgorithmInfo::new(
                "Potential field",
                "Obstacles repel, the longest reading attracts: steers toward a minimum of the potential",
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
        let mut stale_after = drawing_stale_after(self.config.rate_hz);

        while captain.is_running(self.id) {
            if tuner.update(captain, &mut self.config) {
                ticker = Ticker::new(self.config.rate_hz as f64);
                stale_after = drawing_stale_after(self.config.rate_hz);
            }

            let scan = scan_topic.read().into_value();
            let Some(field) = potential_field(&scan, &self.config.field(), |longest_rad| longest_rad) else {
                ticker.wait();
                continue;
            };
            let (steering_rad, speed_mps) = command(&self.config, &scan, &field, &limits_topic.read());

            command_topic
                .write(self.id, VescCommand::new(steering_rad as f64, speed_mps as f64))
                .expect("lost writer authorization for this algorithm's command topic");
            drawing_topic
                .write(self.id, field_drawing(captain, &self.instance.vehicle, &scan, &field).stale_after(stale_after))
                .expect("lost writer authorization for the potential field drawing topic");
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

/// How long a drawing published at `rate_hz` stays valid: a few publishing
/// periods, but never less than the default.
pub(crate) fn drawing_stale_after(rate_hz: f32) -> Duration {
    Drawing::DEFAULT_STALE_AFTER.max(Duration::from_secs_f64(3.0 / rate_hz as f64))
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

    #[test]
    fn an_open_scan_drives_straight_at_full_speed() {
        let config = PotentialFieldConfig::default();
        let scan = scan(361, 4.2, 6.0);
        let field = potential_field(&scan, &config.field(), |longest| longest).unwrap();
        // Nothing repels; with no steering gain, the vehicle drives straight.
        let (steering, speed) = command(&PotentialFieldConfig { steering_gain: 0.0, ..config }, &scan, &field, &limits());
        assert_eq!(steering, 0.0);
        assert_eq!(speed, config.max_speed);
    }

    #[test]
    fn an_obstacle_on_the_positive_side_steers_negative() {
        let config = PotentialFieldConfig { desired_fov_deg: 180.0, ..Default::default() };
        let mut scan = scan(181, PI, 5.0);
        for i in 92..110 {
            scan.points[i] = 1.0;
        }
        // Longest reading straight ahead.
        scan.points[90] = 5.5;
        let field = potential_field(&scan, &config.field(), |longest| longest).unwrap();
        let (steering, speed) = command(&config, &scan, &field, &limits());
        assert!(steering < 0.0, "{steering}");
        assert!(speed < config.max_speed);
    }

    #[test]
    fn steering_is_clamped_to_the_limit() {
        let config = PotentialFieldConfig { steering_gain: 2.0, ..Default::default() };
        let scan = scan(181, PI, 5.0);
        let mut field = potential_field(&scan, &config.field(), |_| 1.2).unwrap();
        field.chosen_cell = field.attractive_cell;
        assert_eq!(command(&config, &scan, &field, &limits()).0, 0.4);
    }

    #[test]
    fn every_config_field_is_tunable() {
        let config = PotentialFieldConfig::default();
        let info = AutonomousAlgorithmInfo::new("Potential field", "").with_parameters(&config, parameters());
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
