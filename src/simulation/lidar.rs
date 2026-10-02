//! [`SimulatedLidar`]: a synthetic LIDAR sensor that raycasts against the
//! currently loaded map from the vehicle's real position, for exercising
//! algorithms against physically grounded readings without real hardware.
//! Every hit's range is corrupted with gaussian noise. Unless configured
//! otherwise, it also sees the other vehicles' bodies (see
//! [`SimulatedLidarConfig::see_vehicles`]). Also draws every hit on its own
//! drawing topic (see [`crate::topics::Drawing`]).

use super::imu::gaussian;
use crate::calibration::CarCalibration;
use crate::environment::cast_ray;
use crate::topics::{
    Color, Drawing, DrawingExt, LidarScan, MAP_TOPIC_NAME, OPPONENTS_TOPIC_NAME, Opponents,
    SelectedMap, Shape, VehicleGeometry, VehicleStatus, VehicleTopics,
};
use crate::{Captain, Executor, Ticker};
use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};
use std::any::Any;
use std::time::Duration;

/// Every tunable parameter [`SimulatedLidar`] needs - loaded from
/// `config/simulation/lidar.toml` (see [`Default`]) or from an
/// arbitrary path via [`crate::config::load`]. Where it's mounted is the
/// simulated car's instead - see [`Self::for_car`], which a loaded config
/// needs before it's used.
#[derive(Debug, Clone, Copy, PartialEq, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SimulatedLidarConfig {
    /// Rate at which [`SimulatedLidar`] publishes a new scan, in Hz.
    pub rate_hz: f64,
    /// How many points [`SimulatedLidar`] puts in every scan.
    pub num_points: usize,
    /// [`SimulatedLidar`]'s reported minimum distance, in meters.
    pub min_distance_m: f32,
    /// [`SimulatedLidar`]'s reported maximum distance, in meters.
    pub max_distance_m: f32,
    /// [`SimulatedLidar`]'s field of view, in radians, centered on the
    /// vehicle's forward direction.
    pub fov_rad: f32,
    /// Seed for the noise generator; `0` picks a fresh random one per run.
    pub seed: u64,
    /// Standard deviation of the zero-mean gaussian noise added to every
    /// hit's range, in meters - `0.0` gives a perfect sensor.
    pub range_std_m: f64,
    /// Whether rays also stop on the other vehicles' bodies - the ego
    /// vehicle's and every opponent's (see [`crate::simulation::opponents`]) - as well as
    /// on the map.
    pub see_vehicles: bool,
    /// Where the sensor sits on the vehicle, in meters, forward of and to
    /// the right of the vehicle's reference point - see
    /// [`LidarScan::mount_x_m`]. The simulated car's - see [`Self::for_car`].
    #[serde(skip)]
    pub mount_x_m: f32,
    #[serde(skip)]
    pub mount_y_m: f32,
}

impl Default for SimulatedLidarConfig {
    /// Mounted on the template car (see [`CarCalibration::template`]).
    fn default() -> Self {
        let config: Self = toml::from_str(include_str!("../../config/simulation/lidar.toml"))
            .expect("config/simulation/lidar.toml must deserialize into SimulatedLidarConfig");
        config.for_car(&CarCalibration::template("template"))
    }
}

impl SimulatedLidarConfig {
    /// This config mounted where `car`'s lidar is.
    pub fn for_car(self, car: &CarCalibration) -> Self {
        let (x_m, y_m) = car.lidar_mount_m();
        Self {
            mount_x_m: x_m as f32,
            mount_y_m: y_m as f32,
            ..self
        }
    }
}

/// A synthetic LIDAR sensor: claims its vehicle's
/// [`VehicleTopics::lidar_scan`] and publishes a
/// [`LidarScan`] built by raycasting [`SimulatedLidarConfig::num_points`]
/// rays - equally spaced across [`SimulatedLidarConfig::fov_rad`], centered
/// on the vehicle's forward direction - against the map published on
/// [`MAP_TOPIC_NAME`], from its mount (see
/// [`SimulatedLidarConfig::mount_x_m`]) on the pose published on its
/// vehicle's [`VehicleTopics::vehicle_status`], at
/// [`SimulatedLidarConfig::rate_hz`].
pub struct SimulatedLidar {
    id: u16,
    name: String,
    config: SimulatedLidarConfig,
    /// Where the vehicle it's mounted on publishes its pose, and where its
    /// scans go.
    vehicle: VehicleTopics,
}

impl SimulatedLidar {
    pub fn new(name: impl Into<String>, config: SimulatedLidarConfig) -> Self {
        Self {
            id: 0,
            name: name.into(),
            config,
            vehicle: VehicleTopics::ego(),
        }
    }

    /// A lidar mounted on the opponent whose topics are `vehicle`. Its hits
    /// aren't drawn unless the user ticks them.
    pub fn opponent(
        name: impl Into<String>,
        config: SimulatedLidarConfig,
        vehicle: VehicleTopics,
    ) -> Self {
        Self {
            vehicle,
            ..Self::new(name, config)
        }
    }
}

impl Executor for SimulatedLidar {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        let config = self.config;
        captain.claim_writer::<LidarScan>(&self.vehicle.lidar_scan(), self.id, move || {
            LidarScan::new(
                Vec::new(),
                Vec::new(),
                config.min_distance_m,
                config.max_distance_m,
                config.fov_rad,
            )
            .mounted_at(config.mount_x_m, config.mount_y_m)
        });
        captain.claim_drawing(self.id);
    }

    fn run(&mut self, captain: &Captain) {
        let lidar_topic = captain.topic::<LidarScan>(&self.vehicle.lidar_scan());
        let vehicle_topic = captain.topic::<VehicleStatus>(&self.vehicle.vehicle_status());
        let map_topic = captain.topic::<SelectedMap>(MAP_TOPIC_NAME);
        let drawing_topic = captain.drawing(self.id);
        // Three missed scans in a row - but never tighter than the default, so
        // a fast lidar isn't flagged stale by a viewer's own polling jitter.
        let stale_after =
            Drawing::DEFAULT_STALE_AFTER.max(Duration::from_secs_f64(3.0 / self.config.rate_hz));
        let mut ticker = Ticker::new(self.config.rate_hz);
        let seed = match self.config.seed {
            0 => rand::rng().random(),
            seed => seed,
        };
        let mut rng = StdRng::seed_from_u64(seed);

        while captain.is_running(self.id) {
            let status = vehicle_topic.read();
            let map = map_topic.read();
            let others = if self.config.see_vehicles {
                other_vehicles(captain, &self.vehicle)
            } else {
                Vec::new()
            };

            // Every ray starts at the sensor, not the vehicle's reference point.
            let (origin_x, origin_y) = LidarScan::sensor_origin_m(
                self.config.mount_x_m,
                self.config.mount_y_m,
                status.x_m,
                status.y_m,
                status.heading_rad,
            );
            let mut hits = Vec::new();
            let (points, intensities) = (0..self.config.num_points)
                .map(|i| {
                    let angle_rad = status.heading_rad + f64::from(ray_offset_rad(&self.config, i));
                    let (distance_m, hit) = match map.info.as_ref() {
                        Some(info) => cast_ray(
                            &map,
                            info,
                            origin_x,
                            origin_y,
                            angle_rad,
                            self.config.max_distance_m,
                        ),
                        None => (self.config.max_distance_m, false),
                    };
                    let vehicle_m = others
                        .iter()
                        .filter_map(|(other, geometry)| {
                            ray_vehicle_distance_m(other, geometry, origin_x, origin_y, angle_rad)
                        })
                        .fold(f64::INFINITY, f64::min);
                    let (distance_m, hit) = if vehicle_m < f64::from(distance_m) {
                        (vehicle_m as f32, true)
                    } else {
                        (distance_m, hit)
                    };
                    let distance_m = measured_range_m(&self.config, &mut rng, distance_m, hit);
                    if hit {
                        let d = f64::from(distance_m);
                        hits.push([
                            (origin_x + d * angle_rad.cos()) as f32,
                            (origin_y + d * angle_rad.sin()) as f32,
                        ]);
                    }
                    (distance_m, if hit { 1.0 } else { 0.0 })
                })
                .unzip();

            lidar_topic
                .write(
                    self.id,
                    LidarScan::new(
                        points,
                        intensities,
                        self.config.min_distance_m,
                        self.config.max_distance_m,
                        self.config.fov_rad,
                    )
                    .mounted_at(self.config.mount_x_m, self.config.mount_y_m),
                )
                .expect("lost writer authorization for the lidar_scan topic");
            drawing_topic
                .write(
                    self.id,
                    Drawing::default()
                        .element(
                            "Hits",
                            [Shape::Points {
                                points: hits,
                                radius_px: 2.5,
                                color: Color::RED,
                            }],
                            self.vehicle.is_ego(),
                        )
                        .stale_after(stale_after)
                        .z_index(5),
                )
                .expect("lost writer authorization for the lidar's drawing topic");

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
        Box::new(SimulatedLidar {
            id: 0,
            name: self.name.clone(),
            config: self.config,
            vehicle: self.vehicle.clone(),
        })
    }
}

/// The angle of ray `index` (of `config.num_points` total), relative to the
/// vehicle's forward direction - see [`LidarScan::ray_angle_rad`], which
/// every consumer of the scan uses to interpret it.
fn ray_offset_rad(config: &SimulatedLidarConfig, index: usize) -> f32 {
    LidarScan::ray_angle_rad(config.fov_rad, config.num_points, index)
}

/// What the sensor reports for a ray whose true range is `distance_m`: hits
/// get gaussian range noise (misses keep reporting max range untouched), and
/// the result is clamped to the sensor's reported range limits.
fn measured_range_m(
    config: &SimulatedLidarConfig,
    rng: &mut StdRng,
    distance_m: f32,
    hit: bool,
) -> f32 {
    let distance_m = if hit {
        distance_m + gaussian(rng, config.range_std_m) as f32
    } else {
        distance_m
    };
    distance_m.clamp(config.min_distance_m, config.max_distance_m)
}

/// Where every vehicle but the one publishing on `own` is, and its size -
/// the ego vehicle and every opponent listed on [`OPPONENTS_TOPIC_NAME`] -
/// skipping any that hasn't published its first status yet.
fn other_vehicles(captain: &Captain, own: &VehicleTopics) -> Vec<(VehicleStatus, VehicleGeometry)> {
    let opponents = captain
        .try_topic::<Opponents>(OPPONENTS_TOPIC_NAME)
        .map(|topic| topic.read().into_value().list)
        .unwrap_or_default();
    std::iter::once(VehicleTopics::ego())
        .chain(
            opponents
                .iter()
                .map(|opponent| VehicleTopics::opponent(opponent.id)),
        )
        .filter(|vehicle| vehicle.prefix() != own.prefix())
        .filter_map(|vehicle| {
            let status = captain
                .try_topic::<VehicleStatus>(&vehicle.vehicle_status())?
                .read();
            status.age()?;
            let geometry = captain
                .try_topic::<VehicleGeometry>(&vehicle.vehicle_geometry())
                .map_or_else(VehicleGeometry::default, |topic| topic.read().into_value());
            Some((status.into_value(), geometry))
        })
        .collect()
}

/// How far a ray from `(origin_x, origin_y)` (world meters) at `angle_rad`
/// (world frame) travels before entering `vehicle`'s body - a rectangle the
/// size of its `geometry`, centered on it and aligned with its heading.
/// `None` if it misses, or if it starts inside it - vehicles don't collide,
/// so one can end up inside another.
fn ray_vehicle_distance_m(
    vehicle: &VehicleStatus,
    geometry: &VehicleGeometry,
    origin_x: f64,
    origin_y: f64,
    angle_rad: f64,
) -> Option<f64> {
    // The ray, in the body frame (x forward, y left).
    let (sin, cos) = vehicle.heading_rad.sin_cos();
    let (rel_x, rel_y) = (origin_x - vehicle.x_m, origin_y - vehicle.y_m);
    let origin = [rel_x * cos + rel_y * sin, -rel_x * sin + rel_y * cos];
    let (dir_y, dir_x) = (angle_rad - vehicle.heading_rad).sin_cos();
    let half = [geometry.body_length_m / 2.0, geometry.body_width_m / 2.0];

    if origin[0].abs() <= half[0] && origin[1].abs() <= half[1] {
        return None;
    }
    // Slab test: the ray is inside the body where it's between both pairs of
    // opposite sides at once.
    let (mut near, mut far) = (0.0_f64, f64::INFINITY);
    for ((o, d), h) in origin.into_iter().zip([dir_x, dir_y]).zip(half) {
        if d == 0.0 {
            if o.abs() > h {
                return None;
            }
            continue;
        }
        let (a, b) = ((-h - o) / d, (h - o) / d);
        near = near.max(a.min(b));
        far = far.min(a.max(b));
    }
    (near <= far).then_some(near)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ray_offsets_span_the_fov_symmetrically_around_the_forward_direction() {
        let config = SimulatedLidarConfig {
            rate_hz: 10.0,
            num_points: 3,
            min_distance_m: 0.0,
            max_distance_m: 1.0,
            fov_rad: 2.0,
            seed: 1,
            range_std_m: 0.0,
            see_vehicles: false,
            mount_x_m: 0.0,
            mount_y_m: 0.0,
        };

        assert_eq!(ray_offset_rad(&config, 0), -1.0);
        assert_eq!(ray_offset_rad(&config, 1), 0.0);
        assert_eq!(ray_offset_rad(&config, 2), 1.0);
    }

    /// A 0.45 m x 0.25 m body.
    const BODY: VehicleGeometry = VehicleGeometry {
        wheelbase_m: 0.32,
        rear_axle_to_cg_m: 0.16,
        track_width_m: 0.2,
        body_length_m: 0.45,
        body_width_m: 0.25,
    };

    fn vehicle_at(x_m: f64, y_m: f64, heading_rad: f64) -> VehicleStatus {
        VehicleStatus {
            x_m,
            y_m,
            heading_rad,
            ..VehicleStatus::default()
        }
    }

    fn assert_close(actual: Option<f64>, expected: f64) {
        let actual = actual.expect("the ray should hit the vehicle");
        assert!((actual - expected).abs() < 1e-9, "{actual} != {expected}");
    }

    #[test]
    fn a_ray_toward_a_vehicle_stops_on_the_near_side_of_its_body() {
        // Ahead, facing along the ray: the rear bumper is half a length closer.
        let ahead = vehicle_at(3.0, 0.0, 0.0);
        assert_close(
            ray_vehicle_distance_m(&ahead, &BODY, 0.0, 0.0, 0.0),
            3.0 - BODY.body_length_m / 2.0,
        );
        // Turned sideways: its flank is half a width closer.
        let sideways = vehicle_at(3.0, 0.0, std::f64::consts::FRAC_PI_2);
        assert_close(
            ray_vehicle_distance_m(&sideways, &BODY, 0.0, 0.0, 0.0),
            3.0 - BODY.body_width_m / 2.0,
        );
        // Straight above, seen by a ray going up.
        let above = vehicle_at(0.0, 2.0, 0.0);
        assert_close(
            ray_vehicle_distance_m(&above, &BODY, 0.0, 0.0, std::f64::consts::FRAC_PI_2),
            2.0 - BODY.body_width_m / 2.0,
        );
    }

    #[test]
    fn a_ray_passing_beside_or_away_from_a_vehicle_misses_it() {
        let beside = vehicle_at(3.0, BODY.body_width_m, 0.0);
        assert_eq!(ray_vehicle_distance_m(&beside, &BODY, 0.0, 0.0, 0.0), None);
        let behind = vehicle_at(-3.0, 0.0, 0.0);
        assert_eq!(ray_vehicle_distance_m(&behind, &BODY, 0.0, 0.0, 0.0), None);
    }

    #[test]
    fn a_ray_starting_inside_a_vehicle_ignores_it() {
        let overlapping = vehicle_at(0.1, 0.0, 0.3);
        assert_eq!(
            ray_vehicle_distance_m(&overlapping, &BODY, 0.0, 0.0, 0.0),
            None
        );
    }

    #[test]
    fn a_zero_range_std_reports_the_true_hit_distance_exactly() {
        let config = SimulatedLidarConfig {
            range_std_m: 0.0,
            ..SimulatedLidarConfig::default()
        };
        let mut rng = StdRng::seed_from_u64(7);

        assert_eq!(measured_range_m(&config, &mut rng, 3.25, true), 3.25);
    }

    #[test]
    fn hit_ranges_are_noisy_with_the_configured_std_but_misses_are_not() {
        let config = SimulatedLidarConfig {
            range_std_m: 0.05,
            ..SimulatedLidarConfig::default()
        };
        let mut rng = StdRng::seed_from_u64(7);

        let samples: Vec<f64> = (0..10_000)
            .map(|_| f64::from(measured_range_m(&config, &mut rng, 5.0, true)))
            .collect();
        let mean = samples.iter().sum::<f64>() / samples.len() as f64;
        let std =
            (samples.iter().map(|s| (s - mean).powi(2)).sum::<f64>() / samples.len() as f64).sqrt();
        assert!((mean - 5.0).abs() < 0.005, "mean {mean}");
        assert!((std - 0.05).abs() < 0.005, "std {std}");

        let max = config.max_distance_m;
        assert_eq!(measured_range_m(&config, &mut rng, max, false), max);
    }
}
