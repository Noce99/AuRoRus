//! [`SimulatedLidar`]: a synthetic LIDAR sensor that raycasts against the
//! currently loaded map from the vehicle's real position, for exercising
//! algorithms against physically grounded readings without real hardware.
//! Also draws every hit on its own drawing topic (see
//! [`crate::topics::Drawing`]).

use crate::environment::MapInfo;
use crate::topics::{
    Color, Drawing, LIDAR_SCAN_TOPIC_NAME, LidarScan, MAP_TOPIC_NAME, SelectedMap, Shape,
    VEHICLE_STATUS_TOPIC_NAME, VehicleStatus,
};
use crate::{Captain, Executor, Ticker};
use std::any::Any;
use std::time::Duration;

/// Every tunable parameter [`SimulatedLidar`] needs - loaded from
/// `config/sensors/simulated_lidar.toml` (see [`Default`]) or from an
/// arbitrary path via [`crate::config::load`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Deserialize)]
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
}

impl Default for SimulatedLidarConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/sensors/simulated_lidar.toml")).expect(
            "config/sensors/simulated_lidar.toml must deserialize into SimulatedLidarConfig",
        )
    }
}

/// A synthetic LIDAR sensor: claims [`LIDAR_SCAN_TOPIC_NAME`] and publishes a
/// [`LidarScan`] built by raycasting [`SimulatedLidarConfig::num_points`]
/// rays - equally spaced across [`SimulatedLidarConfig::fov_rad`], centered
/// on the vehicle's forward direction - against the map published on
/// [`MAP_TOPIC_NAME`], from the position published on
/// [`VEHICLE_STATUS_TOPIC_NAME`], at [`SimulatedLidarConfig::rate_hz`].
pub struct SimulatedLidar {
    id: u8,
    name: String,
    config: SimulatedLidarConfig,
}

impl SimulatedLidar {
    pub fn new(name: impl Into<String>, config: SimulatedLidarConfig) -> Self {
        Self {
            id: 0,
            name: name.into(),
            config,
        }
    }
}

impl Executor for SimulatedLidar {
    fn init(&mut self, id: u8) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        let config = self.config;
        captain.claim_writer::<LidarScan>(LIDAR_SCAN_TOPIC_NAME, self.id, move || {
            LidarScan::new(
                Vec::new(),
                Vec::new(),
                config.min_distance_m,
                config.max_distance_m,
                config.fov_rad,
            )
        });
        captain.claim_drawing(self.id);
    }

    fn run(&mut self, captain: &Captain) {
        let lidar_topic = captain.topic::<LidarScan>(LIDAR_SCAN_TOPIC_NAME);
        let vehicle_topic = captain.topic::<VehicleStatus>(VEHICLE_STATUS_TOPIC_NAME);
        let map_topic = captain.topic::<SelectedMap>(MAP_TOPIC_NAME);
        let drawing_topic = captain.drawing(self.id);
        // Three missed scans in a row - but never tighter than the default, so
        // a fast lidar isn't flagged stale by a viewer's own polling jitter.
        let stale_after =
            Drawing::DEFAULT_STALE_AFTER.max(Duration::from_secs_f64(3.0 / self.config.rate_hz));
        let mut ticker = Ticker::new(self.config.rate_hz);

        while captain.is_running(self.id) {
            let status = vehicle_topic.read();
            let map = map_topic.read();

            let mut hits = Vec::new();
            let (points, intensities) = (0..self.config.num_points)
                .map(|i| {
                    let angle_rad = status.heading_rad + f64::from(ray_offset_rad(&self.config, i));
                    let (distance_m, hit) = match map.info.as_ref() {
                        Some(info) => cast_ray(
                            &map,
                            info,
                            status.x_m,
                            status.y_m,
                            angle_rad,
                            self.config.max_distance_m,
                        ),
                        None => (self.config.max_distance_m, false),
                    };
                    let distance_m =
                        distance_m.clamp(self.config.min_distance_m, self.config.max_distance_m);
                    if hit {
                        let d = f64::from(distance_m);
                        hits.push([
                            (status.x_m + d * angle_rad.cos()) as f32,
                            (status.y_m + d * angle_rad.sin()) as f32,
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
                    ),
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
        Box::new(SimulatedLidar::new(self.name.clone(), self.config))
    }
}

/// The angle of ray `index` (of `config.num_points` total), relative to the
/// vehicle's forward direction - see [`LidarScan::ray_angle_rad`], which
/// every consumer of the scan uses to interpret it.
fn ray_offset_rad(config: &SimulatedLidarConfig, index: usize) -> f32 {
    LidarScan::ray_angle_rad(config.fov_rad, config.num_points, index)
}

/// Marches a ray from `(origin_x, origin_y)` (world meters) at `angle_rad`
/// (world frame) until it either hits an occupied pixel of `map`, leaves the
/// raster, or travels `max_distance_m` - returning the traveled distance and
/// whether it ended in a hit.
fn cast_ray(
    map: &SelectedMap,
    info: &MapInfo,
    origin_x: f64,
    origin_y: f64,
    angle_rad: f64,
    max_distance_m: f32,
) -> (f32, bool) {
    let step_m = info.resolution_m_per_px;
    let dx = angle_rad.cos();
    let dy = angle_rad.sin();

    let mut traveled_m = 0.0;
    while traveled_m < f64::from(max_distance_m) {
        let x = origin_x + dx * traveled_m;
        let y = origin_y + dy * traveled_m;
        let col = ((x - info.origin.x) / info.resolution_m_per_px).floor();
        let row = ((y - info.origin.y) / info.resolution_m_per_px).floor();

        if col < 0.0
            || row < 0.0
            || col >= f64::from(info.width_px)
            || row >= f64::from(info.height_px)
        {
            return (max_distance_m, false);
        }

        let index = row as usize * info.width_px as usize + col as usize;
        if map.pixels[index] != 255 {
            return (traveled_m as f32, true);
        }

        traveled_m += step_m;
    }

    (max_distance_m, false)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::environment::{ImageOrigin, MapSource, StartFinishLine, WorldPoint};

    fn info(width_px: u32, height_px: u32, resolution_m_per_px: f64) -> MapInfo {
        MapInfo {
            resolution_m_per_px,
            width_px,
            height_px,
            origin: ImageOrigin {
                x: 0.0,
                y: 0.0,
                theta_rad: 0.0,
            },
            start_finish_line: StartFinishLine {
                a: WorldPoint { x: 0.0, y: 0.0 },
                b: WorldPoint { x: 0.0, y: 0.0 },
            },
            generated_at: String::new(),
            source: MapSource::Random,
            generation: None,
        }
    }

    fn map(width_px: u32, height_px: u32, pixels: Vec<u8>, info: MapInfo) -> SelectedMap {
        SelectedMap {
            path: None,
            width_px,
            height_px,
            pixels: pixels.into(),
            info: Some(info),
        }
    }

    #[test]
    fn a_ray_over_an_all_white_raster_reports_max_distance_with_no_hit() {
        let info = info(10, 10, 0.1);
        let map = map(10, 10, vec![255u8; 100], info.clone());

        let (distance_m, hit) = cast_ray(&map, &info, 0.5, 0.5, 0.0, 5.0);

        assert_eq!(distance_m, 5.0);
        assert!(!hit);
    }

    #[test]
    fn a_ray_toward_an_obstacle_pixel_reports_the_hit_distance() {
        let info = info(10, 10, 0.1);
        let mut pixels = vec![255u8; 100];
        // Obstacle column at x in [0.8, 0.9) m, directly ahead of the ray.
        for row in 0..10 {
            pixels[row * 10 + 8] = 0;
        }
        let map = map(10, 10, pixels, info.clone());

        let (distance_m, hit) = cast_ray(&map, &info, 0.05, 0.5, 0.0, 5.0);

        assert!(hit);
        assert!(
            (distance_m - 0.75).abs() < 0.15,
            "expected a hit near 0.75m, got {distance_m}"
        );
    }

    #[test]
    fn a_ray_that_leaves_the_map_reports_max_distance_with_no_hit() {
        let info = info(10, 10, 0.1);
        let map = map(10, 10, vec![255u8; 100], info.clone());

        let (distance_m, hit) = cast_ray(&map, &info, 0.05, 0.05, std::f64::consts::PI, 5.0);

        assert_eq!(distance_m, 5.0);
        assert!(!hit);
    }

    #[test]
    fn ray_offsets_span_the_fov_symmetrically_around_the_forward_direction() {
        let config = SimulatedLidarConfig {
            rate_hz: 10.0,
            num_points: 3,
            min_distance_m: 0.0,
            max_distance_m: 1.0,
            fov_rad: 2.0,
        };

        assert_eq!(ray_offset_rad(&config, 0), -1.0);
        assert_eq!(ray_offset_rad(&config, 1), 0.0);
        assert_eq!(ray_offset_rad(&config, 2), 1.0);
    }
}
