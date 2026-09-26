//! Building blocks of the LIDAR-reactive algorithms (`ubm_disparity_extender`,
//! `ubm_potential_field`, `ubm_potential_pursuit`): picking the part of a scan to
//! look at, the speed laws, the obstacle-dependent Gaussian potential field,
//! and drawing helpers.
//!
//! Angles are in the sensor frame, like [`LidarScan::angle_rad`]: 0 straight
//! ahead, positive toward increasing heading - the same convention as
//! [`crate::topics::VescCommand::servo_position_rad`], so an angle can be
//! steered to as is.

use crate::Captain;
use crate::autonomous_control::shared::race_line::Pose;
use crate::topics::{Color, LidarScan, Shape, VehicleStatus, VehicleTopics};
use std::ops::Range;

/// The indices of `scan`'s readings within `fov_rad` centred straight
/// ahead - the whole scan if `fov_rad` is wider than the sensor's.
pub(crate) fn fov_window(scan: &LidarScan, fov_rad: f32) -> Range<usize> {
    let n = scan.num_lidar_points;
    if n <= 1 || fov_rad >= scan.fov {
        return 0..n;
    }
    let step = scan.fov / (n - 1) as f32;
    // Readings strictly outside the window on each side.
    let outside = (((scan.fov - fov_rad.max(0.0)) / 2.0 / step).ceil() as usize).min(n / 2);
    outside..n - outside
}

/// The angle between two consecutive readings of `scan`.
pub(crate) fn ray_step_rad(scan: &LidarScan) -> f32 {
    if scan.num_lidar_points <= 1 {
        return scan.fov;
    }
    scan.fov / (scan.num_lidar_points - 1) as f32
}

/// A speed falling linearly from `max_speed` when driving straight to
/// `min_speed` at `max_steering` or beyond.
pub(crate) fn speed_proportional_steering(
    steering_rad: f32,
    max_steering_rad: f32,
    max_speed: f32,
    min_speed: f32,
) -> f32 {
    let min_speed = min_speed.min(max_speed);
    if max_steering_rad <= 0.0 {
        return min_speed;
    }
    let lock = (steering_rad.abs() / max_steering_rad).min(1.0);
    max_speed - (max_speed - min_speed) * lock
}

/// [`speed_proportional_steering`], plus `speed_distance_gain` times the
/// mean distance `front` holds, minus `brake_gain` over it: faster with room
/// ahead, slower close to a wall. Never negative - the vehicle's own limits
/// clamp the top.
pub(crate) fn speed_steer_and_fov(
    proportional_speed: f32,
    front: &[f32],
    speed_distance_gain: f32,
    brake_gain: f32,
) -> f32 {
    let mean = mean(front);
    if mean <= 0.0 {
        return 0.0;
    }
    (proportional_speed + speed_distance_gain * mean - brake_gain / mean).max(0.0)
}

pub(crate) fn mean(values: &[f32]) -> f32 {
    if values.is_empty() {
        return 0.0;
    }
    values.iter().sum::<f32>() / values.len() as f32
}

/// How to build a [`Field`] - see [`potential_field`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct FieldConfig {
    /// Part of the scan looked at, centred straight ahead.
    pub fov_rad: f32,
    /// Angle between two cells of the field.
    pub resolution_rad: f32,
    /// Readings nearer than this times the mean reading are obstacles.
    pub obstacle_threshold_gain: f32,
    /// Hysteresis around that threshold, in meters.
    pub hysteresis_m: f32,
    /// Weight of the attractive term, pure number.
    pub attractive_power: f32,
    /// Vehicle width, which widens every obstacle's potential.
    pub car_width_m: f32,
    /// Also consider the global minimum, not only strict local ones.
    pub include_global_minima: bool,
    /// Pick the minimum nearest to the attractive direction instead of the
    /// lowest one.
    pub use_minima_near_attractive: bool,
}

/// A run of readings nearer than the obstacle threshold.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Obstacle {
    /// First and last reading (scan indices) of the obstacle.
    pub start: usize,
    pub end: usize,
    /// Direction of its centre.
    pub angle_rad: f32,
    /// Mean distance of its readings.
    pub distance_m: f32,
    /// Spread of its Gaussian potential.
    pub sigma_rad: f32,
    /// Peak of its Gaussian potential.
    pub amplitude: f32,
}

/// The obstacle-dependent Gaussian potential field over a scan, and the
/// direction chosen on it.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Field {
    /// Direction of cell 0; cell `i` points `i * resolution_rad` further.
    pub start_rad: f32,
    pub resolution_rad: f32,
    /// Repulsive (normalized to at most 1) plus attractive potential, per cell.
    pub potential: Vec<f32>,
    pub obstacles: Vec<Obstacle>,
    /// Cell the attractive term pulls toward.
    pub attractive_cell: usize,
    /// Cell chosen to drive toward.
    pub chosen_cell: usize,
}

impl Field {
    pub(crate) fn angle_rad(&self, cell: usize) -> f32 {
        self.start_rad + cell as f32 * self.resolution_rad
    }

    /// The cell nearest to `angle_rad`, clamped into the field.
    pub(crate) fn cell(&self, angle_rad: f32) -> usize {
        let cell = ((angle_rad - self.start_rad) / self.resolution_rad).round();
        (cell.max(0.0) as usize).min(self.potential.len() - 1)
    }

    pub(crate) fn chosen_rad(&self) -> f32 {
        self.angle_rad(self.chosen_cell)
    }
}

/// The obstacles among `scan`'s readings in `window`: runs nearer than
/// `threshold_gain` times their mean, entered below the threshold minus
/// `hysteresis_m` and left above it plus `hysteresis_m`. A single-reading
/// run is dropped as noise.
pub(crate) fn find_obstacles(
    scan: &LidarScan,
    window: Range<usize>,
    threshold_gain: f32,
    hysteresis_m: f32,
    car_width_m: f32,
) -> Vec<Obstacle> {
    let ranges = &scan.points[window.clone()];
    let threshold = threshold_gain * mean(ranges);
    let farthest = ranges.iter().copied().fold(0.0, f32::max);
    let step = ray_step_rad(scan);

    let make = |start: usize, end: usize| {
        let phi = (end - start) as f32 * step;
        let distance_m = mean(&scan.points[start..=end]);
        let sigma_rad = if phi > std::f32::consts::PI {
            std::f32::consts::PI
        } else {
            (distance_m * (phi / 2.0).tan() + car_width_m / 2.0).atan2(distance_m)
        };
        Obstacle {
            start,
            end,
            angle_rad: (scan.angle_rad(start) + scan.angle_rad(end)) / 2.0,
            distance_m,
            sigma_rad,
            amplitude: (farthest - distance_m) * 0.5f32.exp(),
        }
    };

    let mut obstacles = Vec::new();
    let mut start = None;
    for i in window.clone() {
        let range = scan.points[i];
        match start {
            None if range <= threshold - hysteresis_m => start = Some(i),
            Some(first) if range > threshold + hysteresis_m => {
                if i - 1 > first {
                    obstacles.push(make(first, i - 1));
                }
                start = None;
            }
            _ => {}
        }
    }
    if let Some(first) = start
        && window.end - 1 > first
    {
        obstacles.push(make(first, window.end - 1));
    }
    obstacles
}

/// The potential field over `scan`: every obstacle's Gaussian repulsion,
/// normalized, plus an attraction growing linearly away from the direction
/// `attractive_rad` returns given the direction of the longest reading. The
/// direction chosen is a minimum of it - see [`FieldConfig`]. `None` if the
/// scan has too few readings.
///
/// Based on "A Real-Time Obstacle Avoidance Method for Autonomous Vehicles
/// Using an Obstacle-Dependent Gaussian Potential Field",
/// <https://doi.org/10.1155/2018/5041401>.
pub(crate) fn potential_field(
    scan: &LidarScan,
    config: &FieldConfig,
    attractive_rad: impl FnOnce(f32) -> f32,
) -> Option<Field> {
    let window = fov_window(scan, config.fov_rad);
    if window.len() < 3 || config.resolution_rad <= 0.0 {
        return None;
    }
    let start_rad = scan.angle_rad(window.start);
    let span_rad = scan.angle_rad(window.end - 1) - start_rad;
    let cells = (span_rad / config.resolution_rad).floor() as usize + 1;
    if cells < 3 {
        return None;
    }

    let obstacles = find_obstacles(
        scan,
        window.clone(),
        config.obstacle_threshold_gain,
        config.hysteresis_m,
        config.car_width_m,
    );
    let mut field = Field {
        start_rad,
        resolution_rad: config.resolution_rad,
        potential: vec![0.0; cells],
        obstacles,
        attractive_cell: 0,
        chosen_cell: 0,
    };

    for cell in 0..cells {
        let angle = field.angle_rad(cell);
        field.potential[cell] = field
            .obstacles
            .iter()
            .map(|o| {
                o.amplitude * (-(o.angle_rad - angle).powi(2) / (2.0 * o.sigma_rad.powi(2))).exp()
            })
            .sum();
    }
    let peak = field.potential.iter().copied().fold(0.0, f32::max);
    if peak > 0.0 {
        field.potential.iter_mut().for_each(|p| *p /= peak);
    }

    let longest = window
        .clone()
        .max_by(|&a, &b| scan.points[a].total_cmp(&scan.points[b]))
        .expect("the window isn't empty");
    field.attractive_cell = field.cell(attractive_rad(scan.angle_rad(longest)));
    for (cell, potential) in field.potential.iter_mut().enumerate() {
        *potential +=
            config.attractive_power * cell.abs_diff(field.attractive_cell) as f32 / cells as f32;
    }

    let potential = &field.potential;
    let mut minima: Vec<usize> = (1..cells - 1)
        .filter(|&i| potential[i] < potential[i - 1] && potential[i] < potential[i + 1])
        .collect();
    if config.include_global_minima {
        minima.extend((0..cells).min_by(|&a, &b| potential[a].total_cmp(&potential[b])));
    }
    let attractive = field.attractive_cell;
    let chosen = if config.use_minima_near_attractive {
        minima.into_iter().min_by_key(|&i| i.abs_diff(attractive))
    } else {
        minima
            .into_iter()
            .min_by(|&a, &b| potential[a].total_cmp(&potential[b]))
    };
    field.chosen_cell = chosen.unwrap_or(attractive);
    Some(field)
}

/// Where the scan of the vehicle whose topics are `vehicle` was taken from,
/// for drawing: the simulator's ground truth (which the simulated LIDAR casts
/// from), if there is one.
pub(crate) fn scan_origin(captain: &Captain, vehicle: &VehicleTopics) -> Option<Pose> {
    let status = captain
        .try_topic::<VehicleStatus>(&vehicle.vehicle_status())?
        .read();
    status.meta.written_at?;
    Some(Pose {
        x_m: status.x_m,
        y_m: status.y_m,
        heading_rad: status.heading_rad,
    })
}

/// The world point `distance_m` from `origin` along `angle_rad` (sensor frame).
pub(crate) fn world_point(origin: Pose, angle_rad: f32, distance_m: f32) -> [f32; 2] {
    let angle = origin.heading_rad + f64::from(angle_rad);
    let distance = f64::from(distance_m);
    [
        (origin.x_m + distance * angle.cos()) as f32,
        (origin.y_m + distance * angle.sin()) as f32,
    ]
}

/// A line from `origin` along `angle_rad`, `length_m` long.
pub(crate) fn ray(origin: Pose, angle_rad: f32, length_m: f32, color: Color) -> Shape {
    Shape::Polyline {
        points: vec![
            [origin.x_m as f32, origin.y_m as f32],
            world_point(origin, angle_rad, length_m),
        ],
        closed: false,
        width_px: 2.0,
        color,
    }
}

/// `field`'s obstacles (red sectors), its potential as a polar curve
/// (farther out where the potential is higher, amber), and the chosen
/// direction (purple), as `(obstacles, potential, chosen)` shapes.
pub(crate) fn field_shapes(origin: Pose, field: &Field) -> (Vec<Shape>, Vec<Shape>, Vec<Shape>) {
    let obstacles = field
        .obstacles
        .iter()
        .map(|o| Shape::CircularSector {
            x_m: origin.x_m,
            y_m: origin.y_m,
            radius_m: f64::from(o.distance_m),
            start_rad: origin.heading_rad + f64::from(o.angle_rad - o.sigma_rad),
            end_rad: origin.heading_rad + f64::from(o.angle_rad + o.sigma_rad),
            filled: false,
            color: Color::RED,
        })
        .collect();
    let curve = Shape::Polyline {
        points: field
            .potential
            .iter()
            .enumerate()
            .map(|(cell, p)| world_point(origin, field.angle_rad(cell), 0.5 + 1.5 * p))
            .collect(),
        closed: false,
        width_px: 1.5,
        color: Color::AMBER,
    };
    let chosen = ray(origin, field.chosen_rad(), 2.5, Color::PURPLE);
    (obstacles, vec![curve], vec![chosen])
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use std::f32::consts::PI;

    /// A scan of `n` readings across `fov`, all `distance`.
    pub(crate) fn scan(n: usize, fov: f32, distance: f32) -> LidarScan {
        LidarScan::new(vec![distance; n], vec![1.0; n], 0.1, 12.0, fov)
    }

    pub(crate) fn field_config() -> FieldConfig {
        FieldConfig {
            fov_rad: PI,
            resolution_rad: 0.5f32.to_radians(),
            obstacle_threshold_gain: 1.0,
            hysteresis_m: 0.2,
            attractive_power: 0.01,
            car_width_m: 0.34,
            include_global_minima: false,
            use_minima_near_attractive: false,
        }
    }

    #[test]
    fn the_window_is_centred_and_clamped() {
        let scan = scan(241, 4.0, 5.0);
        let window = fov_window(&scan, 2.0);
        assert!((scan.angle_rad(window.start) + 1.0).abs() < 0.02);
        assert!((scan.angle_rad(window.end - 1) - 1.0).abs() < 0.02);
        assert_eq!(window.start, scan.num_lidar_points - window.end);
        assert_eq!(fov_window(&scan, 10.0), 0..241);
    }

    #[test]
    fn speed_is_max_straight_and_min_at_full_lock() {
        assert_eq!(speed_proportional_steering(0.0, 0.4, 3.0, 1.0), 3.0);
        assert_eq!(speed_proportional_steering(-0.4, 0.4, 3.0, 1.0), 1.0);
        assert_eq!(speed_proportional_steering(0.9, 0.4, 3.0, 1.0), 1.0);
        assert!((speed_proportional_steering(0.2, 0.4, 3.0, 1.0) - 2.0).abs() < 1e-6);
    }

    #[test]
    fn distance_gains_brake_near_a_wall() {
        assert!(
            speed_steer_and_fov(2.0, &[0.5; 4], 0.0, 2.0)
                < speed_steer_and_fov(2.0, &[4.0; 4], 0.0, 2.0)
        );
        assert_eq!(speed_steer_and_fov(2.0, &[0.1; 4], 0.0, 2.0), 0.0);
    }

    #[test]
    fn hysteresis_drops_single_reading_blips() {
        let mut scan = scan(181, PI, 5.0);
        scan.points[90] = 1.0;
        let window = fov_window(&scan, PI);
        // Below 1.0 times the mean, so the background clears the hysteresis.
        assert!(find_obstacles(&scan, window.clone(), 0.8, 0.2, 0.3).is_empty());
        scan.points[91] = 1.0;
        let obstacles = find_obstacles(&scan, window, 0.8, 0.2, 0.3);
        assert_eq!(obstacles.len(), 1);
        assert_eq!((obstacles[0].start, obstacles[0].end), (90, 91));
    }

    #[test]
    fn an_open_scan_drives_toward_the_longest_reading() {
        let mut scan = scan(181, PI, 5.0);
        scan.points[90] = 8.0;
        let field = potential_field(&scan, &field_config(), |longest| longest).unwrap();
        assert!(field.chosen_rad().abs() < 0.02, "{}", field.chosen_rad());
    }

    #[test]
    fn an_obstacle_on_the_positive_side_pushes_toward_the_negative_one() {
        let mut scan = scan(181, PI, 5.0);
        // An obstacle a little toward positive angles, straight ahead-ish.
        for i in 92..110 {
            scan.points[i] = 1.0;
        }
        let field = potential_field(&scan, &field_config(), |_| 0.0).unwrap();
        assert!(field.chosen_rad() < 0.0, "{}", field.chosen_rad());
    }
}
