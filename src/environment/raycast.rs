//! [`cast_ray`]: how far a ray travels across a map before it hits a wall -
//! what a simulated lidar measures, and what a detector expects a real one
//! to.

use super::MapInfo;
use crate::topics::SelectedMap;

/// Marches a ray from `(origin_x, origin_y)` (world meters) at `angle_rad`
/// (world frame) until it either hits an occupied pixel of `map`, leaves the
/// raster, or travels `max_distance_m` - returning the traveled distance and
/// whether it ended in a hit.
pub(crate) fn cast_ray(
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
}
