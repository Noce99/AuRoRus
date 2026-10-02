//! Random Voronoi seed-point sampling: scatters points inside a bounded
//! rectangle, keeping a minimum distance between any two of them so the
//! resulting Voronoi cells stay reasonably sized.

use rand::RngExt;
use voronoice::Point;

/// Maximum number of placement attempts for a single point before giving up
/// (see [`sample`]).
const MAX_ATTEMPTS_PER_POINT: usize = 200;

/// Error returned by [`sample`] when `min_spacing_m` and `count` can't both
/// be satisfied inside the requested area.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PointSamplingError {
    /// How many points were requested.
    pub requested: usize,
    /// How many points were actually placed before giving up.
    pub placed: usize,
}

impl std::fmt::Display for PointSamplingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "could only place {} of {} requested points before the minimum spacing made \
             further placement impractical - reduce num_sites, reduce min_site_spacing_m, or \
             enlarge the area",
            self.placed, self.requested,
        )
    }
}

impl std::error::Error for PointSamplingError {}

/// Rejection-samples `count` points uniformly inside a `width_m x height_m`
/// rectangle centered on the origin, each at least `min_spacing_m` away from
/// every previously placed point.
pub fn sample(
    rng: &mut impl RngExt,
    count: usize,
    width_m: f64,
    height_m: f64,
    min_spacing_m: f64,
) -> Result<Vec<Point>, PointSamplingError> {
    let mut points: Vec<Point> = Vec::with_capacity(count);
    let half_width = width_m / 2.0;
    let half_height = height_m / 2.0;
    let min_spacing_sq = min_spacing_m * min_spacing_m;

    while points.len() < count {
        let mut placed = false;
        for _ in 0..MAX_ATTEMPTS_PER_POINT {
            let candidate = Point {
                x: rng.random_range(-half_width..=half_width),
                y: rng.random_range(-half_height..=half_height),
            };
            let far_enough = points.iter().all(|p: &Point| {
                let dx = p.x - candidate.x;
                let dy = p.y - candidate.y;
                dx * dx + dy * dy >= min_spacing_sq
            });
            if far_enough {
                points.push(candidate);
                placed = true;
                break;
            }
        }
        if !placed {
            return Err(PointSamplingError {
                requested: count,
                placed: points.len(),
            });
        }
    }

    Ok(points)
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::SeedableRng;
    use rand::rngs::StdRng;

    #[test]
    fn sample_returns_exact_count_within_bounds_and_spacing() {
        let mut rng = StdRng::seed_from_u64(1);
        let points = sample(&mut rng, 20, 30.0, 30.0, 2.0).unwrap();
        assert_eq!(points.len(), 20);
        for p in &points {
            assert!((-15.0..=15.0).contains(&p.x));
            assert!((-15.0..=15.0).contains(&p.y));
        }
        for i in 0..points.len() {
            for j in (i + 1)..points.len() {
                let dx = points[i].x - points[j].x;
                let dy = points[i].y - points[j].y;
                assert!((dx * dx + dy * dy).sqrt() >= 2.0 - 1e-9);
            }
        }
    }

    #[test]
    fn overly_dense_request_fails() {
        let mut rng = StdRng::seed_from_u64(1);
        let result = sample(&mut rng, 1000, 5.0, 5.0, 2.0);
        assert!(result.is_err());
    }
}
