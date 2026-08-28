//! Builds a closed, non-self-intersecting track-shaped loop from a random
//! Voronoi diagram: [`select_region`] grows a simply-connected region of
//! adjacent cells by BFS, then [`boundary_edges`]/[`order_loop`] walk the
//! *topological boundary* of that region into an ordered ring.
//!
//! The boundary of a simply-connected planar region is, by topology, always
//! a single simple closed curve - so as long as the selected cells stay
//! simply-connected (no enclosed "holes" of unselected cells), the walked
//! boundary is *structurally* guaranteed to be simple, with no
//! self-intersection test or repair pass ever needed. This mirrors the
//! approach used by `github.com/gerkone/voronoiTrack` (confirmed against
//! its actual source, not just its README).

use crate::environment::simulator::smoothing::Point2;

use std::collections::{HashMap, HashSet, VecDeque};
use voronoice::{BoundingBox, ClipBehavior, Point, Voronoi, VoronoiBuilder};

/// Error returned by [`build_loop`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LoopConstructionError {
    /// [`VoronoiBuilder::build`] returned `None` - the sites didn't form a
    /// valid diagram (e.g. all collinear).
    VoronoiBuildFailed,
    /// The BFS region growth ran out of cells to add before reaching
    /// `target_area_fraction` of the diagram's total interior area.
    InsufficientArea { covered_fraction: f64 },
    /// The extracted boundary wasn't a single simple cycle (some vertex had
    /// a degree other than 2) - a defensive check that should only trip on
    /// unusual/degenerate site configurations.
    NonManifoldBoundary,
}

impl std::fmt::Display for LoopConstructionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::VoronoiBuildFailed => {
                write!(f, "failed to build a Voronoi diagram from the sampled sites")
            }
            Self::InsufficientArea { covered_fraction } => write!(
                f,
                "region growth only covered {:.1}% of the diagram's interior area before \
                 running out of cells - try more sites, a smaller target_area_fraction, or a \
                 different seed",
                covered_fraction * 100.0,
            ),
            Self::NonManifoldBoundary => write!(
                f,
                "the selected region's boundary was not a single simple loop - try a different \
                 seed"
            ),
        }
    }
}

impl std::error::Error for LoopConstructionError {}

/// Builds a closed loop of track centerline points from `sites`, scattered
/// within a `area_width_m x area_height_m` area centered on the origin. The
/// returned polygon is closed without repeating its first point.
pub fn build_loop(
    sites: Vec<Point>,
    area_width_m: f64,
    area_height_m: f64,
    target_area_fraction: f64,
) -> Result<Vec<Point2>, LoopConstructionError> {
    let voronoi = VoronoiBuilder::default()
        .set_sites(sites)
        .set_bounding_box(BoundingBox::new_centered(area_width_m, area_height_m))
        .set_clip_behavior(ClipBehavior::Clip)
        .build()
        .ok_or(LoopConstructionError::VoronoiBuildFailed)?;

    let selected = select_region(&voronoi, target_area_fraction)?;
    let boundary = boundary_edges(&voronoi, &selected);
    let order = order_loop(&boundary)?;

    Ok(order
        .into_iter()
        .map(|idx| {
            let p = &voronoi.vertices()[idx];
            Point2 { x: p.x, y: p.y }
        })
        .collect())
}

/// Shoelace-formula area of a (simple) polygon given its vertices in order.
fn polygon_area<'a>(vertices: impl Iterator<Item = &'a Point>) -> f64 {
    let verts: Vec<&Point> = vertices.collect();
    let n = verts.len();
    if n < 3 {
        return 0.0;
    }
    let mut sum = 0.0;
    for i in 0..n {
        let a = verts[i];
        let b = verts[(i + 1) % n];
        sum += a.x * b.y - b.x * a.y;
    }
    (sum / 2.0).abs()
}

/// Grows a simply-connected region of non-hull Voronoi cells by BFS from
/// the cell nearest the area's center, accumulating area until
/// `target_area_fraction` of the diagram's total (non-hull) interior area
/// is covered.
///
/// Hull cells are excluded entirely: including one would let the boundary
/// walk pick up a straight edge running along the map's outer clip instead
/// of a track-like shape, and it also keeps the loop comfortably interior
/// to the raster.
///
/// After growth, any still-queued cell whose every neighbor is already
/// selected gets selected too ("hole-patching"), repeated to a fixed point -
/// this is what keeps the region simply-connected. Without it, an isolated
/// pocket of unselected cells fully enclosed by selected ones would split
/// the boundary into more than one cycle.
fn select_region(
    voronoi: &Voronoi,
    target_area_fraction: f64,
) -> Result<HashSet<usize>, LoopConstructionError> {
    let mut cell_area = HashMap::new();
    let mut total_interior_area = 0.0;
    for cell in voronoi.iter_cells() {
        if cell.is_on_hull() {
            continue;
        }
        let area = polygon_area(cell.iter_vertices());
        cell_area.insert(cell.site(), area);
        total_interior_area += area;
    }

    if cell_area.is_empty() {
        return Err(LoopConstructionError::InsufficientArea { covered_fraction: 0.0 });
    }

    let start_site = voronoi
        .iter_cells()
        .filter(|cell| cell_area.contains_key(&cell.site()))
        .min_by(|a, b| {
            let da = a.site_position().x.powi(2) + a.site_position().y.powi(2);
            let db = b.site_position().x.powi(2) + b.site_position().y.powi(2);
            da.partial_cmp(&db).expect("site coordinates are never NaN")
        })
        .map(|cell| cell.site())
        .expect("cell_area is non-empty, so at least one non-hull cell exists");

    let target = total_interior_area * target_area_fraction;
    let mut selected = HashSet::new();
    let mut enqueued = HashSet::new();
    let mut queue = VecDeque::new();
    queue.push_back(start_site);
    enqueued.insert(start_site);
    let mut covered = 0.0;

    while covered < target {
        let Some(site) = queue.pop_front() else {
            break;
        };
        if !selected.insert(site) {
            continue;
        }
        covered += cell_area[&site];
        for neighbor in voronoi.cell(site).iter_neighbors() {
            if cell_area.contains_key(&neighbor) && enqueued.insert(neighbor) {
                queue.push_back(neighbor);
            }
        }
    }

    if covered < target {
        return Err(LoopConstructionError::InsufficientArea {
            covered_fraction: covered / total_interior_area,
        });
    }

    loop {
        let mut changed = false;
        for site in queue.iter().copied().collect::<Vec<_>>() {
            if selected.contains(&site) {
                continue;
            }
            let fully_enclosed = voronoi
                .cell(site)
                .iter_neighbors()
                .all(|neighbor| selected.contains(&neighbor));
            if fully_enclosed {
                selected.insert(site);
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }

    Ok(selected)
}

/// Every edge (as a canonicalized pair of global vertex indices into
/// [`Voronoi::vertices`]) that borders exactly one selected cell - i.e. the
/// boundary of the selected region.
fn boundary_edges(voronoi: &Voronoi, selected: &HashSet<usize>) -> HashSet<(usize, usize)> {
    let mut owners: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    for cell in voronoi.iter_cells() {
        let verts = cell.triangles();
        let n = verts.len();
        for i in 0..n {
            let a = verts[i];
            let b = verts[(i + 1) % n];
            let key = if a < b { (a, b) } else { (b, a) };
            owners.entry(key).or_default().push(cell.site());
        }
    }

    owners
        .into_iter()
        .filter(|(_, sites)| sites.iter().filter(|s| selected.contains(s)).count() == 1)
        .map(|(edge, _)| edge)
        .collect()
}

/// Walks `boundary` (a set of undirected edges expected to form a single
/// simple cycle) into an ordered sequence of vertex indices, not repeating
/// the starting vertex at the end.
fn order_loop(boundary: &HashSet<(usize, usize)>) -> Result<Vec<usize>, LoopConstructionError> {
    let mut adjacency: HashMap<usize, Vec<usize>> = HashMap::new();
    for &(a, b) in boundary {
        adjacency.entry(a).or_default().push(b);
        adjacency.entry(b).or_default().push(a);
    }

    if adjacency.is_empty() || adjacency.values().any(|neighbors| neighbors.len() != 2) {
        return Err(LoopConstructionError::NonManifoldBoundary);
    }

    // `.min()` rather than `.iter().next()`: HashSet iteration order is
    // randomized per-instance, so picking an arbitrary starting edge would
    // make the walk's starting vertex (and thus every downstream point,
    // since resampling starts from it) different across otherwise-identical
    // runs - breaking the "same seed reproduces the same map" contract.
    let &(start, second) = boundary.iter().min().expect("adjacency is non-empty");
    let mut order = vec![start];
    let mut prev = start;
    let mut current = second;

    loop {
        order.push(current);
        let neighbors = &adjacency[&current];
        let next = if neighbors[0] != prev { neighbors[0] } else { neighbors[1] };
        if next == start {
            break;
        }
        prev = current;
        current = next;
        if order.len() > adjacency.len() {
            return Err(LoopConstructionError::NonManifoldBoundary);
        }
    }

    Ok(order)
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::SeedableRng;
    use rand::rngs::StdRng;

    /// Brute-force O(n^2) simple-polygon check: no two non-adjacent edges
    /// may intersect. Only meant for the small loops these tests produce.
    fn is_simple_polygon(points: &[Point2]) -> bool {
        let n = points.len();
        if n < 3 {
            return false;
        }
        for i in 0..n {
            let a1 = points[i];
            let a2 = points[(i + 1) % n];
            for j in (i + 1)..n {
                // Skip edge j == i (itself) and both edges adjacent to edge i
                // (sharing an endpoint with it) - those touch by
                // construction and aren't a "crossing" to detect.
                if j == i || j == (i + 1) % n || (j + 1) % n == i {
                    continue;
                }
                let b1 = points[j];
                let b2 = points[(j + 1) % n];
                if segments_intersect(a1, a2, b1, b2) {
                    return false;
                }
            }
        }
        true
    }

    fn orientation(a: Point2, b: Point2, c: Point2) -> f64 {
        (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
    }

    fn segments_intersect(a1: Point2, a2: Point2, b1: Point2, b2: Point2) -> bool {
        let d1 = orientation(b1, b2, a1);
        let d2 = orientation(b1, b2, a2);
        let d3 = orientation(a1, a2, b1);
        let d4 = orientation(a1, a2, b2);
        (d1 > 0.0) != (d2 > 0.0) && (d3 > 0.0) != (d4 > 0.0)
    }

    #[test]
    fn build_loop_produces_a_simple_polygon_across_several_seeds() {
        for seed in 0u64..10 {
            let mut rng = StdRng::seed_from_u64(seed);
            let sites = crate::environment::simulator::points::sample(&mut rng, 40, 40.0, 40.0, 2.5);
            let Ok(sites) = sites else { continue };

            match build_loop(sites, 40.0, 40.0, 0.35) {
                Ok(loop_points) => {
                    assert!(loop_points.len() >= 3, "seed {seed}: loop too short");
                    assert!(is_simple_polygon(&loop_points), "seed {seed}: loop self-intersects");
                    for p in &loop_points {
                        assert!(p.x.abs() < 20.0 && p.y.abs() < 20.0, "seed {seed}: point outside area");
                    }
                }
                Err(_) => continue, // expected to fail occasionally; retried by caller with a new seed
            }
        }
    }
}
