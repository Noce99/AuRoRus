//! A centerline for a map that has none (e.g. one saved by
//! [`crate::localization::Slam`]), from its walls alone: the curve where
//! the distance to the inner wall equals the distance to the outer one -
//! the zero line of `d_inner - d_outer`, traced with marching squares. It's
//! a single closed curve with sub-pixel accuracy, without the side branches
//! a skeleton would grow.

use super::PlanError;
use super::track::TrackGrid;
use crate::geometry::{Point2, loop_length};
use std::collections::HashMap;

/// The track's centerline: the longest closed curve where the distance to
/// either wall is the same, in no particular direction or start.
pub fn extract(grid: &TrackGrid) -> Result<Vec<Point2>, PlanError> {
    let (to_inner, to_outer) = grid.wall_distances();
    let field: Vec<f64> = to_inner
        .iter()
        .zip(&to_outer)
        .map(|(inner, outer)| inner - outer)
        .collect();
    let (width, height) = grid.size();
    let corner = |col: usize, row: usize| grid.cell_center(col, row);

    contours(&field, width, height, corner)
        .into_iter()
        .max_by(|a, b| loop_length(a).total_cmp(&loop_length(b)))
        .filter(|contour| contour.len() >= 3)
        .ok_or(PlanError::NoCenterline)
}

/// An edge between two neighboring samples of the field: the horizontal
/// one from (`col`, `row`) to (`col + 1`, `row`), or the vertical one from
/// (`col`, `row`) to (`col`, `row + 1`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum Edge {
    Horizontal(usize, usize),
    Vertical(usize, usize),
}

/// Every closed curve where `field` (row-major, `width` x `height` samples
/// placed at `position(col, row)`) crosses zero, as marching squares finds
/// them. Samples `> 0` count as inside. Open curves - which only a field
/// crossing zero on the grid's border would produce - are dropped.
fn contours(
    field: &[f64],
    width: usize,
    height: usize,
    position: impl Fn(usize, usize) -> Point2,
) -> Vec<Vec<Point2>> {
    let value = |col: usize, row: usize| field[row * width + col];
    let inside = |col: usize, row: usize| value(col, row) > 0.0;

    // Every square with a crossing contributes one or two segments, each
    // joining two edges.
    let mut segments: Vec<(Edge, Edge)> = Vec::new();
    for row in 0..height.saturating_sub(1) {
        for col in 0..width.saturating_sub(1) {
            let corners = [
                inside(col, row),
                inside(col + 1, row),
                inside(col + 1, row + 1),
                inside(col, row + 1),
            ];
            if corners.iter().all(|&c| c == corners[0]) {
                continue;
            }
            let top = Edge::Horizontal(col, row);
            let right = Edge::Vertical(col + 1, row);
            let bottom = Edge::Horizontal(col, row + 1);
            let left = Edge::Vertical(col, row);
            // Each corner's two edges, clockwise from the top-left.
            let corner_edges = [(left, top), (top, right), (right, bottom), (bottom, left)];
            let differing = corners.iter().filter(|&&c| c != corners[0]).count();
            let saddle = differing == 2 && corners[0] == corners[2];
            if saddle {
                // Two opposite corners inside: the square's center decides
                // which pair is connected through it. Every corner on the
                // other side of the center gets cut off on its own.
                let center = (value(col, row)
                    + value(col + 1, row)
                    + value(col + 1, row + 1)
                    + value(col, row + 1))
                    / 4.0
                    > 0.0;
                for (corner, edges) in corners.iter().zip(corner_edges) {
                    if *corner != center {
                        segments.push(edges);
                    }
                }
            } else {
                // One corner differs from the other three: cut it off.
                let lonely = (0..4)
                    .find(|&i| corners.iter().filter(|&&c| c == corners[i]).count() == 1)
                    .map(|i| corner_edges[i]);
                if let Some(edges) = lonely {
                    segments.push(edges);
                } else {
                    // Two neighboring corners inside: a straight cut
                    // between the two edges that cross.
                    let crossing: Vec<Edge> = [top, right, bottom, left]
                        .into_iter()
                        .zip([(0, 1), (1, 2), (2, 3), (3, 0)])
                        .filter(|(_, (a, b))| corners[*a] != corners[*b])
                        .map(|(edge, _)| edge)
                        .collect();
                    segments.push((crossing[0], crossing[1]));
                }
            }
        }
    }

    // Where the field crosses zero along an edge, linearly interpolated.
    let crossing = |edge: Edge| -> Point2 {
        let ((c0, r0), (c1, r1)) = match edge {
            Edge::Horizontal(col, row) => ((col, row), (col + 1, row)),
            Edge::Vertical(col, row) => ((col, row), (col, row + 1)),
        };
        let (v0, v1) = (value(c0, r0), value(c1, r1));
        let t = if (v0 - v1).abs() < 1e-12 {
            0.5
        } else {
            (v0 / (v0 - v1)).clamp(0.0, 1.0)
        };
        let (a, b) = (position(c0, r0), position(c1, r1));
        Point2 {
            x: a.x + t * (b.x - a.x),
            y: a.y + t * (b.y - a.y),
        }
    };

    // Chain the segments into curves through their shared edges: every
    // crossed edge is shared by the two squares either side of it.
    let mut at_edge: HashMap<Edge, Vec<usize>> = HashMap::new();
    for (index, &(a, b)) in segments.iter().enumerate() {
        at_edge.entry(a).or_default().push(index);
        at_edge.entry(b).or_default().push(index);
    }
    let mut used = vec![false; segments.len()];
    let mut curves = Vec::new();
    for first in 0..segments.len() {
        if used[first] {
            continue;
        }
        used[first] = true;
        let (start, mut edge) = segments[first];
        let mut edges = vec![start];
        let closed = loop {
            if edge == start {
                break true;
            }
            edges.push(edge);
            let next = at_edge[&edge]
                .iter()
                .copied()
                .find(|&segment| !used[segment]);
            let Some(next) = next else {
                break false;
            };
            used[next] = true;
            let (a, b) = segments[next];
            edge = if a == edge { b } else { a };
        };
        if closed {
            curves.push(edges.into_iter().map(crossing).collect());
        }
    }
    curves
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::track::tests::ring_map;

    #[test]
    fn a_ring_s_centerline_is_its_mid_radius_circle() {
        let map = ring_map(1.0, 2.0, 0.05);
        let grid = TrackGrid::build(&map).unwrap();
        let centerline = extract(&grid).unwrap();

        assert!(centerline.len() > 100);
        for point in &centerline {
            let radius = (point.x * point.x + point.y * point.y).sqrt();
            assert!((radius - 1.5).abs() < 0.03, "radius {radius}");
        }
    }

    #[test]
    fn a_saddle_is_resolved_by_the_square_s_center() {
        // Two opposite corners inside, with a positive center: one curve
        // through the square, cutting off each outside corner.
        let field = [1.0, -0.1, -0.1, 1.0];
        let position = |col: usize, row: usize| Point2 {
            x: col as f64,
            y: row as f64,
        };
        // A single square has no closed curves - but it mustn't panic.
        assert!(contours(&field, 2, 2, position).is_empty());
    }
}
