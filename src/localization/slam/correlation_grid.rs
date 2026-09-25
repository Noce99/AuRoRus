//! [`CorrelationGrid`]: the grid a scan is correlated against - Karto's
//! `CorrelationGrid` (`Mapper.h`). Every point of the reference scans marks
//! its cell as [`OCCUPIED`], smeared out with a Gaussian kernel so a point
//! landing near (not exactly on) a reference point still scores.

/// Value of a cell a reference point landed in - also the peak of the
/// smearing kernel, and what a response is normalized by. Karto's
/// `GridStates_Occupied`.
pub const OCCUPIED: u8 = 100;

/// A square grid of `roi_side` x `roi_side` cells, padded on every side by
/// a border wide enough for the smearing kernel to never run off it.
///
/// Grid coordinates `(0, 0)` are the region of interest's first cell,
/// centered on [`offset`](Self::set_offset) - world to grid rounds to the
/// nearest cell, as Karto's `CoordinateConverter::WorldToGrid` does.
pub struct CorrelationGrid {
    resolution_m: f64,
    roi_side: i32,
    border: i32,
    width: i32,
    offset: (f64, f64),
    data: Vec<u8>,
    kernel_side: i32,
    kernel: Vec<u8>,
}

impl CorrelationGrid {
    /// # Panics
    ///
    /// Panics if `smear_deviation_m` isn't between half the resolution and
    /// ten times it - Karto's own bounds, outside which smearing either
    /// doesn't cover a cell's neighbors or blurs the grid into mush.
    pub fn new(roi_side: i32, resolution_m: f64, smear_deviation_m: f64) -> Self {
        assert!(
            (0.5 * resolution_m..=10.0 * resolution_m).contains(&smear_deviation_m),
            "CorrelationGrid: smear deviation {smear_deviation_m} m must be between {} and {} m \
             (half to ten times the resolution)",
            0.5 * resolution_m,
            10.0 * resolution_m
        );
        let half_kernel = half_kernel_size(smear_deviation_m, resolution_m);
        // +1 in case of roundoff, as Karto does.
        let border = half_kernel + 1;
        let width = roi_side + 2 * border;

        let kernel_side = 2 * half_kernel + 1;
        let mut kernel = vec![0u8; (kernel_side * kernel_side) as usize];
        for j in -half_kernel..=half_kernel {
            for i in -half_kernel..=half_kernel {
                let distance_m = (f64::from(i) * resolution_m).hypot(f64::from(j) * resolution_m);
                let z = (-0.5 * (distance_m / smear_deviation_m).powi(2)).exp();
                kernel[((i + half_kernel) + kernel_side * (j + half_kernel)) as usize] =
                    (z * f64::from(OCCUPIED)).round() as u8;
            }
        }

        Self {
            resolution_m,
            roi_side,
            border,
            width,
            offset: (0.0, 0.0),
            data: vec![0; (width * width) as usize],
            kernel_side,
            kernel,
        }
    }

    pub fn resolution_m(&self) -> f64 {
        self.resolution_m
    }

    /// Side of the region of interest, in cells.
    pub fn roi_side(&self) -> i32 {
        self.roi_side
    }

    /// Every cell, border included, row-major.
    pub fn data(&self) -> &[u8] {
        &self.data
    }

    /// Resets every cell to empty.
    pub fn clear(&mut self) {
        self.data.fill(0);
    }

    /// Centers grid cell `(0, 0)` on the world point `(x_m, y_m)`.
    pub fn set_offset(&mut self, x_m: f64, y_m: f64) {
        self.offset = (x_m, y_m);
    }

    /// The grid cell `(x_m, y_m)` falls in - possibly outside the region of
    /// interest.
    pub fn world_to_grid(&self, x_m: f64, y_m: f64) -> (i32, i32) {
        (
            ((x_m - self.offset.0) / self.resolution_m).round() as i32,
            ((y_m - self.offset.1) / self.resolution_m).round() as i32,
        )
    }

    /// Whether grid cell `(gx, gy)` is inside the region of interest.
    pub fn in_roi(&self, gx: i32, gy: i32) -> bool {
        (0..self.roi_side).contains(&gx) && (0..self.roi_side).contains(&gy)
    }

    /// Index into [`data`](Self::data) of grid cell `(gx, gy)`, which must be
    /// inside the region of interest or its border.
    pub fn index(&self, gx: i32, gy: i32) -> usize {
        ((gy + self.border) * self.width + (gx + self.border)) as usize
    }

    /// How far apart, in [`data`](Self::data) indices, two points `(dx_m,
    /// dy_m)` apart land - what Karto's `GridIndexLookup` precomputes, so a
    /// scan's points can be looked up relative to any candidate position
    /// with a single addition each.
    pub fn index_offset(&self, dx_m: f64, dy_m: f64) -> isize {
        let gx = (dx_m / self.resolution_m).round() as isize;
        let gy = (dy_m / self.resolution_m).round() as isize;
        gy * self.width as isize + gx
    }

    /// Marks the cell `(x_m, y_m)` falls in as [`OCCUPIED`] and smears the
    /// kernel around it - Karto's `ScanMatcher::AddScan` inner loop plus
    /// `CorrelationGrid::SmearPoint`. Points outside the region of interest
    /// are ignored.
    pub fn add_point(&mut self, x_m: f64, y_m: f64) {
        let (gx, gy) = self.world_to_grid(x_m, y_m);
        if !self.in_roi(gx, gy) {
            return;
        }
        let index = self.index(gx, gy);
        if self.data[index] == OCCUPIED {
            return;
        }
        self.data[index] = OCCUPIED;

        let half_kernel = self.kernel_side / 2;
        for j in -half_kernel..=half_kernel {
            let row = self.index(gx - half_kernel, gy + j);
            let kernel_row = (self.kernel_side * (j + half_kernel)) as usize;
            let cells = &mut self.data[row..row + self.kernel_side as usize];
            let kernel = &self.kernel[kernel_row..kernel_row + self.kernel_side as usize];
            for (cell, &value) in cells.iter_mut().zip(kernel) {
                if value > *cell {
                    *cell = value;
                }
            }
        }
    }
}

/// The smearing kernel's half-size: two standard deviations, Karto's
/// `GetHalfKernelSize`.
fn half_kernel_size(smear_deviation_m: f64, resolution_m: f64) -> i32 {
    (2.0 * smear_deviation_m / resolution_m).round() as i32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_point_marks_its_cell_and_fades_around_it() {
        let mut grid = CorrelationGrid::new(21, 0.01, 0.02);
        grid.set_offset(0.0, 0.0);
        grid.add_point(0.1, 0.1);

        let center = grid.data()[grid.index(10, 10)];
        let near = grid.data()[grid.index(11, 10)];
        let far = grid.data()[grid.index(13, 10)];
        assert_eq!(center, OCCUPIED);
        assert!(near < center && near > far, "{center} {near} {far}");
        assert_eq!(grid.data()[grid.index(20, 20)], 0);
    }

    #[test]
    fn points_outside_the_region_of_interest_are_ignored() {
        let mut grid = CorrelationGrid::new(11, 0.01, 0.02);
        grid.set_offset(0.0, 0.0);
        grid.add_point(-0.5, 0.05);
        assert!(grid.data().iter().all(|&cell| cell == 0));
    }

    #[test]
    fn index_offsets_match_absolute_indices() {
        let grid = CorrelationGrid::new(21, 0.01, 0.02);
        let from = grid.index(5, 5);
        let to = grid.index(8, 3);
        assert_eq!(from as isize + grid.index_offset(0.03, -0.02), to as isize);
    }

    #[test]
    #[should_panic(expected = "smear deviation")]
    fn a_smear_deviation_below_half_the_resolution_is_rejected() {
        CorrelationGrid::new(11, 0.01, 0.001);
    }
}
