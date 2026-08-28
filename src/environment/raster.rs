//! [`Raster`]: a 1-bit-per-pixel occupancy bitmap, independent of how it was
//! produced - by [`crate::environment::simulator::raster::rasterize`] or
//! loaded from disk via [`crate::environment::tiff::read`].

/// A 1-bit-per-pixel raster: `true` means white, i.e. drivable track area.
#[derive(Debug)]
pub struct Raster {
    pub width_px: u32,
    pub height_px: u32,
    white: Vec<bool>,
}

impl Raster {
    pub(crate) fn new(width_px: u32, height_px: u32, white: Vec<bool>) -> Self {
        Self { width_px, height_px, white }
    }

    /// The pixels of row `y`, left to right.
    pub fn row(&self, y: u32) -> &[bool] {
        let start = (y as usize) * (self.width_px as usize);
        &self.white[start..start + self.width_px as usize]
    }
}
