//! [`Raster`]: a binary occupancy bitmap, independent of how it was
//! produced - by [`crate::environment::generator::raster::rasterize`] or
//! loaded from disk via [`crate::environment::tiff::read`].

/// A binary occupancy raster: one `bool` per pixel, `true` meaning white,
/// i.e. drivable track area.
///
/// One *logical* bit of information per pixel, but stored as a `Vec<bool>`,
/// which is a whole byte each rather than a packed bitfield - so a
/// 1200x1200 map is 1.44 MB, not 180 KB. Worth knowing before copying one
/// around.
#[derive(Debug)]
pub struct Raster {
    pub width_px: u32,
    pub height_px: u32,
    white: Vec<bool>,
}

impl Raster {
    pub(crate) fn new(width_px: u32, height_px: u32, white: Vec<bool>) -> Self {
        Self {
            width_px,
            height_px,
            white,
        }
    }

    /// The pixels of row `y`, left to right.
    pub fn row(&self, y: u32) -> &[bool] {
        let start = (y as usize) * (self.width_px as usize);
        &self.white[start..start + self.width_px as usize]
    }

    /// The whole raster as one byte per pixel, row-major: `255` for
    /// drivable (white), `0` otherwise - no image codec needed, a consumer
    /// (e.g. a browser) can build an `ImageData` directly from these.
    pub fn to_bytes(&self) -> Vec<u8> {
        self.white
            .iter()
            .map(|&white| if white { 255u8 } else { 0u8 })
            .collect()
    }
}
