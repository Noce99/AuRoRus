//! Encodes a [`Raster`] as a CCITT Group 4 (T.6) compressed, single-strip
//! TIFF file, via the `fax` crate's encoder and its own `fax::tiff::wrap`
//! helper - which builds a complete, valid minimal TIFF container, so no
//! hand-rolled IFD/offset arithmetic is needed here.

use crate::simulator::environment::raster::Raster;
use std::path::Path;

/// Error returned by [`write`].
#[derive(Debug)]
pub enum TiffWriteError {
    Io(std::io::Error),
}

impl std::fmt::Display for TiffWriteError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(err) => write!(f, "failed to write TIFF file: {err}"),
        }
    }
}

impl std::error::Error for TiffWriteError {}

/// Writes `raster` to `path` as a CCITT Group 4 TIFF with
/// `PhotometricInterpretation = 0` (WhiteIsZero) - the only pairing
/// consistent with T.6's own semantics (a `Color::White` run always decodes
/// to sample value `0`, independent of that tag), so white pixels here
/// actually render as white.
pub fn write(raster: &Raster, path: &Path) -> Result<(), TiffWriteError> {
    let mut encoder = fax::encoder::Encoder::new(fax::VecWriter::new());
    for y in 0..raster.height_px {
        let pels = raster
            .row(y)
            .iter()
            .map(|&is_white| if is_white { fax::Color::White } else { fax::Color::Black });
        encoder
            .encode_line(pels, raster.width_px)
            .expect("VecWriter's BitWriter::Error is Infallible");
    }
    let writer = encoder.finish().expect("VecWriter's BitWriter::Error is Infallible");
    let strip = writer.finish();

    let tiff_bytes = fax::tiff::wrap(&strip, raster.width_px, raster.height_px);
    std::fs::write(path, tiff_bytes).map_err(TiffWriteError::Io)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulator::environment::raster::{ImageTransform, rasterize};
    use crate::simulator::environment::smoothing::Point2;

    #[test]
    fn written_file_starts_with_tiff_magic() {
        let centerline = vec![
            Point2 { x: -1.0, y: 0.0 },
            Point2 { x: 0.0, y: 1.0 },
            Point2 { x: 1.0, y: 0.0 },
            Point2 { x: 0.0, y: -1.0 },
        ];
        let transform = ImageTransform {
            resolution_m_per_px: 0.1,
            width_px: 20,
            height_px: 20,
            origin_x_m: -1.5,
            origin_y_m: -1.5,
        };
        let raster = rasterize(&centerline, 0.5, &transform);

        let path = std::env::temp_dir().join(format!("aurorus_tiff_test_{}.tiff", std::process::id()));
        write(&raster, &path).unwrap();
        let bytes = std::fs::read(&path).unwrap();
        std::fs::remove_file(&path).ok();

        assert_eq!(&bytes[0..4], &[0x49, 0x49, 0x2A, 0x00]);
    }
}
