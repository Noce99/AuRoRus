//! Encodes/decodes a [`Raster`] as a CCITT Group 4 (T.6) compressed,
//! single-strip TIFF file, via the `fax` crate's encoder/decoder and its own
//! `fax::tiff::wrap` helper - which builds a complete, valid minimal TIFF
//! container, so no hand-rolled IFD writing is needed for [`write`]. [`read`]
//! parses just enough of that same minimal, single-IFD, single-strip layout
//! back out (not general TIFF).

use crate::environment::raster::Raster;
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

/// Error returned by [`read`].
#[derive(Debug)]
pub enum TiffReadError {
    Io(std::io::Error),
    Invalid(String),
}

impl std::fmt::Display for TiffReadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(err) => write!(f, "failed to read TIFF file: {err}"),
            Self::Invalid(msg) => write!(f, "invalid TIFF file: {msg}"),
        }
    }
}

impl std::error::Error for TiffReadError {}

impl From<std::io::Error> for TiffReadError {
    fn from(err: std::io::Error) -> Self {
        Self::Io(err)
    }
}

/// Reads back a raster written by [`write`]: a little-endian, single-IFD,
/// single-strip CCITT Group 4 TIFF, exactly the shape `fax::tiff::wrap`
/// produces. Only the four tags `write` relies on
/// (`ImageWidth`/`ImageLength`/`StripOffsets`/`StripByteCounts`, tags
/// `256`/`257`/`273`/`279`) are read; each is a `count = 1` `LONG`, so its
/// 4-byte value sits directly in bytes `8..12` of its 12-byte IFD entry, with
/// no indirection to resolve.
pub fn read(path: &Path) -> Result<Raster, TiffReadError> {
    let bytes = std::fs::read(path)?;
    if bytes.len() < 8 || &bytes[0..4] != b"II*\0" {
        return Err(TiffReadError::Invalid("not a little-endian TIFF".into()));
    }
    let ifd_offset = u32::from_le_bytes(bytes[4..8].try_into().unwrap()) as usize;
    let entry_count = u16::from_le_bytes(bytes[ifd_offset..ifd_offset + 2].try_into().unwrap()) as usize;

    let (mut width, mut height, mut strip_offset, mut strip_len) = (None, None, None, None);
    for i in 0..entry_count {
        let entry_start = ifd_offset + 2 + i * 12;
        let entry = &bytes[entry_start..entry_start + 12];
        let tag = u16::from_le_bytes(entry[0..2].try_into().unwrap());
        let value = u32::from_le_bytes(entry[8..12].try_into().unwrap());
        match tag {
            256 => width = Some(value),
            257 => height = Some(value),
            273 => strip_offset = Some(value as usize),
            279 => strip_len = Some(value as usize),
            _ => {}
        }
    }

    let width = width.ok_or_else(|| TiffReadError::Invalid("missing ImageWidth tag".into()))?;
    let height = height.ok_or_else(|| TiffReadError::Invalid("missing ImageLength tag".into()))?;
    let strip_offset = strip_offset.ok_or_else(|| TiffReadError::Invalid("missing StripOffsets tag".into()))?;
    let strip_len = strip_len.ok_or_else(|| TiffReadError::Invalid("missing StripByteCounts tag".into()))?;
    let strip = bytes
        .get(strip_offset..strip_offset + strip_len)
        .ok_or_else(|| TiffReadError::Invalid("strip data out of bounds".into()))?;

    let mut white = Vec::with_capacity((width * height) as usize);
    fax::decoder::decode_g4(strip.iter().copied(), width, Some(height), |line| {
        white.extend(fax::decoder::pels(line, width).map(|c| c == fax::Color::White));
    })
    .ok_or_else(|| TiffReadError::Invalid("failed to decode Group 4 data".into()))?;

    Ok(Raster::new(width, height, white))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::environment::simulator::raster::{ImageTransform, rasterize};
    use crate::environment::simulator::smoothing::Point2;

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

    #[test]
    fn read_after_write_round_trips_pixels() {
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

        let path = std::env::temp_dir().join(format!("aurorus_tiff_round_trip_test_{}.tiff", std::process::id()));
        write(&raster, &path).unwrap();
        let read_back = read(&path).unwrap();
        std::fs::remove_file(&path).ok();

        assert_eq!(read_back.width_px, raster.width_px);
        assert_eq!(read_back.height_px, raster.height_px);
        for y in 0..raster.height_px {
            assert_eq!(read_back.row(y), raster.row(y), "row {y} mismatch");
        }
    }
}
