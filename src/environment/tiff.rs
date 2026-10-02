//! Encodes/decodes a [`Raster`] as a TIFF file. [`write`] produces a CCITT
//! Group 4 (T.6) compressed, single-strip TIFF via the `fax` crate's encoder
//! and its own `fax::tiff::wrap` helper - which builds a complete, valid
//! minimal TIFF container, so no hand-rolled IFD writing is needed. [`read`]
//! goes through the general `tiff` crate instead, so maps re-saved by an
//! image editor (e.g. GIMP splitting the image into several strips) still load.

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
        let pels = raster.row(y).iter().map(|&is_white| {
            if is_white {
                fax::Color::White
            } else {
                fax::Color::Black
            }
        });
        encoder
            .encode_line(pels, raster.width_px)
            .expect("VecWriter's BitWriter::Error is Infallible");
    }
    let writer = encoder
        .finish()
        .expect("VecWriter's BitWriter::Error is Infallible");
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

/// Reads a map raster from any TIFF [`decode_grayscale`] understands - not
/// only the single-strip Group 4 files [`write`] produces, but also ones
/// re-saved by an image editor (multiple strips, other compressions, 8-bit
/// gray or RGB). Pixels at least half-bright count as white (drivable).
pub fn read(path: &Path) -> Result<Raster, TiffReadError> {
    let bytes = std::fs::read(path)?;
    let (width, height, gray) = decode_grayscale(&bytes).map_err(TiffReadError::Invalid)?;
    let white = gray.into_iter().map(|value| value >= 128).collect();
    Ok(Raster::new(width, height, white))
}

/// Decodes the first image of a TIFF (any compression the `tiff` crate
/// knows, CCITT Group 4 included) into `(width, height, gray)`: one
/// brightness byte per pixel, row-major, `255` being white (the crate
/// already resolves `WhiteIsZero`), treating transparent pixels as black
/// like the browser-side decoding does. Grayscale and RGB, with or without alpha, at
/// 1, 8 or 16 bits per sample.
pub fn decode_grayscale(bytes: &[u8]) -> Result<(u32, u32, Vec<u8>), String> {
    use tiff::ColorType;
    use tiff::decoder::{Decoder, DecodingResult};

    let mut decoder = Decoder::new(std::io::Cursor::new(bytes)).map_err(|err| err.to_string())?;
    let (width, height) = decoder.dimensions().map_err(|err| err.to_string())?;
    let color_type = decoder.colortype().map_err(|err| err.to_string())?;
    let (channels, bits) = match color_type {
        ColorType::Gray(bits) => (1, bits),
        ColorType::GrayA(bits) => (2, bits),
        ColorType::RGB(bits) => (3, bits),
        ColorType::RGBA(bits) => (4, bits),
        other => return Err(format!("unsupported color type {other:?}")),
    };
    let image = decoder.read_image().map_err(|err| err.to_string())?;

    let (width_px, height_px) = (width as usize, height as usize);
    let samples_per_row = width_px * channels;
    // Every sample of pixel row `y`, scaled to 0-255.
    let row_samples: Box<dyn Fn(usize) -> Vec<u8>> = match (bits, &image) {
        (1, DecodingResult::U8(packed)) => {
            // Bit-packed, MSB first, each row padded to a whole byte.
            let row_bytes = samples_per_row.div_ceil(8);
            Box::new(move |y| {
                (0..samples_per_row)
                    .map(|i| {
                        let byte = packed.get(y * row_bytes + i / 8).copied().unwrap_or(0);
                        if byte & (0x80 >> (i % 8)) != 0 {
                            255
                        } else {
                            0
                        }
                    })
                    .collect()
            })
        }
        (8, DecodingResult::U8(samples)) => {
            Box::new(move |y| samples[y * samples_per_row..][..samples_per_row].to_vec())
        }
        (16, DecodingResult::U16(samples)) => Box::new(move |y| {
            samples[y * samples_per_row..][..samples_per_row]
                .iter()
                .map(|&sample| (sample >> 8) as u8)
                .collect()
        }),
        _ => return Err(format!("unsupported {bits}-bit samples")),
    };

    let mut gray = Vec::with_capacity(width_px * height_px);
    for y in 0..height_px {
        for pixel in row_samples(y).chunks_exact(channels) {
            let (value, alpha) = match *pixel {
                [g] => (g, 255),
                [g, a] => (g, a),
                [r, g, b] => (luma(r, g, b), 255),
                [r, g, b, a] => (luma(r, g, b), a),
                _ => unreachable!("chunks_exact(channels) with channels in 1..=4"),
            };
            gray.push((value as u16 * alpha as u16 / 255) as u8);
        }
    }
    Ok((width, height, gray))
}

/// Rec. 601 luma of an RGB pixel, like the browser-side decoding.
fn luma(r: u8, g: u8, b: u8) -> u8 {
    (0.299 * r as f64 + 0.587 * g as f64 + 0.114 * b as f64).round() as u8
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::environment::generator::raster::{ImageTransform, rasterize};
    use crate::geometry::Point2;

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

        let path =
            std::env::temp_dir().join(format!("aurorus_tiff_test_{}.tiff", std::process::id()));
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

        let path = std::env::temp_dir().join(format!(
            "aurorus_tiff_round_trip_test_{}.tiff",
            std::process::id()
        ));
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
