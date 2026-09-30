//! The JSON/binary map API: listing, metadata, the raw raster, map
//! generation and image import. Built entirely on the existing
//! [`crate::environment`] module (`Map::load`, `read_info`,
//! `GenerationConfig`, `generate`, `save`).

use crate::Captain;
use crate::environment::{
    self, GenerationConfig, ImageOrigin, Map, MapGenerationError, MapInfo, MapSource, Raster,
    StartFinishLine, WorldPoint, random_seed,
};
use crate::topics::{MAP_SELECTION_TOPIC_NAME, MAP_TOPIC_NAME, MapSelection, SelectedMap};
use crate::web::{
    bad_request, error_response, header, json_response, not_found, query_param, read_json,
};
use std::path::{Path, PathBuf};
use tiny_http::{Request, Response, ResponseBox};

/// One map's summary, as listed by [`list`].
#[derive(serde::Serialize)]
struct MapSummary {
    name: String,
    width_px: u32,
    height_px: u32,
    resolution_m_per_px: f64,
    source: MapSource,
    /// Only for generated maps.
    seed: Option<u64>,
    generated_at: String,
}

/// `GET /api/maps` - every subfolder of `maps_root` with a valid
/// `info.json`, sorted by name.
pub fn list(maps_root: &Path) -> ResponseBox {
    let mut summaries = Vec::new();
    if let Ok(entries) = std::fs::read_dir(maps_root) {
        for entry in entries.flatten() {
            let path = entry.path();
            let Some(name) = path.file_name().and_then(|n| n.to_str()) else {
                continue;
            };
            let Ok(info) = environment::read_info(&path) else {
                continue;
            };
            summaries.push(MapSummary {
                name: name.to_string(),
                width_px: info.width_px,
                height_px: info.height_px,
                resolution_m_per_px: info.resolution_m_per_px,
                source: info.source,
                seed: info.generation.map(|generation| generation.seed),
                generated_at: info.generated_at,
            });
        }
    }
    summaries.sort_by(|a, b| a.name.cmp(&b.name));
    json_response(&summaries, 200)
}

/// `GET /api/maps/{name}/info` - that map's full `MapInfo`.
pub fn info(name: &str, maps_root: &Path) -> ResponseBox {
    let Some(folder) = safe_map_folder(name, maps_root) else {
        return bad_request("invalid map name");
    };
    match environment::read_info(&folder) {
        Ok(info) => json_response(&info, 200),
        Err(_) => not_found(),
    }
}

/// `GET /api/maps/{name}/raster` - the occupancy grid as raw bytes, one
/// byte per pixel (`0` or `255`), row-major - no image codec needed, the
/// browser builds an `ImageData` directly from these.
pub fn raster(name: &str, maps_root: &Path) -> ResponseBox {
    let Some(folder) = safe_map_folder(name, maps_root) else {
        return bad_request("invalid map name");
    };
    let map = match Map::load(&folder) {
        Ok(map) => map,
        Err(_) => return not_found(),
    };

    Response::from_data(map.raster.to_bytes())
        .with_header(header("Content-Type", "application/octet-stream"))
        .boxed()
}

/// `GET /api/generate/defaults` - [`GenerationConfig::default`]'s values,
/// so the generate popup and the server can never drift apart.
pub fn generate_defaults() -> ResponseBox {
    json_response(&GenerateParams::defaults(), 200)
}

/// `POST /api/maps/generate` - validates the given (or defaulted)
/// parameters and, if they're valid, generates a new map under
/// `maps_root`. Returns `400` with `{ "error": "..." }` on the first
/// invalid parameter, before doing any generation work, and `409` if a map
/// with the requested name already exists and the body didn't ask to
/// replace it (`"overwrite": true`).
pub fn generate(request: &mut Request, maps_root: &Path) -> ResponseBox {
    let mut body = String::new();
    if let Err(err) = request.as_reader().read_to_string(&mut body) {
        return bad_request(&format!("failed to read request body: {err}"));
    }

    let params: GenerateParams = if body.trim().is_empty() {
        GenerateParams::default()
    } else {
        match serde_json::from_str(&body) {
            Ok(params) => params,
            Err(err) => return bad_request(&format!("invalid JSON body: {err}")),
        }
    };

    let folder_name = match &params.name {
        Some(name) => {
            if safe_map_folder(name, maps_root).is_none() {
                return bad_request(
                    "invalid name: must not be empty or contain '/', '\\', or '..'",
                );
            }
            Some(name.clone())
        }
        None => None,
    };

    let overwrite = params.overwrite.unwrap_or(false);
    let config = params.into_generation_config(maps_root.to_path_buf());
    if let Err(message) = config.validate() {
        return bad_request(&message);
    }

    match environment::generate(&config, folder_name.as_deref(), overwrite) {
        Ok(generated) => {
            let name = generated
                .folder
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or_default()
                .to_string();
            json_response(
                &GeneratedMapSummary {
                    name,
                    width_px: generated.width_px,
                    height_px: generated.height_px,
                    num_race_line_points: generated.num_race_line_points,
                },
                200,
            )
        }
        // A name collision is the client's to resolve (pick another name, or
        // re-send with "overwrite": true), not a server fault - 409, not 500.
        Err(err @ MapGenerationError::FolderExists(_)) => error_response(409, &err.to_string()),
        Err(err) => error_response(500, &err.to_string()),
    }
}

/// Every [`GenerationConfig`] field, all optional so a partial JSON body
/// (or none at all) falls back to [`GenerationConfig::default`] field by
/// field, plus an optional output folder name override.
#[derive(Debug, Default, serde::Serialize, serde::Deserialize)]
#[serde(default)]
struct GenerateParams {
    seed: Option<u64>,
    area_width_m: Option<f64>,
    area_height_m: Option<f64>,
    resolution_m_per_px: Option<f64>,
    num_sites: Option<usize>,
    min_site_spacing_m: Option<f64>,
    target_area_fraction: Option<f64>,
    track_width_m: Option<f64>,
    point_spacing_m: Option<f64>,
    smoothing_samples_per_segment: Option<usize>,
    max_speed_mps: Option<f64>,
    max_lateral_accel_mps2: Option<f64>,
    name: Option<String>,
    /// Replace an existing map of the same name instead of failing with
    /// `409`. Never defaulted to `true`, and deliberately absent from
    /// [`Self::defaults`] so the generate popup can't pre-fill it.
    overwrite: Option<bool>,
}

impl GenerateParams {
    /// The values [`GenerationConfig::default`] would use, for
    /// pre-filling the generate popup - with a freshly rolled seed (the
    /// same [`random_seed`] `generate_map`'s CLI defaults to), rather than
    /// the fixed `0` [`GenerationConfig::default`] itself falls back to,
    /// which is meant for library callers that always pass an explicit one.
    fn defaults() -> Self {
        let d = GenerationConfig::default();
        Self {
            seed: Some(random_seed()),
            area_width_m: Some(d.area_width_m),
            area_height_m: Some(d.area_height_m),
            resolution_m_per_px: Some(d.resolution_m_per_px),
            num_sites: Some(d.num_sites),
            min_site_spacing_m: Some(d.min_site_spacing_m),
            target_area_fraction: Some(d.target_area_fraction),
            track_width_m: Some(d.track_width_m),
            point_spacing_m: Some(d.point_spacing_m),
            smoothing_samples_per_segment: Some(d.smoothing_samples_per_segment),
            max_speed_mps: Some(d.max_speed_mps),
            max_lateral_accel_mps2: Some(d.max_lateral_accel_mps2),
            name: None,
            overwrite: None,
        }
    }

    /// Merges this (partial) request into a full [`GenerationConfig`],
    /// filling any missing field from [`GenerationConfig::default`] (or,
    /// for `seed`, a freshly rolled one - see [`Self::defaults`]).
    fn into_generation_config(self, output_root: PathBuf) -> GenerationConfig {
        let d = GenerationConfig::default();
        GenerationConfig {
            seed: self.seed.unwrap_or_else(random_seed),
            area_width_m: self.area_width_m.unwrap_or(d.area_width_m),
            area_height_m: self.area_height_m.unwrap_or(d.area_height_m),
            resolution_m_per_px: self.resolution_m_per_px.unwrap_or(d.resolution_m_per_px),
            num_sites: self.num_sites.unwrap_or(d.num_sites),
            min_site_spacing_m: self.min_site_spacing_m.unwrap_or(d.min_site_spacing_m),
            target_area_fraction: self.target_area_fraction.unwrap_or(d.target_area_fraction),
            track_width_m: self.track_width_m.unwrap_or(d.track_width_m),
            point_spacing_m: self.point_spacing_m.unwrap_or(d.point_spacing_m),
            smoothing_samples_per_segment: self
                .smoothing_samples_per_segment
                .unwrap_or(d.smoothing_samples_per_segment),
            max_speed_mps: self.max_speed_mps.unwrap_or(d.max_speed_mps),
            max_lateral_accel_mps2: self
                .max_lateral_accel_mps2
                .unwrap_or(d.max_lateral_accel_mps2),
            output_root,
        }
    }
}

/// A freshly generated map's summary, returned by a successful [`generate`].
#[derive(serde::Serialize)]
struct GeneratedMapSummary {
    name: String,
    width_px: u32,
    height_px: u32,
    num_race_line_points: usize,
}

/// `POST /api/maps/import?name=..&width_px=..&height_px=..&resolution_m_per_px=..&a_x_px=..&a_y_px=..&b_x_px=..&b_y_px=..`:
/// saves an image the browser already decoded and thresholded as a new
/// map folder (`map.tiff` + `info.json`) under `maps_root`. The body is the
/// raster, one byte per pixel, row-major, non-zero meaning drivable - the
/// same layout [`raster`] serves back. The start/finish line comes in pixel
/// coordinates (`a` on the driver's left, see [`StartFinishLine`]); the
/// image is placed centered on the world origin, like a generated map.
/// Returns `409` if a folder of that name already exists.
pub fn import(request: &mut Request, maps_root: &Path) -> ResponseBox {
    let url = request.url().to_string();
    let Some(name) = query_param(&url, "name") else {
        return bad_request("missing name");
    };
    let Some(folder) = safe_map_folder(&name, maps_root) else {
        return bad_request("invalid name: must not be empty or contain '/', '\\', or '..'");
    };
    let number = |key: &str| -> Result<f64, ResponseBox> {
        query_param(&url, key)
            .and_then(|value| value.parse::<f64>().ok())
            .filter(|value| value.is_finite())
            .ok_or_else(|| bad_request(&format!("missing or invalid {key}")))
    };
    let parsed = (|| {
        Ok::<_, ResponseBox>((
            number("width_px")?,
            number("height_px")?,
            number("resolution_m_per_px")?,
            [number("a_x_px")?, number("a_y_px")?],
            [number("b_x_px")?, number("b_y_px")?],
        ))
    })();
    let (width_px, height_px, resolution_m_per_px, a_px, b_px) = match parsed {
        Ok(parsed) => parsed,
        Err(response) => return response,
    };
    if width_px < 1.0 || height_px < 1.0 || width_px.fract() != 0.0 || height_px.fract() != 0.0 {
        return bad_request("width_px and height_px must be positive integers");
    }
    if resolution_m_per_px <= 0.0 {
        return bad_request("resolution_m_per_px must be positive");
    }
    if a_px == b_px {
        return bad_request("the start line's two ends must differ");
    }
    let (width_px, height_px) = (width_px as u32, height_px as u32);

    let mut pixels = Vec::new();
    if let Err(err) = request.as_reader().read_to_end(&mut pixels) {
        return bad_request(&format!("failed to read request body: {err}"));
    }
    if pixels.len() != width_px as usize * height_px as usize {
        return bad_request(&format!(
            "expected {} raster bytes ({width_px}x{height_px}), got {}",
            width_px as usize * height_px as usize,
            pixels.len()
        ));
    }

    if folder.exists() {
        return error_response(409, &format!("a folder named {name:?} already exists"));
    }

    let origin = ImageOrigin {
        x: -(width_px as f64) * resolution_m_per_px / 2.0,
        y: -(height_px as f64) * resolution_m_per_px / 2.0,
        theta_rad: 0.0,
    };
    let to_world = |[x_px, y_px]: [f64; 2]| WorldPoint {
        x: origin.x + x_px * resolution_m_per_px,
        y: origin.y + y_px * resolution_m_per_px,
    };
    let info = MapInfo {
        resolution_m_per_px,
        width_px,
        height_px,
        origin,
        start_finish_line: StartFinishLine {
            a: to_world(a_px),
            b: to_world(b_px),
        },
        generated_at: environment::now_rfc3339(),
        source: MapSource::Imported,
        generation: None,
    };
    let raster = Raster::new(
        width_px,
        height_px,
        pixels.iter().map(|&pixel| pixel != 0).collect(),
    );

    match environment::save(&folder, &info, &raster) {
        Ok(()) => json_response(&ImportedMapSummary { name }, 200),
        Err(err) => error_response(500, &err.to_string()),
    }
}

/// `POST /api/maps/import/decode_tiff`: decodes the TIFF in the body (the
/// browser can't) into what the import popup works on: an 8-byte header
/// (width, then height, each a little-endian `u32`) followed by one
/// brightness byte per pixel, row-major, `255` being white. See
/// [`environment::decode_tiff_grayscale`].
pub fn decode_tiff(request: &mut Request) -> ResponseBox {
    let mut bytes = Vec::new();
    if let Err(err) = request.as_reader().read_to_end(&mut bytes) {
        return bad_request(&format!("failed to read request body: {err}"));
    }
    match environment::decode_tiff_grayscale(&bytes) {
        Ok((width, height, gray)) => {
            let mut body = Vec::with_capacity(8 + gray.len());
            body.extend_from_slice(&width.to_le_bytes());
            body.extend_from_slice(&height.to_le_bytes());
            body.extend_from_slice(&gray);
            Response::from_data(body)
                .with_header(header("Content-Type", "application/octet-stream"))
                .boxed()
        }
        Err(message) => bad_request(&format!("can't decode this TIFF: {message}")),
    }
}

/// A freshly imported map's summary, returned by a successful [`import`].
#[derive(serde::Serialize)]
struct ImportedMapSummary {
    name: String,
}

#[derive(serde::Deserialize)]
struct StartFinishLineBody {
    name: String,
    a: WorldPoint,
    b: WorldPoint,
}

/// `POST /api/maps/start_finish_line` - body `{"name": ..., "a": {"x":
/// .., "y": ..}, "b": {..}}`, in world coordinates, `a` on the driver's left
/// (see [`StartFinishLine`]): moves that map's start/finish line, rewriting
/// its `info.json`. If it's the live map, `map_selection`'s revision is
/// bumped so [`crate::sensors::MapServer`] reloads it - and with it the
/// start state and the drawn line.
pub fn set_start_finish_line(
    request: &mut Request,
    captain: &Captain,
    writer_id: u16,
    maps_root: &Path,
) -> ResponseBox {
    let body: StartFinishLineBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    let Some(folder) = safe_map_folder(&body.name, maps_root) else {
        return bad_request("invalid map name");
    };
    let (a, b) = (body.a, body.b);
    if ![a.x, a.y, b.x, b.y].iter().all(|value| value.is_finite()) {
        return bad_request("the start line's ends must be finite");
    }
    if a.x == b.x && a.y == b.y {
        return bad_request("the start line's two ends must differ");
    }
    let mut info = match environment::read_info(&folder) {
        Ok(info) => info,
        Err(_) => return not_found(),
    };
    info.start_finish_line = StartFinishLine { a, b };
    if let Err(err) = environment::write_info(&folder, &info) {
        return error_response(500, &err.to_string());
    }

    reload_if_live(captain, writer_id, &folder);
    json_response(&info.start_finish_line, 200)
}

/// Has [`crate::sensors::MapServer`] read `folder` from disk again if it's
/// the live map, by bumping `map_selection`'s revision - after its files
/// were rewritten in place.
pub(super) fn reload_if_live(captain: &Captain, writer_id: u16, folder: &Path) {
    let live = captain
        .topic::<SelectedMap>(MAP_TOPIC_NAME)
        .read()
        .path
        .as_deref()
        == Some(folder);
    if !live {
        return;
    }
    let selection_topic = captain.topic::<MapSelection>(MAP_SELECTION_TOPIC_NAME);
    let revision = selection_topic.read().revision.wrapping_add(1);
    selection_topic
        .write(
            writer_id,
            MapSelection {
                path: Some(folder.to_path_buf()),
                revision,
            },
        )
        .expect("lost writer authorization for the map_selection topic");
}

/// `name`'s folder under `maps_root` - see [`environment::map_folder`].
pub(super) fn safe_map_folder(name: &str, maps_root: &Path) -> Option<PathBuf> {
    environment::map_folder(maps_root, name)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_map_tiff_decodes_to_the_same_pixels_as_its_raster() {
        let (width, height) = (37u32, 11u32);
        let white: Vec<bool> = (0..width * height).map(|i| (i * 7) % 5 < 2).collect();
        let raster = Raster::new(width, height, white.clone());
        let info = MapInfo {
            resolution_m_per_px: 0.05,
            width_px: width,
            height_px: height,
            origin: ImageOrigin {
                x: 0.0,
                y: 0.0,
                theta_rad: 0.0,
            },
            start_finish_line: StartFinishLine {
                a: WorldPoint { x: 0.0, y: 0.0 },
                b: WorldPoint { x: 1.0, y: 0.0 },
            },
            generated_at: environment::now_rfc3339(),
            source: MapSource::Imported,
            generation: None,
        };
        let folder =
            std::env::temp_dir().join(format!("aurorus_decode_tiff_test_{}", std::process::id()));
        environment::save(&folder, &info, &raster).unwrap();
        let bytes = std::fs::read(folder.join("map.tiff")).unwrap();
        std::fs::remove_dir_all(&folder).ok();

        let (decoded_width, decoded_height, gray) =
            environment::decode_tiff_grayscale(&bytes).unwrap();
        assert_eq!((decoded_width, decoded_height), (width, height));
        let expected: Vec<u8> = white.iter().map(|&w| if w { 255 } else { 0 }).collect();
        assert_eq!(gray, expected);
    }

    #[test]
    fn garbage_is_an_error_not_a_panic() {
        assert!(environment::decode_tiff_grayscale(b"not a tiff at all").is_err());
    }
}
