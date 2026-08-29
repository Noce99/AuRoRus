//! The JSON/binary map API: listing, metadata, the raw raster, and map
//! generation. Built entirely on the existing [`crate::environment`] module
//! (`Map::load`, `read_info`, `GenerationConfig`, `generate`).

use crate::environment::{self, GenerationConfig, Map};
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};
use tiny_http::{Request, Response, ResponseBox};

/// One map's summary, as listed by [`list`].
#[derive(serde::Serialize)]
struct MapSummary {
    name: String,
    width_px: u32,
    height_px: u32,
    resolution_m_per_px: f64,
    seed: u64,
    generated_at: String,
}

/// `GET /api/maps` - every subfolder of `maps_root` with a valid
/// `info.json`, sorted by name.
pub fn list(maps_root: &Path) -> ResponseBox {
    let mut summaries = Vec::new();
    if let Ok(entries) = std::fs::read_dir(maps_root) {
        for entry in entries.flatten() {
            let path = entry.path();
            let Some(name) = path.file_name().and_then(|n| n.to_str()) else { continue };
            let Ok(info) = environment::read_info(&path) else { continue };
            summaries.push(MapSummary {
                name: name.to_string(),
                width_px: info.width_px,
                height_px: info.height_px,
                resolution_m_per_px: info.resolution_m_per_px,
                seed: info.seed,
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
        .with_header(super::header("Content-Type", "application/octet-stream"))
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
/// invalid parameter, before doing any generation work.
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
                return bad_request("invalid name: must not be empty or contain '/', '\\', or '..'");
            }
            Some(name.clone())
        }
        None => None,
    };

    let config = params.into_generation_config(maps_root.to_path_buf());
    if let Err(message) = config.validate() {
        return bad_request(&message);
    }

    match environment::generate(&config, folder_name.as_deref()) {
        Ok(generated) => {
            let name = generated.folder.file_name().and_then(|n| n.to_str()).unwrap_or_default().to_string();
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
}

impl GenerateParams {
    /// The values [`GenerationConfig::default`] would use, for
    /// pre-filling the generate popup - with a freshly rolled seed, the
    /// same "current unix time" default `generate_map`'s CLI uses, rather
    /// than the fixed `0` [`GenerationConfig::default`] itself falls back
    /// to (meant for library callers that always pass an explicit seed).
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
            max_lateral_accel_mps2: self.max_lateral_accel_mps2.unwrap_or(d.max_lateral_accel_mps2),
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

fn random_seed() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0)
}

/// Validates `name` as a map folder name: non-empty, and free of any path
/// separator or `..` component, so it can never escape `maps_root` - unlike
/// `generate_map`'s CLI flag, this one comes from an arbitrary HTTP client.
pub(super) fn safe_map_folder(name: &str, maps_root: &Path) -> Option<PathBuf> {
    if name.is_empty() || name == "." || name == ".." || name.contains('/') || name.contains('\\') {
        return None;
    }
    Some(maps_root.join(name))
}

pub(super) fn json_response<T: serde::Serialize>(value: &T, status: u16) -> ResponseBox {
    let body = serde_json::to_string(value).expect("serializing a well-formed API response never fails");
    Response::from_string(body)
        .with_status_code(status)
        .with_header(super::header("Content-Type", "application/json"))
        .boxed()
}

#[derive(serde::Serialize)]
struct ErrorBody {
    error: String,
}

fn error_response(status: u16, message: &str) -> ResponseBox {
    json_response(&ErrorBody { error: message.to_string() }, status)
}

pub(super) fn bad_request(message: &str) -> ResponseBox {
    error_response(400, message)
}

pub fn not_found() -> ResponseBox {
    error_response(404, "not found")
}
