//! The pixel editor's API: what the editor draws (the map's raster, its
//! start/finish line and race line) and saving the edited raster back.
//! Saving keeps the map's original raster aside the first time (see
//! [`environment::replace_raster`]) so the editor can always revert to it.

use super::maps_api::{reload_if_live, safe_map_folder};
use crate::Captain;
use crate::environment::{self, ImageOrigin, Raster, StartFinishLine, race_lines};
use crate::topics::{
    RACE_LINE_TOPIC_NAME, SLAM_STATUS_TOPIC_NAME, SelectedRaceLine, SlamState, SlamStatus,
};
use crate::web::{bad_request, error_response, header, json_response, not_found, query_param};
use std::path::{Path, PathBuf};
use tiny_http::{Request, Response, ResponseBox};

/// Everything the editor draws besides the pixels, in world coordinates -
/// the editor converts them to pixels with `origin` and the resolution.
#[derive(serde::Serialize)]
struct EditInfo {
    width_px: u32,
    height_px: u32,
    resolution_m_per_px: f64,
    origin: ImageOrigin,
    start_finish_line: StartFinishLine,
    /// The race line followed on this map, as `[x, y]` points - empty if
    /// it isn't the live map, or has none.
    race_line: Vec<[f64; 2]>,
    /// Whether an untouched original raster was kept to revert to.
    has_original: bool,
}

/// `name`'s folder, from the `name` query parameter.
fn folder_from(url: &str, maps_root: &Path) -> Result<PathBuf, ResponseBox> {
    let name = query_param(url, "name").ok_or_else(|| bad_request("missing name"))?;
    safe_map_folder(&name, maps_root).ok_or_else(|| bad_request("invalid map name"))
}

/// `GET /api/map_edit?name=..` - that map's [`EditInfo`].
pub fn info(url: &str, captain: &Captain, maps_root: &Path) -> ResponseBox {
    let folder = match folder_from(url, maps_root) {
        Ok(folder) => folder,
        Err(response) => return response,
    };
    let Ok(info) = environment::read_info(&folder) else {
        return not_found();
    };
    let followed = captain
        .try_topic::<SelectedRaceLine>(RACE_LINE_TOPIC_NAME)
        .map(|topic| topic.read().into_value())
        .unwrap_or_default();
    let race_line = if followed.map.as_deref() == Some(&folder) {
        followed.points.iter().map(|p| [p.x, p.y]).collect()
    } else {
        Vec::new()
    };
    json_response(
        &EditInfo {
            width_px: info.width_px,
            height_px: info.height_px,
            resolution_m_per_px: info.resolution_m_per_px,
            origin: info.origin,
            start_finish_line: info.start_finish_line,
            race_line,
            has_original: folder
                .join(environment::ORIGINAL_MAP_TIFF_FILE_NAME)
                .exists(),
        },
        200,
    )
}

/// `GET /api/map_edit/raster?name=..[&original=true]` - the map's raster
/// as read from disk (or, with `original`, the one kept before the first
/// edit - `404` if there's none), one byte per pixel, row-major, `255`
/// being drivable: the same layout `GET /api/maps/{name}/raster` serves.
pub fn raster(url: &str, maps_root: &Path) -> ResponseBox {
    let folder = match folder_from(url, maps_root) {
        Ok(folder) => folder,
        Err(response) => return response,
    };
    let raster = if query_param(url, "original").as_deref() == Some("true") {
        environment::read_original_raster(&folder).map_err(|err| err.to_string())
    } else {
        environment::read_raster(&folder)
            .map(Some)
            .map_err(|err| err.to_string())
    };
    match raster {
        Ok(Some(raster)) => Response::from_data(raster.to_bytes())
            .with_header(header("Content-Type", "application/octet-stream"))
            .boxed(),
        Ok(None) => not_found(),
        Err(message) => error_response(500, &message),
    }
}

/// The reply to a successful [`save`].
#[derive(serde::Serialize)]
struct Saved {
    /// How many race lines (the centerline included) the map has - all
    /// computed on the old pixels, so maybe crossing a wall now.
    race_lines: usize,
}

/// `POST /api/map_edit/raster?name=..` - the body is the edited raster in
/// [`raster`]'s layout, non-zero meaning drivable, sized as the map's
/// `info.json` says. Replaces its `map.tiff` (keeping the original aside
/// the first time) and reloads it if it's the live map. Refused with `409`
/// while SLAM maps or localizes: a reload resets odometry under it.
pub fn save(
    request: &mut Request,
    captain: &Captain,
    writer_id: u16,
    maps_root: &Path,
) -> ResponseBox {
    let url = request.url().to_string();
    let folder = match folder_from(&url, maps_root) {
        Ok(folder) => folder,
        Err(response) => return response,
    };
    let slam_state = captain
        .try_topic::<SlamStatus>(SLAM_STATUS_TOPIC_NAME)
        .map(|topic| topic.read().state)
        .unwrap_or_default();
    if matches!(slam_state, SlamState::Running | SlamState::Localizing) {
        return error_response(
            409,
            "Mapping or localization is running - pause it before saving the map.",
        );
    }
    let Ok(info) = environment::read_info(&folder) else {
        return not_found();
    };

    let mut pixels = Vec::new();
    if let Err(err) = request.as_reader().read_to_end(&mut pixels) {
        return bad_request(&format!("failed to read request body: {err}"));
    }
    let expected = info.width_px as usize * info.height_px as usize;
    if pixels.len() != expected {
        return bad_request(&format!(
            "expected {expected} raster bytes ({}x{}), got {}",
            info.width_px,
            info.height_px,
            pixels.len()
        ));
    }
    let raster = Raster::new(
        info.width_px,
        info.height_px,
        pixels.iter().map(|&pixel| pixel != 0).collect(),
    );
    if let Err(err) = environment::replace_raster(&folder, &raster) {
        return error_response(500, &err.to_string());
    }

    reload_if_live(captain, writer_id, &folder);
    json_response(
        &Saved {
            race_lines: race_lines::list(&folder).len(),
        },
        200,
    )
}
