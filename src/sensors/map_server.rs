//! [`MapServer`]: watches [`MAP_SELECTION_TOPIC_NAME`] for the wanted map
//! folder and keeps [`MAP_TOPIC_NAME`] and [`START_STATE_TOPIC_NAME`] in sync
//! with it, loading from disk only when the selection actually changes, and
//! draws the loaded map on its own drawing topic (see
//! [`crate::topics::Drawing`]).

use crate::environment::Map;
use crate::topics::{
    Color, Drawing, MAP_SELECTION_TOPIC_NAME, Shape, MAP_TOPIC_NAME, MapSelection, SelectedMap, START_STATE_TOPIC_NAME, StartState,
};
use crate::{Captain, Executor, Ticker};
use std::any::Any;
use std::path::Path;
use std::time::Duration;

/// Every tunable parameter [`MapServer`] needs - loaded from
/// `config/sensors/map_server.toml` (see [`Default`]) or from an arbitrary
/// path via [`crate::config::load`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Deserialize)]
pub struct MapServerConfig {
    /// How often [`MapServer`] checks [`MAP_SELECTION_TOPIC_NAME`] for a
    /// change, in milliseconds.
    pub poll_interval_ms: u64,
}

impl Default for MapServerConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/sensors/map_server.toml"))
            .expect("config/sensors/map_server.toml must deserialize into MapServerConfig")
    }
}

/// Loads the map at `path` into a [`SelectedMap`] and its matching
/// [`StartState`], or prints a warning and returns `None` if it can't be
/// read - e.g. the selection points at a folder that no longer exists.
fn load(path: &Path) -> Option<(SelectedMap, StartState)> {
    match Map::load(path) {
        Ok(map) => {
            let selected = SelectedMap {
                path: Some(path.to_path_buf()),
                width_px: map.raster.width_px,
                height_px: map.raster.height_px,
                pixels: map.raster.to_bytes().into(),
                info: Some(map.info.clone()),
            };
            Some((selected, start_state(&map)))
        }
        Err(err) => {
            eprintln!("map_server: failed to load map {path:?}: {err}");
            None
        }
    }
}

/// What [`MapServer`] publishes on its drawing topic for `map`: the raster
/// itself - sharing `map`'s pixel buffer rather than copying it - and the
/// start/finish line on top. Empty when no map is loaded. Painted beneath
/// every other drawing, and never fades: it's only republished when the map
/// changes.
fn drawing(map: &SelectedMap) -> Drawing {
    let Some(info) = &map.info else {
        return Drawing::default().z_index(-100);
    };
    let line = &info.start_finish_line;
    Drawing::new(vec![
        Shape::Raster {
            origin_x_m: info.origin.x,
            origin_y_m: info.origin.y,
            resolution_m_per_px: info.resolution_m_per_px,
            width_px: map.width_px,
            height_px: map.height_px,
            pixels: map.pixels.clone(),
        },
        Shape::Polyline {
            points: vec![[line.a.x as f32, line.a.y as f32], [line.b.x as f32, line.b.y as f32]],
            closed: false,
            width_px: 2.0,
            color: Color::RED,
        },
    ])
    .z_index(-100)
}

/// The vehicle's initial state for `map`: the middle of its start/finish
/// line, heading along the track's direction of travel there - the tangent
/// from the race line's first point (the start/finish line is centered on
/// it, by construction of [`crate::environment::simulator::generate`])
/// toward its second - at rest. Falls back to a heading of `0.0` if the race
/// line has fewer than two points, which a generated map never does, but a
/// malformed one might.
fn start_state(map: &Map) -> StartState {
    let line = &map.info.start_finish_line;
    let x_m = (line.a.x + line.b.x) / 2.0;
    let y_m = (line.a.y + line.b.y) / 2.0;
    let heading_rad = match (map.race_line.first(), map.race_line.get(1)) {
        (Some(p0), Some(p1)) => (p1.y - p0.y).atan2(p1.x - p0.x),
        _ => 0.0,
    };
    StartState { x_m, y_m, heading_rad, speed_mps: 0.0 }
}

/// Claims [`MAP_TOPIC_NAME`] and [`START_STATE_TOPIC_NAME`] and republishes
/// both whenever [`MAP_SELECTION_TOPIC_NAME`]'s wanted path no longer matches
/// the currently published one - loading the new map from disk, or clearing
/// to their `Default`s if the selection was cleared.
pub struct MapServer {
    id: u8,
    name: String,
    config: MapServerConfig,
}

impl MapServer {
    pub fn new(name: impl Into<String>, config: MapServerConfig) -> Self {
        Self { id: 0, name: name.into(), config }
    }
}

impl Executor for MapServer {
    fn init(&mut self, id: u8) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_writer::<SelectedMap>(MAP_TOPIC_NAME, self.id, SelectedMap::default);
        captain.claim_writer::<StartState>(START_STATE_TOPIC_NAME, self.id, StartState::default);
        captain.claim_drawing(self.id);
    }

    fn run(&mut self, captain: &Captain) {
        let map_topic = captain.topic::<SelectedMap>(MAP_TOPIC_NAME);
        let start_state_topic = captain.topic::<StartState>(START_STATE_TOPIC_NAME);
        let selection_topic = captain.topic::<MapSelection>(MAP_SELECTION_TOPIC_NAME);
        let drawing_topic = captain.drawing(self.id);
        let mut ticker = Ticker::from_interval(Duration::from_millis(self.config.poll_interval_ms));

        // Which map is currently published, tracked here rather than read back
        // off the topic every poll. This executor is the `map` topic's only
        // writer, so it already knows; reading it back would clone the whole
        // `SelectedMap` - `MapInfo` and all - just to compare one path.
        // Seeded once from the topic so a mid-run restart of just this
        // executor (`Runner::switch_executor`) doesn't reload a map that's
        // already published.
        let mut published_path = map_topic.read().into_value().path;

        while captain.is_running(self.id) {
            let wanted = selection_topic.read();

            if wanted.path != published_path {
                let next = match &wanted.path {
                    None => Some((SelectedMap::default(), StartState::default())),
                    Some(path) => load(path),
                };
                // Left unchanged when `load` fails, so a selection pointing at
                // an unreadable map is retried on the next poll instead of
                // being recorded as published.
                if let Some((next_map, next_start_state)) = next {
                    published_path = next_map.path.clone();
                    drawing_topic
                        .write(self.id, drawing(&next_map))
                        .expect("lost writer authorization for the map's drawing topic");
                    map_topic
                        .write(self.id, next_map)
                        .expect("lost writer authorization for the map topic");
                    start_state_topic
                        .write(self.id, next_start_state)
                        .expect("lost writer authorization for the start_state topic");
                }
            }

            ticker.wait();
        }
    }

    fn name(&self) -> String {
        self.name.clone()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn fresh(&self) -> Box<dyn Executor> {
        Box::new(MapServer::new(self.name.clone(), self.config))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::environment::{ImageOrigin, MapInfo, MapSource, Raster, SpeedPoint, StartFinishLine, WorldPoint};
    use std::path::PathBuf;

    fn test_map(line: StartFinishLine, race_line: Vec<SpeedPoint>) -> Map {
        Map {
            folder: PathBuf::new(),
            info: MapInfo {
                resolution_m_per_px: 0.05,
                width_px: 10,
                height_px: 10,
                origin: ImageOrigin { x: 0.0, y: 0.0, theta_rad: 0.0 },
                start_finish_line: line,
                generated_at: String::new(),
                source: MapSource::Random,
                track_width_m: 2.0,
                point_spacing_m: 0.25,
                seed: 0,
            },
            raster: Raster::new(10, 10, vec![true; 100]),
            race_line,
        }
    }

    #[test]
    fn start_state_is_centered_on_the_start_finish_line() {
        let line =
            StartFinishLine { a: WorldPoint { x: 1.0, y: 3.0 }, b: WorldPoint { x: 1.0, y: -1.0 } };
        let race_line = vec![
            SpeedPoint { x: 1.0, y: 1.0, speed_mps: 0.0 },
            SpeedPoint { x: 2.0, y: 1.0, speed_mps: 0.0 },
        ];
        let map = test_map(line, race_line);

        let state = start_state(&map);

        assert_eq!(state.x_m, 1.0);
        assert_eq!(state.y_m, 1.0);
        assert_eq!(state.speed_mps, 0.0);
    }

    #[test]
    fn start_state_heading_is_tangent_to_the_track_direction() {
        let line = StartFinishLine { a: WorldPoint { x: 0.0, y: 1.0 }, b: WorldPoint { x: 0.0, y: -1.0 } };
        let race_line = vec![
            SpeedPoint { x: 0.0, y: 0.0, speed_mps: 0.0 },
            SpeedPoint { x: 0.0, y: 1.0, speed_mps: 0.0 },
        ];
        let map = test_map(line, race_line);

        let state = start_state(&map);

        assert!((state.heading_rad - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn start_state_falls_back_to_zero_heading_with_too_few_race_line_points() {
        let line = StartFinishLine { a: WorldPoint { x: 0.0, y: 1.0 }, b: WorldPoint { x: 0.0, y: -1.0 } };
        let map = test_map(line, vec![SpeedPoint { x: 0.0, y: 0.0, speed_mps: 0.0 }]);

        assert_eq!(start_state(&map).heading_rad, 0.0);
    }
}
