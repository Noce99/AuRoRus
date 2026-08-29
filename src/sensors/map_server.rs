//! [`MapServer`]: watches [`MAP_SELECTION_TOPIC_NAME`] for the wanted map
//! folder and keeps [`MAP_TOPIC_NAME`] in sync with it, loading from disk
//! only when the selection actually changes.

use crate::environment::Map;
use crate::topics::{MAP_SELECTION_TOPIC_NAME, MAP_TOPIC_NAME, MapSelection, SelectedMap};
use crate::{Captain, Executor};
use std::any::Any;
use std::path::Path;
use std::thread;
use std::time::Duration;

/// How often [`MapServer`] checks [`MAP_SELECTION_TOPIC_NAME`] for a change.
const POLL_INTERVAL: Duration = Duration::from_millis(200);

/// Loads the map at `path` into a [`SelectedMap`], or prints a warning and
/// returns `None` if it can't be read - e.g. the selection points at a
/// folder that no longer exists.
fn load(path: &Path) -> Option<SelectedMap> {
    match Map::load(path) {
        Ok(map) => Some(SelectedMap {
            path: Some(path.to_path_buf()),
            width_px: map.raster.width_px,
            height_px: map.raster.height_px,
            pixels: map.raster.to_bytes(),
        }),
        Err(err) => {
            eprintln!("map_server: failed to load map {path:?}: {err}");
            None
        }
    }
}

/// Claims [`MAP_TOPIC_NAME`] and republishes it whenever
/// [`MAP_SELECTION_TOPIC_NAME`]'s wanted path no longer matches the
/// currently published one - loading the new map from disk, or clearing to
/// [`SelectedMap::default`] if the selection was cleared.
pub struct MapServer {
    id: u8,
    name: String,
}

impl MapServer {
    pub fn new(name: impl Into<String>) -> Self {
        Self { id: 0, name: name.into() }
    }
}

impl Executor for MapServer {
    fn init(&mut self, id: u8) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_writer::<SelectedMap>(MAP_TOPIC_NAME, self.id, SelectedMap::default);
    }

    fn run(&mut self, captain: &Captain) {
        let map_topic = captain.topic::<SelectedMap>(MAP_TOPIC_NAME);
        let selection_topic = captain.topic::<MapSelection>(MAP_SELECTION_TOPIC_NAME);

        while captain.is_running(self.id) {
            let wanted = selection_topic.read();
            let current = map_topic.read();

            if wanted.path != current.path {
                let next = match &wanted.path {
                    None => Some(SelectedMap::default()),
                    Some(path) => load(path),
                };
                if let Some(next) = next {
                    map_topic
                        .write(self.id, next)
                        .expect("lost writer authorization for the map topic");
                }
            }

            thread::sleep(POLL_INTERVAL);
        }
    }

    fn name(&self) -> String {
        self.name.clone()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}
