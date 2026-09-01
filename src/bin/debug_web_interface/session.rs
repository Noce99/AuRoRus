//! [`Session`]: a `.debug` file, fully decoded into memory for playback -
//! grouped by executor/topic (for the timeline), plus the static map/vehicle
//! model and the full vehicle status timeline (for the main canvas). See
//! `aurorus::debug_format` for the file layout this decodes.

use aurorus::debug_format::DebugFileReader;
use aurorus::environment::MapInfo;
use aurorus::topics::{
    MAP_TOPIC_NAME, SelectedMap, VEHICLE_MODEL_STATUS_TOPIC_NAME, VEHICLE_STATUS_TOPIC_NAME, VehicleModelKind,
    VehicleModelStatus, VehicleStatus,
};
use std::collections::HashMap;
use std::io;
use std::path::Path;

/// Number of distinct colors the frontend's palette defines - kept in sync
/// with `PALETTE` in `static/timeline.js` by hand, since both sides only need
/// to agree on the modulus, not the actual colors.
const PALETTE_SIZE: usize = 12;

/// One topic, as grouped under its writer executor - what the timeline draws
/// one colored tick-row-worth of ticks for.
pub struct TopicEntry {
    pub name: String,
    pub color_index: usize,
    /// Every recorded change's elapsed-microseconds timestamp, in order.
    pub timestamps_us: Vec<u64>,
}

/// One executor and every topic it wrote - one row in the timeline.
pub struct ExecutorEntry {
    pub name: String,
    pub topics: Vec<TopicEntry>,
}

/// The static map shown throughout playback - decoded from the *last*
/// recorded `map` sample (see the module doc comment on why "last" rather
/// than time-varying).
pub struct DecodedMap {
    pub name: Option<String>,
    pub width_px: u32,
    pub height_px: u32,
    pub pixels: Vec<u8>,
    pub info: Option<MapInfo>,
}

/// A `.debug` file, fully decoded into memory. See [`Session::load`].
pub struct Session {
    pub frequency_hz: f64,
    /// The latest timestamp of any recorded sample - the playback duration.
    pub duration_us: u64,
    /// One entry per executor that actually wrote at least one topic -
    /// [`aurorus::core::debug_executor::DebugExecutor`] never appears here,
    /// since it never claims a topic's writer slot.
    pub executors: Vec<ExecutorEntry>,
    pub map: Option<DecodedMap>,
    pub vehicle_model_kind: Option<VehicleModelKind>,
    /// Every recorded `vehicle_status` sample, decoded and time-ordered - the
    /// main canvas's hold-last-value source of truth at any playback time.
    pub vehicle_status_timeline: Vec<(u64, VehicleStatus)>,
}

/// A simple, stable hash (FNV-1a) of a topic name into a palette slot - all
/// that matters is that two different names usually land on different
/// colors, and the same name always lands on the same one.
fn color_index_for(name: &str) -> usize {
    let mut hash: u64 = 0xcbf29ce484222325;
    for byte in name.as_bytes() {
        hash ^= *byte as u64;
        hash = hash.wrapping_mul(0x100000001b3);
    }
    (hash % PALETTE_SIZE as u64) as usize
}

fn decode<T: serde::de::DeserializeOwned>(payload: &[u8]) -> Option<T> {
    bincode::serde::decode_from_slice(payload, bincode::config::standard()).ok().map(|(value, _)| value)
}

impl Session {
    /// Reads and fully decodes `path`. See `aurorus::debug_format::DebugFileReader::open`
    /// for why a full in-memory decode (rather than an on-disk index) is the
    /// right tradeoff at the expected session scale.
    pub fn load(path: &Path) -> io::Result<Self> {
        let reader = DebugFileReader::open(path)?;

        let mut executor_order: Vec<String> = Vec::new();
        let mut executor_topics: HashMap<String, Vec<TopicEntry>> = HashMap::new();
        for (topic_id, meta) in reader.topics.iter().enumerate() {
            let timestamps_us: Vec<u64> = reader
                .samples
                .iter()
                .filter(|sample| sample.topic_id as usize == topic_id)
                .map(|sample| sample.timestamp_us)
                .collect();
            let entry = TopicEntry { name: meta.name.clone(), color_index: color_index_for(&meta.name), timestamps_us };
            executor_topics.entry(meta.writer_name.clone()).or_insert_with(|| {
                executor_order.push(meta.writer_name.clone());
                Vec::new()
            });
            executor_topics.get_mut(&meta.writer_name).unwrap().push(entry);
        }
        let executors = executor_order
            .into_iter()
            .map(|name| {
                let topics = executor_topics.remove(&name).unwrap_or_default();
                ExecutorEntry { name, topics }
            })
            .collect();

        let duration_us = reader.samples.iter().map(|sample| sample.timestamp_us).max().unwrap_or(0);

        let topic_id_of = |name: &str| reader.topics.iter().position(|topic| topic.name == name);

        let map = topic_id_of(MAP_TOPIC_NAME)
            .and_then(|id| reader.samples.iter().rev().find(|sample| sample.topic_id as usize == id))
            .and_then(|sample| decode::<SelectedMap>(&sample.payload))
            .map(|selected| DecodedMap {
                name: selected.path.as_ref().and_then(|p| p.file_name()).and_then(|n| n.to_str()).map(str::to_string),
                width_px: selected.width_px,
                height_px: selected.height_px,
                pixels: selected.pixels,
                info: selected.info,
            });

        let vehicle_model_kind = topic_id_of(VEHICLE_MODEL_STATUS_TOPIC_NAME)
            .and_then(|id| reader.samples.iter().rev().find(|sample| sample.topic_id as usize == id))
            .and_then(|sample| decode::<VehicleModelStatus>(&sample.payload))
            .map(|status| status.kind);

        let vehicle_status_timeline = topic_id_of(VEHICLE_STATUS_TOPIC_NAME)
            .map(|id| {
                reader
                    .samples
                    .iter()
                    .filter(|sample| sample.topic_id as usize == id)
                    .filter_map(|sample| decode::<VehicleStatus>(&sample.payload).map(|value| (sample.timestamp_us, value)))
                    .collect()
            })
            .unwrap_or_default();

        Ok(Session {
            frequency_hz: reader.frequency_hz,
            duration_us,
            executors,
            map,
            vehicle_model_kind,
            vehicle_status_timeline,
        })
    }

    /// The vehicle's status as of `t_us`, holding the last value recorded at
    /// or before that time - matching how a live `RwLockTopic` read behaves.
    /// `None` before the first recorded sample.
    pub fn vehicle_status_at(&self, t_us: u64) -> Option<VehicleStatus> {
        match self.vehicle_status_timeline.binary_search_by_key(&t_us, |(t, _)| *t) {
            Ok(index) => Some(self.vehicle_status_timeline[index].1),
            Err(0) => None,
            Err(index) => Some(self.vehicle_status_timeline[index - 1].1),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use aurorus::debug_format::DebugFileWriter;
    use aurorus::topics::MAP_TOPIC_NAME;

    fn temp_path(name: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("aurorus_session_tests_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        dir.join(name)
    }

    fn encode<T: serde::Serialize>(value: &T) -> Vec<u8> {
        bincode::serde::encode_to_vec(value, bincode::config::standard()).unwrap()
    }

    #[test]
    fn groups_topics_by_writer_and_decodes_the_static_and_timeline_views() {
        let path = temp_path("session.debug");
        let mut writer = DebugFileWriter::create(&path, 100.0).unwrap();

        let map_id = writer.topic_id(MAP_TOPIC_NAME, "MapServer").unwrap();
        let model_id = writer.topic_id(VEHICLE_MODEL_STATUS_TOPIC_NAME, "SimulatedVehicle").unwrap();
        let status_id = writer.topic_id(VEHICLE_STATUS_TOPIC_NAME, "SimulatedVehicle").unwrap();

        let map_a = SelectedMap {
            path: Some("maps/track_a".into()),
            width_px: 2,
            height_px: 2,
            pixels: vec![255, 0, 0, 255],
            info: None,
        };
        let map_b = SelectedMap { path: Some("maps/track_b".into()), width_px: 1, height_px: 1, pixels: vec![0], info: None };
        writer.write_sample(map_id, 0, &encode(&map_a)).unwrap();
        writer.write_sample(map_id, 5_000, &encode(&map_b)).unwrap();

        writer
            .write_sample(model_id, 0, &encode(&VehicleModelStatus { kind: VehicleModelKind::Bicycle }))
            .unwrap();

        let status_1 = VehicleStatus { x_m: 1.0, y_m: 0.0, heading_rad: 0.0, speed_mps: 0.5 };
        let status_2 = VehicleStatus { x_m: 2.0, y_m: 0.0, heading_rad: 0.1, speed_mps: 1.0 };
        writer.write_sample(status_id, 1_000, &encode(&status_1)).unwrap();
        writer.write_sample(status_id, 2_000, &encode(&status_2)).unwrap();
        writer.flush().unwrap();

        let session = Session::load(&path).unwrap();
        assert_eq!(session.duration_us, 5_000);
        assert_eq!(session.executors.len(), 2);

        let map_server = session.executors.iter().find(|e| e.name == "MapServer").unwrap();
        assert_eq!(map_server.topics.len(), 1);
        assert_eq!(map_server.topics[0].timestamps_us, vec![0, 5_000]);

        let vehicle = session.executors.iter().find(|e| e.name == "SimulatedVehicle").unwrap();
        assert_eq!(vehicle.topics.len(), 2);

        // Static map is the LAST recorded sample, not the first.
        assert_eq!(session.map.as_ref().unwrap().name, Some("track_b".to_string()));

        assert_eq!(session.vehicle_model_kind, Some(VehicleModelKind::Bicycle));

        assert_eq!(session.vehicle_status_at(500), None);
        assert_eq!(session.vehicle_status_at(1_000), Some(status_1));
        assert_eq!(session.vehicle_status_at(1_500), Some(status_1));
        assert_eq!(session.vehicle_status_at(2_000), Some(status_2));
        assert_eq!(session.vehicle_status_at(9_999), Some(status_2));
    }
}
