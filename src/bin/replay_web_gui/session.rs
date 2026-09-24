//! [`Session`]: a `.debug` file, loaded into memory for playback - every
//! topic's change timestamps grouped by executor (for the timeline), and
//! every drawing topic's recorded [`Drawing`]s (for the main canvas). See
//! `aurorus::debug_format` for the file layout this reads.
//!
//! Nothing here knows about any particular topic: the canvas shows whatever
//! the recorded executors drew, exactly like `web_gui` does live.

use aurorus::debug_format::{DebugFileReader, Sample};
use aurorus::topics::{DRAW_TOPIC_PREFIX, Drawing};
use std::collections::HashMap;
use std::io;
use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};

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

/// Every recorded version of one drawing topic, in time order - decoded
/// only when asked for (see [`Session::drawing`]): a recording holds many
/// thousands of them, e.g. a LIDAR's hit points at every scan.
pub struct DrawingTrack {
    pub topic: String,
    pub writer: String,
    /// `(timestamp_us, index into Session::samples)` of every recorded
    /// version.
    versions: Vec<(u64, usize)>,
}

/// Which recorded version of a [`DrawingTrack`] is shown at some playback
/// time - the drawing protocol's `write_count`, counted from `1`, with `0`
/// meaning nothing was drawn yet at that time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DrawingVersion {
    pub write_count: u64,
    /// When that version was written, or `None` for `write_count == 0`.
    pub written_at_us: Option<u64>,
}

/// A `.debug` file, loaded into memory. See [`Session::load`].
pub struct Session {
    pub frequency_hz: f64,
    /// The latest timestamp of any recorded sample - the playback duration.
    pub duration_us: u64,
    /// One entry per executor that actually wrote at least one topic -
    /// the debug recorder itself never appears here, since it never claims
    /// a topic's writer slot.
    pub executors: Vec<ExecutorEntry>,
    /// Every recorded drawing topic, in topic-name order.
    pub drawings: Vec<DrawingTrack>,
    /// Identifies this loaded session in the drawing protocol (see
    /// `aurorus::web::draw`), so a client's cached drawings from another
    /// run of this server - possibly of another file - are never mistaken
    /// for this one's.
    pub epoch: u64,
    samples: Vec<Sample>,
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

impl Session {
    /// Reads `path`. See `aurorus::debug_format::DebugFileReader::open` for
    /// why reading the whole file into memory (rather than an on-disk index)
    /// is the right tradeoff at the expected session scale.
    pub fn load(path: &Path) -> io::Result<Self> {
        let reader = DebugFileReader::open(path)?;

        let mut executor_order: Vec<String> = Vec::new();
        let mut executor_topics: HashMap<String, Vec<TopicEntry>> = HashMap::new();
        let mut drawings = Vec::new();
        for (topic_id, meta) in reader.topics.iter().enumerate() {
            let versions: Vec<(u64, usize)> = reader
                .samples
                .iter()
                .enumerate()
                .filter(|(_, sample)| sample.topic_id as usize == topic_id)
                .map(|(index, sample)| (sample.timestamp_us, index))
                .collect();

            if meta.name.starts_with(DRAW_TOPIC_PREFIX) {
                drawings.push(DrawingTrack {
                    topic: meta.name.clone(),
                    writer: meta.writer_name.clone(),
                    versions: versions.clone(),
                });
            }

            let entry = TopicEntry {
                name: meta.name.clone(),
                color_index: color_index_for(&meta.name),
                timestamps_us: versions.iter().map(|(t_us, _)| *t_us).collect(),
            };
            executor_topics
                .entry(meta.writer_name.clone())
                .or_insert_with(|| {
                    executor_order.push(meta.writer_name.clone());
                    Vec::new()
                });
            executor_topics
                .get_mut(&meta.writer_name)
                .unwrap()
                .push(entry);
        }
        let executors = executor_order
            .into_iter()
            .map(|name| {
                let topics = executor_topics.remove(&name).unwrap_or_default();
                ExecutorEntry { name, topics }
            })
            .collect();
        drawings.sort_by(|a, b| a.topic.cmp(&b.topic));

        let duration_us = reader
            .samples
            .iter()
            .map(|sample| sample.timestamp_us)
            .max()
            .unwrap_or(0);
        let epoch = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_micros() as u64;

        Ok(Session {
            frequency_hz: reader.frequency_hz,
            duration_us,
            executors,
            drawings,
            epoch,
            samples: reader.samples,
        })
    }

    /// Which version of `track` is shown at `t_us`: the last one written at
    /// or before it - matching how a live topic read behaves.
    pub fn version_at(&self, track: &DrawingTrack, t_us: u64) -> DrawingVersion {
        let count = track
            .versions
            .partition_point(|(written_at_us, _)| *written_at_us <= t_us);
        DrawingVersion {
            write_count: count as u64,
            written_at_us: count.checked_sub(1).map(|i| track.versions[i].0),
        }
    }

    /// Version `write_count` of `track`, decoded. An empty drawing for
    /// `write_count == 0` (nothing drawn yet) and for a sample that doesn't
    /// decode as a [`Drawing`] - a recording from before a change to the
    /// shape vocabulary, say - so one bad sample can't break playback.
    /// `None` only if `track` has no such version.
    pub fn drawing(&self, track: &DrawingTrack, write_count: u64) -> Option<Drawing> {
        if write_count == 0 {
            return Some(Drawing::default());
        }
        let (_, sample_index) = track.versions.get(usize::try_from(write_count - 1).ok()?)?;
        let payload = &self.samples[*sample_index].payload;
        let decoded =
            bincode::serde::decode_from_slice::<Drawing, _>(payload, bincode::config::standard());
        Some(decoded.map(|(drawing, _)| drawing).unwrap_or_default())
    }

    pub fn track(&self, topic: &str) -> Option<&DrawingTrack> {
        self.drawings.iter().find(|track| track.topic == topic)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use aurorus::debug_format::DebugFileWriter;
    use aurorus::topics::{Color, Shape};

    fn temp_path(name: &str) -> std::path::PathBuf {
        let dir =
            std::env::temp_dir().join(format!("aurorus_session_tests_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        dir.join(name)
    }

    fn encode<T: serde::Serialize>(value: &T) -> Vec<u8> {
        bincode::serde::encode_to_vec(value, bincode::config::standard()).unwrap()
    }

    fn dot_at(x: f32) -> Drawing {
        Drawing::new(vec![Shape::Points {
            points: vec![[x, 0.0]],
            radius_px: 2.0,
            color: Color::RED,
        }])
    }

    #[test]
    fn groups_topics_by_writer_and_replays_drawings_holding_the_last_value() {
        let path = temp_path("session.debug");
        let mut writer = DebugFileWriter::create(&path, 100.0).unwrap();

        let status_id = writer
            .topic_id("vehicle_status", "SimulatedVehicle")
            .unwrap();
        let draw_id = writer
            .topic_id("draw/SimulatedVehicle", "SimulatedVehicle")
            .unwrap();
        let map_id = writer.topic_id("draw/MapServer", "MapServer").unwrap();

        writer.write_sample(status_id, 500, &[1, 2, 3]).unwrap();
        writer
            .write_sample(draw_id, 1_000, &encode(&dot_at(1.0)))
            .unwrap();
        writer
            .write_sample(draw_id, 2_000, &encode(&dot_at(2.0)))
            .unwrap();
        writer
            .write_sample(map_id, 5_000, b"not a drawing")
            .unwrap();
        writer.flush().unwrap();

        let session = Session::load(&path).unwrap();
        assert_eq!(session.duration_us, 5_000);
        assert_eq!(session.executors.len(), 2);
        let vehicle = session
            .executors
            .iter()
            .find(|e| e.name == "SimulatedVehicle")
            .unwrap();
        assert_eq!(vehicle.topics.len(), 2);
        assert_eq!(vehicle.topics[1].timestamps_us, vec![1_000, 2_000]);

        // Only drawing topics are drawing tracks, in topic-name order.
        let topics: Vec<&str> = session
            .drawings
            .iter()
            .map(|track| track.topic.as_str())
            .collect();
        assert_eq!(topics, ["draw/MapServer", "draw/SimulatedVehicle"]);

        let track = session.track("draw/SimulatedVehicle").unwrap();
        assert_eq!(
            session.version_at(track, 999),
            DrawingVersion {
                write_count: 0,
                written_at_us: None
            }
        );
        assert_eq!(
            session.version_at(track, 1_000),
            DrawingVersion {
                write_count: 1,
                written_at_us: Some(1_000)
            }
        );
        assert_eq!(
            session.version_at(track, 1_999),
            DrawingVersion {
                write_count: 1,
                written_at_us: Some(1_000)
            }
        );
        assert_eq!(
            session.version_at(track, 9_999),
            DrawingVersion {
                write_count: 2,
                written_at_us: Some(2_000)
            }
        );

        assert_eq!(session.drawing(track, 0), Some(Drawing::default()));
        assert_eq!(session.drawing(track, 2), Some(dot_at(2.0)));
        assert_eq!(session.drawing(track, 3), None);

        // A sample that doesn't decode plays back as an empty drawing.
        let map = session.track("draw/MapServer").unwrap();
        assert_eq!(session.drawing(map, 1), Some(Drawing::default()));
    }
}
