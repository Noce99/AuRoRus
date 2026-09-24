//! The `.debug` session file format: a hand-rolled, length-prefixed framing (only
//! each record's *payload* is bincode-encoded - the outer structure is fixed-width
//! so a reader always knows exactly how many bytes to expect next) written by
//! [`crate::core::debug_executor::DebugExecutor`] and read back by the
//! `replay_web_gui` binary.
//!
//! Deliberately tolerant of a hard kill mid-write: there is no trailing
//! footer/index a reader depends on, so [`DebugFileReader::open`] simply reads
//! records until EOF or the first incomplete one, and keeps whatever parsed
//! cleanly.
//!
//! Byte layout (all multi-byte integers little-endian):
//!
//! Header (29 bytes, written once):
//! - `magic: [u8; 4]` = [`MAGIC`]
//! - `format_version: u8` = [`FORMAT_VERSION`]
//! - `frequency_hz: f64`
//! - `session_start_unix_micros: u128`
//!
//! Then a stream of records, each starting with a 1-byte tag:
//! - `TopicDef` (`tag = 0x01`, once per topic name, the first time it's seen):
//!   `topic_id: u16` (assigned sequentially from 0, in first-seen order),
//!   `topic_name_len: u16` + UTF-8 name, `writer_name_len: u16` + UTF-8 writer
//!   executor name (captured once via `Captain::name_of`).
//! - `Sample` (`tag = 0x02`, one per recorded write): `topic_id: u16`,
//!   `timestamp_us: u64` (when the value was *written* to its topic, as elapsed
//!   microseconds since `session_start_unix_micros`),
//!   `payload_len: u32`, `payload: [u8; payload_len]` (bincode-encoded value, via
//!   `bincode::serde::encode_to_vec`/`decode_from_slice` with
//!   `bincode::config::standard()`).

use std::collections::HashMap;
use std::fs::{File, OpenOptions};
use std::io::{self, BufReader, BufWriter, Read, Seek, SeekFrom, Write};
use std::path::Path;
use std::time::{Instant, SystemTime, UNIX_EPOCH};
use time::OffsetDateTime;

/// Magic bytes at the start of every `.debug` file.
pub const MAGIC: [u8; 4] = *b"ADBG";
/// The only format version this module currently reads/writes.
pub const FORMAT_VERSION: u8 = 1;

const HEADER_LEN: u64 = 4 + 1 + 8 + 16;
const TAG_TOPIC_DEF: u8 = 0x01;
const TAG_SAMPLE: u8 = 0x02;
/// How often [`DebugFileWriter::maybe_flush`] flushes unconditionally, regardless
/// of [`FLUSH_EVERY_RECORDS`] - bounds how much a hard kill can lose even during a
/// lull with no new samples.
const FLUSH_EVERY: std::time::Duration = std::time::Duration::from_secs(1);
/// How many records [`DebugFileWriter::maybe_flush`] lets accumulate before
/// flushing early, even if [`FLUSH_EVERY`] hasn't elapsed yet.
const FLUSH_EVERY_RECORDS: u32 = 200;

/// One topic's identity, as recorded by a `TopicDef` record - `index == topic_id`
/// in [`DebugFileReader::topics`].
#[derive(Debug, Clone)]
pub struct TopicMeta {
    pub name: String,
    pub writer_name: String,
}

/// One recorded change, as written by a `Sample` record.
#[derive(Debug, Clone)]
pub struct Sample {
    pub topic_id: u16,
    pub timestamp_us: u64,
    pub payload: Vec<u8>,
}

fn io_err(message: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message.into())
}

fn write_u8(w: &mut impl Write, v: u8) -> io::Result<()> {
    w.write_all(&[v])
}
fn write_u16(w: &mut impl Write, v: u16) -> io::Result<()> {
    w.write_all(&v.to_le_bytes())
}
fn write_u32(w: &mut impl Write, v: u32) -> io::Result<()> {
    w.write_all(&v.to_le_bytes())
}
fn write_u64(w: &mut impl Write, v: u64) -> io::Result<()> {
    w.write_all(&v.to_le_bytes())
}
fn write_u128(w: &mut impl Write, v: u128) -> io::Result<()> {
    w.write_all(&v.to_le_bytes())
}
fn write_f64(w: &mut impl Write, v: f64) -> io::Result<()> {
    w.write_all(&v.to_le_bytes())
}
fn write_str16(w: &mut impl Write, s: &str) -> io::Result<()> {
    let bytes = s.as_bytes();
    write_u16(w, bytes.len().try_into().map_err(|_| io_err("string longer than 65535 bytes"))?)?;
    w.write_all(bytes)
}

/// Reads exactly `N` bytes, or fails - used during scanning to detect a truncated
/// tail (a hard-killed write mid-field), which the caller treats as "stop here".
fn read_exact_n<const N: usize>(r: &mut impl Read) -> io::Result<[u8; N]> {
    let mut buf = [0u8; N];
    r.read_exact(&mut buf)?;
    Ok(buf)
}
fn read_u8(r: &mut impl Read) -> io::Result<u8> {
    Ok(read_exact_n::<1>(r)?[0])
}
fn read_u16(r: &mut impl Read) -> io::Result<u16> {
    Ok(u16::from_le_bytes(read_exact_n(r)?))
}
fn read_u32(r: &mut impl Read) -> io::Result<u32> {
    Ok(u32::from_le_bytes(read_exact_n(r)?))
}
fn read_u64(r: &mut impl Read) -> io::Result<u64> {
    Ok(u64::from_le_bytes(read_exact_n(r)?))
}
fn read_u128(r: &mut impl Read) -> io::Result<u128> {
    Ok(u128::from_le_bytes(read_exact_n(r)?))
}
fn read_f64(r: &mut impl Read) -> io::Result<f64> {
    Ok(f64::from_le_bytes(read_exact_n(r)?))
}
fn read_str16(r: &mut impl Read) -> io::Result<String> {
    let len = read_u16(r)? as usize;
    let mut buf = vec![0u8; len];
    r.read_exact(&mut buf)?;
    String::from_utf8(buf).map_err(|err| io_err(format!("invalid UTF-8 in string field: {err}")))
}

/// The result of scanning a `.debug` file from the start: everything
/// [`DebugFileReader::open`] needs to return, plus `valid_len` (the byte offset
/// right after the last fully-parsed record) that [`DebugFileWriter::resume`]
/// needs to truncate away any trailing partial record before appending more.
struct Scanned {
    frequency_hz: f64,
    session_start_unix_micros: u128,
    topics: Vec<TopicMeta>,
    samples: Vec<Sample>,
    valid_len: u64,
}

/// Reads the header, then records one at a time, stopping cleanly at EOF or the
/// first record that can't be fully read (a truncated tail) rather than erroring.
/// A malformed header (bad magic/version, or truncated) is a hard error - unlike a
/// truncated record stream, there is no way to recover a usable file without one.
fn scan(path: &Path) -> io::Result<Scanned> {
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);

    let mut magic = [0u8; 4];
    reader.read_exact(&mut magic)?;
    if magic != MAGIC {
        return Err(io_err(format!("not a debug file: bad magic {magic:?}")));
    }
    let format_version = read_u8(&mut reader)?;
    if format_version != FORMAT_VERSION {
        return Err(io_err(format!(
            "unsupported debug file format version {format_version} (expected {FORMAT_VERSION})"
        )));
    }
    let frequency_hz = read_f64(&mut reader)?;
    let session_start_unix_micros = read_u128(&mut reader)?;

    let mut topics = Vec::new();
    let mut samples = Vec::new();
    let mut valid_len = HEADER_LEN;

    loop {
        let mut tag_buf = [0u8; 1];
        match reader.read(&mut tag_buf) {
            Ok(0) => break, // clean EOF right at a record boundary
            Err(_) => break,
            Ok(_) => {}
        }

        let record: io::Result<u64> = (|| {
            match tag_buf[0] {
                TAG_TOPIC_DEF => {
                    let topic_id = read_u16(&mut reader)?;
                    let name = read_str16(&mut reader)?;
                    let writer_name = read_str16(&mut reader)?;
                    if topic_id as usize != topics.len() {
                        return Err(io_err("topic ids are not sequential in this debug file"));
                    }
                    let len = 1 + 2 + 2 + name.len() + 2 + writer_name.len();
                    topics.push(TopicMeta { name, writer_name });
                    Ok(len as u64)
                }
                TAG_SAMPLE => {
                    let topic_id = read_u16(&mut reader)?;
                    let timestamp_us = read_u64(&mut reader)?;
                    let payload_len = read_u32(&mut reader)?;
                    let mut payload = vec![0u8; payload_len as usize];
                    reader.read_exact(&mut payload)?;
                    let len = 1 + 2 + 8 + 4 + payload.len();
                    samples.push(Sample { topic_id, timestamp_us, payload });
                    Ok(len as u64)
                }
                other => Err(io_err(format!("unknown record tag {other:#04x}"))),
            }
        })();

        match record {
            Ok(len) => valid_len += len,
            Err(_) => break, // truncated or corrupt - stop, keep everything parsed so far
        }
    }

    Ok(Scanned { frequency_hz, session_start_unix_micros, topics, samples, valid_len })
}

/// Writes a fresh `.debug` session: the header immediately, then `TopicDef`/`Sample`
/// records as [`topic_id`](Self::topic_id)/[`write_sample`](Self::write_sample) are
/// called.
pub struct DebugFileWriter {
    file: BufWriter<File>,
    topic_ids: HashMap<String, u16>,
    next_topic_id: u16,
    session_start_unix_micros: u128,
    records_since_flush: u32,
    last_flush: Instant,
}

impl DebugFileWriter {
    /// Starts a brand-new session file at `path` (creating or truncating it),
    /// writing the header immediately.
    pub fn create(path: &Path, frequency_hz: f64) -> io::Result<Self> {
        if let Some(parent) = path.parent()
            && !parent.as_os_str().is_empty()
        {
            std::fs::create_dir_all(parent)?;
        }
        let session_start_unix_micros =
            SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default().as_micros();
        let mut file = BufWriter::new(File::create(path)?);
        file.write_all(&MAGIC)?;
        write_u8(&mut file, FORMAT_VERSION)?;
        write_f64(&mut file, frequency_hz)?;
        write_u128(&mut file, session_start_unix_micros)?;
        file.flush()?;
        Ok(Self {
            file,
            topic_ids: HashMap::new(),
            next_topic_id: 0,
            session_start_unix_micros,
            records_since_flush: 0,
            last_flush: Instant::now(),
        })
    }

    /// Reopens an existing session file for appending - e.g. after a
    /// [`crate::Runner`] restart mid-recording. Re-derives `session_start_unix_micros`
    /// from the file's own header (`frequency_hz` is assumed to already match, since
    /// a restart reuses the same [`crate::Runner::debug_mode`] call) and replays
    /// every `TopicDef` already present so a topic already defined pre-restart is
    /// never given a second, conflicting id. Truncates away any trailing partial
    /// record left by an abrupt prior shutdown before appending more.
    pub fn resume(path: &Path) -> io::Result<Self> {
        let scanned = scan(path)?;

        let mut topic_ids = HashMap::with_capacity(scanned.topics.len());
        for (id, meta) in scanned.topics.iter().enumerate() {
            topic_ids.insert(meta.name.clone(), id as u16);
        }
        let next_topic_id = scanned.topics.len() as u16;

        let file = OpenOptions::new().write(true).open(path)?;
        file.set_len(scanned.valid_len)?;
        let mut file = file;
        file.seek(SeekFrom::Start(scanned.valid_len))?;

        Ok(Self {
            file: BufWriter::new(file),
            topic_ids,
            next_topic_id,
            session_start_unix_micros: scanned.session_start_unix_micros,
            records_since_flush: 0,
            last_flush: Instant::now(),
        })
    }

    pub fn session_start_unix_micros(&self) -> u128 {
        self.session_start_unix_micros
    }

    /// Returns `name`'s topic id, writing a `TopicDef` record the first time this
    /// name is seen (by this writer instance - which, after [`resume`](Self::resume),
    /// already knows about every name the file had before).
    pub fn topic_id(&mut self, name: &str, writer_name: &str) -> io::Result<u16> {
        if let Some(&id) = self.topic_ids.get(name) {
            return Ok(id);
        }
        let id = self.next_topic_id;
        self.next_topic_id =
            self.next_topic_id.checked_add(1).ok_or_else(|| io_err("exceeded u16::MAX topics"))?;

        write_u8(&mut self.file, TAG_TOPIC_DEF)?;
        write_u16(&mut self.file, id)?;
        write_str16(&mut self.file, name)?;
        write_str16(&mut self.file, writer_name)?;

        self.topic_ids.insert(name.to_string(), id);
        self.records_since_flush += 1;
        Ok(id)
    }

    pub fn write_sample(&mut self, topic_id: u16, timestamp_us: u64, payload: &[u8]) -> io::Result<()> {
        write_u8(&mut self.file, TAG_SAMPLE)?;
        write_u16(&mut self.file, topic_id)?;
        write_u64(&mut self.file, timestamp_us)?;
        write_u32(&mut self.file, payload.len().try_into().map_err(|_| io_err("payload larger than 4 GiB"))?)?;
        self.file.write_all(payload)?;
        self.records_since_flush += 1;
        Ok(())
    }

    /// Flushes if [`FLUSH_EVERY_RECORDS`] records or [`FLUSH_EVERY`] have passed
    /// since the last flush, whichever comes first - cheap to call once per tick;
    /// a no-op most ticks. Bounds how much a hard kill (no clean Ctrl+C) can lose.
    pub fn maybe_flush(&mut self) -> io::Result<()> {
        if self.records_since_flush >= FLUSH_EVERY_RECORDS || self.last_flush.elapsed() >= FLUSH_EVERY {
            self.flush()?;
        }
        Ok(())
    }

    pub fn flush(&mut self) -> io::Result<()> {
        self.file.flush()?;
        self.records_since_flush = 0;
        self.last_flush = Instant::now();
        Ok(())
    }
}

/// A fully-decoded `.debug` session, read into memory by [`DebugFileReader::open`].
pub struct DebugFileReader {
    pub frequency_hz: f64,
    pub session_start_unix_micros: u128,
    /// `index == topic_id`.
    pub topics: Vec<TopicMeta>,
    /// In file order. Each topic's own samples are in timestamp order (one writer
    /// per topic, stamped at write time); samples of *different* topics recorded in
    /// the same tick may be slightly out of order relative to each other.
    pub samples: Vec<Sample>,
}

impl DebugFileReader {
    /// Reads the whole file into memory. Given the expected scale (minutes to a
    /// couple of hours, up to a few hundred Hz of *changed-value-only* records -
    /// at most a few hundred thousand small records), this is simpler and plenty
    /// fast compared to building an on-disk seek index.
    pub fn open(path: &Path) -> io::Result<Self> {
        let scanned = scan(path)?;
        Ok(Self {
            frequency_hz: scanned.frequency_hz,
            session_start_unix_micros: scanned.session_start_unix_micros,
            topics: scanned.topics,
            samples: scanned.samples,
        })
    }

    /// Default filename for a freshly started session: `YYYY_MM_DD__HH_mm_ss.debug`,
    /// local time (falling back to UTC on the same `OffsetDateTime::now_local`
    /// failure mode `crate::core::log` already documents).
    pub fn generated_filename() -> String {
        let now = OffsetDateTime::now_local().unwrap_or_else(|_| OffsetDateTime::now_utc());
        format!(
            "{:04}_{:02}_{:02}__{:02}_{:02}_{:02}.debug",
            now.year(),
            u8::from(now.month()),
            now.day(),
            now.hour(),
            now.minute(),
            now.second(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_path(name: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("aurorus_debug_format_tests_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        dir.join(name)
    }

    #[test]
    fn round_trips_topics_and_samples() {
        let path = temp_path("round_trip.debug");
        let mut writer = DebugFileWriter::create(&path, 100.0).unwrap();
        let id_a = writer.topic_id("vehicle_status", "SimulatedVehicle").unwrap();
        let id_b = writer.topic_id("map", "MapServer").unwrap();
        assert_eq!(writer.topic_id("vehicle_status", "SimulatedVehicle").unwrap(), id_a);
        writer.write_sample(id_a, 0, &[1, 2, 3]).unwrap();
        writer.write_sample(id_b, 10_000, &[4, 5]).unwrap();
        writer.write_sample(id_a, 20_000, &[9]).unwrap();
        writer.flush().unwrap();

        let reader = DebugFileReader::open(&path).unwrap();
        assert_eq!(reader.frequency_hz, 100.0);
        assert_eq!(reader.topics.len(), 2);
        assert_eq!(reader.topics[0].name, "vehicle_status");
        assert_eq!(reader.topics[0].writer_name, "SimulatedVehicle");
        assert_eq!(reader.topics[1].name, "map");
        assert_eq!(reader.samples.len(), 3);
        assert_eq!(reader.samples[0].payload, vec![1, 2, 3]);
        assert_eq!(reader.samples[1].timestamp_us, 10_000);
        assert_eq!(reader.samples[2].payload, vec![9]);
    }

    #[test]
    fn open_tolerates_a_truncated_tail() {
        let path = temp_path("truncated.debug");
        let mut writer = DebugFileWriter::create(&path, 50.0).unwrap();
        let id = writer.topic_id("x", "Writer").unwrap();
        writer.write_sample(id, 0, &[1, 2, 3, 4]).unwrap();
        writer.flush().unwrap();
        drop(writer);

        // Simulate a hard kill mid-write: chop off the last few bytes of the
        // second (never-written) sample's would-be payload by appending a
        // dangling, incomplete record tag + partial fields.
        {
            let mut file = OpenOptions::new().append(true).open(&path).unwrap();
            file.write_all(&[TAG_SAMPLE]).unwrap();
            write_u16(&mut file, id).unwrap();
            // stop here - no timestamp/payload_len/payload bytes at all.
        }

        let reader = DebugFileReader::open(&path).unwrap();
        assert_eq!(reader.samples.len(), 1);
        assert_eq!(reader.samples[0].payload, vec![1, 2, 3, 4]);
    }

    #[test]
    fn resume_preserves_topic_ids_and_appends_after_the_valid_prefix() {
        let path = temp_path("resume.debug");
        let mut writer = DebugFileWriter::create(&path, 100.0).unwrap();
        let id_a = writer.topic_id("a", "W1").unwrap();
        writer.write_sample(id_a, 0, &[1]).unwrap();
        writer.flush().unwrap();
        drop(writer);

        let mut resumed = DebugFileWriter::resume(&path).unwrap();
        // Re-claiming an already-known topic must return the same id, not define
        // a second, conflicting one.
        assert_eq!(resumed.topic_id("a", "W1").unwrap(), id_a);
        let id_b = resumed.topic_id("b", "W2").unwrap();
        assert_ne!(id_b, id_a);
        resumed.write_sample(id_a, 5_000, &[2]).unwrap();
        resumed.write_sample(id_b, 6_000, &[3]).unwrap();
        resumed.flush().unwrap();

        let reader = DebugFileReader::open(&path).unwrap();
        assert_eq!(reader.topics.len(), 2);
        assert_eq!(reader.samples.len(), 3);
        assert_eq!(reader.samples[0].payload, vec![1]);
        assert_eq!(reader.samples[1].timestamp_us, 5_000);
        assert_eq!(reader.samples[2].timestamp_us, 6_000);
    }
}
