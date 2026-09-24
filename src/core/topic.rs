//! A [`RwLockTopic`] is a single, typed slot of shared state with exactly one
//! authorized writer and any number of readers. Every write is stamped by the
//! topic itself (see [`WriteMeta`]), so readers always know how old - and how
//! often rewritten - the value they got is.

use std::ops::Deref;
use std::sync::{OnceLock, RwLock};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// Errors returned by [`RwLockTopic::set_writer`] and [`RwLockTopic::write`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TopicError {
    /// [`RwLockTopic::set_writer`] was called on a topic that already has a
    /// *different* writer. Reclaiming the slot with the same `executor_id` that
    /// already holds it is not an error - see [`RwLockTopic::set_writer`].
    WriterAlreadySet,
    /// [`RwLockTopic::write`] was called with an executor id that isn't the
    /// topic's registered writer (including the case where no writer has been set
    /// yet).
    UnauthorizedWriter,
}

/// Bookkeeping [`RwLockTopic::write`] attaches to every value it publishes -
/// stamped by the topic itself, never by the writing executor, so every topic
/// carries it and no writer can forget or fake it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct WriteMeta {
    /// How many times the topic has been written since it was registered (i.e.
    /// since the current [`crate::Captain`] was built - a
    /// [`crate::Captain::request_restart`] starts it over). `0` means the topic
    /// still holds the `initial` seed it was registered with.
    pub write_count: u64,
    /// When the latest write happened, on the monotonic clock - what
    /// [`Stamped::age`] is measured against, so it's immune to wall-clock jumps
    /// (e.g. NTP syncing after boot). `None` while `write_count` is `0`.
    pub written_at: Option<Instant>,
    /// The same instant as `written_at`, as microseconds since the Unix epoch -
    /// for anything that has to leave the process (JSON, `.debug` files), since
    /// an [`Instant`] can't be serialized. `0` while `write_count` is `0`.
    pub written_at_unix_us: u64,
}

/// A topic value together with the [`WriteMeta`] of the write that published it,
/// as returned by [`RwLockTopic::read`]. Derefs to `T`, so field access and
/// method calls on the value work directly; use [`into_value`](Self::into_value)
/// (or `.value`) when an owned `T` is needed.
#[derive(Debug, Clone, PartialEq)]
pub struct Stamped<T> {
    pub value: T,
    pub meta: WriteMeta,
}

impl<T> Stamped<T> {
    /// How long ago this value was written, or `None` if it's still the
    /// `initial` seed nobody has written over yet.
    pub fn age(&self) -> Option<Duration> {
        self.meta.written_at.map(|written_at| written_at.elapsed())
    }

    /// Whether this is still the `initial` seed the topic was registered with.
    pub fn is_seed(&self) -> bool {
        self.meta.write_count == 0
    }

    /// Discards the metadata, keeping just the value.
    pub fn into_value(self) -> T {
        self.value
    }
}

impl<T> Deref for Stamped<T> {
    type Target = T;

    fn deref(&self) -> &T {
        &self.value
    }
}

/// A named, typed channel of the latest-value-wins shared state between executors,
/// backed by a [`RwLock`]: any number of readers may hold the lock concurrently,
/// but a writer briefly excludes every reader (and vice versa) for the duration of
/// one [`write`](Self::write)/[`read`](Self::read) call.
///
/// Exactly one executor may claim the right to write a topic, via
/// [`set_writer`](Self::set_writer); any executor may [`read`](Self::read) it.
/// Readers always see the most recently written value, together with its
/// [`WriteMeta`] - a reader polling faster than the writer publishes will simply
/// observe the same value (and the same `write_count`) more than once, and can
/// compare `write_count` to tell whether anything new arrived, or check
/// [`Stamped::age`] to tell how stale it is. For periodic sensor-style data (e.g. a
/// LIDAR scan), always having the latest reading matters far more than never
/// repeating one.
///
/// [`write`](Self::write) and [`read`](Self::read) may briefly block on that
/// internal lock, so a slow reader can delay a writer (and vice versa) for as long
/// as it holds it. `read` returns an owned clone of `T` (wrapped in a [`Stamped`]) rather than a reference into
/// the topic, which bounds how long a read holds the lock - if it instead handed
/// callers a reference (or a closure over one), slow or long-running caller code
/// would keep the lock held, and delay the writer, for as long as it kept running.
/// For small `T` types published at modest rates (e.g. a LIDAR scan, a few KB at a
/// few hundred Hz) the clone is cheap; for large payloads or very high rates, that
/// tradeoff should be revisited.
pub struct RwLockTopic<T> {
    data: RwLock<Stamped<T>>,
    writer_id: OnceLock<u8>,
}

impl<T: Send + Sync + 'static> RwLockTopic<T> {
    /// Creates a topic with no writer yet claimed, seeded with `initial`.
    pub fn new(initial: T) -> Self {
        Self {
            data: RwLock::new(Stamped {
                value: initial,
                meta: WriteMeta::default(),
            }),
            writer_id: OnceLock::new(),
        }
    }

    /// Claims this topic for writing by `executor_id`. Succeeds - idempotently - if
    /// no writer is claimed yet, or if `executor_id` already holds the slot (e.g.
    /// after being restarted by `Runner::switch_executor`). Fails only if a
    /// *different* `executor_id` already holds it.
    pub fn set_writer(&self, executor_id: u8) -> Result<(), TopicError> {
        match self.writer_id.set(executor_id) {
            Ok(()) => Ok(()),
            Err(_) if self.writer_id.get() == Some(&executor_id) => Ok(()),
            Err(_) => Err(TopicError::WriterAlreadySet),
        }
    }

    /// The currently registered writer, if any.
    pub fn writer(&self) -> Option<u8> {
        self.writer_id.get().copied()
    }

    /// The [`WriteMeta`] of the latest write, without cloning the value - for
    /// cheaply checking whether anything new was published.
    pub fn meta(&self) -> WriteMeta {
        self.data.read().expect("RwLockTopic: lock poisoned").meta
    }
}

impl<T: Clone + Send + Sync + 'static> RwLockTopic<T> {
    /// Publishes `value` as the topic's latest value, if `executor_id` is the
    /// topic's registered writer, stamping it with the current time and bumping
    /// [`WriteMeta::write_count`]. Value and stamp are swapped under the same lock,
    /// so a reader can never see one without the other.
    pub fn write(&self, executor_id: u8, value: T) -> Result<(), TopicError> {
        if self.writer_id.get() == Some(&executor_id) {
            // Read the clocks before taking the lock, to keep its hold time minimal.
            let written_at = Instant::now();
            let written_at_unix_us = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_micros() as u64;
            let mut data = self.data.write().expect("RwLockTopic: lock poisoned");
            data.value = value;
            data.meta = WriteMeta {
                write_count: data.meta.write_count + 1,
                written_at: Some(written_at),
                written_at_unix_us,
            };
            Ok(())
        } else {
            Err(TopicError::UnauthorizedWriter)
        }
    }

    /// Returns a clone of the topic's latest published value, with the
    /// [`WriteMeta`] of the write that published it. Any executor may call this at
    /// any time.
    pub fn read(&self) -> Stamped<T> {
        self.data
            .read()
            .expect("RwLockTopic: lock poisoned")
            .clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn set_writer_by_a_different_id_fails() {
        let topic = RwLockTopic::new(0u32);
        assert_eq!(topic.set_writer(1), Ok(()));
        assert_eq!(topic.set_writer(2), Err(TopicError::WriterAlreadySet));
    }

    #[test]
    fn set_writer_reclaiming_the_same_id_is_idempotent() {
        let topic = RwLockTopic::new(0u32);
        assert_eq!(topic.set_writer(1), Ok(()));
        assert_eq!(topic.set_writer(1), Ok(()));
        assert_eq!(topic.writer(), Some(1));
    }

    #[test]
    fn writer_reports_the_registered_id_once_set() {
        let topic = RwLockTopic::new(0u32);
        assert_eq!(topic.writer(), None);
        topic.set_writer(7).unwrap();
        assert_eq!(topic.writer(), Some(7));
    }

    #[test]
    fn write_from_non_writer_is_rejected() {
        let topic = RwLockTopic::new(0u32);
        assert_eq!(topic.write(1, 42), Err(TopicError::UnauthorizedWriter));

        topic.set_writer(1).unwrap();
        assert_eq!(topic.write(2, 42), Err(TopicError::UnauthorizedWriter));
        assert_eq!(topic.write(1, 42), Ok(()));
        assert_eq!(*topic.read(), 42);
    }

    #[test]
    fn seed_value_has_no_write_meta() {
        let topic = RwLockTopic::new(5u32);
        let read = topic.read();
        assert_eq!(*read, 5);
        assert!(read.is_seed());
        assert_eq!(read.age(), None);
        assert_eq!(read.meta, WriteMeta::default());
    }

    #[test]
    fn each_write_bumps_the_count_and_advances_the_stamp() {
        let topic = RwLockTopic::new(0u32);
        topic.set_writer(1).unwrap();

        topic.write(1, 10).unwrap();
        let first = topic.read();
        assert_eq!(first.meta.write_count, 1);
        assert!(first.meta.written_at.is_some());
        assert!(first.meta.written_at_unix_us > 0);

        topic.write(1, 10).unwrap();
        let second = topic.read();
        assert_eq!(second.meta.write_count, 2);
        assert!(second.meta.written_at >= first.meta.written_at);
        assert_eq!(topic.meta(), second.meta);
    }

    #[test]
    fn rejected_write_leaves_the_meta_untouched() {
        let topic = RwLockTopic::new(0u32);
        topic.set_writer(1).unwrap();
        topic.write(1, 1).unwrap();
        let before = topic.meta();

        assert_eq!(topic.write(2, 99), Err(TopicError::UnauthorizedWriter));
        assert_eq!(topic.meta(), before);
        assert_eq!(*topic.read(), 1);
    }
}
