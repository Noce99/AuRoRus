//! A [`Topic`] is a single, typed slot of shared state with exactly one authorized
//! writer and any number of readers.

use std::sync::{OnceLock, RwLock};

/// Errors returned by [`Topic::set_writer`] and [`Topic::write`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TopicError {
    /// [`Topic::set_writer`] was called on a topic that already has a *different*
    /// writer. Reclaiming the slot with the same `executor_id` that already holds
    /// it is not an error - see [`Topic::set_writer`].
    WriterAlreadySet,
    /// [`Topic::write`] was called with an executor id that isn't the topic's
    /// registered writer (including the case where no writer has been set yet).
    UnauthorizedWriter,
}

/// A named, typed channel of the latest-value-wins shared state between executors.
///
/// Exactly one executor may claim the right to write a topic, via
/// [`set_writer`](Self::set_writer); any executor may [`read`](Self::read) it.
/// Readers always see the most recently written value - there is no notion of
/// "unread" data or staleness, so a reader polling faster than the writer publishes
/// will simply observe the same value more than once. That's by design: for periodic
/// sensor-style data (e.g. a LIDAR scan), always having the latest reading matters far
/// more than never repeating one.
///
/// [`write`](Self::write) and [`read`](Self::read) may briefly block: an
/// implementation is free to use a lock internally, so a slow reader can delay a
/// writer (and vice versa) for as long as it holds that lock.
pub trait Topic {
    /// The type of value carried by this topic.
    type Item;

    /// Claims this topic for writing by `executor_id`. Succeeds - idempotently - if
    /// no writer is claimed yet, or if `executor_id` already holds the slot (e.g.
    /// after being restarted by `Runner::switch_executor`). Fails only if a
    /// *different* `executor_id` already holds it.
    fn set_writer(&self, executor_id: u8) -> Result<(), TopicError>;

    /// The currently registered writer, if any.
    fn writer(&self) -> Option<u8>;

    /// Publishes `value` as the topic's latest value, if `executor_id` is the
    /// topic's registered writer.
    fn write(&self, executor_id: u8, value: Self::Item) -> Result<(), TopicError>;

    /// Returns a clone of the topic's latest published value. Any executor may call
    /// this at any time.
    fn read(&self) -> Self::Item;
}

/// The [`RwLock`]-backed [`Topic`] implementation: any number of readers may hold
/// the lock concurrently, but a writer briefly excludes every reader (and vice
/// versa) for the duration of one `write`/`read` call.
///
/// `read` returns an owned clone of `Item` rather than a reference into the topic.
/// This bounds how long a read holds the lock - if it instead handed callers a
/// reference (or a closure over one), slow or long-running caller code would keep
/// the lock held, and delay the writer, for as long as it kept running. For small
/// `Item` types published at modest rates (e.g. a LIDAR scan, a few KB at a few
/// hundred Hz) the clone is cheap; for large payloads or very high rates, that
/// tradeoff should be revisited.
pub struct RwLockTopic<T> {
    data: RwLock<T>,
    writer_id: OnceLock<u8>,
}

impl<T: Send + Sync + 'static> RwLockTopic<T> {
    /// Creates a topic with no writer yet claimed, seeded with `initial`.
    pub fn new(initial: T) -> Self {
        Self {
            data: RwLock::new(initial),
            writer_id: OnceLock::new(),
        }
    }
}

impl<T: Clone + Send + Sync + 'static> Topic for RwLockTopic<T> {
    type Item = T;

    fn set_writer(&self, executor_id: u8) -> Result<(), TopicError> {
        match self.writer_id.set(executor_id) {
            Ok(()) => Ok(()),
            Err(_) if self.writer_id.get() == Some(&executor_id) => Ok(()),
            Err(_) => Err(TopicError::WriterAlreadySet),
        }
    }

    fn writer(&self) -> Option<u8> {
        self.writer_id.get().copied()
    }

    fn write(&self, executor_id: u8, value: T) -> Result<(), TopicError> {
        if self.writer_id.get() == Some(&executor_id) {
            *self.data.write().expect("RwLockTopic: lock poisoned") = value;
            Ok(())
        } else {
            Err(TopicError::UnauthorizedWriter)
        }
    }

    fn read(&self) -> T {
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
        assert_eq!(topic.read(), 42);
    }
}
