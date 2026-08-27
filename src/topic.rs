//! A [`Topic`] is a single, typed slot of shared state with exactly one authorized
//! writer and any number of readers.

use crate::lock_free_cell::LockFreeCell;
use std::sync::OnceLock;

/// Errors returned by [`Topic::set_writer`] and [`Topic::write`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TopicError {
    /// [`Topic::set_writer`] was called on a topic that already has a writer.
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
pub trait Topic {
    /// The type of value carried by this topic.
    type Item;

    /// Claims this topic for writing by `executor_id`. Fails if a writer has already
    /// been claimed, even by the same id - a topic's writer is fixed for its
    /// lifetime.
    fn set_writer(&self, executor_id: u8) -> Result<(), TopicError>;

    /// The currently registered writer, if any. Lets a caller whose `set_writer` call
    /// failed tell "I already hold this slot" (its own id, e.g. after being restarted
    /// by `ExecutorHandler::switch_executor`) apart from "someone else holds it" (a
    /// genuine writer conflict).
    fn writer(&self) -> Option<u8>;

    /// Publishes `value` as the topic's latest value, if `executor_id` is the
    /// topic's registered writer.
    fn write(&self, executor_id: u8, value: Self::Item) -> Result<(), TopicError>;

    /// Returns a clone of the topic's latest published value. Any executor may call
    /// this at any time.
    fn read(&self) -> Self::Item;
}

/// The lock-free [`Topic`] implementation: readers and the writer never block each
/// other, at the cost of one small heap allocation per write and one clone per read.
///
/// `read` returns an owned clone of `Item` rather than a reference into the topic.
/// This bounds how long a read holds open the epoch pin that protects the value from
/// reclamation - if it instead handed callers a reference (or a closure over one),
/// slow or long-running caller code would delay reclamation of superseded values for
/// as long as it kept running. For small `Item` types published at modest rates (e.g.
/// a LIDAR scan, a few KB at a few hundred Hz) the clone is cheap; for large payloads
/// or very high rates, that tradeoff should be revisited.
pub struct LockFreeTopic<T> {
    cell: LockFreeCell<T>,
    writer_id: OnceLock<u8>,
}

impl<T: Send + Sync + 'static> LockFreeTopic<T> {
    /// Creates a topic with no writer yet claimed, seeded with `initial`.
    pub fn new(initial: T) -> Self {
        Self {
            cell: LockFreeCell::new(initial),
            writer_id: OnceLock::new(),
        }
    }
}

impl<T: Clone + Send + Sync + 'static> Topic for LockFreeTopic<T> {
    type Item = T;

    fn set_writer(&self, executor_id: u8) -> Result<(), TopicError> {
        self.writer_id
            .set(executor_id)
            .map_err(|_| TopicError::WriterAlreadySet)
    }

    fn writer(&self) -> Option<u8> {
        self.writer_id.get().copied()
    }

    fn write(&self, executor_id: u8, value: T) -> Result<(), TopicError> {
        if self.writer_id.get() == Some(&executor_id) {
            self.cell.store(value);
            Ok(())
        } else {
            Err(TopicError::UnauthorizedWriter)
        }
    }

    fn read(&self) -> T {
        self.cell.load_and(|v| v.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_set_writer_succeeds_second_fails() {
        let topic = LockFreeTopic::new(0u32);
        assert_eq!(topic.set_writer(1), Ok(()));
        assert_eq!(topic.set_writer(1), Err(TopicError::WriterAlreadySet));
        assert_eq!(topic.set_writer(2), Err(TopicError::WriterAlreadySet));
    }

    #[test]
    fn writer_reports_the_registered_id_once_set() {
        let topic = LockFreeTopic::new(0u32);
        assert_eq!(topic.writer(), None);
        topic.set_writer(7).unwrap();
        assert_eq!(topic.writer(), Some(7));
    }

    #[test]
    fn write_from_non_writer_is_rejected() {
        let topic = LockFreeTopic::new(0u32);
        assert_eq!(
            topic.write(1, 42),
            Err(TopicError::UnauthorizedWriter)
        );

        topic.set_writer(1).unwrap();
        assert_eq!(topic.write(2, 42), Err(TopicError::UnauthorizedWriter));
        assert_eq!(topic.write(1, 42), Ok(()));
        assert_eq!(topic.read(), 42);
    }
}
