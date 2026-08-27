//! Internal single-writer/multi-reader cell built on [`crossbeam_epoch`].
//!
//! Not part of the public API: [`crate::topic::LockFreeTopic`] is the public-facing
//! wrapper that adds writer-authorization checks on top of this primitive.

use crossbeam_epoch::{self as epoch, Atomic, Owned};
use std::sync::atomic::Ordering;

/// Holds a `T` that one writer publishes and any number of readers observe, without
/// locking.
///
/// # How it works
///
/// [`Atomic<T>`] is a tagged *pointer* (word-sized), not `T`'s bytes inline.
/// [`store`](Self::store) heap-allocates a new `T` and atomically swaps the pointer;
/// the previous value is freed only once every reader that might still be looking at
/// it has moved on ("epoch-based reclamation"). [`load_and`](Self::load_and) never
/// blocks the writer and never blocks other readers - it just follows whatever
/// pointer was most recently published.
///
/// The `#[repr(align(64))]` on this struct only protects the pointer field itself
/// from false sharing with unrelated data on the same cache line; it does not make
/// `T`'s bytes cache-line resident, since those bytes live in a separate heap
/// allocation.
#[repr(align(64))]
pub(crate) struct LockFreeCell<T> {
    data: Atomic<T>,
}

impl<T: Send + Sync + 'static> LockFreeCell<T> {
    /// Creates a cell holding `initial`.
    pub(crate) fn new(initial: T) -> Self {
        Self {
            data: Atomic::new(initial),
        }
    }

    /// Publishes `value` as the latest value.
    ///
    /// Meant to be called from a single writer. Allocates once for the new value and
    /// defers freeing the previous one until it is safe.
    #[inline(always)]
    pub(crate) fn store(&self, value: T) {
        let guard = epoch::pin();
        let new = Owned::new(value);
        let old = self.data.swap(new, Ordering::Release, &guard);
        // Old value will be reclaimed when no readers are using it.
        unsafe { guard.defer_destroy(old) };
    }

    /// Runs `f` against whatever value is currently the latest published one.
    ///
    /// Safe to call concurrently from any number of readers. `f` should be quick: it
    /// runs while pinned to the current epoch, which delays reclamation of any value
    /// a writer swaps out in the meantime.
    #[inline(always)]
    pub(crate) fn load_and<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&T) -> R,
    {
        let guard = epoch::pin();
        let shared = self.data.load(Ordering::Acquire, &guard);
        unsafe { f(&*shared.as_raw()) }
    }
}
