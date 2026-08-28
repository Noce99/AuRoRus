//! Sanity-checks the final state of the shared topic after a run.

use crate::reader_writer::{TOPIC_NAME, TimestampedPayload};
use aurorus::Runner;

/// Checks that the latest payload on [`TOPIC_NAME`] is non-empty and every value in
/// it is finite. There's no real sensor behind this benchmark, so there's no
/// physically-motivated range to check - just that the writer actually produced a
/// sane payload.
pub fn verify_consistency(runner: &Runner) -> bool {
    let payload: TimestampedPayload = runner.topic::<TimestampedPayload>(TOPIC_NAME).read();
    !payload.data.is_empty() && payload.data.iter().all(|v| v.is_finite())
}
