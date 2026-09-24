//! Sanity-checks the final state of the shared topic after a run.

use aurorus::Runner;
use serde::Serialize;
use serde::de::DeserializeOwned;

/// Checks that the latest payload on `topic_name` is non-empty and every value in
/// it is finite. There's no real sensor behind either benchmark, so there's no
/// physically-motivated range to check - just that the writer actually produced a
/// sane payload.
///
/// Generic over the payload type, with `values` pulling the `f32`s out of it, so
/// both benchmarks can share this despite publishing different payload shapes
/// (a bare `Vec<f32>` vs. one carrying a timestamp alongside it).
pub fn verify_consistency<T>(runner: &Runner, topic_name: &str, values: impl Fn(&T) -> &[f32]) -> bool
where
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
{
    let payload: T = runner.topic::<T>(topic_name).read().into_value();
    let values = values(&payload);
    !values.is_empty() && values.iter().all(|v| v.is_finite())
}
