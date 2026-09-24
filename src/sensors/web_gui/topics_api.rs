//! The generic topic inspector API: every registered topic's name, writer,
//! and freshness, and any one topic's current value as JSON - for the
//! Topics panel, which then needs no code of its own per topic.

use crate::Captain;
use crate::core::DebugTopic;
use crate::web::{bad_request, json_response, not_found, query_param};
use std::sync::Arc;
use tiny_http::ResponseBox;

/// Largest value, as JSON, [`value`] sends - anything bigger (e.g. the
/// `map` topic's raster) is reported as too large instead of serialized.
const MAX_VALUE_JSON_BYTES: usize = 256 * 1024;

#[derive(serde::Serialize)]
struct TopicSummary {
    name: String,
    writer: Option<String>,
    write_count: u64,
    /// `null` while the topic still holds its seed.
    age_ms: Option<f64>,
}

fn summary(captain: &Captain, name: String, topic: &Arc<dyn DebugTopic>) -> TopicSummary {
    let meta = topic.meta();
    TopicSummary {
        writer: topic.writer().map(|id| captain.name_of(id)),
        write_count: meta.write_count,
        age_ms: meta
            .written_at
            .map(|written_at| written_at.elapsed().as_secs_f64() * 1000.0),
        name,
    }
}

/// `GET /api/topics` - every registered topic, in name order.
pub fn list(captain: &Captain) -> ResponseBox {
    let mut topics: Vec<TopicSummary> = captain
        .debug_topics_snapshot()
        .into_iter()
        .map(|(name, topic)| summary(captain, name, &topic))
        .collect();
    topics.sort_by(|a, b| a.name.cmp(&b.name));
    json_response(&topics, 200)
}

#[derive(serde::Serialize)]
struct TopicValue {
    #[serde(flatten)]
    summary: TopicSummary,
    written_at_unix_us: u64,
    /// The topic's value, or `null` if it's `too_large` to send.
    value: Option<serde_json::Value>,
    too_large: bool,
}

/// `GET /api/topic?name=...` - one topic's current value as JSON, with its
/// writer and freshness.
pub fn value(url: &str, captain: &Captain) -> ResponseBox {
    let Some(name) = query_param(url, "name") else {
        return bad_request("expected a name query parameter");
    };
    let Some((name, topic)) = captain
        .debug_topics_snapshot()
        .into_iter()
        .find(|(n, _)| *n == name)
    else {
        return not_found();
    };

    let (value, meta) = topic.read_json(MAX_VALUE_JSON_BYTES);
    json_response(
        &TopicValue {
            summary: TopicSummary {
                writer: topic.writer().map(|id| captain.name_of(id)),
                write_count: meta.write_count,
                age_ms: meta
                    .written_at
                    .map(|written_at| written_at.elapsed().as_secs_f64() * 1000.0),
                name,
            },
            written_at_unix_us: meta.written_at_unix_us,
            too_large: value.is_none(),
            value,
        },
        200,
    )
}
