//! Sanity-checks the final state of the LIDAR scan topic after a run.

use crate::lidar::{LIDAR_SCAN_TOPIC, Scan};
use efficient_data_sharing::{Topic, TopicHandler};

/// Checks that the latest scan on [`LIDAR_SCAN_TOPIC`] is fully consistent: every
/// distance should be a finite, physically plausible reading. The bounds are
/// placeholders for this simulator - replace with the real sensor's documented
/// range.
pub fn verify_consistency(topics: &TopicHandler) -> bool {
    let scan: Scan = topics.topic::<Scan>(LIDAR_SCAN_TOPIC).read();
    scan.iter().all(|&d| d.is_finite() && d > 0.0 && d < 20.0)
}
