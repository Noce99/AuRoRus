//! Per-executor timing summary, built once a writer or reader executor stops.

use std::time::Duration;

/// Timing summary for one executor's whole run, computed from the individual
/// per-operation durations (in nanoseconds) it recorded locally while running.
///
/// Every executor times its own operations locally (no shared atomics on the hot
/// path) and folds the samples into a `Report` only when it exits, which is also
/// where mean/standard-deviation/max are computed.
pub struct Report {
    pub label: String,
    pub verb: &'static str,
    pub rate_hz: f64,
    pub count: u64,
    pub mean_ns: f64,
    pub std_dev_ns: f64,
    pub max_ns: u64,
}

impl Report {
    /// Builds a report from `label`/`verb` (e.g. `"Writer 0"` / `"writes"`) and the
    /// raw per-operation durations recorded over the executor's lifetime.
    pub fn from_samples(label: impl Into<String>, verb: &'static str, rate_hz: f64, samples: &[u64]) -> Self {
        let count = samples.len() as u64;
        let mean_ns = if count > 0 {
            samples.iter().sum::<u64>() as f64 / count as f64
        } else {
            0.0
        };
        let std_dev_ns = if count > 1 {
            let variance = samples
                .iter()
                .map(|&s| {
                    let d = s as f64 - mean_ns;
                    d * d
                })
                .sum::<f64>()
                / (count - 1) as f64;
            variance.sqrt()
        } else {
            0.0
        };
        let max_ns = samples.iter().copied().max().unwrap_or(0);

        Self {
            label: label.into(),
            verb,
            rate_hz,
            count,
            mean_ns,
            std_dev_ns,
            max_ns,
        }
    }

    /// How many operations this executor should have completed over `duration` if it
    /// had run at exactly its target rate the whole time.
    pub fn expected_count(&self, duration: Duration) -> u64 {
        (self.rate_hz * duration.as_secs_f64()).round() as u64
    }

    /// The time budget for one operation at this executor's target rate: e.g. a
    /// 300 Hz reader must finish each read within `1/300 s` to keep pace with its own
    /// schedule (ignoring whatever else it also has to do).
    pub fn period(&self) -> Duration {
        Duration::from_secs_f64(1.0 / self.rate_hz)
    }

    /// An indented, multi-line report block for this executor, e.g.:
    ///
    /// ```text
    ///   Reader 3 - target 300 Hz (period 3.333 ms)
    ///     reads      : 2907 (3000 expected)
    ///     avg time   :    4.24 us +-    5.29 us
    ///     max time   :  129.00 us  (3.9% of period, within budget)
    /// ```
    pub fn format_block(&self, duration: Duration) -> String {
        let period_us = self.period().as_secs_f64() * 1_000_000.0;
        let max_us = self.max_ns as f64 / 1000.0;
        let pct_of_period = max_us / period_us * 100.0;
        let budget_note = if pct_of_period <= 100.0 {
            "within budget"
        } else {
            "OVER BUDGET"
        };

        let header = format!(
            "  {label} - target {rate:.0} Hz (period {period_ms:.3} ms)",
            label = self.label,
            rate = self.rate_hz,
            period_ms = period_us / 1000.0
        );
        let count_line = format!(
            "    {verb:<10}: {count:>5} ({expected:>5} expected)",
            verb = self.verb,
            count = self.count,
            expected = self.expected_count(duration)
        );
        let avg_line = format!(
            "    avg time  : {mean:>7.2} us +- {std:>6.2} us",
            mean = self.mean_ns / 1000.0,
            std = self.std_dev_ns / 1000.0
        );
        let max_line = format!(
            "    max time  : {max_us:>7.2} us  ({pct_of_period:.1}% of period, {budget_note})"
        );

        format!("{header}\n{count_line}\n{avg_line}\n{max_line}")
    }
}
