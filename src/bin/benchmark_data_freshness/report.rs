//! Reporting for the freshness benchmark: a plain write count for the writer, and a
//! merged min/mean±std/max data-age report per reader rate tier.

use std::time::Duration;

/// How many writes a writer executor performed, vs. how many it should have at its
/// target rate. The writer always publishes fresh data (age 0 at publish time), so
/// unlike [`AgeReport`] there's no age distribution to report for it - just whether
/// it kept pace.
pub struct WriteReport {
    pub label: String,
    pub rate_hz: f64,
    pub count: u64,
}

impl WriteReport {
    pub fn new(label: impl Into<String>, rate_hz: f64, count: u64) -> Self {
        Self {
            label: label.into(),
            rate_hz,
            count,
        }
    }

    /// How many writes this executor should have completed over `duration` if it
    /// had run at exactly its target rate the whole time.
    pub fn expected_count(&self, duration: Duration) -> u64 {
        (self.rate_hz * duration.as_secs_f64()).round() as u64
    }

    pub fn format_block(&self, duration: Duration) -> String {
        format!(
            "  {label} - target {rate:.0} Hz\n    writes    : {count:>5} ({expected:>5} expected)",
            label = self.label,
            rate = self.rate_hz,
            count = self.count,
            expected = self.expected_count(duration),
        )
    }
}

/// Theoretical age statistics predicted from the writer's rate alone - see
/// [`crate::main`] for how each is derived. All three are independent of any
/// reader's own poll rate: a reader is just sampling a curve the writer alone
/// controls, so how often it samples doesn't change what that curve looks like.
///
/// The age of the topic's current value is a sawtooth driven entirely by the
/// writer: it resets to 0 at each publish and climbs linearly until the next one,
/// over a period of `1 / writer_hz`. A reader samples that sawtooth at points
/// effectively uncorrelated with the writer's schedule, so its observed ages behave
/// like uniform samples over `[0, 1 / writer_hz)`: mean `= period / 2`, std dev
/// `= period / sqrt(12)`.
///
/// The max doesn't follow that same clean model - real writer threads occasionally
/// miss their schedule (OS scheduling jitter, lock contention), spiking the
/// sawtooth well past its nominal period. Since that's a property of the writer's
/// own timing rather than of the sawtooth's ideal shape, and since even a modest
/// sample count is enough for *any* reader tier to catch such a spike (max is an
/// extreme-value statistic, not an average - it converges to the worst thing that
/// happened, and more samples only improve the odds of seeing it, never the
/// underlying ceiling), the max is predicted as a flat multiple of the writer's
/// period rather than tightened by a faster reader's own rate.
#[derive(Clone, Copy)]
pub struct ExpectedAge {
    pub mean_ns: u64,
    pub std_dev_ns: u64,
    pub max_ns: u64,
}

impl ExpectedAge {
    /// How large a multiple of the writer's period the max age is expected to reach,
    /// allowing for the writer occasionally missing one full publish cycle.
    pub const MAX_AGE_JITTER_MULTIPLIER: f64 = 2.0;

    /// Predicts age statistics for a writer publishing at `writer_hz`. Mean and std
    /// dev come from the uniform-sampling model above; max is
    /// [`Self::MAX_AGE_JITTER_MULTIPLIER`] times the writer's period.
    pub fn predict(writer_hz: f64) -> Self {
        let writer_period_secs = 1.0 / writer_hz;
        Self {
            mean_ns: (writer_period_secs / 2.0 * 1e9) as u64,
            std_dev_ns: (writer_period_secs / 12f64.sqrt() * 1e9) as u64,
            max_ns: (writer_period_secs * Self::MAX_AGE_JITTER_MULTIPLIER * 1e9) as u64,
        }
    }
}

/// Merged data-freshness summary for one reader rate tier: how old (in nanoseconds,
/// measured as `read_time - payload.timestamp`) the payload each reader observed
/// was, pooled across every reader sharing that tier.
pub struct AgeReport {
    pub label: String,
    pub rate_hz: f64,
    /// How many readers' samples were merged into this one report.
    pub group_size: u64,
    pub count: u64,
    pub min_ns: u64,
    pub mean_ns: f64,
    pub std_dev_ns: f64,
    pub max_ns: u64,
    /// The theoretical age statistics this tier's readers should observe.
    pub expected: ExpectedAge,
}

impl AgeReport {
    /// Builds a report from `label` (e.g. `"Readers @ 30 Hz (x2)"`), the raw
    /// per-read age samples (in nanoseconds) pooled from every reader in the tier,
    /// `group_size` (how many readers those samples came from), and `expected`, the
    /// predicted age statistics for this tier (see [`ExpectedAge::predict`]).
    pub fn from_samples(
        label: impl Into<String>,
        rate_hz: f64,
        group_size: u64,
        expected: ExpectedAge,
        samples: &[u64],
    ) -> Self {
        let count = samples.len() as u64;
        let min_ns = samples.iter().copied().min().unwrap_or(0);
        let max_ns = samples.iter().copied().max().unwrap_or(0);
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

        Self {
            label: label.into(),
            rate_hz,
            group_size,
            count,
            min_ns,
            mean_ns,
            std_dev_ns,
            max_ns,
            expected,
        }
    }

    /// How many reads this tier's readers should have completed over `duration` if
    /// each had run at exactly its target rate the whole time.
    pub fn expected_count(&self, duration: Duration) -> u64 {
        (self.rate_hz * duration.as_secs_f64()).round() as u64 * self.group_size
    }

    /// The time budget for one operation at this tier's target rate: `1 / rate_hz`.
    pub fn period(&self) -> Duration {
        Duration::from_secs_f64(1.0 / self.rate_hz)
    }

    /// An indented, multi-line report block for this tier, e.g.:
    ///
    /// ```text
    ///   Readers @ 30 Hz (x2) - target 30 Hz (period 33.333 ms)
    ///     reads     :   600 (  600 expected)
    ///     min age   :    0.412 ms
    ///     avg age   :   10.203 ms +-    6.011 ms  (expected   10.000 ms +-    5.774 ms)
    ///     max age   :   32.870 ms  (73.0% of expected max 45.000 ms, within budget)
    /// ```
    pub fn format_block(&self, duration: Duration) -> String {
        let period_ms = self.period().as_secs_f64() * 1000.0;
        let min_ms = self.min_ns as f64 / 1_000_000.0;
        let mean_ms = self.mean_ns / 1_000_000.0;
        let std_ms = self.std_dev_ns / 1_000_000.0;
        let max_ms = self.max_ns as f64 / 1_000_000.0;
        let expected_mean_ms = self.expected.mean_ns as f64 / 1_000_000.0;
        let expected_std_ms = self.expected.std_dev_ns as f64 / 1_000_000.0;
        let expected_max_ms = self.expected.max_ns as f64 / 1_000_000.0;
        let pct_of_expected_max = max_ms / expected_max_ms * 100.0;
        let budget_note = if pct_of_expected_max <= 100.0 {
            "within budget"
        } else {
            "OVER BUDGET"
        };

        let header = format!(
            "  {label} - target {rate:.0} Hz (period {period_ms:.3} ms)",
            label = self.label,
            rate = self.rate_hz,
        );
        let count_line = format!(
            "    reads     : {count:>5} ({expected:>5} expected)",
            count = self.count,
            expected = self.expected_count(duration),
        );
        let min_line = format!("    min age   : {min_ms:>7.3} ms");
        let avg_line = format!(
            "    avg age   : {mean_ms:>7.3} ms +- {std_ms:>7.3} ms  \
             (expected {expected_mean_ms:>7.3} ms +- {expected_std_ms:>7.3} ms)"
        );
        let max_line = format!(
            "    max age   : {max_ms:>7.3} ms  ({pct_of_expected_max:.1}% of expected max \
             {expected_max_ms:.3} ms, {budget_note})"
        );

        format!("{header}\n{count_line}\n{min_line}\n{avg_line}\n{max_line}")
    }
}
