//! [`Ticker`]: the one way a periodic loop in this crate paces itself.

use std::thread;
use std::time::{Duration, Instant};

/// Below this much time left before the deadline, spinning on
/// [`thread::yield_now`] lands closer to it than asking the OS to sleep -
/// `thread::sleep` only guarantees sleeping *at least* the requested time,
/// and its overshoot is on this order.
const SPIN_THRESHOLD: Duration = Duration::from_micros(100);

/// Paces a loop to a fixed rate by sleeping until the next *deadline*, not
/// by sleeping a fixed amount each time around.
///
/// The difference matters. A loop that ends with
/// `thread::sleep(1.0 / rate_hz)` takes `period + work + however much the OS
/// overshoots` per iteration, so it always runs slower than its nominal
/// rate, and the shortfall grows as the period shrinks: measured on this
/// codebase's own benchmark, readers written that way achieved 360/360 ticks
/// at 30 Hz but only 3505/3600 at 300 Hz. A `Ticker` instead advances a
/// deadline by exactly one period each time, so an iteration that runs long
/// is paid back by the next one sleeping less, and the *average* rate stays
/// on target.
///
/// Create one just before the loop and call [`wait`](Self::wait) at the end
/// of each iteration:
///
/// ```no_run
/// # use aurorus::Ticker;
/// let mut ticker = Ticker::new(100.0);
/// loop {
///     // ... one iteration of work ...
///     ticker.wait();
/// }
/// ```
pub struct Ticker {
    interval: Duration,
    /// When the iteration that's about to start should have started. Advanced
    /// by exactly `interval` per [`wait`](Self::wait), which is what keeps the
    /// average rate exact.
    next_tick: Instant,
}

impl Ticker {
    /// A ticker pacing a loop to `rate_hz`, starting its first period now.
    ///
    /// # Panics
    ///
    /// Panics if `rate_hz` isn't finite and positive - a non-positive or NaN
    /// rate has no sensible period, and silently picking one would hide a
    /// misconfigured `*_hz` config value behind a loop running at some
    /// arbitrary speed.
    pub fn new(rate_hz: f64) -> Self {
        assert!(
            rate_hz.is_finite() && rate_hz > 0.0,
            "Ticker: rate_hz must be finite and positive, got {rate_hz}"
        );
        Self::from_interval(Duration::from_secs_f64(1.0 / rate_hz))
    }

    /// A ticker pacing a loop to one iteration per `interval`, for callers
    /// whose config names a period (e.g. `poll_interval_ms`) rather than a
    /// rate.
    pub fn from_interval(interval: Duration) -> Self {
        Self { interval, next_tick: Instant::now() }
    }

    /// The period between ticks.
    pub fn interval(&self) -> Duration {
        self.interval
    }

    /// Blocks until the next tick is due.
    ///
    /// If the loop has fallen more than a full period behind - a long stall,
    /// a suspended machine - the missed ticks are abandoned rather than run
    /// back to back to "catch up", which for a simulation or a sensor poll
    /// is never what's wanted.
    pub fn wait(&mut self) {
        self.next_tick += self.interval;

        // Fell far enough behind that catching up is pointless: re-base the
        // schedule on now instead of racing through every missed deadline.
        let now = Instant::now();
        if now > self.next_tick + self.interval {
            self.next_tick = now + self.interval;
        }

        let remaining = self.next_tick.saturating_duration_since(now);
        if remaining > SPIN_THRESHOLD {
            thread::sleep(remaining);
        }
        // Either the sleep above undershot, or there was too little left to
        // sleep on in the first place - close the last of the gap precisely.
        while Instant::now() < self.next_tick {
            thread::yield_now();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The property that matters, and the one a `sleep(interval)` loop fails:
    /// N iterations of real work take N periods of wall clock, not
    /// N * (period + work).
    #[test]
    fn keeps_the_average_rate_despite_per_iteration_work() {
        let rate_hz = 200.0;
        let iterations = 100;

        let mut ticker = Ticker::new(rate_hz);
        let start = Instant::now();
        for _ in 0..iterations {
            // Work that is a meaningful fraction of the 5 ms period - enough
            // that a fixed-sleep loop would visibly overrun.
            thread::sleep(Duration::from_micros(500));
            ticker.wait();
        }
        let elapsed = start.elapsed().as_secs_f64();

        let expected = f64::from(iterations) / rate_hz;
        let achieved_hz = f64::from(iterations) / elapsed;
        assert!(
            achieved_hz > rate_hz * 0.95,
            "achieved {achieved_hz:.1} Hz of {rate_hz:.1} Hz requested \
             ({elapsed:.3}s for {iterations} iterations, expected ~{expected:.3}s)"
        );
    }

    #[test]
    fn a_long_stall_does_not_replay_every_missed_tick() {
        let mut ticker = Ticker::new(1000.0);
        // Miss ~50 ticks in one go.
        thread::sleep(Duration::from_millis(50));

        // Catching up would return instantly ~50 times over; re-basing means
        // the very next wait is a normal-length one.
        ticker.wait();
        let start = Instant::now();
        ticker.wait();
        assert!(
            start.elapsed() >= Duration::from_micros(500),
            "expected a full period after re-basing, waited {:?}",
            start.elapsed()
        );
    }

    #[test]
    fn interval_reports_the_configured_period() {
        assert_eq!(Ticker::new(50.0).interval(), Duration::from_millis(20));
        assert_eq!(
            Ticker::from_interval(Duration::from_millis(200)).interval(),
            Duration::from_millis(200)
        );
    }

    #[test]
    #[should_panic(expected = "must be finite and positive")]
    fn a_non_positive_rate_is_rejected() {
        Ticker::new(0.0);
    }
}
