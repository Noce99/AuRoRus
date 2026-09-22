//! A dependency-free, tqdm-style progress bar for the benchmark's run phase.

use std::io::{self, Write as _};
use std::time::Duration;

/// Prints a single-line, in-place progress bar tracking `elapsed` out of `total`,
/// tqdm-style. Call repeatedly with growing `elapsed`; the caller is responsible for
/// printing a final newline once done.
pub fn print_progress_bar(elapsed: Duration, total: Duration, width: usize) {
    let frac = (elapsed.as_secs_f64() / total.as_secs_f64()).clamp(0.0, 1.0);
    let filled = (frac * width as f64).round() as usize;
    print!(
        "\r[{}{}] {:5.1}% | {:4.1}s / {:.1}s",
        "#".repeat(filled),
        "-".repeat(width - filled),
        frac * 100.0,
        elapsed.as_secs_f64().min(total.as_secs_f64()),
        total.as_secs_f64(),
    );
    let _ = io::stdout().flush();
}
