//! A tiny ANSI-colored line format shared by [`crate::Runner`]'s verbose logs
//! and [`crate::Captain`]'s fatal writer-conflict diagnostic - both are tagged
//! as coming from the `Runner` that owns the `Captain`.

use std::fmt;
use time::OffsetDateTime;

/// Colors used to tag a log line. Values are ANSI SGR parameters (without the
/// leading `\x1b[` or trailing `m`), always paired with bold.
#[derive(Clone, Copy)]
pub(crate) enum LogColor {
    Pink,
    Orange,
    Green,
    Purple,
    Yellow,
    Red,
}

impl LogColor {
    fn sgr(self) -> &'static str {
        match self {
            Self::Pink => "38;5;213",
            Self::Orange => "38;5;208",
            Self::Green => "32",
            Self::Purple => "38;5;129",
            Self::Yellow => "33",
            Self::Red => "31",
        }
    }
}

/// The current local time as `hh:mm:ss`. Falls back to UTC if the local UTC
/// offset can't be determined (`OffsetDateTime::now_local` can fail e.g. on
/// Unix in a multi-threaded process, for soundness reasons outside our
/// control).
fn now_hhmmss() -> String {
    let now = OffsetDateTime::now_local().unwrap_or_else(|_| OffsetDateTime::now_utc());
    format!("{:02}:{:02}:{:02}", now.hour(), now.minute(), now.second())
}

/// Formats `message` as one `[Runner hh:mm:ss] - ...` line, in bold `color`.
/// Callers choose the output stream (`println!`/`eprintln!`) themselves.
pub(crate) fn format_line(color: LogColor, message: impl fmt::Display) -> String {
    format!(
        "\x1b[{};1m[Runner {}] - {message}\x1b[0m",
        color.sgr(),
        now_hhmmss()
    )
}

/// `T`'s type name with every module path stripped - `std::any::type_name` gives
/// e.g. `alloc::vec::Vec<aurorus::topics::Drawing>`, this gives `Vec<Drawing>`.
pub(crate) fn short_type_name<T: ?Sized>() -> String {
    let full = std::any::type_name::<T>();
    let mut short = String::with_capacity(full.len());
    let mut segment_start = 0;
    for (i, c) in full.char_indices() {
        if c.is_alphanumeric() || c == '_' || c == ':' {
            continue;
        }
        short.push_str(last_path_segment(&full[segment_start..i]));
        short.push(c);
        segment_start = i + c.len_utf8();
    }
    short.push_str(last_path_segment(&full[segment_start..]));
    short
}

fn last_path_segment(path: &str) -> &str {
    path.rsplit("::").next().unwrap_or(path)
}
