//! `cli`, `progress` and `verifier` are byte-identical between
//! `benchmark_communication_time` and `benchmark_data_freshness`, so they
//! live once here and each pulls this module in by path. A plain `mod` can't
//! reach outside a binary's own directory, and these are binary-only helpers
//! that have no business in the library's public API.

pub mod cli;
pub mod progress;
pub mod verifier;
