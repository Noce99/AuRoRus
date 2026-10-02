//! Benchmarks: an autonomous algorithm driving the ego vehicle alone for a
//! fixed number of laps, recorded under a `benchmarks/` folder - one
//! self-contained folder per run (see [`layout`]) holding a human-readable
//! [`BenchmarkSummary`] (`summary.toml`), the vehicle's
//! [`TrajectoryRow`]s (`trajectory.csv`), and copies of the exact map and
//! race line driven.
//!
//! `web_gui`'s Benchmark panel writes them (see
//! `crate::web::gui`); everything here is independent of it, so a
//! viewer can [`scan`] and read them back without running anything.

pub mod hash;
pub mod layout;
pub mod summary;
pub mod trajectory;

pub use layout::{ScanOutcome, ScannedRun, scan, scan_all};
pub use summary::{
    AlgorithmRecord, BenchmarkSummary, CodeVersion, FORMAT_VERSION, LapResult, MapRecord,
    RaceLineRecord, Results, RunStatus, VehicleRecord,
};
pub use trajectory::{TrajectoryRow, TrajectoryWriter, read_trajectory};
