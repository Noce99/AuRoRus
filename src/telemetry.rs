//! Executors that watch how the vehicle is driving rather than drive it:
//! [`LapTelemetryRecorder`] measures the ego vehicle's laps against the race
//! line it follows (see [`crate::topics::LapTelemetry`]).

mod lap_telemetry;

pub use lap_telemetry::{LapTelemetryConfig, LapTelemetryRecorder, TelemetryPoseSource};
