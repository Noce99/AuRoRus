//! Racing: the opponents on track, the race start, and lap telemetry.

mod lap_telemetry;
mod opponents;
mod race_start;

pub use lap_telemetry::{LAP_TELEMETRY_TOPIC_NAME, LapRecord, LapTelemetry, LapTrace};
pub use opponents::{
    NumberedRequest, OPPONENT_REQUESTS_TOPIC_NAME, OPPONENTS_TOPIC_NAME, Opponent, OpponentColor,
    OpponentOutcome, OpponentRequest, OpponentRequests, OpponentSpec, Opponents,
};
pub use race_start::{GridSlot, RACE_START_TOPIC_NAME, RaceStart, Racer, now_ms};
