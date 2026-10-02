//! The topics tying [`crate::simulation::OpponentsManager`] to whoever drives
//! it (e.g. `web_gui`'s Opponents panel): [`OpponentRequests`] asks it to add
//! or delete opponents - other autonomous vehicles, each running its own
//! copy of an autonomous algorithm - and [`Opponents`] reports the ones
//! running and how the latest request went.

use crate::topics::{ActuatorLimits, Color};

/// Name of the topic [`OpponentRequests`] is published on.
pub const OPPONENT_REQUESTS_TOPIC_NAME: &str = "opponent_requests";
/// Name of the topic [`Opponents`] is published on.
pub const OPPONENTS_TOPIC_NAME: &str = "opponents";

/// The colors an opponent can be drawn in - never the ego vehicle's amber.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OpponentColor {
    #[default]
    Red,
    Blue,
    Green,
    Purple,
    White,
    Cyan,
    Pink,
}

impl OpponentColor {
    /// Every color, in the order a picker lists them.
    pub const ALL: [Self; 7] = [
        Self::Red,
        Self::Blue,
        Self::Green,
        Self::Purple,
        Self::White,
        Self::Cyan,
        Self::Pink,
    ];

    pub fn color(self) -> Color {
        match self {
            Self::Red => Color::RED,
            Self::Blue => Color::BLUE,
            Self::Green => Color::GREEN,
            Self::Purple => Color::PURPLE,
            Self::White => Color::WHITE,
            Self::Cyan => Color::CYAN,
            Self::Pink => Color::PINK,
        }
    }
}

/// Everything an opponent is made of - see
/// [`crate::simulation::opponents::validate`] for what's accepted.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct OpponentSpec {
    pub color: OpponentColor,
    /// The file (inside the current map's `race_lines/`) of the race line it
    /// follows - required by an algorithm that follows one.
    pub race_line: Option<String>,
    /// The file stem of the autonomous algorithm it runs, e.g. `gap_follower`.
    pub algorithm: String,
    /// Multiplies every speed its algorithm commands, in `0..=1`.
    pub speed_scale: f64,
    /// Its actuators' limits - within [`ActuatorLimits::tunable_parameters`].
    pub limits: ActuatorLimits,
}

/// One thing asked of [`crate::simulation::OpponentsManager`].
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OpponentRequest {
    /// Spawn a new opponent.
    Add(OpponentSpec),
    /// Delete the opponent with this [`Opponent::id`].
    Delete(u32),
}

/// An [`OpponentRequest`], numbered in the order it was made.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct NumberedRequest {
    pub number: u64,
    pub request: OpponentRequest,
}

/// The latest requests made of [`crate::simulation::OpponentsManager`], oldest
/// first - the last [`OpponentRequests::KEPT`], so several made between two of
/// its reads are all handled, in order.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct OpponentRequests {
    pub requests: Vec<NumberedRequest>,
}

impl OpponentRequests {
    /// How many requests are kept - far more than can pile up between two
    /// reads.
    pub const KEPT: usize = 32;

    /// Appends `request`, numbered one past the latest, dropping the oldest
    /// beyond [`KEPT`](Self::KEPT). Returns its number.
    pub fn push(&mut self, request: OpponentRequest) -> u64 {
        let number = self.latest() + 1;
        self.requests.push(NumberedRequest { number, request });
        let excess = self.requests.len().saturating_sub(Self::KEPT);
        self.requests.drain(..excess);
        number
    }

    /// The latest request's number - `0` if none was ever made.
    pub fn latest(&self) -> u64 {
        self.requests.last().map_or(0, |request| request.number)
    }
}

/// An opponent that's running.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Opponent {
    /// Never reused, so its topics - under
    /// [`crate::topics::VehicleTopics::opponent`] - are its own.
    pub id: u32,
    pub spec: OpponentSpec,
    /// Its algorithm's human-readable name, e.g. `"Gap follower"`.
    pub algorithm_label: String,
}

/// How an [`OpponentRequest`] went.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct OpponentOutcome {
    /// The [`NumberedRequest::number`] this answers.
    pub request: u64,
    /// Why it was refused - `None` if it was carried out.
    pub error: Option<String>,
}

/// What [`crate::simulation::OpponentsManager`] is running.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Opponents {
    /// Oldest first.
    pub list: Vec<Opponent>,
    /// How the latest request went - `None` before the first one.
    pub last_outcome: Option<OpponentOutcome>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn requests_are_numbered_in_order_and_only_the_latest_kept() {
        let mut requests = OpponentRequests::default();
        assert_eq!(requests.latest(), 0);
        for expected in 1..=(OpponentRequests::KEPT as u64 + 5) {
            assert_eq!(requests.push(OpponentRequest::Delete(0)), expected);
        }
        assert_eq!(requests.requests.len(), OpponentRequests::KEPT);
        assert_eq!(requests.requests[0].number, 6);
        assert_eq!(requests.latest(), OpponentRequests::KEPT as u64 + 5);
    }

    #[test]
    fn every_opponent_color_differs_from_the_ego_vehicles() {
        for color in OpponentColor::ALL {
            assert_ne!(color.color(), Color::AMBER);
        }
    }

    #[test]
    fn requests_survive_the_debug_recorders_encoding() {
        let mut requests = OpponentRequests::default();
        requests.push(OpponentRequest::Delete(3));
        let config = bincode::config::standard();
        let encoded = bincode::serde::encode_to_vec(&requests, config).unwrap();
        let (decoded, _): (OpponentRequests, _) =
            bincode::serde::decode_from_slice(&encoded, config).unwrap();
        assert_eq!(decoded, requests);
    }
}
