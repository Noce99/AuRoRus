//! The simplest possible autonomous algorithm: full left steering lock at
//! full speed, forever. Not useful for driving - a placeholder that shows the
//! shape every algorithm in this folder follows (see
//! [`crate::autonomous_control`]).

use crate::topics::{ActuatorLimits, AutonomousAlgorithmInfo, VEHICLE_LIMITS_TOPIC_NAME, VescCommand};
use crate::{Captain, Executor, Ticker};
use std::any::Any;

/// How often a command is published, in Hz.
const RATE_HZ: f64 = 50.0;

/// Entry point `build.rs` calls - see [`crate::autonomous_control`].
pub fn new(name: &str) -> Box<dyn Executor> {
    Box::new(AlwaysLeft { id: 0, name: name.to_string() })
}

struct AlwaysLeft {
    id: u8,
    name: String,
}

impl Executor for AlwaysLeft {
    fn init(&mut self, id: u8) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_autonomous_control(
            self.id,
            AutonomousAlgorithmInfo::new("Always left", "Full left steering lock at full speed. A structural placeholder."),
        );
    }

    fn run(&mut self, captain: &Captain) {
        let command_topic = captain.autonomous_control(self.id);
        let limits_topic = captain.topic::<ActuatorLimits>(VEHICLE_LIMITS_TOPIC_NAME);
        let mut ticker = Ticker::new(RATE_HZ);

        while captain.is_running(self.id) {
            let limits = limits_topic.read();
            // Negative steers left - see `VescCommand::servo_position_rad`.
            command_topic
                .write(self.id, VescCommand::new(-limits.max_steering_angle_rad, limits.max_speed_mps))
                .expect("lost writer authorization for this algorithm's command topic");
            ticker.wait();
        }
    }

    fn name(&self) -> String {
        self.name.clone()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn fresh(&self) -> Box<dyn Executor> {
        new(&self.name)
    }
}
