//! The VESC motor controller: the commands it is sent, what it reports, and its
//! live-tunable parameters.

mod command;
mod parameters;
mod status;

pub use command::{
    AUTONOMOUS_VESC_COMMAND_TOPIC_NAME, HUMAN_VESC_COMMAND_TOPIC_NAME,
    JOYSTICK_VESC_COMMAND_TOPIC_NAME, VESC_COMMAND_TIMEOUT, VescCommand,
};
pub use parameters::{
    VESC_PARAMETERS_STATUS_TOPIC_NAME, VESC_PARAMETERS_TOPIC_NAME, VescParameters,
    VescParametersStatus,
};
pub use status::{VESC_STATUS_TOPIC_NAME, VescStatus};
