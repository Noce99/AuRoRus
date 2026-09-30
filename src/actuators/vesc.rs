//! The real car's motor controller, a VESC 6 MkV, over its USB serial port:
//! the firmware's packet protocol ([`protocol`]), a port reading the VESC's
//! state, moving its steering servo and driving its motor ([`VescPort`]),
//! the car's calibration ([`VescConfig`]) and the [`Vesc`] executor driving
//! the car with them.

mod control;
#[cfg(unix)]
mod driver;
#[cfg(unix)]
mod port;
pub mod protocol;

pub use control::{ImuAxis, VescConfig, battery_charge};
#[cfg(unix)]
pub use driver::Vesc;
#[cfg(unix)]
pub use port::VescPort;
