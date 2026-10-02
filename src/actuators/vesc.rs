//! The real car's motor controller, a VESC 6 MkV, over its USB serial port:
//! the firmware's packet protocol ([`protocol`]), a port reading the VESC's
//! state, moving its steering servo and driving its motor ([`VescPort`]),
//! how the car is driven ([`VescConfig`]) and the [`Vesc`] executor driving
//! the car with them and its calibration ([`crate::calibration::CarCalibration`]).

mod control;
#[cfg(unix)]
mod driver;
#[cfg(unix)]
mod port;
pub mod protocol;

pub use control::{
    VescConfig, VescLimits, battery_charge, config_path, low_battery, save_parameters, saved_values,
};
#[cfg(unix)]
pub use driver::Vesc;
#[cfg(unix)]
pub use port::VescPort;
