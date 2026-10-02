//! Drivers for the real car's input devices: [`crate::Executor`]s that
//! publish what the hardware reads onto a topic - [`HokuyoLidar`]'s scans -
//! or what a human asks for - [`Joystick`], a gamepad to drive with. Their
//! simulated counterparts are in [`crate::simulation`].

mod hokuyo_lidar;
#[cfg(unix)]
mod joystick;

pub use hokuyo_lidar::{HokuyoLidar, HokuyoLidarConfig, Mounting as LidarMounting};
#[cfg(unix)]
pub use joystick::{Joystick, JoystickConfig};
