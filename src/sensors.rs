//! Sensor drivers and other standalone [`crate::Executor`]s: things that run
//! independently on their own thread against a shared [`crate::Captain`],
//! whether or not they publish to a topic - a sensor driver publishes
//! real-world readings onto a topic ([`HokuyoLidar`], [`RandomLidar`],
//! [`SimulatedLidar`], [`SimulatedImu`], [`MapServer`],
//! [`RaceLinePublisher`]), while [`WebGui`] instead serves a local web UI and
//! [`Joystick`] reads a gamepad a human drives with.

mod hokuyo_lidar;
#[cfg(unix)]
mod joystick;
mod map_server;
mod race_line_publisher;
mod random_lidar;
mod simulated_imu;
mod simulated_lidar;
mod web_gui;

pub use hokuyo_lidar::{HokuyoLidar, HokuyoLidarConfig, Mounting as LidarMounting};
#[cfg(unix)]
pub use joystick::{Joystick, JoystickConfig};
pub use map_server::{MapServer, MapServerConfig};
pub use race_line_publisher::RaceLinePublisher;
pub use random_lidar::{RandomLidar, RandomLidarConfig};
pub use simulated_imu::{SimulatedImu, SimulatedImuConfig};
pub(crate) use simulated_lidar::cast_ray;
pub use simulated_lidar::{SimulatedLidar, SimulatedLidarConfig};
pub use web_gui::{BenchmarkSetup, WebGui, WebGuiConfig};
