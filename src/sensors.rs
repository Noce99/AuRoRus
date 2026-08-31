//! Sensor drivers and other standalone [`crate::Executor`]s: things that run
//! independently on their own thread against a shared [`crate::Captain`],
//! whether or not they publish to a topic - a sensor driver publishes
//! real-world readings onto a topic ([`RandomLidar`], [`MapServer`]), while
//! [`WebGui`] instead serves a local web UI.

mod map_server;
mod random_lidar;
mod web_gui;

pub use map_server::{MapServer, MapServerConfig};
pub use random_lidar::{RandomLidar, RandomLidarConfig};
pub use web_gui::{WebGui, WebGuiConfig};
