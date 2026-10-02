//! Shared topic definitions: the concrete types published/read on named
//! [`crate::RwLockTopic`]s, so multiple binaries can agree on the same shape
//! without redefining it.

mod autonomy;
mod drawing;
mod map;
mod race;
mod sensing;
mod vehicle;
mod vesc;

pub use autonomy::*;
pub use drawing::{Color, DRAW_TOPIC_PREFIX, Drawing, DrawingElement, DrawingExt, Shape};
pub use map::*;
pub use race::*;
pub use sensing::*;
pub use vehicle::*;
pub use vesc::*;
