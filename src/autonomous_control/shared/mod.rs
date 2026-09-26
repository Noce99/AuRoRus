//! Helpers shared by several algorithms. They live in a directory, not in
//! `.rs` files next to the algorithms, because `build.rs` turns every `.rs`
//! file directly in `src/autonomous_control/` into an algorithm.

pub(crate) mod race_line;
pub(crate) mod reactive;
