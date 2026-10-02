//! Helpers shared by several algorithms. They live in `shared/`, not in
//! `.rs` files next to the algorithms, because `build.rs` turns every `.rs`
//! file directly in `src/autonomous_control/` - except this one - into an
//! algorithm.

pub(crate) mod frenet;
pub(crate) mod mpc;
pub(crate) mod reactive;
pub(crate) mod steering;
