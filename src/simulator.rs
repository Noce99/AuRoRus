//! The environment/vehicle simulator: lets algorithms and executors be
//! exercised against synthetic tracks and vehicle dynamics instead of real
//! hardware. See [`environment`] for random map generation; [`vehicle`] is
//! reserved for a future simulated vehicle model.

pub mod environment;
pub mod vehicle;
