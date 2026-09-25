//! Planning a race line for a map: the minimum-curvature line through its
//! track, kept a vehicle's half width (plus a margin) from both walls,
//! with a speed profile - saved as the map's `race_lines/race_line.csv`.
//! See documentation/planning.md.
//!
//! The pipeline ([`plan`]):
//! 1. [`track`]: the drivable pixels connected to the start/finish line,
//!    which must loop around exactly one inner wall.
//! 2. A reference line: the map's centerline, or - for a map without one,
//!    e.g. saved by [`crate::localization::Slam`] - one computed from the
//!    walls ([`centerline`]).
//! 3. [`min_curvature`]: every point moved sideways to minimize the lap's
//!    squared curvature, solved with OpEn's PANOC.
//! 4. [`speed_profile`]: a speed for every point, within lateral and
//!    longitudinal acceleration limits.
//!
//! [`Planner`] runs it on request from `web_gui`'s Planning panel, with
//! parameters tuned live from there (see [`PlanningConfig`]).

mod centerline;
mod config;
mod geometry;
mod min_curvature;
mod pipeline;
mod planner;
mod speed_profile;
mod track;

pub use config::{PlanningConfig, config_path, save_parameters, saved_values, tunable_parameters};
pub use pipeline::{PlannedLines, Progress, plan};
pub use planner::Planner;

/// Why a race line couldn't be planned.
#[derive(Debug, Clone, PartialEq)]
pub enum PlanError {
    /// The start pose (the start/finish line's midpoint) isn't on a
    /// drivable pixel.
    StartOffTrack { x_m: f64, y_m: f64 },
    /// The track doesn't loop around exactly one inner wall: `walls` walls
    /// touch it (e.g. `1`: no island to loop around; `3`: an obstacle on
    /// the track).
    NotALoop { walls: usize },
    /// No closed centerline could be traced between the walls.
    NoCenterline,
    /// The track is narrower than the vehicle needs at (`x_m`, `y_m`).
    TooNarrow {
        x_m: f64,
        y_m: f64,
        width_m: f64,
        needed_m: f64,
    },
    /// The optimizer failed.
    Solver(String),
    /// The optimizer didn't converge within its iteration budget.
    NotConverged { iterations: usize },
    /// Stopped before finishing.
    Cancelled,
}

impl std::fmt::Display for PlanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::StartOffTrack { x_m, y_m } => write!(
                f,
                "the start/finish line's midpoint ({x_m:.2}, {y_m:.2}) m isn't on the track"
            ),
            Self::NotALoop { walls } => write!(
                f,
                "the track must loop around exactly one inner wall, but {walls} wall{} touch it - \
                 clean the map up (close gaps, remove obstacles)",
                if *walls == 1 { "" } else { "s" }
            ),
            Self::NoCenterline => {
                write!(f, "no closed centerline could be traced between the walls")
            }
            Self::TooNarrow {
                x_m,
                y_m,
                width_m,
                needed_m,
            } => write!(
                f,
                "the track is {width_m:.2} m wide at ({x_m:.2}, {y_m:.2}) m, but the vehicle needs \
                 {needed_m:.2} m - lower the vehicle width or the safety margin"
            ),
            Self::Solver(err) => write!(f, "the optimizer failed: {err}"),
            Self::NotConverged { iterations } => write!(
                f,
                "the optimizer didn't converge in {iterations} iterations - raise \
                 solver_max_iterations, or the spacing"
            ),
            Self::Cancelled => write!(f, "cancelled"),
        }
    }
}

impl std::error::Error for PlanError {}
