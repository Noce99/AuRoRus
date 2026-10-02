//! Plane geometry shared across the crate: points and closed loops of them
//! ([`smoothing`], [`closed_loop`]), poses ([`Pose`]), and a race line a
//! pose is projected onto ([`Line`]).

pub(crate) mod closed_loop;
mod line;
mod pose;
mod smoothing;

pub(crate) use closed_loop::{
    curvature_jacobian, curvatures, left_normal, loop_length, max_abs_curvature, orient_from,
    smooth, tangents,
};
pub(crate) use line::{Line, Nearest};
pub(crate) use pose::{Pose, wrap_to_pi};
pub(crate) use smoothing::{Point2, densify, resample_even_spacing};
