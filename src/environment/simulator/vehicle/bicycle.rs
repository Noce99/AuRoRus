//! CG-referenced kinematic bicycle model: state is the vehicle's center of
//! gravity (CG) position, heading, and forward speed; control inputs are
//! steering angle and longitudinal acceleration, integrated with classical
//! 4th-order Runge-Kutta (RK4). See
//! `src/environment/simulator/vehicle/README.md` for
//! the equations and their derivation/rationale; this file is the
//! implementation. [`step`] is the entry point a simulation environment's
//! tick loop is expected to call once per tick.

/// Kinematic state of the bicycle model at one instant: position of the
/// vehicle's center of gravity (CG), heading, and forward speed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BicycleState {
    /// X coordinate of the CG in world coordinates, in meters.
    pub x_m: f64,
    /// Y coordinate of the CG in world coordinates, in meters.
    pub y_m: f64,
    /// Heading of the vehicle body relative to the world X axis, in
    /// radians, wrapped to `(-pi, pi]` by [`step`].
    pub heading_rad: f64,
    /// Forward speed along the body's heading, in meters/second.
    pub speed_mps: f64,
}

/// Wheelbase geometry for the kinematic bicycle model: how far the front
/// and rear axles sit from the vehicle's center of gravity (CG).
///
/// No [`Default`] is provided on purpose: unlike
/// [`crate::environment::simulator::GenerationConfig`] (which controls
/// synthetic content generation and has no "correct" answer to get wrong),
/// `lf_m`/`lr_m` describe a specific vehicle's real geometry - silently
/// defaulting them would silently produce a physically wrong trajectory
/// with no signal that anything is off. Callers must state them explicitly.
#[derive(Debug, Clone, Copy, PartialEq, serde::Deserialize)]
pub struct BicycleParams {
    /// Distance from the CG to the front axle, in meters.
    pub lf_m: f64,
    /// Distance from the CG to the rear axle, in meters.
    pub lr_m: f64,
}

impl BicycleParams {
    /// Basic sanity checks on the parameter values.
    pub fn validate(&self) -> Result<(), String> {
        if self.lf_m <= 0.0 {
            return Err("lf_m must be positive".to_string());
        }
        if self.lr_m <= 0.0 {
            return Err("lr_m must be positive".to_string());
        }
        Ok(())
    }
}

/// Instantaneous rate of change of a [`BicycleState`], as returned by
/// [`derivative`] and consumed by [`step`]'s RK4 integration.
#[derive(Debug, Clone, Copy)]
struct BicycleDerivative {
    dx_dt: f64,
    dy_dt: f64,
    dheading_dt: f64,
    dspeed_dt: f64,
}

/// The instantaneous derivative of `state` under a constant control input,
/// from the CG-referenced kinematic bicycle equations:
///
/// ```text
/// beta        = atan((lr_m / (lf_m + lr_m)) * tan(steering_angle_rad))
/// dx/dt       = speed_mps * cos(heading_rad + beta)
/// dy/dt       = speed_mps * sin(heading_rad + beta)
/// dheading/dt = (speed_mps / lr_m) * sin(beta)
/// dspeed/dt   = acceleration_mps2
/// ```
///
/// `beta` is the slip angle: the angle between the vehicle's heading and
/// its actual CG velocity direction, caused by the front wheel steering
/// while the model tracks the CG's motion rather than the rear axle's (the
/// rear-axle-referenced variant of this model has no slip angle term).
fn derivative(
    state: BicycleState,
    params: BicycleParams,
    steering_angle_rad: f64,
    acceleration_mps2: f64,
) -> BicycleDerivative {
    let beta = ((params.lr_m / (params.lf_m + params.lr_m)) * steering_angle_rad.tan()).atan();
    let (sin_hb, cos_hb) = (state.heading_rad + beta).sin_cos();
    BicycleDerivative {
        dx_dt: state.speed_mps * cos_hb,
        dy_dt: state.speed_mps * sin_hb,
        dheading_dt: (state.speed_mps / params.lr_m) * beta.sin(),
        dspeed_dt: acceleration_mps2,
    }
}

/// `state` advanced linearly by `deriv` scaled by `dt_s` - the building
/// block for combining RK4 stages. Heading is left unwrapped here; [`step`]
/// wraps the final result.
fn advance_state(state: BicycleState, deriv: BicycleDerivative, dt_s: f64) -> BicycleState {
    BicycleState {
        x_m: state.x_m + deriv.dx_dt * dt_s,
        y_m: state.y_m + deriv.dy_dt * dt_s,
        heading_rad: state.heading_rad + deriv.dheading_dt * dt_s,
        speed_mps: state.speed_mps + deriv.dspeed_dt * dt_s,
    }
}

/// Wraps an angle in radians to `(-pi, pi]`.
fn wrap_to_pi(angle_rad: f64) -> f64 {
    angle_rad.sin().atan2(angle_rad.cos())
}

/// Advances `state` by one `dt_s` step under the constant control input
/// (`steering_angle_rad`, `acceleration_mps2`), using classical 4th-order
/// Runge-Kutta (RK4) integration of the kinematic bicycle equations (see
/// [`derivative`]).
///
/// The control input is held constant ("frozen") across the whole `dt_s`
/// interval, including at the RK4 substep evaluations - this matches a
/// tick-based simulation environment that supplies exactly one control
/// sample per tick and expects the model to integrate across it.
///
/// `heading_rad` in the returned state is wrapped to `(-pi, pi]`; this is
/// safe because the equations only ever consume `heading_rad` through
/// `sin`/`cos` (periodic), so wrapping never changes the dynamics, and it
/// keeps the value from growing unbounded over a long-running simulation.
///
/// `dt_s` of `0.0` returns `state` unchanged (up to the heading wrap, which
/// is a no-op for an already-wrapped angle to within float precision).
pub fn step(
    state: BicycleState,
    params: BicycleParams,
    steering_angle_rad: f64,
    acceleration_mps2: f64,
    dt_s: f64,
) -> BicycleState {
    let k1 = derivative(state, params, steering_angle_rad, acceleration_mps2);
    let s2 = advance_state(state, k1, dt_s / 2.0);
    let k2 = derivative(s2, params, steering_angle_rad, acceleration_mps2);
    let s3 = advance_state(state, k2, dt_s / 2.0);
    let k3 = derivative(s3, params, steering_angle_rad, acceleration_mps2);
    let s4 = advance_state(state, k3, dt_s);
    let k4 = derivative(s4, params, steering_angle_rad, acceleration_mps2);

    let mut next = BicycleState {
        x_m: state.x_m + (dt_s / 6.0) * (k1.dx_dt + 2.0 * k2.dx_dt + 2.0 * k3.dx_dt + k4.dx_dt),
        y_m: state.y_m + (dt_s / 6.0) * (k1.dy_dt + 2.0 * k2.dy_dt + 2.0 * k3.dy_dt + k4.dy_dt),
        heading_rad: state.heading_rad
            + (dt_s / 6.0)
                * (k1.dheading_dt + 2.0 * k2.dheading_dt + 2.0 * k3.dheading_dt + k4.dheading_dt),
        speed_mps: state.speed_mps
            + (dt_s / 6.0)
                * (k1.dspeed_dt + 2.0 * k2.dspeed_dt + 2.0 * k3.dspeed_dt + k4.dspeed_dt),
    };
    next.heading_rad = wrap_to_pi(next.heading_rad);
    next
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_params() -> BicycleParams {
        BicycleParams {
            lf_m: 1.0,
            lr_m: 1.0,
        }
    }

    #[test]
    fn default_geometry_validates() {
        assert!(test_params().validate().is_ok());
    }

    #[test]
    fn non_positive_lf_is_rejected() {
        let params = BicycleParams {
            lf_m: 0.0,
            ..test_params()
        };
        assert!(params.validate().is_err());
    }

    #[test]
    fn zero_dt_returns_state_unchanged() {
        let state = BicycleState {
            x_m: 1.0,
            y_m: 2.0,
            heading_rad: 0.4,
            speed_mps: 3.0,
        };
        let next = step(state, test_params(), 0.2, 1.0, 0.0);
        assert!((next.x_m - state.x_m).abs() < 1e-12);
        assert!((next.y_m - state.y_m).abs() < 1e-12);
        assert!((next.heading_rad - state.heading_rad).abs() < 1e-12);
        assert!((next.speed_mps - state.speed_mps).abs() < 1e-12);
    }

    #[test]
    fn straight_line_zero_steering_matches_simple_kinematics() {
        // beta = 0 when steering_angle_rad = 0, so this reduces to
        // constant-velocity straight-line motion with a=0; RK4 is exact
        // (zero truncation error) for a constant derivative.
        let state = BicycleState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            speed_mps: 5.0,
        };
        let next = step(state, test_params(), 0.0, 0.0, 2.0);
        assert!((next.x_m - 10.0).abs() < 1e-9);
        assert!(next.y_m.abs() < 1e-9);
        assert!(next.heading_rad.abs() < 1e-9);
        assert!((next.speed_mps - 5.0).abs() < 1e-9);
    }

    #[test]
    fn constant_steering_traces_a_circle_of_the_closed_form_radius() {
        // With a=0 and constant steering_angle_rad, beta and speed_mps are
        // both constant, so dheading/dt is constant too: the CG traces a
        // circle of radius R = lr_m / sin(beta) at constant speed - a
        // standard result for the CG-referenced kinematic bicycle model.
        let params = test_params();
        let steering_angle_rad: f64 = 0.3;
        let beta = ((params.lr_m / (params.lf_m + params.lr_m)) * steering_angle_rad.tan()).atan();
        let radius = params.lr_m / beta.sin();

        let mut state = BicycleState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            speed_mps: 5.0,
        };
        // Center of the circle, computed once from the initial state.
        let phi0 = state.heading_rad + beta;
        let center_x = state.x_m - radius * phi0.sin();
        let center_y = state.y_m + radius * phi0.cos();

        let dt_s = 0.001;
        for _ in 0..2000 {
            state = step(state, params, steering_angle_rad, 0.0, dt_s);
            let dist = ((state.x_m - center_x).powi(2) + (state.y_m - center_y).powi(2)).sqrt();
            assert!((dist - radius.abs()).abs() < 1e-3, "{dist} vs {radius}");
        }
    }
}
