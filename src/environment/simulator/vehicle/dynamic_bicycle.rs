//! Single-track dynamic bicycle model with a linear tire model: state is the
//! vehicle's center of gravity (CG) position, heading, and body-frame
//! velocity (longitudinal, lateral, yaw rate); control inputs are steering
//! angle and longitudinal acceleration, integrated with classical 4th-order
//! Runge-Kutta (RK4). See
//! `src/environment/simulator/vehicle/README.md` for the equations and their
//! derivation/rationale; this file is the implementation. [`step`] is the
//! entry point a simulation environment's tick loop is expected to call once
//! per tick.

/// Dynamic state of the bicycle model at one instant: position of the
/// vehicle's center of gravity (CG), heading, and body-frame velocity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DynamicState {
    /// X coordinate of the CG in world coordinates, in meters.
    pub x_m: f64,
    /// Y coordinate of the CG in world coordinates, in meters.
    pub y_m: f64,
    /// Heading of the vehicle body relative to the world X axis, in
    /// radians, wrapped to `(-pi, pi]` by [`step`].
    pub heading_rad: f64,
    /// Longitudinal velocity in the body frame (along the heading), in
    /// meters/second.
    pub vx_mps: f64,
    /// Lateral velocity in the body frame (perpendicular to the heading), in
    /// meters/second.
    pub vy_mps: f64,
    /// Yaw rate (rate of change of `heading_rad`), in radians/second.
    pub yaw_rate_rad_s: f64,
}

/// Mass, yaw inertia, wheelbase geometry, and linear tire cornering
/// stiffnesses for the dynamic bicycle model.
///
/// No [`Default`] is provided on purpose, for the same reason as
/// [`crate::environment::simulator::vehicle::BicycleParams`]: these describe
/// a specific vehicle's real physical properties, and silently defaulting
/// them would silently produce a physically wrong trajectory with no signal
/// that anything is off.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DynamicParams {
    /// Vehicle mass, in kilograms.
    pub mass_kg: f64,
    /// Yaw moment of inertia about the vertical axis through the CG, in
    /// kilogram-meters^2.
    pub yaw_inertia_kgm2: f64,
    /// Distance from the CG to the front axle, in meters.
    pub lf_m: f64,
    /// Distance from the CG to the rear axle, in meters.
    pub lr_m: f64,
    /// Front tire cornering stiffness (lateral force per radian of slip
    /// angle), in newtons/radian.
    pub cf_n_per_rad: f64,
    /// Rear tire cornering stiffness (lateral force per radian of slip
    /// angle), in newtons/radian.
    pub cr_n_per_rad: f64,
}

impl DynamicParams {
    /// Basic sanity checks on the parameter values.
    pub fn validate(&self) -> Result<(), String> {
        if self.mass_kg <= 0.0 {
            return Err("mass_kg must be positive".to_string());
        }
        if self.yaw_inertia_kgm2 <= 0.0 {
            return Err("yaw_inertia_kgm2 must be positive".to_string());
        }
        if self.lf_m <= 0.0 {
            return Err("lf_m must be positive".to_string());
        }
        if self.lr_m <= 0.0 {
            return Err("lr_m must be positive".to_string());
        }
        if self.cf_n_per_rad <= 0.0 {
            return Err("cf_n_per_rad must be positive".to_string());
        }
        if self.cr_n_per_rad <= 0.0 {
            return Err("cr_n_per_rad must be positive".to_string());
        }
        Ok(())
    }
}

/// Instantaneous rate of change of a [`DynamicState`], as returned by
/// [`derivative`] and consumed by [`step`]'s RK4 integration.
#[derive(Debug, Clone, Copy)]
struct DynamicDerivative {
    dx_dt: f64,
    dy_dt: f64,
    dheading_dt: f64,
    dvx_dt: f64,
    dvy_dt: f64,
    dyaw_rate_dt: f64,
}

/// The instantaneous derivative of `state` under a constant control input,
/// from the linear-tire single-track dynamic bicycle equations:
///
/// ```text
/// alpha_f = atan2(vy_mps + lf_m*yaw_rate_rad_s, vx_mps) - steering_angle_rad
/// alpha_r = atan2(vy_mps - lr_m*yaw_rate_rad_s, vx_mps)
/// Fyf     = -cf_n_per_rad * alpha_f
/// Fyr     = -cr_n_per_rad * alpha_r
///
/// dx/dt        = vx_mps*cos(heading_rad) - vy_mps*sin(heading_rad)
/// dy/dt        = vx_mps*sin(heading_rad) + vy_mps*cos(heading_rad)
/// dheading/dt  = yaw_rate_rad_s
/// dvx/dt       = acceleration_mps2 + vy_mps*yaw_rate_rad_s
/// dvy/dt       = (Fyf*cos(steering_angle_rad) + Fyr)/mass_kg - vx_mps*yaw_rate_rad_s
/// dyaw_rate/dt = (lf_m*Fyf*cos(steering_angle_rad) - lr_m*Fyr)/yaw_inertia_kgm2
/// ```
///
/// `alpha_f`/`alpha_r` are the front/rear tire slip angles: the angle
/// between each axle's heading and its actual velocity direction. Unlike
/// [`crate::environment::simulator::vehicle::bicycle`]'s purely geometric
/// slip angle, these come from the vehicle's actual body-frame velocity
/// (`vx_mps`, `vy_mps`, `yaw_rate_rad_s`), which is what lets this model
/// produce real lateral tire forces (`Fyf`, `Fyr`) via a linear tire model,
/// rather than assuming the tires can always deliver whatever lateral
/// motion the geometry implies.
fn derivative(
    state: DynamicState,
    params: DynamicParams,
    steering_angle_rad: f64,
    acceleration_mps2: f64,
) -> DynamicDerivative {
    let alpha_f = (state.vy_mps + params.lf_m * state.yaw_rate_rad_s).atan2(state.vx_mps) - steering_angle_rad;
    let alpha_r = (state.vy_mps - params.lr_m * state.yaw_rate_rad_s).atan2(state.vx_mps);
    let fyf = -params.cf_n_per_rad * alpha_f;
    let fyr = -params.cr_n_per_rad * alpha_r;
    let cos_delta = steering_angle_rad.cos();

    let (sin_h, cos_h) = state.heading_rad.sin_cos();
    DynamicDerivative {
        dx_dt: state.vx_mps * cos_h - state.vy_mps * sin_h,
        dy_dt: state.vx_mps * sin_h + state.vy_mps * cos_h,
        dheading_dt: state.yaw_rate_rad_s,
        dvx_dt: acceleration_mps2 + state.vy_mps * state.yaw_rate_rad_s,
        dvy_dt: (fyf * cos_delta + fyr) / params.mass_kg - state.vx_mps * state.yaw_rate_rad_s,
        dyaw_rate_dt: (params.lf_m * fyf * cos_delta - params.lr_m * fyr) / params.yaw_inertia_kgm2,
    }
}

/// `state` advanced linearly by `deriv` scaled by `dt_s` - the building
/// block for combining RK4 stages. Heading is left unwrapped here; [`step`]
/// wraps the final result.
fn advance_state(state: DynamicState, deriv: DynamicDerivative, dt_s: f64) -> DynamicState {
    DynamicState {
        x_m: state.x_m + deriv.dx_dt * dt_s,
        y_m: state.y_m + deriv.dy_dt * dt_s,
        heading_rad: state.heading_rad + deriv.dheading_dt * dt_s,
        vx_mps: state.vx_mps + deriv.dvx_dt * dt_s,
        vy_mps: state.vy_mps + deriv.dvy_dt * dt_s,
        yaw_rate_rad_s: state.yaw_rate_rad_s + deriv.dyaw_rate_dt * dt_s,
    }
}

/// Wraps an angle in radians to `(-pi, pi]`.
fn wrap_to_pi(angle_rad: f64) -> f64 {
    angle_rad.sin().atan2(angle_rad.cos())
}

/// Advances `state` by one `dt_s` step under the constant control input
/// (`steering_angle_rad`, `acceleration_mps2`), using classical 4th-order
/// Runge-Kutta (RK4) integration of the dynamic bicycle equations (see
/// [`derivative`]). The control input is held constant ("frozen") across the
/// whole `dt_s` interval, same as
/// [`crate::environment::simulator::vehicle::bicycle::step`].
///
/// `heading_rad` in the returned state is wrapped to `(-pi, pi]`, for the
/// same reason as the kinematic bicycle model.
///
/// `dt_s` of `0.0` returns `state` unchanged (up to the heading wrap, which
/// is a no-op for an already-wrapped angle to within float precision).
pub fn step(
    state: DynamicState,
    params: DynamicParams,
    steering_angle_rad: f64,
    acceleration_mps2: f64,
    dt_s: f64,
) -> DynamicState {
    let k1 = derivative(state, params, steering_angle_rad, acceleration_mps2);
    let s2 = advance_state(state, k1, dt_s / 2.0);
    let k2 = derivative(s2, params, steering_angle_rad, acceleration_mps2);
    let s3 = advance_state(state, k2, dt_s / 2.0);
    let k3 = derivative(s3, params, steering_angle_rad, acceleration_mps2);
    let s4 = advance_state(state, k3, dt_s);
    let k4 = derivative(s4, params, steering_angle_rad, acceleration_mps2);

    let mut next = DynamicState {
        x_m: state.x_m + (dt_s / 6.0) * (k1.dx_dt + 2.0 * k2.dx_dt + 2.0 * k3.dx_dt + k4.dx_dt),
        y_m: state.y_m + (dt_s / 6.0) * (k1.dy_dt + 2.0 * k2.dy_dt + 2.0 * k3.dy_dt + k4.dy_dt),
        heading_rad: state.heading_rad
            + (dt_s / 6.0)
                * (k1.dheading_dt + 2.0 * k2.dheading_dt + 2.0 * k3.dheading_dt + k4.dheading_dt),
        vx_mps: state.vx_mps + (dt_s / 6.0) * (k1.dvx_dt + 2.0 * k2.dvx_dt + 2.0 * k3.dvx_dt + k4.dvx_dt),
        vy_mps: state.vy_mps + (dt_s / 6.0) * (k1.dvy_dt + 2.0 * k2.dvy_dt + 2.0 * k3.dvy_dt + k4.dvy_dt),
        yaw_rate_rad_s: state.yaw_rate_rad_s
            + (dt_s / 6.0)
                * (k1.dyaw_rate_dt + 2.0 * k2.dyaw_rate_dt + 2.0 * k3.dyaw_rate_dt + k4.dyaw_rate_dt),
    };
    next.heading_rad = wrap_to_pi(next.heading_rad);
    next
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_params() -> DynamicParams {
        DynamicParams {
            mass_kg: 3.5,
            yaw_inertia_kgm2: 0.06,
            lf_m: 0.16,
            lr_m: 0.16,
            cf_n_per_rad: 60.0,
            cr_n_per_rad: 60.0,
        }
    }

    #[test]
    fn default_geometry_validates() {
        assert!(test_params().validate().is_ok());
    }

    #[test]
    fn non_positive_mass_is_rejected() {
        let params = DynamicParams { mass_kg: 0.0, ..test_params() };
        assert!(params.validate().is_err());
    }

    #[test]
    fn zero_dt_returns_state_unchanged() {
        let state = DynamicState {
            x_m: 1.0,
            y_m: 2.0,
            heading_rad: 0.4,
            vx_mps: 3.0,
            vy_mps: 0.1,
            yaw_rate_rad_s: 0.05,
        };
        let next = step(state, test_params(), 0.2, 1.0, 0.0);
        assert!((next.x_m - state.x_m).abs() < 1e-12);
        assert!((next.y_m - state.y_m).abs() < 1e-12);
        assert!((next.heading_rad - state.heading_rad).abs() < 1e-12);
        assert!((next.vx_mps - state.vx_mps).abs() < 1e-12);
        assert!((next.vy_mps - state.vy_mps).abs() < 1e-12);
        assert!((next.yaw_rate_rad_s - state.yaw_rate_rad_s).abs() < 1e-12);
    }

    #[test]
    fn straight_line_zero_steering_and_no_slip_matches_simple_kinematics() {
        // With vy_mps=0, yaw_rate_rad_s=0, and zero steering, both slip
        // angles are zero, so both tire forces are zero and the state stays
        // exactly on a straight line - RK4 is exact (zero truncation error)
        // for a constant derivative.
        let state = DynamicState { x_m: 0.0, y_m: 0.0, heading_rad: 0.0, vx_mps: 5.0, vy_mps: 0.0, yaw_rate_rad_s: 0.0 };
        let next = step(state, test_params(), 0.0, 0.0, 2.0);
        assert!((next.x_m - 10.0).abs() < 1e-9);
        assert!(next.y_m.abs() < 1e-9);
        assert!(next.heading_rad.abs() < 1e-9);
        assert!((next.vx_mps - 5.0).abs() < 1e-9);
        assert!(next.vy_mps.abs() < 1e-9);
        assert!(next.yaw_rate_rad_s.abs() < 1e-9);
    }

    #[test]
    fn constant_steering_settles_into_a_bounded_steady_turn() {
        // A known limitation of this model (see the vehicle README) is that
        // it isn't valid near zero forward speed, so this starts at a
        // reasonable cruising speed rather than from a stop. It doesn't
        // assert a closed-form radius (unlike the kinematic model's
        // equivalent test) since the dynamic model's turn radius isn't a
        // simple closed form - only that constant steering input produces a
        // finite, bounded response rather than diverging.
        let params = test_params();
        let mut state = DynamicState { x_m: 0.0, y_m: 0.0, heading_rad: 0.0, vx_mps: 5.0, vy_mps: 0.0, yaw_rate_rad_s: 0.0 };
        let dt_s = 0.001;
        for _ in 0..5000 {
            state = step(state, params, 0.1, 0.0, dt_s);
            assert!(state.yaw_rate_rad_s.is_finite());
            assert!(state.vy_mps.is_finite());
            assert!(state.yaw_rate_rad_s.abs() < 50.0);
            assert!(state.vy_mps.abs() < 50.0);
        }
    }
}
