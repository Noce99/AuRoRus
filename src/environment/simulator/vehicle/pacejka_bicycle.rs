//! Single-track dynamic bicycle model using the full ("similarity") Pacejka
//! Magic Formula for lateral tire force, in place of
//! [`crate::environment::simulator::vehicle::nonlinear_bicycle`]'s
//! simplified 2-parameter curve: adds the curvature factor `E` and lets the
//! front and rear axles have independently tuned curves, and replaces that
//! model's friction-ellipse combined-slip limit with Pacejka's own
//! weighting-function shape. State is the vehicle's center of gravity (CG)
//! position, heading, and body-frame velocity - the same shape as
//! [`crate::environment::simulator::vehicle::NonlinearBicycleState`], kept
//! as its own type per this module's convention that every model owns its
//! full state independently. See
//! `src/environment/simulator/vehicle/README.md` for the equations and
//! their derivation/rationale; this file is the implementation. [`step`] is
//! the entry point a simulation environment's tick loop is expected to call
//! once per tick.

/// Standard gravity, in meters/second^2 - used to compute each axle's static
/// share of the vehicle's weight before any load transfer is applied.
const GRAVITY_MPS2: f64 = 9.81;

/// Dynamic state of the Pacejka bicycle model at one instant - identical
/// fields to
/// [`crate::environment::simulator::vehicle::NonlinearBicycleState`]; see
/// there for field meanings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PacejkaBicycleState {
    pub x_m: f64,
    pub y_m: f64,
    pub heading_rad: f64,
    pub vx_mps: f64,
    pub vy_mps: f64,
    pub yaw_rate_rad_s: f64,
}

/// Mass, yaw inertia, wheelbase geometry, CG height, front/rear Magic
/// Formula tire curves, combined-slip weighting shape, and drive-force
/// split for the Pacejka bicycle model.
///
/// No [`Default`] is provided on purpose, for the same reason as
/// [`crate::environment::simulator::vehicle::NonlinearTireParams`]: these
/// describe a specific vehicle's real physical properties, and silently
/// defaulting them would silently produce a physically wrong trajectory
/// with no signal that anything is off.
#[derive(Debug, Clone, Copy, PartialEq, serde::Deserialize)]
pub struct PacejkaTireParams {
    /// Vehicle mass, in kilograms.
    pub mass_kg: f64,
    /// Yaw moment of inertia about the vertical axis through the CG, in
    /// kilogram-meters^2.
    pub yaw_inertia_kgm2: f64,
    /// Distance from the CG to the front axle, in meters.
    pub lf_m: f64,
    /// Distance from the CG to the rear axle, in meters.
    pub lr_m: f64,
    /// Height of the CG above the ground, in meters - drives how much
    /// longitudinal acceleration shifts load between the front and rear
    /// axles (see [`normal_loads`]).
    pub cg_height_m: f64,
    /// Front tire Magic Formula stiffness factor (`B`).
    pub front_b: f64,
    /// Front tire Magic Formula shape factor (`C`).
    pub front_c: f64,
    /// Front tire peak friction coefficient - the Magic Formula's `D` is
    /// `front_d_mu * Fz_f`.
    pub front_d_mu: f64,
    /// Front tire Magic Formula curvature factor (`E`). `0.0` reduces the
    /// curve to the simplified formula
    /// [`crate::environment::simulator::vehicle::nonlinear_bicycle`] uses;
    /// real tires are commonly modeled with a negative value.
    pub front_e: f64,
    /// Rear tire Magic Formula stiffness factor (`B`).
    pub rear_b: f64,
    /// Rear tire Magic Formula shape factor (`C`).
    pub rear_c: f64,
    /// Rear tire peak friction coefficient - the Magic Formula's `D` is
    /// `rear_d_mu * Fz_r`.
    pub rear_d_mu: f64,
    /// Rear tire Magic Formula curvature factor (`E`).
    pub rear_e: f64,
    /// Stiffness factor of the combined-slip weighting-function curve (see
    /// [`derivative`]), shared front and rear.
    pub combined_slip_b: f64,
    /// Shape factor of the combined-slip weighting-function curve, shared
    /// front and rear.
    pub combined_slip_c: f64,
    /// Fraction (`0.0..=1.0`) of the commanded longitudinal force delivered
    /// through the front axle; the rest goes to the rear. `0.0` is pure
    /// rear-wheel drive, `1.0` pure front-wheel drive, anything in between
    /// an all-wheel-drive split.
    pub front_drive_fraction: f64,
}

impl PacejkaTireParams {
    /// Basic sanity checks on the parameter values. `front_e`/`rear_e` are
    /// curvature factors meaningful at any real value (commonly negative in
    /// practice), so they're left unconstrained.
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
        if self.cg_height_m <= 0.0 {
            return Err("cg_height_m must be positive".to_string());
        }
        if self.front_b <= 0.0 {
            return Err("front_b must be positive".to_string());
        }
        if self.front_c <= 0.0 {
            return Err("front_c must be positive".to_string());
        }
        if self.front_d_mu <= 0.0 {
            return Err("front_d_mu must be positive".to_string());
        }
        if self.rear_b <= 0.0 {
            return Err("rear_b must be positive".to_string());
        }
        if self.rear_c <= 0.0 {
            return Err("rear_c must be positive".to_string());
        }
        if self.rear_d_mu <= 0.0 {
            return Err("rear_d_mu must be positive".to_string());
        }
        if self.combined_slip_b <= 0.0 {
            return Err("combined_slip_b must be positive".to_string());
        }
        if self.combined_slip_c <= 0.0 {
            return Err("combined_slip_c must be positive".to_string());
        }
        if !(0.0..=1.0).contains(&self.front_drive_fraction) {
            return Err("front_drive_fraction must be between 0.0 and 1.0".to_string());
        }
        Ok(())
    }
}

/// The front/rear axle normal loads (in newtons) under the same quasi-static
/// longitudinal load transfer model as
/// [`crate::environment::simulator::vehicle::nonlinear_bicycle`]: each
/// axle's static share of the vehicle's weight, shifted by
/// `acceleration_mps2 * cg_height_m` acting through the CG height, clamped
/// to never go negative.
fn normal_loads(params: PacejkaTireParams, acceleration_mps2: f64) -> (f64, f64) {
    let wheelbase_m = params.lf_m + params.lr_m;
    let static_fz_f = params.mass_kg * GRAVITY_MPS2 * params.lr_m / wheelbase_m;
    let static_fz_r = params.mass_kg * GRAVITY_MPS2 * params.lf_m / wheelbase_m;
    let transfer_n = params.mass_kg * acceleration_mps2 * params.cg_height_m / wheelbase_m;
    ((static_fz_f - transfer_n).max(0.0), (static_fz_r + transfer_n).max(0.0))
}

/// The full ("similarity") Pacejka Magic Formula for one tire's lateral
/// force, given its slip angle `alpha` (radians) and curve parameters:
///
/// ```text
/// Fy = -d * sin(c * atan(b*alpha - e*(b*alpha - atan(b*alpha))))
/// ```
///
/// `d` is the curve's peak force (a tire's `friction coefficient * normal
/// load`); `b`/`c` are the stiffness/shape factors also used by the
/// simplified curve; `e` is the curvature factor the simplified curve
/// doesn't have - `e = 0.0` makes this identical to that simplified curve
/// (`-d*sin(c*atan(b*alpha))`).
fn pacejka_lateral_force(b: f64, c: f64, d: f64, e: f64, alpha: f64) -> f64 {
    let b_alpha = b * alpha;
    let inner = b_alpha - e * (b_alpha - b_alpha.atan());
    -d * (c * inner.atan()).sin()
}

/// The commanded longitudinal force actually deliverable through one axle,
/// and the combined-slip weighting fraction its lateral force should be
/// scaled by afterward - Pacejka's own weighting-function shape
/// (`cos(C*atan(B*x))`, `x` being the axle's force-usage ratio) in place of
/// [`crate::environment::simulator::vehicle::nonlinear_bicycle`]'s simple
/// friction ellipse. An axle with zero normal load (`d_n <= 0.0`) can
/// deliver no force in any direction, so both results are `0.0`.
fn axle_combined_slip(
    combined_slip_b: f64,
    combined_slip_c: f64,
    d_n: f64,
    demanded_fx_n: f64,
) -> (f64, f64) {
    if d_n <= 0.0 {
        return (0.0, 0.0);
    }
    let fx_n = demanded_fx_n.clamp(-d_n, d_n);
    let weighting = (combined_slip_c * (combined_slip_b * (fx_n / d_n)).atan()).cos().max(0.0);
    (fx_n, weighting)
}

/// Instantaneous rate of change of a [`PacejkaBicycleState`], as returned by
/// [`derivative`] and consumed by [`step`]'s RK4 integration.
#[derive(Debug, Clone, Copy)]
struct PacejkaDerivative {
    dx_dt: f64,
    dy_dt: f64,
    dheading_dt: f64,
    dvx_dt: f64,
    dvy_dt: f64,
    dyaw_rate_dt: f64,
}

/// The instantaneous derivative of `state` under a constant control input.
/// Builds on
/// [`crate::environment::simulator::vehicle::nonlinear_bicycle`]'s load
/// transfer and longitudinal force split, replacing its simplified lateral
/// tire curve and friction-ellipse combined slip with the full Magic
/// Formula (see [`pacejka_lateral_force`]) and Pacejka's own weighting-
/// function combined slip (see [`axle_combined_slip`]).
fn derivative(
    state: PacejkaBicycleState,
    params: PacejkaTireParams,
    steering_angle_rad: f64,
    acceleration_mps2: f64,
) -> PacejkaDerivative {
    let (fz_f, fz_r) = normal_loads(params, acceleration_mps2);
    let d_f = params.front_d_mu * fz_f;
    let d_r = params.rear_d_mu * fz_r;

    let alpha_f = (state.vy_mps + params.lf_m * state.yaw_rate_rad_s).atan2(state.vx_mps) - steering_angle_rad;
    let alpha_r = (state.vy_mps - params.lr_m * state.yaw_rate_rad_s).atan2(state.vx_mps);
    let fyf_raw = pacejka_lateral_force(params.front_b, params.front_c, d_f, params.front_e, alpha_f);
    let fyr_raw = pacejka_lateral_force(params.rear_b, params.rear_c, d_r, params.rear_e, alpha_r);

    let fx_total_n = params.mass_kg * acceleration_mps2;
    let (fx_f, weighting_f) =
        axle_combined_slip(params.combined_slip_b, params.combined_slip_c, d_f, params.front_drive_fraction * fx_total_n);
    let (fx_r, weighting_r) = axle_combined_slip(
        params.combined_slip_b,
        params.combined_slip_c,
        d_r,
        (1.0 - params.front_drive_fraction) * fx_total_n,
    );

    let fyf = fyf_raw * weighting_f;
    let fyr = fyr_raw * weighting_r;
    let ax_achieved = (fx_f + fx_r) / params.mass_kg;
    let cos_delta = steering_angle_rad.cos();

    let (sin_h, cos_h) = state.heading_rad.sin_cos();
    PacejkaDerivative {
        dx_dt: state.vx_mps * cos_h - state.vy_mps * sin_h,
        dy_dt: state.vx_mps * sin_h + state.vy_mps * cos_h,
        dheading_dt: state.yaw_rate_rad_s,
        dvx_dt: ax_achieved + state.vy_mps * state.yaw_rate_rad_s,
        dvy_dt: (fyf * cos_delta + fyr) / params.mass_kg - state.vx_mps * state.yaw_rate_rad_s,
        dyaw_rate_dt: (params.lf_m * fyf * cos_delta - params.lr_m * fyr) / params.yaw_inertia_kgm2,
    }
}

/// `state` advanced linearly by `deriv` scaled by `dt_s` - the building
/// block for combining RK4 stages. Heading is left unwrapped here; [`step`]
/// wraps the final result.
fn advance_state(state: PacejkaBicycleState, deriv: PacejkaDerivative, dt_s: f64) -> PacejkaBicycleState {
    PacejkaBicycleState {
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
/// Runge-Kutta (RK4) integration of the Pacejka bicycle equations (see
/// [`derivative`]). The control input is held constant ("frozen") across the
/// whole `dt_s` interval, same as the other three vehicle models.
///
/// `heading_rad` in the returned state is wrapped to `(-pi, pi]`, for the
/// same reason as the other three models.
///
/// `dt_s` of `0.0` returns `state` unchanged (up to the heading wrap, which
/// is a no-op for an already-wrapped angle to within float precision).
pub fn step(
    state: PacejkaBicycleState,
    params: PacejkaTireParams,
    steering_angle_rad: f64,
    acceleration_mps2: f64,
    dt_s: f64,
) -> PacejkaBicycleState {
    let k1 = derivative(state, params, steering_angle_rad, acceleration_mps2);
    let s2 = advance_state(state, k1, dt_s / 2.0);
    let k2 = derivative(s2, params, steering_angle_rad, acceleration_mps2);
    let s3 = advance_state(state, k2, dt_s / 2.0);
    let k3 = derivative(s3, params, steering_angle_rad, acceleration_mps2);
    let s4 = advance_state(state, k3, dt_s);
    let k4 = derivative(s4, params, steering_angle_rad, acceleration_mps2);

    let mut next = PacejkaBicycleState {
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

    fn test_params() -> PacejkaTireParams {
        PacejkaTireParams {
            mass_kg: 3.5,
            yaw_inertia_kgm2: 0.06,
            lf_m: 0.16,
            lr_m: 0.16,
            cg_height_m: 0.05,
            front_b: 2.5,
            front_c: 1.3,
            front_d_mu: 1.1,
            front_e: -0.5,
            rear_b: 2.5,
            rear_c: 1.3,
            rear_d_mu: 1.1,
            rear_e: -0.5,
            combined_slip_b: 1.0,
            combined_slip_c: 1.0,
            front_drive_fraction: 0.5,
        }
    }

    #[test]
    fn default_geometry_validates() {
        assert!(test_params().validate().is_ok());
    }

    #[test]
    fn non_positive_front_d_mu_is_rejected() {
        let params = PacejkaTireParams { front_d_mu: 0.0, ..test_params() };
        assert!(params.validate().is_err());
    }

    #[test]
    fn front_drive_fraction_out_of_range_is_rejected() {
        let params = PacejkaTireParams { front_drive_fraction: 1.5, ..test_params() };
        assert!(params.validate().is_err());
    }

    #[test]
    fn negative_curvature_factor_still_validates() {
        // e is a curvature factor meaningful at any real value - commonly
        // negative in practice - so it's intentionally unconstrained.
        let params = PacejkaTireParams { front_e: -3.0, rear_e: 2.0, ..test_params() };
        assert!(params.validate().is_ok());
    }

    #[test]
    fn zero_curvature_factor_matches_the_simplified_pacejka_curve() {
        for alpha in [-0.5, -0.1, 0.0, 0.05, 0.3, 0.8] {
            let full = pacejka_lateral_force(2.5, 1.3, 18.9, 0.0, alpha);
            let simplified = -18.9 * (1.3 * (2.5 * alpha).atan()).sin();
            assert!((full - simplified).abs() < 1e-12, "alpha={alpha}: {full} vs {simplified}");
        }
    }

    #[test]
    fn zero_dt_returns_state_unchanged() {
        let state = PacejkaBicycleState {
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
        let state =
            PacejkaBicycleState { x_m: 0.0, y_m: 0.0, heading_rad: 0.0, vx_mps: 5.0, vy_mps: 0.0, yaw_rate_rad_s: 0.0 };
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
        let params = test_params();
        let mut state =
            PacejkaBicycleState { x_m: 0.0, y_m: 0.0, heading_rad: 0.0, vx_mps: 5.0, vy_mps: 0.0, yaw_rate_rad_s: 0.0 };
        let dt_s = 0.001;
        for _ in 0..5000 {
            state = step(state, params, 0.1, 0.0, dt_s);
            assert!(state.yaw_rate_rad_s.is_finite());
            assert!(state.vy_mps.is_finite());
            assert!(state.yaw_rate_rad_s.abs() < 50.0);
            assert!(state.vy_mps.abs() < 50.0);
        }
    }

    #[test]
    fn heavy_acceleration_reduces_cornering_grip_under_constant_steering() {
        let params = test_params();
        let initial =
            PacejkaBicycleState { x_m: 0.0, y_m: 0.0, heading_rad: 0.0, vx_mps: 5.0, vy_mps: 0.0, yaw_rate_rad_s: 0.0 };
        let dt_s = 0.001;
        let steps = 500;

        let mut coasting = initial;
        let mut accelerating = initial;
        for _ in 0..steps {
            coasting = step(coasting, params, 0.2, 0.0, dt_s);
            accelerating = step(accelerating, params, 0.2, 8.0, dt_s);
        }

        assert!(
            accelerating.yaw_rate_rad_s.abs() < coasting.yaw_rate_rad_s.abs(),
            "expected heavy acceleration to reduce cornering response: {} vs {}",
            accelerating.yaw_rate_rad_s,
            coasting.yaw_rate_rad_s
        );
    }
}
