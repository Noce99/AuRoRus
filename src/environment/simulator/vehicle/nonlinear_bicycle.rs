//! Single-track dynamic bicycle model with a nonlinear, load-dependent tire
//! model: adds tire force saturation, longitudinal load transfer, and a
//! friction-ellipse combined-slip limit on top of
//! [`crate::environment::simulator::vehicle::dynamic_bicycle`]'s linear
//! tire model. State is the vehicle's center of gravity (CG) position,
//! heading, and body-frame velocity - the same shape as
//! [`crate::environment::simulator::vehicle::DynamicState`], kept as its own
//! type per this module's convention that every model owns its full state
//! independently. See `src/environment/simulator/vehicle/README.md` for the
//! equations and their derivation/rationale; this file is the
//! implementation. [`step`] is the entry point a simulation environment's
//! tick loop is expected to call once per tick.

use super::tunable;
use crate::topics::AlgorithmParameter;

/// Standard gravity, in meters/second^2 - used to compute each axle's static
/// share of the vehicle's weight before any load transfer is applied.
const GRAVITY_MPS2: f64 = 9.81;

/// Dynamic state of the nonlinear bicycle model at one instant - identical
/// fields to
/// [`crate::environment::simulator::vehicle::DynamicState`]; see there for
/// field meanings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NonlinearBicycleState {
    pub x_m: f64,
    pub y_m: f64,
    pub heading_rad: f64,
    pub vx_mps: f64,
    pub vy_mps: f64,
    pub yaw_rate_rad_s: f64,
}

/// Mass, yaw inertia, wheelbase geometry, CG height, tire friction/shape,
/// and drive-force split for the nonlinear bicycle model.
///
/// No [`Default`] is provided on purpose, for the same reason as
/// [`crate::environment::simulator::vehicle::DynamicParams`]: these describe
/// a specific vehicle's real physical properties, and silently defaulting
/// them would silently produce a physically wrong trajectory with no signal
/// that anything is off.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct NonlinearTireParams {
    /// Vehicle mass, in kilograms.
    #[serde(default)]
    pub mass_kg: f64,
    /// Yaw moment of inertia about the vertical axis through the CG, in
    /// kilogram-meters^2.
    pub yaw_inertia_kgm2: f64,
    /// Distance from the CG to the front axle, in meters.
    #[serde(default)]
    pub lf_m: f64,
    /// Distance from the CG to the rear axle, in meters.
    #[serde(default)]
    pub lr_m: f64,
    /// Height of the CG above the ground, in meters - drives how much
    /// longitudinal acceleration shifts load between the front and rear
    /// axles (see [`normal_loads`]).
    pub cg_height_m: f64,
    /// Peak tire/road friction coefficient, shared by the front and rear
    /// tires (a deliberate simplification - see the module README).
    pub tire_mu: f64,
    /// Stiffness factor of the simplified Pacejka lateral force curve (see
    /// [`derivative`]), shared front and rear.
    pub pacejka_b: f64,
    /// Shape factor of the simplified Pacejka lateral force curve, shared
    /// front and rear.
    pub pacejka_c: f64,
    /// Fraction (`0.0..=1.0`) of the commanded longitudinal force delivered
    /// through the front axle; the rest goes to the rear. `0.0` is pure
    /// rear-wheel drive, `1.0` pure front-wheel drive, anything in between
    /// an all-wheel-drive split.
    pub front_drive_fraction: f64,
}

impl NonlinearTireParams {
    /// The live-tunable parameters, one per field - see
    /// [`crate::actuators::SimulatedVehicle`].
    pub fn tunable_parameters() -> Vec<AlgorithmParameter> {
        vec![
            tunable::yaw_inertia_kgm2(),
            tunable::cg_height_m(),
            AlgorithmParameter::float("tire_mu", 0.1, 2.0, 0.05)
                .description("Peak tire/road friction coefficient, front and rear."),
            AlgorithmParameter::float("pacejka_b", 0.5, 15.0, 0.1)
                .description("Stiffness factor of the simplified Pacejka curve."),
            AlgorithmParameter::float("pacejka_c", 0.5, 3.0, 0.05)
                .description("Shape factor of the simplified Pacejka curve."),
            tunable::front_drive_fraction(),
        ]
    }

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
        if self.cg_height_m <= 0.0 {
            return Err("cg_height_m must be positive".to_string());
        }
        if self.tire_mu <= 0.0 {
            return Err("tire_mu must be positive".to_string());
        }
        if self.pacejka_b <= 0.0 {
            return Err("pacejka_b must be positive".to_string());
        }
        if self.pacejka_c <= 0.0 {
            return Err("pacejka_c must be positive".to_string());
        }
        if !(0.0..=1.0).contains(&self.front_drive_fraction) {
            return Err("front_drive_fraction must be between 0.0 and 1.0".to_string());
        }
        Ok(())
    }
}

/// The front/rear axle normal loads (in newtons) under a quasi-static
/// longitudinal load transfer model: each axle's static share of the
/// vehicle's weight, shifted by `acceleration_mps2 * cg_height_m` acting
/// through the CG height - accelerating shifts load onto the rear axle,
/// braking (or reversing acceleration) shifts it onto the front. Clamped to
/// never go negative (an axle can lift off, in which case it carries no
/// load and therefore no tire force at all - see [`derivative`]), but there
/// is no upper bound: this quasi-static model has no suspension travel to
/// limit it.
fn normal_loads(params: NonlinearTireParams, acceleration_mps2: f64) -> (f64, f64) {
    let wheelbase_m = params.lf_m + params.lr_m;
    let static_fz_f = params.mass_kg * GRAVITY_MPS2 * params.lr_m / wheelbase_m;
    let static_fz_r = params.mass_kg * GRAVITY_MPS2 * params.lf_m / wheelbase_m;
    let transfer_n = params.mass_kg * acceleration_mps2 * params.cg_height_m / wheelbase_m;
    (
        (static_fz_f - transfer_n).max(0.0),
        (static_fz_r + transfer_n).max(0.0),
    )
}

/// The commanded longitudinal force actually deliverable through one axle,
/// and how much of that axle's tire grip remains for cornering afterward
/// (the friction-ellipse combined-slip limit): an axle with zero normal
/// load (`fz_n <= 0.0`) can deliver no force in any direction, so both
/// results are `0.0`.
fn axle_combined_slip(tire_mu: f64, fz_n: f64, demanded_fx_n: f64) -> (f64, f64) {
    let fx_max_n = tire_mu * fz_n;
    if fx_max_n <= 0.0 {
        return (0.0, 0.0);
    }
    let fx_n = demanded_fx_n.clamp(-fx_max_n, fx_max_n);
    let remaining_fraction = (1.0 - (fx_n / fx_max_n).powi(2)).max(0.0).sqrt();
    (fx_n, remaining_fraction)
}

/// Minimum longitudinal speed (in either direction), in meters/second, used
/// in place of the true `vx_mps` when computing tire slip angles - see
/// `crate::environment::simulator::vehicle::dynamic_bicycle::LOW_SPEED_FLOOR_MPS`,
/// which this mitigates the same singularity for. This model's saturating
/// tire curve bounds the *force* even at a raw slip angle near +-90 degrees,
/// but the slip angle itself is still numerically unstable near `vx_mps =
/// 0.0` (tiny changes in `vy_mps` swing it wildly), so the floor is kept
/// here too rather than relying on saturation alone.
const LOW_SPEED_FLOOR_MPS: f64 = 1.0;

/// `vx_mps`'s magnitude, floored to [`LOW_SPEED_FLOOR_MPS`] - always
/// positive, deliberately *not* sign-preserving - see
/// `crate::environment::simulator::vehicle::dynamic_bicycle::regularized_vx`,
/// which this needs it for the same reason: with a *signed* denominator, the
/// yaw-rate feedback term's `1/vx_reg` coefficient flips from damping to
/// amplifying whenever the vehicle reverses - a genuine linear instability
/// of the reversed model, not a numerical artifact - so the denominator's
/// sign is kept fixed instead. The true signed `vx_mps` still carries the
/// forward/reverse distinction through the rest of `derivative`.
fn regularized_vx(vx_mps: f64) -> f64 {
    vx_mps.abs().max(LOW_SPEED_FLOOR_MPS)
}

/// Scales an axle's tire force to `0.0` as its true (unfloored) relative
/// speed - the vector `(vx_mps, lateral_mps)` its contact patch actually
/// sees - approaches zero, saturating to `1.0` once that speed reaches
/// [`LOW_SPEED_FLOOR_MPS`] - see
/// `crate::environment::simulator::vehicle::dynamic_bicycle::low_speed_force_scale`,
/// which this is needed for the same reason as: flooring the `atan2`
/// denominator keeps the *slip angle* bounded, but by itself still lets a
/// stationary vehicle produce full lateral tire force from steering angle
/// alone, which is physically wrong since a tire with zero relative velocity
/// isn't sliding and so can't be generating any force.
fn low_speed_force_scale(vx_mps: f64, lateral_mps: f64) -> f64 {
    (vx_mps.hypot(lateral_mps) / LOW_SPEED_FLOOR_MPS).min(1.0)
}

/// Instantaneous rate of change of a [`NonlinearBicycleState`], as returned
/// by [`derivative`] and consumed by [`step`]'s RK4 integration.
#[derive(Debug, Clone, Copy)]
struct NonlinearDerivative {
    dx_dt: f64,
    dy_dt: f64,
    dheading_dt: f64,
    dvx_dt: f64,
    dvy_dt: f64,
    dyaw_rate_dt: f64,
}

/// The instantaneous derivative of `state` under a constant control input.
/// Builds on
/// [`crate::environment::simulator::vehicle::dynamic_bicycle`]'s slip-angle
/// equations, replacing its linear tire force with a saturating,
/// load-dependent one:
///
/// ```text
/// Fz_f, Fz_r = normal_loads(params, acceleration_mps2)   // see normal_loads
///
/// alpha_f = atan((vy_mps + lf_m*yaw_rate_rad_s) / vx_mps) - steering_angle_rad
/// alpha_r = atan((vy_mps - lr_m*yaw_rate_rad_s) / vx_mps)
///
/// Fyf_raw = -tire_mu*Fz_f * sin(pacejka_c * atan(pacejka_b * alpha_f))
/// Fyr_raw = -tire_mu*Fz_r * sin(pacejka_c * atan(pacejka_b * alpha_r))
/// ```
///
/// `alpha_f`/`alpha_r` are computed against `vx_mps`'s always-positive
/// [`regularized_vx`], using plain `atan` rather than `atan2` - see that
/// function's doc comment, and
/// `crate::environment::simulator::vehicle::dynamic_bicycle`'s `derivative`
/// doc comment, for why.
///
/// The commanded longitudinal force (`mass_kg * acceleration_mps2`) is split
/// between the axles by `front_drive_fraction`, and each axle's lateral
/// force is derated by how much of that axle's tire grip its longitudinal
/// force is using (see [`axle_combined_slip`]) - a front or rear tire
/// working hard to accelerate or brake has correspondingly less grip left
/// over for cornering.
fn derivative(
    state: NonlinearBicycleState,
    params: NonlinearTireParams,
    steering_angle_rad: f64,
    acceleration_mps2: f64,
) -> NonlinearDerivative {
    let (fz_f, fz_r) = normal_loads(params, acceleration_mps2);

    let vx_reg = regularized_vx(state.vx_mps);
    let front_lateral_mps = state.vy_mps + params.lf_m * state.yaw_rate_rad_s;
    let rear_lateral_mps = state.vy_mps - params.lr_m * state.yaw_rate_rad_s;
    let alpha_f = (front_lateral_mps / vx_reg).atan() - steering_angle_rad;
    let alpha_r = (rear_lateral_mps / vx_reg).atan();
    let fyf_raw = -params.tire_mu
        * fz_f
        * (params.pacejka_c * (params.pacejka_b * alpha_f).atan()).sin()
        * low_speed_force_scale(state.vx_mps, front_lateral_mps);
    let fyr_raw = -params.tire_mu
        * fz_r
        * (params.pacejka_c * (params.pacejka_b * alpha_r).atan()).sin()
        * low_speed_force_scale(state.vx_mps, rear_lateral_mps);

    let fx_total_n = params.mass_kg * acceleration_mps2;
    let (fx_f, remaining_f) = axle_combined_slip(
        params.tire_mu,
        fz_f,
        params.front_drive_fraction * fx_total_n,
    );
    let (fx_r, remaining_r) = axle_combined_slip(
        params.tire_mu,
        fz_r,
        (1.0 - params.front_drive_fraction) * fx_total_n,
    );

    let fyf = fyf_raw * remaining_f;
    let fyr = fyr_raw * remaining_r;
    let ax_achieved = (fx_f + fx_r) / params.mass_kg;
    let cos_delta = steering_angle_rad.cos();

    let (sin_h, cos_h) = state.heading_rad.sin_cos();
    NonlinearDerivative {
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
fn advance_state(
    state: NonlinearBicycleState,
    deriv: NonlinearDerivative,
    dt_s: f64,
) -> NonlinearBicycleState {
    NonlinearBicycleState {
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
/// Runge-Kutta (RK4) integration of the nonlinear bicycle equations (see
/// [`derivative`]). The control input is held constant ("frozen") across the
/// whole `dt_s` interval, same as the other two vehicle models.
///
/// `heading_rad` in the returned state is wrapped to `(-pi, pi]`, for the
/// same reason as the other two models.
///
/// `dt_s` of `0.0` returns `state` unchanged (up to the heading wrap, which
/// is a no-op for an already-wrapped angle to within float precision).
pub fn step(
    state: NonlinearBicycleState,
    params: NonlinearTireParams,
    steering_angle_rad: f64,
    acceleration_mps2: f64,
    dt_s: f64,
) -> NonlinearBicycleState {
    let k1 = derivative(state, params, steering_angle_rad, acceleration_mps2);
    let s2 = advance_state(state, k1, dt_s / 2.0);
    let k2 = derivative(s2, params, steering_angle_rad, acceleration_mps2);
    let s3 = advance_state(state, k2, dt_s / 2.0);
    let k3 = derivative(s3, params, steering_angle_rad, acceleration_mps2);
    let s4 = advance_state(state, k3, dt_s);
    let k4 = derivative(s4, params, steering_angle_rad, acceleration_mps2);

    let mut next = NonlinearBicycleState {
        x_m: state.x_m + (dt_s / 6.0) * (k1.dx_dt + 2.0 * k2.dx_dt + 2.0 * k3.dx_dt + k4.dx_dt),
        y_m: state.y_m + (dt_s / 6.0) * (k1.dy_dt + 2.0 * k2.dy_dt + 2.0 * k3.dy_dt + k4.dy_dt),
        heading_rad: state.heading_rad
            + (dt_s / 6.0)
                * (k1.dheading_dt + 2.0 * k2.dheading_dt + 2.0 * k3.dheading_dt + k4.dheading_dt),
        vx_mps: state.vx_mps
            + (dt_s / 6.0) * (k1.dvx_dt + 2.0 * k2.dvx_dt + 2.0 * k3.dvx_dt + k4.dvx_dt),
        vy_mps: state.vy_mps
            + (dt_s / 6.0) * (k1.dvy_dt + 2.0 * k2.dvy_dt + 2.0 * k3.dvy_dt + k4.dvy_dt),
        yaw_rate_rad_s: state.yaw_rate_rad_s
            + (dt_s / 6.0)
                * (k1.dyaw_rate_dt
                    + 2.0 * k2.dyaw_rate_dt
                    + 2.0 * k3.dyaw_rate_dt
                    + k4.dyaw_rate_dt),
    };
    next.heading_rad = wrap_to_pi(next.heading_rad);
    next
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_params() -> NonlinearTireParams {
        NonlinearTireParams {
            mass_kg: 3.5,
            yaw_inertia_kgm2: 0.06,
            lf_m: 0.16,
            lr_m: 0.16,
            cg_height_m: 0.05,
            tire_mu: 1.1,
            pacejka_b: 2.5,
            pacejka_c: 1.3,
            front_drive_fraction: 0.5,
        }
    }

    #[test]
    fn default_geometry_validates() {
        assert!(test_params().validate().is_ok());
    }

    #[test]
    fn non_positive_cg_height_is_rejected() {
        let params = NonlinearTireParams {
            cg_height_m: 0.0,
            ..test_params()
        };
        assert!(params.validate().is_err());
    }

    #[test]
    fn front_drive_fraction_out_of_range_is_rejected() {
        let params = NonlinearTireParams {
            front_drive_fraction: 1.5,
            ..test_params()
        };
        assert!(params.validate().is_err());
        let params = NonlinearTireParams {
            front_drive_fraction: -0.1,
            ..test_params()
        };
        assert!(params.validate().is_err());
    }

    #[test]
    fn normal_loads_matches_static_weight_split_at_zero_acceleration() {
        let params = test_params();
        let (fz_f, fz_r) = normal_loads(params, 0.0);
        // Symmetric geometry (lf_m == lr_m) means an even static split.
        assert!((fz_f - fz_r).abs() < 1e-9);
        assert!((fz_f + fz_r - params.mass_kg * GRAVITY_MPS2).abs() < 1e-9);
    }

    #[test]
    fn normal_loads_shifts_toward_the_rear_under_forward_acceleration() {
        let params = test_params();
        let (fz_f, fz_r) = normal_loads(params, 3.0);
        assert!(fz_r > fz_f);
    }

    #[test]
    fn normal_loads_never_goes_negative_under_extreme_acceleration() {
        let params = test_params();
        let (fz_f, fz_r) = normal_loads(params, 1000.0);
        assert_eq!(fz_f, 0.0);
        assert!(fz_r > 0.0);
    }

    #[test]
    fn zero_dt_returns_state_unchanged() {
        let state = NonlinearBicycleState {
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
        let state = NonlinearBicycleState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            vx_mps: 5.0,
            vy_mps: 0.0,
            yaw_rate_rad_s: 0.0,
        };
        let next = step(state, test_params(), 0.0, 0.0, 2.0);
        assert!((next.x_m - 10.0).abs() < 1e-9);
        assert!(next.y_m.abs() < 1e-9);
        assert!(next.heading_rad.abs() < 1e-9);
        assert!((next.vx_mps - 5.0).abs() < 1e-9);
        assert!(next.vy_mps.abs() < 1e-9);
        assert!(next.yaw_rate_rad_s.abs() < 1e-9);
    }

    #[test]
    fn steering_alone_from_a_standstill_does_not_move_the_vehicle() {
        // See dynamic_bicycle's test of the same name - a stationary tire
        // can't be generating any lateral force, so steering with zero
        // throttle should leave a standing vehicle exactly where it is.
        let params = test_params();
        let mut state = NonlinearBicycleState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            vx_mps: 0.0,
            vy_mps: 0.0,
            yaw_rate_rad_s: 0.0,
        };
        let dt_s = 0.01;
        for _ in 0..100 {
            state = step(state, params, 0.4, 0.0, dt_s);
        }
        assert_eq!(
            state,
            NonlinearBicycleState {
                x_m: 0.0,
                y_m: 0.0,
                heading_rad: 0.0,
                vx_mps: 0.0,
                vy_mps: 0.0,
                yaw_rate_rad_s: 0.0
            }
        );
    }

    #[test]
    fn straight_line_reverse_and_no_steering_produces_no_lateral_force() {
        // See dynamic_bicycle's test of the same name - a wheel rolling
        // straight backward with no steering has zero actual slip, so it
        // should produce zero lateral force just like straight forward.
        let state = NonlinearBicycleState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            vx_mps: -5.0,
            vy_mps: 0.0,
            yaw_rate_rad_s: 0.0,
        };
        let next = step(state, test_params(), 0.0, 0.0, 2.0);
        assert!((next.x_m - (-10.0)).abs() < 1e-9);
        assert!(next.y_m.abs() < 1e-9);
        assert!(next.heading_rad.abs() < 1e-9);
        assert!((next.vx_mps - (-5.0)).abs() < 1e-9);
        assert!(next.vy_mps.abs() < 1e-9);
        assert!(next.yaw_rate_rad_s.abs() < 1e-9);
    }

    #[test]
    fn steering_still_turns_the_vehicle_while_reversing() {
        // See dynamic_bicycle's test of the same name.
        let params = test_params();
        let mut state = NonlinearBicycleState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            vx_mps: -3.0,
            vy_mps: 0.0,
            yaw_rate_rad_s: 0.0,
        };
        let dt_s = 0.01;
        for _ in 0..50 {
            state = step(state, params, 0.35, 0.0, dt_s);
        }
        assert!(
            state.yaw_rate_rad_s.abs() > 0.5,
            "expected steering to meaningfully turn the vehicle in reverse: {state:?}"
        );
    }

    #[test]
    fn steering_while_accelerating_into_reverse_does_not_diverge() {
        // See dynamic_bicycle's test of the same name.
        let params = test_params();
        let mut state = NonlinearBicycleState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            vx_mps: 0.0,
            vy_mps: 0.0,
            yaw_rate_rad_s: 0.0,
        };
        let dt_s = 0.01;
        let (target_speed, max_accel, max_decel) = (-3.0_f64, 4.0_f64, 8.0_f64);
        for _ in 0..300 {
            let err = target_speed - state.vx_mps;
            let accel = if err >= 0.0 {
                (err / dt_s).min(max_accel)
            } else {
                (err / dt_s).max(-max_decel)
            };
            state = step(state, params, 0.2, accel, dt_s);
            assert!(state.vy_mps.is_finite() && state.yaw_rate_rad_s.is_finite());
            assert!(
                state.yaw_rate_rad_s.abs() < 10.0,
                "yaw rate diverged while reversing under steering: {state:?}"
            );
        }
    }

    #[test]
    fn full_steering_and_throttle_from_a_standstill_does_not_diverge() {
        // Same regression as
        // crate::environment::simulator::vehicle::dynamic_bicycle's test of
        // the same name - see LOW_SPEED_FLOOR_MPS.
        let params = test_params();
        let mut state = NonlinearBicycleState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            vx_mps: 0.0,
            vy_mps: 0.0,
            yaw_rate_rad_s: 0.0,
        };
        let dt_s = 0.01;
        for _ in 0..300 {
            state = step(state, params, 0.4, 4.0, dt_s);
            assert!(state.vx_mps.is_finite());
            assert!(state.vy_mps.is_finite());
            assert!(
                state.vx_mps.hypot(state.vy_mps) < 15.0,
                "speed diverged: {}",
                state.vx_mps.hypot(state.vy_mps)
            );
        }
    }

    #[test]
    fn constant_steering_settles_into_a_bounded_steady_turn() {
        let params = test_params();
        let mut state = NonlinearBicycleState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            vx_mps: 5.0,
            vy_mps: 0.0,
            yaw_rate_rad_s: 0.0,
        };
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
        let initial = NonlinearBicycleState {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            vx_mps: 5.0,
            vy_mps: 0.0,
            yaw_rate_rad_s: 0.0,
        };
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
