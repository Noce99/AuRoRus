//! Two-track (four-wheel) model: unlike every other model in this module,
//! which collapses each axle into a single force acting on the vehicle's
//! centerline, this one tracks all four wheels individually - their own
//! slip angle, their own (now laterally as well as longitudinally
//! transferred) normal load, and their own steer angle via Ackermann
//! geometry - and sums their forces and moments as a rigid body. It reuses
//! the full Pacejka Magic Formula from
//! [`crate::environment::simulator::vehicle::pacejka_bicycle`] per wheel
//! rather than introducing a new tire curve; the change here is purely
//! geometric (wheel positions and force summation), which is what actually
//! enables lateral (left-right) load transfer and per-wheel asymmetry. State
//! is still just the vehicle's center of gravity (CG) position, heading, and
//! body-frame velocity - the same shape as
//! [`crate::environment::simulator::vehicle::PacejkaBicycleState`] - since
//! each wheel's quantities are derived from that plus geometry every tick,
//! not tracked as separate state. See
//! `src/environment/simulator/vehicle/README.md` for the equations and
//! their derivation/rationale; this file is the implementation. [`step`] is
//! the entry point a simulation environment's tick loop is expected to call
//! once per tick.

/// Standard gravity, in meters/second^2 - used to compute each axle's static
/// share of the vehicle's weight before any load transfer is applied.
const GRAVITY_MPS2: f64 = 9.81;

/// Dynamic state of the two-track model at one instant - identical fields to
/// [`crate::environment::simulator::vehicle::PacejkaBicycleState`]; see
/// there for field meanings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TwoTrackState {
    pub x_m: f64,
    pub y_m: f64,
    pub heading_rad: f64,
    pub vx_mps: f64,
    pub vy_mps: f64,
    pub yaw_rate_rad_s: f64,
}

/// Mass, yaw inertia, wheelbase/track geometry, front/rear Magic Formula
/// tire curves, combined-slip weighting shape, and drive-force split for
/// the two-track model.
///
/// No [`Default`] is provided on purpose, for the same reason as
/// [`crate::environment::simulator::vehicle::PacejkaTireParams`]: these
/// describe a specific vehicle's real physical properties, and silently
/// defaulting them would silently produce a physically wrong trajectory
/// with no signal that anything is off.
#[derive(Debug, Clone, Copy, PartialEq, serde::Deserialize)]
pub struct TwoTrackParams {
    /// Vehicle mass, in kilograms.
    pub mass_kg: f64,
    /// Yaw moment of inertia about the vertical axis through the CG, in
    /// kilogram-meters^2.
    pub yaw_inertia_kgm2: f64,
    /// Distance from the CG to the front axle, in meters.
    pub lf_m: f64,
    /// Distance from the CG to the rear axle, in meters.
    pub lr_m: f64,
    /// Height of the CG above the ground, in meters - drives both
    /// longitudinal and lateral load transfer (see [`wheel_normal_loads`]).
    pub cg_height_m: f64,
    /// Distance between the left and right wheels, assumed the same front
    /// and rear - drives per-wheel slip angles, Ackermann steering, and
    /// lateral load transfer.
    pub track_width_m: f64,
    /// Front tire Magic Formula stiffness factor (`B`), shared by both
    /// front wheels.
    pub front_b: f64,
    /// Front tire Magic Formula shape factor (`C`).
    pub front_c: f64,
    /// Front tire peak friction coefficient - a wheel's Magic Formula `D`
    /// is `front_d_mu * <that wheel's normal load>`.
    pub front_d_mu: f64,
    /// Front tire Magic Formula curvature factor (`E`).
    pub front_e: f64,
    /// Rear tire Magic Formula stiffness factor (`B`), shared by both rear
    /// wheels.
    pub rear_b: f64,
    /// Rear tire Magic Formula shape factor (`C`).
    pub rear_c: f64,
    /// Rear tire peak friction coefficient.
    pub rear_d_mu: f64,
    /// Rear tire Magic Formula curvature factor (`E`).
    pub rear_e: f64,
    /// Stiffness factor of the combined-slip weighting-function curve,
    /// shared by all four wheels.
    pub combined_slip_b: f64,
    /// Shape factor of the combined-slip weighting-function curve, shared
    /// by all four wheels.
    pub combined_slip_c: f64,
    /// Fraction (`0.0..=1.0`) of the commanded longitudinal force delivered
    /// through the front axle; the rest goes to the rear. Within whichever
    /// axle(s) are driven, the split is 50/50 left-right (an
    /// open-differential assumption - see the module README).
    pub front_drive_fraction: f64,
}

impl TwoTrackParams {
    /// Basic sanity checks on the parameter values. `front_e`/`rear_e` are
    /// curvature factors meaningful at any real value, so they're left
    /// unconstrained.
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
        if self.track_width_m <= 0.0 {
            return Err("track_width_m must be positive".to_string());
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

/// The front/rear axle normal load *totals* (in newtons) under the same
/// quasi-static longitudinal load transfer model as
/// [`crate::environment::simulator::vehicle::pacejka_bicycle`]: each axle's
/// static share of the vehicle's weight, shifted by
/// `acceleration_mps2 * cg_height_m` acting through the CG height, clamped
/// to never go negative. [`wheel_normal_loads`] splits each of these
/// further into left/right.
fn axle_loads(params: TwoTrackParams, acceleration_mps2: f64) -> (f64, f64) {
    let wheelbase_m = params.lf_m + params.lr_m;
    let static_fz_f = params.mass_kg * GRAVITY_MPS2 * params.lr_m / wheelbase_m;
    let static_fz_r = params.mass_kg * GRAVITY_MPS2 * params.lf_m / wheelbase_m;
    let transfer_n = params.mass_kg * acceleration_mps2 * params.cg_height_m / wheelbase_m;
    ((static_fz_f - transfer_n).max(0.0), (static_fz_r + transfer_n).max(0.0))
}

/// The four wheels' individual normal loads (`fl`, `fr`, `rl`, `rr` - in
/// newtons), from [`axle_loads`]'s front/rear totals plus a quasi-static
/// *lateral* load transfer using the centripetal-acceleration proxy
/// `ay_mps2` (see the module README for why a proxy rather than the true
/// lateral acceleration is used). The total lateral transfer is split
/// between the front and rear axles in proportion to their (longitudinal-
/// transferred) load share - there's no separate front/rear roll-stiffness-
/// distribution parameter (see Limitations in the README). Positive
/// `ay_mps2` shifts load from the `+track_width_m/2` ("left") wheels to the
/// `-track_width_m/2` ("right") wheels; results are clamped to never go
/// negative.
fn wheel_normal_loads(params: TwoTrackParams, acceleration_mps2: f64, ay_mps2: f64) -> (f64, f64, f64, f64) {
    let (fz_f_total, fz_r_total) = axle_loads(params, acceleration_mps2);
    let front_share = fz_f_total / (fz_f_total + fz_r_total);
    let total_lateral_transfer = params.mass_kg * ay_mps2 * params.cg_height_m / params.track_width_m;

    let front_lateral = total_lateral_transfer * front_share;
    let rear_lateral = total_lateral_transfer * (1.0 - front_share);
    (
        (fz_f_total / 2.0 - front_lateral / 2.0).max(0.0),
        (fz_f_total / 2.0 + front_lateral / 2.0).max(0.0),
        (fz_r_total / 2.0 - rear_lateral / 2.0).max(0.0),
        (fz_r_total / 2.0 + rear_lateral / 2.0).max(0.0),
    )
}

/// The individual front-left/front-right steer angles Ackermann geometry
/// produces from one nominal `steering_angle_rad` (the same "virtual"
/// single-track wheel angle every other model in this module takes as its
/// control input): the two front wheels trace circles of different radii
/// around the same turn center, so they need slightly different angles.
/// `0.0` steering input always produces `(0.0, 0.0)`.
fn ackermann_wheel_angles(steering_angle_rad: f64, wheelbase_m: f64, track_width_m: f64) -> (f64, f64) {
    let kappa = steering_angle_rad.tan() / wheelbase_m;
    let half_track_m = track_width_m / 2.0;
    let delta_left = (wheelbase_m * kappa / (1.0 - half_track_m * kappa)).atan();
    let delta_right = (wheelbase_m * kappa / (1.0 + half_track_m * kappa)).atan();
    (delta_left, delta_right)
}

/// The full ("similarity") Pacejka Magic Formula for one tire's lateral
/// force - identical to
/// [`crate::environment::simulator::vehicle::pacejka_bicycle`]'s helper of
/// the same shape, duplicated here per this module's convention that every
/// model owns its full implementation independently.
fn pacejka_lateral_force(b: f64, c: f64, d: f64, e: f64, alpha: f64) -> f64 {
    let b_alpha = b * alpha;
    let inner = b_alpha - e * (b_alpha - b_alpha.atan());
    -d * (c * inner.atan()).sin()
}

/// The commanded longitudinal force actually deliverable through one wheel,
/// and the combined-slip weighting fraction its lateral force should be
/// scaled by afterward - identical shape to
/// [`crate::environment::simulator::vehicle::pacejka_bicycle`]'s per-axle
/// version, applied per wheel here instead. A wheel with zero normal load
/// (`d_n <= 0.0`) can deliver no force in any direction, so both results
/// are `0.0`.
fn wheel_combined_slip(combined_slip_b: f64, combined_slip_c: f64, d_n: f64, demanded_fx_n: f64) -> (f64, f64) {
    if d_n <= 0.0 {
        return (0.0, 0.0);
    }
    let fx_n = demanded_fx_n.clamp(-d_n, d_n);
    let weighting = (combined_slip_c * (combined_slip_b * (fx_n / d_n)).atan()).cos().max(0.0);
    (fx_n, weighting)
}

/// Minimum longitudinal speed (in either direction), in meters/second, used
/// in place of a wheel's true longitudinal velocity when computing its slip
/// angle - see
/// `crate::environment::simulator::vehicle::dynamic_bicycle::LOW_SPEED_FLOOR_MPS`,
/// which this mitigates the same singularity for, applied per wheel here.
const LOW_SPEED_FLOOR_MPS: f64 = 1.0;

/// `vx_mps`, floored in magnitude to [`LOW_SPEED_FLOOR_MPS`] - sign-preserving,
/// defaulting to the forward direction at exactly zero, since that's the
/// common case (starting from a stop).
fn regularized_vx(vx_mps: f64) -> f64 {
    if vx_mps.abs() >= LOW_SPEED_FLOOR_MPS {
        vx_mps
    } else if vx_mps < 0.0 {
        -LOW_SPEED_FLOOR_MPS
    } else {
        LOW_SPEED_FLOOR_MPS
    }
}

/// One wheel's fixed geometry and this-tick inputs, gathered so
/// [`wheel_body_forces`] can be called once per wheel from a loop instead of
/// once per wheel inline.
struct WheelInput {
    /// Longitudinal offset from the CG, in meters (positive = front).
    a_m: f64,
    /// Lateral offset from the CG, in meters (positive = left).
    b_m: f64,
    /// This wheel's steer angle, in radians (`0.0` for an unsteered wheel).
    delta_rad: f64,
    b: f64,
    c: f64,
    d_mu: f64,
    e: f64,
    fz_n: f64,
    fx_demand_n: f64,
}

/// One wheel's contribution to the vehicle's total body-frame force and yaw
/// moment: its slip angle and resulting tire forces (in the wheel's own
/// steered frame), rotated into the body frame by `delta_rad`, and the yaw
/// moment that force produces about the CG via its `(a_m, b_m)` lever arm.
/// Returns `(fx_body_n, fy_body_n, yaw_moment_nm)`.
fn wheel_body_forces(
    wheel: &WheelInput,
    vx_mps: f64,
    vy_mps: f64,
    yaw_rate_rad_s: f64,
    combined_slip_b: f64,
    combined_slip_c: f64,
) -> (f64, f64, f64) {
    let vx_wheel = vx_mps - yaw_rate_rad_s * wheel.b_m;
    let vy_wheel = vy_mps + yaw_rate_rad_s * wheel.a_m;
    let alpha = vy_wheel.atan2(regularized_vx(vx_wheel)) - wheel.delta_rad;

    let d_n = wheel.d_mu * wheel.fz_n;
    let fy_raw = pacejka_lateral_force(wheel.b, wheel.c, d_n, wheel.e, alpha);
    let (fx_wheel, weighting) = wheel_combined_slip(combined_slip_b, combined_slip_c, d_n, wheel.fx_demand_n);
    let fy_wheel = fy_raw * weighting;

    let (sin_d, cos_d) = wheel.delta_rad.sin_cos();
    let fx_body = fx_wheel * cos_d - fy_wheel * sin_d;
    let fy_body = fx_wheel * sin_d + fy_wheel * cos_d;
    let yaw_moment = wheel.a_m * fy_body - wheel.b_m * fx_body;
    (fx_body, fy_body, yaw_moment)
}

/// Instantaneous rate of change of a [`TwoTrackState`], as returned by
/// [`derivative`] and consumed by [`step`]'s RK4 integration.
#[derive(Debug, Clone, Copy)]
struct TwoTrackDerivative {
    dx_dt: f64,
    dy_dt: f64,
    dheading_dt: f64,
    dvx_dt: f64,
    dvy_dt: f64,
    dyaw_rate_dt: f64,
}

/// The instantaneous derivative of `state` under a constant control input.
/// Computes each of the four wheels' normal load, steer angle, slip angle,
/// and resulting body-frame force (see [`wheel_normal_loads`],
/// [`ackermann_wheel_angles`], [`wheel_body_forces`]), then sums them as a
/// rigid body - the genuinely new part relative to every other model in
/// this module, none of which has a left-right force asymmetry to sum.
fn derivative(state: TwoTrackState, params: TwoTrackParams, steering_angle_rad: f64, acceleration_mps2: f64) -> TwoTrackDerivative {
    let ay_mps2 = state.vx_mps * state.yaw_rate_rad_s;
    let (fz_fl, fz_fr, fz_rl, fz_rr) = wheel_normal_loads(params, acceleration_mps2, ay_mps2);
    let (delta_fl, delta_fr) = ackermann_wheel_angles(steering_angle_rad, params.lf_m + params.lr_m, params.track_width_m);

    let fx_total_n = params.mass_kg * acceleration_mps2;
    let fx_front_each = params.front_drive_fraction * fx_total_n / 2.0;
    let fx_rear_each = (1.0 - params.front_drive_fraction) * fx_total_n / 2.0;
    let half_track_m = params.track_width_m / 2.0;

    let wheels = [
        WheelInput {
            a_m: params.lf_m,
            b_m: half_track_m,
            delta_rad: delta_fl,
            b: params.front_b,
            c: params.front_c,
            d_mu: params.front_d_mu,
            e: params.front_e,
            fz_n: fz_fl,
            fx_demand_n: fx_front_each,
        },
        WheelInput {
            a_m: params.lf_m,
            b_m: -half_track_m,
            delta_rad: delta_fr,
            b: params.front_b,
            c: params.front_c,
            d_mu: params.front_d_mu,
            e: params.front_e,
            fz_n: fz_fr,
            fx_demand_n: fx_front_each,
        },
        WheelInput {
            a_m: -params.lr_m,
            b_m: half_track_m,
            delta_rad: 0.0,
            b: params.rear_b,
            c: params.rear_c,
            d_mu: params.rear_d_mu,
            e: params.rear_e,
            fz_n: fz_rl,
            fx_demand_n: fx_rear_each,
        },
        WheelInput {
            a_m: -params.lr_m,
            b_m: -half_track_m,
            delta_rad: 0.0,
            b: params.rear_b,
            c: params.rear_c,
            d_mu: params.rear_d_mu,
            e: params.rear_e,
            fz_n: fz_rr,
            fx_demand_n: fx_rear_each,
        },
    ];

    let mut fx_total = 0.0;
    let mut fy_total = 0.0;
    let mut mz_total = 0.0;
    for wheel in &wheels {
        let (fx_body, fy_body, yaw_moment) =
            wheel_body_forces(wheel, state.vx_mps, state.vy_mps, state.yaw_rate_rad_s, params.combined_slip_b, params.combined_slip_c);
        fx_total += fx_body;
        fy_total += fy_body;
        mz_total += yaw_moment;
    }

    let (sin_h, cos_h) = state.heading_rad.sin_cos();
    TwoTrackDerivative {
        dx_dt: state.vx_mps * cos_h - state.vy_mps * sin_h,
        dy_dt: state.vx_mps * sin_h + state.vy_mps * cos_h,
        dheading_dt: state.yaw_rate_rad_s,
        dvx_dt: fx_total / params.mass_kg + state.vy_mps * state.yaw_rate_rad_s,
        dvy_dt: fy_total / params.mass_kg - state.vx_mps * state.yaw_rate_rad_s,
        dyaw_rate_dt: mz_total / params.yaw_inertia_kgm2,
    }
}

/// `state` advanced linearly by `deriv` scaled by `dt_s` - the building
/// block for combining RK4 stages. Heading is left unwrapped here; [`step`]
/// wraps the final result.
fn advance_state(state: TwoTrackState, deriv: TwoTrackDerivative, dt_s: f64) -> TwoTrackState {
    TwoTrackState {
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
/// Runge-Kutta (RK4) integration of the two-track equations (see
/// [`derivative`]). The control input is held constant ("frozen") across the
/// whole `dt_s` interval, same as the other four vehicle models.
///
/// `heading_rad` in the returned state is wrapped to `(-pi, pi]`, for the
/// same reason as the other four models.
///
/// `dt_s` of `0.0` returns `state` unchanged (up to the heading wrap, which
/// is a no-op for an already-wrapped angle to within float precision).
pub fn step(
    state: TwoTrackState,
    params: TwoTrackParams,
    steering_angle_rad: f64,
    acceleration_mps2: f64,
    dt_s: f64,
) -> TwoTrackState {
    let k1 = derivative(state, params, steering_angle_rad, acceleration_mps2);
    let s2 = advance_state(state, k1, dt_s / 2.0);
    let k2 = derivative(s2, params, steering_angle_rad, acceleration_mps2);
    let s3 = advance_state(state, k2, dt_s / 2.0);
    let k3 = derivative(s3, params, steering_angle_rad, acceleration_mps2);
    let s4 = advance_state(state, k3, dt_s);
    let k4 = derivative(s4, params, steering_angle_rad, acceleration_mps2);

    let mut next = TwoTrackState {
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

    fn test_params() -> TwoTrackParams {
        TwoTrackParams {
            mass_kg: 3.5,
            yaw_inertia_kgm2: 0.06,
            lf_m: 0.16,
            lr_m: 0.16,
            cg_height_m: 0.05,
            track_width_m: 0.2,
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
    fn non_positive_track_width_is_rejected() {
        let params = TwoTrackParams { track_width_m: 0.0, ..test_params() };
        assert!(params.validate().is_err());
    }

    #[test]
    fn front_drive_fraction_out_of_range_is_rejected() {
        let params = TwoTrackParams { front_drive_fraction: 1.5, ..test_params() };
        assert!(params.validate().is_err());
    }

    #[test]
    fn ackermann_is_neutral_at_zero_steering() {
        let (left, right) = ackermann_wheel_angles(0.0, 0.32, 0.2);
        assert_eq!(left, 0.0);
        assert_eq!(right, 0.0);
    }

    #[test]
    fn ackermann_inner_wheel_gets_the_sharper_angle() {
        let (left, right) = ackermann_wheel_angles(0.2, 0.32, 0.2);
        assert!(left > 0.0 && right > 0.0);
        assert!(left > right, "expected the inner wheel's angle to exceed the outer's: {left} vs {right}");

        let (left, right) = ackermann_wheel_angles(-0.2, 0.32, 0.2);
        assert!(left < 0.0 && right < 0.0);
        assert!(right < left, "expected the inner wheel's angle to exceed the outer's: {right} vs {left}");
    }

    #[test]
    fn wheel_normal_loads_splits_evenly_left_right_at_zero_lateral_acceleration() {
        let params = test_params();
        let (fl, fr, rl, rr) = wheel_normal_loads(params, 0.0, 0.0);
        assert!((fl - fr).abs() < 1e-9);
        assert!((rl - rr).abs() < 1e-9);
    }

    #[test]
    fn wheel_normal_loads_shifts_to_the_outside_wheels_under_lateral_acceleration() {
        let params = test_params();
        let (fl, fr, rl, rr) = wheel_normal_loads(params, 0.0, 3.0);
        assert!(fr > fl, "expected the right (outside) front wheel to gain load: {fr} vs {fl}");
        assert!(rr > rl, "expected the right (outside) rear wheel to gain load: {rr} vs {rl}");
        assert!(fl >= 0.0 && fr >= 0.0 && rl >= 0.0 && rr >= 0.0);
    }

    #[test]
    fn zero_dt_returns_state_unchanged() {
        let state = TwoTrackState {
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
            TwoTrackState { x_m: 0.0, y_m: 0.0, heading_rad: 0.0, vx_mps: 5.0, vy_mps: 0.0, yaw_rate_rad_s: 0.0 };
        let next = step(state, test_params(), 0.0, 0.0, 2.0);
        assert!((next.x_m - 10.0).abs() < 1e-9);
        assert!(next.y_m.abs() < 1e-9);
        assert!(next.heading_rad.abs() < 1e-9);
        assert!((next.vx_mps - 5.0).abs() < 1e-9);
        assert!(next.vy_mps.abs() < 1e-9);
        assert!(next.yaw_rate_rad_s.abs() < 1e-9);
    }

    #[test]
    fn full_steering_and_throttle_from_a_standstill_does_not_diverge() {
        // Same regression as
        // crate::environment::simulator::vehicle::dynamic_bicycle's test of
        // the same name - see LOW_SPEED_FLOOR_MPS.
        let params = test_params();
        let mut state =
            TwoTrackState { x_m: 0.0, y_m: 0.0, heading_rad: 0.0, vx_mps: 0.0, vy_mps: 0.0, yaw_rate_rad_s: 0.0 };
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
        let mut state =
            TwoTrackState { x_m: 0.0, y_m: 0.0, heading_rad: 0.0, vx_mps: 5.0, vy_mps: 0.0, yaw_rate_rad_s: 0.0 };
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
            TwoTrackState { x_m: 0.0, y_m: 0.0, heading_rad: 0.0, vx_mps: 5.0, vy_mps: 0.0, yaw_rate_rad_s: 0.0 };
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
