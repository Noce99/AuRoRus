//! Single-track dynamic bicycle model with a linear tire model: state is the
//! vehicle's center of gravity (CG) position, heading, and body-frame
//! velocity (longitudinal, lateral, yaw rate); control inputs are steering
//! angle and longitudinal acceleration, integrated with classical 4th-order
//! Runge-Kutta (RK4). See
//! `src/environment/simulator/vehicle/README.md` for the equations and their
//! derivation/rationale; this file is the implementation. [`step`] is the
//! entry point a simulation environment's tick loop is expected to call once
//! per tick.

use super::tunable;
use crate::topics::AlgorithmParameter;

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
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DynamicParams {
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
    /// Front tire cornering stiffness (lateral force per radian of slip
    /// angle), in newtons/radian.
    pub cf_n_per_rad: f64,
    /// Rear tire cornering stiffness (lateral force per radian of slip
    /// angle), in newtons/radian.
    pub cr_n_per_rad: f64,
}

impl DynamicParams {
    /// The live-tunable parameters, one per field - see
    /// [`crate::actuators::SimulatedVehicle`].
    pub fn tunable_parameters() -> Vec<AlgorithmParameter> {
        let cornering_stiffness = |name: &str, axle: &str| {
            AlgorithmParameter::float(name, 5.0, 300.0, 1.0)
                .unit("N/rad")
                .description(format!("{axle} tire cornering stiffness."))
        };
        vec![
            tunable::yaw_inertia_kgm2(),
            cornering_stiffness("cf_n_per_rad", "Front"),
            cornering_stiffness("cr_n_per_rad", "Rear"),
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

/// Minimum longitudinal speed (in either direction), in meters/second, used
/// in place of the true `vx_mps` when computing tire slip angles. Below
/// this, `atan2`'s true geometric angle swings toward +-90 degrees for even
/// tiny lateral velocity - the "not valid near zero forward speed"
/// limitation in the module README - which this model's *unsaturated*
/// linear tire force then turns into an unbounded lateral force. That
/// force feeds back into `vx_mps` via the `vy_mps*yaw_rate_rad_s` coupling
/// term in [`derivative`], which can run away well past what
/// `crate::actuators::simulated_vehicle::ActuatorLimits` should allow -
/// observed in practice as the vehicle suddenly rocketing away (forward or
/// backward) from a standing start under full steering and throttle.
/// Flooring the `atan2` denominator's magnitude (see [`regularized_vx`])
/// keeps slip angles - and so tire forces - bounded during that low-speed
/// transient, without changing anything once the vehicle is actually
/// moving faster than this.
const LOW_SPEED_FLOOR_MPS: f64 = 1.0;

/// `vx_mps`'s magnitude, floored to [`LOW_SPEED_FLOOR_MPS`] - always
/// positive, deliberately *not* sign-preserving. Used as the slip angle's
/// `atan` denominator (see [`derivative`]) rather than the signed `vx_mps`:
/// the model's yaw-rate feedback term has a `1/vx_reg` coefficient whose sign
/// determines whether it damps or amplifies yaw rate, and with a *signed*
/// `vx_reg` that coefficient flips from damping to amplifying whenever the
/// vehicle reverses - a genuine linear instability of the reversed model,
/// not a numerical artifact, that (with steering held) grows the yaw rate
/// exponentially every tick. Keeping the denominator's sign fixed (always
/// positive) keeps that feedback term damping in both directions, matching
/// how the *forward* model is dynamically stable. What direction the vehicle
/// actually turns while reversing still comes out sign-correct: `dx/dt`/
/// `dy/dt`/`dvy/dt`/`dyaw_rate/dt` all still multiply by the true signed
/// `vx_mps` elsewhere in [`derivative`], which is what actually carries the
/// forward/reverse distinction through the rest of the model.
fn regularized_vx(vx_mps: f64) -> f64 {
    vx_mps.abs().max(LOW_SPEED_FLOOR_MPS)
}

/// Scales an axle's tire force to `0.0` as its true (unfloored) relative
/// speed - the vector `(vx_mps, lateral_mps)` its contact patch actually
/// sees - approaches zero, saturating to `1.0` once that speed reaches
/// [`LOW_SPEED_FLOOR_MPS`]. Needed on top of [`regularized_vx`]: flooring the
/// `atan2` denominator keeps the *slip angle* bounded, but by itself still
/// lets a stationary vehicle produce full lateral tire force from steering
/// angle alone (`alpha_f = -steering_angle_rad` when `vy_mps` and
/// `yaw_rate_rad_s` are both `0.0`, since `vx_reg` is never `0.0`) - which is
/// physically wrong, since a tire with zero relative velocity isn't sliding
/// and so can't be generating any force. Observed in practice as the vehicle
/// rotating/drifting in place under steering input alone, with no throttle.
fn low_speed_force_scale(vx_mps: f64, lateral_mps: f64) -> f64 {
    (vx_mps.hypot(lateral_mps) / LOW_SPEED_FLOOR_MPS).min(1.0)
}

/// The instantaneous derivative of `state` under a constant control input,
/// from the linear-tire single-track dynamic bicycle equations:
///
/// ```text
/// vx_reg  = regularized_vx(vx_mps)   // see LOW_SPEED_FLOOR_MPS
/// alpha_f = atan((vy_mps + lf_m*yaw_rate_rad_s) / vx_reg) - steering_angle_rad
/// alpha_r = atan((vy_mps - lr_m*yaw_rate_rad_s) / vx_reg)
/// Fyf     = -cf_n_per_rad * alpha_f * low_speed_force_scale(vx_mps, vy_mps + lf_m*yaw_rate_rad_s)
/// Fyr     = -cr_n_per_rad * alpha_r * low_speed_force_scale(vx_mps, vy_mps - lr_m*yaw_rate_rad_s)
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
/// motion the geometry implies. They're computed against `vx_reg` - always
/// positive, see [`regularized_vx`] - rather than the raw (signed) `vx_mps`,
/// using plain `atan` rather than `atan2`: with a positive-only denominator
/// the two are identical (`atan2(y,x) == atan(y/x)` whenever `x > 0`, for
/// any `y`), but plain `atan` is what makes a positive-only `vx_reg`
/// meaningful in the first place - `atan2` would still fold the sign of the
/// *true* `vx_mps` back in via `y`'s sign convention in a way `atan` doesn't
/// need to care about here.
fn derivative(
    state: DynamicState,
    params: DynamicParams,
    steering_angle_rad: f64,
    acceleration_mps2: f64,
) -> DynamicDerivative {
    let vx_reg = regularized_vx(state.vx_mps);
    let front_lateral_mps = state.vy_mps + params.lf_m * state.yaw_rate_rad_s;
    let rear_lateral_mps = state.vy_mps - params.lr_m * state.yaw_rate_rad_s;
    let alpha_f = (front_lateral_mps / vx_reg).atan() - steering_angle_rad;
    let alpha_r = (rear_lateral_mps / vx_reg).atan();
    let fyf =
        -params.cf_n_per_rad * alpha_f * low_speed_force_scale(state.vx_mps, front_lateral_mps);
    let fyr =
        -params.cr_n_per_rad * alpha_r * low_speed_force_scale(state.vx_mps, rear_lateral_mps);
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
        let params = DynamicParams {
            mass_kg: 0.0,
            ..test_params()
        };
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
        let state = DynamicState {
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
        // A stationary tire isn't sliding, so it can't be generating any
        // lateral force - steering with zero throttle should leave a
        // standing vehicle exactly where it is, not spin/drift it in place
        // (see low_speed_force_scale).
        let params = test_params();
        let mut state = DynamicState {
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
            DynamicState {
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
        // A wheel rolling straight backward with no steering has zero actual
        // slip, so it should produce zero lateral force - same as straight
        // forward (straight_line_zero_steering_and_no_slip_matches_simple_kinematics)
        // but with vx_mps negative. Regression for atan2 computing a slip
        // angle near +-pi (not 0) whenever vx_mps < 0, which used to make
        // reversing spuriously drift/rotate even with the wheels straight.
        let state = DynamicState {
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
        // Regression for steering doing almost nothing while reversing: the
        // old atan2-based slip angle saturated near the tire curve's extreme
        // for any nonzero steering once vx_mps went negative, instead of
        // responding proportionally like it does going forward.
        let params = test_params();
        let mut state = DynamicState {
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
        // Regression: with a *signed* regularized_vx, the yaw-rate feedback
        // term's 1/vx_reg coefficient flips from damping to amplifying while
        // reversing at low speed (a genuine linear instability of the
        // reversed model), which under steering grew yaw_rate_rad_s
        // exponentially every tick instead of settling into a bounded turn.
        let params = test_params();
        let mut state = DynamicState {
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
        // Reproduces the scenario LOW_SPEED_FLOOR_MPS exists for: full
        // steering and throttle from a complete stop, where the
        // un-floored atan2(vy_mps, vx_mps) singularity used to let tire
        // forces run away, sending the vehicle's speed far past anything
        // ActuatorLimits should allow before collapsing back down.
        let params = test_params();
        let mut state = DynamicState {
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
        // A known limitation of this model (see the vehicle README) is that
        // it isn't valid near zero forward speed, so this starts at a
        // reasonable cruising speed rather than from a stop. It doesn't
        // assert a closed-form radius (unlike the kinematic model's
        // equivalent test) since the dynamic model's turn radius isn't a
        // simple closed form - only that constant steering input produces a
        // finite, bounded response rather than diverging.
        let params = test_params();
        let mut state = DynamicState {
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
}
