//! Heading-error steering laws - a PD controller and the "P-enhanced"
//! controller - shared by the algorithms that steer toward a lookahead
//! point (`path_follower`, `frenet_overtaking`).

use std::time::Instant;

/// The PD controller's derivative action never exceeds this, in radians.
pub(crate) const MAX_DERIVATIVE_ACTION_RAD: f64 = 0.2;

/// The gains of [`pd`] and [`p_enhanced`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SteeringGains {
    /// Proportional gain on the heading error, pure number.
    pub(crate) kk_s: f64,
    /// Derivative gain on the heading error (PD only), in seconds.
    pub(crate) kd_s: f64,
    /// Above this speed, P-enhanced steers less, in m/s.
    pub(crate) min_speed: f64,
    /// Heading errors below this are damped further by P-enhanced, in radians.
    pub(crate) max_error: f64,
    /// How much P-enhanced damps with speed, pure number.
    pub(crate) decay_v: f64,
    /// How much P-enhanced damps small errors, per m/s above `min_speed`.
    pub(crate) decay_e: f64,
}

/// `kk_s` times the heading `error`, plus `kd_s` times its rate of change
/// since `previous` (clamped to [`MAX_DERIVATIVE_ACTION_RAD`]) - none on the
/// first call. Updates `previous`.
pub(crate) fn pd(
    gains: &SteeringGains,
    error: f64,
    previous: &mut Option<(f64, Instant)>,
    now: Instant,
) -> f64 {
    let derivative = match *previous {
        Some((previous_error, at)) => {
            let dt_s = now.saturating_duration_since(at).as_secs_f64();
            if dt_s > 0.0 {
                (gains.kd_s * (error - previous_error) / dt_s)
                    .clamp(-MAX_DERIVATIVE_ACTION_RAD, MAX_DERIVATIVE_ACTION_RAD)
            } else {
                0.0
            }
        }
        None => 0.0,
    };
    *previous = Some((error, now));
    gains.kk_s * error + derivative
}

/// The P-enhanced controller: `kk_s` times the heading `error`, damped
/// above `min_speed` by `(min_speed / speed)^decay_v` and, for errors up to
/// `max_error`, further by `|error / max_error|^((speed - min_speed) decay_e)`
/// - so small errors at speed barely steer.
pub(crate) fn p_enhanced(gains: &SteeringGains, error: f64, speed_mps: f64) -> f64 {
    let mut steering = gains.kk_s * error;
    if speed_mps >= gains.min_speed && speed_mps > 0.0 {
        steering *= (gains.min_speed / speed_mps).powf(gains.decay_v);
        if error != 0.0 && error.abs() <= gains.max_error {
            let exponent = (speed_mps - gains.min_speed) * gains.decay_e;
            steering *= (error / gains.max_error).abs().powf(exponent);
        }
    }
    steering
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn gains() -> SteeringGains {
        SteeringGains {
            kk_s: 0.9,
            kd_s: 0.0,
            min_speed: 2.5,
            max_error: 0.15,
            decay_v: 0.2,
            decay_e: 0.2,
        }
    }

    #[test]
    fn p_enhanced_damps_small_errors_at_speed() {
        let gains = gains();
        assert_eq!(
            p_enhanced(&gains, 0.1, gains.min_speed - 1.0),
            gains.kk_s * 0.1
        );
        let fast = p_enhanced(&gains, 0.1, gains.min_speed + 2.0);
        assert!(fast > 0.0 && fast < gains.kk_s * 0.1, "{fast}");
    }

    #[test]
    fn the_pd_derivative_is_clamped_and_skipped_on_the_first_tick() {
        let gains = SteeringGains {
            kk_s: 1.0,
            kd_s: 1.0,
            ..gains()
        };
        let mut previous = None;
        let start = Instant::now();
        assert_eq!(pd(&gains, 0.1, &mut previous, start), 0.1);
        let steering = pd(
            &gains,
            0.2,
            &mut previous,
            start + Duration::from_millis(10),
        );
        assert!(
            (steering - (0.2 + MAX_DERIVATIVE_ACTION_RAD)).abs() < 1e-12,
            "{steering}"
        );
    }
}
