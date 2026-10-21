//! [`SensorModel`]: how likely a lidar reading is, given the range the map
//! predicts - the beam model of UBM's particle filter
//! (`get_ray_behaviour_sensor_model`) and range_libc's
//! `eval_sensor_model`, precomputed as a table over whole pixels.
//!
//! Every reading is a mix of four behaviours: a hit near the predicted
//! range (Gaussian), a short reading off something not on the map, a
//! max-range "nothing came back", and uniform noise.

/// The four mixture weights and the hit's spread.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BeamWeights {
    pub short: f64,
    pub max: f64,
    pub random: f64,
    pub hit: f64,
    /// Standard deviation of a hit around the predicted range, in pixels.
    pub hit_sigma_px: f64,
}

/// `ln p(observed | predicted)`, for every whole-pixel pair up to the
/// maximum range.
pub struct SensorModel {
    /// `max_range_px + 1`.
    width: usize,
    /// Row-major `[observed][predicted]`, as range_libc's
    /// `sensor_model[r][d]`.
    log_table: Vec<f64>,
}

impl SensorModel {
    /// The table for ranges up to `max_range_px` - UBM's
    /// `get_ray_behaviour_sensor_model`. Each predicted range's column is
    /// normalized to sum to `1` before the logarithm is taken.
    pub fn new(max_range_px: usize, weights: BeamWeights) -> Self {
        let width = max_range_px + 1;
        let max_range = max_range_px as f64;
        let sigma = weights.hit_sigma_px.max(1e-6);
        let mut table = vec![0.0; width * width];
        for d in 0..width {
            let mut norm = 0.0;
            for r in 0..width {
                let range_diff = r as f64 - d as f64;
                let mut prob = weights.hit
                    * (-(range_diff * range_diff) / (2.0 * sigma * sigma)).exp()
                    / (sigma * (2.0 * std::f64::consts::PI).sqrt());
                // A short reading: something in front of the predicted wall.
                if r < d {
                    prob += 2.0 * weights.short * (d - r) as f64 / d as f64;
                }
                // A max-range reading.
                if r == max_range_px {
                    prob += weights.max;
                }
                // Uniform noise.
                if r < max_range_px {
                    prob += weights.random / max_range;
                }
                norm += prob;
                table[r * width + d] = prob;
            }
            for r in 0..width {
                table[r * width + d] /= norm;
            }
        }
        Self {
            width,
            log_table: table.into_iter().map(f64::ln).collect(),
        }
    }

    /// `ln p(observed | predicted)` for ranges in pixels - each truncated
    /// to a whole pixel and clamped into the table, as range_libc's
    /// `eval_sensor_model` does.
    pub fn log_likelihood(&self, observed_px: f32, predicted_px: f32) -> f64 {
        let last = (self.width - 1) as f32;
        let r = observed_px.clamp(0.0, last) as usize;
        let d = predicted_px.clamp(0.0, last) as usize;
        self.log_table[r * self.width + d]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn weights() -> BeamWeights {
        BeamWeights {
            short: 0.01,
            max: 0.07,
            random: 0.22,
            hit: 0.75,
            hit_sigma_px: 8.0,
        }
    }

    #[test]
    fn every_predicted_range_s_column_sums_to_one() {
        let model = SensorModel::new(200, weights());
        for d in [0usize, 1, 50, 199, 200] {
            let sum: f64 = (0..=200)
                .map(|r| model.log_likelihood(r as f32, d as f32).exp())
                .sum();
            assert!((sum - 1.0).abs() < 1e-9, "column {d} sums to {sum}");
        }
    }

    #[test]
    fn a_reading_at_the_predicted_range_is_the_likeliest() {
        let model = SensorModel::new(200, weights());
        let at = model.log_likelihood(100.0, 100.0);
        assert!(at > model.log_likelihood(80.0, 100.0));
        assert!(at > model.log_likelihood(120.0, 100.0));
        // Equally far off, short beats long: something may be in the way.
        assert!(model.log_likelihood(60.0, 100.0) > model.log_likelihood(140.0, 100.0));
    }

    #[test]
    fn ranges_beyond_the_table_are_clamped_into_it() {
        let model = SensorModel::new(200, weights());
        assert_eq!(
            model.log_likelihood(1e6, 1e6),
            model.log_likelihood(200.0, 200.0)
        );
        assert_eq!(
            model.log_likelihood(-3.0, 10.0),
            model.log_likelihood(0.0, 10.0)
        );
    }
}
