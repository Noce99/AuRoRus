//! [`SimulatedImu`]: a synthetic inertial sensor + wheel-speed feedback that
//! samples the simulated vehicle's true motion and corrupts it with
//! configurable noise, scale error and bias, for exercising dead reckoning,
//! localization and mapping against realistically imperfect readings
//! without real hardware. Its noise is tuned live through
//! [`SIMULATED_IMU_PARAMETERS_TOPIC_NAME`].

use crate::topics::{
    AlgorithmParameter, IMU_TOPIC_NAME, ImuReading, SIMULATED_IMU_PARAMETERS_TOPIC_NAME,
    SIMULATED_IMU_STATUS_TOPIC_NAME, SimulatedSensorParameters, SimulatedSensorStatus,
    VEHICLE_STATUS_TOPIC_NAME, VehicleStatus,
};
use crate::{Captain, Executor, Ticker};
use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};
use std::any::Any;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// Every tunable parameter [`SimulatedImu`] needs - loaded from
/// `config/simulation/imu.toml` (see [`Default`]) or from an
/// arbitrary path via [`crate::config::load`]. See that file for how the
/// error terms combine. Only `noise_scale` is live-tunable, see
/// [`tunable_parameters`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SimulatedImuConfig {
    /// Rate at which [`SimulatedImu`] publishes a new reading, in Hz.
    pub rate_hz: f64,
    /// Seed for the noise generator; `0` picks a fresh random one per run.
    pub seed: u64,
    /// Multiplies every standard deviation and bias below - `0.0` gives a
    /// perfect sensor.
    pub noise_scale: f64,
    /// White noise on the wheel speed, standard deviation in meters/second.
    pub speed_std_mps: f64,
    /// Standard deviation of the per-run multiplicative wheel-speed error.
    pub speed_scale_error_std: f64,
    /// White noise on the yaw rate, standard deviation in radians/second.
    pub yaw_rate_std_rad_s: f64,
    /// Constant yaw-rate bias, in radians/second.
    pub yaw_rate_bias_rad_s: f64,
    /// White noise on each accelerometer axis, standard deviation in
    /// meters/second^2.
    pub accel_std_mps2: f64,
    /// Constant bias on each accelerometer axis, in meters/second^2.
    pub accel_bias_mps2: f64,
}

impl Default for SimulatedImuConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/simulation/imu.toml"))
            .expect("config/simulation/imu.toml must deserialize into SimulatedImuConfig")
    }
}

/// Every live-tunable parameter, each named after the [`SimulatedImuConfig`]
/// field it sets, with its `value` left at `0` - see
/// [`crate::config::refresh_parameter_values`].
pub fn tunable_parameters() -> Vec<AlgorithmParameter> {
    vec![
        AlgorithmParameter::float("noise_scale", 0.0, 10.0, 0.05).description(
            "Multiplies every noise and bias - 0 gives a perfect sensor. Odometry \
             already drifted stays drifted: clear the map after changing it.",
        ),
    ]
}

/// Where the simulated IMU keeps its config: `config/simulation/imu.toml`,
/// relative to the working directory - rewritten by [`save_parameters`].
pub fn config_path() -> PathBuf {
    Path::new(crate::config::DEFAULT_CONFIG_ROOT)
        .join("simulation")
        .join("imu.toml")
}

/// Writes `parameters`' values into [`config_path`], keeping the rest of
/// the file. Returns the file's path.
pub fn save_parameters(parameters: &[AlgorithmParameter]) -> Result<PathBuf, String> {
    let path = config_path();
    crate::config::save_parameter_values(&path, parameters)?;
    Ok(path)
}

/// The values [`config_path`] holds for `parameters`, by name, and the
/// file's path - e.g. to go back to what was last saved.
pub fn saved_values(
    parameters: &[AlgorithmParameter],
) -> Result<(BTreeMap<String, f64>, PathBuf), String> {
    let path = config_path();
    Ok((
        crate::config::saved_parameter_values(&path, parameters)?,
        path,
    ))
}

/// A synthetic IMU: claims [`IMU_TOPIC_NAME`] and, at
/// [`SimulatedImuConfig::rate_hz`], publishes an [`ImuReading`] of the true
/// body-frame motion on [`VEHICLE_STATUS_TOPIC_NAME`] as corrupted by a
/// [`NoiseModel`]. Applies whatever [`SimulatedSensorParameters`] asks for
/// before the next reading, reporting it on
/// [`SIMULATED_IMU_STATUS_TOPIC_NAME`].
pub struct SimulatedImu {
    id: u16,
    name: String,
    config: SimulatedImuConfig,
}

impl SimulatedImu {
    pub fn new(name: impl Into<String>, config: SimulatedImuConfig) -> Self {
        Self {
            id: 0,
            name: name.into(),
            config,
        }
    }

    /// Applies any new [`SimulatedSensorParameters`] to the config. Returns
    /// whether the config changed.
    fn apply_parameters(&mut self, captain: &Captain, seen_write_count: &mut u64) -> bool {
        // Nothing may publish parameters at all (e.g. a binary without
        // `web_gui`) - then the config stays as loaded.
        let Some(requests) =
            captain.try_topic::<SimulatedSensorParameters>(SIMULATED_IMU_PARAMETERS_TOPIC_NAME)
        else {
            return false;
        };
        let write_count = requests.meta().write_count;
        if write_count == *seen_write_count {
            return false;
        }
        *seen_write_count = write_count;
        let wanted = requests.read().into_value().values;
        crate::config::apply_parameters(&mut self.config, &tunable_parameters(), &wanted)
    }

    /// The tunable parameters with the values in effect.
    fn parameters(&self) -> Vec<AlgorithmParameter> {
        let mut parameters = tunable_parameters();
        crate::config::refresh_parameter_values(&mut parameters, &self.config);
        parameters
    }
}

impl Executor for SimulatedImu {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_writer::<ImuReading>(IMU_TOPIC_NAME, self.id, ImuReading::default);
        let parameters = self.parameters();
        captain.claim_writer::<SimulatedSensorStatus>(
            SIMULATED_IMU_STATUS_TOPIC_NAME,
            self.id,
            move || SimulatedSensorStatus {
                parameters: parameters.clone(),
            },
        );
    }

    fn run(&mut self, captain: &Captain) {
        let imu_topic = captain.topic::<ImuReading>(IMU_TOPIC_NAME);
        let vehicle_topic = captain.topic::<VehicleStatus>(VEHICLE_STATUS_TOPIC_NAME);
        let status_topic = captain.topic::<SimulatedSensorStatus>(SIMULATED_IMU_STATUS_TOPIC_NAME);
        let mut noise = NoiseModel::new(self.config);
        let mut ticker = Ticker::new(self.config.rate_hz);
        let mut seen_parameters = 0;

        while captain.is_running(self.id) {
            if self.apply_parameters(captain, &mut seen_parameters) {
                // Keeps the per-run errors already drawn: only their scale changes.
                noise.config = self.config;
                status_topic
                    .write(
                        self.id,
                        SimulatedSensorStatus {
                            parameters: self.parameters(),
                        },
                    )
                    .expect("lost writer authorization for the simulated_imu_status topic");
            }
            let truth = vehicle_topic.read().into_value();
            imu_topic
                .write(self.id, noise.measure(&truth))
                .expect("lost writer authorization for the imu topic");
            ticker.wait();
        }
    }

    fn name(&self) -> String {
        self.name.clone()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn fresh(&self) -> Box<dyn Executor> {
        Box::new(SimulatedImu::new(self.name.clone(), self.config))
    }
}

/// The error model [`SimulatedImu`] applies to every sample: per-run errors
/// (the wheel-speed scale error) are drawn once in [`NoiseModel::new`],
/// white noise fresh in every [`NoiseModel::measure`]. Every term is scaled
/// by `config.noise_scale` when measuring, so it can change between two
/// samples.
struct NoiseModel {
    config: SimulatedImuConfig,
    rng: StdRng,
    /// Not yet multiplied by [`SimulatedImuConfig::noise_scale`].
    speed_scale_error: f64,
}

impl NoiseModel {
    fn new(config: SimulatedImuConfig) -> Self {
        let seed = match config.seed {
            0 => rand::rng().random(),
            seed => seed,
        };
        let mut rng = StdRng::seed_from_u64(seed);
        let speed_scale_error = gaussian(&mut rng, config.speed_scale_error_std);
        Self {
            config,
            rng,
            speed_scale_error,
        }
    }

    /// What the sensor reports for the true motion `truth`.
    fn measure(&mut self, truth: &VehicleStatus) -> ImuReading {
        let c = self.config;
        let k = c.noise_scale;
        let rng = &mut self.rng;
        ImuReading {
            wheel_speed_mps: truth.vx_mps * (1.0 + self.speed_scale_error * k)
                + gaussian(rng, c.speed_std_mps * k),
            yaw_rate_rad_s: truth.yaw_rate_rad_s
                + c.yaw_rate_bias_rad_s * k
                + gaussian(rng, c.yaw_rate_std_rad_s * k),
            ax_mps2: truth.ax_mps2 + c.accel_bias_mps2 * k + gaussian(rng, c.accel_std_mps2 * k),
            ay_mps2: truth.ay_mps2 + c.accel_bias_mps2 * k + gaussian(rng, c.accel_std_mps2 * k),
        }
    }
}

/// One sample from a zero-mean normal distribution with standard deviation
/// `std`, via the Box-Muller transform. Exactly `0.0` when `std` is `0.0`,
/// so a zeroed error term leaves the truth untouched bit for bit.
pub(super) fn gaussian(rng: &mut StdRng, std: f64) -> f64 {
    // `1.0 - random()` lands in (0, 1], keeping `ln` finite.
    let u1: f64 = 1.0 - rng.random::<f64>();
    let u2: f64 = rng.random();
    std * (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config(noise_scale: f64, seed: u64) -> SimulatedImuConfig {
        SimulatedImuConfig {
            noise_scale,
            seed,
            ..SimulatedImuConfig::default()
        }
    }

    fn turning_truth() -> VehicleStatus {
        VehicleStatus {
            speed_mps: 3.0,
            vx_mps: 3.0,
            yaw_rate_rad_s: 1.5,
            ay_mps2: 4.5,
            ..VehicleStatus::default()
        }
    }

    #[test]
    fn the_checked_in_config_deserializes() {
        let config = SimulatedImuConfig::default();
        assert!(config.rate_hz > 0.0);
    }

    #[test]
    fn a_zero_noise_scale_reports_the_truth_exactly() {
        let truth = turning_truth();
        let mut noise = NoiseModel::new(config(0.0, 7));
        for _ in 0..100 {
            assert_eq!(
                noise.measure(&truth),
                ImuReading {
                    wheel_speed_mps: 3.0,
                    yaw_rate_rad_s: 1.5,
                    ax_mps2: 0.0,
                    ay_mps2: 4.5,
                }
            );
        }
    }

    #[test]
    fn noise_scale_can_be_zeroed_mid_run() {
        let truth = turning_truth();
        let mut noise = NoiseModel::new(config(1.0, 7));
        assert_ne!(noise.measure(&truth).yaw_rate_rad_s, truth.yaw_rate_rad_s);
        noise.config.noise_scale = 0.0;
        assert_eq!(noise.measure(&truth).wheel_speed_mps, truth.vx_mps);
        assert_eq!(noise.measure(&truth).yaw_rate_rad_s, truth.yaw_rate_rad_s);
    }

    /// Every declared parameter names a numeric config field - this panics
    /// otherwise - and the shipped defaults sit inside their slider ranges.
    #[test]
    fn every_parameter_names_a_config_field_within_its_range() {
        let mut parameters = tunable_parameters();
        crate::config::refresh_parameter_values(&mut parameters, &SimulatedImuConfig::default());
        for parameter in parameters {
            assert_eq!(
                parameter.kind.sanitize(parameter.value),
                Some(parameter.value),
                "{} = {} is outside its range",
                parameter.name,
                parameter.value
            );
        }
    }

    #[test]
    fn the_same_seed_gives_the_same_readings() {
        let truth = turning_truth();
        let mut a = NoiseModel::new(config(1.0, 42));
        let mut b = NoiseModel::new(config(1.0, 42));
        for _ in 0..100 {
            assert_eq!(a.measure(&truth), b.measure(&truth));
        }
    }

    #[test]
    fn yaw_rate_noise_averages_to_the_truth_plus_the_bias_with_the_configured_spread() {
        let config = SimulatedImuConfig {
            noise_scale: 2.0,
            seed: 1,
            yaw_rate_std_rad_s: 0.05,
            yaw_rate_bias_rad_s: 0.01,
            ..SimulatedImuConfig::default()
        };
        let truth = turning_truth();
        let mut noise = NoiseModel::new(config);
        let n = 20_000;
        let errors: Vec<f64> = (0..n)
            .map(|_| noise.measure(&truth).yaw_rate_rad_s - truth.yaw_rate_rad_s)
            .collect();
        let mean = errors.iter().sum::<f64>() / n as f64;
        let std = (errors.iter().map(|e| (e - mean).powi(2)).sum::<f64>() / n as f64).sqrt();

        // Both scaled by noise_scale = 2.
        assert!((mean - 0.02).abs() < 0.003, "mean error {mean}");
        assert!((std - 0.1).abs() < 0.005, "error std {std}");
    }
}
