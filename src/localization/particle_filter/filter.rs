//! [`ParticleFilter`]: Monte Carlo localization on a known map - the
//! algorithm of UBM's `particle_filter_cpp` node, with its bugs fixed (see
//! `plan/particle_filter.md` and `documentation/slam.md`).
//!
//! Each particle is a guess at the vehicle's pose on the map. On every
//! scan, each one moves by odometry's motion since the last scan, plus
//! noise; is weighted by how well the scan, cast from it, matches the map;
//! and the cloud is resampled toward the heavier ones once too few carry
//! the weight. The estimate is the weighted mean.

use super::ParticleFilterConfig;
use super::cddt::Cddt;
use super::omap::OMap;
use super::ray_marching::RayMarching;
use super::sensor_model::{BeamWeights, SensorModel};
use crate::localization::slam::pose::Pose2;
use crate::topics::{LidarScan, SelectedMap};
use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};

/// [`ParticleFilterConfig::ray_cast_algo`] picking [`RayMarching`] - any
/// other value picks [`Cddt`].
const RAY_CAST_RAY_MARCHING: u8 = 1;

/// Below this much motion between two scans, in meters and radians, the
/// vehicle is standing still: the particles aren't moved, nor spread.
const STANDSTILL_M: f64 = 1e-4;
const STANDSTILL_RAD: f64 = 1e-4;

/// What the filter believes after a scan.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Estimate {
    /// The weighted mean pose, in the map frame - its heading the circular
    /// mean.
    pub pose: Pose2,
    /// Weighted standard deviation of the particles' positions around it,
    /// in meters.
    pub spread_m: f64,
    /// Circular standard deviation of their headings, in radians.
    pub spread_rad: f64,
    /// Effective sample size `1 / sum(w^2)` before any resampling: how many
    /// particles actually carry the weight.
    pub effective_sample_size: f64,
    /// How many particles there are.
    pub particles: usize,
}

/// Where the next scan starts the filter from.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Start {
    /// Around odometry's pose composed onto this `map_to_odom`: where
    /// odometry was last reset, on the map.
    Anchor(Pose2),
    /// Around this pose on the map.
    Pose(Pose2),
    /// Anywhere on the map's free space, facing anywhere.
    Global,
}

/// The ray caster [`ParticleFilterConfig::ray_cast_algo`] picks.
enum Caster {
    Cddt(Cddt),
    RayMarching(RayMarching),
}

impl Caster {
    fn new(map: &OMap, config: &ParticleFilterConfig, max_range_px: f32) -> Self {
        if config.ray_cast_algo == RAY_CAST_RAY_MARCHING {
            Caster::RayMarching(RayMarching::new(map, max_range_px))
        } else {
            let mut cddt = Cddt::new(map, max_range_px, config.theta_discretization as u32);
            cddt.prune(max_range_px);
            Caster::Cddt(cddt)
        }
    }

    fn calc_range(&self, x: f32, y: f32, heading: f32) -> f32 {
        match self {
            Caster::Cddt(cddt) => cddt.calc_range(x, y, heading),
            Caster::RayMarching(rm) => rm.calc_range(x, y, heading),
        }
    }
}

/// The beams of a scan the filter uses: every `range_step`th one across
/// `considered_lidar_fov_rad`, centered on the sensor's forward direction.
#[derive(Debug, Clone, PartialEq)]
struct Beams {
    /// What they were picked for - rebuilt when any of it changes.
    fov: f32,
    num_points: usize,
    considered_fov_rad: f64,
    range_step: usize,
    indices: Vec<usize>,
    /// Each one's angle from the sensor's forward direction, in radians.
    angles: Vec<f32>,
}

impl Beams {
    /// UBM's `scan_callback` first-scan setup (`rifa`/`rafi`), with the
    /// considered field of view clamped to the sensor's rather than
    /// refused.
    fn new(scan: &LidarScan, considered_fov_rad: f64, range_step: usize) -> Self {
        let n = scan.num_lidar_points;
        let (mut indices, mut angles) = (Vec::new(), Vec::new());
        if n > 0 {
            let fov = f64::from(scan.fov);
            let considered = considered_fov_rad.clamp(0.0, fov);
            let increment = if n > 1 { fov / (n - 1) as f64 } else { 1.0 };
            let index_of = |angle: f64| {
                ((angle + fov / 2.0) / increment)
                    .round()
                    .clamp(0.0, (n - 1) as f64) as usize
            };
            let (start, end) = (index_of(-considered / 2.0), index_of(considered / 2.0));
            for i in (start..=end).step_by(range_step.max(1)) {
                indices.push(i);
                angles.push(scan.angle_rad(i));
            }
        }
        Self {
            fov: scan.fov,
            num_points: n,
            considered_fov_rad,
            range_step,
            indices,
            angles,
        }
    }

    fn fits(&self, scan: &LidarScan, config: &ParticleFilterConfig) -> bool {
        self.fov == scan.fov
            && self.num_points == scan.num_lidar_points
            && self.considered_fov_rad == config.considered_lidar_fov_rad
            && self.range_step == config.range_step
    }
}

/// The map in the filter's terms: its occupancy, pixel frame and ray caster.
struct MapModel {
    omap: OMap,
    origin_x_m: f64,
    origin_y_m: f64,
    resolution_m: f64,
    caster: Caster,
    sensor: SensorModel,
    max_range_px: f32,
}

impl MapModel {
    fn new(
        omap: OMap,
        origin: (f64, f64),
        resolution_m: f64,
        config: &ParticleFilterConfig,
    ) -> Self {
        let max_range_px = (config.max_range_m / resolution_m).floor().max(1.0);
        let caster = Caster::new(&omap, config, max_range_px as f32);
        let sensor = SensorModel::new(max_range_px as usize, beam_weights(config));
        Self {
            omap,
            origin_x_m: origin.0,
            origin_y_m: origin.1,
            resolution_m,
            caster,
            sensor,
            max_range_px: max_range_px as f32,
        }
    }

    fn to_px(&self, x_m: f64, y_m: f64) -> (f32, f32) {
        (
            ((x_m - self.origin_x_m) / self.resolution_m) as f32,
            ((y_m - self.origin_y_m) / self.resolution_m) as f32,
        )
    }
}

fn beam_weights(config: &ParticleFilterConfig) -> BeamWeights {
    BeamWeights {
        short: config.ray_short_weight,
        max: config.ray_max_weight,
        random: config.ray_rand_weight,
        hit: config.ray_hit_weight,
        hit_sigma_px: config.ray_hit_sigma_px,
    }
}

/// The particle filter's state on one map.
pub struct ParticleFilter {
    config: ParticleFilterConfig,
    map: MapModel,
    /// Free pixel centers, in meters - built on the first global start.
    free_space: Option<Vec<(f64, f64)>>,
    particles: Vec<Pose2>,
    /// Normalized, parallel to `particles`.
    weights: Vec<f64>,
    /// Odometry's pose at the last scan, in the `odom` frame - `None` right
    /// after a (re)start, so the next scan doesn't move anything.
    last_odometry: Option<Pose2>,
    /// Applied on the next scan.
    start: Option<Start>,
    /// Whether a global start hasn't converged yet.
    relocalizing: bool,
    /// Scans in a row a global start's cloud has been tight for.
    converged_scans: usize,
    beams: Option<Beams>,
    estimate: Option<Estimate>,
    /// The `odom` frame's origin on the map, at the last estimate.
    map_to_odom: Option<Pose2>,
    rng: StdRng,
}

impl ParticleFilter {
    /// A filter on `map`, started from `start` on the first scan - `None`
    /// if no map is loaded.
    pub fn new(config: ParticleFilterConfig, map: &SelectedMap, start: Start) -> Option<Self> {
        let info = map.info.as_ref()?;
        let model = MapModel::new(
            OMap::from_selected_map(map),
            (info.origin.x, info.origin.y),
            info.resolution_m_per_px,
            &config,
        );
        Some(Self {
            config,
            map: model,
            free_space: None,
            particles: Vec::new(),
            weights: Vec::new(),
            last_odometry: None,
            start: Some(start),
            relocalizing: false,
            converged_scans: 0,
            beams: None,
            estimate: None,
            map_to_odom: None,
            rng: StdRng::seed_from_u64(config.seed),
        })
    }

    /// Runs with `config` from the next scan on. A change to anything the
    /// map model is built from rebuilds it; a change of particle count
    /// restarts around the current estimate.
    pub fn set_config(&mut self, config: ParticleFilterConfig) {
        let old = self.config;
        self.config = config;
        let rebuild = old.max_range_m != config.max_range_m
            || old.ray_cast_algo != config.ray_cast_algo
            || old.theta_discretization != config.theta_discretization
            || beam_weights(&old) != beam_weights(&config);
        if rebuild {
            let omap = std::mem::replace(&mut self.map.omap, OMap::new(0, 0));
            self.map = MapModel::new(
                omap,
                (self.map.origin_x_m, self.map.origin_y_m),
                self.map.resolution_m,
                &config,
            );
        }
        if old.num_particles != config.num_particles
            && !self.relocalizing
            && let Some(estimate) = self.estimate
            && self.start.is_none()
        {
            self.start = Some(Start::Pose(estimate.pose));
        }
    }

    /// Restarts from `start` on the next scan.
    pub fn restart(&mut self, start: Start) {
        self.start = Some(start);
    }

    /// The latest estimate - `None` before the first scan.
    pub fn estimate(&self) -> Option<Estimate> {
        self.estimate
    }

    /// Where the `odom` frame's origin sits on the map: odometry's pose
    /// composed onto this is the estimated pose - `None` before the first
    /// scan.
    pub fn map_to_odom(&self) -> Option<Pose2> {
        self.map_to_odom
    }

    /// Whether a global start hasn't converged yet.
    pub fn relocalizing(&self) -> bool {
        self.relocalizing
    }

    /// About `count` particles, evenly picked across the cloud, for
    /// drawing.
    pub fn sample_particles(&self, count: usize) -> Vec<Pose2> {
        if count == 0 || self.particles.is_empty() {
            return Vec::new();
        }
        let stride = self.particles.len().div_ceil(count).max(1);
        self.particles.iter().step_by(stride).copied().collect()
    }

    /// Takes `scan`, taken where odometry says `odometry` (in the `odom`
    /// frame): moves, weighs, and maybe resamples the particles.
    pub fn process(&mut self, odometry: Pose2, scan: &LidarScan) -> Estimate {
        if let Some(start) = self.start.take() {
            match start {
                Start::Anchor(map_to_odom) => self.spread_around(map_to_odom.compose(&odometry)),
                Start::Pose(pose) => self.spread_around(pose),
                Start::Global => self.spread_globally(),
            }
            self.last_odometry = None;
        }
        if let Some(last) = self.last_odometry {
            self.apply_motion(&last.inverse().compose(&odometry));
        }
        self.last_odometry = Some(odometry);

        if !self
            .beams
            .as_ref()
            .is_some_and(|beams| beams.fits(scan, &self.config))
        {
            self.beams = Some(Beams::new(
                scan,
                self.config.considered_lidar_fov_rad,
                self.config.range_step,
            ));
        }
        self.weigh(scan);

        let estimate = self.compute_estimate();
        if self.relocalizing && estimate.spread_m <= self.config.global_converged_spread_m {
            self.converged_scans += 1;
        } else {
            self.converged_scans = 0;
        }
        if self.relocalizing && self.converged_scans >= self.config.global_converged_scans.max(1) {
            self.relocalizing = false;
            // Down to the tracking count right away.
            self.resample(self.config.num_particles.max(1));
        } else if estimate.effective_sample_size
            < self.config.resample_threshold * self.particles.len() as f64
        {
            let count = if self.relocalizing {
                self.particles.len()
            } else {
                self.config.num_particles.max(1)
            };
            self.resample(count);
        }
        self.estimate = Some(estimate);
        self.map_to_odom = Some(estimate.pose.compose(&odometry.inverse()));
        estimate
    }

    /// A Gaussian cloud of [`ParticleFilterConfig::num_particles`] around
    /// `pose`, with [`ParticleFilterConfig::initial_pose_stddev_x_m`] and
    /// co. - UBM's `rviz_pose_callback`.
    fn spread_around(&mut self, pose: Pose2) {
        let n = self.config.num_particles.max(1);
        let config = self.config;
        self.particles = (0..n)
            .map(|_| {
                Pose2::new(
                    pose.x_m + gaussian(&mut self.rng, config.initial_pose_stddev_x_m),
                    pose.y_m + gaussian(&mut self.rng, config.initial_pose_stddev_y_m),
                    pose.heading_rad
                        + gaussian(&mut self.rng, config.initial_pose_stddev_heading_rad),
                )
            })
            .collect();
        self.weights = vec![1.0 / n as f64; n];
        self.relocalizing = false;
    }

    /// [`ParticleFilterConfig::global_num_particles`] particles, uniform
    /// over the map's free pixels and over headings.
    fn spread_globally(&mut self) {
        let free_space = self.free_space.get_or_insert_with(|| {
            let (origin_x, origin_y, resolution) = (
                self.map.origin_x_m,
                self.map.origin_y_m,
                self.map.resolution_m,
            );
            self.map
                .omap
                .free_pixels()
                .into_iter()
                .map(|(x, y)| {
                    (
                        origin_x + (x as f64 + 0.5) * resolution,
                        origin_y + (y as f64 + 0.5) * resolution,
                    )
                })
                .collect()
        });
        if free_space.is_empty() {
            return;
        }
        let n = self.config.global_num_particles.max(1);
        let half_pixel = self.map.resolution_m / 2.0;
        self.particles = (0..n)
            .map(|_| {
                let (x, y) = free_space[self.rng.random_range(0..free_space.len())];
                Pose2::new(
                    x + self.rng.random_range(-half_pixel..half_pixel),
                    y + self.rng.random_range(-half_pixel..half_pixel),
                    self.rng
                        .random_range(-std::f64::consts::PI..std::f64::consts::PI),
                )
            })
            .collect();
        self.weights = vec![1.0 / n as f64; n];
        self.relocalizing = true;
        self.converged_scans = 0;
    }

    /// Moves every particle by `delta` (odometry's motion since the last
    /// scan, in the vehicle's frame at the last scan), plus Gaussian noise
    /// growing with the motion. Fixes UBM's `odom_callback`: both
    /// components of the rotated delta come from the original one, the
    /// heading change is wrapped, and standing still adds no noise.
    fn apply_motion(&mut self, delta: &Pose2) {
        let distance_m = delta.x_m.hypot(delta.y_m);
        let turn_rad = delta.heading_rad.abs();
        if distance_m < STANDSTILL_M && turn_rad < STANDSTILL_RAD {
            return;
        }
        let config = self.config;
        let sigma_x = config.motion_dispersion_stddev_x_m + config.motion_noise_per_m * distance_m;
        let sigma_y = config.motion_dispersion_stddev_y_m + config.motion_noise_per_m * distance_m;
        let sigma_heading =
            config.motion_dispersion_stddev_heading_rad + config.motion_noise_per_rad * turn_rad;
        for particle in &mut self.particles {
            let noisy = Pose2::new(
                delta.x_m + gaussian(&mut self.rng, sigma_x),
                delta.y_m + gaussian(&mut self.rng, sigma_y),
                delta.heading_rad + gaussian(&mut self.rng, sigma_heading),
            );
            *particle = particle.compose(&noisy);
        }
    }

    /// Multiplies every particle's weight by the likelihood of `scan` cast
    /// from it (raised to `1 / squash_factor`), and normalizes. Computed in
    /// logs, which can't underflow as range_libc's product does.
    fn weigh(&mut self, scan: &LidarScan) {
        let beams = self.beams.as_ref().expect("built before weighing");
        let map = &self.map;
        let resolution = map.resolution_m as f32;
        let max_range_m = map.max_range_px * resolution;
        let observed_px: Vec<f32> = beams
            .indices
            .iter()
            .map(|&i| {
                let range = scan.points.get(i).copied().unwrap_or(f32::NAN);
                let usable =
                    range.is_finite() && range > scan.min_distance && range < scan.max_distance;
                // A reading that came back with nothing is a max-range one.
                (if usable {
                    range.min(max_range_m)
                } else {
                    max_range_m
                }) / resolution
            })
            .collect();
        let mount = Pose2::new(f64::from(scan.mount_x_m), f64::from(scan.mount_y_m), 0.0);
        let squash = if self.relocalizing {
            self.config.global_squash_factor
        } else {
            self.config.squash_factor
        };
        let inverse_squash = 1.0 / squash.max(1e-6);

        let log_weight = |particle: &Pose2| -> f64 {
            let sensor = particle.compose(&mount);
            let (x, y) = map.to_px(sensor.x_m, sensor.y_m);
            let heading = sensor.heading_rad as f32;
            let mut sum = 0.0;
            for (&angle, &observed) in beams.angles.iter().zip(&observed_px) {
                let predicted = map.caster.calc_range(x, y, heading + angle);
                sum += map.sensor.log_likelihood(observed, predicted);
            }
            sum * inverse_squash
        };

        let mut log_weights = vec![0.0; self.particles.len()];
        let threads = self.config.threads.clamp(
            1,
            std::thread::available_parallelism().map_or(1, usize::from),
        );
        let chunk = self.particles.len().div_ceil(threads).max(1);
        if threads == 1 {
            for (out, particle) in log_weights.iter_mut().zip(&self.particles) {
                *out = log_weight(particle);
            }
        } else {
            std::thread::scope(|scope| {
                for (outs, particles) in log_weights
                    .chunks_mut(chunk)
                    .zip(self.particles.chunks(chunk))
                {
                    let log_weight = &log_weight;
                    scope.spawn(move || {
                        for (out, particle) in outs.iter_mut().zip(particles) {
                            *out = log_weight(particle);
                        }
                    });
                }
            });
        }

        for (log_weight, &weight) in log_weights.iter_mut().zip(&self.weights) {
            *log_weight += weight.max(f64::MIN_POSITIVE).ln();
        }
        let max = log_weights
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        let mut sum = 0.0;
        for (weight, &log_weight) in self.weights.iter_mut().zip(&log_weights) {
            *weight = (log_weight - max).exp();
            sum += *weight;
        }
        if sum.is_finite() && sum > 0.0 {
            for weight in &mut self.weights {
                *weight /= sum;
            }
        } else {
            let uniform = 1.0 / self.weights.len().max(1) as f64;
            self.weights.fill(uniform);
        }
    }

    /// The weighted mean pose - its heading the circular mean, not UBM's
    /// linear one, which breaks across +-pi - and the spread around it.
    fn compute_estimate(&self) -> Estimate {
        let (mut x, mut y, mut sin, mut cos, mut squared) = (0.0, 0.0, 0.0, 0.0, 0.0);
        for (particle, &weight) in self.particles.iter().zip(&self.weights) {
            x += weight * particle.x_m;
            y += weight * particle.y_m;
            let (s, c) = particle.heading_rad.sin_cos();
            sin += weight * s;
            cos += weight * c;
            squared += weight * weight;
        }
        let spread_m = self
            .particles
            .iter()
            .zip(&self.weights)
            .map(|(particle, &weight)| {
                weight * ((particle.x_m - x).powi(2) + (particle.y_m - y).powi(2))
            })
            .sum::<f64>()
            .sqrt();
        let resultant = sin.hypot(cos).clamp(1e-12, 1.0);
        Estimate {
            pose: Pose2::new(x, y, sin.atan2(cos)),
            spread_m,
            spread_rad: (-2.0 * resultant.ln()).sqrt(),
            effective_sample_size: if squared > 0.0 { 1.0 / squared } else { 0.0 },
            particles: self.particles.len(),
        }
    }

    /// Draws `count` particles in proportion to their weights, by
    /// low-variance (systematic) resampling - O(N), where UBM's search of
    /// the CDF per particle is O(N^2) - and makes the weights uniform.
    fn resample(&mut self, count: usize) {
        if self.particles.is_empty() {
            return;
        }
        let step = 1.0 / count as f64;
        let mut target = self.rng.random_range(0.0..step);
        let mut cumulative = self.weights[0];
        let mut source = 0;
        let mut resampled = Vec::with_capacity(count);
        for _ in 0..count {
            while target > cumulative && source + 1 < self.particles.len() {
                source += 1;
                cumulative += self.weights[source];
            }
            resampled.push(self.particles[source]);
            target += step;
        }
        self.particles = resampled;
        self.weights = vec![step; count];
    }
}

/// A sample of a zero-mean Gaussian with standard deviation `std` (Box-
/// Muller, as [`crate::simulation`]'s noise).
fn gaussian(rng: &mut StdRng, std: f64) -> f64 {
    if std <= 0.0 {
        return 0.0;
    }
    // `1.0 - random()` lands in (0, 1], keeping `ln` finite.
    let u1: f64 = 1.0 - rng.random::<f64>();
    let u2: f64 = rng.random();
    std * (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::environment::cast_ray;
    use crate::environment::{ImageOrigin, MapInfo, MapSource, StartFinishLine, WorldPoint};
    use crate::localization::slam::pose::wrap_to_pi;

    /// An L-shaped corridor - asymmetric, so a global start has one answer:
    /// white inside, black elsewhere.
    pub(crate) fn corridor_map() -> SelectedMap {
        let resolution = 0.05;
        let (width, height) = (200u32, 160u32);
        let mut pixels = Vec::with_capacity((width * height) as usize);
        for row in 0..height {
            for col in 0..width {
                let x = (f64::from(col) + 0.5) * resolution;
                let y = (f64::from(row) + 0.5) * resolution;
                let horizontal = (0.5..9.5).contains(&x) && (0.5..2.5).contains(&y);
                let vertical = (7.0..9.5).contains(&x) && (0.5..7.5).contains(&y);
                let alcove = (2.0..3.0).contains(&x) && (2.5..3.5).contains(&y);
                let pillar = (x - 5.0).hypot(y - 1.5) < 0.3;
                let free = (horizontal || vertical || alcove) && !pillar;
                pixels.push(if free { 255 } else { 0 });
            }
        }
        SelectedMap {
            path: None,
            width_px: width,
            height_px: height,
            pixels: pixels.into(),
            info: Some(MapInfo {
                resolution_m_per_px: resolution,
                width_px: width,
                height_px: height,
                origin: ImageOrigin {
                    x: 0.0,
                    y: 0.0,
                    theta_rad: 0.0,
                },
                start_finish_line: StartFinishLine {
                    a: WorldPoint { x: 0.0, y: 0.0 },
                    b: WorldPoint { x: 0.0, y: 0.0 },
                },
                generated_at: String::new(),
                source: MapSource::Real,
                generation: None,
            }),
        }
    }

    /// A noise-free 270-degree scan of `map` from `pose`, the sensor at the
    /// vehicle's pose.
    pub(crate) fn scan_from(map: &SelectedMap, pose: Pose2) -> LidarScan {
        let info = map.info.as_ref().unwrap();
        let (fov, n) = (4.712_389_f32, 1081);
        let points = (0..n)
            .map(|i| {
                let angle = pose.heading_rad + f64::from(LidarScan::ray_angle_rad(fov, n, i));
                cast_ray(map, info, pose.x_m, pose.y_m, angle, 30.0).0
            })
            .collect::<Vec<_>>();
        LidarScan::new(points, vec![0.0; n], 0.1, 30.0, fov)
    }

    fn config() -> ParticleFilterConfig {
        ParticleFilterConfig {
            threads: 2,
            ..ParticleFilterConfig::default()
        }
    }

    /// The heading difference `a - b`, wrapped to `(-pi, pi]`.
    fn heading_error(a: f64, b: f64) -> f64 {
        wrap_to_pi(a - b)
    }

    fn position_error(a: Pose2, b: Pose2) -> f64 {
        a.squared_distance(&b).sqrt()
    }

    #[test]
    fn zero_noise_motion_moves_every_particle_along_its_own_heading() {
        let map = corridor_map();
        let mut filter = ParticleFilter::new(
            ParticleFilterConfig {
                motion_dispersion_stddev_x_m: 0.0,
                motion_dispersion_stddev_y_m: 0.0,
                motion_dispersion_stddev_heading_rad: 0.0,
                motion_noise_per_m: 0.0,
                motion_noise_per_rad: 0.0,
                ..config()
            },
            &map,
            Start::Global,
        )
        .unwrap();
        filter.particles = vec![Pose2::new(1.0, 1.0, 0.0), Pose2::new(2.0, 1.0, 1.5)];
        // Forward 1 m and 0.5 m to the left: UBM's rotation bug got the
        // lateral part wrong.
        filter.apply_motion(&Pose2::new(1.0, 0.5, 0.2));
        assert!(position_error(filter.particles[0], Pose2::new(2.0, 1.5, 0.2)) < 1e-12);
        // Facing 1.5 rad: forward is (cos 1.5, sin 1.5), left is (-sin 1.5,
        // cos 1.5).
        let (sin, cos) = 1.5f64.sin_cos();
        let expected = Pose2::new(2.0 + cos - 0.5 * sin, 1.0 + sin + 0.5 * cos, 1.7);
        assert!(position_error(filter.particles[1], expected) < 1e-12);
        assert!((filter.particles[1].heading_rad - 1.7).abs() < 1e-12);
    }

    #[test]
    fn a_heading_across_pi_averages_to_pi_not_zero() {
        let map = corridor_map();
        let mut filter = ParticleFilter::new(config(), &map, Start::Global).unwrap();
        filter.particles = vec![Pose2::new(1.0, 1.0, 3.1), Pose2::new(1.0, 1.0, -3.1)];
        filter.weights = vec![0.5, 0.5];
        let estimate = filter.compute_estimate();
        assert!(
            heading_error(estimate.pose.heading_rad, std::f64::consts::PI).abs() < 1e-9,
            "{}",
            estimate.pose.heading_rad
        );
    }

    #[test]
    fn standing_still_doesn_t_spread_the_cloud() {
        let map = corridor_map();
        let mut filter = ParticleFilter::new(config(), &map, Start::Global).unwrap();
        filter.particles = vec![Pose2::new(1.0, 1.0, 0.3); 10];
        filter.apply_motion(&Pose2::default());
        assert!(
            filter
                .particles
                .iter()
                .all(|p| *p == Pose2::new(1.0, 1.0, 0.3))
        );
    }

    #[test]
    fn resampling_with_uniform_weights_keeps_every_particle_once() {
        let map = corridor_map();
        let mut filter = ParticleFilter::new(config(), &map, Start::Global).unwrap();
        filter.particles = (0..100)
            .map(|i| Pose2::new(f64::from(i), 0.0, 0.0))
            .collect();
        filter.weights = vec![0.01; 100];
        filter.resample(100);
        let mut xs: Vec<f64> = filter.particles.iter().map(|p| p.x_m).collect();
        xs.sort_by(f64::total_cmp);
        assert_eq!(xs, (0..100).map(f64::from).collect::<Vec<_>>());
    }

    #[test]
    fn resampling_follows_the_weights() {
        let map = corridor_map();
        let mut filter = ParticleFilter::new(config(), &map, Start::Global).unwrap();
        filter.particles = vec![Pose2::new(0.0, 0.0, 0.0), Pose2::new(1.0, 0.0, 0.0)];
        filter.weights = vec![0.25, 0.75];
        filter.resample(1000);
        let heavy = filter.particles.iter().filter(|p| p.x_m == 1.0).count();
        assert_eq!(heavy, 750);
    }

    /// Drives along the corridor, scanning on the way, with odometry
    /// drifting off the truth; returns the final estimate and truth.
    fn drive(
        filter: &mut ParticleFilter,
        map: &SelectedMap,
        start: Pose2,
        steps: usize,
    ) -> (Estimate, Pose2) {
        let mut truth = start;
        let mut odometry = Pose2::default();
        let mut estimate = filter.process(odometry, &scan_from(map, truth));
        for _ in 0..steps {
            truth = truth.compose(&Pose2::new(0.1, 0.0, 0.0));
            // Odometry overestimates distance and invents a slight turn.
            odometry = odometry.compose(&Pose2::new(0.11, 0.0, 0.004));
            estimate = filter.process(odometry, &scan_from(map, truth));
        }
        (estimate, truth)
    }

    #[test]
    fn a_rough_start_converges_onto_the_truth() {
        let map = corridor_map();
        let truth = Pose2::new(1.5, 1.5, 0.0);
        // Started 0.4 m and 0.3 rad off.
        let anchor = Pose2::new(1.8, 1.25, 0.3);
        let mut filter = ParticleFilter::new(config(), &map, Start::Anchor(anchor)).unwrap();
        let (estimate, truth) = drive(&mut filter, &map, truth, 50);
        assert!(
            position_error(estimate.pose, truth) < 0.1,
            "estimate {:?}, truth {truth:?}",
            estimate.pose
        );
        assert!(heading_error(estimate.pose.heading_rad, truth.heading_rad).abs() < 0.05);
        // The odom frame's correction composes odometry back onto the
        // estimate.
        assert!(!filter.relocalizing());
    }

    #[test]
    fn a_global_start_finds_the_vehicle() {
        let map = corridor_map();
        let truth = Pose2::new(1.2, 1.5, 0.0);
        let mut filter = ParticleFilter::new(
            ParticleFilterConfig {
                global_num_particles: 20_000,
                ..config()
            },
            &map,
            Start::Global,
        )
        .unwrap();
        let (estimate, truth) = drive(&mut filter, &map, truth, 50);
        assert!(!filter.relocalizing(), "never converged: {estimate:?}");
        assert!(
            position_error(estimate.pose, truth) < 0.15,
            "estimate {:?}, truth {truth:?}",
            estimate.pose
        );
        assert_eq!(estimate.particles, config().num_particles);
    }

    #[test]
    fn ray_marching_localizes_too() {
        let map = corridor_map();
        let truth = Pose2::new(1.5, 1.5, 0.0);
        let mut filter = ParticleFilter::new(
            ParticleFilterConfig {
                ray_cast_algo: RAY_CAST_RAY_MARCHING,
                ..config()
            },
            &map,
            Start::Pose(Pose2::new(1.7, 1.4, 0.15)),
        )
        .unwrap();
        let (estimate, truth) = drive(&mut filter, &map, truth, 30);
        assert!(position_error(estimate.pose, truth) < 0.1, "{estimate:?}");
    }

    #[test]
    fn beams_cover_the_considered_field_of_view() {
        let map = corridor_map();
        let scan = scan_from(&map, Pose2::new(1.0, 1.5, 0.0));
        let beams = Beams::new(&scan, 220f64.to_radians(), 15);
        let (first, last) = (beams.angles[0], *beams.angles.last().unwrap());
        assert!((first + 110f32.to_radians()).abs() < 0.01, "{first}");
        assert!(last <= 110f32.to_radians() + 0.01 && last > 105f32.to_radians());
        assert_eq!(beams.indices[1] - beams.indices[0], 15);
        // Wider than the sensor: clamped to it.
        let all = Beams::new(&scan, 10.0, 1);
        assert_eq!(all.indices.len(), 1081);
    }
}
