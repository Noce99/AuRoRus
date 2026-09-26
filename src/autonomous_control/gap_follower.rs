//! This file implement the Gap Follower algorithm as presented in the
//! f1tenth documentation: https://f1tenth-coursekit.readthedocs.io/en/latest/lectures/ModuleB/lecture05.html

use crate::autonomous_control::{Instance, ParameterTuner, load_config};
use crate::topics::{AlgorithmParameter, AutonomousAlgorithmInfo, Drawing, Shape, Color, VescCommand};
// use crate::topics::{VEHICLE_LIMITS_TOPIC_NAME, ActuatorLimits};
use crate::topics::LidarScan;
use crate::topics::VehicleStatus;
use crate::{Captain, Executor, Ticker};
use std::any::Any;
use std::time::Duration;

/// Entry point build.rs calls - required, with exactly this signature.
pub fn new(instance: Instance) -> Box<dyn Executor> {
    Box::new(GapFollower {
        id: 0,
        config: load_config(&instance.config_name),
        instance,
    })
}

/// Every tunable parameter [`GapFollower`] needs - loaded from
/// `config/autonomous_control/gap_follower.toml` at runtime (see [`load_config`]), falling back
/// to the copy compiled in (see [`Default`]). Every field can also be tuned live - see
/// [`parameters`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct GapFollowerConfig {
    /// Rate at which [`GapFollower`] publishes a new control, in Hz.
    pub rate_hz: f32,
    /// Threshold for identifying a far away lidar point, in meters.
    pub t_m: f32,
    /// Minimum number of far away point to create a gap, pure number,
    pub n: usize,
    /// Bubble radius, pure number.
    pub b_radius: usize,
    /// Speed
    pub speed: f32,
}

impl Default for GapFollowerConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/autonomous_control/gap_follower.toml"))
            .expect("config/autonomous_control/gap_follower.toml must deserialize into GapFollowerConfig")
    }
}

/// The live-tunable parameters, one per [`GapFollowerConfig`] field - see
/// [`ParameterTuner`].
fn parameters() -> [AlgorithmParameter; 5] {
    [
        // At least a few Hz: below 1 Hz every command would be stale on arrival
        // (see `VESC_COMMAND_TIMEOUT`), holding the vehicle stopped.
        AlgorithmParameter::float("rate_hz", 5.0, 200.0, 1.0)
            .unit("Hz")
            .description("Rate at which a new control is published."),
        AlgorithmParameter::float("t_m", 0.5, 12.0, 0.1)
            .unit("m")
            .description("Threshold for identifying a far away lidar point."),
        AlgorithmParameter::int("n", 0, 100, 1)
            .unit("points")
            .description("Minimum number of far away points to create a gap."),
        AlgorithmParameter::int("b_radius", 0, 180, 1)
            .unit("points")
            .description("Bubble radius around the closest point, where no gap can start."),
        AlgorithmParameter::float("speed", 0.6, 10.0, 0.2)
            .unit("m/s")
            .description("Speed, in m/s."),
    ]
}

struct GapFollower {
    id: u16,
    instance: Instance,
    config: GapFollowerConfig,
}

impl Executor for GapFollower {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_autonomous_control(
            self.id,
            &self.instance.algorithm_topics(),
            AutonomousAlgorithmInfo::new("Gap follower", "The simplest reactive algorithm")
                .requires_lidar()
                .with_parameters(&self.config, parameters()),
        );
        captain.claim_drawing(self.id);
    }

    fn run(&mut self, captain: &Captain) {
        let command_topic = captain.autonomous_control(&self.instance.algorithm_topics());
        // let limits_topic = captain.topic::<ActuatorLimits>(&self.instance.vehicle.vehicle_limits());
        let scan_topic = captain.topic::<LidarScan>(&self.instance.vehicle.lidar_scan());
        let vehicle_topic = captain.topic::<VehicleStatus>(&self.instance.vehicle.vehicle_status());
        let drawing_topic = captain.drawing(self.id);
        let mut tuner = ParameterTuner::new(self.id, &self.instance);

        // Both derive from `rate_hz`, so are rebuilt whenever it's tuned.
        let mut ticker = Ticker::new(self.config.rate_hz as f64);
        let mut stale_after = drawing_stale_after(&self.config);

        let mut steering:f32 = 0.0;

        while captain.is_running(self.id) {
            if tuner.update(captain, &mut self.config) {
                ticker = Ticker::new(self.config.rate_hz as f64);
                stale_after = drawing_stale_after(&self.config);
            }

            // Optional, only for computationally heavy algorithms
            // if !captain.is_selected_algorithm(&self.name) { ticker.wait(); continue; }

            // let limits = limits_topic.read();


            let vehicle_status = vehicle_topic.read();

            let scan = scan_topic.read();

            if scan.num_lidar_points <= 1 {
                ticker.wait();
                continue;
            }

            let points = &scan.points;
            let fov = scan.fov;
            let rad_per_point = fov / (scan.num_lidar_points-1) as f32;

            let mut gaps: Vec<Gap> = Vec::new();
            let mut last_gap_start: Option<usize> = None;
            let mut last_gap_length: Option<usize> = None;
            let mut last_gap_mean_d: Option<f32> = None;
            let mut current_best_gap: Option<usize> = None;
            let mut current_best_gap_length: Option<usize> = None;

            let vehice_x = vehicle_status.x_m as f32;
            let vehice_y = vehicle_status.y_m as f32;
            let vehicle_heading = vehicle_status.heading_rad as f32;

            let mut obstacle_shapes: Vec<Shape> = Vec::new();
            let mut gap_shapes: Vec<Shape> = Vec::new();

            let mut add_gap = |i: usize, length: usize, mean: f32| {
                let start_angle: f32 = -fov/2. + (i-length) as f32*rad_per_point;
                let end_angle: f32 = -fov/2. + (i-1) as f32*rad_per_point;
                let direction: f32 = (start_angle + end_angle) / 2.;
                let a_gap: Gap = Gap{
                    mean_distance: mean,
                    // size: length,
                    direction: direction,
                    start_angle: start_angle + vehicle_heading,
                    end_angle: end_angle + vehicle_heading,
                };
                gaps.push(a_gap);
                match current_best_gap_length{
                    None => {
                        current_best_gap = Some(gaps.len()-1);
                        current_best_gap_length = Some(length);
                    },
                    Some(best_length) => {
                        if length > best_length {
                            current_best_gap = Some(gaps.len()-1);
                            current_best_gap_length = Some(length);
                        }
                    }
                }
            };

            let mut closer_i: usize = 0;
            let mut closer_distance: f32 = points[0];

            for i in 1..points.len(){
                if points[i] < closer_distance{
                    closer_i  = i;
                    closer_distance = points[i];
                }
            }

            obstacle_shapes.push(
                Shape::CircularSector {
                    x_m: vehice_x as f64,
                    y_m: vehice_y as f64,
                    radius_m: closer_distance as f64,
                    start_rad: (-fov/2. + closer_i.saturating_sub(self.config.b_radius) as f32*rad_per_point + vehicle_heading) as f64,
                    end_rad: (-fov/2. + (closer_i+self.config.b_radius).min(points.len()-1) as f32*rad_per_point + vehicle_heading) as f64,
                    filled: false,
                    color: Color::RED,
                }
            );

            for i in 0..points.len(){
                if points[i] >= self.config.t_m && i.abs_diff(closer_i) > self.config.b_radius{
                    match last_gap_start{
                        None => {
                            last_gap_start = Some(i);
                            last_gap_mean_d = Some(points[i]);
                            last_gap_length = Some(1);
                        },
                        Some(_) => {
                            if let Some(length) = last_gap_length 
                            && let Some(mean) = last_gap_mean_d{
                                last_gap_length = Some(length+1);
                                last_gap_mean_d = Some(mean + (points[i] - mean)/(length+1) as f32);
                            }
                        }
                    }
                }else{
                    if let Some(length) = last_gap_length
                    && let Some(mean) = last_gap_mean_d{
                        if length > self.config.n{
                            add_gap(i, length, mean);
                        }
                        last_gap_start = None;
                        last_gap_mean_d = None;
                        last_gap_length = None;
                    }
                }
            }
            if let Some(length) = last_gap_length
                && let Some(mean) = last_gap_mean_d
                && length > self.config.n
            {
                add_gap(points.len(), length, mean);
            }

            for i_gap in 0..gaps.len(){
                let color;
                if let Some(best_gap) = current_best_gap && i_gap == best_gap{
                    color = Color::PURPLE;
                    steering = gaps[i_gap].direction;
                }else{
                    color = Color::GREEN;
                }
                gap_shapes.push(
                    Shape::CircularSector {
                        x_m: vehice_x as f64,
                        y_m: vehice_y as f64,
                        radius_m: gaps[i_gap].mean_distance as f64,
                        start_rad: gaps[i_gap].start_angle as f64,
                        end_rad: gaps[i_gap].end_angle as f64,
                        filled: false,
                        color: color,
                    }
                );
            }

            let command = VescCommand::new(
                steering as f64 /* steering, rad */,
                self.config.speed as f64/* speed, m/s */
            );
            command_topic
                .write(self.id, command)
                .expect("lost writer authorization for this algorithm's command topic");
            drawing_topic.write(
                self.id,
                Drawing::default()
                    .element("Closest obstacle", obstacle_shapes, false)
                    .element("Gaps", gap_shapes, false)
                    .stale_after(stale_after),
            ).expect("lost writer authorization for the gap follower drawing topic");
            ticker.wait();
        }
    }

    fn name(&self) -> String {
        self.instance.name.clone()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn fresh(&self) -> Box<dyn Executor> {
        new(self.instance.clone())
    }
}

/// How long the drawing stays valid: a few publishing periods, but never
/// less than the default.
fn drawing_stale_after(config: &GapFollowerConfig) -> Duration {
    Drawing::DEFAULT_STALE_AFTER.max(Duration::from_secs_f64(3.0 / config.rate_hz as f64))
}

struct Gap{
    // Mean distance of the lidar points that created this gap, in meters.
    mean_distance: f32,
    // Size of the gap in number of lidar points, pure number.
    // size: usize,
    // Direction of the gap respect to the vehicle, in rad.
    direction: f32,
    // Start angle, in rad.
    start_angle: f32,
    // End angle, in rad.
    end_angle: f32
}