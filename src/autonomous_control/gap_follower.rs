//! This file implement the Gap Follower algorithm as presented in the
//! f1tenth documentation: https://f1tenth-coursekit.readthedocs.io/en/latest/lectures/ModuleB/lecture05.html

use crate::topics::{AutonomousAlgorithmInfo, Drawing, Shape, Color, VescCommand};
// use crate::topics::{VEHICLE_LIMITS_TOPIC_NAME, ActuatorLimits};
use crate::topics::{LIDAR_SCAN_TOPIC_NAME, LidarScan};
use crate::topics::{VEHICLE_STATUS_TOPIC_NAME, VehicleStatus};
use crate::{Captain, Executor, Ticker};
use std::any::Any;
use std::time::Duration;

/// Entry point build.rs calls - required, with exactly this signature.
pub fn new(name: &str) -> Box<dyn Executor> {
    Box::new(GapFollower {
        id: 0,
        name: name.to_string(),
        config: GapFollowerConfig::default(),
    })
}

/// Every tunable parameter [`GapFollower`] needs - loaded from
/// `config/autonomous_control/gap_follower.toml` (see [`Default`]) or from an arbitrary
/// path via [`crate::config::load`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Deserialize)]
pub struct GapFollowerConfig {
    /// Rate at which [`GapFollower`] publishes a new control, in Hz.
    pub rate_hz: f32,
    /// Threshold for identifying a far away lidar point, in meters.
    pub t_m: f32,
    /// Minimum number of far away point to create a gap, pure number,
    pub n: usize,
    /// Bubble radius, pure number.
    pub b_radius: usize,
}

impl Default for GapFollowerConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/autonomous_control/gap_follower.toml"))
            .expect("config/autonomous_control/gap_follower.toml must deserialize into GapFollowerConfig")
    }
}

struct GapFollower {
    id: u8,
    name: String,
    config: GapFollowerConfig,
}

impl Executor for GapFollower {
    fn init(&mut self, id: u8) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_autonomous_control(
            self.id,
            AutonomousAlgorithmInfo::new("Gap follower", "The simplest reactive algorithm"),
        );
        captain.claim_drawing(self.id);
    }

    fn run(&mut self, captain: &Captain) {
        let command_topic = captain.autonomous_control(self.id);
        // let limits_topic = captain.topic::<ActuatorLimits>(VEHICLE_LIMITS_TOPIC_NAME);
        let scan_topic = captain.topic::<LidarScan>(LIDAR_SCAN_TOPIC_NAME);
        let vehicle_topic = captain.topic::<VehicleStatus>(VEHICLE_STATUS_TOPIC_NAME);
        let drawing_topic = captain.drawing(self.id);

        let mut ticker = Ticker::new(self.config.rate_hz as f64);

        let stale_after = Drawing::DEFAULT_STALE_AFTER.max(Duration::from_secs_f64(3.0 / self.config.rate_hz as f64));

        let mut steering:f32 = 0.0;

        while captain.is_running(self.id) {
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

            let mut shapes: Vec<Shape> = Vec::new();

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

            shapes.push(
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
                shapes.push(
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
                1.0 /* speed, m/s */
            );
            command_topic
                .write(self.id, command)
                .expect("lost writer authorization for this algorithm's command topic");
            drawing_topic.write(
                self.id,
                Drawing::new(shapes).stale_after(stale_after),
            ).expect("lost writer authorization for the gap follower drawing topic");
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
        new(&self.name)
    }
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