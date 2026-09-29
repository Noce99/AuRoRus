//! [`UbmDetector`]: ubm's `detector_py` as an [`Executor`] - on every new
//! ego lidar scan, compares it with the scan the map alone would give from
//! the ego vehicle's pose and publishes the opponent that makes up the
//! difference on [`DETECTED_OPPONENT_TOPIC_NAME`], tuned live through
//! [`DETECTOR_PARAMETERS_TOPIC_NAME`].

use super::config::{UbmDetectorConfig, tunable_parameters};
use super::map_difference::{Kalman, PlateauLimits, find_plateau, fit_rectangle, near_wall};
use crate::autonomous_control::shared::race_line::{Pose, pose};
use crate::sensors::cast_ray;
use crate::topics::{
    AlgorithmParameter, Color, DETECTED_OPPONENT_TOPIC_NAME, DETECTOR_PARAMETERS_TOPIC_NAME,
    DETECTOR_STATUS_TOPIC_NAME, DetectedOpponent, DetectorParameters, DetectorStatus, Drawing,
    LidarScan, MAP_TOPIC_NAME, SelectedMap, Shape, VehicleTopics,
};
use crate::{Captain, Executor, Ticker};
use std::any::Any;
use std::time::{Duration, Instant};

/// Paint order of the detector's drawing: above the lidar's hits.
const DRAWING_Z_INDEX: i32 = 6;

/// Looks for one opponent in every new scan on the ego vehicle's lidar
/// topic, publishing it on [`DETECTED_OPPONENT_TOPIC_NAME`] and drawing it,
/// and reports on [`DETECTOR_STATUS_TOPIC_NAME`]. Applies whatever
/// [`DetectorParameters`] asks for before the next scan.
pub struct UbmDetector {
    id: u16,
    name: String,
    config: UbmDetectorConfig,
}

impl UbmDetector {
    pub fn new(name: impl Into<String>, config: UbmDetectorConfig) -> Self {
        Self {
            id: 0,
            name: name.into(),
            config,
        }
    }

    /// Applies any new [`DetectorParameters`] to the config. Returns
    /// whether the config changed.
    fn apply_parameters(&mut self, captain: &Captain, seen_write_count: &mut u64) -> bool {
        // Nothing may publish parameters at all (e.g. a binary without
        // `web_gui`) - then the config stays as loaded.
        let Some(requests) =
            captain.try_topic::<DetectorParameters>(DETECTOR_PARAMETERS_TOPIC_NAME)
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

/// One scan's worth of detection, before the Kalman filter.
struct Detection {
    /// Where the opponent's center was measured, in the map frame.
    measured: [f64; 2],
    /// The scan's points on it, in the map frame.
    points: Vec<[f64; 2]>,
}

/// `detect_opp` up to the Kalman filter: the opponent in `scan`, taken
/// from `pose` on `map`, if any. Also returns where the map alone would
/// have stopped each ray, for drawing.
fn detect(
    config: &UbmDetectorConfig,
    scan: &LidarScan,
    pose: Pose,
    map: &SelectedMap,
    info: &crate::environment::MapInfo,
) -> (Option<Detection>, Vec<[f32; 2]>) {
    // Never past what the lidar itself reports, or every miss would read as
    // an object in front of the map's farther walls.
    let max_range_m = config
        .max_detection_range_m
        .min(f64::from(scan.max_distance));
    let angle = |i: usize| pose.heading_rad + f64::from(scan.angle_rad(i));
    let mut expected_hits = Vec::new();
    let (expected, real): (Vec<f64>, Vec<f64>) = scan
        .points
        .iter()
        .enumerate()
        .map(|(i, &range)| {
            let (expected_m, hit) =
                cast_ray(map, info, pose.x_m, pose.y_m, angle(i), max_range_m as f32);
            let expected_m = f64::from(expected_m);
            if hit {
                expected_hits.push([
                    (pose.x_m + expected_m * angle(i).cos()) as f32,
                    (pose.y_m + expected_m * angle(i).sin()) as f32,
                ]);
            }
            let range = f64::from(range);
            if range >= max_range_m {
                (max_range_m, max_range_m)
            } else {
                (expected_m, range)
            }
        })
        .unzip();

    let limits = PlateauLimits {
        median_kernel_size: config.median_filter_kernel_size,
        gradient_threshold: config.gradient_threshold,
        min_length: config.min_object_size,
        max_std_m: config.object_std_threshold_m,
        min_mean_difference_m: config.distance_from_walls_threshold_m,
    };
    let Some(plateau) = find_plateau(&expected, &real, &limits) else {
        return (None, expected_hits);
    };
    let center = (plateau.start + plateau.end) / 2;
    let distance_m = plateau.median_range_m.abs() + config.robot_radius_m;
    let measured = [
        pose.x_m + distance_m * angle(center).cos(),
        pose.y_m + distance_m * angle(center).sin(),
    ];
    if config.ignore_walls != 0
        && near_wall(
            map,
            info,
            measured[0],
            measured[1],
            config.ignore_walls_radius_px,
        )
    {
        return (None, expected_hits);
    }
    let points = (plateau.start..plateau.end)
        .filter(|&i| real[i] < max_range_m)
        .map(|i| {
            [
                pose.x_m + real[i] * angle(i).cos(),
                pose.y_m + real[i] * angle(i).sin(),
            ]
        })
        .collect();
    (Some(Detection { measured, points }), expected_hits)
}

/// The two axes of `detector_py`'s Kalman filter.
#[derive(Debug, Clone, Copy, Default)]
struct Tracker {
    x: Kalman,
    y: Kalman,
}

impl Tracker {
    fn predict(&mut self, dt_s: f64, config: &UbmDetectorConfig) {
        self.x.predict(dt_s, config.kf_process_noise);
        self.y.predict(dt_s, config.kf_process_noise);
    }

    fn update(&mut self, measured: [f64; 2], config: &UbmDetectorConfig) {
        self.x.update(measured[0], config.kf_measurement_noise);
        self.y.update(measured[1], config.kf_measurement_noise);
    }

    fn position(&self) -> [f64; 2] {
        [self.x.position, self.y.position]
    }

    fn velocity(&self) -> [f64; 2] {
        [self.x.velocity, self.y.velocity]
    }

    /// `_make_predictions`: where a copy of the filter expects the
    /// opponent, now and every `prediction_dt_s` after.
    fn predictions(&self, config: &UbmDetectorConfig) -> Vec<[f64; 2]> {
        let mut copy = *self;
        (0..config.prediction_count)
            .map(|_| {
                let position = copy.position();
                copy.predict(config.prediction_dt_s, config);
                position
            })
            .collect()
    }
}

/// What the detector draws: the fitted box, the filtered opponent with its
/// velocity, its predicted positions, and (off by default) the scan the map
/// alone would give.
fn drawing(opponent: &DetectedOpponent, expected_hits: Vec<[f32; 2]>) -> Drawing {
    let found = opponent.detected;
    let [x_m, y_m] = opponent.position;
    let [vx, vy] = opponent.velocity;
    let as_f32 = |p: &[f64; 2]| [p[0] as f32, p[1] as f32];
    Drawing::default()
        .element(
            "Bounding box",
            opponent.bounding_box.map(|b| Shape::Rect {
                x_m: b.center[0],
                y_m: b.center[1],
                length_m: b.length_m,
                width_m: b.width_m,
                heading_rad: b.heading_rad,
                filled: false,
                color: Color::CYAN,
            }),
            true,
        )
        .element(
            "Opponent",
            found
                .then(|| {
                    [
                        Shape::Circle {
                            x_m,
                            y_m,
                            radius_m: 0.1,
                            filled: true,
                            color: Color::PINK,
                        },
                        // Where it'll be in a second.
                        Shape::Polyline {
                            points: vec![as_f32(&[x_m, y_m]), as_f32(&[x_m + vx, y_m + vy])],
                            closed: false,
                            width_px: 2.0,
                            color: Color::PINK,
                        },
                    ]
                })
                .into_iter()
                .flatten(),
            true,
        )
        .element(
            "Predictions",
            [Shape::Points {
                points: opponent.predictions.iter().map(as_f32).collect(),
                radius_px: 2.0,
                color: Color::PINK.with_alpha(160),
            }],
            false,
        )
        .element(
            "Expected scan",
            [Shape::Points {
                points: expected_hits,
                radius_px: 1.5,
                color: Color::WHITE.with_alpha(160),
            }],
            false,
        )
        .stale_after(Drawing::DEFAULT_STALE_AFTER)
        .z_index(DRAWING_Z_INDEX)
}

impl Executor for UbmDetector {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        let parameters = self.parameters();
        captain.claim_writer::<DetectorStatus>(DETECTOR_STATUS_TOPIC_NAME, self.id, move || {
            DetectorStatus {
                parameters: parameters.clone(),
                ..DetectorStatus::default()
            }
        });
        captain.claim_writer::<DetectedOpponent>(
            DETECTED_OPPONENT_TOPIC_NAME,
            self.id,
            DetectedOpponent::default,
        );
        captain.claim_drawing(self.id);
    }

    fn run(&mut self, captain: &Captain) {
        let status_topic = captain.topic::<DetectorStatus>(DETECTOR_STATUS_TOPIC_NAME);
        let opponent_topic = captain.topic::<DetectedOpponent>(DETECTED_OPPONENT_TOPIC_NAME);
        let map_topic = captain.topic::<SelectedMap>(MAP_TOPIC_NAME);
        let drawing_topic = captain.drawing(self.id);
        let ego = VehicleTopics::ego();
        let mut ticker = Ticker::from_interval(Duration::from_millis(self.config.poll_interval_ms));
        let mut status = DetectorStatus {
            parameters: self.parameters(),
            message: Some("Waiting for a lidar scan.".to_string()),
            scan_ms: 0.0,
        };
        let id = self.id;
        let publish_status = |status: &DetectorStatus| {
            status_topic
                .write(id, status.clone())
                .expect("lost writer authorization for the detector_status topic");
        };
        publish_status(&status);

        let mut seen_parameters = 0;
        let mut seen_scan = 0;
        let mut last_scan_at: Option<Instant> = None;
        let mut tracker = Tracker::default();

        while captain.is_running(self.id) {
            if self.apply_parameters(captain, &mut seen_parameters) {
                status.parameters = self.parameters();
                publish_status(&status);
            }

            // The lidar may only exist once a vehicle is simulated.
            let Some(scan_topic) = captain.try_topic::<LidarScan>(&ego.lidar_scan()) else {
                ticker.wait();
                continue;
            };
            let scan = scan_topic.read();
            if scan.meta.write_count == seen_scan {
                ticker.wait();
                continue;
            }
            seen_scan = scan.meta.write_count;
            let started = Instant::now();

            let map = map_topic.read().into_value();
            let inputs = match (
                pose(captain, &ego, self.config.pose_source),
                map.info.as_ref(),
            ) {
                (Err(message), _) => Err(message),
                (_, None) => Err("No map is loaded.".to_string()),
                (Ok(pose), Some(info)) => Ok((pose, info)),
            };
            let (pose, info) = match inputs {
                Ok(inputs) => inputs,
                Err(message) => {
                    if status.message.as_ref() != Some(&message) {
                        status.message = Some(message);
                        publish_status(&status);
                        opponent_topic
                            .write(self.id, DetectedOpponent::default())
                            .expect("lost writer authorization for the detected_opponent topic");
                        drawing_topic
                            .write(self.id, Drawing::default().z_index(DRAWING_Z_INDEX))
                            .expect("lost writer authorization for the detector's drawing topic");
                    }
                    ticker.wait();
                    continue;
                }
            };

            let (detection, expected_hits) = detect(&self.config, &scan.value, pose, &map, info);
            // Timed by the scans themselves, so a slow loop doesn't skew
            // the filter's model.
            let scan_at = scan.meta.written_at.unwrap_or(started);
            let dt_s = last_scan_at.map_or(0.0, |last| scan_at.duration_since(last).as_secs_f64());
            last_scan_at = Some(scan_at);
            tracker.predict(dt_s, &self.config);
            let opponent = match detection {
                Some(detection) => {
                    tracker.update(detection.measured, &self.config);
                    DetectedOpponent {
                        detected: true,
                        position: tracker.position(),
                        velocity: tracker.velocity(),
                        bounding_box: fit_rectangle(
                            &detection.points,
                            self.config.min_2_points_dist_m,
                        ),
                        predictions: tracker.predictions(&self.config),
                    }
                }
                None => DetectedOpponent {
                    detected: false,
                    position: tracker.position(),
                    velocity: tracker.velocity(),
                    bounding_box: None,
                    predictions: Vec::new(),
                },
            };

            drawing_topic
                .write(self.id, drawing(&opponent, expected_hits))
                .expect("lost writer authorization for the detector's drawing topic");
            opponent_topic
                .write(self.id, opponent)
                .expect("lost writer authorization for the detected_opponent topic");
            status.message = None;
            status.scan_ms = started.elapsed().as_secs_f64() * 1000.0;
            publish_status(&status);

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
        Box::new(UbmDetector::new(self.name.clone(), self.config))
    }
}
