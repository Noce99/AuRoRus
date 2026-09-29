//! [`LapTelemetryRecorder`]: follows the ego vehicle along the race line it
//! drives - its Frenet `s` (meters along the line from the start/finish
//! line) and signed lateral offset `d` - to publish its lateral and speed
//! errors along each lap, and every lap's time, on
//! [`LAP_TELEMETRY_TOPIC_NAME`].
//!
//! A lap is timed from one forward crossing of the start/finish line (`s`
//! wrapping from the line's length back to `0`) to the next, interpolated
//! between the two samples either side of it. A lap only counts once the
//! vehicle has progressed most of the line's length along it (see
//! [`LapTelemetryConfig::min_lap_fraction`]), so reversing over the line
//! and back doesn't complete one. After the vehicle is placed (or jumps) away
//! from the line, the lap up to the line is an untimed out lap; placed on it,
//! the lap is timed from when it starts moving.

use crate::autonomous_control::shared::race_line::{Line, POSE_TIMEOUT, Pose, localization_pose};
use crate::topics::{
    LAP_TELEMETRY_TOPIC_NAME, LapRecord, LapTelemetry, LapTrace, Odometry, Placement,
    PlacementTopics, RACE_LINE_TOPIC_NAME, SelectedRaceLine, VehicleStatus, VehicleTopics,
};
use crate::{Captain, Executor, Ticker};
use std::any::Any;
use std::time::{Duration, Instant};

/// Every tunable parameter [`LapTelemetryRecorder`] needs - loaded from
/// `config/telemetry/lap_telemetry.toml` (see [`Default`]) or from an
/// arbitrary path via [`crate::config::load`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Deserialize)]
pub struct LapTelemetryConfig {
    /// How often the vehicle's pose is sampled, in Hz.
    pub sample_rate_hz: f64,
    /// How often [`LAP_TELEMETRY_TOPIC_NAME`] is published, in Hz - a
    /// completed lap is published right away.
    pub publish_rate_hz: f64,
    pub pose_source: TelemetryPoseSource,
    /// How many bins one lap's errors are stored in, along the line.
    pub bins: usize,
    /// How far along the line, in meters, the vehicle's projection is
    /// searched from the previous sample's.
    pub search_window_m: f64,
    /// A pose farther than this, in meters, from the previous sample is a
    /// teleport: the lap in progress is dropped.
    pub max_jump_m: f64,
    /// Placed within this distance, in meters, past the start/finish line,
    /// the vehicle starts a timed lap as soon as it moves.
    pub start_tolerance_m: f64,
    /// Fraction of the line's length the vehicle must have progressed along
    /// it for a crossing of the start/finish line to complete a lap.
    pub min_lap_fraction: f64,
}

impl Default for LapTelemetryConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/telemetry/lap_telemetry.toml"))
            .expect("config/telemetry/lap_telemetry.toml must deserialize into LapTelemetryConfig")
    }
}

/// Where [`LapTelemetryRecorder`] gets the vehicle's pose and speed from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TelemetryPoseSource {
    /// The simulator's `vehicle_status`.
    GroundTruth,
    /// SLAM's pose, and odometry's speed.
    Localization,
}

/// One pose sample: when it was taken, in seconds on the recorder's own
/// clock, where, and how fast the vehicle was going.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Sample {
    t_s: f64,
    x_m: f64,
    y_m: f64,
    speed_mps: f64,
}

/// A [`Sample`] placed on the line.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Point {
    t_s: f64,
    x_m: f64,
    y_m: f64,
    s_m: f64,
    /// The line's segment `s_m` is on - the next projection's hint.
    segment: usize,
    lateral_m: f64,
    speed_error_mps: f64,
}

/// Whether the lap in progress is being timed.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Timing {
    /// Not until the vehicle next crosses the start/finish line.
    Out,
    /// Placed on the line: from when the vehicle starts moving forward.
    Armed,
    /// Since `start_s`.
    Timed { start_s: f64 },
}

/// The laps on one race line: everything [`LapTelemetry`] holds, kept up to
/// date one [`Sample`] at a time.
struct LapTracker {
    line: Line,
    race_line: Option<String>,
    config: LapTelemetryConfig,
    /// The previous sample, or `None` right after a placement, when the next
    /// one starts afresh.
    last: Option<Point>,
    timing: Timing,
    /// Net meters of `s` progressed on the lap in progress - backward
    /// motion counts negative.
    progress_m: f64,
    /// Meters actually driven on the lap in progress.
    distance_m: f64,
    current: LapTrace,
    previous: Option<LapTrace>,
    laps: Vec<LapRecord>,
}

impl LapTracker {
    fn new(line: Line, race_line: Option<String>, config: LapTelemetryConfig) -> Self {
        let bins = config.bins.max(1);
        Self {
            line,
            race_line,
            config: LapTelemetryConfig { bins, ..config },
            last: None,
            timing: Timing::Out,
            progress_m: 0.0,
            distance_m: 0.0,
            current: LapTrace::empty(0, bins),
            previous: None,
            laps: Vec::new(),
        }
    }

    /// The vehicle was placed somewhere: the next sample starts afresh.
    fn place(&mut self) {
        self.last = None;
    }

    fn update(&mut self, sample: Sample) {
        // A teleport starts afresh - before projecting, so the projection
        // isn't searched for around where the vehicle was.
        if let Some(last) = self.last
            && (sample.x_m - last.x_m).hypot(sample.y_m - last.y_m) > self.config.max_jump_m
        {
            self.last = None;
        }
        let hint = self.last.map(|last| last.segment);
        let nearest = self
            .line
            .nearest(sample.x_m, sample.y_m, hint, self.config.search_window_m);
        let pose = Pose {
            x_m: sample.x_m,
            y_m: sample.y_m,
            heading_rad: 0.0,
        };
        let (lateral_m, _) = self.line.lateral(pose, &nearest);
        let point = Point {
            t_s: sample.t_s,
            x_m: sample.x_m,
            y_m: sample.y_m,
            s_m: nearest.s_m,
            segment: nearest.segment,
            lateral_m,
            speed_error_mps: sample.speed_mps - self.line.at(nearest.s_m).speed_mps,
        };

        let Some(last) = self.last else {
            return self.start_afresh(point);
        };
        let step_m = (point.x_m - last.x_m).hypot(point.y_m - last.y_m);
        let lap_m = self.line.lap_m();
        let ds_m = wrapped_half(point.s_m - last.s_m, lap_m);

        if ds_m > 0.0 && point.s_m < last.s_m {
            // Forward over the start/finish line, `before_m` of `ds_m` short
            // of it.
            let before_m = lap_m - last.s_m;
            let fraction = (before_m / ds_m).clamp(0.0, 1.0);
            let on_line = interpolate(&last, &point, fraction);
            self.fill(&last, last.s_m, &on_line, lap_m);
            self.progress_m += before_m;
            self.distance_m += fraction * step_m;
            if self.cross_line(on_line.t_s) {
                self.progress_m = 0.0;
                self.distance_m = 0.0;
            }
            self.fill(&on_line, 0.0, &point, point.s_m);
            self.progress_m += point.s_m;
            self.distance_m += (1.0 - fraction) * step_m;
        } else {
            if ds_m > 0.0 {
                if self.timing == Timing::Armed {
                    self.timing = Timing::Timed { start_s: last.t_s };
                    self.current.number = self.next_lap_number();
                }
                self.fill(&last, last.s_m, &point, point.s_m);
            } else {
                self.set_bin(&point);
            }
            self.progress_m += ds_m;
            self.distance_m += step_m;
        }
        self.last = Some(point);
    }

    /// Starts over from `point`, after a placement or a jump: a standing
    /// start if it's just past the start/finish line, else an out lap. The
    /// laps completed so far are kept.
    fn start_afresh(&mut self, point: Point) {
        self.timing = if point.s_m <= self.config.start_tolerance_m {
            Timing::Armed
        } else {
            Timing::Out
        };
        self.progress_m = 0.0;
        self.distance_m = 0.0;
        self.current = LapTrace::empty(0, self.config.bins);
        self.set_bin(&point);
        self.last = Some(point);
    }

    /// The vehicle crossed the start/finish line forward at `t_s`: completes
    /// the lap in progress if it went (nearly) all the way round, or starts
    /// timing one. Whether a new lap started.
    fn cross_line(&mut self, t_s: f64) -> bool {
        let number = self.next_lap_number();
        match self.timing {
            Timing::Timed { start_s } => {
                if self.progress_m < self.config.min_lap_fraction * self.line.lap_m() {
                    return false;
                }
                let time_s = t_s - start_s;
                self.laps.push(LapRecord {
                    number,
                    time_s,
                    distance_m: self.distance_m,
                    average_speed_mps: if time_s > 0.0 {
                        self.distance_m / time_s
                    } else {
                        0.0
                    },
                });
                let next = LapTrace::empty(number + 1, self.config.bins);
                self.previous = Some(std::mem::replace(&mut self.current, next));
            }
            Timing::Out | Timing::Armed => {
                self.current = LapTrace::empty(number, self.config.bins);
            }
        }
        self.timing = Timing::Timed { start_s: t_s };
        true
    }

    fn next_lap_number(&self) -> u32 {
        self.laps.len() as u32 + 1
    }

    fn bin(&self, s_m: f64) -> usize {
        let bins = self.config.bins;
        ((s_m / self.line.lap_m() * bins as f64).floor().max(0.0) as usize).min(bins - 1)
    }

    fn set_bin(&mut self, point: &Point) {
        let bin = self.bin(point.s_m);
        self.current.lateral_m[bin] = Some(point.lateral_m as f32);
        self.current.speed_error_mps[bin] = Some(point.speed_error_mps as f32);
    }

    /// Fills every bin from `from_s_m` to `to_s_m` (`from_s_m <= to_s_m`,
    /// no wrap) with the errors interpolated between `from` and `to` at the
    /// bin's middle - so a vehicle fast enough to skip bins between two
    /// samples leaves no gaps.
    fn fill(&mut self, from: &Point, from_s_m: f64, to: &Point, to_s_m: f64) {
        let width_m = self.line.lap_m() / self.config.bins as f64;
        let span_m = to_s_m - from_s_m;
        for bin in self.bin(from_s_m)..=self.bin(to_s_m) {
            let s_m = ((bin as f64 + 0.5) * width_m).clamp(from_s_m, to_s_m);
            let fraction = if span_m > 0.0 {
                (s_m - from_s_m) / span_m
            } else {
                1.0
            };
            let value = interpolate(from, to, fraction);
            self.current.lateral_m[bin] = Some(value.lateral_m as f32);
            self.current.speed_error_mps[bin] = Some(value.speed_error_mps as f32);
        }
    }

    fn telemetry(&self, status: Option<String>) -> LapTelemetry {
        LapTelemetry {
            race_line: self.race_line.clone(),
            lap_length_m: self.line.lap_m(),
            s_m: self.last.map(|last| last.s_m),
            status,
            current_lap_time_s: match (self.timing, self.last) {
                (Timing::Timed { start_s }, Some(last)) => Some(last.t_s - start_s),
                _ => None,
            },
            current: self.current.clone(),
            previous: self.previous.clone(),
            laps: self.laps.clone(),
        }
    }
}

/// `value` wrapped to `(-period / 2, period / 2]`.
fn wrapped_half(value: f64, period: f64) -> f64 {
    let wrapped = value.rem_euclid(period);
    if wrapped > period / 2.0 {
        wrapped - period
    } else {
        wrapped
    }
}

/// The point `fraction` of the way from `a` to `b` - time and errors
/// included; `s` and the segment are `b`'s.
fn interpolate(a: &Point, b: &Point, fraction: f64) -> Point {
    let lerp = |a: f64, b: f64| a + fraction * (b - a);
    Point {
        t_s: lerp(a.t_s, b.t_s),
        x_m: lerp(a.x_m, b.x_m),
        y_m: lerp(a.y_m, b.y_m),
        lateral_m: lerp(a.lateral_m, b.lateral_m),
        speed_error_mps: lerp(a.speed_error_mps, b.speed_error_mps),
        ..*b
    }
}

/// Publishes the ego vehicle's [`LapTelemetry`] on
/// [`LAP_TELEMETRY_TOPIC_NAME`].
pub struct LapTelemetryRecorder {
    id: u16,
    name: String,
    config: LapTelemetryConfig,
}

impl LapTelemetryRecorder {
    pub fn new(name: impl Into<String>, config: LapTelemetryConfig) -> Self {
        Self {
            id: 0,
            name: name.into(),
            config,
        }
    }
}

/// `at` in seconds since `origin` - negative if before it.
fn seconds_since(origin: Instant, at: Instant) -> f64 {
    match at.checked_duration_since(origin) {
        Some(after) => after.as_secs_f64(),
        None => -origin.duration_since(at).as_secs_f64(),
    }
}

/// The latest pose sample from `source`, with the write count of the topic
/// it came from (to tell a new sample from one already used), or why
/// there's none.
fn sample(
    captain: &Captain,
    source: TelemetryPoseSource,
    origin: Instant,
) -> Result<(u64, Sample), String> {
    let vehicle = VehicleTopics::ego();
    let stamped_sample = |meta: crate::WriteMeta, x_m, y_m, speed_mps| {
        let at = meta.written_at.unwrap_or(origin);
        (
            meta.write_count,
            Sample {
                t_s: seconds_since(origin, at),
                x_m,
                y_m,
                speed_mps,
            },
        )
    };
    match source {
        TelemetryPoseSource::GroundTruth => {
            let status = captain
                .try_topic::<VehicleStatus>(&vehicle.vehicle_status())
                .ok_or("No ground truth pose (vehicle_status) in this binary.")?
                .read();
            if status.age().is_none_or(|age| age > POSE_TIMEOUT) {
                return Err("The ground truth pose (vehicle_status) is stale.".into());
            }
            Ok(stamped_sample(
                status.meta,
                status.x_m,
                status.y_m,
                status.speed_mps,
            ))
        }
        TelemetryPoseSource::Localization => {
            let pose = localization_pose(captain, &vehicle)?;
            let odometry = captain
                .try_topic::<Odometry>(&vehicle.odometry())
                .ok_or("No odometry in this binary.")?
                .read();
            Ok(stamped_sample(
                odometry.meta,
                pose.x_m,
                pose.y_m,
                odometry.speed_mps,
            ))
        }
    }
}

impl Executor for LapTelemetryRecorder {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_writer::<LapTelemetry>(
            LAP_TELEMETRY_TOPIC_NAME,
            self.id,
            LapTelemetry::default,
        );
    }

    fn run(&mut self, captain: &Captain) {
        let topic = captain.topic::<LapTelemetry>(LAP_TELEMETRY_TOPIC_NAME);
        let publish_interval =
            Duration::from_secs_f64(1.0 / self.config.publish_rate_hz.max(f64::MIN_POSITIVE));
        let origin = Instant::now();
        let mut ticker = Ticker::new(self.config.sample_rate_hz);

        let mut tracker: Option<LapTracker> = None;
        let mut race_line_writes = None;
        // The ego vehicle's placements, if this binary has the topics.
        let placement_topics = PlacementTopics::try_new(captain);
        let mut placements = placement_topics
            .as_ref()
            .map(|topics| Placement::new(&topics.read()));
        let mut sample_writes = None;
        let mut last_published: Option<Instant> = None;

        while captain.is_running(self.id) {
            ticker.wait();

            // A new race line (or map) starts everything over.
            let race_line = captain.try_topic::<SelectedRaceLine>(RACE_LINE_TOPIC_NAME);
            let writes = race_line.as_ref().map(|topic| topic.meta().write_count);
            if writes != race_line_writes {
                race_line_writes = writes;
                let line = race_line.map(|topic| topic.read().into_value());
                tracker = line.and_then(|line| {
                    Line::new(line.points)
                        .map(|points| LapTracker::new(points, line.file, self.config))
                });
                sample_writes = None;
                last_published = None;
            }

            if let (Some(placements), Some(topics)) = (&mut placements, &placement_topics)
                && placements
                    .update(&topics.read(), &VehicleTopics::ego())
                    .is_some()
                && let Some(tracker) = &mut tracker
            {
                tracker.place();
            }

            let mut lap_completed = false;
            let status = match (
                &mut tracker,
                sample(captain, self.config.pose_source, origin),
            ) {
                (None, _) => Some("No race line on the selected map.".to_string()),
                (Some(_), Err(why)) => Some(why),
                (Some(tracker), Ok((writes, sample))) => {
                    if sample_writes != Some(writes) {
                        sample_writes = Some(writes);
                        let laps = tracker.laps.len();
                        tracker.update(sample);
                        lap_completed = tracker.laps.len() != laps;
                    }
                    None
                }
            };

            let due = last_published.is_none_or(|at| at.elapsed() >= publish_interval);
            if due || lap_completed {
                last_published = Some(Instant::now());
                let telemetry = match &tracker {
                    Some(tracker) => tracker.telemetry(status),
                    None => LapTelemetry {
                        status,
                        ..LapTelemetry::default()
                    },
                };
                topic
                    .write(self.id, telemetry)
                    .expect("lost writer authorization for the lap_telemetry topic");
            }
        }
    }

    fn name(&self) -> String {
        self.name.clone()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn fresh(&self) -> Box<dyn Executor> {
        Box::new(Self::new(self.name.clone(), self.config))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::environment::SpeedPoint;
    use std::f64::consts::PI;

    const RADIUS_M: f64 = 5.0;
    const LINE_SPEED_MPS: f64 = 2.0;

    /// A counterclockwise circle of radius [`RADIUS_M`] around the origin,
    /// starting at `(RADIUS_M, 0)`.
    fn circle() -> Line {
        let n = 400;
        Line::new(
            (0..n)
                .map(|i| {
                    let angle = 2.0 * PI * i as f64 / n as f64;
                    SpeedPoint {
                        x: RADIUS_M * angle.cos(),
                        y: RADIUS_M * angle.sin(),
                        speed_mps: LINE_SPEED_MPS,
                    }
                })
                .collect(),
        )
        .unwrap()
    }

    fn tracker() -> LapTracker {
        LapTracker::new(
            circle(),
            Some("line.csv".into()),
            LapTelemetryConfig {
                bins: 100,
                ..LapTelemetryConfig::default()
            },
        )
    }

    /// Where a vehicle at `angle_rad` round the circle, `offset_m` outside
    /// it (to the line's right), is at `t_s`.
    fn at(t_s: f64, angle_rad: f64, offset_m: f64, speed_mps: f64) -> Sample {
        let radius_m = RADIUS_M + offset_m;
        Sample {
            t_s,
            x_m: radius_m * angle_rad.cos(),
            y_m: radius_m * angle_rad.sin(),
            speed_mps,
        }
    }

    /// Drives `laps` laps counterclockwise from `start_rad`, one sample
    /// every `step_rad`, at one radian per second.
    fn drive(tracker: &mut LapTracker, start_rad: f64, laps: f64, step_rad: f64) {
        let steps = (laps * 2.0 * PI / step_rad).round() as usize;
        for i in 0..=steps {
            let angle = start_rad + i as f64 * step_rad;
            tracker.update(at(angle - start_rad, angle, 0.1, 2.5));
        }
    }

    #[test]
    fn a_standing_start_times_every_lap_from_the_line() {
        let mut tracker = tracker();
        drive(&mut tracker, 0.0, 3.5, 0.013);

        assert_eq!(tracker.laps.len(), 3);
        for (i, lap) in tracker.laps.iter().enumerate() {
            assert_eq!(lap.number, i as u32 + 1);
            // One radian per second: a lap takes 2 pi seconds.
            assert!((lap.time_s - 2.0 * PI).abs() < 1e-3, "{lap:?}");
            // Driven 0.1 m outside the line.
            let expected_m = 2.0 * PI * (RADIUS_M + 0.1);
            assert!((lap.distance_m - expected_m).abs() < 0.01, "{lap:?}");
            assert!((lap.average_speed_mps - expected_m / lap.time_s).abs() < 1e-3);
        }
        assert_eq!(tracker.current.number, 4);
        assert_eq!(tracker.previous.as_ref().unwrap().number, 3);
    }

    #[test]
    fn the_errors_are_signed_and_fill_every_bin_of_a_lap() {
        let mut tracker = tracker();
        // Big steps, so most bins are skipped between two samples.
        drive(&mut tracker, 0.0, 1.5, 0.15);

        let previous = tracker.previous.as_ref().unwrap();
        for bin in 0..100 {
            // Outside a counterclockwise circle is right of the line.
            let lateral = previous.lateral_m[bin].unwrap();
            assert!((lateral + 0.1).abs() < 0.01, "bin {bin}: {lateral}");
            let speed_error = previous.speed_error_mps[bin].unwrap();
            assert!((speed_error - 0.5).abs() < 1e-6, "bin {bin}");
        }
        // Half a lap into the next one.
        assert!(tracker.current.lateral_m[40].is_some());
        assert!(tracker.current.lateral_m[60].is_none());
    }

    #[test]
    fn a_placement_away_from_the_line_makes_an_untimed_out_lap() {
        let mut tracker = tracker();
        drive(&mut tracker, PI, 1.25, 0.01);

        // Crossed the line half a lap in; not a whole lap since.
        assert!(tracker.laps.is_empty());
        assert_eq!(tracker.current.number, 1);
        let telemetry = tracker.telemetry(None);
        let lap_time_s = telemetry.current_lap_time_s.unwrap();
        assert!((lap_time_s - 0.75 * 2.0 * PI).abs() < 0.02, "{lap_time_s}");
    }

    #[test]
    fn reversing_over_the_line_and_back_completes_no_lap() {
        let mut tracker = tracker();
        tracker.update(at(0.0, 0.3, 0.0, 1.0));
        tracker.update(at(1.0, 0.1, 0.0, 1.0));
        tracker.update(at(2.0, -0.1, 0.0, 1.0));
        tracker.update(at(3.0, 0.1, 0.0, 1.0));
        tracker.update(at(4.0, 0.3, 0.0, 1.0));
        assert!(tracker.laps.is_empty());
    }

    #[test]
    fn a_jump_drops_the_lap_in_progress() {
        let mut tracker = tracker();
        drive(&mut tracker, 0.0, 0.8, 0.01);
        // Teleported back just before the line.
        tracker.update(at(100.0, -0.2, 0.0, 1.0));
        tracker.update(at(100.1, -0.1, 0.0, 1.0));
        tracker.update(at(100.2, 0.1, 0.0, 1.0));

        assert!(tracker.laps.is_empty(), "{:?}", tracker.laps);
        let telemetry = tracker.telemetry(None);
        // Timed from the crossing, halfway between the last two samples.
        let lap_time_s = telemetry.current_lap_time_s.unwrap();
        assert!((lap_time_s - 0.05).abs() < 1e-6, "{lap_time_s}");
    }

    #[test]
    fn a_placement_forgets_the_previous_sample() {
        let mut tracker = tracker();
        drive(&mut tracker, 0.0, 0.8, 0.01);
        tracker.place();
        // Close enough not to be a jump, but placed nonetheless.
        tracker.update(at(50.0, 0.8 * 2.0 * PI + 0.05, 0.0, 0.0));
        assert_eq!(tracker.timing, Timing::Out);
        assert!(tracker.current.lateral_m.iter().flatten().count() == 1);
    }

    #[test]
    fn wrapped_half_picks_the_shorter_way_round() {
        assert!((wrapped_half(9.0, 10.0) + 1.0).abs() < 1e-12);
        assert!((wrapped_half(-9.0, 10.0) - 1.0).abs() < 1e-12);
        assert!((wrapped_half(2.0, 10.0) - 2.0).abs() < 1e-12);
    }
}
