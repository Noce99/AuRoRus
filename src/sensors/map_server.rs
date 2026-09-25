//! [`MapServer`]: watches [`MAP_SELECTION_TOPIC_NAME`] for the wanted map
//! folder and keeps [`MAP_TOPIC_NAME`], [`START_STATE_TOPIC_NAME`] and
//! [`RACE_LINE_TOPIC_NAME`] in sync with it, loading from disk only when the
//! selection actually changes - or, for the race line, when
//! [`crate::planning::Planner`] saves a new one for it - and draws the
//! loaded map and its race line on its own drawing topic (see
//! [`crate::topics::Drawing`]).

use crate::environment::{Map, SpeedPoint};
use crate::topics::{
    Color, Drawing, MAP_SELECTION_TOPIC_NAME, MAP_TOPIC_NAME, MapSelection,
    PLANNING_STATUS_TOPIC_NAME, PlanningStatus, RACE_LINE_TOPIC_NAME, RaceLineKind,
    START_STATE_TOPIC_NAME, SelectedMap, SelectedRaceLine, Shape, StartState,
};
use crate::{Captain, Executor, Ticker};
use std::any::Any;
use std::path::Path;
use std::time::Duration;

/// Every tunable parameter [`MapServer`] needs - loaded from
/// `config/sensors/map_server.toml` (see [`Default`]) or from an arbitrary
/// path via [`crate::config::load`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Deserialize)]
pub struct MapServerConfig {
    /// How often [`MapServer`] checks [`MAP_SELECTION_TOPIC_NAME`] for a
    /// change, in milliseconds.
    pub poll_interval_ms: u64,
}

impl Default for MapServerConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/sensors/map_server.toml"))
            .expect("config/sensors/map_server.toml must deserialize into MapServerConfig")
    }
}

/// Everything [`MapServer`] publishes for one map.
struct Loaded {
    map: SelectedMap,
    start_state: StartState,
    race_line: SelectedRaceLine,
}

/// Loads the map at `path` into a [`SelectedMap`], its matching
/// [`StartState`], and its [`SelectedRaceLine`], or prints a warning and
/// returns `None` if it can't be read - e.g. the selection points at a
/// folder that no longer exists.
fn load(path: &Path) -> Option<Loaded> {
    match Map::load(path) {
        Ok(map) => {
            let selected = SelectedMap {
                path: Some(path.to_path_buf()),
                width_px: map.raster.width_px,
                height_px: map.raster.height_px,
                pixels: map.raster.to_bytes().into(),
                info: Some(map.info.clone()),
            };
            let start_state = start_state(&map);
            Some(Loaded {
                map: selected,
                start_state,
                race_line: race_line(map),
            })
        }
        Err(err) => {
            eprintln!("map_server: failed to load map {path:?}: {err}");
            None
        }
    }
}

/// The line to follow on `map`: its planned minimum-time line if it has
/// one, else its planned race line, else its centerline.
fn race_line(map: Map) -> SelectedRaceLine {
    let (kind, points) = if !map.min_time_race_line.is_empty() {
        (RaceLineKind::MinTime, map.min_time_race_line)
    } else if !map.race_line.is_empty() {
        (RaceLineKind::RaceLine, map.race_line)
    } else if !map.centerline.is_empty() {
        (RaceLineKind::Centerline, map.centerline)
    } else {
        (RaceLineKind::None, Vec::new())
    };
    SelectedRaceLine {
        map: Some(map.folder),
        kind,
        points,
    }
}

/// How many colors [`race_line_shapes`] spreads the speed range over.
const SPEED_COLOR_BINS: usize = 12;

/// What [`MapServer`] publishes on its drawing topic for `map`: the raster
/// itself - sharing `map`'s pixel buffer rather than copying it - its race
/// line, and the start/finish line on top. Empty when no map is loaded.
/// Painted beneath every other drawing, and never fades: it's only
/// republished when the map or its race line changes.
fn drawing(map: &SelectedMap, race_line: &SelectedRaceLine) -> Drawing {
    let Some(info) = &map.info else {
        return Drawing::default().z_index(-100);
    };
    let line = &info.start_finish_line;
    let mut shapes = vec![Shape::Raster {
        origin_x_m: info.origin.x,
        origin_y_m: info.origin.y,
        resolution_m_per_px: info.resolution_m_per_px,
        width_px: map.width_px,
        height_px: map.height_px,
        pixels: map.pixels.clone(),
    }];
    shapes.extend(race_line_shapes(race_line));
    shapes.push(Shape::Polyline {
        points: vec![
            [line.a.x as f32, line.a.y as f32],
            [line.b.x as f32, line.b.y as f32],
        ],
        closed: false,
        width_px: 2.0,
        color: Color::RED,
    });
    Drawing::new(shapes).z_index(-100)
}

/// `race_line` drawn colored by speed - blue at its slowest point, through
/// green, to red at its fastest - as one polyline per run of points in the
/// same color bin (there's no per-vertex color shape). A centerline is
/// drawn thinner than a planned race line.
fn race_line_shapes(race_line: &SelectedRaceLine) -> Vec<Shape> {
    let points = &race_line.points;
    if points.len() < 2 {
        return Vec::new();
    }
    let width_px = match race_line.kind {
        RaceLineKind::RaceLine | RaceLineKind::MinTime => 3.0,
        _ => 1.5,
    };
    let (slowest, fastest) = points.iter().fold((f64::INFINITY, 0.0f64), |(lo, hi), p| {
        (lo.min(p.speed_mps), hi.max(p.speed_mps))
    });
    let bin = |point: &SpeedPoint| -> usize {
        if fastest - slowest < 1e-9 {
            return SPEED_COLOR_BINS / 2;
        }
        let t = (point.speed_mps - slowest) / (fastest - slowest);
        ((t * SPEED_COLOR_BINS as f64) as usize).min(SPEED_COLOR_BINS - 1)
    };

    let mut shapes = Vec::new();
    let n = points.len();
    let mut start = 0;
    while start < n {
        let color_bin = bin(&points[start]);
        let mut end = start + 1;
        while end < n && bin(&points[end]) == color_bin {
            end += 1;
        }
        // Each run reaches on to the next run's first point (wrapping to the
        // first at the end), so the runs join up into the whole closed line.
        let run: Vec<[f32; 2]> = (start..=end)
            .map(|i| &points[i % n])
            .map(|p| [p.x as f32, p.y as f32])
            .collect();
        shapes.push(Shape::Polyline {
            points: run,
            closed: false,
            width_px,
            color: speed_color(color_bin as f64 / (SPEED_COLOR_BINS - 1) as f64),
        });
        start = end;
    }
    shapes
}

/// Blue at `t = 0`, green at `0.5`, red at `1`.
fn speed_color(t: f64) -> Color {
    let lerp = |a: Color, b: Color, t: f64| {
        let mix = |x: u8, y: u8| (x as f64 + (y as f64 - x as f64) * t).round() as u8;
        Color::rgb(mix(a.r, b.r), mix(a.g, b.g), mix(a.b, b.b))
    };
    if t < 0.5 {
        lerp(Color::BLUE, Color::GREEN, t * 2.0)
    } else {
        lerp(Color::GREEN, Color::RED, (t - 0.5) * 2.0)
    }
}

/// The vehicle's initial state for `map`: at rest, on its start/finish
/// line - see [`crate::environment::StartFinishLine::start_pose`].
fn start_state(map: &Map) -> StartState {
    let (x_m, y_m, heading_rad) = map.info.start_finish_line.start_pose();
    StartState {
        x_m,
        y_m,
        heading_rad,
        speed_mps: 0.0,
    }
}

/// Claims [`MAP_TOPIC_NAME`], [`START_STATE_TOPIC_NAME`] and
/// [`RACE_LINE_TOPIC_NAME`] and republishes them whenever
/// [`MAP_SELECTION_TOPIC_NAME`]'s wanted path no longer matches the
/// currently published one - loading the new map from disk, or clearing to
/// their `Default`s if the selection was cleared. Also reloads the race line
/// whenever [`PLANNING_STATUS_TOPIC_NAME`] reports one saved for the
/// published map.
pub struct MapServer {
    id: u8,
    name: String,
    config: MapServerConfig,
}

impl MapServer {
    pub fn new(name: impl Into<String>, config: MapServerConfig) -> Self {
        Self {
            id: 0,
            name: name.into(),
            config,
        }
    }
}

impl Executor for MapServer {
    fn init(&mut self, id: u8) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_writer::<SelectedMap>(MAP_TOPIC_NAME, self.id, SelectedMap::default);
        captain.claim_writer::<StartState>(START_STATE_TOPIC_NAME, self.id, StartState::default);
        captain.claim_writer::<SelectedRaceLine>(
            RACE_LINE_TOPIC_NAME,
            self.id,
            SelectedRaceLine::default,
        );
        captain.claim_drawing(self.id);
    }

    fn run(&mut self, captain: &Captain) {
        let map_topic = captain.topic::<SelectedMap>(MAP_TOPIC_NAME);
        let start_state_topic = captain.topic::<StartState>(START_STATE_TOPIC_NAME);
        let race_line_topic = captain.topic::<SelectedRaceLine>(RACE_LINE_TOPIC_NAME);
        let selection_topic = captain.topic::<MapSelection>(MAP_SELECTION_TOPIC_NAME);
        let drawing_topic = captain.drawing(self.id);
        let mut ticker = Ticker::from_interval(Duration::from_millis(self.config.poll_interval_ms));

        // Which map is currently published, tracked here rather than read back
        // off the topic every poll. This executor is the `map` topic's only
        // writer, so it already knows; reading it back would clone the whole
        // `SelectedMap` - `MapInfo` and all - just to compare one path.
        // Seeded once from the topic so a mid-run restart of just this
        // executor (`Runner::switch_executor`) doesn't reload a map that's
        // already published.
        let mut published = map_topic.read().into_value();
        // The planner's latest outcome already accounted for - seeded the
        // same way, so a restart doesn't reload for an old one.
        let planned_request = || {
            captain
                .try_topic::<PlanningStatus>(PLANNING_STATUS_TOPIC_NAME)
                .and_then(|topic| topic.read().into_value().last_outcome)
        };
        let mut seen_planned = planned_request().map(|outcome| outcome.requested);

        while captain.is_running(self.id) {
            let wanted = selection_topic.read();

            if wanted.path != published.path {
                let next = match &wanted.path {
                    None => Some(Loaded {
                        map: SelectedMap::default(),
                        start_state: StartState::default(),
                        race_line: SelectedRaceLine::default(),
                    }),
                    Some(path) => load(path),
                };
                // Left unchanged when `load` fails, so a selection pointing at
                // an unreadable map is retried on the next poll instead of
                // being recorded as published.
                if let Some(next) = next {
                    drawing_topic
                        .write(self.id, drawing(&next.map, &next.race_line))
                        .expect("lost writer authorization for the map's drawing topic");
                    map_topic
                        .write(self.id, next.map.clone())
                        .expect("lost writer authorization for the map topic");
                    start_state_topic
                        .write(self.id, next.start_state)
                        .expect("lost writer authorization for the start_state topic");
                    race_line_topic
                        .write(self.id, next.race_line)
                        .expect("lost writer authorization for the race_line topic");
                    published = next.map;
                }
            }

            // A race line newly saved for the published map: reload its lines.
            if let Some(outcome) = planned_request()
                && seen_planned != Some(outcome.requested)
            {
                seen_planned = Some(outcome.requested);
                let for_published = published.path.as_ref().is_some_and(|path| {
                    outcome.map.as_deref() == Some(&*path.display().to_string())
                });
                if outcome.saved_to.is_some() && for_published {
                    let path = published.path.clone().expect("checked just above");
                    match Map::load(&path) {
                        Ok(map) => {
                            let reloaded = race_line(map);
                            drawing_topic
                                .write(self.id, drawing(&published, &reloaded))
                                .expect("lost writer authorization for the map's drawing topic");
                            race_line_topic
                                .write(self.id, reloaded)
                                .expect("lost writer authorization for the race_line topic");
                        }
                        Err(err) => {
                            eprintln!("map_server: failed to reload map {path:?}: {err}")
                        }
                    }
                }
            }

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
        Box::new(MapServer::new(self.name.clone(), self.config))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::environment::{
        ImageOrigin, MapInfo, MapSource, Raster, StartFinishLine, WorldPoint,
    };
    use std::path::PathBuf;

    fn test_map(line: StartFinishLine) -> Map {
        Map {
            folder: PathBuf::new(),
            info: MapInfo {
                resolution_m_per_px: 0.05,
                width_px: 10,
                height_px: 10,
                origin: ImageOrigin {
                    x: 0.0,
                    y: 0.0,
                    theta_rad: 0.0,
                },
                start_finish_line: line,
                generated_at: String::new(),
                source: MapSource::Real,
                generation: None,
            },
            raster: Raster::new(10, 10, vec![true; 100]),
            centerline: Vec::new(),
            race_line: Vec::new(),
            min_time_race_line: Vec::new(),
        }
    }

    #[test]
    fn start_state_is_placed_by_the_start_finish_line_alone() {
        // `a` on the left of a vehicle heading along +x.
        let line = StartFinishLine {
            a: WorldPoint { x: 1.0, y: 3.0 },
            b: WorldPoint { x: 1.0, y: -1.0 },
        };

        let state = start_state(&test_map(line));

        assert_eq!(state.x_m, 1.0);
        assert_eq!(state.y_m, 1.0);
        assert!(state.heading_rad.abs() < 1e-12);
        assert_eq!(state.speed_mps, 0.0);
    }

    fn point(x: f64, speed_mps: f64) -> SpeedPoint {
        SpeedPoint {
            x,
            y: 0.0,
            speed_mps,
        }
    }

    #[test]
    fn the_fastest_planned_line_wins() {
        let line = StartFinishLine {
            a: WorldPoint { x: 0.0, y: 1.0 },
            b: WorldPoint { x: 0.0, y: -1.0 },
        };
        let mut map = test_map(line);
        assert_eq!(race_line(test_map(line)).kind, RaceLineKind::None);

        map.centerline = vec![point(0.0, 1.0), point(1.0, 1.0)];
        let centerline_only = race_line(Map {
            centerline: map.centerline.clone(),
            ..test_map(line)
        });
        assert_eq!(centerline_only.kind, RaceLineKind::Centerline);

        map.race_line = vec![point(0.0, 2.0), point(1.0, 3.0), point(2.0, 4.0)];
        let planned = race_line(Map {
            centerline: map.centerline.clone(),
            race_line: map.race_line.clone(),
            ..test_map(line)
        });
        assert_eq!(planned.kind, RaceLineKind::RaceLine);
        assert_eq!(planned.points.len(), 3);

        map.min_time_race_line = vec![point(0.0, 5.0), point(1.0, 5.0)];
        let fastest = race_line(map);
        assert_eq!(fastest.kind, RaceLineKind::MinTime);
        assert_eq!(fastest.points.len(), 2);
    }

    #[test]
    fn the_race_line_is_drawn_as_joined_runs_of_one_color() {
        // Slow, slow, fast, fast: two runs, each reaching on to the next
        // run's first point, the last one wrapping back to the first point.
        let line = SelectedRaceLine {
            map: None,
            kind: RaceLineKind::RaceLine,
            points: vec![
                point(0.0, 1.0),
                point(1.0, 1.0),
                point(2.0, 5.0),
                point(3.0, 5.0),
            ],
        };
        let shapes = race_line_shapes(&line);
        assert_eq!(shapes.len(), 2);
        let xs = |shape: &Shape| match shape {
            Shape::Polyline { points, .. } => points.iter().map(|p| p[0]).collect::<Vec<_>>(),
            _ => panic!("not a polyline"),
        };
        assert_eq!(xs(&shapes[0]), vec![0.0, 1.0, 2.0]);
        assert_eq!(xs(&shapes[1]), vec![2.0, 3.0, 0.0]);
        assert_eq!(speed_color(0.0), Color::BLUE);
        assert_eq!(speed_color(1.0), Color::RED);
    }
}
