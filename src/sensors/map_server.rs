//! [`MapServer`]: watches [`MAP_SELECTION_TOPIC_NAME`] for the wanted map
//! folder and keeps [`MAP_TOPIC_NAME`], [`START_STATE_TOPIC_NAME`] and
//! [`RACE_LINE_TOPIC_NAME`] in sync with it, loading from disk only when the
//! selection actually changes - or, for the race line, when
//! [`RACE_LINE_SELECTION_TOPIC_NAME`] picks another one or
//! [`crate::planning::Planner`] saves a new one - and draws the loaded map
//! and its race line on its own drawing topic (see
//! [`crate::topics::Drawing`]).

use crate::environment::{Map, RaceLineMethod, SpeedPoint, race_lines};
use crate::topics::{
    Color, Drawing, MAP_SELECTION_TOPIC_NAME, MAP_TOPIC_NAME, MapSelection,
    PLANNING_STATUS_TOPIC_NAME, PlanningStatus, RACE_LINE_SELECTION_TOPIC_NAME,
    RACE_LINE_TOPIC_NAME, RaceLineSelection, START_STATE_TOPIC_NAME, SelectedMap, SelectedRaceLine,
    Shape, StartState,
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
            Some(Loaded {
                map: selected,
                start_state: start_state(&map),
                race_line: default_race_line(path),
            })
        }
        Err(err) => {
            eprintln!("map_server: failed to load map {path:?}: {err}");
            None
        }
    }
}

/// The line to follow on the map in `folder` when nobody picked one - see
/// [`race_lines::default_line`] - or no line at all if it has none.
fn default_race_line(folder: &Path) -> SelectedRaceLine {
    let entries = race_lines::list(folder);
    race_lines::default_line(&entries)
        .and_then(|entry| race_line(folder, &entry.file))
        .unwrap_or_else(|| SelectedRaceLine {
            map: Some(folder.to_path_buf()),
            ..SelectedRaceLine::default()
        })
}

/// The map in `folder`'s race line `file`, or `None` (with a warning) if it
/// can't be read.
fn race_line(folder: &Path, file: &str) -> Option<SelectedRaceLine> {
    match race_lines::read(folder, file) {
        Ok(points) => Some(SelectedRaceLine {
            map: Some(folder.to_path_buf()),
            file: Some(file.to_string()),
            method: race_lines::meta(folder, file).method,
            points,
        }),
        Err(err) => {
            eprintln!("map_server: failed to load race line {file:?} of {folder:?}: {err}");
            None
        }
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
    let raster = Shape::Raster {
        origin_x_m: info.origin.x,
        origin_y_m: info.origin.y,
        resolution_m_per_px: info.resolution_m_per_px,
        width_px: map.width_px,
        height_px: map.height_px,
        pixels: map.pixels.clone(),
    };
    let start_finish_line = Shape::Polyline {
        points: vec![
            [line.a.x as f32, line.a.y as f32],
            [line.b.x as f32, line.b.y as f32],
        ],
        closed: false,
        width_px: 2.0,
        color: Color::RED,
    };
    Drawing::default()
        .element("Map", [raster], true)
        .element("Race line", race_line_shapes(race_line), true)
        .element("Start/finish line", [start_finish_line], true)
        .z_index(-100)
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
    let width_px = match race_line.method {
        RaceLineMethod::Centerline => 1.5,
        _ => 3.0,
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
/// their `Default`s if the selection was cleared. Also switches the race
/// line whenever [`RACE_LINE_SELECTION_TOPIC_NAME`] picks one of the
/// published map's, and to the newest one whenever
/// [`PLANNING_STATUS_TOPIC_NAME`] reports one saved for it.
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
        // Likewise the race line selections already accounted for, by write
        // count: every new write is acted on, even of an unchanged value.
        let race_line_selection_writes = || {
            captain
                .try_topic::<RaceLineSelection>(RACE_LINE_SELECTION_TOPIC_NAME)
                .map(|topic| topic.meta().write_count)
        };
        let mut seen_selection = race_line_selection_writes();
        let publish_race_line = |map: &SelectedMap, line: SelectedRaceLine| {
            drawing_topic
                .write(self.id, drawing(map, &line))
                .expect("lost writer authorization for the map's drawing topic");
            race_line_topic
                .write(self.id, line)
                .expect("lost writer authorization for the race_line topic");
        };

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

            // A race line picked for the published map: switch to it. One
            // picked for another map (e.g. just before switching maps) is
            // dropped - the new map starts on its default line.
            let selection_writes = race_line_selection_writes();
            if selection_writes != seen_selection {
                seen_selection = selection_writes;
                let selection = captain
                    .topic::<RaceLineSelection>(RACE_LINE_SELECTION_TOPIC_NAME)
                    .read()
                    .into_value();
                if let Some(path) = &published.path
                    && selection.map.as_ref() == Some(path)
                    && let Some(line) = race_line(path, &selection.file)
                {
                    publish_race_line(&published, line);
                }
            }

            // A race line newly saved for the published map: switch to the
            // newest one - the one just saved.
            if let Some(outcome) = planned_request()
                && seen_planned != Some(outcome.requested)
            {
                seen_planned = Some(outcome.requested);
                let for_published = published.path.as_ref().is_some_and(|path| {
                    outcome.map.as_deref() == Some(&*path.display().to_string())
                });
                if outcome.saved_to.is_some() && for_published {
                    let path = published.path.clone().expect("checked just above");
                    publish_race_line(&published, default_race_line(&path));
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
    fn a_map_s_default_race_line_is_its_newest_planned_one() {
        let folder =
            std::env::temp_dir().join(format!("aurorus_map_server_{}", std::process::id()));
        std::fs::remove_dir_all(&folder).ok();
        assert_eq!(default_race_line(&folder).file, None);

        let line = vec![point(0.0, 1.0), point(1.0, 1.0), point(1.0, 2.0)];
        let older = race_lines::save_new(&folder, RaceLineMethod::MinCurvature, &line).unwrap();
        let newer = race_lines::save_new(&folder, RaceLineMethod::MinTime, &line).unwrap();

        let default = default_race_line(&folder);
        let picked = race_line(&folder, &older).unwrap();
        std::fs::remove_dir_all(&folder).ok();

        assert_eq!(default.file.as_deref(), Some(newer.as_str()));
        assert_eq!(default.method, RaceLineMethod::MinTime);
        assert_eq!(default.points.len(), 3);
        assert_eq!(picked.method, RaceLineMethod::MinCurvature);
    }

    #[test]
    fn the_race_line_is_drawn_as_joined_runs_of_one_color() {
        // Slow, slow, fast, fast: two runs, each reaching on to the next
        // run's first point, the last one wrapping back to the first point.
        let line = SelectedRaceLine {
            map: None,
            file: None,
            method: RaceLineMethod::MinCurvature,
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
