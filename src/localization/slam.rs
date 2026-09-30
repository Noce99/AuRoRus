//! [`Slam`]: builds an occupancy map of the track from lidar scans and
//! odometry while the vehicle drives - a Rust port of the mapping core of
//! slam_toolbox (Karto's `Mapper`) - and saves it under `maps/`; or
//! localizes the vehicle on a known map, without changing it. See
//! `documentation/slam.md` for how it's structured, how it's driven from
//! `web_gui`, and what's next.
//!
//! Reads [`LIDAR_SCAN_TOPIC_NAME`], [`ODOMETRY_TOPIC_NAME`],
//! [`SLAM_COMMAND_TOPIC_NAME`], [`SLAM_SAVE_TOPIC_NAME`] and - to localize
//! on it - [`MAP_TOPIC_NAME`]; publishes
//! [`SLAM_STATUS_TOPIC_NAME`],
//! [`SLAM_MAP_TOPIC_NAME`], and - optionally - a drawing of the map, the
//! trajectory, and the estimated vehicle, anchored where odometry was last
//! reset - [`START_STATE_TOPIC_NAME`], or the pose a
//! [`PLACE_AT_START_TOPIC_NAME`] request placed the vehicle at (see
//! [`Placement`]) - so it overlays the true map.

mod correlation_grid;
mod localizer;
mod map_saver;
mod mapper;
mod matrix3;
mod occupancy_grid;
mod odometry_buffer;
mod optimizer;
mod pose;
mod pose_graph;
mod scan;
mod scan_matcher;

use crate::environment;
use crate::topics::{
    Color, Drawing, LIDAR_SCAN_TOPIC_NAME, LidarScan, MAP_TOPIC_NAME, ODOMETRY_TOPIC_NAME,
    Odometry, Placement, PlacementTopics, SLAM_COMMAND_TOPIC_NAME, SLAM_MAP_TOPIC_NAME,
    SLAM_SAVE_TOPIC_NAME, SLAM_STATUS_TOPIC_NAME, SelectedMap, Shape, SlamCommand, SlamMap,
    SlamSaveOutcome, SlamSaveRequest, SlamState, SlamStatus, StartState, VehicleTopics,
};
use crate::{Captain, Executor, Ticker};
use localizer::{Localizer, LocalizerParams};
use mapper::{Mapper, MapperParams, Processed};
use odometry_buffer::{OdometryBuffer, PoseAt};
use pose::Pose2;
use scan::LocalizedScan;
use scan_matcher::MatchParams;
use std::any::Any;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

/// Every tunable parameter [`Slam`] needs - loaded from
/// `config/localization/slam.toml` (see [`Default`]) or from an arbitrary
/// path via [`crate::config::load`]. Names follow slam_toolbox's
/// `mapper_params_online_async.yaml`, with units added.
#[derive(Debug, Clone, Copy, PartialEq, serde::Deserialize)]
pub struct SlamConfig {
    /// How often [`Slam`] polls its input topics, in Hz.
    pub rate_hz: f64,
    /// How many odometry samples are kept to interpolate a scan's pose from.
    pub odometry_buffer_len: usize,
    /// Shortest time between two publications of the map and its drawing,
    /// in seconds.
    pub map_publish_period_s: f64,
    /// Whether to draw the map, trajectory, and estimated vehicle.
    pub draw: bool,

    /// A scan is kept whenever at least this long passed since the last one
    /// kept, moved or not, in seconds.
    pub minimum_time_interval_s: f64,
    /// ... or the vehicle moved at least this far, in meters.
    pub minimum_travel_distance_m: f64,
    /// ... or it turned at least this much, in radians.
    pub minimum_travel_heading_rad: f64,
    /// Most recent scans a new scan is matched against.
    pub scan_buffer_size: usize,
    /// Longest distance the scans matched against may span, in meters.
    pub scan_buffer_maximum_scan_distance_m: f64,

    /// Side of the square window searched around the odometry prior, in
    /// meters.
    pub correlation_search_space_dimension_m: f64,
    /// Resolution of that search, in meters.
    pub correlation_search_space_resolution_m: f64,
    /// How much reference points are blurred before matching, in meters.
    pub correlation_search_space_smear_deviation_m: f64,
    /// Heading search either side of the prior, in radians.
    pub coarse_search_angle_offset_rad: f64,
    /// Heading step of the coarse search, in radians.
    pub coarse_angle_resolution_rad: f64,
    /// Heading step of the fine search, in radians.
    pub fine_search_angle_offset_rad: f64,
    /// How fast a candidate's score falls with its distance from the prior.
    pub distance_variance_penalty: f64,
    /// How fast a candidate's score falls with its heading change from the
    /// prior.
    pub angle_variance_penalty: f64,
    /// Floor of the distance penalty factor.
    pub minimum_distance_penalty: f64,
    /// Floor of the heading penalty factor.
    pub minimum_angle_penalty: f64,
    /// Whether to widen the heading search when nothing overlaps at all.
    pub use_response_expansion: bool,

    /// Readings farther than this only free the cells they cross, and
    /// aren't matched against, in meters.
    pub max_laser_range_m: f64,
    /// Side of one cell of the published map, in meters.
    pub resolution_m: f64,
    /// Beams that must cross a cell before it's anything but unknown.
    pub min_pass_through: u32,
    /// Hit/pass ratio above which a cell is occupied.
    pub occupancy_threshold: f64,

    /// Whether to link scans to older passes over the same place and close
    /// loops - `false` keeps only the chain of sequential matches.
    pub do_loop_closing: bool,
    /// A scan is linked to a nearby chain only if it matches it better than
    /// this.
    pub link_match_minimum_response_fine: f64,
    /// Farthest two scans may be apart to be linked, in meters.
    pub link_scan_maximum_distance_m: f64,
    /// Older scans closer than this are candidates for a loop closure, in
    /// meters.
    pub loop_search_maximum_distance_m: f64,
    /// Fewest consecutive scans a chain needs to be matched against.
    pub loop_match_minimum_chain_size: usize,
    /// A coarse loop match is only trusted if both its position variances
    /// are below this, in square meters.
    pub loop_match_maximum_variance_coarse: f64,
    /// Coarse response a loop match must beat to be refined.
    pub loop_match_minimum_response_coarse: f64,
    /// Fine response a loop match must reach to close the loop.
    pub loop_match_minimum_response_fine: f64,
    /// Side of the square window a loop closure is searched in, in meters.
    pub loop_search_space_dimension_m: f64,
    /// Resolution of that search, in meters.
    pub loop_search_space_resolution_m: f64,
    /// How much reference points are blurred for that search, in meters.
    pub loop_search_space_smear_deviation_m: f64,
    /// Most Levenberg-Marquardt iterations one optimization may take.
    pub optimizer_max_iterations: usize,

    /// While localizing, a match against the map is only trusted at or
    /// above this response - below it, the pose follows odometry alone.
    pub localization_minimum_response: f64,
}

impl Default for SlamConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/localization/slam.toml"))
            .expect("config/localization/slam.toml must deserialize into SlamConfig")
    }
}

impl SlamConfig {
    fn localizer_params(&self) -> LocalizerParams {
        LocalizerParams {
            correlation_search_space_dimension_m: self.correlation_search_space_dimension_m,
            correlation_search_space_resolution_m: self.correlation_search_space_resolution_m,
            correlation_search_space_smear_deviation_m: self
                .correlation_search_space_smear_deviation_m,
            max_laser_range_m: self.max_laser_range_m,
            minimum_response: self.localization_minimum_response,
            matching: self.match_params(),
        }
    }

    fn match_params(&self) -> MatchParams {
        MatchParams {
            coarse_search_angle_offset_rad: self.coarse_search_angle_offset_rad,
            coarse_angle_resolution_rad: self.coarse_angle_resolution_rad,
            fine_search_angle_offset_rad: self.fine_search_angle_offset_rad,
            distance_variance_penalty: self.distance_variance_penalty,
            angle_variance_penalty: self.angle_variance_penalty,
            minimum_distance_penalty: self.minimum_distance_penalty,
            minimum_angle_penalty: self.minimum_angle_penalty,
            use_response_expansion: self.use_response_expansion,
        }
    }

    fn mapper_params(&self) -> MapperParams {
        MapperParams {
            minimum_time_interval_s: self.minimum_time_interval_s,
            minimum_travel_distance_m: self.minimum_travel_distance_m,
            minimum_travel_heading_rad: self.minimum_travel_heading_rad,
            scan_buffer_size: self.scan_buffer_size,
            scan_buffer_maximum_scan_distance_m: self.scan_buffer_maximum_scan_distance_m,
            correlation_search_space_dimension_m: self.correlation_search_space_dimension_m,
            correlation_search_space_resolution_m: self.correlation_search_space_resolution_m,
            correlation_search_space_smear_deviation_m: self
                .correlation_search_space_smear_deviation_m,
            max_laser_range_m: self.max_laser_range_m,
            resolution_m: self.resolution_m,
            min_pass_through: self.min_pass_through,
            occupancy_threshold: self.occupancy_threshold,
            do_loop_closing: self.do_loop_closing,
            link_match_minimum_response_fine: self.link_match_minimum_response_fine,
            link_scan_maximum_distance_m: self.link_scan_maximum_distance_m,
            loop_search_maximum_distance_m: self.loop_search_maximum_distance_m,
            loop_match_minimum_chain_size: self.loop_match_minimum_chain_size,
            loop_match_maximum_variance_coarse: self.loop_match_maximum_variance_coarse,
            loop_match_minimum_response_coarse: self.loop_match_minimum_response_coarse,
            loop_match_minimum_response_fine: self.loop_match_minimum_response_fine,
            loop_search_space_dimension_m: self.loop_search_space_dimension_m,
            loop_search_space_resolution_m: self.loop_search_space_resolution_m,
            loop_search_space_smear_deviation_m: self.loop_search_space_smear_deviation_m,
            optimizer_max_iterations: self.optimizer_max_iterations,
            matching: self.match_params(),
        }
    }
}

/// Size the estimated vehicle is drawn at - the same roughly 1/10-scale RC
/// car [`crate::actuators::SimulatedVehicle`] draws.
const DRAWN_BODY_LENGTH_M: f64 = 0.45;
const DRAWN_BODY_WIDTH_M: f64 = 0.25;
const DRAWN_AXLE_M: f64 = 0.16;
/// Translucent, so the true vehicle stays visible where they overlap.
const DRAWN_COLOR: Color = Color::GREEN.with_alpha(170);
/// The vehicle localized on a known map, told apart from mapping's.
const LOCALIZED_COLOR: Color = Color::BLUE.with_alpha(190);
/// Loop-closure edges stand out from the trajectory.
const LOOP_EDGE_COLOR: Color = Color::AMBER;
/// Radius of the circle marking where a loop was closed, in meters.
const LOOP_MARKER_RADIUS_M: f64 = 0.75;
/// Above [`crate::sensors::MapServer`]'s map (`-100`), below everything
/// else - the raster would otherwise hide the lidar hits and vehicles.
const DRAWN_Z_INDEX: i32 = -50;

/// SLAM: claims [`SLAM_STATUS_TOPIC_NAME`] and [`SLAM_MAP_TOPIC_NAME`] and,
/// while [`SlamCommand::state`] is [`SlamState::Running`], adds every new
/// [`LidarScan`] - posed by interpolating [`Odometry`] at the instant it was
/// written - to the map. [`SlamState::Off`], a bumped
/// [`SlamCommand::clear_requested`], or odometry being reset all throw the
/// map away. A [`SlamSaveRequest`] saves the map built so far as a new map
/// folder under `maps_root`, reporting how it went on
/// [`SlamStatus::last_save`].
///
/// While [`SlamCommand::state`] is [`SlamState::Localizing`], every new scan
/// is matched against the selected map ([`MAP_TOPIC_NAME`]) instead,
/// starting from [`START_STATE_TOPIC_NAME`] - where odometry was reset -
/// and the map is never changed. That's refused while SLAM has a map of its
/// own in memory: clear or save it first.
pub struct Slam {
    id: u16,
    name: String,
    maps_root: PathBuf,
    config: SlamConfig,
}

impl Slam {
    /// Creates a `Slam` that saves maps under `maps_root`.
    pub fn new(name: impl Into<String>, maps_root: impl Into<PathBuf>, config: SlamConfig) -> Self {
        Self {
            id: 0,
            name: name.into(),
            maps_root: maps_root.into(),
            config,
        }
    }
}

impl Executor for Slam {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_writer::<SlamStatus>(SLAM_STATUS_TOPIC_NAME, self.id, SlamStatus::default);
        captain.claim_writer::<SlamMap>(SLAM_MAP_TOPIC_NAME, self.id, SlamMap::default);
        if self.config.draw {
            captain.claim_drawing(self.id);
        }
    }

    fn run(&mut self, captain: &Captain) {
        let lidar_topic = captain.topic::<LidarScan>(LIDAR_SCAN_TOPIC_NAME);
        let odometry_topic = captain.topic::<Odometry>(ODOMETRY_TOPIC_NAME);
        let command_topic = captain.topic::<SlamCommand>(SLAM_COMMAND_TOPIC_NAME);
        let save_topic = captain.topic::<SlamSaveRequest>(SLAM_SAVE_TOPIC_NAME);
        let selected_map_topic = captain.topic::<SelectedMap>(MAP_TOPIC_NAME);
        let placement_topics = PlacementTopics::new(captain);
        let status_topic = captain.topic::<SlamStatus>(SLAM_STATUS_TOPIC_NAME);
        let map_topic = captain.topic::<SlamMap>(SLAM_MAP_TOPIC_NAME);
        let drawing_topic = self.config.draw.then(|| captain.drawing(self.id));

        let mut state = State::new(&self.config);
        // Where odometry was last reset: the start line, or wherever the
        // vehicle was placed since.
        let mut placement = Placement::new(&placement_topics.read());
        // Start from whatever is there now, so a scan left over from before
        // this executor started isn't taken.
        let mut last_scan_write = lidar_topic.read().meta.write_count;
        let mut last_odometry_write = None;
        let mut applied_clear = command_topic.read().clear_requested;
        // A request left over from before this executor started isn't
        // answered.
        let mut applied_save = save_topic.read().requested;
        let map_publish_period = Duration::from_secs_f64(self.config.map_publish_period_s);
        let mut last_published: Option<Instant> = None;
        let mut ticker = Ticker::new(self.config.rate_hz);

        while captain.is_running(self.id) {
            placement.update(&placement_topics.read(), &VehicleTopics::ego());
            let odometry = odometry_topic.read();
            if last_odometry_write != Some(odometry.meta.write_count) {
                last_odometry_write = Some(odometry.meta.write_count);
                if let Some(written_at) = odometry.meta.written_at {
                    state.odometry.push(written_at, odometry.value);
                }
            }

            let command = command_topic.read().into_value();
            let clear_requested = command.clear_requested != applied_clear;
            applied_clear = command.clear_requested;
            if clear_requested || command.state == SlamState::Off {
                state.clear();
            }
            // A reset of odometry not announced through the command (e.g.
            // from something other than web_gui): the map's frame is gone.
            if let (Some(built_in), Some(current)) =
                (state.map_reset_count, state.odometry.reset_count())
                && built_in != current
            {
                state.clear();
            }

            // Localization never runs over a map being built.
            let effective_state =
                if command.state.is_localization() && state.mapper.scan_count() > 0 {
                    SlamState::Waiting
                } else {
                    command.state
                };
            if effective_state.is_localization() {
                let selected_map = selected_map_topic.read();
                state.update_localization(
                    &selected_map.value,
                    selected_map.meta.write_count,
                    &placement.anchor(),
                    self.config.localizer_params(),
                );
            } else {
                state.stop_localizing();
            }

            let scan = lidar_topic.read();
            let new_scan = scan.meta.write_count != last_scan_write;
            last_scan_write = scan.meta.write_count;
            let processing = effective_state == SlamState::Running
                || (effective_state == SlamState::Localizing && state.localization.is_some());
            if processing {
                // An older scan still waiting for odometry to catch up goes
                // first; a newer one replaces it only if it's still waiting
                // after that.
                state.try_pending();
                if new_scan && let Some(written_at) = scan.meta.written_at {
                    state.pending = Some((written_at, scan.value));
                    state.try_pending();
                }
            } else {
                state.pending = None;
            }

            let save = save_topic.read().into_value();
            if save.requested != applied_save {
                applied_save = save.requested;
                state.last_save = Some(state.save(&save, &self.maps_root));
            }

            status_topic
                .write(self.id, state.status(effective_state))
                .expect("lost writer authorization for the slam_status topic");

            if let Some(localization) = &state.localization {
                if state.localization_changed {
                    state.localization_changed = false;
                    if let Some(drawing_topic) = &drawing_topic {
                        drawing_topic
                            .write(self.id, localized_drawing(localization.localizer.pose()))
                            .expect("lost writer authorization for SLAM's drawing topic");
                    }
                }
            } else if state.map_changed
                && last_published.is_none_or(|at| at.elapsed() >= map_publish_period)
            {
                state.map_changed = false;
                last_published = Some(Instant::now());
                let map = state.mapper.map();
                if let Some(drawing_topic) = &drawing_topic {
                    drawing_topic
                        .write(
                            self.id,
                            drawing(
                                &map,
                                state.mapper.pose(),
                                &state.mapper.loop_edges(),
                                &placement.anchor(),
                            ),
                        )
                        .expect("lost writer authorization for SLAM's drawing topic");
                }
                map_topic
                    .write(self.id, map)
                    .expect("lost writer authorization for the slam_map topic");
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
        Box::new(Slam::new(
            self.name.clone(),
            self.maps_root.clone(),
            self.config,
        ))
    }
}

/// Everything [`Slam::run`] accumulates.
struct State {
    mapper: Mapper,
    odometry: OdometryBuffer,
    /// The latest scan not processed yet, and when it was written - waiting
    /// for odometry to cover that instant.
    pending: Option<(Instant, LidarScan)>,
    /// The [`Odometry::reset_count`] the map is built in, once it has a
    /// scan.
    map_reset_count: Option<u64>,
    last_match_response: Option<f64>,
    last_process_ms: Option<f64>,
    /// Whether the map changed since it was last published.
    map_changed: bool,
    last_save: Option<SlamSaveOutcome>,
    /// Localizing on a known map - `None` unless localizing, or with no map
    /// selected.
    localization: Option<Localization>,
    /// Whether the localized pose changed since it was last drawn.
    localization_changed: bool,
}

/// A [`Localizer`], and what it was built for: once either changes, it's
/// rebuilt.
struct Localization {
    localizer: Localizer,
    /// The write of [`MAP_TOPIC_NAME`] whose map it localizes on.
    map_write: u64,
    /// The [`Odometry::reset_count`] its starting pose was taken for.
    odometry_reset_count: Option<u64>,
}

impl State {
    fn new(config: &SlamConfig) -> Self {
        Self {
            mapper: Mapper::new(config.mapper_params()),
            odometry: OdometryBuffer::new(config.odometry_buffer_len),
            pending: None,
            map_reset_count: None,
            last_match_response: None,
            last_process_ms: None,
            // Publish the (empty) map once right away.
            map_changed: true,
            last_save: None,
            localization: None,
            localization_changed: false,
        }
    }

    /// Localizes on `map` (the `map_write`th write of [`MAP_TOPIC_NAME`]),
    /// (re)starting from `start` - where odometry was reset - whenever the
    /// map or odometry's reset count changed. Stops localizing if no map
    /// is loaded.
    fn update_localization(
        &mut self,
        map: &SelectedMap,
        map_write: u64,
        start: &StartState,
        params: LocalizerParams,
    ) {
        let odometry_reset_count = self.odometry.reset_count();
        let current = self.localization.as_ref().is_some_and(|localization| {
            localization.map_write == map_write
                && localization.odometry_reset_count == odometry_reset_count
        });
        if current {
            return;
        }
        let rebuilt = map.info.is_some().then(|| Localization {
            localizer: Localizer::new(
                params,
                localizer::wall_points(map),
                Pose2::new(start.x_m, start.y_m, start.heading_rad),
            ),
            map_write,
            odometry_reset_count,
        });
        // A scan waiting since before is for the old start or map.
        self.pending = None;
        self.last_match_response = None;
        self.last_process_ms = None;
        self.localization = rebuilt;
        self.localization_changed = true;
        // Nothing drawn left over from the old localization, if it's gone.
        self.map_changed = true;
    }

    /// Forgets the localization, if any.
    fn stop_localizing(&mut self) {
        if self.localization.take().is_some() {
            self.pending = None;
            self.last_match_response = None;
            self.last_process_ms = None;
            // Draw the mapping state again, replacing the localized vehicle.
            self.map_changed = true;
        }
    }

    /// Throws the map away. Odometry samples are kept - they're still
    /// valid, just not tied to any map anymore.
    fn clear(&mut self) {
        let had_anything = self.mapper.scan_count() > 0 || self.map_reset_count.is_some();
        self.mapper.reset();
        self.pending = None;
        self.map_reset_count = None;
        self.last_match_response = None;
        self.last_process_ms = None;
        if had_anything {
            self.map_changed = true;
        }
    }

    /// Processes the pending scan if odometry covers the instant it was
    /// written, drops it if odometry never will, or leaves it waiting.
    fn try_pending(&mut self) {
        let Some((written_at, _)) = &self.pending else {
            return;
        };
        let pose = match self.odometry.pose_at(*written_at) {
            PoseAt::Pose(pose) => pose,
            PoseAt::NotYet => return,
            PoseAt::Unavailable => {
                self.pending = None;
                return;
            }
        };
        let (written_at, scan) = self.pending.take().expect("checked above");
        let started = Instant::now();

        if let Some(localization) = &mut self.localization {
            let threshold_m = localization
                .localizer
                .range_threshold_m(f64::from(scan.max_distance));
            let localized = localization.localizer.process(LocalizedScan::new(
                &scan,
                written_at,
                pose,
                threshold_m,
            ));
            self.last_match_response = Some(localized.response);
            self.last_process_ms = Some(started.elapsed().as_secs_f64() * 1000.0);
            self.localization_changed = true;
            return;
        }

        let threshold_m = self.mapper.range_threshold_m(f64::from(scan.max_distance));
        let processed =
            self.mapper
                .process(LocalizedScan::new(&scan, written_at, pose, threshold_m));
        match processed {
            Processed::Skipped => return,
            Processed::First => self.last_match_response = None,
            Processed::Matched { result, .. } => self.last_match_response = Some(result.response),
        }
        self.last_process_ms = Some(started.elapsed().as_secs_f64() * 1000.0);
        self.map_reset_count = self.odometry.reset_count();
        self.map_changed = true;
    }

    fn status(&self, state: SlamState) -> SlamStatus {
        let as_array = |pose: Pose2| [pose.x_m, pose.y_m, pose.heading_rad];
        let (pose, map_to_odom) = match &self.localization {
            Some(localization) => (
                localization.localizer.pose(),
                Some(localization.localizer.map_to_odom()),
            ),
            None => (self.mapper.pose(), self.mapper.map_to_odom()),
        };
        SlamStatus {
            state,
            scans: self.mapper.scan_count(),
            pose: pose.map(as_array),
            map_to_odom: map_to_odom.map(as_array),
            odometry_reset_count: self
                .map_reset_count
                .or(self.odometry.reset_count())
                .unwrap_or(0),
            last_match_response: self.last_match_response,
            last_process_ms: self.last_process_ms,
            loop_closures: self.mapper.loop_closures(),
            last_optimization_ms: self.mapper.last_optimization_ms(),
            last_save: self.last_save.clone(),
        }
    }

    /// Saves the map built so far as `request`'s folder under `maps_root`
    /// (see [`map_saver`]).
    fn save(&self, request: &SlamSaveRequest, maps_root: &Path) -> SlamSaveOutcome {
        let saved = save_map(
            &self.mapper.map(),
            self.mapper.first_pose(),
            &request.name,
            maps_root,
        );
        match &saved {
            Ok(folder) => println!("slam: saved the map to {folder:?}"),
            Err(err) => eprintln!("slam: failed to save the map: {err}"),
        }
        SlamSaveOutcome {
            requested: request.requested,
            saved_to: saved
                .as_ref()
                .ok()
                .map(|folder| folder.display().to_string()),
            error: saved.err(),
        }
    }
}

/// Saves `map`, started from `first_pose`, as the map folder `name` under
/// `maps_root` - the folder, or why it couldn't be saved.
fn save_map(
    map: &SlamMap,
    first_pose: Option<Pose2>,
    name: &str,
    maps_root: &Path,
) -> Result<PathBuf, String> {
    let folder = environment::map_folder(maps_root, name.trim())
        .ok_or_else(|| format!("invalid map name {name:?}"))?;
    let start = first_pose.ok_or_else(|| map_saver::ExportError::Empty.to_string())?;
    let (info, raster) = map_saver::export(map, start).map_err(|err| err.to_string())?;
    environment::save(&folder, &info, &raster).map_err(|err| err.to_string())?;
    Ok(folder)
}

/// What [`Slam`] draws: `map`, its trajectory, and a vehicle at `pose`,
/// with SLAM's frame placed at `anchor` - where dead reckoning (whose
/// `odom` frame SLAM's frame is) was last reset - so it all overlays the
/// true map. Empty when the map is.
fn drawing(
    map: &SlamMap,
    pose: Option<Pose2>,
    loop_edges: &[(Pose2, Pose2)],
    anchor: &StartState,
) -> Drawing {
    let empty = Drawing::default().z_index(DRAWN_Z_INDEX);
    let Some(pose) = pose else {
        return empty;
    };
    if map.width_px == 0 {
        return empty;
    }
    let anchor = Pose2::new(anchor.x_m, anchor.y_m, anchor.heading_rad);
    let vehicle = anchor.compose(&pose);
    let trajectory = map
        .trajectory
        .iter()
        .map(|&[x, y]| {
            let (x, y) = anchor.transform_point(f64::from(x), f64::from(y));
            [x as f32, y as f32]
        })
        .collect();

    let mut loop_closures = Vec::new();
    for (from, to) in loop_edges {
        let world = |pose: &Pose2| {
            let (x, y) = anchor.transform_point(pose.x_m, pose.y_m);
            [x as f32, y as f32]
        };
        // Once optimized, the two ends of a loop edge are usually only
        // centimeters apart: the circle is what makes the closure visible.
        let [x, y] = world(to);
        loop_closures.push(Shape::Circle {
            x_m: f64::from(x),
            y_m: f64::from(y),
            radius_m: LOOP_MARKER_RADIUS_M,
            filled: false,
            color: LOOP_EDGE_COLOR,
        });
        loop_closures.push(Shape::Polyline {
            points: vec![world(from), world(to)],
            closed: false,
            width_px: 3.0,
            color: LOOP_EDGE_COLOR,
        });
    }
    let vehicle = Shape::Vehicle {
        x_m: vehicle.x_m,
        y_m: vehicle.y_m,
        heading_rad: vehicle.heading_rad,
        // Only redrawn every `map_publish_period_s`: don't let a viewer
        // extrapolate it forward in between.
        speed_mps: 0.0,
        steering_rad: 0.0,
        length_m: DRAWN_BODY_LENGTH_M,
        width_m: DRAWN_BODY_WIDTH_M,
        front_axle_m: DRAWN_AXLE_M,
        rear_axle_m: DRAWN_AXLE_M,
        color: DRAWN_COLOR,
    };
    Drawing::default()
        .element("Map", [world_raster(map, &anchor)], true)
        .element(
            "Trajectory",
            [Shape::Polyline {
                points: trajectory,
                closed: false,
                width_px: 2.0,
                color: DRAWN_COLOR,
            }],
            true,
        )
        .element("Loop closures", loop_closures, true)
        .element("Vehicle", [vehicle], true)
        .z_index(DRAWN_Z_INDEX)
}

/// What [`Slam`] draws while localizing: the vehicle at `pose`, already in
/// the map's frame. Empty before the first scan.
fn localized_drawing(pose: Option<Pose2>) -> Drawing {
    let vehicle = pose.map(|pose| Shape::Vehicle {
        x_m: pose.x_m,
        y_m: pose.y_m,
        heading_rad: pose.heading_rad,
        // Redrawn on every scan only: don't extrapolate in between.
        speed_mps: 0.0,
        steering_rad: 0.0,
        length_m: DRAWN_BODY_LENGTH_M,
        width_m: DRAWN_BODY_WIDTH_M,
        front_axle_m: DRAWN_AXLE_M,
        rear_axle_m: DRAWN_AXLE_M,
        color: LOCALIZED_COLOR,
    });
    Drawing::default()
        .element("Vehicle", vehicle, true)
        .z_index(DRAWN_Z_INDEX)
}

/// `map` resampled into the world frame, with SLAM's frame placed at
/// `anchor`: a [`Shape::Raster`] can't be rotated, so every world pixel
/// looks up the map pixel under its center (nearest neighbor). World pixels
/// outside the map are [`SlamMap::UNKNOWN`].
fn world_raster(map: &SlamMap, anchor: &Pose2) -> Shape {
    let resolution_m = map.resolution_m_per_px;
    let width_m = f64::from(map.width_px) * resolution_m;
    let height_m = f64::from(map.height_px) * resolution_m;
    let corners = [
        (0.0, 0.0),
        (width_m, 0.0),
        (0.0, height_m),
        (width_m, height_m),
    ]
    .map(|(dx, dy)| anchor.transform_point(map.origin_x_m + dx, map.origin_y_m + dy));
    let min_x = corners.iter().map(|c| c.0).fold(f64::INFINITY, f64::min);
    let min_y = corners.iter().map(|c| c.1).fold(f64::INFINITY, f64::min);
    let max_x = corners
        .iter()
        .map(|c| c.0)
        .fold(f64::NEG_INFINITY, f64::max);
    let max_y = corners
        .iter()
        .map(|c| c.1)
        .fold(f64::NEG_INFINITY, f64::max);
    let world_width = ((max_x - min_x) / resolution_m).ceil() as u32;
    let world_height = ((max_y - min_y) / resolution_m).ceil() as u32;

    let to_map = anchor.inverse();
    let mut pixels = Vec::with_capacity((world_width * world_height) as usize);
    for row in 0..world_height {
        let y = min_y + (f64::from(row) + 0.5) * resolution_m;
        for col in 0..world_width {
            let x = min_x + (f64::from(col) + 0.5) * resolution_m;
            let (mx, my) = to_map.transform_point(x, y);
            let map_col = ((mx - map.origin_x_m) / resolution_m).floor();
            let map_row = ((my - map.origin_y_m) / resolution_m).floor();
            let inside = map_col >= 0.0
                && map_row >= 0.0
                && map_col < f64::from(map.width_px)
                && map_row < f64::from(map.height_px);
            pixels.push(if inside {
                map.pixels[map_row as usize * map.width_px as usize + map_col as usize]
            } else {
                SlamMap::UNKNOWN
            });
        }
    }

    Shape::Raster {
        origin_x_m: min_x,
        origin_y_m: min_y,
        resolution_m_per_px: resolution_m,
        width_px: world_width,
        height_px: world_height,
        pixels: pixels.into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    #[test]
    fn the_checked_in_config_deserializes() {
        let config = SlamConfig::default();
        assert!(config.rate_hz > 0.0);
        assert!(config.odometry_buffer_len >= 2);
    }

    /// A 2x1 map, free then occupied, 1 m pixels, corner at the origin.
    fn tiny_map() -> SlamMap {
        SlamMap {
            resolution_m_per_px: 1.0,
            origin_x_m: 0.0,
            origin_y_m: 0.0,
            width_px: 2,
            height_px: 1,
            pixels: vec![SlamMap::FREE, SlamMap::OCCUPIED].into(),
            trajectory: vec![[0.5, 0.5]],
        }
    }

    #[test]
    fn the_world_raster_is_the_map_itself_under_an_identity_anchor() {
        let Shape::Raster {
            width_px,
            height_px,
            pixels,
            ..
        } = world_raster(&tiny_map(), &Pose2::default())
        else {
            panic!("world_raster must return a raster");
        };
        assert_eq!((width_px, height_px), (2, 1));
        assert_eq!(&*pixels, &[SlamMap::FREE, SlamMap::OCCUPIED]);
    }

    #[test]
    fn the_world_raster_is_rotated_with_the_anchor() {
        // A quarter turn: the map's +x runs along the world's +y.
        let anchor = Pose2::new(10.0, 0.0, FRAC_PI_2);
        let Shape::Raster {
            origin_x_m,
            origin_y_m,
            width_px,
            height_px,
            pixels,
            ..
        } = world_raster(&tiny_map(), &anchor)
        else {
            panic!("world_raster must return a raster");
        };
        assert_eq!((width_px, height_px), (1, 2));
        assert!((origin_x_m - 9.0).abs() < 1e-9 && origin_y_m.abs() < 1e-9);
        // Row 0 (low y) is the map's first pixel.
        assert_eq!(&*pixels, &[SlamMap::FREE, SlamMap::OCCUPIED]);
    }

    #[test]
    fn nothing_is_drawn_before_the_first_scan() {
        let drawing = drawing(&SlamMap::default(), None, &[], &StartState::default());
        assert!(drawing.shapes.is_empty());
    }

    #[test]
    fn loop_edges_are_drawn_through_the_anchor() {
        let anchor = StartState {
            x_m: 10.0,
            y_m: 5.0,
            heading_rad: FRAC_PI_2,
            speed_mps: 0.0,
        };
        let edge = (Pose2::new(0.0, 0.0, 0.0), Pose2::new(1.0, 0.0, 0.0));
        let drawing = drawing(&tiny_map(), Some(Pose2::default()), &[edge], &anchor);
        let segment = drawing
            .shapes
            .iter()
            .find_map(|shape| match shape {
                Shape::Polyline { points, color, .. } if *color == LOOP_EDGE_COLOR => {
                    Some(points.clone())
                }
                _ => None,
            })
            .expect("the drawing must include the loop edge");
        assert_eq!(segment, vec![[10.0, 5.0], [10.0, 6.0]]);
        let marked = drawing.shapes.iter().any(|shape| {
            matches!(shape, Shape::Circle { x_m, y_m, color, .. }
                if *color == LOOP_EDGE_COLOR && (*x_m - 10.0).abs() < 1e-6 && (*y_m - 6.0).abs() < 1e-6)
        });
        assert!(marked, "the closure must be marked with a circle");
    }

    #[test]
    fn the_drawn_vehicle_is_placed_through_the_anchor() {
        let anchor = StartState {
            x_m: 10.0,
            y_m: 5.0,
            heading_rad: FRAC_PI_2,
            speed_mps: 0.0,
        };
        let drawing = drawing(&tiny_map(), Some(Pose2::new(1.0, 0.0, 0.2)), &[], &anchor);
        let vehicle = drawing
            .shapes
            .iter()
            .find_map(|shape| match shape {
                Shape::Vehicle {
                    x_m,
                    y_m,
                    heading_rad,
                    ..
                } => Some((*x_m, *y_m, *heading_rad)),
                _ => None,
            })
            .expect("the drawing must include the estimated vehicle");
        assert!((vehicle.0 - 10.0).abs() < 1e-9);
        assert!((vehicle.1 - 6.0).abs() < 1e-9);
        assert!((vehicle.2 - (FRAC_PI_2 + 0.2)).abs() < 1e-9);
    }
}
