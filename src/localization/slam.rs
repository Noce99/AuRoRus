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
//! [`Placement`]) - so it overlays the true map. Its parameters are tuned
//! live through [`SLAM_PARAMETERS_TOPIC_NAME`].
//!
//! Localization runs one of two backends, picked on
//! [`LOCALIZATION_COMMAND_TOPIC_NAME`]: the scan matcher ([`localizer`]) or
//! the particle filter ([`crate::localization::particle_filter`], tuned
//! through [`PARTICLE_FILTER_PARAMETERS_TOPIC_NAME`]). Both report the
//! same pose and `map_to_odom` on [`SLAM_STATUS_TOPIC_NAME`], and what the
//! backend is doing on [`LOCALIZATION_STATUS_TOPIC_NAME`].

mod correlation_grid;
mod localizer;
mod map_saver;
mod mapper;
mod matrix3;
mod occupancy_grid;
mod odometry_buffer;
mod optimizer;
pub(super) mod pose;
mod pose_graph;
mod rebuild;
mod scan;
mod scan_matcher;
mod scores;

pub use rebuild::{KeptScan, ProcessedScan, RebuildOutcome, map_walls, rebuild};
pub use scores::{Scores, TrajectoryError, WallError, score};

use super::particle_filter::{self, ParticleFilter, ParticleFilterConfig, Start};
use crate::environment;
use crate::topics::{
    AlgorithmParameter, Color, Drawing, DrawingExt, LIDAR_SCAN_TOPIC_NAME,
    LOCALIZATION_COMMAND_TOPIC_NAME, LOCALIZATION_STATUS_TOPIC_NAME, LidarScan,
    LocalizationAlgorithm, LocalizationCommand, LocalizationStatus, MAP_TOPIC_NAME,
    ODOMETRY_TOPIC_NAME, Odometry, PARTICLE_FILTER_PARAMETERS_TOPIC_NAME, ParticleFilterParameters,
    Placement, PlacementTopics, SLAM_COMMAND_TOPIC_NAME, SLAM_MAP_TOPIC_NAME,
    SLAM_PARAMETERS_TOPIC_NAME, SLAM_SAVE_TOPIC_NAME, SLAM_STATUS_TOPIC_NAME, SelectedMap, Shape,
    SlamCommand, SlamMap, SlamParameters, SlamSaveOutcome, SlamSaveRequest, SlamState, SlamStatus,
    StartState, VehicleGeometry, VehicleTopics,
};
use crate::{Captain, Executor, Ticker};
use localizer::{Localizer, LocalizerParams};
use mapper::{Mapper, MapperParams, Processed};
use odometry_buffer::{OdometryBuffer, PoseAt};
use pose::Pose2;
use scan::LocalizedScan;
use scan_matcher::MatchParams;
use std::any::Any;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

/// Every tunable parameter [`Slam`] needs - loaded from
/// `config/localization/slam.toml` (see [`Default`]) or from an arbitrary
/// path via [`crate::config::load`]. Names follow slam_toolbox's
/// `mapper_params_online_async.yaml`, with units added. Every numeric field
/// is live-tunable, see [`tunable_parameters`].
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
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
    /// Older scans are candidates for a loop closure only if their heading
    /// is closer than this to the new one's, in radians.
    pub loop_search_maximum_heading_difference_rad: f64,
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
    /// This config with `values` - tunable parameters by name, see
    /// [`tunable_parameters`] - applied, each brought into its range, or why
    /// it can't be run with.
    pub fn with_parameters(&self, values: &BTreeMap<String, f64>) -> Result<Self, String> {
        let mut tuned = *self;
        crate::config::apply_parameters(&mut tuned, &tunable_parameters(), values);
        tuned.validate()?;
        Ok(tuned)
    }

    /// Why this config can't be run with, if it can't: each search's smear
    /// deviation must be between half and ten times its resolution (see
    /// [`correlation_grid::CorrelationGrid::new`]).
    fn validate(&self) -> Result<(), String> {
        let searches = [
            (
                "correlation_search_space",
                self.correlation_search_space_resolution_m,
                self.correlation_search_space_smear_deviation_m,
            ),
            (
                "loop_search_space",
                self.loop_search_space_resolution_m,
                self.loop_search_space_smear_deviation_m,
            ),
        ];
        for (search, resolution_m, smear_m) in searches {
            if !(0.5 * resolution_m..=10.0 * resolution_m).contains(&smear_m) {
                return Err(format!(
                    "{search}_smear_deviation_m ({smear_m} m) must be between half and ten \
                     times {search}_resolution_m ({resolution_m} m)"
                ));
            }
        }
        Ok(())
    }

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
            loop_search_maximum_heading_difference_rad: self
                .loop_search_maximum_heading_difference_rad,
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

/// Every live-tunable parameter, each named after the [`SlamConfig`] field
/// it sets, with its `value` left at `0` - see
/// [`crate::config::refresh_parameter_values`]. Every numeric field is one;
/// the switches (`draw`, `use_response_expansion`, `do_loop_closing`) only
/// change in the file.
pub fn tunable_parameters() -> Vec<AlgorithmParameter> {
    vec![
        AlgorithmParameter::float("rate_hz", 1.0, 200.0, 1.0)
            .unit("Hz")
            .description(
                "How often SLAM polls its input topics - above the lidar's rate, so a new \
                 scan is picked up soon after it's written.",
            ),
        AlgorithmParameter::int("odometry_buffer_len", 2, 500, 1).description(
            "How many odometry samples are kept to interpolate the pose each scan was taken \
             from.",
        ),
        AlgorithmParameter::float("map_publish_period_s", 0.05, 5.0, 0.05)
            .unit("s")
            .description(
                "Shortest time between two publications of the map and its drawing - each \
                 sends the whole raster to every viewer.",
            ),
        AlgorithmParameter::float("minimum_time_interval_s", 0.0, 5.0, 0.05)
            .unit("s")
            .description("A scan is kept if at least this long passed since the last one kept..."),
        AlgorithmParameter::float("minimum_travel_distance_m", 0.0, 2.0, 0.01)
            .unit("m")
            .description("... or the vehicle moved at least this far..."),
        AlgorithmParameter::float("minimum_travel_heading_rad", 0.0, 1.57, 0.01)
            .unit("rad")
            .description("... or turned at least this much."),
        AlgorithmParameter::int("scan_buffer_size", 1, 100, 1)
            .description("How many of the most recent kept scans a new scan is matched against."),
        AlgorithmParameter::float("scan_buffer_maximum_scan_distance_m", 0.5, 50.0, 0.5)
            .unit("m")
            .description("Longest distance those scans may span."),
        AlgorithmParameter::float("correlation_search_space_dimension_m", 0.05, 2.0, 0.05)
            .unit("m")
            .description(
                "Side of the square window searched around the odometry prior - odometry \
                 must be at least this close to the truth for a match to be found.",
            ),
        AlgorithmParameter::float("correlation_search_space_resolution_m", 0.005, 0.1, 0.005)
            .unit("m")
            .description("Resolution of that search."),
        AlgorithmParameter::float(
            "correlation_search_space_smear_deviation_m",
            0.005,
            0.5,
            0.005,
        )
        .unit("m")
        .description(
            "How much reference points are blurred before matching - between half and \
                 ten times the resolution.",
        ),
        AlgorithmParameter::float("coarse_search_angle_offset_rad", 0.0, 1.571, 0.001)
            .unit("rad")
            .description("Heading searched either side of the prior."),
        AlgorithmParameter::float("coarse_angle_resolution_rad", 0.001, 0.2, 0.0001)
            .unit("rad")
            .description("Heading step of the coarse search."),
        AlgorithmParameter::float("fine_search_angle_offset_rad", 0.0001, 0.05, 0.00001)
            .unit("rad")
            .description("Heading step of the fine search."),
        AlgorithmParameter::float("distance_variance_penalty", 0.01, 5.0, 0.01).description(
            "How fast a candidate pose's score falls with its distance from the prior - \
             larger trusts odometry less.",
        ),
        AlgorithmParameter::float("angle_variance_penalty", 0.01, 5.0, 0.01).description(
            "How fast a candidate pose's score falls with its heading change from the prior - \
             larger trusts odometry less.",
        ),
        AlgorithmParameter::float("minimum_distance_penalty", 0.0, 1.0, 0.01)
            .description("Floor of the distance penalty factor."),
        AlgorithmParameter::float("minimum_angle_penalty", 0.0, 1.0, 0.01)
            .description("Floor of the heading penalty factor."),
        AlgorithmParameter::float("max_laser_range_m", 1.0, 30.0, 0.5)
            .unit("m")
            .description(
                "Readings farther than this only mark the cells they cross as free, and \
                 aren't matched against. Scans already kept keep theirs.",
            ),
        AlgorithmParameter::float("resolution_m", 0.01, 0.5, 0.01)
            .unit("m")
            .description("Side of one cell of the map - the map is rebuilt with it."),
        AlgorithmParameter::int("min_pass_through", 0, 20, 1)
            .description("How many beams must cross a cell before it's anything but unknown."),
        AlgorithmParameter::float("occupancy_threshold", 0.0, 1.0, 0.01)
            .description("Hit/pass ratio above which a cell is occupied rather than free."),
        AlgorithmParameter::float("link_match_minimum_response_fine", 0.0, 1.0, 0.01).description(
            "A scan is linked to a nearby, already linked chain only if it matches it \
                 better than this.",
        ),
        AlgorithmParameter::float("link_scan_maximum_distance_m", 0.0, 10.0, 0.1)
            .unit("m")
            .description("Farthest two scans may be apart to be linked."),
        AlgorithmParameter::float("loop_search_maximum_distance_m", 0.0, 20.0, 0.1)
            .unit("m")
            .description("Older scans closer than this are candidates for a loop closure..."),
        AlgorithmParameter::float(
            "loop_search_maximum_heading_difference_rad",
            0.0,
            std::f64::consts::PI,
            0.001,
        )
        .unit("rad")
        .description(
            "... and only if their heading is within this of the new one's - so a scan \
                 doesn't match the stretch of track behind a thin wall, driven the other way.",
        ),
        AlgorithmParameter::int("loop_match_minimum_chain_size", 1, 100, 1).description(
            "Fewest consecutive scans a chain needs to be matched against, for loop closures \
             and links to nearby chains.",
        ),
        AlgorithmParameter::float("loop_match_maximum_variance_coarse", 0.0, 5.0, 0.01)
            .unit("m²")
            .description(
                "A coarse loop match is only trusted if both its position variances are \
                 below this.",
            ),
        AlgorithmParameter::float("loop_match_minimum_response_coarse", 0.0, 1.0, 0.01)
            .description("Coarse response a loop match must beat to be refined."),
        AlgorithmParameter::float("loop_match_minimum_response_fine", 0.0, 1.0, 0.01)
            .description("Fine response a refined loop match must reach to close the loop."),
        AlgorithmParameter::float("loop_search_space_dimension_m", 0.5, 20.0, 0.5)
            .unit("m")
            .description(
                "Side of the square window a loop closure is searched in - how far off the \
                 drifted pose may be when coming back to a known place.",
            ),
        AlgorithmParameter::float("loop_search_space_resolution_m", 0.005, 0.5, 0.005)
            .unit("m")
            .description("Resolution of that search."),
        AlgorithmParameter::float("loop_search_space_smear_deviation_m", 0.005, 0.5, 0.005)
            .unit("m")
            .description(
                "How much reference points are blurred for that search - between half and \
                 ten times its resolution.",
            ),
        AlgorithmParameter::int("optimizer_max_iterations", 1, 200, 1).description(
            "Most Levenberg-Marquardt iterations one pose graph optimization may take.",
        ),
        AlgorithmParameter::float("localization_minimum_response", 0.0, 1.0, 0.01).description(
            "While localizing, a match is only trusted at or above this response - below it, \
             the pose follows odometry alone.",
        ),
    ]
}

/// Where SLAM keeps its config: `config/localization/slam.toml`, relative
/// to the working directory - rewritten by [`save_parameters`].
pub fn config_path() -> PathBuf {
    Path::new(crate::config::DEFAULT_CONFIG_ROOT)
        .join("localization")
        .join("slam.toml")
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

/// Translucent, so the true vehicle stays visible where they overlap.
const DRAWN_COLOR: Color = Color::GREEN.with_alpha(170);
/// The vehicle localized on a known map, told apart from mapping's.
const LOCALIZED_COLOR: Color = Color::BLUE.with_alpha(190);
/// The particle filter's particles: the localized vehicle's blue, fainter.
const PARTICLE_COLOR: Color = Color::BLUE.with_alpha(110);
/// Length of the tick drawn along each particle's heading, in meters.
const PARTICLE_TICK_M: f64 = 0.15;
/// Loop-closure edges stand out from the trajectory.
const LOOP_EDGE_COLOR: Color = Color::AMBER;
/// Radius of the circle marking where a loop was closed, in meters.
const LOOP_MARKER_RADIUS_M: f64 = 0.75;
/// Above [`crate::environment::MapServer`]'s map (`-100`), below everything
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
///
/// [`LocalizationCommand::algorithm`] picks the backend that localizes:
/// the scan matcher or the particle filter. Switching takes effect when not
/// [`SlamState::Localizing`], restarting from where the old backend had the
/// vehicle. [`LocalizationCommand::set_pose_requested`] restarts either
/// from a pose given on the map, and
/// [`LocalizationCommand::relocalize_requested`] the particle filter from
/// anywhere on the map.
///
/// Applies whatever [`SlamParameters`] and [`ParticleFilterParameters`] ask
/// for before its next tick, keeping the map and the pose, and reports the
/// values in effect in [`SlamStatus::parameters`] and
/// [`LocalizationStatus::particle_filter_parameters`].
pub struct Slam {
    id: u16,
    name: String,
    maps_root: PathBuf,
    config: SlamConfig,
    particle_filter_config: ParticleFilterConfig,
}

impl Slam {
    /// Creates a `Slam` that saves maps under `maps_root`.
    pub fn new(name: impl Into<String>, maps_root: impl Into<PathBuf>, config: SlamConfig) -> Self {
        Self {
            id: 0,
            name: name.into(),
            maps_root: maps_root.into(),
            config,
            particle_filter_config: ParticleFilterConfig::default(),
        }
    }

    /// Runs the particle filter, when picked, with `config` - the checked-in
    /// one otherwise.
    pub fn with_particle_filter(mut self, config: ParticleFilterConfig) -> Self {
        self.particle_filter_config = config;
        self
    }

    /// Applies any new [`ParticleFilterParameters`]. Returns whether the
    /// config changed.
    fn apply_particle_filter_parameters(
        &mut self,
        captain: &Captain,
        seen_write_count: &mut u64,
    ) -> bool {
        let Some(requests) =
            captain.try_topic::<ParticleFilterParameters>(PARTICLE_FILTER_PARAMETERS_TOPIC_NAME)
        else {
            return false;
        };
        let write_count = requests.meta().write_count;
        if write_count == *seen_write_count {
            return false;
        }
        *seen_write_count = write_count;
        let wanted = requests.read().into_value().values;
        crate::config::apply_parameters(
            &mut self.particle_filter_config,
            &particle_filter::tunable_parameters(),
            &wanted,
        )
    }

    /// The particle filter's tunable parameters with the values in effect.
    fn particle_filter_parameters(&self) -> Vec<AlgorithmParameter> {
        let mut parameters = particle_filter::tunable_parameters();
        crate::config::refresh_parameter_values(&mut parameters, &self.particle_filter_config);
        parameters
    }

    /// Applies any new [`SlamParameters`] to the config, refusing values it
    /// can't run with (see [`SlamConfig::validate`]). Returns whether the
    /// config changed.
    fn apply_parameters(&mut self, captain: &Captain, seen_write_count: &mut u64) -> bool {
        // Nothing may publish parameters at all (e.g. a binary without
        // `web_gui`) - then the config stays as loaded.
        let Some(requests) = captain.try_topic::<SlamParameters>(SLAM_PARAMETERS_TOPIC_NAME) else {
            return false;
        };
        let write_count = requests.meta().write_count;
        if write_count == *seen_write_count {
            return false;
        }
        *seen_write_count = write_count;
        let wanted = requests.read().into_value().values;
        let mut tuned = self.config;
        if !crate::config::apply_parameters(&mut tuned, &tunable_parameters(), &wanted) {
            return false;
        }
        if let Err(err) = tuned.validate() {
            eprintln!("{}: refused the new parameters: {err}", self.name);
            return false;
        }
        self.config = tuned;
        true
    }

    /// The tunable parameters with the values in effect.
    fn parameters(&self) -> Vec<AlgorithmParameter> {
        let mut parameters = tunable_parameters();
        crate::config::refresh_parameter_values(&mut parameters, &self.config);
        parameters
    }
}

impl Executor for Slam {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_writer::<SlamStatus>(SLAM_STATUS_TOPIC_NAME, self.id, SlamStatus::default);
        captain.claim_writer::<SlamMap>(SLAM_MAP_TOPIC_NAME, self.id, SlamMap::default);
        captain.claim_writer::<LocalizationStatus>(
            LOCALIZATION_STATUS_TOPIC_NAME,
            self.id,
            LocalizationStatus::default,
        );
        if self.config.draw {
            captain.claim_drawing(self.id);
        }
    }

    fn run(&mut self, captain: &Captain) {
        let lidar_topic = captain.topic::<LidarScan>(LIDAR_SCAN_TOPIC_NAME);
        let odometry_topic = captain.topic::<Odometry>(ODOMETRY_TOPIC_NAME);
        let command_topic = captain.topic::<SlamCommand>(SLAM_COMMAND_TOPIC_NAME);
        let localization_command_topic =
            captain.topic::<LocalizationCommand>(LOCALIZATION_COMMAND_TOPIC_NAME);
        let localization_status_topic =
            captain.topic::<LocalizationStatus>(LOCALIZATION_STATUS_TOPIC_NAME);
        let save_topic = captain.topic::<SlamSaveRequest>(SLAM_SAVE_TOPIC_NAME);
        let selected_map_topic = captain.topic::<SelectedMap>(MAP_TOPIC_NAME);
        let placement_topics = PlacementTopics::new(captain);
        let status_topic = captain.topic::<SlamStatus>(SLAM_STATUS_TOPIC_NAME);
        let map_topic = captain.topic::<SlamMap>(SLAM_MAP_TOPIC_NAME);
        let drawing_topic = self.config.draw.then(|| captain.drawing(self.id));
        // The size the vehicle is drawn at - the template car's if nothing
        // publishes the vehicle's.
        let geometry_topic =
            captain.try_topic::<VehicleGeometry>(&VehicleTopics::ego().vehicle_geometry());
        let geometry = || {
            geometry_topic
                .as_ref()
                .map_or_else(VehicleGeometry::default, |topic| topic.read().into_value())
        };

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
        let mut map_publish_period = Duration::from_secs_f64(self.config.map_publish_period_s);
        let mut last_published: Option<Instant> = None;
        let mut ticker = Ticker::new(self.config.rate_hz);
        let mut seen_parameters = 0;
        let mut parameters = self.parameters();
        let mut seen_particle_filter_parameters = 0;
        let mut particle_filter_parameters = self.particle_filter_parameters();
        // Requests left over from before this executor started aren't
        // applied.
        let initial_command = localization_command_topic.read().into_value();
        let mut applied_set_pose = initial_command.set_pose_requested;
        let mut applied_relocalize = initial_command.relocalize_requested;
        let mut last_effective_state = SlamState::Off;

        while captain.is_running(self.id) {
            let rate_hz = self.config.rate_hz;
            if self.apply_parameters(captain, &mut seen_parameters) {
                state.set_config(&self.config);
                map_publish_period = Duration::from_secs_f64(self.config.map_publish_period_s);
                if self.config.rate_hz != rate_hz {
                    ticker = Ticker::new(self.config.rate_hz);
                }
                parameters = self.parameters();
            }
            if self.apply_particle_filter_parameters(captain, &mut seen_particle_filter_parameters)
            {
                state.set_particle_filter_config(self.particle_filter_config);
                particle_filter_parameters = self.particle_filter_parameters();
            }
            placement.update(&placement_topics.read(), &VehicleTopics::ego());
            let odometry = odometry_topic.read();
            if last_odometry_write != Some(odometry.meta.write_count) {
                last_odometry_write = Some(odometry.meta.write_count);
                if let Some(written_at) = odometry.meta.written_at {
                    state.push_odometry(written_at, odometry.value);
                }
            }

            let command = command_topic.read().into_value();
            let clear_requested = command.clear_requested != applied_clear;
            applied_clear = command.clear_requested;
            if clear_requested || command.state == SlamState::Off {
                state.clear();
            }

            // Localization never runs over a map being built.
            let effective_state =
                if command.state.is_localization() && state.mapper.scan_count() > 0 {
                    SlamState::Waiting
                } else {
                    command.state
                };
            // The backend only changes while not localizing - judged by the
            // last tick, so a switch written together with Start (both
            // landing within one tick) still applies.
            let localization_command = localization_command_topic.read().into_value();
            if last_effective_state != SlamState::Localizing {
                state.algorithm = localization_command.algorithm;
            }
            last_effective_state = effective_state;
            if effective_state.is_localization() {
                let selected_map = selected_map_topic.read();
                state.update_localization(
                    &selected_map.value,
                    selected_map.meta.write_count,
                    &placement.anchor(),
                    self.config.localizer_params(),
                    self.particle_filter_config,
                );
            } else {
                state.stop_localizing();
            }
            if localization_command.set_pose_requested != applied_set_pose {
                applied_set_pose = localization_command.set_pose_requested;
                if let Some([x_m, y_m, heading_rad]) = localization_command.set_pose {
                    state.set_pose(Pose2::new(x_m, y_m, heading_rad));
                }
            }
            if localization_command.relocalize_requested != applied_relocalize {
                applied_relocalize = localization_command.relocalize_requested;
                state.relocalize();
            }

            let scan = lidar_topic.read();
            let new_scan = scan.meta.write_count != last_scan_write;
            last_scan_write = scan.meta.write_count;
            let processing = effective_state == SlamState::Running
                || (effective_state == SlamState::Localizing && state.localization.is_some());
            if processing {
                state.try_pending();
                if new_scan && let Some(written_at) = scan.meta.written_at {
                    state.add_scan(written_at, scan.value);
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
                .write(
                    self.id,
                    state.status(effective_state, &parameters, placement.anchor()),
                )
                .expect("lost writer authorization for the slam_status topic");

            localization_status_topic
                .write(
                    self.id,
                    state.localization_status(&particle_filter_parameters),
                )
                .expect("lost writer authorization for the localization_status topic");

            if let Some(localization) = &state.localization {
                if state.localization_changed {
                    state.localization_changed = false;
                    if let Some(drawing_topic) = &drawing_topic {
                        let particles = match &localization.backend {
                            Backend::ParticleFilter(filter) => {
                                filter.sample_particles(self.particle_filter_config.draw_particles)
                            }
                            Backend::ScanMatching(_) => Vec::new(),
                        };
                        drawing_topic
                            .write(
                                self.id,
                                localized_drawing(
                                    localization.backend.pose(),
                                    &particles,
                                    &geometry(),
                                ),
                            )
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
                                &geometry(),
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
        Box::new(
            Slam::new(self.name.clone(), self.maps_root.clone(), self.config)
                .with_particle_filter(self.particle_filter_config),
        )
    }
}

/// How [`State`] processed a scan - see [`State::try_pending`].
#[derive(Debug, Clone, Copy, PartialEq)]
struct ScanProcessing {
    /// Whether the scan was added to the map - `false` for one skipped
    /// because the vehicle barely moved, and while localizing.
    kept: bool,
    /// How well it matched - `None` for the first scan of a map, which has
    /// nothing to match against, and for a skipped one.
    response: Option<f64>,
    /// How long processing it took, in milliseconds - including any loop
    /// closure it triggered.
    process_ms: f64,
    /// Whether it closed a loop.
    loop_closed: bool,
}

/// Everything [`Slam::run`] accumulates - and [`rebuild`] too, offline: both
/// feed it through [`State::push_odometry`], [`State::add_scan`] and
/// [`State::try_pending`], so a map built offline goes through the very code
/// a live one does.
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
    /// The backend localization runs (or will) with.
    algorithm: LocalizationAlgorithm,
    /// What the particle filter said about the latest scan.
    last_estimate: Option<particle_filter::Estimate>,
    /// Whether the localized pose changed since it was last drawn.
    localization_changed: bool,
    /// Scans never processed: replaced by a newer one while waiting for
    /// odometry, or older than every odometry sample held.
    dropped_scans: usize,
}

/// A localization backend, and what it was built for: once any of it
/// changes, it's rebuilt.
struct Localization {
    backend: Backend,
    /// The write of [`MAP_TOPIC_NAME`] whose map it localizes on.
    map_write: u64,
    /// The [`Odometry::reset_count`] its starting pose was taken for.
    odometry_reset_count: Option<u64>,
}

/// The algorithm localizing - see [`LocalizationAlgorithm`].
enum Backend {
    // Both boxed: hundreds of bytes each, carried in `State` while mapping
    // too.
    ScanMatching(Box<Localizer>),
    ParticleFilter(Box<ParticleFilter>),
}

impl Backend {
    fn algorithm(&self) -> LocalizationAlgorithm {
        match self {
            Backend::ScanMatching(_) => LocalizationAlgorithm::ScanMatching,
            Backend::ParticleFilter(_) => LocalizationAlgorithm::ParticleFilter,
        }
    }

    /// The latest pose, in the map frame.
    fn pose(&self) -> Option<Pose2> {
        match self {
            Backend::ScanMatching(localizer) => localizer.pose(),
            Backend::ParticleFilter(filter) => filter.estimate().map(|estimate| estimate.pose),
        }
    }

    /// Where the `odom` frame's origin sits on the map.
    fn map_to_odom(&self) -> Option<Pose2> {
        match self {
            Backend::ScanMatching(localizer) => Some(localizer.map_to_odom()),
            Backend::ParticleFilter(filter) => filter.map_to_odom(),
        }
    }
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
            algorithm: LocalizationAlgorithm::default(),
            last_estimate: None,
            localization_changed: false,
            dropped_scans: 0,
        }
    }

    /// Adds an odometry sample, written at `at`. A reset of odometry that
    /// nothing announced (e.g. through [`SlamCommand`] by something other
    /// than web_gui) throws the map away: its frame is gone.
    fn push_odometry(&mut self, at: Instant, odometry: Odometry) {
        self.odometry.push(at, odometry);
        if let (Some(built_in), Some(current)) = (self.map_reset_count, self.odometry.reset_count())
            && built_in != current
        {
            self.clear();
        }
    }

    /// Takes a new scan, written at `at`, and processes it if odometry
    /// already covers that instant (see [`Self::try_pending`]). An older scan
    /// still waiting for odometry is dropped for it - call
    /// [`Self::try_pending`] first to give that one its chance.
    fn add_scan(&mut self, at: Instant, scan: LidarScan) -> Option<ScanProcessing> {
        if self.pending.replace((at, scan)).is_some() {
            self.dropped_scans += 1;
        }
        self.try_pending()
    }

    /// Runs with `config` from now on, keeping the map, the odometry and
    /// any localization.
    fn set_config(&mut self, config: &SlamConfig) {
        self.mapper.set_params(config.mapper_params());
        self.odometry.set_capacity(config.odometry_buffer_len);
        if let Some(Localization {
            backend: Backend::ScanMatching(localizer),
            ..
        }) = &mut self.localization
        {
            localizer.set_params(config.localizer_params());
        }
        // The grid may have been rebuilt.
        self.map_changed = true;
    }

    /// Runs the particle filter with `config` from the next scan on.
    fn set_particle_filter_config(&mut self, config: ParticleFilterConfig) {
        if let Some(Localization {
            backend: Backend::ParticleFilter(filter),
            ..
        }) = &mut self.localization
        {
            filter.set_config(config);
        }
    }

    /// Localizes on `map` (the `map_write`th write of [`MAP_TOPIC_NAME`])
    /// with [`Self::algorithm`], (re)starting from `start` - where odometry
    /// was reset - whenever the map or odometry's reset count changed, and
    /// from where the old backend had the vehicle when only the algorithm
    /// did. Stops localizing if no map is loaded.
    fn update_localization(
        &mut self,
        map: &SelectedMap,
        map_write: u64,
        start: &StartState,
        params: LocalizerParams,
        particle_filter_config: ParticleFilterConfig,
    ) {
        let odometry_reset_count = self.odometry.reset_count();
        let same_frame = self.localization.as_ref().filter(|localization| {
            localization.map_write == map_write
                && localization.odometry_reset_count == odometry_reset_count
        });
        if same_frame.is_some_and(|localization| localization.backend.algorithm() == self.algorithm)
        {
            return;
        }
        // Switching algorithm on the same map and odometry carries the pose
        // over.
        let map_to_odom = same_frame
            .and_then(|localization| localization.backend.map_to_odom())
            .unwrap_or_else(|| Pose2::new(start.x_m, start.y_m, start.heading_rad));
        let backend = match self.algorithm {
            LocalizationAlgorithm::ScanMatching => map.info.is_some().then(|| {
                Backend::ScanMatching(Box::new(Localizer::new(
                    params,
                    localizer::wall_points(map),
                    map_to_odom,
                )))
            }),
            LocalizationAlgorithm::ParticleFilter => {
                ParticleFilter::new(particle_filter_config, map, Start::Anchor(map_to_odom))
                    .map(|filter| Backend::ParticleFilter(Box::new(filter)))
            }
        };
        let rebuilt = backend.map(|backend| Localization {
            backend,
            map_write,
            odometry_reset_count,
        });
        // A scan waiting since before is for the old start or map.
        self.pending = None;
        self.last_match_response = None;
        self.last_process_ms = None;
        self.last_estimate = None;
        self.localization = rebuilt;
        self.localization_changed = true;
        // Nothing drawn left over from the old localization, if it's gone.
        self.map_changed = true;
    }

    /// Restarts localization from `pose` on the map, if localizing.
    fn set_pose(&mut self, pose: Pose2) {
        let latest_odometry = self.odometry.latest_pose();
        match self
            .localization
            .as_mut()
            .map(|localization| &mut localization.backend)
        {
            Some(Backend::ScanMatching(localizer)) => {
                if let Some(odometry) = latest_odometry {
                    localizer.set_map_to_odom(pose.compose(&odometry.inverse()));
                }
            }
            Some(Backend::ParticleFilter(filter)) => filter.restart(Start::Pose(pose)),
            None => {}
        }
    }

    /// Restarts the particle filter from anywhere on the map, if it's
    /// localizing - scan matching can't search the whole map.
    fn relocalize(&mut self) {
        if let Some(Localization {
            backend: Backend::ParticleFilter(filter),
            ..
        }) = &mut self.localization
        {
            filter.restart(Start::Global);
        }
    }

    /// Forgets the localization, if any.
    fn stop_localizing(&mut self) {
        if self.localization.take().is_some() {
            self.pending = None;
            self.last_match_response = None;
            self.last_process_ms = None;
            self.last_estimate = None;
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
    /// Returns how it was processed, if it was.
    fn try_pending(&mut self) -> Option<ScanProcessing> {
        let (written_at, _) = self.pending.as_ref()?;
        let pose = match self.odometry.pose_at(*written_at) {
            PoseAt::Pose(pose) => pose,
            PoseAt::NotYet => return None,
            PoseAt::Unavailable => {
                self.pending = None;
                self.dropped_scans += 1;
                return None;
            }
        };
        let (written_at, scan) = self.pending.take().expect("checked above");
        let started = Instant::now();

        if let Some(localization) = &mut self.localization {
            let response = match &mut localization.backend {
                Backend::ScanMatching(localizer) => {
                    let threshold_m = localizer.range_threshold_m(f64::from(scan.max_distance));
                    let localized =
                        localizer.process(LocalizedScan::new(&scan, written_at, pose, threshold_m));
                    self.last_match_response = Some(localized.response);
                    Some(localized.response)
                }
                Backend::ParticleFilter(filter) => {
                    self.last_estimate = Some(filter.process(pose, &scan));
                    None
                }
            };
            let process_ms = started.elapsed().as_secs_f64() * 1000.0;
            self.last_process_ms = Some(process_ms);
            self.localization_changed = true;
            return Some(ScanProcessing {
                kept: false,
                response,
                process_ms,
                loop_closed: false,
            });
        }

        let threshold_m = self.mapper.range_threshold_m(f64::from(scan.max_distance));
        let processed =
            self.mapper
                .process(LocalizedScan::new(&scan, written_at, pose, threshold_m));
        let process_ms = started.elapsed().as_secs_f64() * 1000.0;
        let (response, loop_closed) = match processed {
            Processed::Skipped => {
                return Some(ScanProcessing {
                    kept: false,
                    response: None,
                    process_ms,
                    loop_closed: false,
                });
            }
            Processed::First => (None, false),
            Processed::Matched {
                result,
                loop_closed,
            } => (Some(result.response), loop_closed),
        };
        self.last_match_response = response;
        self.last_process_ms = Some(process_ms);
        self.map_reset_count = self.odometry.reset_count();
        self.map_changed = true;
        Some(ScanProcessing {
            kept: true,
            response,
            process_ms,
            loop_closed,
        })
    }

    fn status(
        &self,
        state: SlamState,
        parameters: &[AlgorithmParameter],
        anchor: StartState,
    ) -> SlamStatus {
        let as_array = |pose: Pose2| [pose.x_m, pose.y_m, pose.heading_rad];
        let (pose, map_to_odom) = match &self.localization {
            Some(localization) => (
                localization.backend.pose(),
                localization.backend.map_to_odom(),
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
            parameters: parameters.to_vec(),
            anchor,
        }
    }

    fn localization_status(
        &self,
        particle_filter_parameters: &[AlgorithmParameter],
    ) -> LocalizationStatus {
        let estimate = self.last_estimate;
        let quality = match self.algorithm {
            LocalizationAlgorithm::ScanMatching => self.last_match_response,
            LocalizationAlgorithm::ParticleFilter => estimate.map(|estimate| estimate.spread_m),
        };
        let localization = self.localization.as_ref();
        LocalizationStatus {
            algorithm: localization.map_or(self.algorithm, |localization| {
                localization.backend.algorithm()
            }),
            active: localization.is_some(),
            last_process_ms: localization.and(self.last_process_ms),
            quality: localization.and(quality),
            particle_spread_rad: estimate.map(|estimate| estimate.spread_rad),
            effective_sample_size: estimate.map(|estimate| estimate.effective_sample_size),
            particles: estimate.map(|estimate| estimate.particles),
            relocalizing: localization.is_some_and(|localization| {
                matches!(&localization.backend, Backend::ParticleFilter(filter) if filter.relocalizing())
            }),
            particle_filter_parameters: particle_filter_parameters.to_vec(),
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

/// What [`Slam`] draws: `map`, its trajectory, and a vehicle the size of
/// `geometry` at `pose`, with SLAM's frame placed at `anchor` - where dead reckoning (whose
/// `odom` frame SLAM's frame is) was last reset - so it all overlays the
/// true map. Empty when the map is.
fn drawing(
    map: &SlamMap,
    pose: Option<Pose2>,
    loop_edges: &[(Pose2, Pose2)],
    anchor: &StartState,
    geometry: &VehicleGeometry,
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
        length_m: geometry.body_length_m,
        width_m: geometry.body_width_m,
        front_axle_m: geometry.lf_m(),
        rear_axle_m: geometry.lr_m(),
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

/// What [`Slam`] draws while localizing: a vehicle the size of `geometry` at
/// `pose`, already in the map's frame, and the particle filter's
/// `particles`, if any. Empty before the first scan.
fn localized_drawing(
    pose: Option<Pose2>,
    particles: &[Pose2],
    geometry: &VehicleGeometry,
) -> Drawing {
    let vehicle = pose.map(|pose| Shape::Vehicle {
        x_m: pose.x_m,
        y_m: pose.y_m,
        heading_rad: pose.heading_rad,
        // Redrawn on every scan only: don't extrapolate in between.
        speed_mps: 0.0,
        steering_rad: 0.0,
        length_m: geometry.body_length_m,
        width_m: geometry.body_width_m,
        front_axle_m: geometry.lf_m(),
        rear_axle_m: geometry.lr_m(),
        color: LOCALIZED_COLOR,
    });
    let mut drawing = Drawing::default();
    if !particles.is_empty() {
        // A dot at each particle, and a short tick along its heading.
        let points = particles
            .iter()
            .map(|particle| [particle.x_m as f32, particle.y_m as f32])
            .collect();
        let ticks = particles.iter().map(|particle| {
            let (x, y) = particle.transform_point(PARTICLE_TICK_M, 0.0);
            Shape::Polyline {
                points: vec![
                    [particle.x_m as f32, particle.y_m as f32],
                    [x as f32, y as f32],
                ],
                closed: false,
                width_px: 1.0,
                color: PARTICLE_COLOR,
            }
        });
        drawing = drawing.element(
            "Particles",
            std::iter::once(Shape::Points {
                points,
                radius_px: 2.0,
                color: PARTICLE_COLOR,
            })
            .chain(ticks),
            true,
        );
    }
    drawing
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
        assert_eq!(config.validate(), Ok(()));
    }

    /// Every declared parameter names a numeric config field - this panics
    /// otherwise - and the shipped defaults sit inside their slider ranges.
    #[test]
    fn every_parameter_names_a_config_field_within_its_range() {
        let mut parameters = tunable_parameters();
        crate::config::refresh_parameter_values(&mut parameters, &SlamConfig::default());
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
    fn a_smear_out_of_its_resolutions_bounds_is_invalid() {
        let config = SlamConfig {
            correlation_search_space_resolution_m: 0.1,
            correlation_search_space_smear_deviation_m: 0.01,
            ..SlamConfig::default()
        };
        assert!(config.validate().is_err());
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
        let drawing = drawing(
            &SlamMap::default(),
            None,
            &[],
            &StartState::default(),
            &VehicleGeometry::default(),
        );
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
        let drawing = drawing(
            &tiny_map(),
            Some(Pose2::default()),
            &[edge],
            &anchor,
            &VehicleGeometry::default(),
        );
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
        let drawing = drawing(
            &tiny_map(),
            Some(Pose2::new(1.0, 0.0, 0.2)),
            &[],
            &anchor,
            &VehicleGeometry::default(),
        );
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
