//! [`rebuild`]: builds a map again, offline, from a recorded
//! [`MappingSession`] - with other parameters than it was recorded with, to
//! compare the results on the very same drive.
//!
//! The session's writes are replayed in the order they were written,
//! through the same [`State`] steps [`super::Slam`] takes live
//! ([`State::push_odometry`], [`State::try_pending`], [`State::add_scan`]),
//! as if it polled infinitely fast: every scan is taken, none missed, and
//! with no [`crate::Ticker`] to wait on it runs as fast as the CPU allows.
//! Recorded timestamps become [`Instant`]s by adding them to the instant the
//! rebuild started, which is all the odometry interpolation needs.

use super::pose::Pose2;
use super::{SlamConfig, State, save_map};
use crate::localization::mapping_session::MappingSession;
use crate::topics::{LidarScan, Odometry, SlamCommand, SlamMap, SlamState};
use std::ops::ControlFlow;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

/// How many scans go by between two calls to [`rebuild`]'s `progress`.
const PROGRESS_EVERY_SCANS: usize = 20;

/// One scan the map was built from.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct KeptScan {
    /// When it was written, in microseconds since the session started.
    pub t_us: u64,
    /// Its pose `(x_m, y_m, heading_rad)` in SLAM's frame (see [`SlamMap`]),
    /// once every scan matched and every loop closed.
    pub pose: [f64; 3],
    /// Where odometry alone put it, in the same frame.
    pub odometric_pose: [f64; 3],
}

/// One scan [`rebuild`] processed, kept or not.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct ProcessedScan {
    /// When it was written, in microseconds since the session started.
    pub t_us: u64,
    /// Whether it was added to the map - `false` if the vehicle had barely
    /// moved since the last one kept.
    pub kept: bool,
    /// How well it matched, from `0` to `1` - `None` for the first scan of
    /// a map, and for one not kept.
    pub response: Option<f64>,
    /// How long processing it took, in milliseconds - including any loop
    /// closure it triggered.
    pub process_ms: f64,
    pub loop_closed: bool,
}

/// What [`rebuild`] built.
#[derive(Debug, Clone)]
pub struct RebuildOutcome {
    /// The map at the end of the session, in SLAM's frame - placed in the
    /// world by [`MappingSession::anchor`].
    pub map: SlamMap,
    /// Every scan the map is built from, oldest first.
    pub kept: Vec<KeptScan>,
    /// Every scan processed, oldest first.
    pub processed: Vec<ProcessedScan>,
    /// Each loop closure's edge, from the older scan to the newer one, as
    /// poses `(x_m, y_m, heading_rad)` in SLAM's frame.
    pub loop_edges: Vec<([f64; 3], [f64; 3])>,
    pub loop_closures: usize,
    /// Scans never processed: odometry didn't cover the instant they were
    /// taken at.
    pub dropped_scans: usize,
    /// Scans recorded while mapping was paused, and ignored as live.
    pub paused_scans: usize,
    /// How long the rebuild took.
    pub elapsed: Duration,
    /// The first kept scan's pose, which a saved map's start line goes
    /// through.
    first_pose: Option<Pose2>,
}

impl RebuildOutcome {
    /// Saves the map as the map folder `name` under `maps_root`, as
    /// **Save map** does live - the folder, or why it couldn't be saved.
    pub fn save(&self, name: &str, maps_root: &Path) -> Result<PathBuf, String> {
        save_map(&self.map, self.first_pose, name, maps_root)
    }
}

/// One recorded write, in the order [`rebuild`] replays them.
enum Event<'a> {
    Command(&'a SlamCommand),
    Odometry(Odometry),
    Scan(&'a LidarScan),
}

impl Event<'_> {
    /// Breaks ties between writes stamped the same microsecond the way a
    /// live tick takes them: the command first, then odometry, then the scan.
    fn order(&self) -> u8 {
        match self {
            Self::Command(_) => 0,
            Self::Odometry(_) => 1,
            Self::Scan(_) => 2,
        }
    }
}

/// Builds `session`'s map again with `config` - see the module docs. Scans
/// are only taken while the recorded [`SlamCommand`] says
/// [`SlamState::Running`] (throughout, for a session with none). A command
/// clearing the map ends the rebuild there, with the map built so far: a
/// session's recording stops when the map is cleared, so that's the Clear
/// that ended it, caught just before the recording stopped.
///
/// `progress` is called every few scans with how far the rebuild got, from
/// `0` to `1`; [`ControlFlow::Break`] stops it, and `None` is returned.
pub fn rebuild(
    session: &MappingSession,
    config: &SlamConfig,
    mut progress: impl FnMut(f64) -> ControlFlow<()>,
) -> Option<RebuildOutcome> {
    let started = Instant::now();
    let at = |t_us: u64| started + Duration::from_micros(t_us);

    let mut events: Vec<(u64, Event)> = Vec::with_capacity(
        session.slam_commands.len() + session.odometry.len() + session.scans.len(),
    );
    events.extend(
        session
            .slam_commands
            .iter()
            .map(|(t, command)| (*t, Event::Command(command))),
    );
    events.extend(
        session
            .odometry
            .iter()
            .map(|&(t, odometry)| (t, Event::Odometry(odometry))),
    );
    events.extend(
        session
            .scans
            .iter()
            .map(|(t, scan)| (*t, Event::Scan(scan))),
    );
    // Stable, so each topic's own writes stay in the order they were made.
    events.sort_by_key(|(t, event)| (*t, event.order()));

    let mut state = State::new(config);
    let mut running = session.slam_commands.is_empty();
    let mut applied_clear = session
        .slam_commands
        .first()
        .map(|(_, command)| command.clear_requested);
    let mut processed = Vec::new();
    let mut paused_scans = 0;
    let mut scans_seen = 0;
    let mut record = |t_us: u64, processing: Option<super::ScanProcessing>| {
        if let Some(processing) = processing {
            processed.push(ProcessedScan {
                t_us,
                kept: processing.kept,
                response: processing.response,
                process_ms: processing.process_ms,
                loop_closed: processing.loop_closed,
            });
        }
    };

    for (t_us, event) in events {
        match event {
            Event::Command(command) => {
                let cleared = applied_clear != Some(command.clear_requested);
                applied_clear = Some(command.clear_requested);
                if cleared || command.state == SlamState::Off {
                    break;
                }
                running = command.state == SlamState::Running;
                if !running {
                    state.pending = None;
                }
            }
            Event::Odometry(odometry) => {
                state.push_odometry(at(t_us), odometry);
                if running {
                    let pending_t_us = pending_t_us(&state, started);
                    record(pending_t_us, state.try_pending());
                }
            }
            Event::Scan(scan) => {
                scans_seen += 1;
                if !running {
                    paused_scans += 1;
                    continue;
                }
                let pending_t_us = pending_t_us(&state, started);
                record(pending_t_us, state.try_pending());
                record(t_us, state.add_scan(at(t_us), scan.clone()));
                if scans_seen % PROGRESS_EVERY_SCANS == 0 {
                    let done = scans_seen as f64 / session.scans.len() as f64;
                    if progress(done).is_break() {
                        return None;
                    }
                }
            }
        }
    }
    // A scan still waiting at the end is one odometry never covered.
    if state.pending.take().is_some() {
        state.dropped_scans += 1;
    }

    let as_array = |pose: Pose2| [pose.x_m, pose.y_m, pose.heading_rad];
    let since_start = |time: Instant| time.duration_since(started).as_micros() as u64;
    let kept = state
        .mapper
        .scans()
        .iter()
        .map(|scan| KeptScan {
            t_us: since_start(scan.time),
            pose: as_array(scan.corrected_pose()),
            odometric_pose: as_array(scan.odometric_pose),
        })
        .collect();
    Some(RebuildOutcome {
        map: state.mapper.map(),
        kept,
        processed,
        loop_edges: state
            .mapper
            .loop_edges()
            .into_iter()
            .map(|(from, to)| (as_array(from), as_array(to)))
            .collect(),
        loop_closures: state.mapper.loop_closures(),
        dropped_scans: state.dropped_scans,
        paused_scans,
        elapsed: started.elapsed(),
        first_pose: state.mapper.first_pose(),
    })
}

/// When the scan waiting for odometry was written, in microseconds since
/// `started` - `0` if none is.
fn pending_t_us(state: &State, started: Instant) -> u64 {
    state.pending.as_ref().map_or(0, |(written_at, _)| {
        written_at.duration_since(started).as_micros() as u64
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::localization::mapping_session::MappingSessionInfo;
    use crate::localization::slam::scan_matcher::tests::room_scan;

    /// A session driving `steps` 5 cm steps along +x through the test room,
    /// a scan every 25 ms between odometry at 100 Hz, odometry drifting
    /// sideways so matching has something to correct.
    fn driving_session(steps: usize) -> MappingSession {
        let mut session = MappingSession {
            info: MappingSessionInfo::default(),
            ..MappingSession::default()
        };
        let true_pose = |t_us: u64| Pose2::new(-1.0 + t_us as f64 * 2e-6, 0.0, 0.0);
        let end_us = steps as u64 * 25_000;
        for t_us in (0..=end_us + 10_000).step_by(10_000) {
            let pose = true_pose(t_us);
            session.odometry.push((
                t_us,
                Odometry {
                    x_m: pose.x_m,
                    y_m: pose.y_m + t_us as f64 * 1e-7,
                    heading_rad: pose.heading_rad,
                    ..Odometry::default()
                },
            ));
        }
        for t_us in (0..=end_us).step_by(25_000) {
            session.scans.push((t_us, room_scan(true_pose(t_us), 360)));
        }
        session.duration_us = end_us + 10_000;
        session
    }

    fn quick_config() -> SlamConfig {
        SlamConfig {
            minimum_time_interval_s: 0.1,
            ..SlamConfig::default()
        }
    }

    fn keep_going(_: f64) -> ControlFlow<()> {
        ControlFlow::Continue(())
    }

    #[test]
    fn a_session_rebuilds_into_a_map() {
        let outcome = rebuild(&driving_session(40), &quick_config(), keep_going).unwrap();
        assert!(outcome.kept.len() > 5, "kept {}", outcome.kept.len());
        assert!(outcome.map.width_px > 0);
        assert_eq!(outcome.dropped_scans, 0);
        assert_eq!(outcome.paused_scans, 0);
        // Matching against the room undoes odometry's sideways drift.
        let last = outcome.kept.last().unwrap();
        assert!(last.pose[1].abs() < last.odometric_pose[1].abs());
    }

    #[test]
    fn rebuilding_twice_gives_the_same_map() {
        let session = driving_session(30);
        let first = rebuild(&session, &quick_config(), keep_going).unwrap();
        let second = rebuild(&session, &quick_config(), keep_going).unwrap();
        assert_eq!(first.map, second.map);
        assert_eq!(first.kept, second.kept);
    }

    #[test]
    fn scans_are_ignored_while_paused() {
        let mut session = driving_session(40);
        let running = SlamCommand {
            state: SlamState::Running,
            ..SlamCommand::default()
        };
        let paused = SlamCommand {
            state: SlamState::Waiting,
            ..running
        };
        session.slam_commands = vec![(0, running), (500_000, paused), (800_000, running)];
        let outcome = rebuild(&session, &quick_config(), keep_going).unwrap();
        // Scans at 0.5, 0.525, ... 0.775 s.
        assert_eq!(outcome.paused_scans, 12);
        assert!(
            outcome
                .processed
                .iter()
                .all(|scan| !(500_000..800_000).contains(&scan.t_us))
        );
    }

    #[test]
    fn the_clear_ending_a_session_keeps_the_map_built_until_then() {
        let mut session = driving_session(40);
        let running = SlamCommand {
            state: SlamState::Running,
            ..SlamCommand::default()
        };
        let cleared = SlamCommand {
            state: SlamState::Off,
            clear_requested: 1,
        };
        session.slam_commands = vec![(0, running), (600_000, cleared)];
        let outcome = rebuild(&session, &quick_config(), keep_going).unwrap();
        assert!(!outcome.kept.is_empty());
        assert!(outcome.processed.iter().all(|scan| scan.t_us < 600_000));
        assert!(outcome.map.width_px > 0);
    }

    #[test]
    fn progress_can_stop_it() {
        let mut calls = 0;
        let outcome = rebuild(&driving_session(60), &quick_config(), |done| {
            calls += 1;
            assert!((0.0..=1.0).contains(&done));
            ControlFlow::Break(())
        });
        assert!(outcome.is_none());
        assert_eq!(calls, 1);
    }
}
