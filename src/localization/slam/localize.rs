//! [`localize`]: localizes a recorded drive again, offline, with either
//! backend - for [`crate::localization::evaluate`] to compare them on the
//! very same drive.
//!
//! The drive is replayed as [`super::rebuild`] replays a mapping session:
//! odometry and scans in the order they were written, through the same
//! [`State`] steps [`super::Slam`] takes live ([`State::push_odometry`],
//! [`State::try_pending`], [`State::add_scan`]), as fast as the CPU allows.
//!
//! What's kept is the pose a driving algorithm would have read - odometry
//! composed onto the latest `map_to_odom`, at every odometry sample - with
//! each correction applied only once the scan it came from would have been
//! processed live: its processing time after it was taken. A slower
//! algorithm's estimate lags, as it would on the car.

use super::pose::Pose2;
use super::{SlamConfig, State};
use crate::localization::ParticleFilterConfig;
use crate::localization::session::Timed;
use crate::topics::{LidarScan, LocalizationAlgorithm, Odometry, SelectedMap, StartState};
use std::collections::VecDeque;
use std::ops::ControlFlow;
use std::time::{Duration, Instant};

/// How many scans go by between two calls to [`localize`]'s `progress`.
const PROGRESS_EVERY_SCANS: usize = 20;

/// One scan [`localize`] processed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct LocalizedStep {
    /// When it was taken, in microseconds since the session started.
    pub t_us: u64,
    /// How long the backend took on it, in milliseconds.
    pub process_ms: f64,
    /// How confident the backend is, in its own terms: scan matching's
    /// response, the particle filter's position spread in meters.
    pub quality: Option<f64>,
    /// The pose it put the vehicle at when the scan was taken.
    pub pose: Pose2,
}

/// What [`localize`] made of a drive.
#[derive(Debug, Clone, Default)]
pub(crate) struct Localized {
    /// The pose a driving algorithm would have read at every odometry
    /// sample, oldest first.
    pub track: Vec<Timed<Pose2>>,
    pub steps: Vec<LocalizedStep>,
    /// Scans never processed: odometry didn't cover the instant they were
    /// taken at.
    pub dropped_scans: usize,
}

/// One recorded write, in the order [`localize`] replays them.
enum Event<'a> {
    Odometry(Odometry),
    Scan(&'a LidarScan),
}

/// Localizes the drive made of `odometry` and `scans` on `map` with
/// `algorithm`, starting with odometry's frame at `map_to_odom` on the map -
/// see the module docs. `None` for `algorithm` dead-reckons alone: the
/// track is odometry placed at `map_to_odom`, and nothing corrects it.
///
/// `progress` is called every few scans with how far it got, from `0` to
/// `1`; [`ControlFlow::Break`] stops it, and `Ok(None)` is returned. Fails
/// if the backend can't localize on `map`.
#[allow(clippy::too_many_arguments)]
pub(crate) fn localize(
    odometry: &[Timed<Odometry>],
    scans: &[Timed<LidarScan>],
    map: &SelectedMap,
    algorithm: Option<LocalizationAlgorithm>,
    config: &SlamConfig,
    particle_filter: ParticleFilterConfig,
    map_to_odom: Pose2,
    mut progress: impl FnMut(f64) -> ControlFlow<()>,
) -> Result<Option<Localized>, String> {
    let started = Instant::now();
    let at = |t_us: u64| started + Duration::from_micros(t_us);
    let since_start = |time: Instant| time.duration_since(started).as_micros() as u64;

    let mut events: Vec<(u64, Event)> = Vec::with_capacity(odometry.len() + scans.len());
    events.extend(
        odometry
            .iter()
            .map(|&(t, odometry)| (t, Event::Odometry(odometry))),
    );
    if algorithm.is_some() {
        events.extend(scans.iter().map(|(t, scan)| (*t, Event::Scan(scan))));
    }
    // Stable, so each topic's own writes stay in the order they were made;
    // odometry first on a tie, as a live tick takes it.
    events.sort_by_key(|(t, event)| (*t, matches!(event, Event::Scan(_))));

    let mut state = State::new(config);
    if let Some(algorithm) = algorithm {
        state.algorithm = algorithm;
    }
    let mut out = Localized::default();
    // The `map_to_odom` a driving algorithm reads, and the corrections not
    // available to it yet - each with the instant it becomes available.
    let mut applied = map_to_odom;
    let mut upcoming: VecDeque<(u64, Pose2)> = VecDeque::new();
    let mut scans_seen = 0;

    // Takes how the latest scan was processed, if it was, at `now_us`.
    let record = |state: &mut State,
                  processed: Option<super::ScanProcessing>,
                  taken_us: u64,
                  now_us: u64,
                  out: &mut Localized,
                  upcoming: &mut VecDeque<(u64, Pose2)>| {
        let Some(processed) = processed else {
            return;
        };
        let Some(localization) = &state.localization else {
            return;
        };
        let (Some(pose), Some(map_to_odom)) = (
            localization.backend.pose(),
            localization.backend.map_to_odom(),
        ) else {
            return;
        };
        let quality = processed
            .response
            .or(state.last_estimate.map(|estimate| estimate.spread_m));
        out.steps.push(LocalizedStep {
            t_us: taken_us,
            process_ms: processed.process_ms,
            quality,
            pose,
        });
        let available_us = now_us + (processed.process_ms * 1000.0).round() as u64;
        upcoming.push_back((available_us, map_to_odom));
    };
    let pending_t_us = |state: &State| {
        state
            .pending
            .as_ref()
            .map_or(0, |(written_at, _)| since_start(*written_at))
    };

    for (t_us, event) in events {
        match event {
            Event::Odometry(sample) => {
                state.push_odometry(at(t_us), sample);
                if algorithm.is_some() && state.localization.is_none() {
                    let start = StartState {
                        x_m: map_to_odom.x_m,
                        y_m: map_to_odom.y_m,
                        heading_rad: map_to_odom.heading_rad,
                        speed_mps: 0.0,
                    };
                    state.update_localization(
                        map,
                        1,
                        &start,
                        config.localizer_params(),
                        particle_filter,
                    );
                    if state.localization.is_none() {
                        return Err(format!(
                            "{} can't localize on this map",
                            algorithm.map_or("", LocalizationAlgorithm::api_str)
                        ));
                    }
                }
                if state.localization.is_some() {
                    let taken_us = pending_t_us(&state);
                    let processed = state.try_pending();
                    record(
                        &mut state,
                        processed,
                        taken_us,
                        t_us,
                        &mut out,
                        &mut upcoming,
                    );
                }
                while let Some(&(available_us, correction)) = upcoming.front()
                    && available_us <= t_us
                {
                    applied = correction;
                    upcoming.pop_front();
                }
                out.track
                    .push((t_us, applied.compose(&Pose2::from_odometry(&sample))));
            }
            Event::Scan(scan) => {
                scans_seen += 1;
                if state.localization.is_none() {
                    // Before the first odometry sample: nothing to place it.
                    out.dropped_scans += 1;
                    continue;
                }
                let taken_us = pending_t_us(&state);
                let processed = state.try_pending();
                record(
                    &mut state,
                    processed,
                    taken_us,
                    t_us,
                    &mut out,
                    &mut upcoming,
                );
                let processed = state.add_scan(at(t_us), scan.clone());
                record(&mut state, processed, t_us, t_us, &mut out, &mut upcoming);
                if scans_seen % PROGRESS_EVERY_SCANS == 0
                    && progress(scans_seen as f64 / scans.len() as f64).is_break()
                {
                    return Ok(None);
                }
            }
        }
    }
    // A scan still waiting at the end is one odometry never covered.
    if state.pending.take().is_some() {
        state.dropped_scans += 1;
    }
    out.dropped_scans += state.dropped_scans;
    Ok(Some(out))
}
