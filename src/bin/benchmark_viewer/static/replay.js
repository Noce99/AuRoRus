"use strict";

// ---------------------------------------------------------------------
// benchmark_viewer's frontend, part 2: replaying the selected runs on the
// map - one track at a time (runs on the exact same map files), each run
// a ghost vehicle in its color at the playback time, with its trail - and
// the transport bar driving the shared `PlaybackClock` (/playback_clock.js).
//
// Nothing here fetches drawings from a server: every frame's shapes are
// built from the recorded trajectories, in the drawing protocol's shape
// format, and handed to `MapView` directly. charts.js reads `Replay`'s
// state and sets its hover markers.
// ---------------------------------------------------------------------

/** How long the bright tail of a trail is, in seconds. */
const TRAIL_RECENT_S = 3;
/** The full path is drawn from every n-th sample - 10 Hz of the 40 Hz. */
const TRAIL_FULL_STRIDE = 4;
const LABEL_INK = { r: 16, g: 20, b: 24, a: 255 };
const CENTERLINE_COLOR = { r: 150, g: 150, b: 150, a: 200 };

function hexColor(hex, alpha = 255) {
  const value = parseInt(hex.slice(1), 16);
  return { r: (value >> 16) & 255, g: (value >> 8) & 255, b: value & 255, a: alpha };
}

window.Replay = {
  /** The track shown: a `trackKey`, or null with nothing selected. */
  track: null,
  /** `{info, offscreen, name}` of `track`, once loaded. */
  map: null,
  /** The selected runs on `track`, each `{...Viewer.selected() entry,
   *  trajectory, lines, starts, end, trail, axles}` once loaded. */
  runs: [],
  /** The lap the charts show, and "Align: lap start" lines up. */
  lap: 1,
  /** "go" or "lap". */
  align: "go",
  /** Run id the view follows, or null. */
  follow: null,
  /** `[{x, y, color}]` charts.js marks on the map (its hover). */
  hoverMarkers: [],

  /** Where `run`'s own clock is at the playback time. */
  runTime(run) {
    return window.PlaybackClock.currentTimeUs / 1e6 + this.offset(run);
  },

  /** How far ahead of the playback time `run`'s own clock is: `0` from
   *  the "Go!", or the start of lap `lap` when lined up on it (a run that
   *  never reached it is parked at its end). */
  offset(run) {
    if (this.align !== "lap") return 0;
    return run.starts[this.lap] ?? run.end;
  },

  setHoverMarkers(markers) {
    this.hoverMarkers = markers;
    MapView.requestRedraw();
  },
};

const playPauseBtn = document.getElementById("play-pause-btn");
const seekSliderEl = document.getElementById("seek-slider");
const playbackTimeLabelEl = document.getElementById("playback-time-label");
const lapSelectEl = document.getElementById("lap-select");
const alignSelectEl = document.getElementById("align-select");
const followSelectEl = document.getElementById("follow-select");
const trackRowEl = document.getElementById("track-row");
const trackSelectEl = document.getElementById("track-select");
const statusTimeEl = document.getElementById("status-time");

function formatClock(seconds) {
  const sign = seconds < 0 ? "-" : "";
  const s = Math.abs(seconds);
  const minutes = Math.floor(s / 60);
  return `${sign}${minutes}:${(s - 60 * minutes).toFixed(1).padStart(4, "0")}`;
}

// ---------------------------------------------------------------------
// Following the selection
// ---------------------------------------------------------------------

/** Bumped on every reload, so a slow fetch for an older selection is
 *  dropped. */
let loadGeneration = 0;

async function loadTrackMap(run) {
  const base = `/api/runs/${run.id}`;
  const [info, raster] = await Promise.all([
    fetchJSON(`${base}/map_info`),
    fetch(`${base}/map_raster`).then((response) => {
      if (!response.ok) throw new Error(`map raster: ${response.status}`);
      return response.arrayBuffer();
    }),
  ]);
  return {
    info,
    name: run.summary.map.name,
    offscreen: MapView.offscreenFromRaster(new Uint8Array(raster), info.width_px, info.height_px),
  };
}

function prepareRun(run, { trajectory, lines }) {
  const t = trajectory.t_s;
  const trail = [];
  for (let i = 0; i < t.length; i += TRAIL_FULL_STRIDE) trail.push([trajectory.x_m[i], trajectory.y_m[i]]);
  const params = run.summary.vehicle.parameters;
  const length = run.summary.vehicle.body_length_m;
  return {
    ...run,
    trajectory,
    lines,
    starts: TrackMath.lapStarts(trajectory),
    end: t.length ? t[t.length - 1] : 0,
    trail,
    axles: { front: params.lf_m ?? 0.4 * length, rear: params.lr_m ?? 0.4 * length },
  };
}

async function reload() {
  const generation = ++loadGeneration;
  const selected = Viewer.selected();
  const tracks = [...new Set(selected.map((run) => trackKey(run.summary)))];
  const previousTrack = Replay.track;
  if (!tracks.includes(Replay.track)) Replay.track = tracks[0] ?? null;

  // The track picker, when the selection spans several.
  trackRowEl.hidden = tracks.length < 2;
  trackSelectEl.replaceChildren(
    ...tracks.map((track) => {
      const run = selected.find((r) => trackKey(r.summary) === track);
      const count = selected.filter((r) => trackKey(r.summary) === track).length;
      return el("option", { value: track, text: `${trackLabel(track)} (${count} run${count === 1 ? "" : "s"})` });
    }),
  );
  if (Replay.track) trackSelectEl.value = Replay.track;

  const onTrack = selected.filter((run) => trackKey(run.summary) === Replay.track);
  let map = Replay.track === previousTrack ? Replay.map : null;
  let runs;
  try {
    if (!map && onTrack.length) map = await loadTrackMap(onTrack[0]);
    const data = await Promise.all(onTrack.map((run) => Viewer.data(run.id)));
    runs = onTrack.map((run, i) => prepareRun(run, data[i]));
  } catch (err) {
    console.error(err);
    MapView.setText(document.getElementById("status-map-name"), `Couldn't load the runs: ${err.message}`);
    return;
  }
  if (generation !== loadGeneration) return;

  const newMap = map !== Replay.map;
  Replay.map = map;
  Replay.runs = runs;
  if (!runs.some((run) => run.id === Replay.follow)) Replay.follow = null;
  syncControls();
  updateDuration();
  if (newMap && map) {
    // A new track: the whole of it in view.
    const bounds = worldBounds();
    MapView.home();
    MapView.view.verticalSizeM = 1.05 * (bounds.maxY - bounds.minY);
  }
  MapView.requestRedraw();
  window.dispatchEvent(new CustomEvent("viewer:replay"));
}

/** The lap, follow and speed pickers, for the runs on the track. */
function syncControls() {
  const laps = Math.max(1, ...Replay.runs.map((run) => run.starts.length - 1));
  const previousLap = Replay.lap;
  lapSelectEl.replaceChildren(...[...Array(laps).keys()].map((i) => el("option", { value: i + 1, text: `${i + 1}` })));
  Replay.lap = Math.min(previousLap, laps);
  lapSelectEl.value = Replay.lap;

  followSelectEl.replaceChildren(
    el("option", { value: "", text: "none" }),
    ...Replay.runs.map((run) => el("option", { value: run.id, text: `${run.label} ${run.summary.algorithm.name}` })),
  );
  followSelectEl.value = Replay.follow ?? "";
}

/** The playback covers the longest run, from where the alignment starts
 *  each one. */
function updateDuration() {
  const clock = window.PlaybackClock;
  const lengths = Replay.runs.map((run) => Math.max(0, run.end - Replay.offset(run)));
  clock.durationUs = Math.max(0, ...lengths) * 1e6;
  clock.setTime(Math.min(clock.currentTimeUs, clock.durationUs));
}

window.addEventListener("viewer:selection", () => reload());

// ---------------------------------------------------------------------
// The layers - built fresh every frame from the playback time
// ---------------------------------------------------------------------

function layersAt() {
  const layers = [];
  const map = Replay.map;
  const layerSwitches = Viewer.layers;
  if (!map) return layers;

  const shapes = [];
  if (layerSwitches.raster) {
    shapes.push({
      raster: {
        origin_x_m: map.info.origin.x,
        origin_y_m: map.info.origin.y,
        resolution_m_per_px: map.info.resolution_m_per_px,
        width_px: map.info.width_px,
        height_px: map.info.height_px,
        offscreen: map.offscreen,
      },
    });
  }
  const centerline = Replay.runs.find((run) => run.lines.centerline)?.lines.centerline;
  if (layerSwitches.centerline && centerline) {
    shapes.push({ polyline: { points: centerline.map(([x, y]) => [x, y]), closed: true, width_px: 1, color: CENTERLINE_COLOR } });
  }
  if (layerSwitches.raceLines) {
    // A line several runs share is drawn once, in the first one's color.
    const drawn = new Set();
    for (const run of Replay.runs) {
      if (!run.summary.race_line.used_by_algorithm || drawn.has(run.summary.race_line.sha256)) continue;
      drawn.add(run.summary.race_line.sha256);
      shapes.push({ polyline: { points: run.lines.race_line.map(([x, y]) => [x, y]), closed: true, width_px: 1, color: hexColor(run.color, 150) } });
    }
  }
  layers.push({ opacity: 1, shapes });

  for (const run of Replay.runs) {
    const t = Replay.runTime(run);
    const pose = TrackMath.poseAt(run.trajectory, t);
    if (!pose) continue;
    const runShapes = [];
    if (layerSwitches.trails === "full") {
      runShapes.push({ polyline: { points: run.trail, closed: false, width_px: 1.5, color: hexColor(run.color, 90) } });
    }
    if (layerSwitches.trails !== "off") {
      const times = run.trajectory.t_s;
      const from = Math.max(0, TrackMath.lastAtOrBefore(times, t - TRAIL_RECENT_S));
      const to = TrackMath.lastAtOrBefore(times, t);
      const points = [];
      for (let i = from; i <= to; i++) points.push([run.trajectory.x_m[i], run.trajectory.y_m[i]]);
      if (!pose.ended) points.push([pose.x, pose.y]);
      runShapes.push({ polyline: { points, closed: false, width_px: 3, color: hexColor(run.color) } });
    }
    if (layerSwitches.ghosts) {
      runShapes.push({
        vehicle: {
          x_m: pose.x,
          y_m: pose.y,
          heading_rad: pose.heading,
          speed_mps: pose.speed,
          steering_rad: pose.steering,
          length_m: run.summary.vehicle.body_length_m,
          width_m: run.summary.vehicle.body_width_m,
          front_axle_m: run.axles.front,
          rear_axle_m: run.axles.rear,
          color: hexColor(run.color),
        },
      });
    }
    if (layerSwitches.labels) {
      const suffix = pose.ended ? ` ${STATUS_LABELS[run.summary.status] ?? ""}` : "";
      runShapes.push({ text: { x_m: pose.x, y_m: pose.y - 0.45, text: `${run.label}${suffix}`, size_px: 13, color: LABEL_INK } });
    }
    // A run that's over is parked, faded.
    layers.push({ opacity: pose.ended ? 0.5 : 1, shapes: runShapes });
  }

  if (Replay.hoverMarkers.length) {
    layers.push({
      opacity: 1,
      shapes: Replay.hoverMarkers.flatMap(({ x, y, color }) => [
        { circle: { x_m: x, y_m: y, radius_m: 0.14, filled: true, color: hexColor(color) } },
        { circle: { x_m: x, y_m: y, radius_m: 0.14, filled: false, color: LABEL_INK } },
      ]),
    });
  }
  return layers;
}

function followedRun() {
  return Replay.runs.find((run) => run.id === Replay.follow) ?? null;
}

function worldBounds() {
  const info = Replay.map?.info;
  if (!info) return null;
  return {
    minX: info.origin.x,
    minY: info.origin.y,
    maxX: info.origin.x + info.width_px * info.resolution_m_per_px,
    maxY: info.origin.y + info.height_px * info.resolution_m_per_px,
  };
}

MapView.init({
  layersAt: (nowMs) => {
    // Following: the view moves with the run, before anything is painted.
    const run = followedRun();
    if (run) {
      const pose = TrackMath.poseAt(run.trajectory, Replay.runTime(run));
      if (pose) {
        MapView.view.centerX = pose.x;
        MapView.view.centerY = pose.y;
      }
    }
    return layersAt(nowMs);
  },
  worldBounds,
  homeTarget: () => {
    const run = followedRun();
    if (run) {
      const pose = TrackMath.poseAt(run.trajectory, Replay.runTime(run));
      if (pose) return { x: pose.x, y: pose.y };
    }
    const bounds = worldBounds();
    return bounds ? { x: (bounds.minX + bounds.maxX) / 2, y: (bounds.minY + bounds.maxY) / 2 } : null;
  },
  statusTitle: () => (Replay.map ? `${Replay.map.name} · ${Replay.runs.length} run${Replay.runs.length === 1 ? "" : "s"}` : "Select runs to replay them"),
  speedMps: () => {
    const run = followedRun() ?? Replay.runs[0];
    return run ? TrackMath.poseAt(run.trajectory, Replay.runTime(run))?.speed ?? null : null;
  },
  isAnimating: () => window.PlaybackClock.playing,
  onFrame: () => {
    const tS = window.PlaybackClock.currentTimeUs / 1e6;
    const text = Replay.align === "lap" ? `lap ${Replay.lap} + ${formatClock(tS)}` : `t = ${formatClock(tS)}`;
    MapView.setText(statusTimeEl, text);
  },
});

// ---------------------------------------------------------------------
// The transport bar
// ---------------------------------------------------------------------

window.PlaybackClock.subscribe((timeUs) => {
  const clock = window.PlaybackClock;
  playPauseBtn.textContent = clock.playing ? "⏸" : "▶";
  seekSliderEl.value = clock.durationUs > 0 ? Math.round((1000 * timeUs) / clock.durationUs) : 0;
  playbackTimeLabelEl.textContent = `${formatClock(timeUs / 1e6)} / ${formatClock(clock.durationUs / 1e6)}`;
});

playPauseBtn.addEventListener("click", () => {
  window.PlaybackClock.togglePlaying();
  // Paused at the end, the subscriber isn't called again: refresh the icon.
  playPauseBtn.textContent = window.PlaybackClock.playing ? "⏸" : "▶";
});
document.getElementById("seek-start-btn").addEventListener("click", () => window.PlaybackClock.setTime(0));
document.getElementById("speed-select").addEventListener("change", (event) => {
  window.PlaybackClock.speedMultiplier = Number(event.target.value);
});
seekSliderEl.addEventListener("input", () => {
  window.PlaybackClock.setTime((Number(seekSliderEl.value) / 1000) * window.PlaybackClock.durationUs);
});

lapSelectEl.addEventListener("change", () => {
  Replay.lap = Number(lapSelectEl.value);
  if (Replay.align === "lap") updateDuration();
  window.dispatchEvent(new CustomEvent("viewer:replay"));
  MapView.requestRedraw();
});

alignSelectEl.addEventListener("change", () => {
  Replay.align = alignSelectEl.value;
  updateDuration();
  window.PlaybackClock.setTime(0);
});

/** To the start of the picked lap: every run's own start when lined up on
 *  it, else the followed (or first) run's. */
document.getElementById("lap-jump-btn").addEventListener("click", () => {
  if (Replay.align === "lap") {
    window.PlaybackClock.setTime(0);
    return;
  }
  const run = followedRun() ?? Replay.runs[0];
  const start = run?.starts[Replay.lap];
  if (start !== undefined) window.PlaybackClock.setTime(start * 1e6);
});

followSelectEl.addEventListener("change", () => {
  Replay.follow = followSelectEl.value || null;
  MapView.requestRedraw();
});

trackSelectEl.addEventListener("change", () => {
  Replay.track = trackSelectEl.value;
  reload();
});

window.addEventListener("keydown", (event) => {
  if (event.key !== " " || event.repeat) return;
  const target = event.target;
  if (target instanceof HTMLInputElement || target instanceof HTMLSelectElement || target instanceof HTMLButtonElement) return;
  event.preventDefault();
  window.PlaybackClock.togglePlaying();
  playPauseBtn.textContent = window.PlaybackClock.playing ? "⏸" : "▶";
});

document.getElementById("bottom-panel-toggle-btn").addEventListener("click", () => {
  document.getElementById("bottom-panel").classList.toggle("collapsed");
});

window.PlaybackClock.setTime(0);
