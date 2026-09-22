"use strict";

// ---------------------------------------------------------------------
// debug_web_interface's frontend: everything specific to *replaying* a
// recorded session - the playback clock, the sidebar summary, and loading
// the recorded map.
//
// The map canvas itself (drawing, panning, zooming, the redraw loop) is
// shared with web_gui and lives in /map_view.js, loaded before this file.
// `fetchJSON` comes from there too. timeline.js loads after this one and
// drives the same PlaybackClock.
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// PlaybackClock: the single source of truth for "what time is it" during
// playback, shared by this file (main canvas) and timeline.js (timeline +
// transport bar). Holds the full, pre-fetched vehicle status timeline so
// scrubbing/playing never needs a network round trip per frame.
// ---------------------------------------------------------------------

window.PlaybackClock = {
  /** @type {{t_us:number,x_m:number,y_m:number,heading_rad:number,speed_mps:number}[]} */
  vehicleTimeline: [],
  durationUs: 0,
  currentTimeUs: 0,
  playing: false,
  speedMultiplier: 1,
  /** @type {((timeUs:number) => void)[]} */
  listeners: [],
  _lastFrameMs: null,

  subscribe(fn) {
    this.listeners.push(fn);
  },

  _notify() {
    for (const fn of this.listeners) fn(this.currentTimeUs);
    MapView.requestRedraw();
  },

  setTime(timeUs) {
    this.currentTimeUs = Math.max(0, Math.min(this.durationUs, timeUs));
    this._notify();
  },

  play() {
    if (this.currentTimeUs >= this.durationUs) this.currentTimeUs = 0;
    this.playing = true;
    this._lastFrameMs = null;
    requestAnimationFrame((ms) => this._tick(ms));
  },

  pause() {
    this.playing = false;
  },

  togglePlaying() {
    if (this.playing) this.pause();
    else this.play();
  },

  _tick(nowMs) {
    if (!this.playing) return;
    if (this._lastFrameMs !== null) {
      const deltaUs = (nowMs - this._lastFrameMs) * 1000 * this.speedMultiplier;
      this.currentTimeUs = Math.min(this.durationUs, this.currentTimeUs + deltaUs);
    }
    this._lastFrameMs = nowMs;
    this._notify();
    if (this.currentTimeUs >= this.durationUs) {
      this.playing = false;
      return;
    }
    requestAnimationFrame((ms) => this._tick(ms));
  },

  /** Hold-last-value lookup, matching how a live RwLockTopic read behaves. */
  currentVehicleStatus() {
    return statusAt(this.vehicleTimeline, this.currentTimeUs);
  },
};

function statusAt(timeline, timeUs) {
  if (timeline.length === 0) return null;
  let lo = 0;
  let hi = timeline.length - 1;
  if (timeUs < timeline[0].t_us) return null;
  while (lo < hi) {
    const mid = (lo + hi + 1) >> 1;
    if (timeline[mid].t_us <= timeUs) lo = mid;
    else hi = mid - 1;
  }
  return timeline[lo];
}

// ---------------------------------------------------------------------
// Status bar extras
//
// The map name, vertical size and speed are MapView's; playback time is
// this UI's own, so it's filled in from the same frame via `onFrame`.
// ---------------------------------------------------------------------

const statusTime = document.getElementById("status-time");
const playbackTimeLabel = document.getElementById("playback-time-label");

function updatePlaybackTime() {
  const tS = (window.PlaybackClock.currentTimeUs / 1e6).toFixed(2);
  const durS = (window.PlaybackClock.durationUs / 1e6).toFixed(2);
  MapView.setText(statusTime, `t = ${tS} s`);
  MapView.setText(playbackTimeLabel, `${tS} s / ${durS} s`);
}

MapView.init({
  vehiclePoseAt: () => window.PlaybackClock.currentVehicleStatus(),
  isAnimating: () => window.PlaybackClock.playing,
  onFrame: updatePlaybackTime,
});

// ---------------------------------------------------------------------
// Startup
// ---------------------------------------------------------------------

async function loadMap(mapSummary) {
  if (!mapSummary || !mapSummary.info) {
    MapView.setMap(null);
    return;
  }
  const rasterResponse = await fetch("/api/map/raster");
  if (!rasterResponse.ok) throw new Error("failed to load the recorded map raster");
  const bytes = new Uint8Array(await rasterResponse.arrayBuffer());

  MapView.setMap({
    name: mapSummary.name,
    info: mapSummary.info,
    offscreen: MapView.offscreenFromRaster(bytes, mapSummary.width_px, mapSummary.height_px),
  });
}

async function start() {
  const session = await fetchJSON("/api/session");

  document.getElementById("sidebar-map-name").textContent =
    session.map && session.map.name ? session.map.name : "No map recorded";
  // The label comes from the server (which reads it off VehicleModelKind),
  // so there's no second copy of the kind -> label mapping to keep in sync.
  document.getElementById("sidebar-vehicle-model").textContent =
    session.vehicle_model_label || session.vehicle_model || "-";
  document.getElementById("sidebar-frequency").textContent = `Recorded at ${session.frequency_hz} Hz`;
  document.getElementById("sidebar-duration").textContent = `Duration: ${(session.duration_us / 1e6).toFixed(2)} s`;

  window.PlaybackClock.durationUs = session.duration_us;
  window.PlaybackClock.vehicleTimeline = await fetchJSON("/api/vehicle_status_timeline");

  await loadMap(session.map);
  updatePlaybackTime();
  MapView.requestRedraw();

  window.dispatchEvent(new CustomEvent("aurorus:session-loaded", { detail: session }));
}

start().catch((err) => console.error(err));
