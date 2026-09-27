"use strict";

// ---------------------------------------------------------------------
// replay_web_gui's frontend: everything specific to *replaying* a
// recorded session - the playback clock, and the sidebar summary.
//
// The map canvas itself (painting, panning, zooming, the redraw loop) and
// the drawing layers behind it are shared with web_gui and live in
// /map_view.js and /draw_layers.js, loaded before this file - the canvas
// shows whatever the recorded executors drew, exactly as web_gui showed
// it live. `fetchJSON`, `startPolling` and `formatAge` come from there too.
// timeline.js loads after this one and drives the same PlaybackClock.
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// PlaybackClock: the single source of truth for "what time is it" during
// playback, shared by this file (main canvas) and timeline.js (timeline +
// transport bar).
// ---------------------------------------------------------------------

window.PlaybackClock = {
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
};

// ---------------------------------------------------------------------
// Drawing layers - aged and dead-reckoned against the playback time, not
// the wall clock, so a paused recording stays still, and a recorded
// executor that stopped publishing fades out on replay just as it did live.
// ---------------------------------------------------------------------

/** How often the drawings at the current playback time are fetched. In
 *  between, vehicles are dead-reckoned along the playback clock. */
const DRAW_RATE_HZ = 30;

/** Never dead-reckon a vehicle further than this past its sample (or 1.5
 *  recording periods, for a recording made slowly enough that this would
 *  otherwise make it stutter between samples). */
const MIN_MAX_EXTRAPOLATION_MS = 150;
let recordingFrequencyHz = 100;

let fileName = null;

const drawLayers = DrawLayers.create({
  clock: () => window.PlaybackClock.currentTimeUs / 1000,
  requestExtras: () => ({ t_us: Math.round(window.PlaybackClock.currentTimeUs) }),
  maxExtrapolationMs: () => Math.max(MIN_MAX_EXTRAPOLATION_MS, 1.5 * (1000 / recordingFrequencyHz)),
  listEl: document.getElementById("layer-list"),
  // The view is the user's: only center it the first time a map shows up,
  // never again on seeking back and forth across when it was loaded.
  homeOnEveryNewRaster: false,
});

// ---------------------------------------------------------------------
// Status bar extras
//
// The title, vertical size and speed are MapView's; playback time is this
// UI's own, so it's filled in from the same frame via `onFrame`.
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
  layersAt: drawLayers.layersAt,
  worldBounds: drawLayers.worldBounds,
  homeTarget: drawLayers.homeTarget,
  statusTitle: () => fileName,
  speedMps: drawLayers.speedMps,
  isAnimating: () => window.PlaybackClock.playing || drawLayers.isAnimating(),
  onFrame: (nowMs) => {
    updatePlaybackTime();
    drawLayers.renderFreshness();
  },
});

// ---------------------------------------------------------------------
// Bottom panel: the lap telemetry recorded at the playback time (see
// /lap_panel.js) - not refetched while that time stands still.
// ---------------------------------------------------------------------

const playbackTimeUs = () => Math.round(window.PlaybackClock.currentTimeUs);

LapPanel.init({
  fetchTelemetry: () => fetchJSON(`/api/lap_telemetry?t_us=${playbackTimeUs()}`),
  pollKey: playbackTimeUs,
});

// ---------------------------------------------------------------------
// Startup
// ---------------------------------------------------------------------

async function start() {
  const session = await fetchJSON("/api/session");

  fileName = session.file_name;
  recordingFrequencyHz = session.frequency_hz;
  document.getElementById("sidebar-file-name").textContent = session.file_name;
  document.getElementById("sidebar-frequency").textContent = `Recorded at ${session.frequency_hz} Hz`;
  document.getElementById("sidebar-duration").textContent = `Duration: ${(session.duration_us / 1e6).toFixed(2)} s`;

  window.PlaybackClock.durationUs = session.duration_us;
  updatePlaybackTime();
  startPolling(drawLayers.poll, 1000 / DRAW_RATE_HZ);

  window.dispatchEvent(new CustomEvent("aurorus:session-loaded", { detail: session }));
}

start().catch((err) => console.error(err));
