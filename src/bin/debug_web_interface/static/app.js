"use strict";

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
// State
// ---------------------------------------------------------------------

/** @type {{name:string|null, info:object|null, offscreen:HTMLCanvasElement}|null} */
let currentMap = null;

/** World-space view: how many meters of world height are visible, and
 *  which world point (in meters, same frame as MapInfo) is centered. */
const view = {
  verticalSizeM: 10,
  centerX: 0,
  centerY: 0,
};

const canvas = document.getElementById("map-canvas");
const ctx = canvas.getContext("2d");

// ---------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

async function fetchJSON(url) {
  const response = await fetch(url);
  const body = await response.json();
  if (!response.ok) {
    throw new Error(body && body.error ? body.error : `request failed (${response.status})`);
  }
  return body;
}

function maxVerticalSizeM() {
  if (!currentMap || !currentMap.info) return 100;
  return currentMap.info.height_px * currentMap.info.resolution_m_per_px;
}

const MIN_VERTICAL_SIZE_M = 0.01;

// ---------------------------------------------------------------------
// Canvas sizing (device-pixel aware)
// ---------------------------------------------------------------------

function resizeCanvasToDisplaySize() {
  const rect = canvas.parentElement.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.round(rect.width * dpr));
  const height = Math.max(1, Math.round(rect.height * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
}

// ---------------------------------------------------------------------
// World <-> screen transforms (device pixels)
// ---------------------------------------------------------------------

function scalePxPerMeter() {
  return canvas.height / view.verticalSizeM;
}

function screenToWorld(screenX, screenY) {
  const scale = scalePxPerMeter();
  return {
    x: (screenX - canvas.width / 2) / scale + view.centerX,
    y: (screenY - canvas.height / 2) / scale + view.centerY,
  };
}

function worldToScreen(worldX, worldY) {
  const scale = scalePxPerMeter();
  return {
    x: (worldX - view.centerX) * scale + canvas.width / 2,
    y: (worldY - view.centerY) * scale + canvas.height / 2,
  };
}

// Client (CSS) pixels -> device pixels, for mouse/wheel event coordinates.
function clientToDevice(clientX, clientY) {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  return { x: (clientX - rect.left) * dpr, y: (clientY - rect.top) * dpr };
}

// ---------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------

function draw() {
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.fillStyle = "#008080";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  if (!currentMap || !currentMap.info) return;

  const info = currentMap.info;
  const scale = scalePxPerMeter();
  const pixelScale = scale * info.resolution_m_per_px;
  const e = (info.origin.x - view.centerX) * scale + canvas.width / 2;
  const f = (info.origin.y - view.centerY) * scale + canvas.height / 2;

  ctx.imageSmoothingEnabled = false;
  ctx.setTransform(pixelScale, 0, 0, pixelScale, e, f);
  ctx.drawImage(currentMap.offscreen, 0, 0);

  ctx.setTransform(1, 0, 0, 1, 0, 0);
  const a = worldToScreen(info.start_finish_line.a.x, info.start_finish_line.a.y);
  const b = worldToScreen(info.start_finish_line.b.x, info.start_finish_line.b.y);
  ctx.strokeStyle = "#ff3b3b";
  ctx.lineWidth = 2 * (window.devicePixelRatio || 1);
  ctx.beginPath();
  ctx.moveTo(a.x, a.y);
  ctx.lineTo(b.x, b.y);
  ctx.stroke();

  drawVehicle();
  updateStatusBar();
  syncZoomSlider();
}

// ---------------------------------------------------------------------
// Vehicle
// ---------------------------------------------------------------------

const VEHICLE_LENGTH_M = 0.45;
const VEHICLE_WIDTH_M = 0.25;

function drawVehicle() {
  const status = window.PlaybackClock.currentVehicleStatus();
  if (!status) return;

  const { x, y } = worldToScreen(status.x_m, status.y_m);
  const scale = scalePxPerMeter();
  const lengthPx = VEHICLE_LENGTH_M * scale;
  const widthPx = VEHICLE_WIDTH_M * scale;

  ctx.save();
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.translate(x, y);
  ctx.rotate(status.heading_rad);

  ctx.fillStyle = "#ffb020";
  ctx.fillRect(-lengthPx / 2, -widthPx / 2, lengthPx, widthPx);
  ctx.strokeStyle = "#101418";
  ctx.lineWidth = 1.5 * (window.devicePixelRatio || 1);
  ctx.strokeRect(-lengthPx / 2, -widthPx / 2, lengthPx, widthPx);

  ctx.beginPath();
  ctx.moveTo(lengthPx / 2, 0);
  ctx.lineTo(lengthPx / 2 - widthPx * 0.4, -widthPx * 0.35);
  ctx.lineTo(lengthPx / 2 - widthPx * 0.4, widthPx * 0.35);
  ctx.closePath();
  ctx.fillStyle = "#101418";
  ctx.fill();

  ctx.restore();
}

function frame() {
  resizeCanvasToDisplaySize();
  draw();
}

// ---------------------------------------------------------------------
// Status bar
// ---------------------------------------------------------------------

const statusName = document.getElementById("status-map-name");
const statusVerticalSize = document.getElementById("status-vertical-size");
const statusSpeed = document.getElementById("status-speed");
const statusTime = document.getElementById("status-time");
const playbackTimeLabel = document.getElementById("playback-time-label");

function updateStatusBar() {
  statusName.textContent = currentMap && currentMap.name ? currentMap.name : "No map loaded";
  statusVerticalSize.textContent = `Vertical size: ${view.verticalSizeM.toFixed(2)} m`;
  const status = window.PlaybackClock.currentVehicleStatus();
  statusSpeed.textContent = status ? `Speed: ${status.speed_mps.toFixed(2)} m/s` : "";
  const tS = (window.PlaybackClock.currentTimeUs / 1e6).toFixed(2);
  const durS = (window.PlaybackClock.durationUs / 1e6).toFixed(2);
  statusTime.textContent = `t = ${tS} s`;
  playbackTimeLabel.textContent = `${tS} s / ${durS} s`;
}

// ---------------------------------------------------------------------
// Zoom (mouse wheel + slider) and pan (left-button drag)
// ---------------------------------------------------------------------

function setVerticalSize(size) {
  view.verticalSizeM = clamp(size, MIN_VERTICAL_SIZE_M, maxVerticalSizeM());
}

function zoomAt(deviceX, deviceY, factor) {
  const before = screenToWorld(deviceX, deviceY);
  setVerticalSize(view.verticalSizeM * factor);
  const after = screenToWorld(deviceX, deviceY);
  view.centerX += before.x - after.x;
  view.centerY += before.y - after.y;
  frame();
}

canvas.addEventListener(
  "wheel",
  (event) => {
    event.preventDefault();
    const { x, y } = clientToDevice(event.clientX, event.clientY);
    const factor = Math.exp(event.deltaY * 0.0015);
    zoomAt(x, y, factor);
  },
  { passive: false }
);

let dragging = false;
let lastDevice = { x: 0, y: 0 };

canvas.addEventListener("mousedown", (event) => {
  if (event.button !== 0) return;
  dragging = true;
  canvas.classList.add("panning");
  lastDevice = clientToDevice(event.clientX, event.clientY);
});

window.addEventListener("mousemove", (event) => {
  if (!dragging) return;
  const device = clientToDevice(event.clientX, event.clientY);
  const scale = scalePxPerMeter();
  view.centerX -= (device.x - lastDevice.x) / scale;
  view.centerY -= (device.y - lastDevice.y) / scale;
  lastDevice = device;
  frame();
});

window.addEventListener("mouseup", () => {
  dragging = false;
  canvas.classList.remove("panning");
});

// ---------------------------------------------------------------------
// Zoom slider (log-scaled, top = zoomed in) and home button
// ---------------------------------------------------------------------

const zoomSlider = document.getElementById("zoom-slider");
const SLIDER_STEPS = 1000;

function sliderFromVerticalSize(sizeM) {
  const logMin = Math.log(MIN_VERTICAL_SIZE_M);
  const logMax = Math.log(maxVerticalSizeM());
  const t = (Math.log(sizeM) - logMin) / (logMax - logMin);
  return Math.round(clamp(t, 0, 1) * SLIDER_STEPS);
}

function verticalSizeFromSlider(value) {
  const logMin = Math.log(MIN_VERTICAL_SIZE_M);
  const logMax = Math.log(maxVerticalSizeM());
  const t = value / SLIDER_STEPS;
  return Math.exp(logMin + t * (logMax - logMin));
}

let syncingSlider = false;

function syncZoomSlider() {
  syncingSlider = true;
  zoomSlider.value = String(sliderFromVerticalSize(view.verticalSizeM));
  syncingSlider = false;
}

zoomSlider.addEventListener("input", () => {
  if (syncingSlider || !currentMap) return;
  setVerticalSize(verticalSizeFromSlider(Number(zoomSlider.value)));
  frame();
});

document.getElementById("home-btn").addEventListener("click", () => {
  if (!currentMap || !currentMap.info) return;
  const line = currentMap.info.start_finish_line;
  view.centerX = (line.a.x + line.b.x) / 2;
  view.centerY = (line.a.y + line.b.y) / 2;
  setVerticalSize(10);
  frame();
});

// ---------------------------------------------------------------------
// Sidebar collapse (same mechanism as web_gui)
// ---------------------------------------------------------------------

const sidebar = document.getElementById("sidebar");
document.getElementById("sidebar-toggle-btn").addEventListener("click", () => {
  sidebar.classList.toggle("collapsed");
});

// ---------------------------------------------------------------------
// Startup
// ---------------------------------------------------------------------

function buildImageData(bytes, width, height) {
  const rgba = new Uint8ClampedArray(width * height * 4);
  for (let i = 0; i < width * height; i++) {
    const drivable = bytes[i] === 255;
    const o = i * 4;
    rgba[o] = drivable ? 235 : 30;
    rgba[o + 1] = drivable ? 235 : 34;
    rgba[o + 2] = drivable ? 235 : 40;
    rgba[o + 3] = 255;
  }
  return new ImageData(rgba, width, height);
}

async function loadMap(mapSummary) {
  if (!mapSummary || !mapSummary.info) {
    currentMap = null;
    return;
  }
  const rasterResponse = await fetch("/api/map/raster");
  if (!rasterResponse.ok) throw new Error("failed to load the recorded map raster");
  const bytes = new Uint8Array(await rasterResponse.arrayBuffer());

  const offscreen = document.createElement("canvas");
  offscreen.width = mapSummary.width_px;
  offscreen.height = mapSummary.height_px;
  offscreen.getContext("2d").putImageData(buildImageData(bytes, mapSummary.width_px, mapSummary.height_px), 0, 0);

  currentMap = { name: mapSummary.name, info: mapSummary.info, offscreen };
  const line = mapSummary.info.start_finish_line;
  view.centerX = (line.a.x + line.b.x) / 2;
  view.centerY = (line.a.y + line.b.y) / 2;
  setVerticalSize(10);
}

const VEHICLE_MODEL_LABELS = {
  bicycle: "Kinematic bicycle",
  dynamic_bicycle: "Dynamic bicycle (tire forces)",
  nonlinear_bicycle: "Nonlinear bicycle (tire saturation + load transfer)",
  pacejka_bicycle: "Pacejka bicycle (full Magic Formula)",
  two_track: "Two-track (four-wheel, lateral load transfer)",
};

new ResizeObserver(frame).observe(canvas.parentElement);
window.addEventListener("resize", frame);
window.PlaybackClock.subscribe(() => frame());

async function start() {
  const session = await fetchJSON("/api/session");

  document.getElementById("sidebar-map-name").textContent = session.map && session.map.name ? session.map.name : "No map recorded";
  document.getElementById("sidebar-vehicle-model").textContent = session.vehicle_model
    ? VEHICLE_MODEL_LABELS[session.vehicle_model] || session.vehicle_model
    : "-";
  document.getElementById("sidebar-frequency").textContent = `Recorded at ${session.frequency_hz} Hz`;
  document.getElementById("sidebar-duration").textContent = `Duration: ${(session.duration_us / 1e6).toFixed(2)} s`;

  window.PlaybackClock.durationUs = session.duration_us;

  const vehicleTimeline = await fetchJSON("/api/vehicle_status_timeline");
  window.PlaybackClock.vehicleTimeline = vehicleTimeline;

  await loadMap(session.map);
  frame();

  window.dispatchEvent(new CustomEvent("aurorus:session-loaded", { detail: session }));
}

start().catch((err) => console.error(err));
