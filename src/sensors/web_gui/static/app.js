"use strict";

// ---------------------------------------------------------------------
// State
// ---------------------------------------------------------------------

/** @type {{name:string, info:object, offscreen:HTMLCanvasElement}|null} */
let currentMap = null;

/** Name of the map the `map` topic currently holds (server-side selection),
 *  or null - tracked separately from `currentMap.name` so polling can tell
 *  when the live selection has actually changed. */
let liveMapName = null;

/** @type {{x_m:number, y_m:number, heading_rad:number, speed_mps:number}|null} */
let vehicleStatus = null;

/** Vehicle model kind the `vehicle_model_status` topic currently holds
 *  (server-side, actually-running model), or null before the first poll -
 *  tracked separately from the `<select>`'s own value so polling can tell
 *  when the live selection has actually changed (e.g. from another tab). */
let liveVehicleModelKind = null;

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

async function fetchJSON(url, options) {
  const response = await fetch(url, options);
  const body = await response.json();
  if (!response.ok) {
    throw new Error(body && body.error ? body.error : `request failed (${response.status})`);
  }
  return body;
}

function maxVerticalSizeM() {
  if (!currentMap) return 100;
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

  if (!currentMap) return;

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
  if (!vehicleStatus) return;

  const { x, y } = worldToScreen(vehicleStatus.x_m, vehicleStatus.y_m);
  const scale = scalePxPerMeter();
  const lengthPx = VEHICLE_LENGTH_M * scale;
  const widthPx = VEHICLE_WIDTH_M * scale;

  ctx.save();
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.translate(x, y);
  ctx.rotate(vehicleStatus.heading_rad);

  ctx.fillStyle = "#ffb020";
  ctx.fillRect(-lengthPx / 2, -widthPx / 2, lengthPx, widthPx);
  ctx.strokeStyle = "#101418";
  ctx.lineWidth = 1.5 * (window.devicePixelRatio || 1);
  ctx.strokeRect(-lengthPx / 2, -widthPx / 2, lengthPx, widthPx);

  // Small triangle marking the front, so heading is visible at a glance.
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

function updateStatusBar() {
  statusName.textContent = currentMap ? currentMap.name : "No map loaded";
  statusVerticalSize.textContent = `Vertical size: ${view.verticalSizeM.toFixed(2)} m`;
  statusSpeed.textContent = vehicleStatus ? `Speed: ${vehicleStatus.speed_mps.toFixed(2)} m/s` : "";
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
  if (!currentMap) return;
  const line = currentMap.info.start_finish_line;
  view.centerX = (line.a.x + line.b.x) / 2;
  view.centerY = (line.a.y + line.b.y) / 2;
  setVerticalSize(10);
  frame();
});

// ---------------------------------------------------------------------
// Sidebar collapse
// ---------------------------------------------------------------------

const sidebar = document.getElementById("sidebar");
document.getElementById("sidebar-toggle-btn").addEventListener("click", () => {
  sidebar.classList.toggle("collapsed");
});

// ---------------------------------------------------------------------
// Map list + loading
// ---------------------------------------------------------------------

const mapListEl = document.getElementById("map-list");

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

// The sidebar only ever *requests* a map via `/api/map_selection` -
// `map_server` is the one that actually reads it off disk and republishes
// the `map` topic, which `pollLiveMap` below picks up and renders via this.
async function loadLiveMap(name, widthPx, heightPx) {
  const info = await fetchJSON(`/api/maps/${encodeURIComponent(name)}/info`);
  const rasterResponse = await fetch("/api/map/raster");
  if (!rasterResponse.ok) throw new Error(`failed to load live raster for ${name}`);
  const bytes = new Uint8Array(await rasterResponse.arrayBuffer());

  const offscreen = document.createElement("canvas");
  offscreen.width = widthPx;
  offscreen.height = heightPx;
  offscreen.getContext("2d").putImageData(buildImageData(bytes, widthPx, heightPx), 0, 0);

  currentMap = { name, info, offscreen };
  const line = info.start_finish_line;
  view.centerX = (line.a.x + line.b.x) / 2;
  view.centerY = (line.a.y + line.b.y) / 2;
  setVerticalSize(10);
}

// Writes the wanted map folder to `map_selection` - `map_server` picks it
// up on its own poll cycle, `pollLiveMap` then reflects it here.
async function selectMap(name) {
  await fetchJSON("/api/map_selection", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ name }),
  });
}

function renderMapList(maps, selectedName) {
  mapListEl.innerHTML = "";
  if (maps.length === 0) {
    const li = document.createElement("li");
    li.className = "empty";
    li.textContent = "No maps yet";
    mapListEl.appendChild(li);
    return;
  }
  for (const map of maps) {
    const li = document.createElement("li");
    li.textContent = map.name;
    li.title = `${map.width_px}x${map.height_px} px, seed ${map.seed}`;
    if (map.name === selectedName) li.classList.add("selected");
    li.addEventListener("click", () => {
      renderMapList(maps, map.name); // optimistic highlight; pollLiveMap confirms it
      selectMap(map.name).catch((err) => console.error(err));
    });
    mapListEl.appendChild(li);
  }
}

async function refreshMapList() {
  const maps = await fetchJSON("/api/maps");
  renderMapList(maps, liveMapName);
  return maps;
}

// Polls the `map` topic (via `/api/map`) and reloads the canvas whenever the
// live selection actually changes - driven by `map_selection`, written by
// `selectMap` above (sidebar clicks, or a freshly generated map).
async function pollLiveMap() {
  const live = await fetchJSON("/api/map");
  if (live.name === liveMapName) return;

  liveMapName = live.name;
  if (live.name) {
    await loadLiveMap(live.name, live.width_px, live.height_px);
  } else {
    currentMap = null;
  }
  renderMapList(await fetchJSON("/api/maps"), liveMapName);
  frame();
}

const LIVE_MAP_POLL_MS = 500;
const VEHICLE_STATUS_POLL_MS = 50;

async function pollVehicleStatus() {
  vehicleStatus = await fetchJSON("/api/vehicle_status");
  frame();
}

// ---------------------------------------------------------------------
// Vehicle model selection
// ---------------------------------------------------------------------

const vehicleModelSelectEl = document.getElementById("vehicle-model-select");

// Writes the wanted model kind to `vehicle_model_selection` - `SimulatedVehicle`
// picks it up on its own poll cycle, `pollVehicleModel` then reflects it here.
async function selectVehicleModel(kind) {
  await fetchJSON("/api/vehicle_model_selection", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ kind }),
  });
}

async function populateVehicleModelOptions() {
  const options = await fetchJSON("/api/vehicle_models");
  vehicleModelSelectEl.innerHTML = "";
  for (const option of options) {
    const el = document.createElement("option");
    el.value = option.kind;
    el.textContent = option.label;
    vehicleModelSelectEl.appendChild(el);
  }
}

vehicleModelSelectEl.addEventListener("change", () => {
  liveVehicleModelKind = vehicleModelSelectEl.value; // optimistic; pollVehicleModel confirms it
  selectVehicleModel(vehicleModelSelectEl.value).catch((err) => console.error(err));
});

// Polls the `vehicle_model_status` topic (via `/api/vehicle_model`) and
// updates the dropdown whenever the live selection actually changes -
// driven by `vehicle_model_selection`, written by `selectVehicleModel`
// above (this tab's dropdown, or another client's).
async function pollVehicleModel() {
  const live = await fetchJSON("/api/vehicle_model");
  if (live.kind === liveVehicleModelKind) return;
  liveVehicleModelKind = live.kind;
  vehicleModelSelectEl.value = live.kind;
}

// ---------------------------------------------------------------------
// Generate Map modal
// ---------------------------------------------------------------------

const overlay = document.getElementById("generate-overlay");
const form = document.getElementById("generate-form");
const errorEl = document.getElementById("generate-error");
const confirmBtn = document.getElementById("generate-confirm-btn");

function showError(message) {
  errorEl.textContent = message;
  errorEl.hidden = false;
}

function hideError() {
  errorEl.hidden = true;
}

async function openGenerateModal() {
  hideError();
  try {
    const defaults = await fetchJSON("/api/generate/defaults");
    for (const input of form.elements) {
      if (!input.name) continue;
      const value = defaults[input.name];
      input.value = value === null || value === undefined ? "" : value;
    }
  } catch (err) {
    showError(`failed to load defaults: ${err.message}`);
  }
  overlay.hidden = false;
}

function closeGenerateModal() {
  overlay.hidden = true;
}

document.getElementById("generate-btn").addEventListener("click", openGenerateModal);
document.getElementById("generate-cancel-btn").addEventListener("click", closeGenerateModal);

form.addEventListener("submit", async (event) => {
  event.preventDefault();
  hideError();
  confirmBtn.disabled = true;

  const payload = {};
  for (const input of form.elements) {
    if (!input.name || input.value === "") continue;
    payload[input.name] = input.type === "number" ? Number(input.value) : input.value;
  }

  try {
    const generated = await fetchJSON("/api/maps/generate", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    closeGenerateModal();
    await refreshMapList();
    await selectMap(generated.name);
  } catch (err) {
    showError(err.message);
  } finally {
    confirmBtn.disabled = false;
  }
});

// ---------------------------------------------------------------------
// WASD human control -> human_vesc_command
// ---------------------------------------------------------------------

/** Overwritten from `GET /api/config` at startup - these are only
 *  fallbacks for the brief window before that first fetch resolves. */
let humanMaxSpeedMps = 3.0;
let humanMaxSteeringRad = 0.35;
const HUMAN_COMMAND_POST_MS = 50;

const keys = { w: false, a: false, s: false, d: false };

function isTypingTarget(target) {
  return target instanceof HTMLInputElement || target instanceof HTMLTextAreaElement;
}

window.addEventListener("keydown", (event) => {
  if (isTypingTarget(event.target)) return;
  const key = event.key.toLowerCase();
  if (!(key in keys)) return;
  keys[key] = true;
  event.preventDefault();
});

window.addEventListener("keyup", (event) => {
  const key = event.key.toLowerCase();
  if (!(key in keys)) return;
  keys[key] = false;
});

// Also release every key when focus leaves the window/tab, so the car
// doesn't keep driving after e.g. alt-tabbing away mid-turn.
window.addEventListener("blur", () => {
  keys.w = keys.a = keys.s = keys.d = false;
});

function currentHumanCommand() {
  const speed = (keys.w ? 1 : 0) - (keys.s ? 1 : 0);
  // Positive steering_angle_rad is a right turn (heading rotates clockwise
  // in this world frame - see bicycle.rs) - D is the right key, so D is
  // positive and A is negative.
  const steer = (keys.d ? 1 : 0) - (keys.a ? 1 : 0);
  return {
    servo_position_rad: steer * humanMaxSteeringRad,
    speed_mps: speed * humanMaxSpeedMps,
  };
}

setInterval(() => {
  fetch("/api/human_vesc_command", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(currentHumanCommand()),
  }).catch((err) => console.error(err));
}, HUMAN_COMMAND_POST_MS);

// ---------------------------------------------------------------------
// "R" -> restart everything
// ---------------------------------------------------------------------

window.addEventListener("keydown", (event) => {
  if (isTypingTarget(event.target)) return;
  // Ignore held-key auto-repeat and modified presses, so this doesn't fire
  // on every repeat while held, or steal the browser's own Ctrl/Cmd+R reload.
  if (event.repeat || event.ctrlKey || event.metaKey || event.altKey) return;
  if (event.key.toLowerCase() !== "r") return;
  event.preventDefault();
  fetch("/api/restart", { method: "POST" }).catch((err) => console.error(err));
});

// ---------------------------------------------------------------------
// "P" -> place the vehicle at the start line
// ---------------------------------------------------------------------

window.addEventListener("keydown", (event) => {
  if (isTypingTarget(event.target)) return;
  if (event.repeat || event.ctrlKey || event.metaKey || event.altKey) return;
  if (event.key.toLowerCase() !== "p") return;
  event.preventDefault();
  fetch("/api/place_at_start", { method: "POST" }).catch((err) => console.error(err));
});

// ---------------------------------------------------------------------
// Startup
// ---------------------------------------------------------------------

new ResizeObserver(frame).observe(canvas.parentElement);
window.addEventListener("resize", frame);

fetchJSON("/api/config")
  .then((config) => {
    humanMaxSpeedMps = config.human_max_speed_mps;
    humanMaxSteeringRad = config.human_max_steering_rad;
  })
  .catch((err) => console.error(err));

setInterval(() => pollLiveMap().catch((err) => console.error(err)), LIVE_MAP_POLL_MS);
setInterval(() => pollVehicleStatus().catch((err) => console.error(err)), VEHICLE_STATUS_POLL_MS);
setInterval(() => pollVehicleModel().catch((err) => console.error(err)), LIVE_MAP_POLL_MS);

refreshMapList()
  .then(async (maps) => {
    const live = await fetchJSON("/api/map");
    if (!live.name && maps.length > 0) {
      await selectMap(maps[0].name);
    }
    frame();
  })
  .catch((err) => console.error(err));

populateVehicleModelOptions()
  .then(() => pollVehicleModel())
  .catch((err) => console.error(err));
