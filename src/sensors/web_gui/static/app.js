"use strict";

// ---------------------------------------------------------------------
// web_gui's frontend: everything specific to driving the vehicle live -
// the map list and generator, the model picker, WASD control, and the
// live topic polling behind them.
//
// The map canvas itself (drawing, panning, zooming, the redraw loop) is
// shared with debug_web_interface and lives in /map_view.js, loaded before
// this file. `fetchJSON` and `startPolling` come from there too.
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// State
// ---------------------------------------------------------------------

/** Name of the map the `map` topic currently holds (server-side selection),
 *  or null - tracked separately from the loaded map so polling can tell
 *  when the live selection has actually changed. */
let liveMapName = null;

/** The most recent sample from `/api/vehicle_status`, and the
 *  `performance.now()` at which it arrived. Rendering never draws this
 *  sample directly - it draws `predictedVehiclePose()`, which dead-reckons
 *  forward from it, so the vehicle moves at display rate instead of
 *  stepping once per poll.
 *  @type {{x_m:number, y_m:number, heading_rad:number, speed_mps:number}|null} */
let vehicleStatus = null;
let vehicleStatusAtMs = 0;

/** The most recent sample from `/api/lidar_scan`, or null before the first
 *  poll. Reprojected into world-frame points (relative to the current
 *  predicted vehicle pose) by `lidarPointsAt` below, rather than storing
 *  world coordinates directly - `SimulatedLidar` reports distances/angles
 *  relative to the vehicle, not absolute positions.
 *  @type {{points:number[], intensities:number[], num_lidar_points:number, fov:number}|null} */
let lidarScan = null;

/** Vehicle model kind the `vehicle_model_status` topic currently holds
 *  (server-side, actually-running model), or null before the first poll -
 *  tracked separately from the `<select>`'s own value so polling can tell
 *  when the live selection has actually changed (e.g. from another tab). */
let liveVehicleModelKind = null;

// ---------------------------------------------------------------------
// Vehicle pose
// ---------------------------------------------------------------------

/** Never dead-reckon further than this past the last sample: if polling
 *  stalls (tab backgrounded, server busy) we'd rather park the vehicle a
 *  little behind than fling it across the map on stale data. */
const MAX_EXTRAPOLATION_MS = 150;

/** The last sample advanced to `nowMs` along its own heading at its own
 *  speed - the same straight-line motion the simulator itself integrates
 *  between ticks. Steering curvature within one poll period is not
 *  modelled, which at a 33 ms period and 8 m/s is a few millimetres. */
function predictedVehiclePose(nowMs) {
  if (!vehicleStatus) return null;
  const dt_s = Math.min(Math.max(nowMs - vehicleStatusAtMs, 0), MAX_EXTRAPOLATION_MS) / 1000;
  return {
    x_m: vehicleStatus.x_m + vehicleStatus.speed_mps * Math.cos(vehicleStatus.heading_rad) * dt_s,
    y_m: vehicleStatus.y_m + vehicleStatus.speed_mps * Math.sin(vehicleStatus.heading_rad) * dt_s,
    heading_rad: vehicleStatus.heading_rad,
    speed_mps: vehicleStatus.speed_mps,
  };
}

/** Turns the latest `lidarScan` into world-frame hit points, relative to the
 *  current predicted vehicle pose: ray `i`'s angle is spread evenly across
 *  `fov`, centered on the vehicle's forward direction, the same formula
 *  `SimulatedLidar` casts its rays with (see `ray_offset_rad` in
 *  `src/sensors/simulated_lidar.rs`). Only rays that actually hit something
 *  are drawn - a no-return ray reports `intensity` 0, an actual hit reports 1. */
function lidarPointsAt(nowMs) {
  if (!lidarScan) return [];
  const pose = predictedVehiclePose(nowMs);
  if (!pose) return [];

  const numPoints = lidarScan.num_lidar_points;
  const points = [];
  for (let i = 0; i < numPoints; i++) {
    if (lidarScan.intensities[i] !== 1) continue;
    const offset = numPoints > 1 ? -lidarScan.fov / 2 + (i * lidarScan.fov) / (numPoints - 1) : 0;
    const angle = pose.heading_rad + offset;
    const distance = lidarScan.points[i];
    points.push({ x_m: pose.x_m + distance * Math.cos(angle), y_m: pose.y_m + distance * Math.sin(angle) });
  }
  return points;
}

MapView.init({
  vehiclePoseAt: predictedVehiclePose,
  lidarPointsAt,
  // A moving vehicle changes the picture every frame even with no input.
  isAnimating: () => vehicleStatus !== null && Math.abs(vehicleStatus.speed_mps) > 1e-3,
});

// ---------------------------------------------------------------------
// Map list + loading
// ---------------------------------------------------------------------

const mapListEl = document.getElementById("map-list");

// The sidebar only ever *requests* a map via `/api/map_selection` -
// `map_server` is the one that actually reads it off disk and republishes
// the `map` topic, which `pollLiveMap` below picks up and renders via this.
async function loadLiveMap(name, widthPx, heightPx) {
  const info = await fetchJSON(`/api/maps/${encodeURIComponent(name)}/info`);
  const rasterResponse = await fetch("/api/map/raster");
  if (!rasterResponse.ok) throw new Error(`failed to load live raster for ${name}`);
  const bytes = new Uint8Array(await rasterResponse.arrayBuffer());

  MapView.setMap({ name, info, offscreen: MapView.offscreenFromRaster(bytes, widthPx, heightPx) });
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
    MapView.setMap(null);
  }
  await refreshMapList();
}

const LIVE_MAP_POLL_MS = 500;
const VEHICLE_STATUS_POLL_MS = 33;
const LIDAR_SCAN_POLL_MS = 33;

async function pollVehicleStatus() {
  const status = await fetchJSON("/api/vehicle_status");
  // A still vehicle produces an identical sample every poll; repainting for
  // those is pure waste, and MapView already keeps painting by itself while
  // `isAnimating()` holds.
  const moved =
    vehicleStatus === null ||
    status.x_m !== vehicleStatus.x_m ||
    status.y_m !== vehicleStatus.y_m ||
    status.heading_rad !== vehicleStatus.heading_rad ||
    status.speed_mps !== vehicleStatus.speed_mps;
  vehicleStatus = status;
  vehicleStatusAtMs = performance.now();
  if (moved) MapView.requestRedraw();
}

async function pollLidarScan() {
  lidarScan = await fetchJSON("/api/lidar_scan");
  MapView.requestRedraw();
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
  // The dropdown's own value has to be checked too, not just the last kind
  // we saw: this poll starts before `populateVehicleModelOptions` has added
  // any `<option>`s, and assigning `.value` on an empty `<select>` silently
  // does nothing. Tracking only `liveVehicleModelKind` would record the
  // kind as applied, and every later poll would early-return - leaving the
  // dropdown showing the wrong model for the rest of the session.
  if (live.kind === liveVehicleModelKind && vehicleModelSelectEl.value === live.kind) return;
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
    // Includes the 409 the server returns when a map of that name already
    // exists - the message tells the user to pick another name.
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

/** How often the currently held command is re-sent even though nothing
 *  changed. The command itself goes out the instant a key goes down or up
 *  (see `sendHumanCommandIfChanged`), so this is purely a safety net: if one
 *  POST is lost, the server would otherwise hold that stale command until
 *  the next key event - which, with a key held down, might be never. */
const HUMAN_COMMAND_HEARTBEAT_MS = 250;

const keys = { w: false, a: false, s: false, d: false };

function isTypingTarget(target) {
  return target instanceof HTMLInputElement || target instanceof HTMLTextAreaElement;
}

window.addEventListener("keydown", (event) => {
  if (isTypingTarget(event.target)) return;
  const key = event.key.toLowerCase();
  if (!(key in keys)) return;
  event.preventDefault();
  if (keys[key]) return; // auto-repeat, not a new press
  keys[key] = true;
  sendHumanCommandIfChanged();
});

window.addEventListener("keyup", (event) => {
  const key = event.key.toLowerCase();
  if (!(key in keys)) return;
  keys[key] = false;
  sendHumanCommandIfChanged();
});

// Also release every key when focus leaves the window/tab, so the car
// doesn't keep driving after e.g. alt-tabbing away mid-turn.
window.addEventListener("blur", () => {
  keys.w = keys.a = keys.s = keys.d = false;
  sendHumanCommandIfChanged();
});

function currentHumanCommand() {
  const speed = (keys.w ? 1 : 0) - (keys.s ? 1 : 0);
  // Positive servo_position_rad steers toward increasing heading, which in
  // this world frame (x right, y down) draws as clockwise - a right turn.
  // See the `servo_position_rad` doc on topics/vesc_command.rs. D is the
  // right key, so D is positive and A is negative.
  const steer = (keys.d ? 1 : 0) - (keys.a ? 1 : 0);
  return {
    servo_position_rad: steer * humanMaxSteeringRad,
    speed_mps: speed * humanMaxSpeedMps,
  };
}

/** The last command actually sent, so `sendHumanCommandIfChanged` can tell
 *  a real change from a repeat. `null` until the first send. */
let lastSentCommand = null;

function sendHumanCommand(command) {
  lastSentCommand = command;
  fetch("/api/human_vesc_command", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(command),
  }).catch((err) => console.error(err));
}

// Sending on the key edge rather than on a timer is what makes the controls
// feel immediate: a press used to wait up to one full post interval before
// it was even put on the wire.
function sendHumanCommandIfChanged() {
  const command = currentHumanCommand();
  if (
    lastSentCommand !== null &&
    command.servo_position_rad === lastSentCommand.servo_position_rad &&
    command.speed_mps === lastSentCommand.speed_mps
  ) {
    return;
  }
  sendHumanCommand(command);
}

setInterval(() => sendHumanCommand(currentHumanCommand()), HUMAN_COMMAND_HEARTBEAT_MS);

// ---------------------------------------------------------------------
// "R" -> restart everything, then reload this page
// "P" -> place the vehicle at the start line
// ---------------------------------------------------------------------

window.addEventListener("keydown", (event) => {
  if (isTypingTarget(event.target)) return;
  // Ignore held-key auto-repeat and modified presses, so these don't fire
  // on every repeat while held, or steal the browser's own Ctrl/Cmd+R reload.
  if (event.repeat || event.ctrlKey || event.metaKey || event.altKey) return;

  switch (event.key.toLowerCase()) {
    case "r":
      event.preventDefault();
      fetch("/api/restart", { method: "POST" }).catch((err) => console.error(err));
      // The restart tears down and relaunches the backend (including this
      // page's server), so wait for the new generation to come back up
      // before reloading.
      setTimeout(() => window.location.reload(), 2000);
      break;
    case "p":
      event.preventDefault();
      fetch("/api/place_at_start", { method: "POST" }).catch((err) => console.error(err));
      break;
  }
});

// ---------------------------------------------------------------------
// Startup
// ---------------------------------------------------------------------

fetchJSON("/api/config")
  .then((config) => {
    humanMaxSpeedMps = config.human_max_speed_mps;
    humanMaxSteeringRad = config.human_max_steering_rad;
  })
  .catch((err) => console.error(err));

startPolling(pollLiveMap, LIVE_MAP_POLL_MS);
startPolling(pollVehicleStatus, VEHICLE_STATUS_POLL_MS);
startPolling(pollLidarScan, LIDAR_SCAN_POLL_MS);
startPolling(pollVehicleModel, LIVE_MAP_POLL_MS);

refreshMapList()
  .then(async (maps) => {
    const live = await fetchJSON("/api/map");
    if (!live.name && maps.length > 0) {
      await selectMap(maps[0].name);
    }
  })
  .catch((err) => console.error(err));

populateVehicleModelOptions()
  .then(() => pollVehicleModel())
  .catch((err) => console.error(err));
