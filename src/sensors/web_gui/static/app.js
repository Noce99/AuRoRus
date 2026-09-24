"use strict";

// ---------------------------------------------------------------------
// web_gui's frontend: everything specific to driving the vehicle live -
// the map list and generator, the model picker, WASD control, the drawing
// layers polled onto the map canvas, and the generic topic inspector.
//
// The map canvas itself (drawing every shape kind, panning, zooming, the
// redraw loop) is shared with debug_web_interface and lives in
// /map_view.js, loaded before this file. `fetchJSON` and `startPolling`
// come from there too.
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// Drawing layers
//
// Everything on the canvas comes from drawing topics (`draw/<executor>`,
// see `src/topics/drawing.rs`): one layer per topic, polled together from
// `POST /api/draw`. This page knows how to paint each shape kind, and
// nothing about which executor drew what.
// ---------------------------------------------------------------------

/** How often every drawing topic is polled, in Hz, until the Layers
 *  panel's slider changes it. */
const DEFAULT_DRAW_RATE_HZ = 30;
let drawRateHz = DEFAULT_DRAW_RATE_HZ;

/** A layer older than its drawing's `stale_after_ms` fades out over this
 *  long, down to `STALE_OPACITY` - kept faintly visible rather than hidden,
 *  so the last thing a crashed executor drew is still there to look at. */
const FADE_DURATION_MS = 1000;
const STALE_OPACITY = 0.2;

/** Never dead-reckon a vehicle further than this past its sample (or 1.5
 *  poll periods, when polling slowly enough that this would otherwise make
 *  it stutter between samples): if polling stalls (tab backgrounded, server
 *  busy) we'd rather park the vehicle a little behind than fling it across
 *  the map on stale data. */
const MIN_MAX_EXTRAPOLATION_MS = 150;

/** The server's `Captain` epoch the layers below were read under - a
 *  different one in a response means the backend restarted, and every
 *  cached layer is stale. */
let drawEpoch = null;

/** Topic name -> layer:
 *  `{topic, writer, writeCount, drawing, sampledAtMs, rasters}` -
 *  `drawing` as last received (`{shapes, stale_after_ms, z_index}`),
 *  `sampledAtMs` the `performance.now()` time it was written (receipt time
 *  minus the server-measured age, or null for a never-written seed), and
 *  `rasters` the decoded offscreen canvas of each raster shape, by index. */
const layers = new Map();

/** Topics the user unticked in the Layers panel - kept by name, so a layer
 *  stays hidden across a backend restart. */
const hiddenLayers = new Set();

/** Set when a new raster arrives (i.e. the map changed): the view is homed
 *  once a vehicle drawing written *after* it arrives, so it centers on
 *  where the vehicle was placed on the new map, not where it last was on
 *  the old one. Holds the poll sequence number the raster arrived in. */
let pendingHomeSince = null;
let drawPollSeq = 0;

function shapeKind(shape) {
  return Object.keys(shape)[0];
}

function layerAgeMs(layer, nowMs) {
  return layer.sampledAtMs === null ? null : nowMs - layer.sampledAtMs;
}

function layerOpacity(layer, nowMs) {
  const staleAfterMs = layer.drawing.stale_after_ms;
  const ageMs = layerAgeMs(layer, nowMs);
  if (staleAfterMs === null || ageMs === null || ageMs <= staleAfterMs) return 1;
  return Math.max(STALE_OPACITY, 1 - (ageMs - staleAfterMs) / FADE_DURATION_MS);
}

function isFading(layer, nowMs) {
  const staleAfterMs = layer.drawing.stale_after_ms;
  const ageMs = layerAgeMs(layer, nowMs);
  return staleAfterMs !== null && ageMs !== null && ageMs > staleAfterMs && ageMs < staleAfterMs + FADE_DURATION_MS;
}

/** Visible layers with a drawing, bottom first: by `z_index`, then topic. */
function paintOrder() {
  return [...layers.values()]
    .filter((layer) => layer.drawing && !hiddenLayers.has(layer.topic))
    .sort((a, b) => a.drawing.z_index - b.drawing.z_index || a.topic.localeCompare(b.topic));
}

/** `vehicle` advanced to `nowMs` along its own heading at its own speed -
 *  the same straight-line motion the simulator integrates between ticks.
 *  Steering curvature within one poll period is not modelled, which at a
 *  33 ms period and 8 m/s is a few millimetres. */
function extrapolatedVehicle(vehicle, layer, nowMs) {
  if (layer.sampledAtMs === null || vehicle.speed_mps === 0) return vehicle;
  const maxMs = Math.max(MIN_MAX_EXTRAPOLATION_MS, 1.5 * (1000 / drawRateHz));
  const dtS = Math.min(Math.max(nowMs - layer.sampledAtMs, 0), maxMs) / 1000;
  return {
    ...vehicle,
    x_m: vehicle.x_m + vehicle.speed_mps * Math.cos(vehicle.heading_rad) * dtS,
    y_m: vehicle.y_m + vehicle.speed_mps * Math.sin(vehicle.heading_rad) * dtS,
  };
}

/** A layer's shapes as they should be painted at `nowMs`: vehicles
 *  dead-reckoned forward, rasters with their decoded image attached. */
function shapesAt(layer, nowMs) {
  return layer.drawing.shapes.map((shape, i) => {
    const kind = shapeKind(shape);
    if (kind === "vehicle") return { vehicle: extrapolatedVehicle(shape.vehicle, layer, nowMs) };
    if (kind === "raster") return { raster: { ...shape.raster, offscreen: layer.rasters[i] } };
    return shape;
  });
}

function layersAt(nowMs) {
  return paintOrder().map((layer) => ({ opacity: layerOpacity(layer, nowMs), shapes: shapesAt(layer, nowMs) }));
}

/** The first vehicle any visible layer draws, dead-reckoned to `nowMs`. */
function firstVehicle(nowMs) {
  for (const layer of paintOrder()) {
    for (const shape of layer.drawing.shapes) {
      if (shapeKind(shape) === "vehicle") return extrapolatedVehicle(shape.vehicle, layer, nowMs);
    }
  }
  return null;
}

/** Union of every visible raster's extent. */
function worldBounds() {
  let bounds = null;
  for (const layer of paintOrder()) {
    for (const shape of layer.drawing.shapes) {
      if (shapeKind(shape) !== "raster") continue;
      const r = shape.raster;
      const minX = r.origin_x_m;
      const minY = r.origin_y_m;
      const maxX = minX + r.width_px * r.resolution_m_per_px;
      const maxY = minY + r.height_px * r.resolution_m_per_px;
      bounds = bounds
        ? {
            minX: Math.min(bounds.minX, minX),
            minY: Math.min(bounds.minY, minY),
            maxX: Math.max(bounds.maxX, maxX),
            maxY: Math.max(bounds.maxY, maxY),
          }
        : { minX, minY, maxX, maxY };
    }
  }
  return bounds;
}

/** The vehicle if one is drawn, else the middle of the drawn world. */
function homeTarget() {
  const vehicle = firstVehicle(performance.now());
  if (vehicle) return { x: vehicle.x_m, y: vehicle.y_m };
  const bounds = worldBounds();
  return bounds ? { x: (bounds.minX + bounds.maxX) / 2, y: (bounds.minY + bounds.maxY) / 2 } : null;
}

/** Fetches and decodes every raster shape of `drawing`, or returns null if
 *  any of them couldn't be - e.g. a `409` because the drawing was rewritten
 *  in the meantime - so the caller keeps its previous copy of the layer and
 *  picks the newer drawing up on the next poll. */
async function loadRasters(topic, writeCount, drawing) {
  const rasters = [];
  for (const [i, shape] of drawing.shapes.entries()) {
    if (shapeKind(shape) !== "raster") continue;
    const r = shape.raster;
    const query = new URLSearchParams({ topic, shape: i, epoch: drawEpoch, write_count: writeCount });
    const response = await fetch(`/api/draw/raster?${query}`);
    if (!response.ok) return null;
    const bytes = new Uint8Array(await response.arrayBuffer());
    if (bytes.length !== r.width_px * r.height_px) return null;
    rasters[i] = MapView.offscreenFromRaster(bytes, r.width_px, r.height_px);
  }
  return rasters;
}

/** Applies one `/api/draw` layer entry to `layers`. Returns which shape
 *  kinds a newly received drawing contained, for the auto-home logic. */
async function applyLayer(entry, receivedAtMs) {
  let layer = layers.get(entry.topic);
  if (!layer) {
    layer = { topic: entry.topic, writer: null, writeCount: null, drawing: null, sampledAtMs: null, rasters: [] };
    layers.set(entry.topic, layer);
  }
  layer.writer = entry.writer;
  if (!entry.drawing) {
    // Unchanged since our copy - only its age moved on.
    layer.sampledAtMs = entry.age_ms === null ? null : receivedAtMs - entry.age_ms;
    return new Set();
  }

  const hasRaster = entry.drawing.shapes.some((shape) => shapeKind(shape) === "raster");
  const rasters = hasRaster ? await loadRasters(entry.topic, entry.write_count, entry.drawing) : [];
  if (!rasters) return new Set();

  layer.drawing = entry.drawing;
  layer.writeCount = entry.write_count;
  layer.rasters = rasters;
  layer.sampledAtMs = entry.age_ms === null ? null : receivedAtMs - entry.age_ms;
  return new Set(entry.drawing.shapes.map(shapeKind));
}

async function pollDraw() {
  const known = {};
  for (const layer of layers.values()) {
    if (layer.writeCount !== null) known[layer.topic] = layer.writeCount;
  }
  const response = await fetchJSON("/api/draw", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ epoch: drawEpoch, known }),
  });
  const receivedAtMs = performance.now();
  const seq = ++drawPollSeq;

  if (response.epoch !== drawEpoch) {
    // The backend restarted: every cached layer belongs to the old one, and
    // the server ignored `known`, sending everything in full.
    layers.clear();
    drawEpoch = response.epoch;
  }

  const topicsBefore = [...layers.keys()].join("\n");
  const present = new Set(response.layers.map((entry) => entry.topic));
  for (const topic of [...layers.keys()]) {
    if (!present.has(topic)) layers.delete(topic);
  }

  let newRaster = false;
  let newVehicle = false;
  for (const entry of response.layers) {
    const kinds = await applyLayer(entry, receivedAtMs);
    newRaster ||= kinds.has("raster");
    newVehicle ||= kinds.has("vehicle");
  }

  if (newRaster) pendingHomeSince = seq;
  const anyVehicle = firstVehicle(receivedAtMs) !== null;
  if (pendingHomeSince !== null && ((newVehicle && seq > pendingHomeSince) || !anyVehicle)) {
    pendingHomeSince = null;
    MapView.home();
  }

  if ([...layers.keys()].join("\n") !== topicsBefore) renderLayerList();
  MapView.requestRedraw();
}

MapView.init({
  layersAt,
  worldBounds,
  homeTarget,
  mapName: () => liveMapName,
  speedMps: (nowMs) => {
    const vehicle = firstVehicle(nowMs);
    return vehicle ? vehicle.speed_mps : null;
  },
  // A moving vehicle, or a layer mid-fade, changes the picture every frame
  // even with no input.
  isAnimating: () => {
    const nowMs = performance.now();
    const vehicle = firstVehicle(nowMs);
    if (vehicle && Math.abs(vehicle.speed_mps) > 1e-3) return true;
    return paintOrder().some((layer) => isFading(layer, nowMs));
  },
});

// ---------------------------------------------------------------------
// Layers panel - one checkbox per drawing topic, plus the draw poll rate.
// ---------------------------------------------------------------------

const layerListEl = document.getElementById("layer-list");
const drawRateEl = document.getElementById("draw-rate");
const drawRateValueEl = document.getElementById("draw-rate-value");

/** Range of the read-rate sliders, in Hz. */
const POLL_RATE_MIN_HZ = 1;
const POLL_RATE_MAX_HZ = 100;

function renderLayerList() {
  layerListEl.innerHTML = "";
  if (layers.size === 0) {
    const li = document.createElement("li");
    li.className = "empty";
    li.textContent = "Nothing is drawing yet";
    layerListEl.appendChild(li);
    return;
  }
  for (const topic of [...layers.keys()].sort()) {
    const li = document.createElement("li");
    const label = document.createElement("label");
    const checkbox = document.createElement("input");
    checkbox.type = "checkbox";
    checkbox.checked = !hiddenLayers.has(topic);
    checkbox.addEventListener("change", () => {
      if (checkbox.checked) hiddenLayers.delete(topic);
      else hiddenLayers.add(topic);
      MapView.requestRedraw();
    });
    const name = document.createElement("span");
    name.className = "layer-name";
    name.textContent = topic;
    const freshness = document.createElement("span");
    freshness.className = "layer-freshness";
    freshness.dataset.topic = topic;
    label.append(checkbox, name, freshness);
    li.appendChild(label);
    layerListEl.appendChild(li);
  }
  renderLayerFreshness();
}

/** Human-readable age, e.g. "42 ms", "3.1 s", "5 min". */
function formatAge(ms) {
  if (ms < 1000) return `${Math.round(ms)} ms`;
  if (ms < 60_000) return `${(ms / 1000).toFixed(1)} s`;
  return `${Math.round(ms / 60_000)} min`;
}

function renderLayerFreshness() {
  const nowMs = performance.now();
  for (const el of layerListEl.querySelectorAll(".layer-freshness")) {
    const layer = layers.get(el.dataset.topic);
    if (!layer || !layer.drawing) continue;
    const ageMs = layerAgeMs(layer, nowMs);
    const staleAfterMs = layer.drawing.stale_after_ms;
    MapView.setText(el, ageMs === null ? "never drawn" : `${formatAge(ageMs)} ago`);
    el.classList.toggle("stale", ageMs === null || (staleAfterMs !== null && ageMs > staleAfterMs));
  }
}

drawRateEl.min = POLL_RATE_MIN_HZ;
drawRateEl.max = POLL_RATE_MAX_HZ;
drawRateEl.value = drawRateHz;
drawRateValueEl.textContent = `${drawRateHz} Hz`;
drawRateEl.addEventListener("input", () => {
  drawRateHz = Number(drawRateEl.value);
  drawRateValueEl.textContent = `${drawRateHz} Hz`;
  drawPoller.setIntervalMs(1000 / drawRateHz);
});

// ---------------------------------------------------------------------
// Map list + selection
// ---------------------------------------------------------------------

/** Name of the map the `map` topic currently holds (server-side selection),
 *  or null - for the map list's highlight and the status bar. The map
 *  itself is drawn from `MapServer`'s drawing topic like anything else. */
let liveMapName = null;

const mapListEl = document.getElementById("map-list");

// Writes the wanted map folder to `map_selection` - `map_server` picks it
// up on its own poll cycle and redraws, `pollLiveMap` then reflects the new
// selection here.
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

// Polls the `map` topic (via `/api/map`) and refreshes the list's highlight
// whenever the live selection actually changes - driven by `map_selection`,
// written by `selectMap` above (sidebar clicks, or a freshly generated map).
async function pollLiveMap() {
  const live = (await fetchJSON("/api/map")).value;
  if (live.name === liveMapName) return;
  liveMapName = live.name;
  MapView.requestRedraw();
  await refreshMapList();
}

/** Vehicle model kind the `vehicle_model_status` topic currently holds
 *  (server-side, actually-running model), or null before the first poll -
 *  tracked separately from the `<select>`'s own value so polling can tell
 *  when the live selection has actually changed (e.g. from another tab). */
let liveVehicleModelKind = null;

/** `{kind, label, description}` options fetched once from
 *  `/api/vehicle_models`, kept around so the description paragraph can be
 *  updated without re-fetching every time the selection changes. */
let vehicleModelOptions = [];

// ---------------------------------------------------------------------
// Vehicle model selection
// ---------------------------------------------------------------------

const vehicleModelSelectEl = document.getElementById("vehicle-model-select");
const vehicleModelDescriptionEl = document.getElementById("vehicle-model-description");

// Writes the wanted model kind to `vehicle_model_selection` - `SimulatedVehicle`
// picks it up on its own poll cycle, `pollVehicleModel` then reflects it here.
async function selectVehicleModel(kind) {
  await fetchJSON("/api/vehicle_model_selection", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ kind }),
  });
}

function updateVehicleModelDescription(kind) {
  const option = vehicleModelOptions.find((o) => o.kind === kind);
  vehicleModelDescriptionEl.textContent = option ? option.description : "";
}

async function populateVehicleModelOptions() {
  vehicleModelOptions = await fetchJSON("/api/vehicle_models");
  vehicleModelSelectEl.innerHTML = "";
  for (const option of vehicleModelOptions) {
    const el = document.createElement("option");
    el.value = option.kind;
    el.textContent = option.label;
    vehicleModelSelectEl.appendChild(el);
  }
  updateVehicleModelDescription(vehicleModelSelectEl.value);
}

vehicleModelSelectEl.addEventListener("change", () => {
  liveVehicleModelKind = vehicleModelSelectEl.value; // optimistic; pollVehicleModel confirms it
  updateVehicleModelDescription(vehicleModelSelectEl.value);
  selectVehicleModel(vehicleModelSelectEl.value).catch((err) => console.error(err));
});

// Polls the `vehicle_model_status` topic (via `/api/vehicle_model`) and
// updates the dropdown whenever the live selection actually changes -
// driven by `vehicle_model_selection`, written by `selectVehicleModel`
// above (this tab's dropdown, or another client's).
async function pollVehicleModel() {
  const live = (await fetchJSON("/api/vehicle_model")).value;
  // The dropdown's own value has to be checked too, not just the last kind
  // we saw: this poll starts before `populateVehicleModelOptions` has added
  // any `<option>`s, and assigning `.value` on an empty `<select>` silently
  // does nothing. Tracking only `liveVehicleModelKind` would record the
  // kind as applied, and every later poll would early-return - leaving the
  // dropdown showing the wrong model for the rest of the session.
  if (live.kind === liveVehicleModelKind && vehicleModelSelectEl.value === live.kind) return;
  liveVehicleModelKind = live.kind;
  vehicleModelSelectEl.value = live.kind;
  updateVehicleModelDescription(live.kind);
}

// ---------------------------------------------------------------------
// Left nav rail -> right panel switching
// ---------------------------------------------------------------------

const panelNavButtons = document.querySelectorAll(".panel-nav-btn");

function selectPanel(name) {
  for (const btn of panelNavButtons) {
    btn.classList.toggle("selected", btn.dataset.panel === name);
  }
  for (const section of document.querySelectorAll(".panel-section")) {
    section.hidden = section.id !== `panel-${name}`;
  }
}

for (const btn of panelNavButtons) {
  btn.addEventListener("click", () => selectPanel(btn.dataset.panel));
}

// ---------------------------------------------------------------------
// Topics panel - inspects any registered topic, generically: the list
// comes from `/api/topics`, the picked topic's value from `/api/topic`.
// ---------------------------------------------------------------------

const topicSelectEl = document.getElementById("topic-select");
const topicFreshnessEl = document.getElementById("topic-freshness");
const topicRateEl = document.getElementById("topic-rate");
const topicContentEl = document.getElementById("topic-content");

/** How often the topic list itself is refreshed, while the panel is open. */
const TOPIC_LIST_REFRESH_MS = 2000;
const TOPIC_CONTENT_REFRESH_MS = 200;
/** Where the read-rate slider starts for every topic. */
const DEFAULT_TOPIC_RATE_HZ = 10;
/** A topic whose latest write is older than this is flagged as stale. */
const TOPIC_STALE_AFTER_MS = 1000;
/** Window the Topics panel's mean write rate is measured over. */
const TOPIC_RATE_WINDOW_MS = 10_000;

/** The selected topic's latest `/api/topic` response, plus `receivedAtMs`
 *  (`performance.now()` on arrival), or null. */
let topicSnapshot = null;
/** The selected topic's `{atMs, writeCount}` for every poll within the
 *  last `TOPIC_RATE_WINDOW_MS`, plus the newest one just before it as an
 *  anchor, so the window stays fully covered even for slowly polled topics
 *  - see `meanWriteRateHz`. Reset whenever the selection changes. */
let topicWriteHistory = [];
/** Per topic name, the read rate the slider was last set to. */
const topicRatesHz = {};

const pollRateRowEl = document.getElementById("poll-rate-row");
const pollRateEl = document.getElementById("poll-rate");
const pollRateValueEl = document.getElementById("poll-rate-value");
pollRateEl.min = POLL_RATE_MIN_HZ;
pollRateEl.max = POLL_RATE_MAX_HZ;

function topicPanelVisible() {
  return !document.getElementById("panel-topics").hidden;
}

function selectedTopicRateHz() {
  return topicRatesHz[topicSelectEl.value] ?? DEFAULT_TOPIC_RATE_HZ;
}

async function refreshTopicList() {
  const topics = await fetchJSON("/api/topics");
  const selected = topicSelectEl.value;
  const names = topics.map((topic) => topic.name);
  const current = [...topicSelectEl.options].slice(1).map((option) => option.value);
  if (names.join("\n") === current.join("\n")) return;

  topicSelectEl.length = 1; // keep the "Select a topic..." placeholder
  for (const name of names) {
    const option = document.createElement("option");
    option.value = name;
    option.textContent = name;
    topicSelectEl.appendChild(option);
  }
  // A topic that disappeared (e.g. across a restart) leaves nothing picked.
  topicSelectEl.value = names.includes(selected) ? selected : "";
  if (topicSelectEl.value !== selected) onTopicSelected();
}

/** Mean writes per second of the selected topic over (up to) the last
 *  `TOPIC_RATE_WINDOW_MS` - `0` if its write count didn't move in that
 *  time - or `null` until two polls have been seen. Measured on this page's
 *  poll arrival times, so it's exact over the window up to one poll's
 *  jitter at either end. */
function meanWriteRateHz() {
  if (topicWriteHistory.length < 2) return null;
  const oldest = topicWriteHistory[0];
  const newest = topicWriteHistory[topicWriteHistory.length - 1];
  const spanS = (newest.atMs - oldest.atMs) / 1000;
  return spanS > 0 ? (newest.writeCount - oldest.writeCount) / spanS : null;
}

async function pollSelectedTopic() {
  const name = topicSelectEl.value;
  if (!name || !topicPanelVisible()) return;
  const snapshot = await fetchJSON(`/api/topic?${new URLSearchParams({ name })}`);
  if (topicSelectEl.value !== name) return; // selection changed mid-flight
  const receivedAtMs = performance.now();
  topicSnapshot = { ...snapshot, receivedAtMs };

  // A server-side restart starts every write count over from 0 - earlier
  // samples no longer compare, so start measuring afresh.
  const history = topicWriteHistory;
  if (history.length > 0 && snapshot.write_count < history[history.length - 1].writeCount) history.length = 0;
  history.push({ atMs: receivedAtMs, writeCount: snapshot.write_count });
  while (history.length > 1 && history[1].atMs <= receivedAtMs - TOPIC_RATE_WINDOW_MS) history.shift();
}

function renderTopicContent() {
  const snapshot = topicSelectEl.value ? topicSnapshot : null;
  if (!snapshot) {
    topicFreshnessEl.textContent = "";
    topicFreshnessEl.classList.remove("stale");
    topicRateEl.textContent = "";
    topicContentEl.textContent = "";
    return;
  }
  const rateHz = meanWriteRateHz();
  topicRateEl.textContent =
    rateHz === null ? "measuring write rate…" : `${rateHz.toFixed(1)} Hz mean over the last ${TOPIC_RATE_WINDOW_MS / 1000} s`;
  const writer = snapshot.writer ? ` by ${snapshot.writer}` : "";
  if (snapshot.age_ms === null) {
    topicFreshnessEl.textContent = `never written (initial value)${snapshot.writer ? ` · writer: ${snapshot.writer}` : ""}`;
    topicFreshnessEl.classList.add("stale");
  } else {
    // The server measured age_ms when it answered; add how long ago that was.
    const ageMs = snapshot.age_ms + (performance.now() - snapshot.receivedAtMs);
    topicFreshnessEl.textContent = `written ${formatAge(ageMs)} ago${writer} · write #${snapshot.write_count}`;
    topicFreshnessEl.classList.toggle("stale", ageMs > TOPIC_STALE_AFTER_MS);
  }
  const { value, too_large, receivedAtMs, name, writer: _, ...meta } = snapshot;
  topicContentEl.textContent = too_large
    ? "Value too large to display."
    : JSON.stringify({ ...meta, value }, null, 2);
}

/** Points the slider at the currently selected topic's rate. */
function syncPollRateSlider() {
  const name = topicSelectEl.value;
  pollRateRowEl.hidden = !name;
  if (!name) return;
  pollRateEl.value = selectedTopicRateHz();
  pollRateValueEl.textContent = `${selectedTopicRateHz()} Hz`;
}

function onTopicSelected() {
  topicSnapshot = null;
  topicWriteHistory = [];
  syncPollRateSlider();
  renderTopicContent();
  topicPoller.setIntervalMs(1000 / selectedTopicRateHz());
}

topicSelectEl.addEventListener("change", onTopicSelected);

pollRateEl.addEventListener("input", () => {
  const name = topicSelectEl.value;
  if (!name) return;
  topicRatesHz[name] = Number(pollRateEl.value);
  pollRateValueEl.textContent = `${topicRatesHz[name]} Hz`;
  topicPoller.setIntervalMs(1000 / topicRatesHz[name]);
});

setInterval(() => {
  // Only worth the work while the panel it updates is actually visible.
  if (topicPanelVisible() && topicSelectEl.value) renderTopicContent();
  if (!document.getElementById("panel-layers").hidden) renderLayerFreshness();
}, TOPIC_CONTENT_REFRESH_MS);

setInterval(() => {
  if (topicPanelVisible()) refreshTopicList().catch((err) => console.error(err));
}, TOPIC_LIST_REFRESH_MS);

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

/** How often the current map and vehicle model selections are re-read, to
 *  reflect changes made from another tab. */
const SELECTION_POLL_MS = 500;

const drawPoller = startPolling(pollDraw, 1000 / drawRateHz);
const topicPoller = startPolling(pollSelectedTopic, 1000 / DEFAULT_TOPIC_RATE_HZ);
startPolling(pollLiveMap, SELECTION_POLL_MS);
startPolling(pollVehicleModel, SELECTION_POLL_MS);
renderLayerList();
refreshTopicList().catch((err) => console.error(err));
syncPollRateSlider();

refreshMapList()
  .then(async (maps) => {
    const live = (await fetchJSON("/api/map")).value;
    if (!live.name && maps.length > 0) {
      await selectMap(maps[0].name);
    }
  })
  .catch((err) => console.error(err));

populateVehicleModelOptions()
  .then(() => pollVehicleModel())
  .catch((err) => console.error(err));
