"use strict";

// ---------------------------------------------------------------------
// web_gui's frontend: everything specific to driving the vehicle live -
// the map list and generator, the model picker, WASD control, the drawing
// layers polled onto the map canvas, and the generic topic inspector.
//
// The map canvas itself (painting every shape kind, panning, zooming, the
// redraw loop) and the drawing layers behind it are shared with
// replay_web_gui and live in /map_view.js and /draw_layers.js, loaded
// before this file. `fetchJSON`, `startPolling` and `formatAge` come from
// there too.
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// Drawing layers - everything on the canvas, via /draw_layers.js, aged
// against the wall clock - and the Layers panel listing them.
// ---------------------------------------------------------------------

/** How often every drawing topic is polled, in Hz, until the Layers
 *  panel's slider changes it. */
const DEFAULT_DRAW_RATE_HZ = 30;
let drawRateHz = DEFAULT_DRAW_RATE_HZ;

/** Never dead-reckon a vehicle further than this past its sample (or 1.5
 *  poll periods, when polling slowly enough that this would otherwise make
 *  it stutter between samples). */
const MIN_MAX_EXTRAPOLATION_MS = 150;

const drawLayers = DrawLayers.create({
  clock: () => performance.now(),
  maxExtrapolationMs: () => Math.max(MIN_MAX_EXTRAPOLATION_MS, 1.5 * (1000 / drawRateHz)),
  listEl: document.getElementById("layer-list"),
});

MapView.init({
  layersAt: drawLayers.layersAt,
  worldBounds: drawLayers.worldBounds,
  homeTarget: drawLayers.homeTarget,
  statusTitle: () => liveMapName,
  speedMps: drawLayers.speedMps,
  isAnimating: drawLayers.isAnimating,
});

/** Range of the read-rate sliders, in Hz. */
const POLL_RATE_MIN_HZ = 1;
const POLL_RATE_MAX_HZ = 100;

const drawRateEl = document.getElementById("draw-rate");
const drawRateValueEl = document.getElementById("draw-rate-value");
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
// Autonomous algorithm selection - the list of algorithms comes entirely
// from the `autonomous_algorithm_status` topic, so a new algorithm shows up
// here without this page knowing anything about it. The dropdown picks the
// algorithm; Start and Pause hand control to it and take it back.
// ---------------------------------------------------------------------

const algorithmSelectEl = document.getElementById("algorithm-select");
const algorithmDescriptionEl = document.getElementById("algorithm-description");
const algorithmStatusEl = document.getElementById("algorithm-status");
const algorithmStartBtn = document.getElementById("algorithm-start-btn");
const algorithmPauseBtn = document.getElementById("algorithm-pause-btn");

/** Algorithm the `autonomous_algorithm_status` topic last reported selected,
 *  or null before the first poll - tracked separately from the `<select>`'s
 *  own value for the same reason as `liveVehicleModelKind`. */
let liveAlgorithm = null;

/** Whether the selected algorithm was last reported running (not paused). */
let algorithmRunning = false;

/** `{name, label, description, parameters}` of every algorithm last reported available. */
let algorithmOptions = [];

async function selectAlgorithm(name, running) {
  await fetchJSON("/api/autonomous_algorithm_selection", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ name: name || null, running }),
  });
}

function updateAlgorithmDescription(value) {
  const option = algorithmOptions.find((o) => o.name === value);
  algorithmDescriptionEl.textContent = option ? option.description : "No autonomous algorithm available.";
}

/** Reflects `algorithmRunning` on the Start/Pause buttons. */
function updateAlgorithmButtons() {
  algorithmStartBtn.disabled = algorithmRunning || !algorithmSelectEl.value;
  algorithmPauseBtn.disabled = !algorithmRunning;
}

for (const [button, running] of [[algorithmStartBtn, true], [algorithmPauseBtn, false]]) {
  button.addEventListener("click", () => {
    algorithmRunning = running; // optimistic; pollAlgorithms confirms it
    updateAlgorithmButtons();
    selectAlgorithm(algorithmSelectEl.value, running)
      .then(() => pollAlgorithms())
      .catch((err) => console.error(err));
  });
}

/** Rebuilds the `<option>`s, but only when the available algorithms actually
 *  changed - rebuilding on every poll would close the dropdown while open. */
function syncAlgorithmOptions(available) {
  // Only what the dropdown shows counts - parameters' values change while
  // tuning, and that alone mustn't rebuild it.
  const shown = (options) => JSON.stringify(options.map(({ name, label }) => [name, label]));
  const unchanged = shown(available) === shown(algorithmOptions);
  algorithmOptions = available;
  if (unchanged) return;
  algorithmSelectEl.innerHTML = "";
  for (const option of available) {
    const el = document.createElement("option");
    el.value = option.name;
    el.textContent = option.label;
    algorithmSelectEl.appendChild(el);
  }
  // Force the next check in `pollAlgorithms` to re-apply the live value.
  liveAlgorithm = null;
}

// Switching algorithm keeps the current state: a running one hands control
// straight to the new pick, a paused one stays paused.
algorithmSelectEl.addEventListener("change", () => {
  liveAlgorithm = algorithmSelectEl.value; // optimistic; pollAlgorithms confirms it
  updateAlgorithmDescription(algorithmSelectEl.value);
  selectAlgorithm(algorithmSelectEl.value, algorithmRunning).catch((err) => console.error(err));
});

// Polls the `autonomous_algorithm_status` topic (via
// `/api/autonomous_algorithms`): the available algorithms, the one selected
// (this tab's pick, or another client's), whether it's running, and whether
// its command is fresh.
async function pollAlgorithms() {
  const status = (await fetchJSON("/api/autonomous_algorithms")).value;
  syncAlgorithmOptions(status.available);

  // Nothing picked yet (e.g. right after a restart): pick what the dropdown
  // shows, paused, so there's an algorithm to tune and start.
  if (status.selected === null && algorithmSelectEl.value) {
    await selectAlgorithm(algorithmSelectEl.value, false);
    return;
  }

  const selected = status.selected ?? "";
  if (selected !== liveAlgorithm || algorithmSelectEl.value !== selected) {
    liveAlgorithm = selected;
    algorithmSelectEl.value = selected;
    updateAlgorithmDescription(selected);
  }
  algorithmRunning = status.active !== null;
  updateAlgorithmButtons();

  const stale = algorithmRunning && !status.command_fresh;
  algorithmStatusEl.classList.toggle("stale", stale);
  if (!algorithmRunning) {
    algorithmStatusEl.textContent = "Paused - only a human drives (WASD).";
  } else if (stale) {
    algorithmStatusEl.textContent = "No recent command from this algorithm - vehicle held stopped.";
  } else {
    algorithmStatusEl.textContent = "In control. Any WASD key overrides it.";
  }

  syncAlgorithmParameters(status.selected);
}

// ---------------------------------------------------------------------
// Live tuning of the selected algorithm, running or paused - one slider per parameter it
// declares in its `AutonomousAlgorithmInfo`. A slider sends the wanted value
// to `autonomous_parameters`; what it then shows comes back from the
// algorithm itself (via `autonomous_algorithm_status`), so it always
// reflects the value actually in effect.
// ---------------------------------------------------------------------

const algorithmParametersEl = document.getElementById("algorithm-parameters");
const algorithmSaveEl = document.getElementById("algorithm-save");
const algorithmSaveBtn = document.getElementById("algorithm-save-btn");
const algorithmSaveStatusEl = document.getElementById("algorithm-save-status");

/** Algorithm the sliders (and the Save button) are for, or null for none. */
let parameterRowsAlgorithm = null;

/** After a slider was last moved, polls leave it alone this long - long
 *  enough for the value to reach the algorithm and come back. */
const PARAMETER_EDIT_GRACE_MS = 1000;
/** Minimum time between two sends while a slider is being dragged. */
const PARAMETER_SEND_INTERVAL_MS = 100;

/** What the rendered sliders were built for - the selected algorithm and its
 *  parameters' declarations, without their values - so they're only rebuilt
 *  (losing a drag in progress) when that changes. */
let parameterRowsKey = null;
/** Parameter name -> `{input, valueEl, parameter, lastEditMs, held}`. */
let parameterRows = new Map();

window.addEventListener("pointerup", () => parameterRows.forEach((row) => (row.held = false)));
window.addEventListener("pointercancel", () => parameterRows.forEach((row) => (row.held = false)));

/** `{min, max, step}` of a parameter, whatever its kind (`float`/`int`). */
function parameterRange(parameter) {
  return Object.values(parameter.kind)[0];
}

function formatParameterValue(parameter, value) {
  const decimals = (String(parameterRange(parameter).step).split(".")[1] ?? "").length;
  const unit = parameter.unit ? ` ${parameter.unit}` : "";
  return `${Number(value).toFixed(decimals)}${unit}`;
}

/** Calls `fn` with the latest value at most once per `ms`, always ending on
 *  the last one - so dragging sends a steady trickle, and where it's
 *  released is never lost. */
function throttleLatest(fn, ms) {
  let lastCallMs = -Infinity;
  let timer = null;
  let latest;
  return (value) => {
    latest = value;
    if (timer !== null) return;
    const wait = Math.max(0, lastCallMs + ms - performance.now());
    timer = setTimeout(() => {
      timer = null;
      lastCallMs = performance.now();
      fn(latest);
    }, wait);
  };
}

function buildParameterRow(algorithm, parameter) {
  const { min, max, step } = parameterRange(parameter);
  const rowEl = document.createElement("div");
  rowEl.className = "parameter-row";

  const headEl = document.createElement("div");
  headEl.className = "parameter-head";
  const nameEl = document.createElement("span");
  nameEl.className = "parameter-name";
  nameEl.textContent = parameter.name;
  const valueEl = document.createElement("span");
  valueEl.className = "parameter-value";
  headEl.append(nameEl, valueEl);

  const input = document.createElement("input");
  input.type = "range";
  input.min = min;
  input.max = max;
  input.step = step;

  const descriptionEl = document.createElement("p");
  descriptionEl.className = "parameter-description";
  descriptionEl.textContent = parameter.description;

  rowEl.append(headEl, input, descriptionEl);

  const row = { input, valueEl, parameter, lastEditMs: -Infinity, held: false };
  const send = throttleLatest((value) => {
    fetchJSON("/api/autonomous_parameter", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ algorithm, name: parameter.name, value }),
    }).catch((err) => console.error(err));
  }, PARAMETER_SEND_INTERVAL_MS);
  input.addEventListener("pointerdown", () => (row.held = true));
  input.addEventListener("input", () => {
    row.lastEditMs = performance.now();
    valueEl.textContent = formatParameterValue(parameter, input.value);
    send(Number(input.value));
  });
  parameterRows.set(parameter.name, row);
  return rowEl;
}

/** Shows sliders for `selected`'s parameters (none if nothing is selected),
 *  refreshing their values from the last poll unless one is being edited. */
function syncAlgorithmParameters(selected) {
  const parameters = algorithmOptions.find((o) => o.name === selected)?.parameters ?? [];
  const key = JSON.stringify([selected, parameters.map(({ value, ...declaration }) => declaration)]);
  if (key !== parameterRowsKey) {
    parameterRowsKey = key;
    parameterRowsAlgorithm = selected;
    parameterRows = new Map();
    algorithmParametersEl.replaceChildren(...parameters.map((p) => buildParameterRow(selected, p)));
    algorithmParametersEl.hidden = parameters.length === 0;
    algorithmSaveEl.hidden = parameters.length === 0;
    algorithmSaveStatusEl.textContent = "";
  }

  const now = performance.now();
  for (const parameter of parameters) {
    const row = parameterRows.get(parameter.name);
    if (row.held || now - row.lastEditMs < PARAMETER_EDIT_GRACE_MS) continue;
    row.input.value = parameter.value;
    row.valueEl.textContent = formatParameterValue(parameter, parameter.value);
  }
}

// Saves the values the algorithm currently runs with - what the sliders
// show once a move has come back - into its config file.
algorithmSaveBtn.addEventListener("click", async () => {
  algorithmSaveBtn.disabled = true;
  algorithmSaveStatusEl.classList.remove("error");
  try {
    const { path } = await fetchJSON("/api/autonomous_parameters_save", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ algorithm: parameterRowsAlgorithm }),
    });
    algorithmSaveStatusEl.textContent = `Saved to ${path} - used from the next restart (R).`;
  } catch (err) {
    algorithmSaveStatusEl.classList.add("error");
    algorithmSaveStatusEl.textContent = `Not saved: ${err.message}`;
  } finally {
    algorithmSaveBtn.disabled = false;
  }
});

// ---------------------------------------------------------------------
// Mapping panel - drives SLAM through the `slam_command` topic (Play:
// running, Pause: waiting, Clear: off) and shows what it's actually doing,
// from the `slam_status` topic.
// ---------------------------------------------------------------------

const slamStateEl = document.getElementById("slam-state");
const slamDetailsEl = document.getElementById("slam-details");
const slamButtons = document.querySelectorAll("#slam-controls button");

/** `slam_status` older than this means SLAM isn't running at all. */
const SLAM_STATUS_STALE_MS = 1000;

const SLAM_STATE_LABELS = { off: "Off", waiting: "Waiting", running: "Running" };

for (const button of slamButtons) {
  button.addEventListener("click", () => {
    fetchJSON("/api/slam_command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ state: button.dataset.state }),
    })
      .then(() => pollSlam())
      .catch((err) => console.error(err));
  });
}

// Polls the `slam_status` topic (via `/api/slam`): the state SLAM is
// actually in - whoever asked for it, this tab or another - and how far the
// map has got.
async function pollSlam() {
  const response = await fetchJSON("/api/slam");
  const status = response.value;
  const stale = response.age_ms === null || response.age_ms > SLAM_STATUS_STALE_MS;

  slamStateEl.className = stale ? "stale" : status.state;
  slamStateEl.textContent = stale ? "SLAM not running" : SLAM_STATE_LABELS[status.state];
  for (const button of slamButtons) {
    button.disabled = !stale && button.dataset.state === status.state;
  }

  if (stale) {
    slamDetailsEl.textContent = "";
    return;
  }
  const parts = [`${status.scans} scan${status.scans === 1 ? "" : "s"} in the map.`];
  if (status.last_match_response !== null) {
    parts.push(`Last match: ${(status.last_match_response * 100).toFixed(0)}%`);
  }
  if (status.last_process_ms !== null) {
    parts.push(`in ${status.last_process_ms.toFixed(1)} ms.`);
  }
  parts.push(`${status.loop_closures} loop closure${status.loop_closures === 1 ? "" : "s"}`);
  if (status.last_optimization_ms !== null) {
    parts.push(`(last optimized in ${status.last_optimization_ms.toFixed(1)} ms)`);
  }
  slamDetailsEl.textContent = parts.join(" ") + ".";
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
  if (!document.getElementById("panel-layers").hidden) drawLayers.renderFreshness();
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

/** How often the current map, vehicle model, autonomous algorithm, and
 *  SLAM state are re-read, to reflect changes made from another tab. */
const SELECTION_POLL_MS = 500;

const drawPoller = startPolling(drawLayers.poll, 1000 / drawRateHz);
const topicPoller = startPolling(pollSelectedTopic, 1000 / DEFAULT_TOPIC_RATE_HZ);
startPolling(pollLiveMap, SELECTION_POLL_MS);
startPolling(pollVehicleModel, SELECTION_POLL_MS);
startPolling(pollAlgorithms, SELECTION_POLL_MS);
startPolling(pollSlam, SELECTION_POLL_MS);
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
