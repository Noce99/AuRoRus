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
    const origin =
      map.seed != null
        ? `seed ${map.seed}`
        : `${map.source === "imported" ? "imported" : "recorded"} ${map.generated_at}`;
    li.title = `${map.width_px}x${map.height_px} px, ${origin}`;
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
  // A line half-drawn on the old map means nothing on the new one.
  disarmStartLine();
  startLineBtn.disabled = !liveMapName;
  mapEditBtn.disabled = !liveMapName;
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
  syncVehicleModelParameters(live.kind, live.parameters);
  syncVehicleLimits("limits", live.limits);
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
const algorithmMessageEl = document.getElementById("algorithm-message");
const algorithmStatsEl = document.getElementById("algorithm-stats");
const algorithmStartBtn = document.getElementById("algorithm-start-btn");
const algorithmPauseBtn = document.getElementById("algorithm-pause-btn");

/** Algorithm the `autonomous_algorithm_status` topic last reported selected,
 *  or null before the first poll - tracked separately from the `<select>`'s
 *  own value for the same reason as `liveVehicleModelKind`. */
let liveAlgorithm = null;

/** Whether the selected algorithm was last reported running (not paused). */
let algorithmRunning = false;

/** `{name, label, description, parameters, message, stats}` of every algorithm last reported available. */
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

  // What the selected algorithm itself says, e.g. why it holds the vehicle.
  const message = algorithmOptions.find((o) => o.name === status.selected)?.message ?? null;
  algorithmMessageEl.hidden = message === null;
  algorithmMessageEl.textContent = message ?? "";

  // Its live figures, e.g. the solve time - only while it's in control.
  const stats = algorithmRunning ? (algorithmOptions.find((o) => o.name === status.selected)?.stats ?? null) : null;
  algorithmStatsEl.hidden = stats === null;
  algorithmStatsEl.textContent = stats ?? "";

  syncAlgorithmParameters(status.selected);
  focusAlgorithmDrawing(status.selected, status.available);
}

/** What `focusAlgorithmDrawing` last applied - the selected algorithm and
 *  every available one - so it only acts when that changes. */
let focusedAlgorithmDrawing = null;

/** Shows the selected algorithm's drawing (`draw/<name>`) and hides every
 *  other algorithm's, whenever the selection (or the set of algorithms)
 *  changes. In between, the layer list is the user's to tick and untick. */
function focusAlgorithmDrawing(selected, available) {
  const topics = available.map((algorithm) => `draw/${algorithm.name}`);
  const signature = JSON.stringify([selected, topics]);
  if (signature === focusedAlgorithmDrawing) return;
  focusedAlgorithmDrawing = signature;
  drawLayers.focus(selected === null ? null : `draw/${selected}`, topics);
}

// ---------------------------------------------------------------------
// Live parameter tuning - one slider per parameter the owner (the selected
// algorithm, or the running vehicle model) declares. A slider sends the
// wanted value to the server; what it then shows comes back from the owner
// itself (via the status it's polled from), so it always reflects the value
// actually in effect. Save writes those values into the owner's config file.
// ---------------------------------------------------------------------

/** After a slider was last moved, polls leave it alone this long - long
 *  enough for the value to reach its owner and come back. */
const PARAMETER_EDIT_GRACE_MS = 1000;
/** Minimum time between two sends while a slider is being dragged. */
const PARAMETER_SEND_INTERVAL_MS = 100;

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

/** A slider panel for one kind of owner, built inside `containerEl`, with
 *  its Save/Load buttons and status in `saveEl`. `setUrl`/`saveUrl`/`loadUrl`
 *  are the endpoints a slider move/Save click/Load click POST to,
 *  identifying the owner as `ownerKey` (e.g. `{algorithm: ...}`) unless
 *  there's none to name, plus `name`/`value` for a move. Returns its
 *  `sync(owner, parameters)`, to call on every poll. */
function createParameterPanel({ containerEl, saveEl, setUrl, saveUrl, loadUrl, ownerKey = null }) {
  const saveBtn = saveEl.querySelector(".parameter-save-btn");
  const loadBtn = saveEl.querySelector(".parameter-load-btn");
  const saveStatusEl = saveEl.querySelector("p");
  const ownerBody = (owner) => (ownerKey ? { [ownerKey]: owner } : {});

  /** Owner the sliders (and the Save button) are for, or null for none. */
  let rowsOwner = null;
  /** What the rendered sliders were built for - the owner and its
   *  parameters' declarations, without their values - so they're only
   *  rebuilt (losing a drag in progress) when that changes. */
  let rowsKey = null;
  /** Parameter name -> `{input, valueEl, parameter, lastEditMs, held}`. */
  let rows = new Map();

  window.addEventListener("pointerup", () => rows.forEach((row) => (row.held = false)));
  window.addEventListener("pointercancel", () => rows.forEach((row) => (row.held = false)));

  function buildRow(owner, parameter) {
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
      fetchJSON(setUrl, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ ...ownerBody(owner), name: parameter.name, value }),
      }).catch((err) => console.error(err));
    }, PARAMETER_SEND_INTERVAL_MS);
    input.addEventListener("pointerdown", () => (row.held = true));
    input.addEventListener("input", () => {
      row.lastEditMs = performance.now();
      valueEl.textContent = formatParameterValue(parameter, input.value);
      send(Number(input.value));
    });
    rows.set(parameter.name, row);
    return rowEl;
  }

  /** POSTs the owner to `url` with both buttons disabled, reporting the
   *  outcome in the status line via `done(path)`/`failed`. */
  async function fileAction(url, done, failed) {
    saveBtn.disabled = loadBtn.disabled = true;
    saveStatusEl.classList.remove("error");
    try {
      const { path } = await fetchJSON(url, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(ownerBody(rowsOwner)),
      });
      saveStatusEl.textContent = done(path);
    } catch (err) {
      saveStatusEl.classList.add("error");
      saveStatusEl.textContent = `${failed}: ${err.message}`;
    } finally {
      saveBtn.disabled = loadBtn.disabled = false;
    }
  }

  // Saves the values the owner currently runs with - what the sliders show
  // once a move has come back - into its config file.
  saveBtn.addEventListener("click", () =>
    fileAction(saveUrl, (path) => `Saved to ${path} - used from the next restart (R).`, "Not saved"),
  );

  // Makes the owner run with its config file's values again; the sliders
  // follow once they come back, so any edit grace is dropped.
  loadBtn.addEventListener("click", () => {
    rows.forEach((row) => (row.lastEditMs = -Infinity));
    fileAction(loadUrl, (path) => `Loaded from ${path}.`, "Not loaded");
  });

  /** Shows sliders for `owner`'s `parameters` (none if there's no owner),
   *  refreshing their values from the last poll unless one is being edited. */
  return function sync(owner, parameters) {
    const key = JSON.stringify([owner, parameters.map(({ value, ...declaration }) => declaration)]);
    if (key !== rowsKey) {
      rowsKey = key;
      rowsOwner = owner;
      rows = new Map();
      containerEl.replaceChildren(...parameters.map((p) => buildRow(owner, p)));
      containerEl.hidden = parameters.length === 0;
      saveEl.hidden = parameters.length === 0;
      saveStatusEl.textContent = "";
    }

    const now = performance.now();
    for (const parameter of parameters) {
      const row = rows.get(parameter.name);
      if (row.held || now - row.lastEditMs < PARAMETER_EDIT_GRACE_MS) continue;
      row.input.value = parameter.value;
      row.valueEl.textContent = formatParameterValue(parameter, parameter.value);
    }
  };
}

const syncAlgorithmParameterPanel = createParameterPanel({
  containerEl: document.getElementById("algorithm-parameters"),
  saveEl: document.getElementById("algorithm-save"),
  setUrl: "/api/autonomous_parameter",
  saveUrl: "/api/autonomous_parameters_save",
  loadUrl: "/api/autonomous_parameters_load",
  ownerKey: "algorithm",
});

/** Shows sliders for the `selected` algorithm's parameters (none if nothing is selected). */
function syncAlgorithmParameters(selected) {
  const parameters = algorithmOptions.find((o) => o.name === selected)?.parameters ?? [];
  syncAlgorithmParameterPanel(selected, parameters);
}

const syncVehicleModelParameters = createParameterPanel({
  containerEl: document.getElementById("vehicle-model-parameters"),
  saveEl: document.getElementById("vehicle-model-save"),
  setUrl: "/api/vehicle_model_parameter",
  saveUrl: "/api/vehicle_model_parameters_save",
  loadUrl: "/api/vehicle_model_parameters_load",
  ownerKey: "kind",
});

const syncVehicleLimits = createParameterPanel({
  containerEl: document.getElementById("vehicle-limits-parameters"),
  saveEl: document.getElementById("vehicle-limits-save"),
  setUrl: "/api/vehicle_limit",
  saveUrl: "/api/vehicle_limits_save",
  loadUrl: "/api/vehicle_limits_load",
});

// ---------------------------------------------------------------------
// Mapping panel - drives SLAM through the `slam_command` topic (Play:
// running, Pause: waiting, Clear: off), saves its map through the
// `slam_save` topic, and shows what it's actually doing, from the
// `slam_status` topic.
// ---------------------------------------------------------------------

const slamStateEl = document.getElementById("slam-state");
const slamDetailsEl = document.getElementById("slam-details");
const slamButtons = document.querySelectorAll("#slam-controls button");
const slamSaveFormEl = document.getElementById("slam-save");
const slamSaveNameEl = document.getElementById("slam-save-name");
const slamSaveBtn = document.getElementById("slam-save-btn");
const slamSaveStatusEl = document.getElementById("slam-save-status");

/** The save request this tab is waiting on SLAM to answer, or null. */
let pendingSlamSave = null;
/** Whether SLAM currently has anything to save. */
let slamHasMap = false;

/** `slam_status` older than this means SLAM isn't running at all. */
const SLAM_STATUS_STALE_MS = 1000;

const SLAM_STATE_LABELS = {
  off: "Off",
  waiting: "Waiting",
  running: "Running",
  localizing: "Localizing",
  localization_paused: "Localization paused",
};

const localizationStateEl = document.getElementById("localization-state");
const localizationDetailsEl = document.getElementById("localization-details");
const localizationStartBtn = document.getElementById("localization-start-btn");
const localizationPauseBtn = document.getElementById("localization-pause-btn");

for (const button of [...slamButtons, localizationStartBtn, localizationPauseBtn]) {
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

// SLAM saves on its own thread: the POST only files the request, and the
// outcome shows up on `slam_status` under the returned request number.
slamSaveFormEl.addEventListener("submit", (event) => {
  event.preventDefault();
  const name = slamSaveNameEl.value.trim();
  if (!name) return;
  slamSaveBtn.disabled = true;
  slamSaveStatusEl.classList.remove("error");
  slamSaveStatusEl.textContent = "Saving...";
  fetchJSON("/api/slam_save", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ name }),
  })
    .then(({ requested }) => {
      pendingSlamSave = requested;
      return pollSlam();
    })
    .catch((err) => {
      slamSaveStatusEl.classList.add("error");
      slamSaveStatusEl.textContent = `Couldn't save: ${err.message}`;
      slamSaveBtn.disabled = !slamHasMap;
    });
});

/** Shows how SLAM handled this tab's pending save request, once it has. */
function showSlamSaveOutcome(outcome) {
  if (pendingSlamSave === null || !outcome || outcome.requested !== pendingSlamSave) return;
  pendingSlamSave = null;
  if (outcome.error !== null) {
    slamSaveStatusEl.classList.add("error");
    slamSaveStatusEl.textContent = `Couldn't save: ${outcome.error}`;
  } else {
    slamSaveStatusEl.textContent = `Saved to ${outcome.saved_to}`;
    slamSaveNameEl.value = "";
    refreshMapList().catch((err) => console.error(err));
  }
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
  renderLocalization(status, stale);

  if (!stale) showSlamSaveOutcome(status.last_save);
  slamHasMap = !stale && status.scans > 0;
  slamSaveBtn.disabled = !slamHasMap || pendingSlamSave !== null;
  if (stale) {
    slamDetailsEl.textContent = "";
    return;
  }
  if (status.state === "localizing" || status.state === "localization_paused") {
    slamDetailsEl.textContent = "Localizing on the selected map - see the Localization panel.";
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

// Localization panel - the localization half of `slam_status`. Start is
// only offered with a map selected, and never over a map SLAM is building:
// that one has to be cleared or saved first.
function renderLocalization(status, stale) {
  const localizing = !stale && status.state === "localizing";
  const paused = !stale && status.state === "localization_paused";
  const hasSlamMap = !stale && status.scans > 0;

  localizationStateEl.className = stale ? "stale" : status.state;
  localizationStateEl.textContent = stale
    ? "SLAM not running"
    : localizing
      ? "Localizing"
      : paused
        ? "Paused"
        : "Not localizing";
  localizationStartBtn.disabled = stale || localizing || hasSlamMap || liveMapName === null;
  localizationPauseBtn.disabled = !localizing;

  if (stale) {
    localizationDetailsEl.textContent = "";
  } else if (hasSlamMap) {
    localizationDetailsEl.textContent =
      "SLAM has a map in memory: clear or save it in the Mapping panel first.";
  } else if (liveMapName === null) {
    localizationDetailsEl.textContent = "Select a map to localize on.";
  } else if ((localizing || paused) && status.pose !== null) {
    const [x, y, heading] = status.pose;
    const parts = [
      `On ${liveMapName} at (${x.toFixed(2)}, ${y.toFixed(2)}) m,`,
      `heading ${((heading * 180) / Math.PI).toFixed(1)}°.`,
    ];
    if (status.last_match_response !== null) {
      parts.push(`Last match: ${(status.last_match_response * 100).toFixed(0)}%`);
    }
    if (status.last_process_ms !== null) {
      parts.push(`in ${status.last_process_ms.toFixed(1)} ms.`);
    }
    localizationDetailsEl.textContent = parts.join(" ");
  } else if (localizing || paused) {
    localizationDetailsEl.textContent = `On ${liveMapName}, waiting for the first scan.`;
  } else {
    localizationDetailsEl.textContent = `Tracks the car on ${liveMapName} from its start line, where odometry was last reset.`;
  }
}

// ---------------------------------------------------------------------
// Detector panel - shows what `UbmDetector` (ubm's detector_py) found in the
// ego vehicle's latest scan and why it isn't detecting, from `/api/detector`
// (the `detector_status` and `detected_opponent` topics), and tunes it
// through `detector_parameters`. The opponent and its bounding box are
// drawn by the detector itself (see the Layers panel).
// ---------------------------------------------------------------------

const detectorStateEl = document.getElementById("detector-state");
const detectorDetailsEl = document.getElementById("detector-details");
const detectorOpponentEl = document.getElementById("detector-opponent");

const syncDetectorParameters = createParameterPanel({
  containerEl: document.getElementById("detector-parameters"),
  saveEl: document.getElementById("detector-save"),
  setUrl: "/api/detector_parameter",
  saveUrl: "/api/detector_parameters_save",
  loadUrl: "/api/detector_parameters_load",
});

/** The detected opponent, in words. */
function describeDetectedOpponent(opponent) {
  const [x, y] = opponent.position;
  const [vx, vy] = opponent.velocity;
  const parts = [
    `At (${x.toFixed(2)}, ${y.toFixed(2)}) m, moving at ${Math.hypot(vx, vy).toFixed(2)} m/s.`,
  ];
  const box = opponent.bounding_box;
  if (box !== null) {
    parts.push(
      `Box ${box.length_m.toFixed(2)} x ${box.width_m.toFixed(2)} m,`,
      `turned ${((box.heading_rad * 180) / Math.PI).toFixed(0)}°.`,
    );
  }
  return parts.join(" ");
}

// Polls `/api/detector`. The detector writes its status after every scan,
// or when it stops detecting - only a status never written at all means
// there's no detector.
async function pollDetector() {
  const response = await fetchJSON("/api/detector");
  const { status, opponent } = response.value;
  const missing = status === null || response.age_ms === null;
  const waiting = !missing && status.message !== null;

  detectorStateEl.className = missing ? "stale" : waiting ? "waiting" : opponent.detected ? "detected" : "";
  detectorStateEl.textContent = missing
    ? "Detector not running"
    : waiting
      ? "Waiting"
      : opponent.detected
        ? "Opponent detected"
        : "No opponent in sight";

  if (missing) {
    detectorDetailsEl.textContent = "";
  } else if (waiting) {
    detectorDetailsEl.textContent = status.message;
  } else {
    detectorDetailsEl.textContent =
      `Compares every lidar scan with the one the map alone would give - last scan took ${status.scan_ms.toFixed(1)} ms.`;
  }
  detectorOpponentEl.textContent = !missing && !waiting && opponent.detected ? describeDetectedOpponent(opponent) : "";

  syncDetectorParameters("detector", missing ? [] : status.parameters);
}

// ---------------------------------------------------------------------
// Planning panel - asks the planner for a race line for the selected map
// through the `planning_request` topic, tunes it through
// `planning_parameters` (applied right away, so before starting), and
// shows what it's doing and how its latest request went, from the
// `planning_status` topic. The race line itself is drawn by `MapServer`
// once saved.
// ---------------------------------------------------------------------

const planningStateEl = document.getElementById("planning-state");
const planningDetailsEl = document.getElementById("planning-details");
const planningOutcomeEl = document.getElementById("planning-outcome");
const planningStartBtn = document.getElementById("planning-start-btn");
const planningObjectiveEl = document.getElementById("planning-objective");

/** Where this browser remembers the picked objective. */
const PLANNING_OBJECTIVE_KEY = "aurorus.planning.objective";
try {
  const saved = localStorage.getItem(PLANNING_OBJECTIVE_KEY);
  if (saved && [...planningObjectiveEl.options].some((o) => o.value === saved)) {
    planningObjectiveEl.value = saved;
  }
} catch {
  // No storage (e.g. a private window): the default objective it is.
}
planningObjectiveEl.addEventListener("change", () => {
  try {
    localStorage.setItem(PLANNING_OBJECTIVE_KEY, planningObjectiveEl.value);
  } catch {
    // Not remembered - it still applies to this page.
  }
});

const syncPlanningParameters = createParameterPanel({
  containerEl: document.getElementById("planning-parameters"),
  saveEl: document.getElementById("planning-save"),
  setUrl: "/api/planning_parameter",
  saveUrl: "/api/planning_parameters_save",
  loadUrl: "/api/planning_parameters_load",
});

/** Whether the planner was last reported planning. */
let planningComputing = false;

planningStartBtn.addEventListener("click", () => {
  planningStartBtn.disabled = true;
  planningOutcomeEl.classList.remove("error");
  planningOutcomeEl.textContent = "";
  fetchJSON("/api/planning_start", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ objective: planningObjectiveEl.value }),
  })
    .then(() => pollPlanning())
    .catch((err) => {
      planningOutcomeEl.classList.add("error");
      planningOutcomeEl.textContent = `Couldn't start: ${err.message}`;
      planningStartBtn.disabled = planningComputing || liveMapName === null;
    });
});

/** The last outcome, in words. */
function describePlanningOutcome(outcome) {
  const map = outcome.map ? outcome.map.split("/").pop() : "no map";
  if (outcome.error !== null) return `Planning for ${map} failed: ${outcome.error}`;
  const parts = [
    `Race line for ${map} saved to ${outcome.saved_to} (${(outcome.elapsed_ms / 1000).toFixed(1)} s).`,
    `Lap: ${outcome.lap_length_m.toFixed(1)} m in ${outcome.lap_time_s.toFixed(2)} s.`,
    `Max curvature ${outcome.max_curvature_per_m.toFixed(2)} 1/m`,
    `(centerline: ${outcome.reference_max_curvature_per_m.toFixed(2)} 1/m).`,
  ];
  if (outcome.computed_centerline) {
    parts.push("The map had no centerline: one was computed from its walls and saved too.");
  }
  if (outcome.objective === "min_time") {
    if (outcome.min_time_error !== null) {
      parts.push(`Minimum time failed: ${outcome.min_time_error}`);
    } else if (outcome.min_time_saved_to !== null) {
      const gain = (1 - outcome.min_time_lap_time_s / outcome.lap_time_s) * 100;
      parts.push(
        `Minimum-time line saved to ${outcome.min_time_saved_to}:`,
        `lap ${outcome.min_time_lap_length_m.toFixed(1)} m in ${outcome.min_time_lap_time_s.toFixed(2)} s`,
        `(${gain.toFixed(1)}% faster than minimum curvature).`,
      );
    }
  }
  return parts.join(" ");
}

// Polls the `planning_status` topic (via `/api/planning`). The planner only
// writes it when something changes, so its age says nothing about whether
// it's alive - only a status never written at all means there's no planner.
async function pollPlanning() {
  const response = await fetchJSON("/api/planning");
  const status = response.value;
  const missing = response.age_ms === null;
  planningComputing = !missing && status.state === "computing";

  planningStateEl.className = missing ? "stale" : status.state;
  planningStateEl.textContent = missing ? "Planner not running" : planningComputing ? "Computing" : "Idle";
  planningStartBtn.disabled = missing || planningComputing || liveMapName === null;
  planningObjectiveEl.disabled = planningComputing;

  if (missing) {
    planningDetailsEl.textContent = "";
  } else if (planningComputing) {
    planningDetailsEl.textContent = `${status.stage}...`;
  } else if (liveMapName === null) {
    planningDetailsEl.textContent = "Select a map to plan a race line for.";
  } else {
    planningDetailsEl.textContent =
      planningObjectiveEl.value === "min_time"
        ? `Plans the minimum-curvature race line for ${liveMapName}, then the minimum-time line from it.`
        : `Plans a minimum-curvature race line for ${liveMapName}, with a speed profile.`;
  }

  const outcome = status.last_outcome;
  planningOutcomeEl.classList.toggle(
    "error",
    outcome !== null && (outcome.error !== null || outcome.min_time_error !== null),
  );
  if (outcome !== null) planningOutcomeEl.textContent = describePlanningOutcome(outcome);

  syncPlanningParameters("planner", status.parameters);
}

// ---------------------------------------------------------------------
// Race Lines panel - every race line of the loaded map, newest first (from
// `/api/race_lines`, read off its folder), with the one `MapServer`
// publishes on `race_line` highlighted. Clicking one writes
// `race_line_selection`; `MapServer` switches to it on its next poll.
// ---------------------------------------------------------------------

const raceLineListEl = document.getElementById("race-line-list");
const raceLinesDetailsEl = document.getElementById("race-lines-details");

const RACE_LINE_METHOD_LABELS = {
  centerline: "Centerline",
  min_curvature: "Minimum curvature",
  min_time: "Minimum time",
  unknown: "Unknown method",
};

/** What the list last showed, so it's only rebuilt when that changes. */
let raceLinesSignature = null;

function raceLinesPanelVisible() {
  return !document.getElementById("panel-race-lines").hidden;
}

function formatLapTime(seconds) {
  return Number.isFinite(seconds) ? `${seconds.toFixed(2)} s` : "∞ s";
}

async function selectRaceLine(file) {
  await fetchJSON("/api/race_line_selection", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ file }),
  });
}

function renderRaceLines(response, selected) {
  raceLineListEl.innerHTML = "";
  raceLinesDetailsEl.textContent =
    response.map === null
      ? "Select a map to see its race lines."
      : `Race lines of ${response.map}, newest first. Click one to follow it.`;
  if (response.map !== null && response.lines.length === 0) {
    const li = document.createElement("li");
    li.className = "empty";
    li.textContent = "No race lines yet - plan one in the Planning panel";
    raceLineListEl.appendChild(li);
  }
  for (const line of response.lines) {
    const li = document.createElement("li");
    if (line.file === selected) li.classList.add("selected");
    const name = document.createElement("span");
    name.className = "race-line-name";
    name.textContent = line.file.replace(/\.csv$/, "");
    const stats = document.createElement("span");
    stats.className = "race-line-stats";
    stats.textContent = [
      RACE_LINE_METHOD_LABELS[line.method] ?? line.method,
      formatLapTime(line.lap_time_s),
      `${line.lap_length_m.toFixed(1)} m`,
    ].join(" · ");
    li.title = `${line.file}, ${line.num_points} points`;
    li.append(name, stats);
    li.addEventListener("click", () => {
      renderRaceLines(response, line.file); // optimistic highlight; the next poll confirms it
      selectRaceLine(line.file).catch((err) => console.error(err));
    });
    raceLineListEl.appendChild(li);
  }
}

// Reads the list off disk, so only while the panel is visible.
async function pollRaceLines() {
  if (!raceLinesPanelVisible()) return;
  const response = await fetchJSON("/api/race_lines");
  const signature = JSON.stringify(response);
  if (signature === raceLinesSignature) return;
  raceLinesSignature = signature;
  renderRaceLines(response, response.selected);
}

// Show it right away when the panel is opened, not a poll later - once
// `MapView`'s own click handler has unhidden the panel.
document.querySelector('.panel-nav-btn[data-panel="race-lines"]').addEventListener("click", () => {
  raceLinesSignature = null;
  setTimeout(() => pollRaceLines().catch((err) => console.error(err)), 0);
});

// ---------------------------------------------------------------------
// Opponents panel - the other autonomous vehicles `OpponentsManager` runs
// (`/api/opponents`), each deletable, and a "+" opening a form to add one.
// Adding or deleting only queues a request: the list shows the outcome once
// the manager has handled it, a poll later.
// ---------------------------------------------------------------------

const opponentListEl = document.getElementById("opponent-list");
const opponentsErrorEl = document.getElementById("opponents-error");
const opponentOverlay = document.getElementById("opponent-overlay");
const opponentForm = document.getElementById("opponent-form");
const opponentColorsEl = document.getElementById("opponent-colors");
const opponentAlgorithmEl = document.getElementById("opponent-algorithm");
const opponentRaceLineEl = document.getElementById("opponent-race-line");
const opponentSpeedEl = document.getElementById("opponent-speed");
const opponentSpeedValueEl = document.getElementById("opponent-speed-value");
const opponentLimitsEl = document.getElementById("opponent-limits");
const opponentHintEl = document.getElementById("opponent-form-hint");
const opponentFormErrorEl = document.getElementById("opponent-form-error");
const opponentConfirmBtn = document.getElementById("opponent-confirm-btn");

/** The algorithm a new opponent runs unless the user picks another. */
const DEFAULT_OPPONENT_ALGORITHM = "gap_follower";
/** The color a new opponent gets unless the user picks another. */
const DEFAULT_OPPONENT_COLOR = "red";

/** What the list last showed, so it's only rebuilt when that changes. */
let opponentsSignature = null;
/** The number of the latest request queued from this page, whose outcome
 *  is reported once `last_outcome` answers it. */
let awaitedOpponentRequest = null;
/** The latest `/api/opponents` response - what the form offers. */
let opponentChoices = null;
/** Whether the user picked a race line in the open form - until then, it
 *  follows the picked algorithm (see `defaultOpponentRaceLine`). */
let opponentRaceLinePicked = false;

function opponentsPanelVisible() {
  return !document.getElementById("panel-opponents").hidden;
}

function showOpponentsError(message) {
  opponentsErrorEl.textContent = message;
  opponentsErrorEl.hidden = message === null;
}

function paletteCss(response, name) {
  return response.palette.find((color) => color.name === name)?.css ?? "#888";
}

async function deleteOpponent(id) {
  showOpponentsError(null);
  try {
    const { request } = await fetchJSON("/api/opponents/delete", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ id }),
    });
    awaitedOpponentRequest = request;
  } catch (err) {
    showOpponentsError(err.message);
  }
  await pollOpponents();
}

function renderOpponents(response) {
  opponentListEl.innerHTML = "";
  if (response.list.length === 0) {
    const li = document.createElement("li");
    li.className = "empty";
    li.textContent = "No opponents - add one with +";
    opponentListEl.appendChild(li);
  }
  for (const opponent of response.list) {
    const { spec } = opponent;
    const li = document.createElement("li");
    const swatch = document.createElement("span");
    swatch.className = "opponent-swatch";
    swatch.style.background = paletteCss(response, spec.color);
    const text = document.createElement("span");
    text.className = "opponent-text";
    const name = document.createElement("span");
    name.className = "opponent-name";
    name.textContent = `#${opponent.id} ${opponent.algorithm_label}`;
    const stats = document.createElement("span");
    stats.className = "opponent-stats";
    stats.textContent = [
      spec.race_line === null ? "No race line" : spec.race_line.replace(/\.csv$/, ""),
      `speed ×${spec.speed_scale.toFixed(2)}`,
      `max ${spec.limits.max_speed_mps.toFixed(1)} m/s`,
    ].join(" · ");
    text.append(name, stats);
    const deleteBtn = document.createElement("button");
    deleteBtn.type = "button";
    deleteBtn.className = "opponent-delete-btn";
    deleteBtn.textContent = "✕";
    deleteBtn.title = `Delete opponent #${opponent.id}`;
    deleteBtn.setAttribute("aria-label", deleteBtn.title);
    deleteBtn.addEventListener("click", () => {
      deleteBtn.disabled = true;
      deleteOpponent(opponent.id);
    });
    li.append(swatch, text, deleteBtn);
    opponentListEl.appendChild(li);
  }
}

async function pollOpponents() {
  if (!opponentsPanelVisible()) return;
  const response = await fetchJSON("/api/opponents");
  opponentChoices = response;
  updateRace(response);
  const outcome = response.last_outcome;
  if (outcome !== null && outcome.request === awaitedOpponentRequest) {
    awaitedOpponentRequest = null;
    showOpponentsError(outcome.error);
  }
  const signature = JSON.stringify([response.list, response.palette]);
  if (signature === opponentsSignature) return;
  opponentsSignature = signature;
  renderOpponents(response);
}

document.querySelector('.panel-nav-btn[data-panel="opponents"]').addEventListener("click", () => {
  opponentsSignature = null;
  setTimeout(() => pollOpponents().catch((err) => console.error(err)), 0);
});

// --- the "add" form ---

function optionEl(value, label) {
  const option = document.createElement("option");
  option.value = value;
  option.textContent = label;
  return option;
}

/** The algorithm picked in the form, as `/api/opponents` describes it. */
function pickedOpponentAlgorithm() {
  return opponentChoices?.algorithms.find((a) => a.name === opponentAlgorithmEl.value) ?? null;
}

/** Why the form can't be submitted as it is, or null if it can - the same
 *  rule the server enforces. */
function opponentFormProblem() {
  const algorithm = pickedOpponentAlgorithm();
  if (algorithm === null) return "There's no autonomous algorithm to run.";
  if (opponentLimitsEl.childElementCount === 0) return "There's no ego vehicle to copy the model of.";
  if (algorithm.requires.race_line && opponentRaceLineEl.value === "") {
    return opponentChoices.race_lines.files.length === 0
      ? `${algorithm.label} follows a race line, and this map has none - plan one in the Planning panel.`
      : `${algorithm.label} follows a race line: pick one.`;
  }
  return null;
}

/** The race line a new opponent running the picked algorithm follows unless
 *  the user picks one: the ego vehicle's, for an algorithm that follows a
 *  race line - none for a reactive one, which doesn't. */
function defaultOpponentRaceLine() {
  return pickedOpponentAlgorithm()?.requires.race_line
    ? (opponentChoices.race_lines.selected ?? "")
    : "";
}

function updateOpponentForm() {
  opponentSpeedValueEl.textContent = `×${Number(opponentSpeedEl.value).toFixed(2)}`;
  const problem = opponentFormProblem();
  opponentHintEl.textContent = problem ?? "";
  opponentHintEl.hidden = problem === null;
  opponentConfirmBtn.disabled = problem !== null;
}

function buildOpponentForm(response) {
  opponentColorsEl.innerHTML = "";
  for (const color of response.palette) {
    const label = document.createElement("label");
    label.title = color.name[0].toUpperCase() + color.name.slice(1);
    const input = document.createElement("input");
    input.type = "radio";
    input.name = "opponent-color";
    input.value = color.name;
    input.checked = color.name === DEFAULT_OPPONENT_COLOR;
    input.setAttribute("aria-label", label.title);
    const swatch = document.createElement("span");
    swatch.className = "opponent-swatch";
    swatch.style.background = color.css;
    label.append(input, swatch);
    opponentColorsEl.appendChild(label);
  }

  opponentAlgorithmEl.innerHTML = "";
  for (const algorithm of response.algorithms) {
    opponentAlgorithmEl.appendChild(optionEl(algorithm.name, algorithm.label));
  }
  if (response.algorithms.some((a) => a.name === DEFAULT_OPPONENT_ALGORITHM)) {
    opponentAlgorithmEl.value = DEFAULT_OPPONENT_ALGORITHM;
  }

  opponentRaceLineEl.innerHTML = "";
  opponentRaceLineEl.appendChild(optionEl("", "None"));
  for (const file of response.race_lines.files) {
    opponentRaceLineEl.appendChild(optionEl(file, file.replace(/\.csv$/, "")));
  }
  opponentRaceLinePicked = false;
  opponentRaceLineEl.value = defaultOpponentRaceLine();

  opponentSpeedEl.value = 1;

  opponentLimitsEl.innerHTML = "";
  for (const parameter of response.limits) {
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
    input.value = parameter.value;
    input.dataset.name = parameter.name;
    const show = () => (valueEl.textContent = formatParameterValue(parameter, input.value));
    input.addEventListener("input", show);
    show();
    const descriptionEl = document.createElement("p");
    descriptionEl.className = "parameter-description";
    descriptionEl.textContent = parameter.description;
    rowEl.append(headEl, input, descriptionEl);
    opponentLimitsEl.appendChild(rowEl);
  }
  updateOpponentForm();
}

function showOpponentFormError(message) {
  opponentFormErrorEl.textContent = message ?? "";
  opponentFormErrorEl.hidden = message === null;
}

async function openOpponentForm() {
  showOpponentFormError(null);
  try {
    opponentChoices = await fetchJSON("/api/opponents");
    buildOpponentForm(opponentChoices);
    opponentOverlay.hidden = false;
  } catch (err) {
    showOpponentsError(`Couldn't open the form: ${err.message}`);
  }
}

function closeOpponentForm() {
  opponentOverlay.hidden = true;
}

document.getElementById("opponent-add-btn").addEventListener("click", openOpponentForm);
document.getElementById("opponent-cancel-btn").addEventListener("click", closeOpponentForm);
opponentAlgorithmEl.addEventListener("change", () => {
  if (!opponentRaceLinePicked) opponentRaceLineEl.value = defaultOpponentRaceLine();
  updateOpponentForm();
});
opponentRaceLineEl.addEventListener("change", () => {
  opponentRaceLinePicked = true;
  updateOpponentForm();
});
opponentSpeedEl.addEventListener("input", updateOpponentForm);

opponentForm.addEventListener("submit", async (event) => {
  event.preventDefault();
  if (opponentFormProblem() !== null) return;
  showOpponentFormError(null);
  opponentConfirmBtn.disabled = true;
  const limits = {};
  for (const input of opponentLimitsEl.querySelectorAll("input")) {
    limits[input.dataset.name] = Number(input.value);
  }
  const spec = {
    color: opponentColorsEl.querySelector("input:checked")?.value ?? DEFAULT_OPPONENT_COLOR,
    race_line: opponentRaceLineEl.value === "" ? null : opponentRaceLineEl.value,
    algorithm: opponentAlgorithmEl.value,
    speed_scale: Number(opponentSpeedEl.value),
    limits,
  };
  try {
    const { request } = await fetchJSON("/api/opponents", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(spec),
    });
    awaitedOpponentRequest = request;
    showOpponentsError(null);
    closeOpponentForm();
    await pollOpponents();
  } catch (err) {
    showOpponentFormError(err.message);
  } finally {
    updateOpponentForm();
  }
});

// --- the race start ---
// "Start race" opens a dialog ordering every racer - the opponents by id,
// then the ego vehicle - pole position first; starting lines them all up on
// the grid (`/api/race/start`) and counts down to when the server releases
// them.

const raceOpenBtn = document.getElementById("race-open-btn");
const raceOverlay = document.getElementById("race-overlay");
const raceOrderEl = document.getElementById("race-order");
const raceErrorEl = document.getElementById("race-error");
const raceStartBtn = document.getElementById("race-start-btn");
const raceCountdownEl = document.getElementById("race-countdown");

/** The ego vehicle's color - `Color::AMBER`. */
const EGO_CSS = "#ffb020";

/** The racers in the open dialog, pole position first - each `"ego"` or
 *  `{ opponent: id }`, as `/api/race/start` takes them. */
let raceOrder = [];

function racerKey(racer) {
  return racer === "ego" ? "ego" : `opponent/${racer.opponent}`;
}

function defaultRaceOrder(list) {
  const ids = list.map((opponent) => opponent.id).sort((a, b) => a - b);
  return [...ids.map((id) => ({ opponent: id })), "ego"];
}

/** `raceOrder` for the opponents in `list`: the ones still running keep
 *  their place, new ones are added last. */
function syncRaceOrder(list) {
  const running = new Map(defaultRaceOrder(list).map((racer) => [racerKey(racer), racer]));
  const kept = raceOrder.filter((racer) => running.has(racerKey(racer)));
  const keptKeys = new Set(kept.map(racerKey));
  raceOrder = [...kept, ...[...running.values()].filter((racer) => !keptKeys.has(racerKey(racer)))];
}

function showRaceError(message) {
  raceErrorEl.textContent = message ?? "";
  raceErrorEl.hidden = message === null;
}

function moveRacer(index, by) {
  const [racer] = raceOrder.splice(index, 1);
  raceOrder.splice(index + by, 0, racer);
  renderRaceOrder();
}

function renderRaceOrder() {
  raceOrderEl.innerHTML = "";
  const opponents = new Map((opponentChoices?.list ?? []).map((opponent) => [opponent.id, opponent]));
  raceOrder.forEach((racer, index) => {
    const opponent = racer === "ego" ? null : opponents.get(racer.opponent);
    const li = document.createElement("li");
    const position = document.createElement("span");
    position.className = "race-position";
    position.textContent = `P${index + 1}`;
    const swatch = document.createElement("span");
    swatch.className = "opponent-swatch";
    swatch.style.background =
      opponent === null ? EGO_CSS : paletteCss(opponentChoices, opponent?.spec.color);
    const name = document.createElement("span");
    name.className = "race-name";
    name.textContent =
      opponent === null ? "Ego vehicle" : `#${racer.opponent} ${opponent?.algorithm_label ?? ""}`;
    const moveBtn = (label, by, title) => {
      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = "race-move-btn";
      btn.textContent = label;
      btn.title = title;
      btn.setAttribute("aria-label", `${title}: ${name.textContent}`);
      btn.disabled = index + by < 0 || index + by >= raceOrder.length;
      btn.addEventListener("click", () => moveRacer(index, by));
      return btn;
    };
    li.append(
      position,
      swatch,
      name,
      moveBtn("▲", -1, "Move up"),
      moveBtn("▼", 1, "Move down"),
    );
    raceOrderEl.appendChild(li);
  });
}

/** Called on every `/api/opponents` poll: the button's state, and the open
 *  dialog's racers. */
function updateRace(response) {
  raceOpenBtn.disabled = !response.race_start.available;
  raceOpenBtn.title =
    response.race_start.reason ??
    "Line every vehicle up on the starting grid and start them all at once";
  if (raceOverlay.hidden) return;
  const before = raceOrder.map(racerKey).join();
  syncRaceOrder(response.list);
  if (raceOrder.map(racerKey).join() !== before) renderRaceOrder();
}

async function openRaceDialog() {
  showRaceError(null);
  try {
    opponentChoices = await fetchJSON("/api/opponents");
  } catch (err) {
    showOpponentsError(`Couldn't open the dialog: ${err.message}`);
    return;
  }
  raceOrder = defaultRaceOrder(opponentChoices.list);
  renderRaceOrder();
  raceStartBtn.disabled = false;
  raceOverlay.hidden = false;
}

/** Shows 3, 2, 1 - one a second - then "Go!" `goInMs` from now. */
function countDown(goInMs) {
  const goAt = performance.now() + goInMs;
  const show = (text, go) => {
    raceCountdownEl.textContent = text;
    raceCountdownEl.classList.toggle("race-go", go);
    // Restart the animation for every number.
    raceCountdownEl.classList.remove("race-tick");
    void raceCountdownEl.offsetWidth;
    raceCountdownEl.classList.add("race-tick");
    raceCountdownEl.hidden = false;
  };
  const tick = () => {
    const leftMs = goAt - performance.now();
    if (leftMs > 0) {
      show(String(Math.ceil(leftMs / 1000)), false);
      // Just past each whole second, so a timer firing early never shows
      // the same number twice.
      setTimeout(tick, (leftMs % 1000 || 1000) + 5);
    } else {
      show("Go!", true);
      setTimeout(() => (raceCountdownEl.hidden = true), 1000);
    }
  };
  tick();
}

raceOpenBtn.addEventListener("click", openRaceDialog);
document.getElementById("race-cancel-btn").addEventListener("click", () => {
  raceOverlay.hidden = true;
});
raceStartBtn.addEventListener("click", async () => {
  showRaceError(null);
  raceStartBtn.disabled = true;
  try {
    const { go_in_ms } = await fetchJSON("/api/race/start", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ order: raceOrder }),
    });
    raceOverlay.hidden = true;
    countDown(go_in_ms);
  } catch (err) {
    showRaceError(err.message);
  } finally {
    raceStartBtn.disabled = false;
  }
});

// ---------------------------------------------------------------------
// Topics panel - inspects any registered topic, generically: the list
// comes from `/api/topics`, the picked topic's value from `/api/topic`.
// ---------------------------------------------------------------------

const topicSelectEl = document.getElementById("topic-select");
const topicWriterEl = document.getElementById("topic-writer");
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
    topicWriterEl.textContent = "";
    topicFreshnessEl.textContent = "";
    topicFreshnessEl.classList.remove("stale");
    topicRateEl.textContent = "";
    topicContentEl.textContent = "";
    return;
  }
  const rateHz = meanWriteRateHz();
  topicRateEl.textContent =
    rateHz === null ? "measuring write rate…" : `${rateHz.toFixed(1)} Hz mean over the last ${TOPIC_RATE_WINDOW_MS / 1000} s`;
  topicWriterEl.textContent = snapshot.writer ?? "";
  if (snapshot.age_ms === null) {
    topicFreshnessEl.textContent = "never written (initial value)";
    topicFreshnessEl.classList.add("stale");
  } else {
    // The server measured age_ms when it answered; add how long ago that was.
    const ageMs = snapshot.age_ms + (performance.now() - snapshot.receivedAtMs);
    topicFreshnessEl.textContent = `written ${formatAge(ageMs)} ago · write #${snapshot.write_count}`;
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
// Import Map modal - the browser decodes the chosen image (anything it can
// display, plus PGM, the ROS map format - TIFFs go through the server),
// thresholds it into a drivable / not-drivable raster, lets the user click
// the start line on a preview, and uploads the raster to `POST /api/maps/import`, which writes the new
// map folder (`map.tiff` + `info.json`).
// ---------------------------------------------------------------------

const importOverlay = document.getElementById("import-overlay");
const importForm = document.getElementById("import-form");
const importFileEl = document.getElementById("import-file");
const importNameEl = document.getElementById("import-name");
const importResolutionEl = document.getElementById("import-resolution");
const importThresholdEl = document.getElementById("import-threshold");
const importInvertEl = document.getElementById("import-invert");
const importPreviewWrapEl = document.getElementById("import-preview-wrap");
const importHintEl = document.getElementById("import-hint");
const importPreviewEl = document.getElementById("import-preview");
const importFlipBtn = document.getElementById("import-flip-btn");
const importClearBtn = document.getElementById("import-clear-btn");
const importErrorEl = document.getElementById("import-error");
const importSuccessEl = document.getElementById("import-success");
const importCancelBtn = document.getElementById("import-cancel-btn");
const importReloadBtn = document.getElementById("import-reload-btn");
const importConfirmBtn = document.getElementById("import-confirm-btn");

/** The decoded image: `{ width, height, gray }`, `gray` one 0-255
 *  brightness per pixel (fully transparent pixels count as black), or
 *  null before a file is chosen. */
let importImage = null;
/** The start line's ends, in (fractional) image pixels - `[a, b]`, each
 *  `{ x, y }`, in click order until flipped. */
let importLinePoints = [];

function showImportError(message) {
  importErrorEl.textContent = message;
  importErrorEl.hidden = false;
}

/** Parses a binary (P5) or ASCII (P2) PGM, which browsers can't decode. */
function parsePgm(bytes) {
  const magic = String.fromCharCode(bytes[0], bytes[1]);
  if (magic !== "P5" && magic !== "P2") throw new Error("not a PGM file");
  let pos = 2;
  // Next whitespace-separated header token, skipping `#` comments.
  function token() {
    for (;;) {
      while (pos < bytes.length && /\s/.test(String.fromCharCode(bytes[pos]))) pos++;
      if (bytes[pos] !== 0x23) break;
      while (pos < bytes.length && bytes[pos] !== 0x0a) pos++;
    }
    let text = "";
    while (pos < bytes.length && !/\s/.test(String.fromCharCode(bytes[pos]))) {
      text += String.fromCharCode(bytes[pos++]);
    }
    return Number(text);
  }
  const width = token();
  const height = token();
  const maxValue = token();
  if (!(width > 0 && height > 0 && maxValue > 0)) throw new Error("malformed PGM header");
  const gray = new Uint8ClampedArray(width * height);
  if (magic === "P5") {
    pos++; // the single whitespace byte ending the header
    const wide = maxValue > 255;
    for (let i = 0; i < gray.length; i++) {
      const value = wide ? (bytes[pos + 2 * i] << 8) | bytes[pos + 2 * i + 1] : bytes[pos + i];
      gray[i] = Math.round((value * 255) / maxValue);
    }
  } else {
    for (let i = 0; i < gray.length; i++) gray[i] = Math.round((token() * 255) / maxValue);
  }
  return { width, height, gray };
}

/** Browsers can't decode TIFF, so the server does: it answers with the
 *  width and height (little-endian u32s) followed by one brightness byte
 *  per pixel. */
async function decodeTiff(bytes) {
  const response = await fetch("/api/maps/import/decode_tiff", {
    method: "POST",
    headers: { "Content-Type": "application/octet-stream" },
    body: bytes,
  });
  if (!response.ok) {
    const body = await response.json().catch(() => null);
    throw new Error(body && body.error ? body.error : `request failed (${response.status})`);
  }
  const data = new Uint8Array(await response.arrayBuffer());
  const header = new DataView(data.buffer, 0, 8);
  const width = header.getUint32(0, true);
  const height = header.getUint32(4, true);
  return { width, height, gray: new Uint8ClampedArray(data.buffer, 8) };
}

async function decodeImage(file) {
  const bytes = new Uint8Array(await file.arrayBuffer());
  if (bytes[0] === 0x50 && (bytes[1] === 0x35 || bytes[1] === 0x32)) return parsePgm(bytes);
  const isTiff =
    (bytes[0] === 0x49 && bytes[1] === 0x49 && bytes[2] === 0x2a && bytes[3] === 0x00) ||
    (bytes[0] === 0x4d && bytes[1] === 0x4d && bytes[2] === 0x00 && bytes[3] === 0x2a);
  if (isTiff) return decodeTiff(bytes);

  let bitmap;
  try {
    bitmap = await createImageBitmap(file);
  } catch {
    throw new Error("this browser can't decode that image - try PNG, JPEG, BMP, WebP, PGM or TIFF");
  }
  const canvas = document.createElement("canvas");
  canvas.width = bitmap.width;
  canvas.height = bitmap.height;
  const ctx = canvas.getContext("2d");
  ctx.drawImage(bitmap, 0, 0);
  const rgba = ctx.getImageData(0, 0, bitmap.width, bitmap.height).data;
  const gray = new Uint8ClampedArray(bitmap.width * bitmap.height);
  for (let i = 0; i < gray.length; i++) {
    const luma = 0.299 * rgba[4 * i] + 0.587 * rgba[4 * i + 1] + 0.114 * rgba[4 * i + 2];
    gray[i] = Math.round((luma * rgba[4 * i + 3]) / 255);
  }
  return { width: bitmap.width, height: bitmap.height, gray };
}

/** One byte per pixel, 255 for drivable and 0 otherwise - the body
 *  `/api/maps/import` expects. */
function importRaster() {
  const threshold = Number(importThresholdEl.value);
  const invert = importInvertEl.checked;
  const raster = new Uint8Array(importImage.gray.length);
  for (let i = 0; i < raster.length; i++) {
    const light = importImage.gray[i] >= threshold;
    raster[i] = light !== invert ? 255 : 0;
  }
  return raster;
}

/** The direction of travel across the start line `a -> b`, as a unit
 *  vector - `b - a` rotated a quarter turn, matching
 *  `StartFinishLine::start_pose` on the server. */
function importTravelDirection([a, b]) {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const length = Math.hypot(dx, dy);
  return { x: -dy / length, y: dx / length };
}

function drawImportPreview() {
  if (!importImage) return;
  const { width, height } = importImage;
  importPreviewEl.width = width;
  importPreviewEl.height = height;
  const ctx = importPreviewEl.getContext("2d");
  const imageData = ctx.createImageData(width, height);
  const raster = importRaster();
  for (let i = 0; i < raster.length; i++) {
    // Drivable pixels white, the rest dark, like the main map view.
    const value = raster[i] ? 255 : 40;
    imageData.data[4 * i] = value;
    imageData.data[4 * i + 1] = value;
    imageData.data[4 * i + 2] = value;
    imageData.data[4 * i + 3] = 255;
  }
  ctx.putImageData(imageData, 0, 0);

  // Stroke widths in image pixels, sized to stay visible however much the
  // canvas is scaled down to fit the modal.
  const onScreenScale = importPreviewEl.getBoundingClientRect().width / width || 1;
  const px = 1 / onScreenScale;
  ctx.fillStyle = "#e5484d";
  ctx.strokeStyle = "#e5484d";
  ctx.lineWidth = 3 * px;
  for (const point of importLinePoints) {
    ctx.beginPath();
    ctx.arc(point.x, point.y, 5 * px, 0, 2 * Math.PI);
    ctx.fill();
  }
  if (importLinePoints.length === 2) {
    const [a, b] = importLinePoints;
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();

    const direction = importTravelDirection(importLinePoints);
    const mid = { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
    const length = 40 * px;
    const tip = { x: mid.x + direction.x * length, y: mid.y + direction.y * length };
    const head = 12 * px;
    ctx.strokeStyle = ctx.fillStyle = "#3f9ce5";
    ctx.beginPath();
    ctx.moveTo(mid.x, mid.y);
    ctx.lineTo(tip.x, tip.y);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(tip.x, tip.y);
    ctx.lineTo(
      tip.x - direction.x * head - direction.y * head * 0.6,
      tip.y - direction.y * head + direction.x * head * 0.6,
    );
    ctx.lineTo(
      tip.x - direction.x * head + direction.y * head * 0.6,
      tip.y - direction.y * head - direction.x * head * 0.6,
    );
    ctx.closePath();
    ctx.fill();
  }
}

function syncImportControls() {
  const hints = [
    "Click one end of the start line.",
    "Click the other end of the start line.",
    "The arrow shows the direction of travel. Flip it if needed, or click again to start over.",
  ];
  importHintEl.textContent = hints[importLinePoints.length];
  importFlipBtn.disabled = importLinePoints.length !== 2;
  importClearBtn.disabled = importLinePoints.length === 0;
  importConfirmBtn.disabled = !importImage || importLinePoints.length !== 2;
}

function resetImportModal() {
  importForm.reset();
  importImage = null;
  importLinePoints = [];
  importPreviewWrapEl.hidden = true;
  importErrorEl.hidden = true;
  importSuccessEl.hidden = true;
  importReloadBtn.hidden = true;
  importConfirmBtn.hidden = false;
  importCancelBtn.textContent = "Cancel";
  for (const el of importForm.elements) el.disabled = false;
  syncImportControls();
}

document.getElementById("import-btn").addEventListener("click", () => {
  resetImportModal();
  importOverlay.hidden = false;
});
importCancelBtn.addEventListener("click", () => {
  importOverlay.hidden = true;
});
importReloadBtn.addEventListener("click", () => window.location.reload());

importFileEl.addEventListener("change", async () => {
  importErrorEl.hidden = true;
  importImage = null;
  importLinePoints = [];
  importPreviewWrapEl.hidden = true;
  syncImportControls();
  const file = importFileEl.files[0];
  if (!file) return;
  if (!importNameEl.value) importNameEl.value = file.name.replace(/\.[^.]*$/, "");
  try {
    importImage = await decodeImage(file);
  } catch (err) {
    showImportError(err.message);
    return;
  }
  importPreviewWrapEl.hidden = false;
  syncImportControls();
  drawImportPreview();
  // Now laid out: redraw so the markers are sized for the on-screen scale.
  requestAnimationFrame(drawImportPreview);
});

importThresholdEl.addEventListener("input", drawImportPreview);
importInvertEl.addEventListener("change", drawImportPreview);

importPreviewEl.addEventListener("click", (event) => {
  if (!importImage || importConfirmBtn.hidden) return;
  const rect = importPreviewEl.getBoundingClientRect();
  const point = {
    x: ((event.clientX - rect.left) / rect.width) * importImage.width,
    y: ((event.clientY - rect.top) / rect.height) * importImage.height,
  };
  if (importLinePoints.length === 2) importLinePoints = [];
  importLinePoints.push(point);
  syncImportControls();
  drawImportPreview();
});

importFlipBtn.addEventListener("click", () => {
  importLinePoints.reverse();
  drawImportPreview();
});

importClearBtn.addEventListener("click", () => {
  importLinePoints = [];
  syncImportControls();
  drawImportPreview();
});

importForm.addEventListener("submit", async (event) => {
  event.preventDefault();
  if (!importImage || importLinePoints.length !== 2) return;
  importErrorEl.hidden = true;
  importConfirmBtn.disabled = true;

  const [a, b] = importLinePoints;
  const name = importNameEl.value.trim();
  const params = new URLSearchParams({
    name,
    width_px: importImage.width,
    height_px: importImage.height,
    resolution_m_per_px: importResolutionEl.value,
    a_x_px: a.x,
    a_y_px: a.y,
    b_x_px: b.x,
    b_y_px: b.y,
  });
  try {
    await fetchJSON(`/api/maps/import?${params}`, {
      method: "POST",
      headers: { "Content-Type": "application/octet-stream" },
      body: importRaster(),
    });
  } catch (err) {
    // Includes the 409 when a folder of that name already exists.
    showImportError(err.message);
    importConfirmBtn.disabled = false;
    return;
  }

  importSuccessEl.textContent = `Map "${name}" imported into maps/${name}. Reload the page to see it in the list.`;
  importSuccessEl.hidden = false;
  for (const el of importForm.elements) {
    if (el !== importCancelBtn && el !== importReloadBtn) el.disabled = true;
  }
  importConfirmBtn.hidden = true;
  importReloadBtn.hidden = false;
  importCancelBtn.textContent = "Close";
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

/** Whether a key pressed on `target` belongs to it rather than to the
 *  shortcuts: a text field, or anything in an open dialog (e.g. a select
 *  in the "add opponent" form - "r" there must not restart everything). */
function isTypingTarget(target) {
  return (
    target instanceof HTMLInputElement ||
    target instanceof HTMLTextAreaElement ||
    (target instanceof Element && target.closest(".modal-overlay, #generate-overlay") !== null)
  );
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
// Place-vehicle tool (the button under the zoom slider): the first click
// on the map picks where the vehicle goes, the second where it faces - an
// arrow follows the mouse in between - then the ego vehicle is placed
// there at rest, like RViz's "2D Pose Estimate".
// ---------------------------------------------------------------------

const placePoseBtn = document.getElementById("place-pose-btn");

/** Clicks closer than this to the first one (in CSS pixels) are too short
 *  to give a heading, and are ignored. */
const PLACE_POSE_MIN_ARROW_PX = 5;
const PLACE_POSE_COLOR = "#ff8ae8";

/** `"idle"`, `"position"` (waiting for the first click) or `"heading"`
 *  (waiting for the second), plus the world points picked so far. */
const placePose = { stage: "idle", start: null, current: null };

const placePoseTool = {
  cursor: "crosshair",

  onClick(world) {
    if (placePose.stage === "position") {
      placePose.start = world;
      placePose.current = world;
      placePose.stage = "heading";
      MapView.requestRedraw();
      return;
    }
    const { start } = placePose;
    const lengthPx = Math.hypot(world.x - start.x, world.y - start.y) * MapView.scalePxPerMeter();
    if (lengthPx < PLACE_POSE_MIN_ARROW_PX * (window.devicePixelRatio || 1)) return;
    // Same axes as the `vehicle` painter, which rotates by the heading
    // straight in screen space.
    const pose = { x_m: start.x, y_m: start.y, heading_rad: Math.atan2(world.y - start.y, world.x - start.x) };
    disarmPlacePose();
    fetch("/api/place_at_start", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(pose),
    }).catch((err) => console.error(err));
  },

  onMove(world) {
    if (placePose.stage !== "heading") return;
    placePose.current = world;
    MapView.requestRedraw();
  },

  onCancel() {
    resetPlacePose();
  },

  drawOverlay(ctx, { worldToScreen, dpr }) {
    if (placePose.stage !== "heading") return;
    const scale = dpr();
    const from = worldToScreen(placePose.start.x, placePose.start.y);
    const to = worldToScreen(placePose.current.x, placePose.current.y);
    ctx.fillStyle = PLACE_POSE_COLOR;
    ctx.strokeStyle = PLACE_POSE_COLOR;

    ctx.beginPath();
    ctx.arc(from.x, from.y, 4 * scale, 0, 2 * Math.PI);
    ctx.fill();

    const length = Math.hypot(to.x - from.x, to.y - from.y);
    if (length < 1) return;
    const angle = Math.atan2(to.y - from.y, to.x - from.x);
    const headLength = Math.min(14 * scale, length);
    ctx.lineWidth = 3 * scale;
    ctx.lineCap = "round";
    ctx.beginPath();
    ctx.moveTo(from.x, from.y);
    ctx.lineTo(to.x - headLength * 0.8 * Math.cos(angle), to.y - headLength * 0.8 * Math.sin(angle));
    ctx.stroke();

    ctx.translate(to.x, to.y);
    ctx.rotate(angle);
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(-headLength, -headLength * 0.5);
    ctx.lineTo(-headLength, headLength * 0.5);
    ctx.closePath();
    ctx.fill();
  },
};

function resetPlacePose() {
  placePose.stage = "idle";
  placePose.start = null;
  placePose.current = null;
  placePoseBtn.classList.remove("active");
}

function disarmPlacePose() {
  resetPlacePose();
  MapView.setPointerTool(null);
}

placePoseBtn.addEventListener("click", () => {
  if (placePose.stage !== "idle") {
    disarmPlacePose();
    return;
  }
  resetStartLine();
  showMapToolsStatus(null);
  placePose.stage = "position";
  placePoseBtn.classList.add("active");
  MapView.setPointerTool(placePoseTool);
});

// ---------------------------------------------------------------------
// Start/finish line tool (the Maps panel's bottom button): the first click
// on the map picks the line's left end, the second its right end - as seen
// driving through it, so an arrow across the line shows the direction of
// travel in between - then the selected map's info.json is rewritten and
// the map reloaded with it.
// ---------------------------------------------------------------------

const startLineBtn = document.getElementById("start-line-btn");
const mapToolsStatusEl = document.getElementById("map-tools-status");

/** Lines shorter than this (in CSS pixels) are taken as a stray click. */
const START_LINE_MIN_PX = 5;
const START_LINE_COLOR = "#e5484d";
const START_LINE_ARROW_COLOR = "#3f9ce5";

/** `"idle"`, `"a"` (waiting for the left end) or `"b"` (waiting for the
 *  right end), plus the world points picked so far. */
const startLine = { stage: "idle", a: null, current: null };

function showMapToolsStatus(message) {
  mapToolsStatusEl.textContent = message ?? "";
  mapToolsStatusEl.hidden = message == null;
}

const startLineTool = {
  cursor: "crosshair",

  onClick(world) {
    if (startLine.stage === "a") {
      startLine.a = world;
      startLine.current = world;
      startLine.stage = "b";
      showMapToolsStatus("Click the right end. The arrow shows the direction of travel.");
      MapView.requestRedraw();
      return;
    }
    const { a } = startLine;
    const lengthPx = Math.hypot(world.x - a.x, world.y - a.y) * MapView.scalePxPerMeter();
    if (lengthPx < START_LINE_MIN_PX * (window.devicePixelRatio || 1)) return;
    const name = liveMapName;
    disarmStartLine();
    saveStartLine(name, a, world);
  },

  onMove(world) {
    if (startLine.stage !== "b") return;
    startLine.current = world;
    MapView.requestRedraw();
  },

  onCancel() {
    resetStartLine();
    showMapToolsStatus(null);
  },

  drawOverlay(ctx, { worldToScreen, dpr }) {
    if (startLine.stage !== "b") return;
    const scale = dpr();
    const a = worldToScreen(startLine.a.x, startLine.a.y);
    const b = worldToScreen(startLine.current.x, startLine.current.y);
    ctx.fillStyle = START_LINE_COLOR;
    ctx.strokeStyle = START_LINE_COLOR;
    ctx.lineWidth = 3 * scale;
    ctx.lineCap = "round";

    ctx.beginPath();
    ctx.arc(a.x, a.y, 4 * scale, 0, 2 * Math.PI);
    ctx.fill();

    const length = Math.hypot(b.x - a.x, b.y - a.y);
    if (length < 1) return;
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();

    // The direction of travel: `b - a` rotated a quarter turn, as
    // `StartFinishLine::start_pose` does - the canvas keeps world axes, so
    // this holds in screen space too.
    const direction = { x: -(b.y - a.y) / length, y: (b.x - a.x) / length };
    const mid = { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
    const arrowLength = 30 * scale;
    const head = 10 * scale;
    const tip = { x: mid.x + direction.x * arrowLength, y: mid.y + direction.y * arrowLength };
    ctx.fillStyle = START_LINE_ARROW_COLOR;
    ctx.strokeStyle = START_LINE_ARROW_COLOR;
    ctx.lineWidth = 2 * scale;
    ctx.beginPath();
    ctx.moveTo(mid.x, mid.y);
    ctx.lineTo(tip.x - direction.x * head * 0.8, tip.y - direction.y * head * 0.8);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(tip.x, tip.y);
    ctx.lineTo(
      tip.x - direction.x * head - direction.y * head * 0.6,
      tip.y - direction.y * head + direction.x * head * 0.6,
    );
    ctx.lineTo(
      tip.x - direction.x * head + direction.y * head * 0.6,
      tip.y - direction.y * head - direction.x * head * 0.6,
    );
    ctx.closePath();
    ctx.fill();
  },
};

async function saveStartLine(name, a, b) {
  showMapToolsStatus("Saving...");
  try {
    const response = await fetch("/api/maps/start_finish_line", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name, a: { x: a.x, y: a.y }, b: { x: b.x, y: b.y } }),
    });
    if (!response.ok) {
      const body = await response.json().catch(() => ({}));
      throw new Error(body.error || `HTTP ${response.status}`);
    }
    showMapToolsStatus(`Saved in ${name}/info.json.`);
  } catch (err) {
    showMapToolsStatus(`Couldn't save the start/finish line: ${err.message}`);
  }
}

function resetStartLine() {
  startLine.stage = "idle";
  startLine.a = null;
  startLine.current = null;
  startLineBtn.classList.remove("active");
}

function disarmStartLine() {
  if (startLine.stage === "idle") return;
  resetStartLine();
  MapView.setPointerTool(null);
}

startLineBtn.addEventListener("click", () => {
  if (startLine.stage !== "idle") {
    disarmStartLine();
    showMapToolsStatus(null);
    return;
  }
  if (!liveMapName) return;
  resetPlacePose();
  startLine.stage = "a";
  startLineBtn.classList.add("active");
  showMapToolsStatus("Click the left end of the new start/finish line, as seen driving through it.");
  MapView.setPointerTool(startLineTool);
});

// ---------------------------------------------------------------------
// Map pixel editor (the Maps panel's "Edit map pixels" button): a pop-up
// with its own pan/zoom canvas of the selected map's raster, painted
// black (wall) or white (drivable) with a round pen. Saving replaces the
// map's map.tiff - the server keeps the map as it was made aside the first
// time, which "Revert to original" brings back - and reloads it.
// ---------------------------------------------------------------------

const mapEditBtn = document.getElementById("map-edit-btn");
const mapEditOverlay = document.getElementById("map-edit-overlay");
const mapEditModal = document.getElementById("map-edit-modal");
const mapEditNameEl = document.getElementById("map-edit-name");
const mapEditColorBtns = [...document.querySelectorAll(".map-edit-color")];
const mapEditRadiusEl = document.getElementById("map-edit-radius");
const mapEditRadiusValueEl = document.getElementById("map-edit-radius-value");
const mapEditUndoBtn = document.getElementById("map-edit-undo-btn");
const mapEditRevertBtn = document.getElementById("map-edit-revert-btn");
const mapEditFitBtn = document.getElementById("map-edit-fit-btn");
const mapEditStageEl = document.getElementById("map-edit-stage");
const mapEditCanvas = document.getElementById("map-edit-canvas");
const mapEditErrorEl = document.getElementById("map-edit-error");
const mapEditCancelBtn = document.getElementById("map-edit-cancel-btn");
const mapEditSaveBtn = document.getElementById("map-edit-save-btn");

/** Pixel values, as the raster API sends them. */
const MAP_EDIT_WALL = 0;
const MAP_EDIT_TRACK = 255;
/** How a wall pixel is shown - true black, the stage around the map being
 *  a lighter gray so the map's edge stays visible. */
const MAP_EDIT_WALL_RGB = [0, 0, 0];
/** The pixel grid appears from this many screen pixels per map pixel, and
 *  is fully visible by twice that. */
const MAP_EDIT_GRID_MIN_PX = 8;
const MAP_EDIT_GRID_COLOR = "#3f9ce5";
const MAP_EDIT_RACE_LINE_COLOR = "rgba(255, 159, 67, 0.85)";
const MAP_EDIT_START_LINE_COLOR = "#e5484d";
/** Zoom range, in screen pixels per map pixel. */
const MAP_EDIT_MAX_SCALE = 80;
const MAP_EDIT_MIN_SCALE_OF_FIT = 0.25;

/** Everything about the map being edited - null while the pop-up is closed. */
let mapEdit = null;
/** The pen's pixel value, kept from one opening of the pop-up to the next. */
let mapEditColor = MAP_EDIT_WALL;

function mapEditDpr() {
  return window.devicePixelRatio || 1;
}

function showMapEditError(message) {
  mapEditErrorEl.textContent = message ?? "";
  mapEditErrorEl.hidden = message == null;
}

function setMapEditColor(value) {
  mapEditColor = value;
  for (const btn of mapEditColorBtns) {
    btn.setAttribute("aria-checked", String(Number(btn.dataset.value) === value));
  }
  requestMapEditDraw();
}

function syncMapEditRadius() {
  const radius = Number(mapEditRadiusEl.value);
  const diameter = 2 * radius + 1;
  const meters = mapEdit ? ` (${(diameter * mapEdit.info.resolution_m_per_px).toFixed(2)} m)` : "";
  mapEditRadiusValueEl.textContent = `${radius} px${meters}`;
  requestMapEditDraw();
}

function syncMapEditButtons() {
  const edited = mapEdit !== null && mapEdit.undo.length > 0;
  mapEditUndoBtn.disabled = !edited || mapEdit.saving;
  mapEditSaveBtn.disabled = !edited || mapEdit.saving;
  mapEditRevertBtn.disabled = mapEdit === null || !mapEdit.info.has_original || mapEdit.saving;
}

async function openMapEditor() {
  const name = liveMapName;
  if (!name) return;
  disarmStartLine();
  disarmPlacePose();
  showMapToolsStatus(null);
  showMapEditError(null);
  mapEditNameEl.textContent = name;
  mapEdit = null;
  syncMapEditButtons();
  mapEditOverlay.hidden = false;
  mapEditModal.focus({ preventScroll: true });
  resizeMapEditCanvas();

  const query = new URLSearchParams({ name });
  let info;
  let pixels;
  try {
    const [infoResponse, rasterResponse] = await Promise.all([
      fetchJSON(`/api/map_edit?${query}`),
      fetch(`/api/map_edit/raster?${query}`),
    ]);
    if (!rasterResponse.ok) throw new Error(`can't read the map's pixels (HTTP ${rasterResponse.status})`);
    info = infoResponse;
    pixels = new Uint8Array(await rasterResponse.arrayBuffer());
    if (pixels.length !== info.width_px * info.height_px) throw new Error("the map's pixels don't match its size");
  } catch (err) {
    showMapEditError(`Couldn't open the map: ${err.message}`);
    return;
  }
  if (mapEditOverlay.hidden) return; // closed while loading

  const bitmap = document.createElement("canvas");
  bitmap.width = info.width_px;
  bitmap.height = info.height_px;
  const bitmapCtx = bitmap.getContext("2d");
  mapEdit = {
    name,
    info,
    pixels,
    original: null,
    bitmap,
    bitmapCtx,
    imageData: bitmapCtx.createImageData(info.width_px, info.height_px),
    view: { scale: 1, x: 0, y: 0 },
    undo: [],
    stroke: null,
    lastPaint: null,
    pan: null,
    hover: null,
    saving: false,
  };
  for (let i = 0; i < pixels.length; i++) paintMapEditImagePixel(i);
  bitmapCtx.putImageData(mapEdit.imageData, 0, 0);
  syncMapEditRadius();
  syncMapEditButtons();
  fitMapEditView();
}

/** Copies pixel `index`'s value into the bitmap's `ImageData`. */
function paintMapEditImagePixel(index) {
  const { pixels, imageData } = mapEdit;
  const rgb = pixels[index] ? [255, 255, 255] : MAP_EDIT_WALL_RGB;
  const data = imageData.data;
  data[4 * index] = rgb[0];
  data[4 * index + 1] = rgb[1];
  data[4 * index + 2] = rgb[2];
  data[4 * index + 3] = 255;
}

function refreshMapEditBitmap() {
  for (let i = 0; i < mapEdit.pixels.length; i++) paintMapEditImagePixel(i);
  mapEdit.bitmapCtx.putImageData(mapEdit.imageData, 0, 0);
  requestMapEditDraw();
}

/** Closes the pop-up, first asking before throwing away unsaved strokes. */
function closeMapEditor() {
  if (mapEdit && mapEdit.saving) return;
  if (mapEdit && mapEdit.undo.length > 0 && !window.confirm("Discard your edits to this map?")) return;
  mapEdit = null;
  mapEditOverlay.hidden = true;
}

// --- view -------------------------------------------------------------

function resizeMapEditCanvas() {
  const dpr = mapEditDpr();
  const width = Math.max(1, Math.round(mapEditStageEl.clientWidth * dpr));
  const height = Math.max(1, Math.round(mapEditStageEl.clientHeight * dpr));
  if (mapEditCanvas.width !== width || mapEditCanvas.height !== height) {
    mapEditCanvas.width = width;
    mapEditCanvas.height = height;
  }
  requestMapEditDraw();
}

new ResizeObserver(resizeMapEditCanvas).observe(mapEditStageEl);

/** Scale (device pixels per map pixel) that fits the whole map in view. */
function mapEditFitScale() {
  const { width_px, height_px } = mapEdit.info;
  return 0.95 * Math.min(mapEditCanvas.width / width_px, mapEditCanvas.height / height_px);
}

function fitMapEditView() {
  if (!mapEdit) return;
  const { width_px, height_px } = mapEdit.info;
  const scale = mapEditFitScale();
  mapEdit.view = {
    scale,
    x: (mapEditCanvas.width - width_px * scale) / 2,
    y: (mapEditCanvas.height - height_px * scale) / 2,
  };
  requestMapEditDraw();
}

/** A mouse event's position in device pixels on the canvas. */
function mapEditDevicePoint(event) {
  const rect = mapEditCanvas.getBoundingClientRect();
  const dpr = mapEditDpr();
  return { x: (event.clientX - rect.left) * dpr, y: (event.clientY - rect.top) * dpr };
}

/** A device-pixel point in map pixel coordinates (fractional). */
function mapEditToPixel({ x, y }) {
  const { view } = mapEdit;
  return { x: (x - view.x) / view.scale, y: (y - view.y) / view.scale };
}

/** A world point (meters) in map pixel coordinates - the image's origin is
 *  its (0, 0) corner, axes unflipped (see `ImageOrigin`). */
function mapEditWorldToPixel(x, y) {
  const { origin, resolution_m_per_px } = mapEdit.info;
  return { x: (x - origin.x) / resolution_m_per_px, y: (y - origin.y) / resolution_m_per_px };
}

// --- painting -----------------------------------------------------------

/** Paints the pen's disc around the pixel containing `point` (map pixel
 *  coordinates), recording each pixel's old value in the current stroke. */
function stampMapEdit(point) {
  const { pixels, info, stroke } = mapEdit;
  const color = mapEditColor;
  const radius = Number(mapEditRadiusEl.value);
  const cx = Math.floor(point.x);
  const cy = Math.floor(point.y);
  // r^2 + r rather than r^2: rounder discs, without the single pixel
  // sticking out at each compass point.
  const reach = radius * radius + radius;
  const x0 = Math.max(0, cx - radius);
  const x1 = Math.min(info.width_px - 1, cx + radius);
  const y0 = Math.max(0, cy - radius);
  const y1 = Math.min(info.height_px - 1, cy + radius);
  let changed = false;
  for (let y = y0; y <= y1; y++) {
    for (let x = x0; x <= x1; x++) {
      if ((x - cx) ** 2 + (y - cy) ** 2 > reach) continue;
      const index = y * info.width_px + x;
      if (pixels[index] === color) continue;
      if (!stroke.has(index)) stroke.set(index, pixels[index]);
      pixels[index] = color;
      paintMapEditImagePixel(index);
      changed = true;
    }
  }
  return changed;
}

/** Paints from the last painted point to `point`, stamping every pixel or
 *  so along the way - mouse moves arrive spaced out, and a fast drag would
 *  otherwise leave a dotted line. */
function paintMapEditTo(point) {
  const from = mapEdit.lastPaint ?? point;
  const steps = Math.max(1, Math.ceil(Math.hypot(point.x - from.x, point.y - from.y)));
  let changed = false;
  for (let step = 1; step <= steps; step++) {
    const t = step / steps;
    changed = stampMapEdit({ x: from.x + (point.x - from.x) * t, y: from.y + (point.y - from.y) * t }) || changed;
  }
  mapEdit.lastPaint = point;
  if (changed) {
    mapEdit.bitmapCtx.putImageData(mapEdit.imageData, 0, 0);
    requestMapEditDraw();
  }
}

function endMapEditStroke() {
  if (!mapEdit || !mapEdit.stroke) return;
  if (mapEdit.stroke.size > 0) mapEdit.undo.push(mapEdit.stroke);
  mapEdit.stroke = null;
  mapEdit.lastPaint = null;
  syncMapEditButtons();
}

function undoMapEdit() {
  if (!mapEdit || mapEdit.stroke || mapEdit.saving) return;
  const stroke = mapEdit.undo.pop();
  if (!stroke) return;
  for (const [index, value] of stroke) mapEdit.pixels[index] = value;
  refreshMapEditBitmap();
  syncMapEditButtons();
}

async function revertMapEdit() {
  if (!mapEdit || !mapEdit.info.has_original) return;
  const edit = mapEdit;
  if (!edit.original) {
    try {
      const response = await fetch(`/api/map_edit/raster?${new URLSearchParams({ name: edit.name, original: "true" })}`);
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      edit.original = new Uint8Array(await response.arrayBuffer());
    } catch (err) {
      showMapEditError(`Couldn't read the original map: ${err.message}`);
      return;
    }
  }
  if (mapEdit !== edit) return; // closed meanwhile
  if (edit.original.length !== edit.pixels.length) {
    showMapEditError("The original map has a different size - it can't be restored here.");
    return;
  }
  // One stroke covering every differing pixel, so reverting is undoable
  // like any other edit.
  const stroke = new Map();
  for (let i = 0; i < edit.pixels.length; i++) {
    if (edit.pixels[i] !== edit.original[i]) {
      stroke.set(i, edit.pixels[i]);
      edit.pixels[i] = edit.original[i];
    }
  }
  if (stroke.size > 0) edit.undo.push(stroke);
  refreshMapEditBitmap();
  syncMapEditButtons();
}

async function saveMapEdit() {
  if (!mapEdit || mapEdit.undo.length === 0) return;
  const edit = mapEdit;
  edit.saving = true;
  syncMapEditButtons();
  showMapEditError(null);
  try {
    const saved = await fetchJSON(`/api/map_edit/raster?${new URLSearchParams({ name: edit.name })}`, {
      method: "POST",
      headers: { "Content-Type": "application/octet-stream" },
      body: edit.pixels,
    });
    mapEdit = null;
    mapEditOverlay.hidden = true;
    const stale =
      saved.race_lines > 0
        ? ` Its ${saved.race_lines === 1 ? "race line was" : `${saved.race_lines} race lines were`} computed on the old pixels - plan again if the edit touched the track.`
        : "";
    showMapToolsStatus(`Saved the pixels of ${edit.name}.${stale}`);
  } catch (err) {
    // Includes the 409 while mapping or localization runs.
    edit.saving = false;
    syncMapEditButtons();
    showMapEditError(`Couldn't save: ${err.message}`);
  }
}

// --- drawing ------------------------------------------------------------

let mapEditDrawRequested = false;

function requestMapEditDraw() {
  if (mapEditDrawRequested) return;
  mapEditDrawRequested = true;
  requestAnimationFrame(() => {
    mapEditDrawRequested = false;
    drawMapEdit();
  });
}

function drawMapEdit() {
  const ctx = mapEditCanvas.getContext("2d");
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.clearRect(0, 0, mapEditCanvas.width, mapEditCanvas.height);
  if (!mapEdit) return;
  const { view, info } = mapEdit;
  const dpr = mapEditDpr();

  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(mapEdit.bitmap, view.x, view.y, info.width_px * view.scale, info.height_px * view.scale);

  drawMapEditGrid(ctx);
  drawMapEditOverlays(ctx, dpr);
  drawMapEditPen(ctx, dpr);
}

/** Blue lines along the pixel edges, fading in once zoomed in enough to
 *  tell pixels apart - only over the part of the map in view. */
function drawMapEditGrid(ctx) {
  const { view, info } = mapEdit;
  const alpha = Math.min(1, (view.scale - MAP_EDIT_GRID_MIN_PX) / MAP_EDIT_GRID_MIN_PX);
  if (alpha <= 0) return;
  const i0 = Math.max(0, Math.floor(-view.x / view.scale));
  const i1 = Math.min(info.width_px, Math.ceil((mapEditCanvas.width - view.x) / view.scale));
  const j0 = Math.max(0, Math.floor(-view.y / view.scale));
  const j1 = Math.min(info.height_px, Math.ceil((mapEditCanvas.height - view.y) / view.scale));
  if (i1 < i0 || j1 < j0) return;
  const top = view.y + j0 * view.scale;
  const bottom = view.y + j1 * view.scale;
  const left = view.x + i0 * view.scale;
  const right = view.x + i1 * view.scale;
  ctx.globalAlpha = 0.6 * alpha;
  ctx.strokeStyle = MAP_EDIT_GRID_COLOR;
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let i = i0; i <= i1; i++) {
    const x = Math.round(view.x + i * view.scale) + 0.5;
    ctx.moveTo(x, top);
    ctx.lineTo(x, bottom);
  }
  for (let j = j0; j <= j1; j++) {
    const y = Math.round(view.y + j * view.scale) + 0.5;
    ctx.moveTo(left, y);
    ctx.lineTo(right, y);
  }
  ctx.stroke();
  ctx.globalAlpha = 1;
}

/** The race line and the start/finish line, to paint around. */
function drawMapEditOverlays(ctx, dpr) {
  const { view, info } = mapEdit;
  const toScreen = (x, y) => {
    const pixel = mapEditWorldToPixel(x, y);
    return { x: view.x + pixel.x * view.scale, y: view.y + pixel.y * view.scale };
  };
  ctx.lineCap = "round";
  ctx.lineJoin = "round";

  if (info.race_line.length > 1) {
    ctx.strokeStyle = MAP_EDIT_RACE_LINE_COLOR;
    ctx.lineWidth = 2 * dpr;
    ctx.beginPath();
    info.race_line.forEach(([x, y], index) => {
      const point = toScreen(x, y);
      if (index === 0) ctx.moveTo(point.x, point.y);
      else ctx.lineTo(point.x, point.y);
    });
    ctx.closePath();
    ctx.stroke();
  }

  const { a, b } = info.start_finish_line;
  const from = toScreen(a.x, a.y);
  const to = toScreen(b.x, b.y);
  ctx.strokeStyle = MAP_EDIT_START_LINE_COLOR;
  ctx.lineWidth = 3 * dpr;
  ctx.beginPath();
  ctx.moveTo(from.x, from.y);
  ctx.lineTo(to.x, to.y);
  ctx.stroke();
}

/** The pen's outline, covering exactly the pixels a click would paint. */
function drawMapEditPen(ctx, dpr) {
  const { view, hover, pan } = mapEdit;
  const color = mapEditColor;
  if (!hover || pan) return;
  const pixel = mapEditToPixel(hover);
  const radius = Number(mapEditRadiusEl.value);
  const cx = view.x + (Math.floor(pixel.x) + 0.5) * view.scale;
  const cy = view.y + (Math.floor(pixel.y) + 0.5) * view.scale;
  const r = Math.max((Math.sqrt(radius * radius + radius) + 0.5) * view.scale, 3 * dpr);
  ctx.lineWidth = 3 * dpr;
  ctx.strokeStyle = color === MAP_EDIT_WALL ? "rgba(255, 255, 255, 0.9)" : "rgba(0, 0, 0, 0.9)";
  ctx.beginPath();
  ctx.arc(cx, cy, r, 0, 2 * Math.PI);
  ctx.stroke();
  ctx.lineWidth = 1.5 * dpr;
  ctx.strokeStyle = color === MAP_EDIT_WALL ? "#000" : "#fff";
  ctx.stroke();
}

// --- mouse and keys -----------------------------------------------------

mapEditCanvas.addEventListener("contextmenu", (event) => event.preventDefault());

mapEditCanvas.addEventListener("mousedown", (event) => {
  if (!mapEdit || mapEdit.saving) return;
  event.preventDefault();
  // Keeps keyboard focus inside the pop-up, so WASD never drives the car
  // from here.
  mapEditModal.focus({ preventScroll: true });
  const point = mapEditDevicePoint(event);
  if (event.button === 2) {
    mapEdit.pan = { x: point.x, y: point.y };
    mapEditCanvas.classList.add("panning");
    requestMapEditDraw();
  } else if (event.button === 0 && !mapEdit.pan) {
    mapEdit.stroke = new Map();
    mapEdit.lastPaint = null;
    paintMapEditTo(mapEditToPixel(point));
  }
});

window.addEventListener("mousemove", (event) => {
  if (!mapEdit) return;
  const point = mapEditDevicePoint(event);
  if (mapEdit.pan) {
    mapEdit.view.x += point.x - mapEdit.pan.x;
    mapEdit.view.y += point.y - mapEdit.pan.y;
    mapEdit.pan = point;
  } else if (mapEdit.stroke) {
    paintMapEditTo(mapEditToPixel(point));
  }
  const inside = event.target === mapEditCanvas;
  mapEdit.hover = inside ? point : null;
  requestMapEditDraw();
});

window.addEventListener("mouseup", (event) => {
  if (!mapEdit) return;
  if (event.button === 2 && mapEdit.pan) {
    mapEdit.pan = null;
    mapEditCanvas.classList.remove("panning");
    requestMapEditDraw();
  } else if (event.button === 0) {
    endMapEditStroke();
  }
});

mapEditCanvas.addEventListener("mouseleave", () => {
  if (!mapEdit) return;
  mapEdit.hover = null;
  requestMapEditDraw();
});

mapEditCanvas.addEventListener(
  "wheel",
  (event) => {
    event.preventDefault();
    if (!mapEdit) return;
    const point = mapEditDevicePoint(event);
    const { view } = mapEdit;
    const factor = Math.exp(-event.deltaY * (event.deltaMode === 1 ? 0.05 : 0.0015));
    const minScale = MAP_EDIT_MIN_SCALE_OF_FIT * mapEditFitScale();
    const scale = Math.min(MAP_EDIT_MAX_SCALE * mapEditDpr(), Math.max(minScale, view.scale * factor));
    // Zoom around the cursor: the map pixel under it stays under it.
    view.x = point.x - ((point.x - view.x) * scale) / view.scale;
    view.y = point.y - ((point.y - view.y) * scale) / view.scale;
    view.scale = scale;
    requestMapEditDraw();
  },
  { passive: false },
);

mapEditOverlay.addEventListener("keydown", (event) => {
  if (!mapEdit && event.key !== "Escape") return;
  const key = event.key.toLowerCase();
  if ((event.ctrlKey || event.metaKey) && key === "z") {
    event.preventDefault();
    undoMapEdit();
    return;
  }
  if (event.ctrlKey || event.metaKey || event.altKey) return;
  switch (key) {
    case "b":
      setMapEditColor(MAP_EDIT_WALL);
      break;
    case "w":
      setMapEditColor(MAP_EDIT_TRACK);
      break;
    case "[":
    case "]":
      mapEditRadiusEl.value = Number(mapEditRadiusEl.value) + (key === "]" ? 1 : -1);
      syncMapEditRadius();
      break;
    case "escape":
      closeMapEditor();
      break;
    default:
      return;
  }
  event.preventDefault();
});

// A click on the blurred backdrop closes the pop-up - only one that also
// started there, not the end of a stroke dragged out of the canvas.
let mapEditBackdropPressed = false;
mapEditOverlay.addEventListener("mousedown", (event) => {
  mapEditBackdropPressed = event.target === mapEditOverlay;
});
mapEditOverlay.addEventListener("click", (event) => {
  if (event.target === mapEditOverlay && mapEditBackdropPressed) closeMapEditor();
});

for (const btn of mapEditColorBtns) {
  btn.addEventListener("click", () => setMapEditColor(Number(btn.dataset.value)));
}
mapEditRadiusEl.addEventListener("input", syncMapEditRadius);
mapEditUndoBtn.addEventListener("click", undoMapEdit);
mapEditRevertBtn.addEventListener("click", revertMapEdit);
mapEditFitBtn.addEventListener("click", fitMapEditView);
mapEditCancelBtn.addEventListener("click", closeMapEditor);
mapEditSaveBtn.addEventListener("click", saveMapEdit);
mapEditBtn.addEventListener("click", () => {
  openMapEditor().catch((err) => console.error(err));
});
setMapEditColor(MAP_EDIT_WALL);
syncMapEditRadius();

// ---------------------------------------------------------------------
// "R" -> restart everything, then reload this page
// "P" -> place the vehicle at the start line
// "Esc" -> cancel the place-vehicle or start/finish line tool
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
    case "escape":
      if (placePose.stage !== "idle") {
        event.preventDefault();
        disarmPlacePose();
      }
      if (startLine.stage !== "idle") {
        event.preventDefault();
        disarmStartLine();
        showMapToolsStatus(null);
      }
      break;
  }
});

// ---------------------------------------------------------------------
// Debug panel - starting and stopping a debug recording (`/api/debug`).
// The recording is the server's, not this tab's: every tab shows the same
// one, and a restart ends it. Stopping only asks for it: the file is
// complete once the status says `saved`.
// ---------------------------------------------------------------------

const debugStateEl = document.getElementById("debug-state");
const debugFolderEl = document.getElementById("debug-folder");
const debugStartFormEl = document.getElementById("debug-start");
const debugNameEl = document.getElementById("debug-name");
const debugFrequencyEl = document.getElementById("debug-frequency");
const debugStartBtn = document.getElementById("debug-start-btn");
const debugStopBtn = document.getElementById("debug-stop-btn");
const debugDetailsEl = document.getElementById("debug-details");
const debugErrorEl = document.getElementById("debug-error");
const debugNavBtn = document.querySelector('.panel-nav-btn[data-panel="debug"]');

const DEBUG_STATE_LABELS = {
  idle: "Not recording",
  recording: "Recording",
  saving: "Saving...",
  saved: "Saved",
  failed: "Recording failed",
};

function showDebugError(message) {
  debugErrorEl.textContent = message;
  debugErrorEl.hidden = message === null;
}

function formatDuration(seconds) {
  const s = Math.floor(seconds);
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const pad = (n) => String(n).padStart(2, "0");
  return h > 0 ? `${h}:${pad(m)}:${pad(s % 60)}` : `${m}:${pad(s % 60)}`;
}

function formatBytes(bytes) {
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
  return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
}

function renderDebug(status) {
  debugFolderEl.textContent = `${status.folder}/`;
  // Seeded once from the server's default, then left to the user.
  if (debugFrequencyEl.value === "") debugFrequencyEl.value = status.frequency_hz;

  const running = status.state === "recording" || status.state === "saving";
  debugStateEl.className = status.state;
  debugStateEl.textContent = DEBUG_STATE_LABELS[status.state];
  debugNavBtn.classList.toggle("recording", status.state === "recording");
  debugStartFormEl.hidden = running;
  debugStopBtn.hidden = !running;
  debugStopBtn.disabled = status.state === "saving";

  if (status.path === null) {
    debugDetailsEl.textContent = "";
    return;
  }
  const parts = [
    formatDuration(status.duration_s),
    `${status.samples.toLocaleString()} sample${status.samples === 1 ? "" : "s"}`,
  ];
  if (status.file_size_bytes !== null) parts.push(formatBytes(status.file_size_bytes));
  let text = `${status.state === "saved" ? "Saved to" : "File:"} ${status.path} - ${parts.join(" · ")}.`;
  if (status.state === "recording" && status.falling_behind && status.achieved_hz !== null) {
    text += ` Falling behind: ~${status.achieved_hz.toFixed(0)} of ${status.frequency_hz} Hz.`;
  }
  if (status.state === "saved" && status.interrupted) {
    text += " Ended by a restart.";
  }
  if (status.state === "failed") text += ` ${status.error}`;
  debugDetailsEl.textContent = text;
}

async function pollDebug() {
  renderDebug(await fetchJSON("/api/debug"));
}

debugStartFormEl.addEventListener("submit", (event) => {
  event.preventDefault();
  showDebugError(null);
  debugStartBtn.disabled = true;
  fetchJSON("/api/debug/start", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      name: debugNameEl.value.trim(),
      frequency_hz: Number(debugFrequencyEl.value),
    }),
  })
    .then(() => {
      debugNameEl.value = "";
      return pollDebug();
    })
    .catch((err) => showDebugError(`Couldn't start: ${err.message}`))
    .finally(() => {
      debugStartBtn.disabled = false;
    });
});

debugStopBtn.addEventListener("click", () => {
  showDebugError(null);
  fetchJSON("/api/debug/stop", { method: "POST" })
    .then(() => pollDebug())
    .catch((err) => showDebugError(`Couldn't stop: ${err.message}`));
});

// ---------------------------------------------------------------------
// VESC panel - the real car's motor controller (`/api/vesc`), only shown
// on the real car (see `startHardwarePanels`).
// ---------------------------------------------------------------------

const vescNavBtn = document.querySelector('.panel-nav-btn[data-panel="vesc"]');
const vescStateEl = document.getElementById("vesc-state");
const vescBatteryEl = document.getElementById("vesc-battery");
const vescChargeEl = document.getElementById("vesc-charge");
const vescSpeedEl = document.getElementById("vesc-speed");
const vescCommandedEl = document.getElementById("vesc-commanded");
const vescServoEl = document.getElementById("vesc-servo");
const vescCurrentEl = document.getElementById("vesc-current");
const vescTemperatureEl = document.getElementById("vesc-temperature");
const vescTachometerEl = document.getElementById("vesc-tachometer");

/** How often the VESC panel is refreshed. */
const VESC_POLL_MS = 250;
/** Older than this, the VESC's readings mean the VESC was lost. */
const VESC_STALE_MS = 500;

/** The panel's headline, and how alarming it is: `ok`, `warning` or `fault`. */
function vescState(response) {
  const status = response.value;
  if (response.age_ms === null) return ["Waiting for the VESC...", "warning"];
  if (response.age_ms > VESC_STALE_MS) return ["VESC lost - reconnecting", "fault"];
  if (status.fault !== "NONE") return [`Fault: ${status.fault}`, "fault"];
  if (status.low_battery) return ["Battery low - recharge it", "warning"];
  if (status.timed_out) return ["Stopped by the VESC's own timeout", "warning"];
  return ["Connected", "ok"];
}

function renderVesc(response) {
  const [text, level] = vescState(response);
  vescStateEl.textContent = text;
  vescStateEl.className = level;
  vescNavBtn.classList.toggle("warning", level === "warning");
  vescNavBtn.classList.toggle("fault", level === "fault");

  const status = response.value;
  const charge = Math.round(status.battery_charge * 100);
  vescBatteryEl.textContent = `${status.input_voltage_v.toFixed(1)} V, ~${charge}%`;
  vescChargeEl.value = status.battery_charge;
  vescChargeEl.classList.toggle("warning", status.low_battery);
  vescBatteryEl.classList.toggle("warning", status.low_battery);
  vescSpeedEl.textContent = `${status.wheel_speed_mps.toFixed(2)} m/s (${Math.round(status.erpm)} ERPM)`;
  vescCommandedEl.textContent =
    status.commanded_erpm === null ? "braking" : `${status.commanded_erpm} ERPM`;
  vescServoEl.textContent = status.servo_position.toFixed(3);
  vescCurrentEl.textContent =
    `${status.motor_current_a.toFixed(2)} A motor, ${status.input_current_a.toFixed(2)} A battery`;
  vescTemperatureEl.textContent = `${status.temp_fet_c.toFixed(1)} °C MOSFETs, ${status.temp_motor_c.toFixed(1)} °C motor`;
  vescTachometerEl.textContent = status.tachometer.toLocaleString();
}

async function pollVesc() {
  renderVesc(await fetchJSON("/api/vesc"));
}

// The VESC's settings and limits, tuned live like a vehicle model's -
// both groups go through the same endpoints, told apart by `group`.
const vescParameterPanel = (group) =>
  createParameterPanel({
    containerEl: document.getElementById(`vesc-${group}-parameters`),
    saveEl: document.getElementById(`vesc-${group}-save`),
    setUrl: "/api/vesc_parameter",
    saveUrl: "/api/vesc_parameters_save",
    loadUrl: "/api/vesc_parameters_load",
    ownerKey: "group",
  });
const syncVescCalibration = vescParameterPanel("calibration");
const syncVescLimits = vescParameterPanel("limits");

async function pollVescParameters() {
  const live = (await fetchJSON("/api/vesc_parameters")).value;
  syncVescCalibration("calibration", live.parameters);
  syncVescLimits("limits", live.limits);
}

// ---------------------------------------------------------------------
// Startup
// ---------------------------------------------------------------------

// Which binary this is decides what's polled: the real car (`web_gui` on a car)
// has a VESC and no simulated vehicle, opponents or benchmarks.
fetchJSON("/api/config")
  .then((config) => {
    humanMaxSpeedMps = config.human_max_speed_mps;
    humanMaxSteeringRad = config.human_max_steering_rad;
    if (config.hardware) showCarName(config.car_name);
    return config.hardware;
  })
  .catch((err) => {
    console.error(err);
    return false;
  })
  .then((hardware) => (hardware ? startHardwarePanels() : startSimulatorPanels()));

/** How often the current map, vehicle model, autonomous algorithm, SLAM
 *  state, and planner state are re-read, to reflect changes made from
 *  another tab. */
const SELECTION_POLL_MS = 500;

const drawPoller = startPolling(drawLayers.poll, 1000 / drawRateHz);
const topicPoller = startPolling(pollSelectedTopic, 1000 / DEFAULT_TOPIC_RATE_HZ);
startPolling(pollLiveMap, SELECTION_POLL_MS);
startPolling(pollAlgorithms, SELECTION_POLL_MS);
startPolling(pollSlam, SELECTION_POLL_MS);
startPolling(pollDetector, SELECTION_POLL_MS);
startPolling(pollPlanning, SELECTION_POLL_MS);
startPolling(pollRaceLines, SELECTION_POLL_MS);
startPolling(pollDebug, SELECTION_POLL_MS);
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

/** The panels only a simulated vehicle has: its model, and the opponents
 *  racing it. */
function startSimulatorPanels() {
  startPolling(pollVehicleModel, SELECTION_POLL_MS);
  startPolling(pollOpponents, SELECTION_POLL_MS);
  populateVehicleModelOptions()
    .then(() => pollVehicleModel())
    .catch((err) => console.error(err));
}

/** The real car's panels instead: the simulator-only ones are hidden, and
 *  the VESC's shown. */
/** Marks the map as the real car's: a red background, and its name in
 *  the bottom right corner. */
function showCarName(name) {
  document.getElementById("canvas-area").classList.add("hardware");
  const badge = document.getElementById("car-name");
  badge.textContent = name ?? "Real car";
  badge.hidden = false;
  MapView.requestRedraw();
}

function startHardwarePanels() {
  for (const btn of document.querySelectorAll(".panel-nav-btn[data-simulator-only]")) {
    btn.hidden = true;
  }
  vescNavBtn.hidden = false;
  startPolling(pollVesc, VESC_POLL_MS);
  startPolling(pollVescParameters, SELECTION_POLL_MS);
}

// ---------------------------------------------------------------------
// Bottom panel: lap telemetry (see /lap_panel.js)
// ---------------------------------------------------------------------

LapPanel.init({ fetchTelemetry: () => fetchJSON("/api/lap_telemetry") });
