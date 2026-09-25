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
    const origin = map.seed == null ? `recorded ${map.generated_at}` : `seed ${map.seed}`;
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
const algorithmStartBtn = document.getElementById("algorithm-start-btn");
const algorithmPauseBtn = document.getElementById("algorithm-pause-btn");

/** Algorithm the `autonomous_algorithm_status` topic last reported selected,
 *  or null before the first poll - tracked separately from the `<select>`'s
 *  own value for the same reason as `liveVehicleModelKind`. */
let liveAlgorithm = null;

/** Whether the selected algorithm was last reported running (not paused). */
let algorithmRunning = false;

/** `{name, label, description, parameters, message}` of every algorithm last reported available. */
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

  syncAlgorithmParameters(status.selected);
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

/** How often the current map, vehicle model, autonomous algorithm, SLAM
 *  state, and planner state are re-read, to reflect changes made from
 *  another tab. */
const SELECTION_POLL_MS = 500;

const drawPoller = startPolling(drawLayers.poll, 1000 / drawRateHz);
const topicPoller = startPolling(pollSelectedTopic, 1000 / DEFAULT_TOPIC_RATE_HZ);
startPolling(pollLiveMap, SELECTION_POLL_MS);
startPolling(pollVehicleModel, SELECTION_POLL_MS);
startPolling(pollAlgorithms, SELECTION_POLL_MS);
startPolling(pollSlam, SELECTION_POLL_MS);
startPolling(pollPlanning, SELECTION_POLL_MS);
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
