"use strict";

// ---------------------------------------------------------------------
// Parameter sliders shared by every web UI that tunes parameters
// (web_gui's panels, mapping_lab's runs): `createParameterPanel` and the
// helpers it needs. Needs `fetchJSON` from /map_view.js, loaded before.
// ---------------------------------------------------------------------

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
