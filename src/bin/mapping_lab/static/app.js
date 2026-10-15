"use strict";

// ---------------------------------------------------------------------
// mapping_lab's frontend: picking a session, the draft parameters, the
// runs, and which run's map is shown and which one compared to it.
//
// The map canvas and its drawing layers are shared with the other web UIs
// (/map_view.js, /draw_layers.js), and so are the parameter sliders
// (/parameter_panel.js) - all loaded before this file, along with
// `fetchJSON` and `startPolling`.
// ---------------------------------------------------------------------

/** How often the lab (session, draft, runs) is polled. */
const LAB_POLL_MS = 500;
/** How often the drawings are polled - they only change when a run is
 *  picked or finishes. */
const DRAW_POLL_MS = 250;

/** The run whose map is shown, and the one laid over it - by id. */
let selectedRun = null;
let comparedRun = null;
let sessionName = null;

// ---------------------------------------------------------------------
// Map canvas
// ---------------------------------------------------------------------

const drawLayers = DrawLayers.create({
  clock: () => performance.now(),
  requestExtras: () => ({ selected: selectedRun, compared: comparedRun }),
  listEl: document.getElementById("layer-list"),
  // The view is the user's: only center it on the first map shown, not on
  // every run picked after - so two runs can be flicked between in place.
  homeOnEveryNewRaster: false,
});

MapView.init({
  layersAt: drawLayers.layersAt,
  worldBounds: drawLayers.worldBounds,
  homeTarget: drawLayers.homeTarget,
  statusTitle: () => sessionName,
  speedMps: () => null,
  isAnimating: drawLayers.isAnimating,
  onFrame: () => drawLayers.renderFreshness(),
});

/** Asks for the drawings again right away, e.g. after picking a run. */
function redraw() {
  drawLayers.poll().catch((err) => console.error(err));
}

// ---------------------------------------------------------------------
// Session
// ---------------------------------------------------------------------

const sessionSelectEl = document.getElementById("session-select");
const sessionLoadBtn = document.getElementById("session-load-btn");
const sessionStatusEl = document.getElementById("session-status");
const sessionInfoEl = document.getElementById("session-info");

function formatBytes(bytes) {
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(0)} KB`;
  return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
}

/** Fills the session picker from the sessions folder, keeping the pick. */
async function refreshSessions() {
  const files = await fetchJSON("/api/sessions");
  const picked = sessionSelectEl.value || sessionName;
  sessionSelectEl.replaceChildren(
    ...files.map((file) => new Option(`${file.name} (${formatBytes(file.size_bytes)})`, file.name)),
  );
  if (files.length === 0) {
    sessionSelectEl.append(new Option("No session recorded yet", ""));
  } else if (files.some((file) => file.name === picked)) {
    sessionSelectEl.value = picked;
  }
  sessionLoadBtn.disabled = files.length === 0;
}

sessionSelectEl.addEventListener("focus", () => refreshSessions().catch((err) => console.error(err)));

sessionLoadBtn.addEventListener("click", async () => {
  const name = sessionSelectEl.value;
  if (!name) return;
  sessionLoadBtn.disabled = true;
  sessionStatusEl.classList.remove("error");
  sessionStatusEl.textContent = `Loading ${name}...`;
  try {
    await fetchJSON("/api/session", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name }),
    });
    selectedRun = comparedRun = null;
    sessionStatusEl.textContent = "";
    await pollLab();
    redraw();
  } catch (err) {
    sessionStatusEl.classList.add("error");
    sessionStatusEl.textContent = `Couldn't load it: ${err.message}`;
  } finally {
    sessionLoadBtn.disabled = false;
  }
});

function renderSession(lab) {
  const session = lab.session;
  sessionName = session ? session.name : null;
  sessionInfoEl.hidden = session === null;
  if (session === null) {
    if (!sessionStatusEl.classList.contains("error")) {
      sessionStatusEl.textContent = lab.load_error ?? `Pick a session from ${lab.sessions_folder}/ and load it.`;
    }
    return;
  }
  document.getElementById("session-duration").textContent = `${session.duration_s.toFixed(1)} s`;
  document.getElementById("session-scans").textContent =
    `${session.scans.toLocaleString()} (${session.odometry.toLocaleString()} odometry samples)`;
  document.getElementById("session-source").textContent = session.car
    ? `the car ${session.car}`
    : session.ground_truth
      ? "the simulator, with ground truth"
      : "the simulator";
  let map = session.map ?? "none selected";
  if (session.true_map_error) map += ` (not loaded: ${session.true_map_error})`;
  document.getElementById("session-map").textContent = map;
}

// ---------------------------------------------------------------------
// Draft parameters
// ---------------------------------------------------------------------

const syncDraftParameters = createParameterPanel({
  containerEl: document.getElementById("draft-parameters"),
  saveEl: document.getElementById("draft-save"),
  setUrl: "/api/draft_parameter",
  saveUrl: "/api/draft_parameters_save",
  loadUrl: "/api/draft_parameters_load",
});

document.getElementById("draft-reset-btn").addEventListener("click", () => {
  fetchJSON("/api/draft_reset", { method: "POST" })
    .then(pollLab)
    .catch((err) => showRunStatus(`Couldn't reset: ${err.message}`, true));
});

// ---------------------------------------------------------------------
// Runs
// ---------------------------------------------------------------------

const runListEl = document.getElementById("run-list");
const runStatusEl = document.getElementById("run-status");
const runButtons = [document.getElementById("run-btn"), document.getElementById("draft-run-btn")];

function showRunStatus(text, isError = false) {
  runStatusEl.textContent = text;
  runStatusEl.classList.toggle("error", isError);
}

for (const button of runButtons) {
  button.addEventListener("click", async () => {
    try {
      // Let a slider move still on its way land first.
      await new Promise((resolve) => setTimeout(resolve, PARAMETER_SEND_INTERVAL_MS));
      const { id } = await fetchJSON("/api/runs", { method: "POST" });
      showRunStatus(`Run #${id} queued.`);
      await pollLab();
    } catch (err) {
      showRunStatus(`Couldn't run: ${err.message}`, true);
    }
  });
}

/** POSTs `{id}` to `url`, then polls - reporting a failure as `what`. */
async function runAction(url, id, what) {
  try {
    await fetchJSON(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ id }),
    });
    await pollLab();
  } catch (err) {
    showRunStatus(`Couldn't ${what}: ${err.message}`, true);
  }
}

const STATE_LABELS = {
  queued: "Queued",
  running: "Running",
  done: "Done",
  failed: "Failed",
  cancelled: "Cancelled",
};

/** Rendered run cards, by id - updated in place, so a map name being typed
 *  survives the polls. */
const runCards = new Map();

function percent(value) {
  return `${(value * 100).toFixed(0)}%`;
}

function describeChanges(run) {
  if (run.changes.length === 0) return "The session's own parameters";
  return run.changes.map((c) => `${c.name} ${c.baseline} → ${c.value}`).join(", ");
}

function describeStats(stats) {
  const parts = [`${stats.kept.toLocaleString()} scans kept`];
  parts.push(`${stats.loop_closures} loop closure${stats.loop_closures === 1 ? "" : "s"}`);
  if (stats.mean_response !== null) {
    parts.push(`match ${percent(stats.mean_response)} on average, ${percent(stats.min_response)} at worst`);
  }
  if (stats.mean_process_ms !== null) {
    parts.push(`${stats.mean_process_ms.toFixed(1)} ms a scan (${stats.max_process_ms.toFixed(0)} at most)`);
  }
  if (stats.dropped > 0) parts.push(`${stats.dropped} dropped`);
  parts.push(`map ${stats.map_width_m.toFixed(0)} × ${stats.map_height_m.toFixed(0)} m`);
  parts.push(`built in ${stats.elapsed_s.toFixed(1)} s`);
  return parts.join(" · ");
}

function buildRunCard(id) {
  const li = document.createElement("li");
  li.className = "run";
  li.innerHTML = `
    <div class="run-head"><span class="run-id"></span><span class="run-state"></span></div>
    <p class="run-changes"></p>
    <progress class="run-progress" max="1" value="0"></progress>
    <p class="run-stats"></p>
    <div class="run-actions">
      <button type="button" class="run-compare" title="Lay this run's walls and trajectory over the shown map">Compare</button>
      <button type="button" class="run-use" title="Copy this run's parameters into the draft">Use parameters</button>
      <button type="button" class="run-save-toggle" title="Save this run's map under maps/">Save map…</button>
      <button type="button" class="run-cancel" title="Stop building it">Cancel</button>
      <button type="button" class="run-delete" title="Forget this run">Delete</button>
    </div>
    <form class="run-save" hidden>
      <input type="text" required placeholder="Map name" title="Name of the new folder under maps/" />
      <button type="submit">Save</button>
    </form>
    <p class="run-save-status"></p>`;
  const el = (selector) => li.querySelector(selector);
  li.addEventListener("click", (event) => {
    if (event.target.closest("button, form") || el(".run-state").dataset.state !== "done") return;
    selectedRun = id;
    if (comparedRun === id) comparedRun = null;
    renderSelection();
    redraw();
  });
  el(".run-compare").addEventListener("click", () => {
    comparedRun = comparedRun === id ? null : id;
    renderSelection();
    redraw();
  });
  el(".run-use").addEventListener("click", () => runAction("/api/runs/use", id, "use its parameters"));
  el(".run-cancel").addEventListener("click", () => runAction("/api/runs/cancel", id, "cancel it"));
  el(".run-delete").addEventListener("click", () => runAction("/api/runs/delete", id, "delete it"));
  el(".run-save-toggle").addEventListener("click", () => {
    el(".run-save").hidden = !el(".run-save").hidden;
    if (!el(".run-save").hidden) el(".run-save input").focus();
  });
  el(".run-save").addEventListener("submit", async (event) => {
    event.preventDefault();
    const statusEl = el(".run-save-status");
    statusEl.classList.remove("error");
    statusEl.textContent = "Saving...";
    try {
      const { path } = await fetchJSON("/api/runs/save_map", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ id, name: el(".run-save input").value.trim() }),
      });
      statusEl.textContent = `Saved to ${path}.`;
      el(".run-save").hidden = true;
    } catch (err) {
      statusEl.classList.add("error");
      statusEl.textContent = `Couldn't save: ${err.message}`;
    }
  });
  return li;
}

function renderRuns(runs) {
  const ids = new Set(runs.map((run) => run.id));
  for (const [id, card] of runCards) {
    if (!ids.has(id)) {
      card.remove();
      runCards.delete(id);
    }
  }
  if (!ids.has(selectedRun)) selectedRun = null;
  if (!ids.has(comparedRun)) comparedRun = null;

  let previous = null;
  for (const run of runs) {
    let card = runCards.get(run.id);
    if (!card) {
      card = buildRunCard(run.id);
      runCards.set(run.id, card);
    }
    // Newest first, as the server lists them.
    if (card.previousElementSibling !== previous || card.parentElement !== runListEl) {
      if (previous) previous.after(card);
      else runListEl.prepend(card);
    }
    previous = card;

    const el = (selector) => card.querySelector(selector);
    const done = run.state === "done";
    const active = run.state === "queued" || run.state === "running";
    el(".run-id").textContent = `#${run.id}`;
    el(".run-state").textContent =
      run.state === "running" ? `Running, ${percent(run.progress)}` : STATE_LABELS[run.state];
    el(".run-state").dataset.state = run.state;
    el(".run-changes").textContent = describeChanges(run);
    el(".run-progress").hidden = !active;
    el(".run-progress").value = run.progress;
    el(".run-stats").textContent = done ? describeStats(run.stats) : run.error ?? "";
    el(".run-stats").classList.toggle("error", run.state === "failed");
    el(".run-compare").hidden = !done;
    el(".run-save-toggle").hidden = !done;
    el(".run-cancel").hidden = !active;
    el(".run-delete").hidden = active;
    if (!done) el(".run-save").hidden = true;
  }

  // Show the first run to finish, rather than an empty canvas.
  if (selectedRun === null) {
    const first = [...runs].reverse().find((run) => run.state === "done");
    if (first) {
      selectedRun = first.id;
      redraw();
    }
  }
  renderSelection();

  if (runs.length === 0 && sessionName !== null) {
    showRunStatus("No run yet: run the draft to build the map with the session's own parameters.");
  } else if (runs.length > 0 && !runStatusEl.classList.contains("error")) {
    showRunStatus("");
  }
}

function renderSelection() {
  for (const [id, card] of runCards) {
    card.classList.toggle("selected", id === selectedRun);
    card.classList.toggle("compared", id === comparedRun);
    card.querySelector(".run-compare").textContent = id === comparedRun ? "Stop comparing" : "Compare";
    card.querySelector(".run-compare").disabled = id === selectedRun;
  }
}

// ---------------------------------------------------------------------
// Polling
// ---------------------------------------------------------------------

async function pollLab() {
  const lab = await fetchJSON("/api/lab");
  renderSession(lab);
  for (const button of runButtons) button.disabled = lab.session === null;
  syncDraftParameters("draft", lab.session === null ? [] : lab.parameters);
  renderRuns(lab.runs);
}

refreshSessions().catch((err) => console.error(err));
startPolling(pollLab, LAB_POLL_MS);
startPolling(drawLayers.poll, DRAW_POLL_MS);
