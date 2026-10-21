"use strict";

// ---------------------------------------------------------------------
// localization_lab's frontend: picking a session, the race line and the
// reference runs are scored against, the draft pipeline, the runs, and
// which of them are shown - on the map, in the charts under it, and in the
// Compare table - over which lap and at which time.
//
// The map canvas and its drawing layers are shared with the other web UIs
// (/map_view.js, /draw_layers.js), and so are the parameter sliders
// (/parameter_panel.js) and the chart helpers (/chart.js) - all loaded
// before this file, along with `fetchJSON` and `startPolling`. The charts
// themselves are /charts.js.
// ---------------------------------------------------------------------

/** How often the lab (session, draft, runs) is polled. */
const LAB_POLL_MS = 500;
/** How often the drawings are polled. */
const DRAW_POLL_MS = 200;
/** At most this many runs are shown at once - one color each. */
const MAX_SHOWN = 8;

/** Run colors, by position among the shown runs - the server's. */
let colors = ["#3b8eff", "#ffb020", "#3bd16f", "#c07bff", "#2ed8e6", "#ff5c5c", "#e8e850", "#ffffff"];
/** The shown runs' ids, in the order they were ticked. */
let shownRuns = [];
/** The lap shown - null for the whole session. */
let lap = null;
/** The time the vehicles are drawn at, in seconds - null before any. */
let timeS = null;
let playing = false;
/** The last `/api/lab` answer. */
let labState = null;
let sessionName = null;

function colorOf(id) {
  const index = shownRuns.indexOf(id);
  return index < 0 ? null : colors[index % colors.length];
}

// ---------------------------------------------------------------------
// Map canvas
// ---------------------------------------------------------------------

const drawLayers = DrawLayers.create({
  clock: () => performance.now(),
  requestExtras: () => ({
    runs: shownRuns,
    lap,
    t_us: timeS === null ? null : Math.round(timeS * 1e6),
  }),
  listEl: document.getElementById("layer-list"),
  homeOnEveryNewRaster: false,
});

MapView.init({
  layersAt: drawLayers.layersAt,
  worldBounds: drawLayers.worldBounds,
  homeTarget: drawLayers.homeTarget,
  statusTitle: () => sessionName,
  speedMps: () => null,
  isAnimating: () => playing || drawLayers.isAnimating(),
  onFrame: () => drawLayers.renderFreshness(),
});

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
const sessionWarningEl = document.getElementById("session-warning");

function formatBytes(bytes) {
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(0)} KB`;
  return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
}

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
    await postJSON("/api/session", { name });
    // A new session, even under the same name: forget what was shown.
    sessionName = null;
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

function postJSON(url, body) {
  return fetchJSON(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body ?? {}),
  });
}

function renderSession(lab) {
  const session = lab.session;
  const firstSession = session !== null && session.name !== sessionName;
  sessionName = session ? session.name : null;
  if (firstSession) {
    // Show what came back with it, up to the limit.
    shownRuns = lab.runs.filter((run) => run.state === "done").slice(-MAX_SHOWN).map((run) => run.id);
    lap = null;
    timeS = null;
  }
  sessionInfoEl.hidden = session === null;
  if (session === null) {
    sessionWarningEl.hidden = true;
    if (!sessionStatusEl.classList.contains("error")) {
      sessionStatusEl.textContent = lab.load_error ?? `Pick a session from ${lab.sessions_folder}/ and load it.`;
    }
    return;
  }
  if (firstSession) setTimeout(() => MapView.home(), 300);
  document.getElementById("session-duration").textContent = `${session.duration_s.toFixed(1)} s`;
  let scans = `${session.scans.toLocaleString()}`;
  if (session.scan_period_ms !== null) scans += `, one every ${session.scan_period_ms.toFixed(0)} ms`;
  document.getElementById("session-scans").textContent = scans;
  document.getElementById("session-source").textContent = session.car
    ? `the car ${session.car}`
    : session.ground_truth
      ? "the simulator, with ground truth"
      : "the simulator";
  document.getElementById("session-map").textContent = session.map ?? "-";
  document.getElementById("session-driver").textContent = session.driver
    ? `${session.driver}${session.driven_on_localization ? ", on localization's pose" : ", on the ground truth"}`
    : "a human";
  sessionWarningEl.hidden = !session.driven_on_localization || !session.ground_truth;
  sessionWarningEl.textContent =
    "The algorithm driving took its pose from localization: the drive depends on the localization that ran, so it's not a fair comparison.";
}

// ---------------------------------------------------------------------
// Scoring: race line and reference
// ---------------------------------------------------------------------

const raceLineSelectEl = document.getElementById("race-line-select");
const referenceSelectEl = document.getElementById("reference-select");
const contextStatusEl = document.getElementById("context-status");

/** Replaces `select`'s options with `options` (`[value, label]`), only if
 *  they changed, keeping the pick. */
function setOptions(select, options, value) {
  const same =
    select.options.length === options.length &&
    options.every(([v, label], i) => select.options[i].value === v && select.options[i].text === label);
  if (!same) select.replaceChildren(...options.map(([v, label]) => new Option(label, v)));
  if (document.activeElement !== select && value !== null) select.value = value;
}

function referenceValue(reference) {
  return reference.kind === "ground_truth" ? "ground_truth" : `run:${reference.run}`;
}

function renderContext(lab) {
  const session = lab.session;
  if (session === null) {
    setOptions(raceLineSelectEl, [], null);
    setOptions(referenceSelectEl, [], null);
    contextStatusEl.textContent = "";
    return;
  }
  setOptions(
    raceLineSelectEl,
    session.race_lines.map((file) => [file, file]),
    lab.context?.race_line ?? null,
  );
  const options = [];
  if (session.ground_truth) options.push(["ground_truth", "Ground truth"]);
  for (const run of lab.runs.filter((run) => run.state === "done")) {
    options.push([`run:${run.id}`, `Run #${run.id}: ${run.label}`]);
  }
  setOptions(referenceSelectEl, options, lab.context ? referenceValue(lab.context.reference) : null);
  contextStatusEl.classList.toggle("error", lab.context_error !== null);
  contextStatusEl.textContent =
    lab.context_error ??
    (lab.context
      ? `${lab.context.laps.length} lap${lab.context.laps.length === 1 ? "" : "s"} of ${lab.context.lap_m.toFixed(1)} m (lap 0 up to the first crossing).`
      : "");
}

raceLineSelectEl.addEventListener("change", () => {
  postJSON("/api/race_line", { file: raceLineSelectEl.value })
    .then(() => pollLab())
    .then(() => refreshAll())
    .catch((err) => showRunStatus(`Couldn't pick it: ${err.message}`, true));
});

referenceSelectEl.addEventListener("change", () => {
  const value = referenceSelectEl.value;
  const reference = value === "ground_truth" ? { kind: "ground_truth" } : { kind: "run", run: Number(value.split(":")[1]) };
  postJSON("/api/reference", { reference })
    .then(() => pollLab())
    .then(() => refreshAll())
    .catch((err) => showRunStatus(`Couldn't pick it: ${err.message}`, true));
});

// ---------------------------------------------------------------------
// Draft
// ---------------------------------------------------------------------

const draftOdometryEl = document.getElementById("draft-odometry");
const draftAlgorithmEl = document.getElementById("draft-algorithm");
const draftLabelEl = document.getElementById("draft-label");
const scanMatchingSettingsEl = document.getElementById("scan-matching-settings");
const particleFilterSettingsEl = document.getElementById("particle-filter-settings");

const syncScanMatching = createParameterPanel({
  containerEl: document.getElementById("scan-matching-parameters"),
  saveEl: document.getElementById("scan-matching-save"),
  setUrl: "/api/draft/scan_matching_parameter",
  saveUrl: "/api/draft/scan_matching_parameters_save",
  loadUrl: "/api/draft/scan_matching_parameters_load",
});

const syncParticleFilter = createParameterPanel({
  containerEl: document.getElementById("particle-filter-parameters"),
  saveEl: document.getElementById("particle-filter-save"),
  setUrl: "/api/draft/particle_filter_parameter",
  saveUrl: "/api/draft/particle_filter_parameters_save",
  loadUrl: "/api/draft/particle_filter_parameters_load",
});

/** Until the server reports the draft this tab last sent. */
let pendingDraft = null;

function sendDraft() {
  const draft = {
    odometry: draftOdometryEl.value,
    algorithm: draftAlgorithmEl.value === "none" ? null : draftAlgorithmEl.value,
  };
  pendingDraft = draft;
  postJSON("/api/draft", draft)
    .then(() => pollLab())
    .catch((err) => showRunStatus(`Couldn't change the draft: ${err.message}`, true));
}

draftOdometryEl.addEventListener("change", sendDraft);
draftAlgorithmEl.addEventListener("change", sendDraft);

document.getElementById("draft-reset-btn").addEventListener("click", () => {
  postJSON("/api/draft_reset")
    .then(() => pollLab())
    .catch((err) => showRunStatus(`Couldn't reset: ${err.message}`, true));
});

function renderDraft(draft) {
  if (draft === null) {
    syncScanMatching("draft", []);
    syncParticleFilter("draft", []);
    draftLabelEl.textContent = "";
    return;
  }
  if (
    pendingDraft !== null &&
    pendingDraft.odometry === draft.odometry &&
    pendingDraft.algorithm === draft.algorithm
  ) {
    pendingDraft = null;
  }
  if (pendingDraft === null) {
    draftOdometryEl.value = draft.odometry;
    draftAlgorithmEl.value = draft.algorithm ?? "none";
  }
  draftLabelEl.textContent = `Draft: ${draft.label}.`;
  scanMatchingSettingsEl.hidden = draft.algorithm !== "scan_matching";
  particleFilterSettingsEl.hidden = draft.algorithm !== "particle_filter";
  syncScanMatching("draft", draft.algorithm === "scan_matching" ? draft.scan_matching : []);
  syncParticleFilter("draft", draft.algorithm === "particle_filter" ? draft.particle_filter : []);
}

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
      const { id } = await postJSON("/api/runs");
      showRunStatus(`Run #${id} queued.`);
      if (shownRuns.length < MAX_SHOWN) shownRuns.push(id);
      await pollLab();
    } catch (err) {
      showRunStatus(`Couldn't run: ${err.message}`, true);
    }
  });
}

async function runAction(url, id, what) {
  try {
    await postJSON(url, { id });
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

function percent(value) {
  return `${(value * 100).toFixed(0)}%`;
}

function degrees(rad) {
  return `${((rad * 180) / Math.PI).toFixed(1)}°`;
}

function cm(m) {
  return `${(m * 100).toFixed(1)} cm`;
}

/** The scores runs are compared by: each one's value in a lap's scores
 *  (null if it has none), whether lower is better, and how it reads. */
const SCORES = [
  { key: "rms", label: "Position RMS", value: (s) => s.rms_m, format: cm },
  { key: "p95", label: "Position p95", value: (s) => s.p95_m, format: cm },
  { key: "max", label: "Position worst", value: (s) => s.max_m, format: cm },
  { key: "cross", label: "Cross-track RMS", value: (s) => s.cross_rms_m, format: cm },
  { key: "heading", label: "Heading RMS", value: (s) => s.heading_rms_rad, format: degrees },
  { key: "drift", label: "Drift over 5 m", value: (s) => s.drift_rms_m, format: cm },
  { key: "lost", label: "Time lost", value: (s) => s.lost_s, format: (v) => `${v.toFixed(1)} s` },
  { key: "mean_ms", label: "Time per scan", value: (s) => s.mean_ms, format: (v) => `${v.toFixed(2)} ms` },
  { key: "p95_ms", label: "Time per scan p95", value: (s) => s.p95_ms, format: (v) => `${v.toFixed(2)} ms` },
  { key: "slow", label: "Scans too slow", value: (s) => (s.scans > 0 ? s.slow_scans : null), format: (v) => `${v}` },
];

/** A run's scores over `lapNumber` (null for the whole drive), or null. */
function lapScores(run, lapNumber) {
  if (run.state !== "done") return null;
  return run.stats.scores.find((s) => s.lap === lapNumber) ?? null;
}

/** The best value of every score among `runs` over `lapNumber`, by key. */
function bestScores(runs, lapNumber) {
  const best = {};
  for (const score of SCORES) {
    const values = runs
      .map((run) => lapScores(run, lapNumber))
      .filter((s) => s !== null)
      .map((s) => score.value(s))
      .filter((v) => v !== null && v !== undefined);
    if (values.length > 1) best[score.key] = Math.min(...values);
  }
  return best;
}

const runCards = new Map();

function buildRunCard(id) {
  const li = document.createElement("li");
  li.className = "run";
  li.innerHTML = `
    <div class="run-head"><span class="run-id"><span class="run-swatch"></span><span class="run-name"></span></span><span class="run-state"></span></div>
    <p class="run-changes"></p>
    <p class="run-tags"></p>
    <progress class="run-progress" max="1" value="0"></progress>
    <dl class="run-scores"></dl>
    <p class="run-stats"></p>
    <div class="run-actions">
      <label class="run-show-label" title="Draw this run on the map and in the charts"><input type="checkbox" class="run-show" /> Show</label>
      <button type="button" class="run-use" title="Copy this run's pipeline into the draft">Use</button>
      <button type="button" class="run-reference" title="Score every run against this one - on the car, where there's no ground truth">Reference</button>
      <button type="button" class="run-cancel" title="Stop localizing">Cancel</button>
      <button type="button" class="run-delete" title="Forget this run, and delete its saved file">Delete</button>
    </div>`;
  const el = (selector) => li.querySelector(selector);
  el(".run-show").addEventListener("change", (event) => {
    if (event.target.checked) {
      if (shownRuns.length >= MAX_SHOWN) {
        event.target.checked = false;
        showRunStatus(`At most ${MAX_SHOWN} runs are shown at once: untick one first.`, true);
        return;
      }
      shownRuns.push(id);
    } else {
      shownRuns = shownRuns.filter((shown) => shown !== id);
    }
    refreshAll();
  });
  el(".run-use").addEventListener("click", () => runAction("/api/runs/use", id, "use it"));
  el(".run-cancel").addEventListener("click", () => runAction("/api/runs/cancel", id, "cancel it"));
  el(".run-delete").addEventListener("click", () => runAction("/api/runs/delete", id, "delete it"));
  el(".run-reference").addEventListener("click", () => {
    postJSON("/api/reference", { reference: { kind: "run", run: id } })
      .then(() => pollLab())
      .then(() => refreshAll())
      .catch((err) => showRunStatus(`Couldn't use it as the reference: ${err.message}`, true));
  });
  return li;
}

function describeChanges(run) {
  if (run.changes.length === 0) return "The session's own parameters";
  return run.changes.map((c) => `${c.name} ${c.baseline} → ${c.value}`).join(", ");
}

function describeStats(stats) {
  const parts = [`localized in ${stats.elapsed_s.toFixed(1)} s`];
  if (stats.scans > 0) parts.push(`${stats.scans.toLocaleString()} scans`);
  if (stats.dropped_scans > 0) parts.push(`${stats.dropped_scans} dropped`);
  if (stats.odometry_us_per_sample !== null) parts.push(`odometry ${stats.odometry_us_per_sample.toFixed(1)} µs a sample`);
  if (stats.commit) parts.push(`commit ${stats.commit.slice(0, 8)}${stats.dirty ? " (with changes)" : ""}`);
  return parts.join(" · ");
}

function renderRuns(lab) {
  const runs = lab.runs;
  const ids = new Set(runs.map((run) => run.id));
  shownRuns = shownRuns.filter((id) => ids.has(id));
  for (const [id, card] of runCards) {
    if (!ids.has(id)) {
      card.remove();
      runCards.delete(id);
    }
  }
  const shown = runs.filter((run) => shownRuns.includes(run.id));
  const best = bestScores(shown, null);
  const reference = lab.context?.reference;
  // Newest first.
  let previous = null;
  for (const run of [...runs].reverse()) {
    let card = runCards.get(run.id);
    if (!card) {
      card = buildRunCard(run.id);
      runCards.set(run.id, card);
    }
    if (card.previousElementSibling !== previous || card.parentElement !== runListEl) {
      if (previous) previous.after(card);
      else runListEl.prepend(card);
    }
    previous = card;

    const el = (selector) => card.querySelector(selector);
    const done = run.state === "done";
    const active = run.state === "queued" || run.state === "running";
    const color = colorOf(run.id);
    card.classList.toggle("shown", color !== null);
    card.style.setProperty("--run-color", color ?? "transparent");
    el(".run-name").textContent = `#${run.id} ${run.label}`;
    el(".run-state").textContent = run.state === "running" ? `Running, ${percent(run.progress)}` : STATE_LABELS[run.state];
    el(".run-state").dataset.state = run.state;
    el(".run-changes").textContent = run.spec.algorithm === null ? "" : describeChanges(run);
    const isReference = reference?.kind === "run" && reference.run === run.id;
    el(".run-tags").textContent = isReference ? "The reference every run is scored against." : "";
    el(".run-progress").hidden = !active;
    el(".run-progress").value = run.progress;
    const scores = done ? lapScores(run, null) : null;
    const dl = el(".run-scores");
    dl.replaceChildren();
    if (scores) {
      for (const score of SCORES) {
        const value = score.value(scores);
        if (value === null || value === undefined) continue;
        const dt = document.createElement("dt");
        dt.textContent = score.label;
        const dd = document.createElement("dd");
        dd.textContent = score.format(value);
        dd.classList.toggle("best", color !== null && best[score.key] === value);
        dl.append(dt, dd);
      }
    }
    dl.hidden = !scores;
    el(".run-stats").textContent = done ? describeStats(run.stats) : (run.error ?? "");
    el(".run-stats").classList.toggle("error", run.state === "failed");
    el(".run-show").checked = color !== null;
    el(".run-show-label").hidden = !done;
    el(".run-reference").hidden = !done || isReference;
    el(".run-cancel").hidden = !active;
    el(".run-delete").hidden = active;
  }
  if (runs.length === 0 && sessionName !== null) {
    showRunStatus("No run yet: run the draft to localize the session as it was recorded.");
  } else if (runs.length > 0 && !runStatusEl.classList.contains("error")) {
    showRunStatus("");
  }
}

// ---------------------------------------------------------------------
// Compare table
// ---------------------------------------------------------------------

const compareScoreEl = document.getElementById("compare-score");
const compareTableEl = document.getElementById("compare-table");
compareScoreEl.replaceChildren(...SCORES.map((score) => new Option(score.label, score.key)));
compareScoreEl.addEventListener("change", () => labState && renderCompare(labState));

function renderCompare(lab) {
  const score = SCORES.find((s) => s.key === compareScoreEl.value) ?? SCORES[0];
  const runs = lab.runs.filter((run) => run.state === "done");
  const laps = lab.context ? lab.context.laps.map((span) => span.lap) : [];
  const columns = [null, ...laps];
  const header = document.createElement("tr");
  for (const text of ["Run", "Whole", ...laps.map((l) => `Lap ${l}`)]) {
    const th = document.createElement("th");
    th.textContent = text;
    header.append(th);
  }
  const rows = [header];
  const best = Object.fromEntries(columns.map((column) => [column, bestScores(runs, column)[score.key]]));
  for (const run of runs) {
    const tr = document.createElement("tr");
    const name = document.createElement("td");
    name.textContent = `#${run.id} ${run.label}`;
    const color = colorOf(run.id);
    if (color) name.style.color = color;
    tr.append(name);
    for (const column of columns) {
      const td = document.createElement("td");
      const scores = lapScores(run, column);
      const value = scores ? score.value(scores) : null;
      td.textContent = value === null || value === undefined ? "-" : score.format(value);
      td.classList.toggle("best", value !== null && value === best[column]);
      tr.append(td);
    }
    rows.push(tr);
  }
  compareTableEl.replaceChildren(...rows);
}

// ---------------------------------------------------------------------
// Lap, time and charts
// ---------------------------------------------------------------------

const lapSelectEl = document.getElementById("lap-select");
const timeSliderEl = document.getElementById("time-slider");
const timeLabelEl = document.getElementById("time-label");
const playBtn = document.getElementById("play-btn");
const chartsHintEl = document.getElementById("charts-hint");
const chartCanvases = Object.fromEntries(
  [...document.querySelectorAll("#charts-grid canvas")].map((canvas) => [canvas.dataset.chart, canvas]),
);

/** The shown runs' series, as last fetched, and what they were fetched
 *  for. */
let series = [];
let seriesKey = null;

/** The lap span shown, in seconds: `{start_s, end_s}`. */
function span() {
  const laps = labState?.context?.laps ?? [];
  if (lap !== null) {
    const found = laps.find((l) => l.lap === lap);
    if (found) return found;
  }
  return { start_s: 0, end_s: labState?.session?.duration_s ?? 0 };
}

function renderLaps(lab) {
  const laps = lab.context?.laps ?? [];
  if (lap !== null && !laps.some((l) => l.lap === lap)) lap = null;
  setOptions(
    lapSelectEl,
    [["all", "All laps"], ...laps.map((l) => [String(l.lap), `Lap ${l.lap} (${(l.end_s - l.start_s).toFixed(1)} s)`])],
    lap === null ? "all" : String(lap),
  );
}

lapSelectEl.addEventListener("change", () => {
  lap = lapSelectEl.value === "all" ? null : Number(lapSelectEl.value);
  const { start_s } = span();
  timeS = start_s;
  refreshAll();
});

function setTime(seconds) {
  const { start_s, end_s } = span();
  timeS = Math.min(end_s, Math.max(start_s, seconds));
  renderTime();
  renderCharts();
  redraw();
}

function renderTime() {
  const { start_s, end_s } = span();
  timeSliderEl.min = String(start_s);
  timeSliderEl.max = String(Math.max(start_s + 0.01, end_s));
  if (timeS !== null && document.activeElement !== timeSliderEl) timeSliderEl.value = String(timeS);
  timeLabelEl.textContent = timeS === null ? "" : `${timeS.toFixed(2)} s`;
}

timeSliderEl.addEventListener("input", () => setTime(Number(timeSliderEl.value)));

playBtn.addEventListener("click", () => {
  playing = !playing;
  playBtn.innerHTML = playing ? "&#10074;&#10074;" : "&#9654;";
  if (playing) {
    const { start_s, end_s } = span();
    if (timeS === null || timeS >= end_s) timeS = start_s;
    let last = performance.now();
    const step = (now) => {
      if (!playing) return;
      const dt = (now - last) / 1000;
      last = now;
      const { end_s: end } = span();
      if (timeS + dt >= end) {
        setTime(end);
        playing = false;
        playBtn.innerHTML = "&#9654;";
        return;
      }
      setTime(timeS + dt);
      requestAnimationFrame(step);
    };
    requestAnimationFrame(step);
  }
});

/** Fetches the shown runs' series again if what they're for changed. */
async function refreshSeries(force = false) {
  const done = new Set((labState?.runs ?? []).filter((run) => run.state === "done").map((run) => run.id));
  const ids = shownRuns.filter((id) => done.has(id));
  const contextKey = labState?.context ? `${labState.context.race_line}|${referenceValue(labState.context.reference)}` : "";
  const key = `${ids.join(",")}|${lap}|${contextKey}|${sessionName}`;
  if (!force && key === seriesKey) return;
  seriesKey = key;
  if (ids.length === 0 || !labState?.context) {
    series = [];
  } else {
    const lapParam = lap === null ? "" : `&lap=${lap}`;
    series = await fetchJSON(`/api/series?runs=${ids.join(",")}${lapParam}`);
  }
  renderCharts();
}

/** Every chart: its title, which column of a run's series it plots, and
 *  whether its values are signed. */
const CHARTS = {
  position: { title: "Position error (m)", column: 2 },
  cross: { title: "Cross-track error (m)", column: 4, signed: true },
  heading: { title: "Heading error (°)", column: 5, signed: true, scale: 180 / Math.PI },
  drift: { title: "Drift over 5 m (m)", column: 6 },
  cost: { title: "Time per scan (ms)", scans: true },
};

function renderCharts() {
  const byLap = lap !== null;
  const lapM = labState?.context?.lap_m ?? 0;
  const { start_s, end_s } = span();
  const xOf = (row) => (byLap ? row[1] : row[0]);
  // The cursor: where the reference was along the line at the scrubbed
  // time - read off the first shown series.
  let cursorX = null;
  if (timeS !== null) {
    if (!byLap) {
      cursorX = timeS;
    } else if (series.length > 0 && series[0].errors.length > 0) {
      let nearest = series[0].errors[0];
      for (const row of series[0].errors) {
        if (Math.abs(row[0] - timeS) < Math.abs(nearest[0] - timeS)) nearest = row;
      }
      cursorX = nearest[1];
    }
  }
  for (const [key, chart] of Object.entries(CHARTS)) {
    const canvas = chartCanvases[key];
    const lines = series.map((runSeries) => {
      const rows = chart.scans ? runSeries.scans : runSeries.errors;
      const points = [];
      for (const row of rows) {
        const y = chart.scans ? row[2] : row[chart.column];
        if (y === null) continue;
        points.push([xOf(row), y * (chart.scale ?? 1)]);
      }
      return { color: colorOf(runSeries.id) ?? "#ffffff", points };
    });
    LabCharts.draw(canvas, {
      title: chart.title,
      series: lines,
      xMin: byLap ? 0 : start_s,
      xMax: byLap ? lapM : end_s,
      xLabel: byLap ? "m along the race line" : "s",
      cursorX,
      signed: chart.signed ?? false,
      empty: labState?.context ? "Show a finished run" : "Nothing to score against",
    });
  }
  chartsHintEl.textContent = series.length === 0 ? "" : "Click a chart to move the vehicles there.";
}

// Clicking a chart moves the time there: by time over the whole session,
// by the nearest point along the line over one lap.
for (const canvas of Object.values(chartCanvases)) {
  canvas.addEventListener("click", (event) => {
    const x = LabCharts.xAt(canvas, event);
    if (x === null) return;
    if (lap === null) {
      setTime(x);
      return;
    }
    const rows = series[0]?.errors ?? [];
    if (rows.length === 0) return;
    let nearest = rows[0];
    for (const row of rows) {
      if (Math.abs(row[1] - x) < Math.abs(nearest[1] - x)) nearest = row;
    }
    setTime(nearest[0]);
  });
}

new ResizeObserver(() => renderCharts()).observe(document.getElementById("charts-grid"));

/** After the shown runs, the lap or the scoring changed. */
function refreshAll() {
  if (labState) {
    renderRuns(labState);
    renderCompare(labState);
    renderLaps(labState);
  }
  renderTime();
  refreshSeries().catch((err) => console.error(err));
  redraw();
}

// ---------------------------------------------------------------------
// Polling
// ---------------------------------------------------------------------

/** Ids of the runs done at the last poll - one finishing fetches the
 *  series again. */
let doneBefore = "";

async function pollLab() {
  const lab = await fetchJSON("/api/lab");
  labState = lab;
  renderSession(lab);
  renderContext(lab);
  for (const button of runButtons) button.disabled = lab.session === null;
  renderDraft(lab.draft);
  renderRuns(lab);
  renderCompare(lab);
  renderLaps(lab);
  if (timeS === null && lab.session) timeS = span().start_s;
  renderTime();
  const done = lab.runs.filter((run) => run.state === "done").map((run) => run.id).join(",");
  if (done !== doneBefore) {
    doneBefore = done;
    seriesKey = null;
    redraw();
  }
  await refreshSeries();
}

fetchJSON("/api/colors")
  .then((served) => (colors = served))
  .catch((err) => console.error(err));
refreshSessions().catch((err) => console.error(err));
startPolling(pollLab, LAB_POLL_MS);
startPolling(drawLayers.poll, DRAW_POLL_MS);
