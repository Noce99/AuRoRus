"use strict";

// ---------------------------------------------------------------------
// The Sweeps panel: setting up a sweep (up to three parameters, each a
// range), the sweeps queued, and the open one's results - a chart of the
// score it's ranked by (a line over one parameter, a heatmap over two, a
// heatmap per value of a third) and a table of every run.
//
// Loaded after /chart.js and before /app.js, whose `SCORES`, `selectRun`,
// `selectedRun`, `pollLab` and `percent` it uses - only from functions
// called once every script has run.
// ---------------------------------------------------------------------

/** At most this many parameters are swept at once - as the lab checks. */
const MAX_SWEEP_AXES = 3;
/** The parameters a new row offers first, in this order - the ones that
 *  shape the map most - then the rest in the panel's order. */
const SUGGESTED_SWEEP_PARAMETERS = [
  "correlation_search_space_smear_deviation_m",
  "distance_variance_penalty",
  "angle_variance_penalty",
  "loop_match_minimum_response_fine",
];
/** How many values a newly added parameter is swept over. */
const DEFAULT_SWEEP_COUNT = 5;
/** How long the editor waits after a change before asking what the sweep
 *  would queue. */
const SWEEP_PLAN_DELAY_MS = 250;
/** A run takes roughly this fraction of the session's length, before any
 *  run has finished to tell. */
const RUN_TIME_PER_SESSION_S = 0.22;

const sweepAxesEl = document.getElementById("sweep-axes");
const sweepAddAxisBtn = document.getElementById("sweep-add-axis");
const sweepPlanEl = document.getElementById("sweep-plan");
const sweepStartBtn = document.getElementById("sweep-start");
const sweepStatusEl = document.getElementById("sweep-status");
const sweepListEl = document.getElementById("sweep-list");
const sweepResultsEl = document.getElementById("sweep-results");
const sweepScoreEl = document.getElementById("sweep-score");
const sweepSliceRowEl = document.getElementById("sweep-slice-row");
const sweepSliceEl = document.getElementById("sweep-slice");
const sweepChartEl = document.getElementById("sweep-chart");
const sweepChartWrapEl = document.getElementById("sweep-chart-wrap");
const sweepTooltipEl = document.getElementById("sweep-tooltip");
const sweepTableEl = document.getElementById("sweep-table");

/** Every tunable parameter (with the draft's value), from the last poll. */
let sweepParameters = [];
/** The last poll's lab, and its runs by id. */
let sweepLab = null;
let sweepRunsById = new Map();
/** The sweep whose results are shown, by id. */
let openSweepId = null;
/** The third parameter's value index the heatmap shows - per sweep. */
const sweepSlices = new Map();
/** Where each run is on the chart, for hovering and clicking: `{x, y, w,
 *  h, run, cell}` boxes in CSS pixels. */
let chartHits = [];

function sweepStatus(text, isError = false) {
  sweepStatusEl.textContent = text;
  sweepStatusEl.classList.toggle("error", isError);
}

async function postJSON(url, body) {
  return fetchJSON(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
}

function parameterNamed(name) {
  return sweepParameters.find((p) => p.name === name);
}

function rangeOf(parameter) {
  return Object.values(parameter.kind)[0];
}

/** `value` as an axis label: no more digits than it needs. */
function formatValue(value) {
  return String(Number(value.toPrecision(6)));
}

/** A parameter's name, short enough for a column header. */
function shortName(name) {
  return name.replaceAll("_", " ");
}

// ---------------------------------------------------------------------
// Editor
// ---------------------------------------------------------------------

/** One editor row per swept parameter. */
function axisRows() {
  return [...sweepAxesEl.querySelectorAll(".sweep-axis")];
}

function axisSpecs() {
  return axisRows().map((row) => ({
    name: row.querySelector(".sweep-name").value,
    from: Number(row.querySelector(".sweep-from").value),
    to: Number(row.querySelector(".sweep-to").value),
    count: Number(row.querySelector(".sweep-count").value),
  }));
}

/** A range around `parameter`'s draft value: half to one and a half times
 *  it, within its own range. */
function defaultRange(parameter) {
  const { min, max } = rangeOf(parameter);
  const value = parameter.value;
  if (value === 0) return [min, min + (max - min) / 4];
  const [a, b] = [value * 0.5, value * 1.5].sort((x, y) => x - y);
  return [Math.max(min, a), Math.min(max, b)];
}

/** Adds an editor row: `spec` (`{name, from, to, count}`), or the first
 *  parameter not swept yet over its default range. */
function addAxisRow(spec = null) {
  if (axisRows().length >= MAX_SWEEP_AXES || sweepParameters.length === 0) return;
  const used = new Set(axisSpecs().map((s) => s.name));
  if (spec === null) {
    const parameter =
      SUGGESTED_SWEEP_PARAMETERS.map(parameterNamed).find((p) => p && !used.has(p.name)) ??
      sweepParameters.find((p) => !used.has(p.name));
    const [from, to] = defaultRange(parameter);
    spec = { name: parameter.name, from, to, count: DEFAULT_SWEEP_COUNT };
  }
  const row = document.createElement("div");
  row.className = "sweep-axis";
  row.innerHTML = `
    <div class="sweep-axis-head">
      <select class="sweep-name"></select>
      <button type="button" class="sweep-remove" title="Don't sweep this parameter" aria-label="Remove">&times;</button>
    </div>
    <div class="sweep-axis-range">
      <label>from <input type="number" class="sweep-from" step="any" /></label>
      <label>to <input type="number" class="sweep-to" step="any" /></label>
      <label>values <input type="number" class="sweep-count" min="1" step="1" /></label>
    </div>
    <p class="sweep-values"></p>`;
  const nameEl = row.querySelector(".sweep-name");
  nameEl.replaceChildren(
    ...sweepParameters.map((p) => new Option(p.unit ? `${p.name} (${p.unit})` : p.name, p.name)),
  );
  nameEl.value = spec.name;
  row.querySelector(".sweep-from").value = formatValue(spec.from);
  row.querySelector(".sweep-to").value = formatValue(spec.to);
  row.querySelector(".sweep-count").value = spec.count;
  nameEl.addEventListener("change", () => {
    const [from, to] = defaultRange(parameterNamed(nameEl.value));
    row.querySelector(".sweep-from").value = formatValue(from);
    row.querySelector(".sweep-to").value = formatValue(to);
    planSoon();
  });
  for (const input of row.querySelectorAll("input")) input.addEventListener("input", planSoon);
  row.querySelector(".sweep-remove").addEventListener("click", () => {
    row.remove();
    planSoon();
  });
  sweepAxesEl.append(row);
  planSoon();
}

/** Replaces the editor's rows with `specs`. */
function setAxisRows(specs) {
  sweepAxesEl.replaceChildren();
  for (const spec of specs) addAxisRow(spec);
  planSoon();
}

sweepAddAxisBtn.addEventListener("click", () => addAxisRow());

let planTimer = null;
let planGeneration = 0;

function planSoon() {
  clearTimeout(planTimer);
  planTimer = setTimeout(() => plan().catch((err) => console.error(err)), SWEEP_PLAN_DELAY_MS);
  sweepAddAxisBtn.disabled = axisRows().length >= MAX_SWEEP_AXES;
}

/** How long `runs` runs take, roughly: the recent runs' mean - or a
 *  fraction of the session's length before any finished - per batch of as
 *  many as there are workers. */
function estimateSeconds(runs) {
  const done = [...sweepRunsById.values()].filter((run) => run.state === "done").slice(0, 10);
  const perRun = done.length
    ? done.reduce((sum, run) => sum + run.stats.elapsed_s, 0) / done.length
    : (sweepLab?.session?.duration_s ?? 0) * RUN_TIME_PER_SESSION_S;
  return Math.ceil(runs / Math.max(1, sweepLab?.workers ?? 1)) * perRun;
}

function formatDuration(seconds) {
  if (seconds < 90) return `${Math.max(1, Math.round(seconds))} s`;
  return `${Math.round(seconds / 60)} min`;
}

/** Asks the lab what the editor's sweep would queue, and says so. */
async function plan() {
  const generation = ++planGeneration;
  const specs = axisSpecs();
  const rows = axisRows();
  if (specs.length === 0) {
    sweepPlanEl.textContent = sweepParameters.length ? "Add a parameter to sweep." : "";
    sweepPlanEl.classList.remove("error");
    sweepStartBtn.disabled = true;
    return;
  }
  try {
    const result = await postJSON("/api/sweeps/plan", { axes: specs });
    if (generation !== planGeneration) return;
    result.axes.forEach((axis, i) => {
      if (rows[i]) rows[i].querySelector(".sweep-values").textContent = axis.values.map(formatValue).join(", ");
    });
    let text = `${result.runs} run${result.runs === 1 ? "" : "s"}`;
    if (result.skipped > 0) text += ` (${result.skipped} skipped: SLAM can't run with them)`;
    if (result.runs > 0) text += `, about ${formatDuration(estimateSeconds(result.runs))}`;
    sweepPlanEl.textContent = `${text}.`;
    sweepPlanEl.classList.remove("error");
    sweepStartBtn.disabled = result.runs === 0;
  } catch (err) {
    if (generation !== planGeneration) return;
    sweepPlanEl.textContent = err.message;
    sweepPlanEl.classList.add("error");
    sweepStartBtn.disabled = true;
  }
}

sweepStartBtn.addEventListener("click", async () => {
  sweepStartBtn.disabled = true;
  try {
    // Let a draft slider move still on its way land first.
    await new Promise((resolve) => setTimeout(resolve, PARAMETER_SEND_INTERVAL_MS));
    const { id } = await postJSON("/api/sweeps", { axes: axisSpecs() });
    openSweepId = id;
    sweepStatus(`Sweep #${id} queued.`);
    await pollLab();
  } catch (err) {
    sweepStatus(`Couldn't sweep: ${err.message}`, true);
  } finally {
    sweepStartBtn.disabled = false;
  }
});

// ---------------------------------------------------------------------
// Sweep list
// ---------------------------------------------------------------------

function sweepTitle(sweep) {
  return `#${sweep.id} ${sweep.axes.map((axis) => shortName(axis.name)).join(" × ")}`;
}

function renderSweepList(sweeps) {
  sweepListEl.replaceChildren(
    ...sweeps.map((sweep) => {
      const li = document.createElement("li");
      li.className = "sweep";
      li.classList.toggle("selected", sweep.id === openSweepId);
      const active = sweep.over < sweep.runs;
      li.innerHTML = `
        <div class="sweep-head"><span class="sweep-name-text"></span><span class="sweep-progress"></span></div>
        <div class="run-actions">
          <button type="button" class="sweep-cancel" title="Stop its runs still queued or running">Cancel</button>
          <button type="button" class="sweep-delete" title="Forget the sweep and every run of it">Delete</button>
        </div>`;
      li.querySelector(".sweep-name-text").textContent = sweepTitle(sweep);
      li.querySelector(".sweep-progress").textContent = active
        ? `${sweep.over} / ${sweep.runs} runs`
        : `${sweep.runs} runs`;
      li.querySelector(".sweep-cancel").hidden = !active;
      li.addEventListener("click", (event) => {
        if (event.target.closest("button")) return;
        openSweepId = sweep.id;
        renderSweeps(sweepLab);
      });
      li.querySelector(".sweep-cancel").addEventListener("click", () => sweepAction("/api/sweeps/cancel", sweep.id, "cancel it"));
      li.querySelector(".sweep-delete").addEventListener("click", () => sweepAction("/api/sweeps/delete", sweep.id, "delete it"));
      return li;
    }),
  );
}

async function sweepAction(url, id, what) {
  try {
    await postJSON(url, { id });
    await pollLab();
  } catch (err) {
    sweepStatus(`Couldn't ${what}: ${err.message}`, true);
  }
}

// ---------------------------------------------------------------------
// Results
// ---------------------------------------------------------------------

/** The scores at least one done run of `sweep` has. */
function availableScores(sweep) {
  const runs = sweepRuns(sweep);
  return SCORES.filter((score) => runs.some(({ run }) => run.state === "done" && score.value(run.stats) !== null));
}

/** `sweep`'s combinations that have a run: `{cell, run}`. */
function sweepRuns(sweep) {
  return sweep.cells
    .filter((cell) => cell.run !== null && sweepRunsById.has(cell.run))
    .map((cell) => ({ cell: cell.cell, run: sweepRunsById.get(cell.run) }));
}

/** `run`'s value of `score`, or null if it has none (yet). */
function scoreOf(score, run) {
  return run && run.state === "done" ? score.value(run.stats) : null;
}

/** The done run of `entries` best at `score`. */
function bestOf(score, entries) {
  let best = null;
  for (const entry of entries) {
    const value = scoreOf(score, entry.run);
    if (value === null) continue;
    if (best === null || (score.lowerIsBetter ? value < best.value : value > best.value)) best = { ...entry, value };
  }
  return best;
}

/** Fills the Rank by menu with the scores `sweep` has, keeping the pick -
 *  trajectory error first when there's ground truth. */
function renderScoreMenu(sweep) {
  const scores = availableScores(sweep);
  const picked = sweepScoreEl.value;
  const keys = scores.map((score) => score.key);
  if (keys.join() !== [...sweepScoreEl.options].map((o) => o.value).join()) {
    sweepScoreEl.replaceChildren(...scores.map((score) => new Option(`${score.label} (${score.unit})`, score.key)));
  }
  if (keys.includes(picked)) sweepScoreEl.value = picked;
  else if (keys.includes("trajectory")) sweepScoreEl.value = "trajectory";
  else if (keys.includes("thickness")) sweepScoreEl.value = "thickness";
  return SCORES.find((score) => score.key === sweepScoreEl.value) ?? null;
}

sweepScoreEl.addEventListener("change", () => renderSweeps(sweepLab));
sweepSliceEl.addEventListener("change", () => {
  sweepSlices.set(openSweepId, Number(sweepSliceEl.value));
  renderSweeps(sweepLab);
});

/** Renders the Sweeps panel from `lab` - called with every poll. */
function renderSweeps(lab) {
  if (lab === null) return;
  const firstParameters = sweepParameters.length === 0;
  sweepLab = lab;
  sweepParameters = lab.session === null ? [] : lab.parameters;
  sweepRunsById = new Map(lab.runs.map((run) => [run.id, run]));
  if (firstParameters && sweepParameters.length > 0 && axisRows().length === 0) addAxisRow();
  if (sweepParameters.length === 0) sweepAxesEl.replaceChildren();
  sweepAddAxisBtn.disabled = sweepParameters.length === 0 || axisRows().length >= MAX_SWEEP_AXES;
  if (sweepParameters.length === 0) {
    sweepStartBtn.disabled = true;
    sweepPlanEl.textContent = "Load a session first.";
  }

  if (!lab.sweeps.some((sweep) => sweep.id === openSweepId)) openSweepId = lab.sweeps[0]?.id ?? null;
  renderSweepList(lab.sweeps);
  const sweep = lab.sweeps.find((s) => s.id === openSweepId) ?? null;
  sweepResultsEl.hidden = sweep === null;
  if (sweep === null) return;

  document.getElementById("sweep-title").textContent = `Sweep ${sweepTitle(sweep)}`;
  const score = renderScoreMenu(sweep);
  const entries = sweepRuns(sweep);
  const best = score ? bestOf(score, entries) : null;
  document.getElementById("sweep-use-best").disabled = best === null;
  document.getElementById("sweep-narrow").disabled = best === null;
  const csvLink = document.getElementById("sweep-csv");
  csvLink.href = `/api/sweeps/csv?id=${sweep.id}`;
  csvLink.download = `sweep${sweep.id}.csv`;
  const savedEl = document.getElementById("sweep-saved");
  savedEl.textContent = sweep.saved_to
    ? `Results written to ${sweep.saved_to}.`
    : sweep.save_error && sweep.over === sweep.runs
      ? sweep.save_error
      : `The results are written next to the session once every run is over.`;
  savedEl.classList.toggle("error", Boolean(sweep.save_error) && sweep.over === sweep.runs && !sweep.saved_to);

  // The heatmap's third parameter.
  const sliced = sweep.axes.length === 3;
  sweepSliceRowEl.hidden = !sliced;
  if (sliced) {
    const axis = sweep.axes[2];
    if (!sweepSlices.has(sweep.id) && best) sweepSlices.set(sweep.id, best.cell[2]);
    const slice = Math.min(sweepSlices.get(sweep.id) ?? 0, axis.values.length - 1);
    document.getElementById("sweep-slice-name").textContent = shortName(axis.name);
    if (sweepSliceEl.options.length !== axis.values.length || sweepSliceEl.dataset.sweep !== String(sweep.id)) {
      sweepSliceEl.replaceChildren(...axis.values.map((value, i) => new Option(formatValue(value), String(i))));
      sweepSliceEl.dataset.sweep = String(sweep.id);
    }
    sweepSliceEl.value = String(slice);
  }

  drawSweepChart(sweep, score, best);
  renderSweepTable(sweep);
}

document.getElementById("sweep-use-best").addEventListener("click", async () => {
  const sweep = sweepLab?.sweeps.find((s) => s.id === openSweepId);
  const score = SCORES.find((s) => s.key === sweepScoreEl.value);
  const best = sweep && score ? bestOf(score, sweepRuns(sweep)) : null;
  if (!best) return;
  try {
    await postJSON("/api/runs/use", { id: best.run.id });
    sweepStatus(`The draft has run #${best.run.id}'s parameters now.`);
    await pollLab();
  } catch (err) {
    sweepStatus(`Couldn't use them: ${err.message}`, true);
  }
});

/** Sets the editor up for a sweep around the best run: each parameter over
 *  half its range, centered on the best value - within its own range. */
document.getElementById("sweep-narrow").addEventListener("click", () => {
  const sweep = sweepLab?.sweeps.find((s) => s.id === openSweepId);
  const score = SCORES.find((s) => s.key === sweepScoreEl.value);
  const best = sweep && score ? bestOf(score, sweepRuns(sweep)) : null;
  if (!best) return;
  setAxisRows(
    sweep.axes.map((axis, i) => {
      const center = axis.values[best.cell[i]];
      const half = Math.abs(axis.to - axis.from) / 4;
      const parameter = parameterNamed(axis.name);
      const { min, max } = parameter ? rangeOf(parameter) : { min: -Infinity, max: Infinity };
      return {
        name: axis.name,
        from: Math.max(min, center - half),
        to: Math.min(max, center + half),
        count: axis.count,
      };
    }),
  );
  sweepStatus(`Set up around run #${best.run.id}: adjust it, then Sweep.`);
  sweepAxesEl.scrollIntoView({ block: "nearest" });
});

// ---------------------------------------------------------------------
// Table
// ---------------------------------------------------------------------

/** One row per combination, best first by the Rank by score; the best of
 *  every score column in green. */
function renderSweepTable(sweep) {
  const scores = availableScores(sweep);
  const rankScore = SCORES.find((s) => s.key === sweepScoreEl.value);
  const head = document.createElement("tr");
  head.append(
    th("#"),
    ...sweep.axes.map((axis) => th(shortName(axis.name), axis.name)),
    ...scores.map((score) => {
      const cell = th(`${score.label} ${score.unit}`, `Rank by ${score.label.toLowerCase()}`);
      cell.classList.add("sortable");
      cell.classList.toggle("sorted", score === rankScore);
      cell.addEventListener("click", () => {
        sweepScoreEl.value = score.key;
        renderSweeps(sweepLab);
      });
      return cell;
    }),
  );
  const bests = new Map(scores.map((score) => [score.key, bestOf(score, sweepRuns(sweep))?.value]));
  const rows = sweep.cells.map((cell) => ({ cell, run: cell.run === null ? null : sweepRunsById.get(cell.run) ?? null }));
  const rank = (row) => (rankScore ? scoreOf(rankScore, row.run) : null);
  rows.sort((a, b) => {
    const [va, vb] = [rank(a), rank(b)];
    if (va === null || vb === null) return (va === null) - (vb === null);
    return rankScore.lowerIsBetter ? va - vb : vb - va;
  });
  const body = rows.map(({ cell, run }) => {
    const tr = document.createElement("tr");
    tr.append(td(cell.run === null ? "–" : `${cell.run}`));
    for (const [i, axis] of sweep.axes.entries()) tr.append(td(formatValue(axis.values[cell.cell[i]])));
    if (run && run.state === "done") {
      for (const score of scores) {
        const value = score.value(run.stats);
        const cellEl = td(value === null ? "" : score.short(value));
        cellEl.classList.toggle("best", value !== null && value === bests.get(score.key));
        tr.append(cellEl);
      }
      tr.classList.add("clickable");
      tr.classList.toggle("selected", run.id === selectedRun);
      tr.addEventListener("click", () => {
        selectRun(run.id);
        renderSweeps(sweepLab);
      });
    } else {
      const state = cell.run === null
        ? `skipped: ${cell.skipped}`
        : !run
          ? "deleted"
          : run.state === "running"
            ? `running, ${percent(run.progress)}`
            : run.state === "failed"
              ? `failed: ${run.error}`
              : run.state;
      const stateEl = td(state);
      stateEl.colSpan = Math.max(1, scores.length);
      stateEl.className = "sweep-state";
      tr.append(stateEl);
    }
    return tr;
  });
  sweepTableEl.replaceChildren(head, ...body);
}

function th(text, title = "") {
  const el = document.createElement("th");
  el.textContent = text;
  if (title) el.title = title;
  return el;
}

function td(text) {
  const el = document.createElement("td");
  el.textContent = text;
  return el;
}

// ---------------------------------------------------------------------
// Chart
// ---------------------------------------------------------------------

/** Green for the best of `[lo, hi]`, through yellow, to red for the worst. */
function heatColor(value, lo, hi, lowerIsBetter) {
  let t = hi > lo ? (value - lo) / (hi - lo) : 0;
  if (!lowerIsBetter) t = 1 - t;
  return `hsl(${Math.round(130 * (1 - t))}, 65%, 42%)`;
}

function drawSweepChart(sweep, score, best) {
  const { ctx, width, height } = Chart.prepare(sweepChartEl, sweepChartWrapEl);
  chartHits = [];
  const plot = { left: 52, top: 22, right: width - 12, bottom: height - 40 };
  if (!score) {
    Chart.placeholder(ctx, plot, "No run of this sweep is done yet.");
    return;
  }
  if (sweep.axes.length === 1) drawLine(ctx, plot, sweep, score, best);
  else drawHeatmap(ctx, plot, sweep, score, best);
}

/** One parameter: the score over its values, odometry's trajectory error
 *  as a dashed line for reference. */
function drawLine(ctx, plot, sweep, score, best) {
  const axis = sweep.axes[0];
  const points = sweepRuns(sweep)
    .map(({ cell, run }) => ({ x: axis.values[cell[0]], y: scoreOf(score, run), run, cell }))
    .filter((p) => p.y !== null)
    .sort((a, b) => a.x - b.x);
  const odometry = score.key === "trajectory"
    ? points.map((p) => p.run.stats.scores.odometry?.rms_m).filter((v) => v !== undefined)
    : [];
  const reference = odometry.length ? odometry.reduce((a, b) => a + b, 0) / odometry.length : null;
  // Charted in the units the table shows.
  const shown = (value) => Number(score.short(value));
  const ys = points.map((p) => shown(p.y));
  if (reference !== null) ys.push(shown(reference));
  let [yMin, yMax] = [Math.min(...ys), Math.max(...ys)];
  const yPad = (yMax - yMin) * 0.1 || Math.abs(yMax) * 0.1 || 1;
  [yMin, yMax] = [yMin - yPad, yMax + yPad];
  let [xMin, xMax] = [Math.min(...axis.values), Math.max(...axis.values)];
  if (xMin === xMax) [xMin, xMax] = [xMin - 1, xMax + 1];
  const { toX, toY } = Chart.axes(ctx, plot, {
    xMin,
    xMax,
    xStep: Chart.niceStep(xMax - xMin, Math.max(2, Math.floor((plot.right - plot.left) / 70))),
    yMin,
    yMax,
    yStep: Chart.niceStep(yMax - yMin, 5),
    xLabel: shortName(axis.name),
    yLabel: `${score.label} (${score.unit})`,
  });
  if (reference !== null) {
    ctx.save();
    ctx.setLineDash([5, 4]);
    ctx.strokeStyle = "rgba(255, 255, 255, 0.5)";
    ctx.beginPath();
    ctx.moveTo(plot.left, toY(shown(reference)));
    ctx.lineTo(plot.right, toY(shown(reference)));
    ctx.stroke();
    ctx.restore();
    ctx.fillStyle = "rgba(255, 255, 255, 0.7)";
    ctx.textAlign = "right";
    ctx.textBaseline = "bottom";
    ctx.fillText("odometry alone", plot.right, toY(shown(reference)) - 3);
  }
  ctx.strokeStyle = "#5b8def";
  ctx.lineWidth = 2;
  ctx.beginPath();
  points.forEach((p, i) => (i === 0 ? ctx.moveTo(toX(p.x), toY(shown(p.y))) : ctx.lineTo(toX(p.x), toY(shown(p.y)))));
  ctx.stroke();
  for (const p of points) {
    const [x, y] = [toX(p.x), toY(shown(p.y))];
    ctx.fillStyle = best && p.run.id === best.run.id ? "#3bd16f" : "#5b8def";
    ctx.beginPath();
    ctx.arc(x, y, 4, 0, 2 * Math.PI);
    ctx.fill();
    if (p.run.id === selectedRun) {
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(x, y, 7, 0, 2 * Math.PI);
      ctx.stroke();
    }
    chartHits.push({ x: x - 8, y: y - 8, w: 16, h: 16, run: p.run, cell: p.cell });
  }
}

/** Two parameters: a cell per combination, colored from the best (green)
 *  to the worst (red) - of the slice the menu picks, with three. */
function drawHeatmap(ctx, plot, sweep, score, best) {
  const [rowAxis, colAxis] = sweep.axes;
  const slice = sweep.axes.length === 3 ? Math.min(sweepSlices.get(sweep.id) ?? 0, sweep.axes[2].values.length - 1) : null;
  const inSlice = (cell) => slice === null || cell.cell[2] === slice;
  const cells = sweep.cells.filter(inSlice);
  // Colored over every slice, so slices compare.
  const values = sweepRuns(sweep).map(({ run }) => scoreOf(score, run)).filter((v) => v !== null);
  const [lo, hi] = [Math.min(...values), Math.max(...values)];

  plot.left = 64;
  plot.bottom = plot.bottom - 4;
  const cols = colAxis.values.length;
  const rows = rowAxis.values.length;
  const w = (plot.right - plot.left) / cols;
  const h = (plot.bottom - plot.top) / rows;
  ctx.font = "11px system-ui, sans-serif";
  for (const cell of cells) {
    const [r, c] = cell.cell;
    // The first row value at the bottom, as on a plot.
    const x = plot.left + c * w;
    const y = plot.bottom - (r + 1) * h;
    const run = cell.run === null ? null : sweepRunsById.get(cell.run);
    const value = scoreOf(score, run);
    ctx.fillStyle = value === null ? (cell.run === null ? "#16191d" : "#2a3038") : heatColor(value, lo, hi, score.lowerIsBetter);
    ctx.fillRect(x + 1, y + 1, w - 2, h - 2);
    let label = value === null ? (cell.run === null ? "×" : run && run.state === "running" ? percent(run.progress) : "") : score.short(value);
    ctx.fillStyle = value === null ? "#9aa4b2" : "#ffffff";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    if (ctx.measureText(label).width > w - 4) label = "";
    ctx.fillText(label, x + w / 2, y + h / 2);
    if (run && run.id === selectedRun) {
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 2;
      ctx.strokeRect(x + 2, y + 2, w - 4, h - 4);
    } else if (best && run && run.id === best.run.id) {
      ctx.strokeStyle = "#3bd16f";
      ctx.lineWidth = 2;
      ctx.strokeRect(x + 2, y + 2, w - 4, h - 4);
    }
    chartHits.push({ x, y, w, h, run, cell: cell.cell, skipped: cell.skipped });
  }
  ctx.fillStyle = Chart.AXIS_COLOR;
  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  colAxis.values.forEach((value, c) => {
    const label = formatValue(value);
    if (ctx.measureText(label).width < w + 8 || c % 2 === 0) ctx.fillText(label, plot.left + (c + 0.5) * w, plot.bottom + 4);
  });
  ctx.textAlign = "right";
  ctx.fillText(shortName(colAxis.name), plot.right, plot.bottom + 20);
  ctx.textBaseline = "middle";
  rowAxis.values.forEach((value, r) => ctx.fillText(formatValue(value), plot.left - 6, plot.bottom - (r + 0.5) * h));
  ctx.textAlign = "left";
  ctx.textBaseline = "top";
  ctx.fillText(`${shortName(rowAxis.name)} ↑`, 4, 4);
}

function hitAt(event) {
  const rect = sweepChartEl.getBoundingClientRect();
  const [x, y] = [event.clientX - rect.left, event.clientY - rect.top];
  return { point: { x, y }, hit: chartHits.find((b) => x >= b.x && x < b.x + b.w && y >= b.y && y < b.y + b.h) ?? null };
}

sweepChartEl.addEventListener("mousemove", (event) => {
  const { point, hit } = hitAt(event);
  sweepChartEl.style.cursor = hit && hit.run && hit.run.state === "done" ? "pointer" : "";
  const sweep = sweepLab?.sweeps.find((s) => s.id === openSweepId);
  if (!hit || !sweep) {
    sweepTooltipEl.hidden = true;
    return;
  }
  const lines = sweep.axes.map((axis, i) => `${axis.name} = ${formatValue(axis.values[hit.cell[i]])}`);
  if (hit.skipped) lines.push(`Skipped: ${hit.skipped}`);
  else if (hit.run && hit.run.state === "done") {
    lines.unshift(`Run #${hit.run.id}`);
    for (const score of availableScores(sweep)) {
      const value = score.value(hit.run.stats);
      if (value !== null) lines.push(`${score.label}: ${score.short(value)} ${score.unit}`);
    }
  } else if (hit.run) lines.push(`Run #${hit.run.id}: ${hit.run.state}`);
  sweepTooltipEl.replaceChildren(...lines.map((line) => Object.assign(document.createElement("div"), { textContent: line })));
  Chart.placeTooltip(sweepTooltipEl, point, sweepChartWrapEl.clientWidth);
});

sweepChartEl.addEventListener("mouseleave", () => {
  sweepTooltipEl.hidden = true;
});

sweepChartEl.addEventListener("click", (event) => {
  const { hit } = hitAt(event);
  if (hit && hit.run && hit.run.state === "done") {
    selectRun(hit.run.id);
    renderSweeps(sweepLab);
  }
});

new ResizeObserver(() => {
  if (sweepLab) renderSweeps(sweepLab);
}).observe(sweepChartWrapEl);
