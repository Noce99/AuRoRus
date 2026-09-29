"use strict";

// ---------------------------------------------------------------------
// benchmark_viewer's frontend, part 1: the runs - loading them from
// `/api/runs`, filtering, sorting and selecting them (the Runs panel),
// comparing the selected ones' lap times (Compare) and parameters
// (Parameters), and the Layers panel's switches. replay.js (the map and
// the playback) and charts.js (the bottom charts) load after this file
// and follow the selection through `Viewer`.
//
// `fetchJSON`, `MapView` come from /map_view.js, `Chart` from /chart.js.
// ---------------------------------------------------------------------

/** How many runs can be compared at once: one color each. Past about
 *  eight, colors stop being told apart on thin lines and on the map. */
const MAX_SELECTED_RUNS = 8;

/** One color per selected run, in the order runs get them - validated as
 *  a categorical palette against the panels' dark surface (#1b1f24). On
 *  the light track some sit under 3:1, so every ghost also carries its
 *  `#n` label. */
const RUN_COLORS = ["#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300", "#9085e9", "#e66767"];

const STATUS_LABELS = { completed: "Completed", timeout: "Timeout", aborted_by_user: "Aborted" };

/** What the other scripts share: the runs, the selection, and events. */
window.Viewer = {
  /** Every readable run, as `/api/runs` lists them: `{id, summary}`. */
  runs: [],
  /** Selected run ids -> their slot (0-based: color and `#n` label). Kept
   *  in insertion order: the first selected is the default reference. */
  selection: new Map(),
  /** What the map shows - see the Layers panel. */
  layers: {
    raster: true,
    centerline: true,
    raceLines: true,
    trails: "full",
    ghosts: true,
    labels: true,
  },

  run(id) {
    return this.runs.find((run) => run.id === id) ?? null;
  },

  /** The selected runs, in selection order, each `{id, summary, slot,
   *  color, label}`. */
  selected() {
    return [...this.selection].flatMap(([id, slot]) => {
      const run = this.run(id);
      return run ? [{ ...run, slot, color: RUN_COLORS[slot], label: `#${slot + 1}` }] : [];
    });
  },

  /** Tells replay.js and charts.js the selection (or the runs) changed. */
  notify() {
    window.dispatchEvent(new CustomEvent("viewer:selection"));
  },

  /** A run's trajectory and lines, fetched once. */
  _data: new Map(),
  data(id) {
    if (!this._data.has(id)) {
      const base = `/api/runs/${id}`;
      const promise = Promise.all([fetchJSON(`${base}/trajectory`), fetchJSON(`${base}/lines`)]).then(
        ([trajectory, lines]) => ({ trajectory, lines }),
      );
      // A failure isn't cached: the next ask tries again.
      promise.catch(() => this._data.delete(id));
      this._data.set(id, promise);
    }
    return this._data.get(id);
  },
};

/** The track a run was driven on: the exact map files, not just the name. */
function trackKey(summary) {
  return `${summary.map.info_sha256}:${summary.map.tiff_sha256}`;
}

function formatLapTime(seconds) {
  if (seconds == null) return "–";
  if (seconds < 60) return `${seconds.toFixed(2)} s`;
  const minutes = Math.floor(seconds / 60);
  return `${minutes}:${(seconds - 60 * minutes).toFixed(2).padStart(5, "0")}`;
}

function formatDate(startedAt) {
  const date = new Date(startedAt);
  if (Number.isNaN(date.getTime())) return startedAt;
  const pad = (n) => String(n).padStart(2, "0");
  return `${date.getFullYear()}-${pad(date.getMonth() + 1)}-${pad(date.getDate())} ${pad(date.getHours())}:${pad(date.getMinutes())}`;
}

function lineLabel(raceLine) {
  const stem = raceLine.file.replace(/\.csv$/, "");
  const method = raceLine.method.replace(/_/g, " ");
  return stem === method ? stem : `${stem} · ${method}`;
}

function el(tag, attrs = {}, children = []) {
  const element = document.createElement(tag);
  for (const [key, value] of Object.entries(attrs)) {
    if (key === "text") element.textContent = value;
    else if (key === "className") element.className = value;
    else element.setAttribute(key, value);
  }
  for (const child of children) element.append(child);
  return element;
}

function swatch(color) {
  const span = el("span", { className: "swatch" });
  span.style.background = color;
  return span;
}

// ---------------------------------------------------------------------
// Persistence - filters, sort and selection survive a reload. Browser
// storage may be unavailable (private windows): everything works without.
// ---------------------------------------------------------------------

const STORAGE_KEY = "benchmark_viewer.state";

function loadState() {
  try {
    return JSON.parse(localStorage.getItem(STORAGE_KEY)) ?? {};
  } catch {
    return {};
  }
}

function saveState() {
  try {
    localStorage.setItem(
      STORAGE_KEY,
      JSON.stringify({
        excluded: Object.fromEntries(Object.entries(excluded).map(([group, set]) => [group, [...set]])),
        dateFrom: dateFromEl.value,
        dateTo: dateToEl.value,
        rules: paramRules,
        sort: sortSelectEl.value,
        descending: sortDescending,
        selection: [...Viewer.selection],
      }),
    );
  } catch {
    // Not saved - fine.
  }
}

// ---------------------------------------------------------------------
// Filters
// ---------------------------------------------------------------------

/** Each filter group: how a run's value and its label are found. */
const FILTER_GROUPS = [
  {
    key: "track",
    label: "Map",
    value: (summary) => trackKey(summary),
    // Two versions of a map under one name are told apart.
    labelOf: (value, runs) => {
      const summary = runs.find((run) => trackKey(run.summary) === value).summary;
      const name = summary.map.name;
      const versions = [...new Set(runs.filter((run) => run.summary.map.name === name).map((run) => trackKey(run.summary)))];
      return versions.length > 1 ? `${name} (version ${versions.indexOf(value) + 1})` : name;
    },
  },
  { key: "algorithm", label: "Algorithm", value: (summary) => summary.algorithm.name },
  { key: "model", label: "Vehicle model", value: (summary) => summary.vehicle.model },
  {
    key: "line",
    label: "Race line",
    value: (summary) => (summary.race_line.used_by_algorithm ? summary.race_line.method : "none"),
    labelOf: (value) => (value === "none" ? "none (timed on the centerline)" : value.replace(/_/g, " ")),
  },
  {
    key: "status",
    label: "Status",
    value: (summary) => summary.status,
    labelOf: (value) => STATUS_LABELS[value] ?? value,
  },
  {
    key: "saved",
    label: "Parameters",
    value: (summary) => (summary.algorithm.saved_to_config && summary.vehicle.saved_to_config ? "saved" : "unsaved"),
    labelOf: (value) => (value === "saved" ? "as saved in the config files" : "tuned, not saved"),
  },
  {
    key: "code",
    label: "Code",
    value: (summary) => `${summary.code.commit?.slice(0, 7) ?? "unknown"}${summary.code.dirty ? " + changes" : ""}`,
  },
];

/** A track's name as the Map filter lists it - with its version when
 *  several tracks share the name. */
function trackLabel(track) {
  return FILTER_GROUPS[0].labelOf(track, Viewer.runs);
}

/** Per group, the values unticked - so a value never seen before shows up
 *  ticked. */
const excluded = Object.fromEntries(FILTER_GROUPS.map((group) => [group.key, new Set()]));

const filtersEl = document.getElementById("filters");
const dateFromEl = document.getElementById("date-from");
const dateToEl = document.getElementById("date-to");
const paramRuleListEl = document.getElementById("param-rule-list");
const sortSelectEl = document.getElementById("sort-select");
const sortDirBtn = document.getElementById("sort-dir-btn");
const runListEl = document.getElementById("run-list");
const shownCountEl = document.getElementById("shown-count");
const selectionCountEl = document.getElementById("selection-count");
const selectionWarningEl = document.getElementById("selection-warning");

/** `{name, op, value}` rules, all of which a run must meet. */
let paramRules = [];
let sortDescending = true;

const OPS = {
  "=": (a, b) => Math.abs(a - b) <= 1e-9 * Math.max(1, Math.abs(b)),
  "≠": (a, b) => Math.abs(a - b) > 1e-9 * Math.max(1, Math.abs(b)),
  "<": (a, b) => a < b,
  "≤": (a, b) => a <= b,
  ">": (a, b) => a > b,
  "≥": (a, b) => a >= b,
};

/** A run's parameter `name` - its algorithm's, its vehicle model's, or a
 *  limit - or undefined. */
function parameterOf(summary, name) {
  return summary.algorithm.parameters[name] ?? summary.vehicle.parameters[name] ?? summary.vehicle.limits[name];
}

/** Whether `summary` passes every filter. */
function passes(summary) {
  for (const group of FILTER_GROUPS) {
    if (excluded[group.key].has(group.value(summary))) return false;
  }
  const day = summary.started_at.slice(0, 10);
  if (dateFromEl.value && day < dateFromEl.value) return false;
  if (dateToEl.value && day > dateToEl.value) return false;
  for (const rule of paramRules) {
    if (!rule.name || rule.value === "" || Number.isNaN(Number(rule.value))) continue;
    const value = parameterOf(summary, rule.name);
    if (value === undefined || !OPS[rule.op](value, Number(rule.value))) return false;
  }
  return true;
}

function renderFilters() {
  const open = new Set([...filtersEl.querySelectorAll("details[open]")].map((details) => details.dataset.group));
  filtersEl.replaceChildren(
    ...FILTER_GROUPS.map((group) => {
      const counts = new Map();
      for (const run of Viewer.runs) {
        const value = group.value(run.summary);
        counts.set(value, (counts.get(value) ?? 0) + 1);
      }
      const values = [...counts.keys()].sort();
      const label = (value) => (group.labelOf ? group.labelOf(value, Viewer.runs) : value);
      const ticked = values.filter((value) => !excluded[group.key].has(value)).length;

      const all = el("button", { type: "button", text: "All", title: `Every ${group.label.toLowerCase()}` });
      const none = el("button", { type: "button", text: "None", title: `No ${group.label.toLowerCase()}` });
      all.addEventListener("click", (event) => {
        event.preventDefault();
        excluded[group.key].clear();
        filtersChanged();
      });
      none.addEventListener("click", (event) => {
        event.preventDefault();
        for (const value of values) excluded[group.key].add(value);
        filtersChanged();
      });
      const summaryEl = el("summary", {}, [
        el("span", { className: "group-label", text: group.label }),
        el("span", { className: "group-count", text: `(${ticked}/${values.length})` }),
        all,
        none,
      ]);
      const list = el(
        "ul",
        { className: "group-options" },
        values.map((value) => {
          const input = el("input", { type: "checkbox" });
          input.checked = !excluded[group.key].has(value);
          input.addEventListener("change", () => {
            if (input.checked) excluded[group.key].delete(value);
            else excluded[group.key].add(value);
            filtersChanged();
          });
          return el("li", {}, [
            el("label", {}, [input, el("span", { text: label(value) }), el("span", { className: "group-count", text: counts.get(value) })]),
          ]);
        }),
      );
      const details = el("details", { className: "filter-group" }, [summaryEl, list]);
      details.dataset.group = group.key;
      if (open.has(group.key)) details.open = true;
      return details;
    }),
  );
}

/** Every parameter name any run has, for the rule dropdowns. */
function parameterNames() {
  const names = new Set();
  for (const { summary } of Viewer.runs) {
    for (const table of [summary.algorithm.parameters, summary.vehicle.parameters, summary.vehicle.limits]) {
      for (const name of Object.keys(table)) names.add(name);
    }
  }
  return [...names].sort();
}

function renderParamRules() {
  const names = parameterNames();
  paramRuleListEl.replaceChildren(
    ...paramRules.map((rule, index) => {
      const name = el("select", { title: "Parameter" }, [
        el("option", { value: "", text: "parameter…" }),
        ...names.map((n) => el("option", { value: n, text: n })),
      ]);
      name.value = rule.name;
      const op = el("select", { title: "Comparison" }, Object.keys(OPS).map((o) => el("option", { value: o, text: o })));
      op.value = rule.op;
      const value = el("input", { type: "number", step: "any", placeholder: "value" });
      value.value = rule.value;
      const remove = el("button", { type: "button", text: "✕", title: "Remove this filter" });
      name.addEventListener("change", () => {
        rule.name = name.value;
        filtersChanged(false);
      });
      op.addEventListener("change", () => {
        rule.op = op.value;
        filtersChanged(false);
      });
      value.addEventListener("input", () => {
        rule.value = value.value;
        filtersChanged(false);
      });
      remove.addEventListener("click", () => {
        paramRules.splice(index, 1);
        renderParamRules();
        filtersChanged(false);
      });
      return el("div", { className: "param-rule" }, [name, op, value, remove]);
    }),
  );
}

document.getElementById("add-rule-btn").addEventListener("click", () => {
  paramRules.push({ name: "", op: "=", value: "" });
  renderParamRules();
});

function filtersChanged(rerenderFilters = true) {
  if (rerenderFilters) renderFilters();
  renderRunList();
  saveState();
}

for (const input of [dateFromEl, dateToEl]) input.addEventListener("change", () => filtersChanged(false));

// ---------------------------------------------------------------------
// The runs list
// ---------------------------------------------------------------------

const SORT_KEYS = {
  date: (summary) => summary.started_at,
  total: (summary) => summary.results?.total_time_s ?? null,
  best: (summary) => summary.results?.best_lap_s ?? null,
  mean: (summary) => summary.results?.mean_lap_s ?? null,
  map: (summary) => summary.map.name,
  algorithm: (summary) => summary.algorithm.name,
};

function sortedRuns(runs) {
  const key = SORT_KEYS[sortSelectEl.value];
  const sign = sortDescending ? -1 : 1;
  return [...runs].sort((a, b) => {
    const [x, y] = [key(a.summary), key(b.summary)];
    // Runs without a value (e.g. no total: not completed) always go last.
    if (x === null || y === null) return (x === null) - (y === null);
    return sign * (x < y ? -1 : x > y ? 1 : 0) || b.summary.started_at.localeCompare(a.summary.started_at);
  });
}

function showSelectionWarning(show) {
  selectionWarningEl.textContent = `At most ${MAX_SELECTED_RUNS} runs can be compared at once - untick one first.`;
  selectionWarningEl.hidden = !show;
}

/** Selects `id` in the first free slot, or unselects it. */
function setSelected(id, on) {
  if (!on) {
    Viewer.selection.delete(id);
    showSelectionWarning(false);
  } else if (!Viewer.selection.has(id)) {
    const used = new Set(Viewer.selection.values());
    const slot = [...Array(MAX_SELECTED_RUNS).keys()].find((s) => !used.has(s));
    if (slot === undefined) {
      showSelectionWarning(true);
      return false;
    }
    Viewer.selection.set(id, slot);
  }
  selectionChanged();
  return true;
}

function selectionChanged() {
  renderRunList();
  renderCompare();
  renderParameters();
  saveState();
  Viewer.notify();
}

function renderRunList() {
  const shown = sortedRuns(Viewer.runs.filter((run) => passes(run.summary)));
  shownCountEl.textContent = `${shown.length} of ${Viewer.runs.length} shown`;
  selectionCountEl.textContent = `${Viewer.selection.size}/${MAX_SELECTED_RUNS} selected`;
  document.getElementById("sidebar-counts").textContent = `${Viewer.runs.length} run${Viewer.runs.length === 1 ? "" : "s"}`;

  runListEl.replaceChildren(
    ...shown.map((run) => {
      const { summary } = run;
      const slot = Viewer.selection.get(run.id);
      const selected = slot !== undefined;
      const input = el("input", { type: "checkbox", title: "Compare this run" });
      input.checked = selected;
      input.addEventListener("change", () => {
        if (!setSelected(run.id, input.checked)) input.checked = false;
      });
      const mark = selected
        ? el("span", { className: "run-mark", text: `#${slot + 1}` })
        : el("span", { className: "run-mark empty" });
      if (selected) mark.style.background = RUN_COLORS[slot];

      const results = summary.results;
      const numbers = results
        ? [`best ${formatLapTime(results.best_lap_s)}`, results.total_time_s != null ? `total ${formatLapTime(results.total_time_s)}` : null]
        : [];
      const li = el("li", { className: selected ? "selected" : "" }, [
        el("label", { className: "run-check" }, [input, mark]),
        el("div", { className: "run-body" }, [
          el("div", { className: "run-title" }, [
            el("span", { text: `${summary.algorithm.name} · ${summary.vehicle.model}` }),
            el("span", { className: `run-status ${summary.status}`, text: STATUS_LABELS[summary.status] ?? summary.status }),
          ]),
          el("div", { className: "run-sub", text: `${summary.map.name} · ${lineLabel(summary.race_line)} · ${formatDate(summary.started_at)}` }),
          el("div", {
            className: "run-sub",
            text: [`${summary.laps.length}/${summary.laps_requested} laps`, ...numbers.filter(Boolean)]
              .concat(summary.algorithm.saved_to_config && summary.vehicle.saved_to_config ? [] : ["unsaved parameters"])
              .join(" · "),
          }),
        ]),
      ]);
      li.title = run.id;
      return li;
    }),
  );
  if (shown.length === 0) {
    runListEl.append(el("li", { className: "empty", text: Viewer.runs.length ? "No run matches the filters." : "No benchmarks yet." }));
  }
}

sortSelectEl.addEventListener("change", () => filtersChanged(false));
sortDirBtn.addEventListener("click", () => {
  sortDescending = !sortDescending;
  sortDirBtn.textContent = sortDescending ? "↓" : "↑";
  filtersChanged(false);
});
document.getElementById("clear-selection-btn").addEventListener("click", () => {
  Viewer.selection.clear();
  showSelectionWarning(false);
  selectionChanged();
});

// ---------------------------------------------------------------------
// Compare panel - every selected run's lap times, as a table and a chart
// ---------------------------------------------------------------------

const compareHintEl = document.getElementById("compare-hint");
const lapTableWrapEl = document.getElementById("lap-table-wrap");
const lapChartWrapEl = document.getElementById("lap-chart-wrap");
const lapChartEl = document.getElementById("lap-chart");
const lapChartTooltipEl = document.getElementById("lap-chart-tooltip");
const lapChartLegendEl = document.getElementById("lap-chart-legend");
let lapChartHover = null;

function legend(container, runs) {
  container.replaceChildren(
    ...runs.map((run) => el("span", { className: "legend-item" }, [swatch(run.color), el("span", { text: `${run.label} ${run.summary.algorithm.name} · ${run.summary.vehicle.model}` })])),
  );
}

function renderCompare() {
  const runs = Viewer.selected();
  compareHintEl.hidden = runs.length > 0;
  lapChartWrapEl.hidden = runs.length === 0;
  legend(lapChartLegendEl, runs.length >= 2 ? runs : []);
  if (runs.length === 0) {
    lapTableWrapEl.replaceChildren();
    return;
  }
  const tracks = new Set(runs.map((run) => trackKey(run.summary)));
  const laps = Math.max(...runs.map((run) => run.summary.laps.length), 0);
  // The fastest time of each lap, highlighted.
  const best = [...Array(laps).keys()].map((i) =>
    Math.min(...runs.map((run) => run.summary.laps[i]?.time_s ?? Infinity)),
  );
  const cell = (value, isBest) => el("td", { className: isBest ? "best" : "", text: formatLapTime(value) });
  const header = el("tr", {}, [
    el("th", { text: "Run" }),
    ...[...Array(laps).keys()].map((i) => el("th", { text: `L${i + 1}` })),
    el("th", { text: "Total" }),
    el("th", { text: "Mean ± std" }),
  ]);
  const rows = runs.map((run) => {
    const { summary } = run;
    const title = el("th", { className: "run-cell" }, [swatch(run.color), el("span", { text: run.label })]);
    title.title = `${summary.algorithm.name} · ${summary.vehicle.model} · ${summary.map.name} · ${lineLabel(summary.race_line)}`;
    if (tracks.size > 1) title.append(el("span", { className: "track-tag", text: trackLabel(trackKey(summary)) }));
    const results = summary.results;
    return el("tr", {}, [
      title,
      ...[...Array(laps).keys()].map((i) => {
        const lap = summary.laps[i];
        return lap ? cell(lap.time_s, runs.length > 1 && lap.time_s === best[i]) : el("td", { className: "empty", text: "–" });
      }),
      el("td", { text: formatLapTime(results?.total_time_s) }),
      el("td", { text: results ? `${results.mean_lap_s.toFixed(2)} ± ${results.std_lap_s.toFixed(2)}` : "–" }),
    ]);
  });
  lapTableWrapEl.replaceChildren(el("table", { className: "lap-table" }, [el("thead", {}, [header]), el("tbody", {}, rows)]));
  drawLapChart();
}

/** Lap number on x, lap time on y, one line per selected run. */
function drawLapChart() {
  if (lapChartWrapEl.hidden || lapChartWrapEl.offsetParent === null) return;
  const runs = Viewer.selected().filter((run) => run.summary.laps.length > 0);
  const { ctx, width, height } = Chart.prepare(lapChartEl, lapChartWrapEl);
  lapChartTooltipEl.hidden = true;
  const plot = { left: 52, right: width - 14, top: 18, bottom: height - 30 };
  if (plot.right <= plot.left || plot.bottom <= plot.top) return;
  if (runs.length === 0) {
    Chart.placeholder(ctx, plot, "No completed lap.");
    return;
  }
  const times = runs.flatMap((run) => run.summary.laps.map((lap) => lap.time_s));
  const laps = Math.max(...runs.map((run) => run.summary.laps.length));
  let [yMin, yMax] = [Math.min(...times), Math.max(...times)];
  const pad = Math.max(0.05, (yMax - yMin) * 0.1);
  yMin -= pad;
  yMax += pad;
  const yStep = Chart.niceStep(yMax - yMin, 5);
  yMin = Math.floor(yMin / yStep) * yStep;
  yMax = Math.ceil(yMax / yStep) * yStep;
  const { toX, toY } = Chart.axes(ctx, plot, {
    xMin: 0.5,
    xMax: laps + 0.5,
    xStep: Math.max(1, Chart.niceStep(laps, 8)),
    yMin,
    yMax,
    yStep,
    xLabel: "lap",
    yLabel: "lap time [s]",
  });

  ctx.lineWidth = 2;
  ctx.lineJoin = "round";
  for (const run of runs) {
    ctx.strokeStyle = run.color;
    ctx.fillStyle = run.color;
    ctx.beginPath();
    run.summary.laps.forEach((lap, i) => (i ? ctx.lineTo : ctx.moveTo).call(ctx, toX(i + 1), toY(lap.time_s)));
    ctx.stroke();
    for (const [i, lap] of run.summary.laps.entries()) {
      ctx.beginPath();
      ctx.arc(toX(i + 1), toY(lap.time_s), 3.5, 0, 2 * Math.PI);
      ctx.fill();
    }
  }

  // The tooltip: every run's time at the lap under the mouse.
  if (!lapChartHover) return;
  const lap = Math.round(((lapChartHover.x - plot.left) / (plot.right - plot.left)) * laps + 0.5);
  if (lap < 1 || lap > laps) return;
  const x = toX(lap);
  ctx.strokeStyle = "rgba(255, 255, 255, 0.4)";
  ctx.beginPath();
  ctx.moveTo(Math.round(x) + 0.5, plot.top);
  ctx.lineTo(Math.round(x) + 0.5, plot.bottom);
  ctx.stroke();
  lapChartTooltipEl.replaceChildren(
    el("div", { className: "bottom-tooltip-title", text: `Lap ${lap}` }),
    ...runs.map((run) =>
      el("div", { className: "tooltip-row" }, [swatch(run.color), el("span", { text: `${run.label} ${formatLapTime(run.summary.laps[lap - 1]?.time_s)}` })]),
    ),
  );
  Chart.placeTooltip(lapChartTooltipEl, { x, y: lapChartHover.y }, width);
}

lapChartEl.addEventListener("mousemove", (event) => {
  const rect = lapChartEl.getBoundingClientRect();
  lapChartHover = { x: event.clientX - rect.left, y: event.clientY - rect.top };
  drawLapChart();
});
lapChartEl.addEventListener("mouseleave", () => {
  lapChartHover = null;
  drawLapChart();
});
new ResizeObserver(drawLapChart).observe(lapChartWrapEl);

// ---------------------------------------------------------------------
// Parameters panel - the selected runs side by side
// ---------------------------------------------------------------------

const paramTableWrapEl = document.getElementById("param-table-wrap");
const showAllParamsEl = document.getElementById("show-all-params");

function renderParameters() {
  const runs = Viewer.selected();
  if (runs.length === 0) {
    paramTableWrapEl.replaceChildren(el("p", { className: "panel-hint", text: "Select runs in the Runs panel to compare their parameters." }));
    return;
  }
  const differs = (values) => {
    const present = values.filter((value) => value !== undefined);
    return present.length !== values.length || present.some((value) => value !== present[0]);
  };
  const header = el("tr", {}, [
    el("th", { text: "" }),
    ...runs.map((run) => {
      const th = el("th", {}, [swatch(run.color), el("span", { text: run.label })]);
      const s = run.summary;
      th.title = `${s.algorithm.name} · ${s.vehicle.model} · ${s.map.name} · ${lineLabel(s.race_line)}\ncommit ${s.code.commit?.slice(0, 7) ?? "unknown"}${s.code.dirty ? " + changes" : ""}`;
      return th;
    }),
  ]);
  const rows = [];
  const section = (title, values) => {
    const names = [...new Set(runs.flatMap((run) => Object.keys(values(run.summary))))].sort();
    const shown = names.filter((name) => showAllParamsEl.checked || differs(runs.map((run) => values(run.summary)[name])));
    rows.push(el("tr", { className: "section" }, [el("th", { colspan: runs.length + 1, text: `${title}${shown.length === 0 ? " - all the same" : ""}` })]));
    for (const name of shown) {
      const cells = runs.map((run) => values(run.summary)[name]);
      rows.push(
        el("tr", { className: differs(cells) ? "differs" : "" }, [
          el("th", { text: name }),
          ...cells.map((value) => el("td", { text: value === undefined ? "–" : String(Number(value.toPrecision(6))) })),
        ]),
      );
    }
  };
  const texts = (title, value) =>
    el("tr", { className: differs(runs.map((run) => value(run.summary))) ? "differs" : "" }, [
      el("th", { text: title }),
      ...runs.map((run) => el("td", { text: value(run.summary) })),
    ]);
  rows.push(el("tr", { className: "section" }, [el("th", { colspan: runs.length + 1, text: "Run" })]));
  rows.push(texts("algorithm", (s) => s.algorithm.name));
  rows.push(texts("vehicle model", (s) => s.vehicle.model));
  rows.push(texts("map", (s) => s.map.name));
  rows.push(texts("race line", (s) => lineLabel(s.race_line)));
  rows.push(texts("saved", (s) => (s.algorithm.saved_to_config && s.vehicle.saved_to_config ? "yes" : "no")));
  rows.push(texts("commit", (s) => `${s.code.commit?.slice(0, 7) ?? "?"}${s.code.dirty ? "+" : ""}`));
  section("Algorithm", (s) => s.algorithm.parameters);
  section("Vehicle model", (s) => s.vehicle.parameters);
  section("Actuator limits", (s) => s.vehicle.limits);
  paramTableWrapEl.replaceChildren(el("table", { className: "param-table" }, [el("thead", {}, [header]), el("tbody", {}, rows)]));
}

showAllParamsEl.addEventListener("change", renderParameters);

// ---------------------------------------------------------------------
// Layers panel
// ---------------------------------------------------------------------

const LAYER_TOGGLES = [
  ["raster", "Map"],
  ["centerline", "Centerline"],
  ["raceLines", "Race lines"],
  ["ghosts", "Vehicles"],
  ["labels", "Labels (#n)"],
];

function renderLayers() {
  const toggles = LAYER_TOGGLES.map(([key, label]) => {
    const input = el("input", { type: "checkbox" });
    input.checked = Viewer.layers[key];
    input.addEventListener("change", () => {
      Viewer.layers[key] = input.checked;
      MapView.requestRedraw();
    });
    return el("li", {}, [el("label", {}, [input, el("span", { text: label })])]);
  });
  const trails = el("select", { title: "How much of each run's path is drawn" }, [
    el("option", { value: "full", text: "Full path + last 3 s" }),
    el("option", { value: "recent", text: "Last 3 s" }),
    el("option", { value: "off", text: "None" }),
  ]);
  trails.value = Viewer.layers.trails;
  trails.addEventListener("change", () => {
    Viewer.layers.trails = trails.value;
    MapView.requestRedraw();
  });
  toggles.push(el("li", {}, [el("label", {}, [el("span", { text: "Trails" }), trails])]));
  document.getElementById("layer-toggles").replaceChildren(...toggles);
}

// ---------------------------------------------------------------------
// Loading
// ---------------------------------------------------------------------

async function loadRuns() {
  const response = await fetchJSON("/api/runs");
  Viewer.runs = response.runs;
  document.getElementById("sidebar-root").textContent = response.root;
  // Runs gone from the folder leave the selection.
  for (const id of [...Viewer.selection.keys()]) {
    if (!Viewer.run(id)) Viewer.selection.delete(id);
  }
  const unreadableEl = document.getElementById("unreadable");
  unreadableEl.hidden = response.unreadable.length === 0;
  unreadableEl.querySelector("summary").textContent = `${response.unreadable.length} unreadable run folder${response.unreadable.length === 1 ? "" : "s"}`;
  document.getElementById("unreadable-list").replaceChildren(
    ...response.unreadable.map((entry) => el("li", {}, [el("strong", { text: entry.id }), el("span", { text: entry.error })])),
  );
  renderFilters();
  renderParamRules();
  selectionChanged();
}

document.getElementById("refresh-btn").addEventListener("click", () => loadRuns().catch((err) => console.error(err)));

// Panels that draw on a canvas redraw once they're shown.
for (const btn of document.querySelectorAll(".panel-nav-btn")) {
  btn.addEventListener("click", () => requestAnimationFrame(drawLapChart));
}

(function restore() {
  const state = loadState();
  for (const group of FILTER_GROUPS) {
    for (const value of state.excluded?.[group.key] ?? []) excluded[group.key].add(value);
  }
  dateFromEl.value = state.dateFrom ?? "";
  dateToEl.value = state.dateTo ?? "";
  paramRules = Array.isArray(state.rules) ? state.rules : [];
  if (state.sort in SORT_KEYS) sortSelectEl.value = state.sort;
  sortDescending = state.descending ?? true;
  sortDirBtn.textContent = sortDescending ? "↓" : "↑";
  for (const [id, slot] of state.selection ?? []) {
    if (Number.isInteger(slot) && slot >= 0 && slot < MAX_SELECTED_RUNS) Viewer.selection.set(id, slot);
  }
})();

renderLayers();
loadRuns().catch((err) => console.error(err));
