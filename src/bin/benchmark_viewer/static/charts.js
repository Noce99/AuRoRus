"use strict";

// ---------------------------------------------------------------------
// benchmark_viewer's frontend, part 3: the bottom charts - the replayed
// runs' lap `Replay.lap` over the distance driven along one reference line
// (the track's centerline, else the first run's race line), so runs that
// followed different race lines still share one axis. Speed, time lost to
// a reference run, distance from each run's own race line, steering, and
// - for runs localization ran in - how far off its estimate was.
// Hovering a chart marks where each run was at that distance on the map.
// ---------------------------------------------------------------------

const CHARTS = {
  speed: { label: "speed [m/s]", value: (profile) => profile.speed, signed: false },
  delta: { label: "Δ time [s] (+ behind the reference)", value: null, signed: true },
  lateral: { label: "lateral [m]", value: (profile) => profile.lateral, signed: true },
  steering: { label: "steering [°]", value: (profile) => profile.steering.map((rad) => (rad * 180) / Math.PI), signed: true },
};

/** The Localization tab's metrics: each the chart it draws, and the
 *  digits its values are shown with. Quality is in each algorithm's own
 *  terms: scan matching's response (0-1, higher is better), the particle
 *  filter's spread (meters, lower is better). */
const LOCALIZATION_METRICS = {
  position: { label: "position error [m]", value: (profile) => profile.posErr, signed: false, digits: 3 },
  heading: {
    label: "heading error [°]",
    value: (profile) => profile.headErr.map((rad) => (rad == null ? null : (rad * 180) / Math.PI)),
    signed: true,
    digits: 2,
  },
  ms: { label: "time per scan [ms]", value: (profile) => profile.locMs, signed: false, digits: 1 },
  quality: { label: "quality (response, or spread [m])", value: (profile) => profile.locQuality, signed: false, digits: 3 },
};
const CHART_MARGIN = { left: 58, right: 18, top: 14, bottom: 34 };

const chartWrapEl = document.getElementById("chart-wrap");
const chartEl = document.getElementById("chart");
const chartTooltipEl = document.getElementById("chart-tooltip");
const chartInfoEl = document.getElementById("chart-info");
const chartLegendEl = document.getElementById("chart-legend");
const referenceSelectEl = document.getElementById("reference-select");
const referenceRowEl = document.getElementById("reference-row");
const locMetricRowEl = document.getElementById("loc-metric-row");
const locMetricSelectEl = document.getElementById("loc-metric-select");

let chartKind = "speed";
/** The Localization tab's metric - a `LOCALIZATION_METRICS` key. */
let locMetric = "position";
/** The run Δ time is measured against: an id, null for the first one. */
let referenceId = null;
/** Per replayed run with lap `Replay.lap`: `{run, profile, series}`. */
let profiles = [];
let chartHover = null;

/** The line every run's distance is measured along. */
function referenceLineOf(runs) {
  const withCenterline = runs.find((run) => run.lines.centerline);
  if (withCenterline) return { line: TrackMath.referenceLine(withCenterline.lines.centerline), name: "centerline" };
  return runs.length ? { line: TrackMath.referenceLine(runs[0].lines.race_line), name: `${runs[0].label}'s race line` } : null;
}

function rebuildProfiles() {
  const runs = Replay.runs;
  const reference = referenceLineOf(runs);
  profiles = reference
    ? runs
        .map((run) => ({ run, profile: TrackMath.lapProfile(run.trajectory, Replay.lap, reference.line) }))
        .filter(({ profile }) => profile.d.length > 1)
    : [];
  if (!profiles.some(({ run }) => run.id === referenceId)) referenceId = profiles[0]?.run.id ?? null;
  const referenceProfile = profiles.find(({ run }) => run.id === referenceId)?.profile;
  for (const entry of profiles) {
    entry.series = {
      speed: CHARTS.speed.value(entry.profile),
      lateral: CHARTS.lateral.value(entry.profile),
      steering: CHARTS.steering.value(entry.profile),
      delta: referenceProfile ? TrackMath.deltaTime(entry.profile, referenceProfile) : [],
      // Only for runs localization ran in - the others aren't drawn.
      localization: TrackMath.hasEstimate(entry.run.trajectory)
        ? LOCALIZATION_METRICS[locMetric].value(entry.profile)
        : null,
    };
  }

  referenceSelectEl.replaceChildren(
    ...profiles.map(({ run }) => el("option", { value: run.id, text: `${run.label} ${run.summary.algorithm.name}` })),
  );
  if (referenceId) referenceSelectEl.value = referenceId;

  const selected = Viewer.selected().length;
  const elsewhere = selected - runs.length;
  const missing = runs.length - profiles.length;
  const parts = [];
  if (reference) parts.push(`Lap ${Replay.lap} · distance along the ${reference.name}`);
  if (missing > 0) parts.push(`${missing} run${missing === 1 ? "" : "s"} never drove lap ${Replay.lap}`);
  if (elsewhere > 0) parts.push(`${elsewhere} selected run${elsewhere === 1 ? "" : "s"} on another track not shown`);
  if (chartKind === "localization") {
    const without = profiles.filter(({ series }) => series.localization === null).map(({ run }) => run.label);
    if (without.length) parts.push(`no localization in ${without.join(", ")}`);
  }
  chartInfoEl.textContent = parts.join("  ·  ");
  legend(chartLegendEl, profiles.length >= 2 ? profiles.map(({ run }) => run) : []);
  drawChart();
}

function selectChart(kind) {
  chartKind = kind;
  for (const button of document.querySelectorAll("#chart-tabs .bottom-tab")) {
    button.classList.toggle("selected", button.dataset.chart === kind);
  }
  referenceRowEl.hidden = kind !== "delta";
  locMetricRowEl.hidden = kind !== "localization";
  // The info line names the runs without localization on that tab only.
  rebuildProfiles();
}

/** The chart spec of the tab shown. */
function chartSpec() {
  return chartKind === "localization" ? LOCALIZATION_METRICS[locMetric] : CHARTS[chartKind];
}

/** The profiles the tab shown draws - on the Localization tab, only the
 *  runs localization ran in. */
function chartedProfiles() {
  return profiles.filter(({ series }) => series[chartKind] != null);
}

/** Where each run is now, if on the lap shown: `run id -> distance`. */
function currentDistances() {
  const distances = new Map();
  for (const { run, profile } of profiles) {
    const start = run.starts[Replay.lap];
    if (start === undefined) continue;
    const t = Replay.runTime(run) - start;
    if (t < 0 || t > profile.t[profile.t.length - 1]) continue;
    distances.set(run.id, TrackMath.interpolate(profile.t, TrackMath.monotone(profile.d), t));
  }
  return distances;
}

function drawChart() {
  if (!chartEl.isConnected || chartWrapEl.offsetParent === null) return;
  const { ctx, width, height } = Chart.prepare(chartEl, chartWrapEl);
  chartTooltipEl.hidden = true;
  const plot = {
    left: CHART_MARGIN.left,
    top: CHART_MARGIN.top,
    right: width - CHART_MARGIN.right,
    bottom: height - CHART_MARGIN.bottom,
  };
  if (plot.right <= plot.left || plot.bottom <= plot.top) return;

  const spec = chartSpec();
  const charted = chartedProfiles();
  const values = charted.flatMap(({ series }) => series[chartKind].filter((value) => value != null));
  const xMax = Math.max(1, ...profiles.map(({ profile }) => profile.d[profile.d.length - 1]));
  let yMin = values.length ? Math.min(...values) : 0;
  let yMax = values.length ? Math.max(...values) : 1;
  if (spec.signed) {
    const extent = Math.max(Math.abs(yMin), Math.abs(yMax), 0.05);
    [yMin, yMax] = [-extent, extent];
  } else {
    yMin = Math.min(0, yMin);
  }
  const yStep = Chart.niceStep(yMax - yMin || 1, 6);
  yMin = Math.floor(yMin / yStep) * yStep;
  yMax = Math.ceil((yMax + 1e-9) / yStep) * yStep;
  const xStep = Chart.niceStep(xMax, Math.max(2, Math.floor((plot.right - plot.left) / 80)));
  const { toX, toY } = Chart.axes(ctx, plot, {
    xMax,
    xStep,
    yMin,
    yMax,
    yStep,
    xLabel: "distance [m]",
    yLabel: spec.label,
    signedY: spec.signed,
  });

  if (charted.length === 0) {
    const why =
      profiles.length > 0
        ? "Localization didn't run in these runs."
        : Replay.runs.length
          ? `No run drove lap ${Replay.lap}.`
          : "Select runs to chart them.";
    Chart.placeholder(ctx, plot, why);
    Replay.setHoverMarkers([]);
    return;
  }

  ctx.save();
  ctx.beginPath();
  ctx.rect(plot.left, plot.top, plot.right - plot.left, plot.bottom - plot.top);
  ctx.clip();
  ctx.lineWidth = 2;
  ctx.lineJoin = "round";
  for (const { run, profile, series } of charted) {
    ctx.strokeStyle = run.color;
    ctx.beginPath();
    let drawing = false;
    series[chartKind].forEach((value, i) => {
      if (value == null) {
        drawing = false;
        return;
      }
      const x = toX(profile.d[i]);
      const y = toY(value);
      if (drawing) ctx.lineTo(x, y);
      else ctx.moveTo(x, y);
      drawing = true;
    });
    ctx.stroke();
  }
  ctx.restore();

  // Where each run is at the playback time: a tick on the top edge.
  for (const [id, d] of currentDistances()) {
    const run = profiles.find((entry) => entry.run.id === id).run;
    const x = toX(d);
    ctx.fillStyle = run.color;
    ctx.strokeStyle = Chart.AXIS_COLOR;
    ctx.beginPath();
    ctx.moveTo(x - 5, plot.top - 8);
    ctx.lineTo(x + 5, plot.top - 8);
    ctx.lineTo(x, plot.top);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
  }

  // The hover: a crosshair at one distance, every run's value there in the
  // tooltip, and where every run was on the map.
  if (!chartHover || chartHover.x < plot.left || chartHover.x > plot.right) {
    Replay.setHoverMarkers([]);
    return;
  }
  const d = ((chartHover.x - plot.left) / (plot.right - plot.left)) * xMax;
  const x = Math.round(toX(d)) + 0.5;
  ctx.strokeStyle = "rgba(255, 255, 255, 0.45)";
  ctx.beginPath();
  ctx.moveTo(x, plot.top);
  ctx.lineTo(x, plot.bottom);
  ctx.stroke();

  const rows = [];
  const markers = [];
  for (const { run, profile, series } of profiles) {
    const px = TrackMath.atDistance(profile, "x", d);
    const py = TrackMath.atDistance(profile, "y", d);
    if (px !== null && py !== null) markers.push({ x: px, y: py, color: run.color });
    if (series[chartKind] == null) continue;
    const value = TrackMath.atDistance({ ...profile, value: series[chartKind] }, "value", d);
    if (value === null) continue;
    ctx.fillStyle = run.color;
    ctx.strokeStyle = Chart.AXIS_COLOR;
    ctx.beginPath();
    ctx.arc(x, toY(value), 4, 0, 2 * Math.PI);
    ctx.fill();
    ctx.stroke();
    const digits = spec.digits ?? (chartKind === "steering" ? 1 : chartKind === "speed" ? 2 : 3);
    const text = spec.signed ? Chart.formatSigned(value, digits) : value.toFixed(digits);
    rows.push(el("div", { className: "tooltip-row" }, [swatch(run.color), el("span", { text: `${run.label} ${text}` })]));
  }
  Replay.setHoverMarkers(markers);
  if (rows.length === 0) return;
  chartTooltipEl.replaceChildren(el("div", { className: "bottom-tooltip-title", text: `${d.toFixed(1)} m` }), ...rows);
  Chart.placeTooltip(chartTooltipEl, { x, y: chartHover.y }, width);
}

for (const button of document.querySelectorAll("#chart-tabs .bottom-tab")) {
  button.addEventListener("click", () => selectChart(button.dataset.chart));
}
locMetricSelectEl.addEventListener("change", () => {
  locMetric = locMetricSelectEl.value;
  rebuildProfiles();
});
referenceSelectEl.addEventListener("change", () => {
  referenceId = referenceSelectEl.value;
  rebuildProfiles();
});
chartEl.addEventListener("mousemove", (event) => {
  const rect = chartEl.getBoundingClientRect();
  chartHover = { x: event.clientX - rect.left, y: event.clientY - rect.top };
  drawChart();
});
chartEl.addEventListener("mouseleave", () => {
  chartHover = null;
  drawChart();
});
new ResizeObserver(drawChart).observe(chartWrapEl);

window.addEventListener("viewer:replay", rebuildProfiles);

// The position ticks follow the playback - at most once per frame.
let chartFrameRequested = false;
window.PlaybackClock.subscribe(() => {
  if (chartFrameRequested) return;
  chartFrameRequested = true;
  requestAnimationFrame(() => {
    chartFrameRequested = false;
    drawChart();
  });
});

selectChart(chartKind);
