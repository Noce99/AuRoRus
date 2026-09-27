"use strict";

// ---------------------------------------------------------------------
// The bottom panel, shared by web_gui and replay_web_gui: the ego
// vehicle's lap telemetry (the `lap_telemetry` topic, see
// `aurorus::topics::LapTelemetry`) - its lateral and speed errors along
// the race line, for the lap in progress and the one before, and every
// completed lap's time.
//
// Each UI's page holds an empty `#bottom-panel` (under the map, collapsed)
// and its `#bottom-panel-toggle-btn`; this fills the panel in, and only
// polls while it's open. Needs `startPolling` from /map_view.js.
// ---------------------------------------------------------------------

const LapPanel = (() => {
  const LATERAL_COLOR = [77, 159, 255];
  const SPEED_COLOR = [255, 92, 92];
  /** Opacity of the previous lap's line, under the current one's. */
  const PREVIOUS_ALPHA = 0.35;
  const AXIS_COLOR = "#ffffff";
  const GRID_COLOR = "rgba(255, 255, 255, 0.12)";
  /** Chart margins around the plot area, in CSS pixels. */
  const MARGIN = { left: 58, right: 18, top: 14, bottom: 34 };
  /** Hovering farther than this from a lap's point, in CSS pixels, doesn't
   *  pick it for the tooltip. */
  const HOVER_RADIUS_PX = 40;

  const CHARTS = {
    lateral: {
      series: "lateral_m",
      color: LATERAL_COLOR,
      unit: "m",
      label: "d",
      minRange: 0.1,
      describe: (value) => (value > 0 ? " (left)" : value < 0 ? " (right)" : ""),
    },
    speed: {
      series: "speed_error_mps",
      color: SPEED_COLOR,
      unit: "m/s",
      label: "Δv",
      minRange: 0.2,
      describe: (value) => (value > 0 ? " (over)" : value < 0 ? " (under)" : ""),
    },
  };

  let telemetry = null;
  let tab = "lateral";
  /** The mouse over the chart, in CSS pixels relative to it - or null. */
  let hover = null;
  let lapsKey = null;

  let panel, chartWrap, chart, tooltip, lapsWrap, lapsBody, info;

  function el(tag, attrs = {}, children = []) {
    const element = document.createElement(tag);
    for (const [key, value] of Object.entries(attrs)) {
      if (key === "text") element.textContent = value;
      else element.setAttribute(key, value);
    }
    for (const child of children) element.appendChild(child);
    return element;
  }

  function build() {
    const tabs = el("nav", { class: "bottom-tabs" });
    for (const [name, label] of [
      ["lateral", "Lateral Error"],
      ["speed", "Speed Error"],
      ["laps", "Lap Time History"],
    ]) {
      const button = el("button", { class: "bottom-tab", "data-tab": name, text: label });
      button.addEventListener("click", () => selectTab(name));
      tabs.appendChild(button);
    }
    info = el("span", { class: "bottom-info" });
    tabs.appendChild(info);

    chart = el("canvas", { class: "bottom-chart" });
    tooltip = el("div", { class: "bottom-tooltip", hidden: "" });
    chartWrap = el("div", { class: "bottom-chart-wrap" }, [chart, tooltip]);

    lapsBody = el("tbody");
    lapsWrap = el("div", { class: "bottom-laps", hidden: "" }, [
      el("table", {}, [
        el("thead", {}, [
          el("tr", {}, [
            el("th", { text: "Lap" }),
            el("th", { text: "Time" }),
            el("th", { text: "Distance" }),
            el("th", { text: "Avg speed" }),
          ]),
        ]),
        lapsBody,
      ]),
    ]);

    panel.append(tabs, el("div", { class: "bottom-body" }, [chartWrap, lapsWrap]));

    chart.addEventListener("mousemove", (event) => {
      const rect = chart.getBoundingClientRect();
      hover = { x: event.clientX - rect.left, y: event.clientY - rect.top };
      drawChart();
    });
    chart.addEventListener("mouseleave", () => {
      hover = null;
      drawChart();
    });
    new ResizeObserver(drawChart).observe(chartWrap);
    selectTab(tab);
  }

  function selectTab(name) {
    tab = name;
    for (const button of panel.querySelectorAll(".bottom-tab")) {
      button.classList.toggle("selected", button.dataset.tab === name);
    }
    chartWrap.hidden = name === "laps";
    lapsWrap.hidden = name !== "laps";
    render();
  }

  function isOpen() {
    return !panel.classList.contains("collapsed");
  }

  // -------------------------------------------------------------------
  // Formatting
  // -------------------------------------------------------------------

  function formatLapTime(seconds) {
    if (seconds < 60) return `${seconds.toFixed(3)} s`;
    const minutes = Math.floor(seconds / 60);
    return `${minutes}:${(seconds - 60 * minutes).toFixed(3).padStart(6, "0")}`;
  }

  function formatSigned(value, digits) {
    const text = value.toFixed(digits);
    return value > 0 ? `+${text}` : text;
  }

  /** A tick step giving roughly `count` ticks over `range`: 1, 2 or 5 times
   *  a power of ten. */
  function niceStep(range, count) {
    const raw = range / Math.max(1, count);
    const power = 10 ** Math.floor(Math.log10(raw));
    for (const factor of [1, 2, 5]) {
      if (factor * power >= raw) return factor * power;
    }
    return 10 * power;
  }

  function decimalsFor(step) {
    return Math.max(0, -Math.floor(Math.log10(step) + 1e-9));
  }

  // -------------------------------------------------------------------
  // Header
  // -------------------------------------------------------------------

  function renderInfo() {
    if (!telemetry) {
      info.textContent = "";
      return;
    }
    const parts = [];
    if (telemetry.status) parts.push(telemetry.status);
    if (telemetry.race_line) parts.push(`Line ${telemetry.race_line} · ${telemetry.lap_length_m.toFixed(1)} m`);
    if (tab !== "laps" && telemetry.current) {
      const lap = telemetry.current.number;
      parts.push(lap > 0 ? `Lap ${lap}` : "Out lap");
      if (telemetry.previous) parts.push(`faded: lap ${telemetry.previous.number}`);
    }
    if (telemetry.current_lap_time_s != null) parts.push(formatLapTime(telemetry.current_lap_time_s));
    info.textContent = parts.join("  ·  ");
  }

  // -------------------------------------------------------------------
  // Chart
  // -------------------------------------------------------------------

  function drawChart() {
    if (!chart || chartWrap.hidden) return;
    const dpr = window.devicePixelRatio || 1;
    const rect = chartWrap.getBoundingClientRect();
    const widthPx = Math.max(1, Math.round(rect.width * dpr));
    const heightPx = Math.max(1, Math.round(rect.height * dpr));
    if (chart.width !== widthPx || chart.height !== heightPx) {
      chart.width = widthPx;
      chart.height = heightPx;
    }
    const ctx = chart.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, rect.width, rect.height);
    tooltip.hidden = true;

    const spec = CHARTS[tab];
    const lapM = telemetry ? telemetry.lap_length_m : 0;
    const plot = {
      left: MARGIN.left,
      top: MARGIN.top,
      right: rect.width - MARGIN.right,
      bottom: rect.height - MARGIN.bottom,
    };
    if (plot.right <= plot.left || plot.bottom <= plot.top) return;

    const laps = [];
    if (telemetry && lapM > 0) {
      if (telemetry.previous) laps.push({ trace: telemetry.previous, alpha: PREVIOUS_ALPHA });
      laps.push({ trace: telemetry.current, alpha: 1 });
    }

    let maxAbs = spec.minRange;
    for (const { trace } of laps) {
      for (const value of trace[spec.series]) {
        if (value != null) maxAbs = Math.max(maxAbs, Math.abs(value));
      }
    }
    const yStep = niceStep(2 * maxAbs, 6);
    const yMax = Math.ceil((maxAbs * 1.05) / yStep) * yStep;
    const xMax = lapM > 0 ? lapM : 100;
    const xStep = niceStep(xMax, Math.max(2, Math.floor((plot.right - plot.left) / 80)));

    const toX = (s) => plot.left + (s / xMax) * (plot.right - plot.left);
    const toY = (v) => plot.top + ((yMax - v) / (2 * yMax)) * (plot.bottom - plot.top);

    // Grid, axes, and their numbers.
    ctx.font = "11px system-ui, sans-serif";
    ctx.fillStyle = AXIS_COLOR;
    ctx.lineWidth = 1;
    const yDecimals = decimalsFor(yStep);
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    for (let v = -yMax; v <= yMax + yStep / 2; v += yStep) {
      const y = Math.round(toY(v)) + 0.5;
      ctx.strokeStyle = Math.abs(v) < yStep / 2 ? "rgba(255, 255, 255, 0.55)" : GRID_COLOR;
      ctx.beginPath();
      ctx.moveTo(plot.left, y);
      ctx.lineTo(plot.right, y);
      ctx.stroke();
      ctx.fillText(formatSigned(Math.abs(v) < yStep / 2 ? 0 : v, yDecimals), plot.left - 6, y);
    }
    const xDecimals = decimalsFor(xStep);
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    for (let s = 0; s <= xMax + 1e-9; s += xStep) {
      const x = Math.round(toX(s)) + 0.5;
      ctx.strokeStyle = GRID_COLOR;
      ctx.beginPath();
      ctx.moveTo(x, plot.top);
      ctx.lineTo(x, plot.bottom);
      ctx.stroke();
      ctx.fillText(s.toFixed(xDecimals), x, plot.bottom + 5);
    }
    ctx.strokeStyle = AXIS_COLOR;
    ctx.beginPath();
    ctx.moveTo(plot.left + 0.5, plot.top);
    ctx.lineTo(plot.left + 0.5, plot.bottom + 0.5);
    ctx.lineTo(plot.right, plot.bottom + 0.5);
    ctx.stroke();
    ctx.textAlign = "right";
    ctx.fillText("s [m]", plot.right, plot.bottom + 19);
    ctx.textAlign = "left";
    ctx.textBaseline = "top";
    ctx.fillText(`${spec.label} [${spec.unit}]`, 6, 2);

    if (laps.length === 0) {
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillStyle = "rgba(255, 255, 255, 0.6)";
      ctx.fillText(
        (telemetry && telemetry.status) || "No lap telemetry yet.",
        (plot.left + plot.right) / 2,
        (plot.top + plot.bottom) / 2,
      );
      return;
    }

    // Where the vehicle is now.
    if (telemetry.s_m != null) {
      const x = Math.round(toX(telemetry.s_m)) + 0.5;
      ctx.strokeStyle = "rgba(255, 255, 255, 0.4)";
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(x, plot.top);
      ctx.lineTo(x, plot.bottom);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // The laps: the previous one faded, the current one on top of it -
    // each broken wherever the vehicle hasn't been.
    const binM = (trace) => lapM / trace[spec.series].length;
    const [r, g, b] = spec.color;
    ctx.save();
    ctx.beginPath();
    ctx.rect(plot.left, plot.top, plot.right - plot.left, plot.bottom - plot.top);
    ctx.clip();
    ctx.lineWidth = 1.75;
    ctx.lineJoin = "round";
    for (const { trace, alpha } of laps) {
      const values = trace[spec.series];
      const width = binM(trace);
      ctx.strokeStyle = `rgba(${r}, ${g}, ${b}, ${alpha})`;
      ctx.beginPath();
      let drawing = false;
      values.forEach((value, i) => {
        if (value == null) {
          drawing = false;
          return;
        }
        const x = toX((i + 0.5) * width);
        const y = toY(value);
        if (drawing) ctx.lineTo(x, y);
        else ctx.moveTo(x, y);
        drawing = true;
      });
      ctx.stroke();
    }
    ctx.restore();

    // The tooltip: at the lap point nearest the mouse, in its bin.
    if (!hover || hover.x < plot.left || hover.x > plot.right) return;
    const s = ((hover.x - plot.left) / (plot.right - plot.left)) * xMax;
    let best = null;
    for (const { trace, alpha } of laps) {
      const values = trace[spec.series];
      const bin = Math.min(values.length - 1, Math.max(0, Math.floor(s / binM(trace))));
      const value = values[bin];
      if (value == null) continue;
      const point = { x: toX((bin + 0.5) * binM(trace)), y: toY(value) };
      const distance = Math.abs(point.y - hover.y);
      if (distance <= HOVER_RADIUS_PX && (!best || distance < best.distance)) {
        best = { trace, alpha, value, point, distance, s: (bin + 0.5) * binM(trace) };
      }
    }
    if (!best) return;

    ctx.fillStyle = `rgba(${r}, ${g}, ${b}, ${best.alpha})`;
    ctx.strokeStyle = AXIS_COLOR;
    ctx.beginPath();
    ctx.arc(best.point.x, best.point.y, 4, 0, 2 * Math.PI);
    ctx.fill();
    ctx.stroke();

    const lap = best.trace.number > 0 ? `Lap ${best.trace.number}` : "Out lap";
    const percent = (100 * best.s) / lapM;
    tooltip.innerHTML = "";
    tooltip.append(
      el("div", { class: "bottom-tooltip-title", text: lap }),
      el("div", { text: `s = ${best.s.toFixed(2)} m (${percent.toFixed(1)} %)` }),
      el("div", {
        text: `${spec.label} = ${formatSigned(best.value, 3)} ${spec.unit}${spec.describe(best.value)}`,
      }),
    );
    tooltip.hidden = false;
    // Beside the point, flipped to its other side near the right/bottom edge.
    const gap = 12;
    const tipWidth = tooltip.offsetWidth;
    const tipHeight = tooltip.offsetHeight;
    let left = best.point.x + gap;
    if (left + tipWidth > rect.width) left = best.point.x - gap - tipWidth;
    let top = best.point.y - tipHeight - gap;
    if (top < 0) top = best.point.y + gap;
    tooltip.style.left = `${Math.max(0, left)}px`;
    tooltip.style.top = `${Math.max(0, top)}px`;
  }

  // -------------------------------------------------------------------
  // Lap time history
  // -------------------------------------------------------------------

  function renderLaps() {
    const laps = (telemetry && telemetry.laps) || [];
    const key = `${telemetry && telemetry.race_line}|${laps.length}`;
    if (key === lapsKey) return;
    lapsKey = key;

    lapsBody.innerHTML = "";
    if (laps.length === 0) {
      lapsBody.appendChild(
        el("tr", {}, [el("td", { class: "empty", colspan: "4", text: "No completed laps yet." })]),
      );
      return;
    }
    const times = laps.map((lap) => lap.time_s);
    const bestTime = Math.min(...times);
    const worstTime = Math.max(...times);
    for (const lap of [...laps].reverse()) {
      const row = el("tr", {}, [
        el("td", { text: String(lap.number) }),
        el("td", { text: formatLapTime(lap.time_s) }),
        el("td", { text: `${lap.distance_m.toFixed(2)} m` }),
        el("td", { text: `${lap.average_speed_mps.toFixed(2)} m/s` }),
      ]);
      if (lap.time_s === bestTime) row.classList.add("best");
      else if (lap.time_s === worstTime) row.classList.add("worst");
      lapsBody.appendChild(row);
    }
  }

  function render() {
    if (!panel) return;
    renderInfo();
    if (tab === "laps") renderLaps();
    else drawChart();
  }

  // -------------------------------------------------------------------
  // Setup
  // -------------------------------------------------------------------

  /** Fills `#bottom-panel` in and wires its toggle. `fetchTelemetry()`
   *  resolves to `{ value: LapTelemetry, ... }`; it's polled at `rateHz`
   *  while the panel is open - skipped while `pollKey()`, if given, returns
   *  what it did for the last fetch (e.g. a paused replay's time). */
  function init({ fetchTelemetry, rateHz = 10, pollKey = null }) {
    panel = document.getElementById("bottom-panel");
    const toggle = document.getElementById("bottom-panel-toggle-btn");
    if (!panel || !toggle) return;
    build();

    let lastKey;
    let fetched = false;
    const poll = async () => {
      if (!isOpen()) return;
      const key = pollKey ? pollKey() : undefined;
      if (fetched && pollKey && key === lastKey) return;
      const body = await fetchTelemetry();
      lastKey = key;
      fetched = true;
      telemetry = body.value;
      render();
    };
    startPolling(poll, 1000 / rateHz);

    toggle.addEventListener("click", () => {
      panel.classList.toggle("collapsed");
      if (isOpen()) render();
    });
  }

  return { init };
})();
