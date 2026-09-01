"use strict";

// A small, stable palette - PALETTE_SIZE in session.rs must stay in sync with
// this array's length; the two sides only need to agree on the modulus.
const PALETTE = [
  "#e6b800", "#3ea6ff", "#ff5c8a", "#3ddc84", "#c792ea",
  "#ff9d5c", "#5cd6d6", "#ff6b6b", "#8ab4f8", "#f4a261",
  "#a8e063", "#e879f9",
];

const RULER_HEIGHT = 24; // css px
const ROW_HEIGHT = 36; // css px
const TICK_HALF_HEIGHT = 10; // css px
const HOVER_TOLERANCE_PX = 5; // css px, in both axes

const MIN_PX_PER_S = 4;
const MAX_PX_PER_S = 4000;
const ZOOM_SLIDER_STEPS = 1000;

/** @type {{name:string, topics:{name:string, colorIndex:number, timestamps:number[]}[]}[]} */
let rows = [];

let pxPerUs = 100 / 1e6; // 100 px/s by default, until the session's duration sets a sensible initial fit.

const wrap = document.getElementById("timeline-canvas-wrap");
const timelineCanvas = document.getElementById("timeline-canvas");
const timelineCtx = timelineCanvas.getContext("2d");
const tooltip = document.getElementById("timeline-tooltip");

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function niceStepSeconds(pxPerS) {
  // Pick a gridline spacing (in seconds) so gridlines land roughly every
  // 60-140 px on screen, from a fixed set of "nice" round steps.
  const steps = [0.1, 0.2, 0.5, 1, 2, 5, 10, 30, 60, 120, 300, 600, 1800, 3600];
  for (const step of steps) {
    if (step * pxPerS >= 70) return step;
  }
  return steps[steps.length - 1];
}

function durationUs() {
  return window.PlaybackClock.durationUs;
}

function contentWidthCss() {
  return Math.max(wrap.clientWidth, durationUs() * pxPerUs + 40);
}

function contentHeightCss() {
  return RULER_HEIGHT + Math.max(1, rows.length) * ROW_HEIGHT;
}

function resizeCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const cssWidth = contentWidthCss();
  const cssHeight = contentHeightCss();
  timelineCanvas.style.width = `${cssWidth}px`;
  timelineCanvas.style.height = `${cssHeight}px`;
  const width = Math.max(1, Math.round(cssWidth * dpr));
  const height = Math.max(1, Math.round(cssHeight * dpr));
  if (timelineCanvas.width !== width || timelineCanvas.height !== height) {
    timelineCanvas.width = width;
    timelineCanvas.height = height;
  }
}

function render() {
  resizeCanvas();
  const dpr = window.devicePixelRatio || 1;
  timelineCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const cssWidth = contentWidthCss();
  const cssHeight = contentHeightCss();

  timelineCtx.fillStyle = "#15191e";
  timelineCtx.fillRect(0, 0, cssWidth, cssHeight);

  // Row backgrounds (alternating) + labels.
  timelineCtx.font = "12px system-ui, sans-serif";
  timelineCtx.textBaseline = "middle";
  for (let i = 0; i < rows.length; i++) {
    const y = RULER_HEIGHT + i * ROW_HEIGHT;
    timelineCtx.fillStyle = i % 2 === 0 ? "#181c22" : "#15191e";
    timelineCtx.fillRect(0, y, cssWidth, ROW_HEIGHT);
    timelineCtx.fillStyle = "#9aa4b2";
    timelineCtx.fillText(rows[i].name, 8, y + ROW_HEIGHT / 2);
  }

  // Gridlines + second labels.
  const stepS = niceStepSeconds(pxPerUs * 1e6);
  const stepUs = stepS * 1e6;
  timelineCtx.strokeStyle = "#2a3038";
  timelineCtx.fillStyle = "#6b7684";
  timelineCtx.lineWidth = 1;
  for (let t = 0; t <= durationUs() + stepUs; t += stepUs) {
    const x = t * pxPerUs;
    timelineCtx.beginPath();
    timelineCtx.moveTo(x + 0.5, RULER_HEIGHT);
    timelineCtx.lineTo(x + 0.5, cssHeight);
    timelineCtx.stroke();
    timelineCtx.fillText(`${(t / 1e6).toFixed(stepS < 1 ? 1 : 0)} s`, x + 4, RULER_HEIGHT / 2);
  }

  // Ticks.
  for (let i = 0; i < rows.length; i++) {
    const rowMidY = RULER_HEIGHT + i * ROW_HEIGHT + ROW_HEIGHT / 2;
    for (const topic of rows[i].topics) {
      timelineCtx.strokeStyle = PALETTE[topic.colorIndex % PALETTE.length];
      timelineCtx.lineWidth = 2;
      timelineCtx.beginPath();
      for (const t of topic.timestamps) {
        const x = t * pxPerUs;
        timelineCtx.moveTo(x, rowMidY - TICK_HALF_HEIGHT);
        timelineCtx.lineTo(x, rowMidY + TICK_HALF_HEIGHT);
      }
      timelineCtx.stroke();
    }
  }

  // Playhead.
  const playheadX = window.PlaybackClock.currentTimeUs * pxPerUs;
  timelineCtx.strokeStyle = "#c77dff";
  timelineCtx.lineWidth = 2;
  timelineCtx.beginPath();
  timelineCtx.moveTo(playheadX, RULER_HEIGHT);
  timelineCtx.lineTo(playheadX, cssHeight);
  timelineCtx.stroke();
  timelineCtx.fillStyle = "#c77dff";
  timelineCtx.beginPath();
  timelineCtx.moveTo(playheadX - 5, RULER_HEIGHT);
  timelineCtx.lineTo(playheadX + 5, RULER_HEIGHT);
  timelineCtx.lineTo(playheadX, RULER_HEIGHT + 8);
  timelineCtx.closePath();
  timelineCtx.fill();
}

// ---------------------------------------------------------------------
// Zoom (mouse wheel + slider), keeping the time under the cursor fixed.
// ---------------------------------------------------------------------

function sliderFromPxPerS(pxPerS) {
  const logMin = Math.log(MIN_PX_PER_S);
  const logMax = Math.log(MAX_PX_PER_S);
  const t = (Math.log(pxPerS) - logMin) / (logMax - logMin);
  return Math.round(clamp(t, 0, 1) * ZOOM_SLIDER_STEPS);
}

function pxPerSFromSlider(value) {
  const logMin = Math.log(MIN_PX_PER_S);
  const logMax = Math.log(MAX_PX_PER_S);
  const t = value / ZOOM_SLIDER_STEPS;
  return Math.exp(logMin + t * (logMax - logMin));
}

const timelineZoomSlider = document.getElementById("timeline-zoom-slider");
let syncingZoomSlider = false;

function syncZoomSlider() {
  syncingZoomSlider = true;
  timelineZoomSlider.value = String(sliderFromPxPerS(pxPerUs * 1e6));
  syncingZoomSlider = false;
}

function setPxPerUs(next, anchorClientX) {
  const rect = timelineCanvas.getBoundingClientRect();
  const cursorContentX = anchorClientX === undefined ? wrap.scrollLeft : wrap.scrollLeft + (anchorClientX - rect.left);
  const timeAtCursorUs = cursorContentX / pxPerUs;
  pxPerUs = clamp(next, MIN_PX_PER_S / 1e6, MAX_PX_PER_S / 1e6);
  render();
  if (anchorClientX !== undefined) {
    const newRect = timelineCanvas.getBoundingClientRect();
    wrap.scrollLeft = timeAtCursorUs * pxPerUs - (anchorClientX - newRect.left);
  }
  syncZoomSlider();
}

wrap.addEventListener(
  "wheel",
  (event) => {
    event.preventDefault();
    const factor = Math.exp(-event.deltaY * 0.0015);
    setPxPerUs(pxPerUs * factor, event.clientX);
  },
  { passive: false }
);

timelineZoomSlider.addEventListener("input", () => {
  if (syncingZoomSlider) return;
  setPxPerUs(pxPerSFromSlider(Number(timelineZoomSlider.value)) / 1e6);
});

// ---------------------------------------------------------------------
// Hover tooltip (topic name only) + right-click seek
// ---------------------------------------------------------------------

function eventToContentPoint(event) {
  const rect = timelineCanvas.getBoundingClientRect();
  return { x: event.clientX - rect.left, y: event.clientY - rect.top };
}

timelineCanvas.addEventListener("mousemove", (event) => {
  const { x, y } = eventToContentPoint(event);
  if (y < RULER_HEIGHT) {
    tooltip.hidden = true;
    return;
  }
  const rowIndex = Math.floor((y - RULER_HEIGHT) / ROW_HEIGHT);
  const row = rows[rowIndex];
  if (!row) {
    tooltip.hidden = true;
    return;
  }

  let hit = null;
  for (const topic of row.topics) {
    for (const t of topic.timestamps) {
      if (Math.abs(t * pxPerUs - x) <= HOVER_TOLERANCE_PX) {
        hit = topic.name;
        break;
      }
    }
    if (hit) break;
  }

  if (!hit) {
    tooltip.hidden = true;
    return;
  }
  tooltip.hidden = false;
  tooltip.textContent = hit;
  tooltip.style.left = `${x + 12}px`;
  tooltip.style.top = `${y - 8}px`;
});

timelineCanvas.addEventListener("mouseleave", () => {
  tooltip.hidden = true;
});

timelineCanvas.addEventListener("contextmenu", (event) => {
  event.preventDefault();
  const { x } = eventToContentPoint(event);
  window.PlaybackClock.setTime(x / pxPerUs);
});

// ---------------------------------------------------------------------
// Transport bar
// ---------------------------------------------------------------------

const playPauseBtn = document.getElementById("play-pause-btn");

playPauseBtn.addEventListener("click", () => {
  window.PlaybackClock.togglePlaying();
});

document.getElementById("seek-start-btn").addEventListener("click", () => {
  window.PlaybackClock.setTime(0);
});

document.getElementById("speed-select").addEventListener("change", (event) => {
  window.PlaybackClock.speedMultiplier = Number(event.target.value);
});

window.PlaybackClock.subscribe(() => {
  playPauseBtn.textContent = window.PlaybackClock.playing ? "⏸" : "▶";
  // Auto-scroll to keep the playhead in view while playing.
  const playheadX = window.PlaybackClock.currentTimeUs * pxPerUs;
  if (window.PlaybackClock.playing && (playheadX < wrap.scrollLeft || playheadX > wrap.scrollLeft + wrap.clientWidth - 40)) {
    wrap.scrollLeft = playheadX - wrap.clientWidth / 3;
  }
  render();
});

// ---------------------------------------------------------------------
// Timeline panel collapse (same mechanism as the sidebar)
// ---------------------------------------------------------------------

const timelinePanel = document.getElementById("timeline-panel");
document.getElementById("timeline-toggle-btn").addEventListener("click", () => {
  timelinePanel.classList.toggle("collapsed");
});

// ---------------------------------------------------------------------
// Startup
// ---------------------------------------------------------------------

async function fetchJSON(url) {
  const response = await fetch(url);
  const body = await response.json();
  if (!response.ok) throw new Error(body && body.error ? body.error : `request failed (${response.status})`);
  return body;
}

window.addEventListener("resize", render);
new ResizeObserver(render).observe(wrap);

window.addEventListener("aurorus:session-loaded", async () => {
  try {
    const executors = await fetchJSON("/api/timeline");
    rows = executors.map((executor) => ({
      name: executor.name,
      topics: executor.topics.map((topic) => ({ name: topic.name, colorIndex: topic.color_index, timestamps: topic.timestamps_us })),
    }));

    // Pick an initial zoom that fits the whole recording in the visible width.
    const duration = durationUs();
    if (duration > 0) {
      pxPerUs = clamp(wrap.clientWidth / duration, MIN_PX_PER_S / 1e6, MAX_PX_PER_S / 1e6);
    }
    syncZoomSlider();
    render();
  } catch (err) {
    console.error(err);
  }
});
