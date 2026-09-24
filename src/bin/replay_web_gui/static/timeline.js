"use strict";

// The timeline panel, its transport bar, and the Analyzed Topics list that
// picks which topics the timeline shows. Loaded after /map_view.js (for
// `fetchJSON`) and app.js (for `PlaybackClock`).
//
// One row per analyzed topic, each tick one recorded change. The canvas is
// only ever as big as the visible part of the panel: `#timeline-spacer` is
// sized to the whole timeline so the panel scrolls over it, while the
// canvas sticks to the visible area and draws just what's in view - with
// the topic labels pinned to its left edge and the time ruler to its top,
// so neither ever scrolls away or gets drawn over.

// A small, stable palette - PALETTE_SIZE in session.rs must stay in sync with
// this array's length; the two sides only need to agree on the modulus.
const PALETTE = [
  "#e6b800", "#3ea6ff", "#ff5c8a", "#3ddc84", "#c792ea",
  "#ff9d5c", "#5cd6d6", "#ff6b6b", "#8ab4f8", "#f4a261",
  "#a8e063", "#e879f9",
];

const RULER_HEIGHT = 24; // css px
const ROW_HEIGHT = 26; // css px
const TICK_HALF_HEIGHT = 8; // css px
const LABEL_WIDTH = 190; // css px - the pinned topic-label column
const LABEL_PADDING = 8; // css px
const RIGHT_MARGIN = 40; // css px of empty timeline after the last sample
const HOVER_TOLERANCE_PX = 5; // css px, in both axes

const MIN_PX_PER_S = 4;
const MAX_PX_PER_S = 4000;
const ZOOM_SLIDER_STEPS = 1000;

/** Every recorded topic, in writer-then-recording order:
 *  @type {{name:string, writer:string, colorIndex:number, timestamps:number[]}[]} */
let topics = [];
/** Names of the topics the user unticked in the Analyzed Topics list. */
const hiddenTopics = new Set();

let pxPerUs = 100 / 1e6; // 100 px/s by default, until the session's duration sets a sensible initial fit.

const wrap = document.getElementById("timeline-canvas-wrap");
const spacer = document.getElementById("timeline-spacer");
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

function visibleRows() {
  return topics.filter((topic) => !hiddenTopics.has(topic.name));
}

/** Timeline time -> x within the whole (scrollable) timeline, in css px. */
function timeToContentX(timeUs) {
  return LABEL_WIDTH + timeUs * pxPerUs;
}

function contentXToTime(contentX) {
  return (contentX - LABEL_WIDTH) / pxPerUs;
}

/** Sizes the spacer to the whole timeline, and the canvas to the visible
 *  part of it. */
function resize(rowCount) {
  const contentWidth = Math.max(wrap.clientWidth, timeToContentX(durationUs()) + RIGHT_MARGIN);
  const contentHeight = Math.max(wrap.clientHeight, RULER_HEIGHT + rowCount * ROW_HEIGHT);
  spacer.style.width = `${contentWidth}px`;
  spacer.style.height = `${contentHeight}px`;

  const cssWidth = wrap.clientWidth;
  const cssHeight = wrap.clientHeight;
  timelineCanvas.style.width = `${cssWidth}px`;
  timelineCanvas.style.height = `${cssHeight}px`;
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.round(cssWidth * dpr));
  const height = Math.max(1, Math.round(cssHeight * dpr));
  if (timelineCanvas.width !== width || timelineCanvas.height !== height) {
    timelineCanvas.width = width;
    timelineCanvas.height = height;
  }
  return { cssWidth, cssHeight };
}

/** `text` cut down with an ellipsis until it fits `maxWidth` css px. */
function fitText(text, maxWidth) {
  if (timelineCtx.measureText(text).width <= maxWidth) return text;
  let end = text.length;
  while (end > 0 && timelineCtx.measureText(`${text.slice(0, end)}…`).width > maxWidth) end--;
  return `${text.slice(0, end)}…`;
}

/** Index of the first of the sorted `timestamps` at or after `timeUs`. */
function firstAtOrAfter(timestamps, timeUs) {
  let lo = 0;
  let hi = timestamps.length;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (timestamps[mid] < timeUs) lo = mid + 1;
    else hi = mid;
  }
  return lo;
}

function render() {
  const rows = visibleRows();
  const { cssWidth, cssHeight } = resize(rows.length);
  const scrollX = wrap.scrollLeft;
  const scrollY = wrap.scrollTop;
  const dpr = window.devicePixelRatio || 1;
  const ctx = timelineCtx;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.font = "12px system-ui, sans-serif";
  ctx.textBaseline = "middle";

  ctx.fillStyle = "#15191e";
  ctx.fillRect(0, 0, cssWidth, cssHeight);

  // Only the rows and the time range actually in view get drawn.
  const firstRow = Math.max(0, Math.floor(scrollY / ROW_HEIGHT));
  const lastRow = Math.min(rows.length - 1, Math.floor((scrollY + cssHeight - RULER_HEIGHT) / ROW_HEIGHT));
  const rowY = (i) => RULER_HEIGHT + i * ROW_HEIGHT - scrollY;
  const timeToX = (timeUs) => timeToContentX(timeUs) - scrollX;
  const fromUs = Math.max(0, contentXToTime(scrollX + LABEL_WIDTH));
  const toUs = contentXToTime(scrollX + cssWidth);

  // Row backgrounds (alternating).
  for (let i = firstRow; i <= lastRow; i++) {
    ctx.fillStyle = i % 2 === 0 ? "#181c22" : "#15191e";
    ctx.fillRect(0, rowY(i), cssWidth, ROW_HEIGHT);
  }

  // Gridlines.
  const stepS = niceStepSeconds(pxPerUs * 1e6);
  const stepUs = stepS * 1e6;
  const firstGridUs = Math.floor(fromUs / stepUs) * stepUs;
  ctx.strokeStyle = "#2a3038";
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let t = firstGridUs; t <= toUs; t += stepUs) {
    const x = Math.round(timeToX(t)) + 0.5;
    ctx.moveTo(x, RULER_HEIGHT);
    ctx.lineTo(x, cssHeight);
  }
  ctx.stroke();

  // Ticks - only the ones within the visible time range.
  for (let i = firstRow; i <= lastRow; i++) {
    const topic = rows[i];
    const midY = rowY(i) + ROW_HEIGHT / 2;
    ctx.strokeStyle = PALETTE[topic.colorIndex % PALETTE.length];
    ctx.lineWidth = 2;
    ctx.beginPath();
    const timestamps = topic.timestamps;
    for (let k = firstAtOrAfter(timestamps, fromUs - 1); k < timestamps.length && timestamps[k] <= toUs; k++) {
      const x = timeToX(timestamps[k]);
      ctx.moveTo(x, midY - TICK_HALF_HEIGHT);
      ctx.lineTo(x, midY + TICK_HALF_HEIGHT);
    }
    ctx.stroke();
  }

  // Playhead.
  const playheadX = timeToX(window.PlaybackClock.currentTimeUs);
  if (playheadX >= LABEL_WIDTH) {
    ctx.strokeStyle = "#c77dff";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(playheadX, RULER_HEIGHT);
    ctx.lineTo(playheadX, cssHeight);
    ctx.stroke();
  }

  // Pinned label column, painted over whatever scrolled underneath it.
  ctx.fillStyle = "#1b1f24";
  ctx.fillRect(0, RULER_HEIGHT, LABEL_WIDTH, cssHeight - RULER_HEIGHT);
  for (let i = firstRow; i <= lastRow; i++) {
    const topic = rows[i];
    const y = rowY(i);
    ctx.fillStyle = PALETTE[topic.colorIndex % PALETTE.length];
    ctx.fillRect(LABEL_PADDING, y + ROW_HEIGHT / 2 - 4, 8, 8);
    ctx.fillStyle = "#cdd3db";
    ctx.fillText(fitText(topic.name, LABEL_WIDTH - 3 * LABEL_PADDING - 8), 2 * LABEL_PADDING + 8, y + ROW_HEIGHT / 2);
  }
  ctx.strokeStyle = "#303640";
  ctx.beginPath();
  ctx.moveTo(LABEL_WIDTH - 0.5, 0);
  ctx.lineTo(LABEL_WIDTH - 0.5, cssHeight);
  ctx.stroke();

  // Pinned ruler.
  ctx.fillStyle = "#15191e";
  ctx.fillRect(0, 0, cssWidth, RULER_HEIGHT);
  ctx.fillStyle = "#6b7684";
  ctx.save();
  ctx.beginPath();
  ctx.rect(LABEL_WIDTH, 0, cssWidth - LABEL_WIDTH, RULER_HEIGHT);
  ctx.clip();
  for (let t = firstGridUs; t <= toUs; t += stepUs) {
    ctx.fillText(`${(t / 1e6).toFixed(stepS < 1 ? 1 : 0)} s`, timeToX(t) + 4, RULER_HEIGHT / 2);
  }
  if (playheadX >= LABEL_WIDTH) {
    ctx.fillStyle = "#c77dff";
    ctx.beginPath();
    ctx.moveTo(playheadX - 5, RULER_HEIGHT - 8);
    ctx.lineTo(playheadX + 5, RULER_HEIGHT - 8);
    ctx.lineTo(playheadX, RULER_HEIGHT);
    ctx.closePath();
    ctx.fill();
  }
  ctx.restore();
  ctx.fillStyle = "#6b7684";
  ctx.fillText(rows.length === 0 ? "No topics selected" : `${rows.length} topics`, LABEL_PADDING, RULER_HEIGHT / 2);
  ctx.strokeStyle = "#303640";
  ctx.beginPath();
  ctx.moveTo(0, RULER_HEIGHT - 0.5);
  ctx.lineTo(cssWidth, RULER_HEIGHT - 0.5);
  ctx.stroke();
}

let renderQueued = false;

/** Coalesces a burst of scroll/resize/clock events into one render per
 *  frame. */
function requestRender() {
  if (renderQueued) return;
  renderQueued = true;
  requestAnimationFrame(() => {
    renderQueued = false;
    render();
  });
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

/** Zooms to `next` px/us, keeping the time at `anchorViewX` (css px from
 *  the canvas's left edge) where it is - the left edge of the time area
 *  when not given. */
function setPxPerUs(next, anchorViewX = LABEL_WIDTH) {
  const anchorUs = contentXToTime(wrap.scrollLeft + anchorViewX);
  pxPerUs = clamp(next, MIN_PX_PER_S / 1e6, MAX_PX_PER_S / 1e6);
  resize(visibleRows().length);
  wrap.scrollLeft = timeToContentX(anchorUs) - anchorViewX;
  syncZoomSlider();
  render();
}

wrap.addEventListener(
  "wheel",
  (event) => {
    // Over the topic labels - or with Shift held - the wheel scrolls
    // through the rows, natively; over the ticks it zooms time.
    const viewX = event.clientX - timelineCanvas.getBoundingClientRect().left;
    if (event.shiftKey || viewX < LABEL_WIDTH) return;
    event.preventDefault();
    const factor = Math.exp(-event.deltaY * 0.0015);
    setPxPerUs(pxPerUs * factor, viewX);
  },
  { passive: false }
);

timelineZoomSlider.addEventListener("input", () => {
  if (syncingZoomSlider) return;
  setPxPerUs(pxPerSFromSlider(Number(timelineZoomSlider.value)) / 1e6);
});

wrap.addEventListener("scroll", requestRender);

// ---------------------------------------------------------------------
// Hover tooltip, left-drag pan, right-button seek/scrub
// ---------------------------------------------------------------------

/** Where `event` is, in css px from the canvas's (i.e. the visible area's)
 *  top-left corner. */
function eventToViewPoint(event) {
  const rect = timelineCanvas.getBoundingClientRect();
  return { x: event.clientX - rect.left, y: event.clientY - rect.top };
}

function formatSeconds(timeUs) {
  return `${(timeUs / 1e6).toFixed(3)} s`;
}

timelineCanvas.addEventListener("mousemove", (event) => {
  if (drag) return;
  const { x, y } = eventToViewPoint(event);
  const topic = y >= RULER_HEIGHT ? visibleRows()[Math.floor((y - RULER_HEIGHT + wrap.scrollTop) / ROW_HEIGHT)] : null;
  let text = null;
  if (topic && x < LABEL_WIDTH) {
    text = `${topic.name} · written by ${topic.writer} · ${topic.timestamps.length} changes`;
  } else if (topic) {
    const hoverUs = contentXToTime(wrap.scrollLeft + x);
    const toleranceUs = HOVER_TOLERANCE_PX / pxPerUs;
    const k = firstAtOrAfter(topic.timestamps, hoverUs - toleranceUs);
    if (k < topic.timestamps.length && topic.timestamps[k] <= hoverUs + toleranceUs) {
      text = `${topic.name} @ ${formatSeconds(topic.timestamps[k])}`;
    }
  }

  if (!text) {
    tooltip.hidden = true;
    return;
  }
  tooltip.hidden = false;
  tooltip.textContent = text;
  tooltip.style.left = `${x + 12}px`;
  tooltip.style.top = `${y - 8}px`;
});

timelineCanvas.addEventListener("mouseleave", () => {
  tooltip.hidden = true;
});

/** The drag in progress, if any: `{kind: "pan", pointerId, startClientX,
 *  startScrollLeft}` for a left-button drag, or `{kind: "scrub",
 *  pointerId}` while the right button is held. Both start only over the
 *  ticks, never over the topic labels. */
let drag = null;

/** Seeks to the time under `event`'s pointer - clamped to the recording,
 *  and to the left edge of the time area if the pointer is over the
 *  labels or past the canvas. */
function seekToPointer(event) {
  const { x } = eventToViewPoint(event);
  window.PlaybackClock.setTime(contentXToTime(wrap.scrollLeft + Math.max(x, LABEL_WIDTH)));
}

timelineCanvas.addEventListener("pointerdown", (event) => {
  const { x } = eventToViewPoint(event);
  if (x < LABEL_WIDTH || drag) return;
  if (event.button === 0) {
    // A plain click with no movement pans by nothing - still a no-op.
    drag = { kind: "pan", pointerId: event.pointerId, startClientX: event.clientX, startScrollLeft: wrap.scrollLeft };
    timelineCanvas.classList.add("panning");
  } else if (event.button === 2) {
    drag = { kind: "scrub", pointerId: event.pointerId };
    seekToPointer(event);
  } else {
    return;
  }
  // Keep getting moves (and the release) even once the pointer leaves the
  // canvas mid-drag.
  timelineCanvas.setPointerCapture(event.pointerId);
  tooltip.hidden = true;
  event.preventDefault();
});

timelineCanvas.addEventListener("pointermove", (event) => {
  if (!drag || event.pointerId !== drag.pointerId) return;
  if (drag.kind === "pan") {
    // Scrolling by exactly how far the pointer moved keeps the time that
    // was under it when the drag began under it still.
    wrap.scrollLeft = drag.startScrollLeft - (event.clientX - drag.startClientX);
  } else {
    // The playhead follows the pointer; the map view follows the playhead
    // (see `PlaybackClock`), so it replays live while scrubbing.
    seekToPointer(event);
  }
});

function endDrag(event) {
  if (!drag || event.pointerId !== drag.pointerId) return;
  drag = null;
  timelineCanvas.classList.remove("panning");
}

timelineCanvas.addEventListener("pointerup", endDrag);
timelineCanvas.addEventListener("pointercancel", endDrag);

// The right button seeks and scrubs (see `pointerdown`), so it never opens
// the browser's context menu here.
timelineCanvas.addEventListener("contextmenu", (event) => event.preventDefault());

// ---------------------------------------------------------------------
// Analyzed Topics list (right panel) - which topics get a timeline row.
// ---------------------------------------------------------------------

const topicListEl = document.getElementById("topic-list");

function renderTopicList() {
  topicListEl.innerHTML = "";
  let writer = null;
  for (const topic of topics) {
    if (topic.writer !== writer) {
      writer = topic.writer;
      const group = document.createElement("li");
      group.className = "topic-group";
      group.textContent = writer;
      topicListEl.appendChild(group);
    }
    const li = document.createElement("li");
    const label = document.createElement("label");
    const checkbox = document.createElement("input");
    checkbox.type = "checkbox";
    checkbox.checked = !hiddenTopics.has(topic.name);
    checkbox.addEventListener("change", () => {
      if (checkbox.checked) hiddenTopics.delete(topic.name);
      else hiddenTopics.add(topic.name);
      requestRender();
    });
    const swatch = document.createElement("span");
    swatch.className = "topic-swatch";
    swatch.style.background = PALETTE[topic.colorIndex % PALETTE.length];
    const name = document.createElement("span");
    name.className = "topic-name";
    name.textContent = topic.name;
    name.title = topic.name;
    const count = document.createElement("span");
    count.className = "topic-count";
    count.textContent = String(topic.timestamps.length);
    count.title = "recorded changes";
    label.append(checkbox, swatch, name, count);
    li.appendChild(label);
    topicListEl.appendChild(li);
  }
}

document.getElementById("topics-all-btn").addEventListener("click", () => {
  hiddenTopics.clear();
  renderTopicList();
  requestRender();
});

document.getElementById("topics-none-btn").addEventListener("click", () => {
  for (const topic of topics) hiddenTopics.add(topic.name);
  renderTopicList();
  requestRender();
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
  const playheadViewX = timeToContentX(window.PlaybackClock.currentTimeUs) - wrap.scrollLeft;
  if (window.PlaybackClock.playing && (playheadViewX < LABEL_WIDTH || playheadViewX > wrap.clientWidth - RIGHT_MARGIN)) {
    wrap.scrollLeft = timeToContentX(window.PlaybackClock.currentTimeUs) - LABEL_WIDTH - (wrap.clientWidth - LABEL_WIDTH) / 3;
  }
  requestRender();
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

new ResizeObserver(requestRender).observe(wrap);

window.addEventListener("aurorus:session-loaded", async () => {
  try {
    const executors = await fetchJSON("/api/timeline");
    topics = executors.flatMap((executor) =>
      executor.topics.map((topic) => ({
        name: topic.name,
        writer: executor.name,
        colorIndex: topic.color_index,
        timestamps: topic.timestamps_us,
      }))
    );
    renderTopicList();

    // Pick an initial zoom that fits the whole recording in the visible width.
    const duration = durationUs();
    if (duration > 0) {
      pxPerUs = clamp((wrap.clientWidth - LABEL_WIDTH - RIGHT_MARGIN) / duration, MIN_PX_PER_S / 1e6, MAX_PX_PER_S / 1e6);
    }
    syncZoomSlider();
    render();
  } catch (err) {
    console.error(err);
  }
});
