"use strict";

// ---------------------------------------------------------------------
// Chart - the few canvas helpers every chart in this project's web UIs
// draws with: sizing a canvas to its box, "nice" tick steps, a gridded
// plot area with numbered axes, and placing a tooltip beside a point.
// Used by /lap_panel.js and by benchmark_viewer's charts. Served at
// /chart.js, before either.
// ---------------------------------------------------------------------

window.Chart = (() => {
  const AXIS_COLOR = "#ffffff";
  const GRID_COLOR = "rgba(255, 255, 255, 0.12)";
  const ZERO_COLOR = "rgba(255, 255, 255, 0.55)";

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

  /** How many decimals the ticks of `step` need. */
  function decimalsFor(step) {
    return Math.max(0, -Math.floor(Math.log10(step) + 1e-9));
  }

  function formatSigned(value, digits) {
    const text = value.toFixed(digits);
    return value > 0 ? `+${text}` : text;
  }

  /** Sizes `canvas` to `wrap`'s box at the device pixel ratio and clears it.
   *  Returns `{ctx, width, height}`, drawn in CSS pixels. */
  function prepare(canvas, wrap) {
    const dpr = window.devicePixelRatio || 1;
    const rect = wrap.getBoundingClientRect();
    const widthPx = Math.max(1, Math.round(rect.width * dpr));
    const heightPx = Math.max(1, Math.round(rect.height * dpr));
    if (canvas.width !== widthPx || canvas.height !== heightPx) {
      canvas.width = widthPx;
      canvas.height = heightPx;
    }
    const ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, rect.width, rect.height);
    return { ctx, width: rect.width, height: rect.height };
  }

  /** Draws the grid and numbered axes of `plot` (`{left, top, right,
   *  bottom}`, CSS pixels) over `[xMin, xMax] x [yMin, yMax]`, one grid line
   *  per tick of `xStep`/`yStep` - `y = 0` brighter when in range, y ticks
   *  signed if `signedY`. Returns `{toX, toY}`, value -> pixel. */
  function axes(ctx, plot, { xMin = 0, xMax, xStep, yMin, yMax, yStep, xLabel, yLabel, signedY = false }) {
    const toX = (x) => plot.left + ((x - xMin) / (xMax - xMin)) * (plot.right - plot.left);
    const toY = (y) => plot.top + ((yMax - y) / (yMax - yMin)) * (plot.bottom - plot.top);

    ctx.font = "11px system-ui, sans-serif";
    ctx.fillStyle = AXIS_COLOR;
    ctx.lineWidth = 1;
    const yDecimals = decimalsFor(yStep);
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    for (let v = Math.ceil(yMin / yStep - 1e-9) * yStep; v <= yMax + yStep / 2; v += yStep) {
      const isZero = Math.abs(v) < yStep / 2;
      const y = Math.round(toY(v)) + 0.5;
      ctx.strokeStyle = isZero ? ZERO_COLOR : GRID_COLOR;
      ctx.beginPath();
      ctx.moveTo(plot.left, y);
      ctx.lineTo(plot.right, y);
      ctx.stroke();
      const value = isZero ? 0 : v;
      ctx.fillText(signedY ? formatSigned(value, yDecimals) : value.toFixed(yDecimals), plot.left - 6, y);
    }
    const xDecimals = decimalsFor(xStep);
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    for (let x = Math.ceil(xMin / xStep - 1e-9) * xStep; x <= xMax + 1e-9; x += xStep) {
      const px = Math.round(toX(x)) + 0.5;
      ctx.strokeStyle = GRID_COLOR;
      ctx.beginPath();
      ctx.moveTo(px, plot.top);
      ctx.lineTo(px, plot.bottom);
      ctx.stroke();
      ctx.fillText(x.toFixed(xDecimals), px, plot.bottom + 5);
    }
    ctx.strokeStyle = AXIS_COLOR;
    ctx.beginPath();
    ctx.moveTo(plot.left + 0.5, plot.top);
    ctx.lineTo(plot.left + 0.5, plot.bottom + 0.5);
    ctx.lineTo(plot.right, plot.bottom + 0.5);
    ctx.stroke();
    ctx.textAlign = "right";
    ctx.fillText(xLabel, plot.right, plot.bottom + 19);
    ctx.textAlign = "left";
    ctx.textBaseline = "top";
    ctx.fillText(yLabel, 6, 2);
    return { toX, toY };
  }

  /** Writes `message` in the middle of `plot`. */
  function placeholder(ctx, plot, message) {
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillStyle = "rgba(255, 255, 255, 0.6)";
    ctx.fillText(message, (plot.left + plot.right) / 2, (plot.top + plot.bottom) / 2);
  }

  /** Shows `tooltip` beside `point` (CSS pixels in a box `width` x
   *  `height`), flipped to its other side near the right/top edge. */
  function placeTooltip(tooltip, point, width) {
    tooltip.hidden = false;
    const gap = 12;
    const tipWidth = tooltip.offsetWidth;
    const tipHeight = tooltip.offsetHeight;
    let left = point.x + gap;
    if (left + tipWidth > width) left = point.x - gap - tipWidth;
    let top = point.y - tipHeight - gap;
    if (top < 0) top = point.y + gap;
    tooltip.style.left = `${Math.max(0, left)}px`;
    tooltip.style.top = `${Math.max(0, top)}px`;
  }

  return { AXIS_COLOR, niceStep, decimalsFor, formatSigned, prepare, axes, placeholder, placeTooltip };
})();
