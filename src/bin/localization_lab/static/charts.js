"use strict";

// ---------------------------------------------------------------------
// The lab's charts: one line per shown run over a lap (by meters along the
// race line) or over the whole session (by time), with a cursor at the
// scrubbed time. Drawn with the shared /chart.js helpers, loaded before.
// ---------------------------------------------------------------------

window.LabCharts = (() => {
  const PLOT_MARGIN = { left: 46, right: 10, top: 28, bottom: 32 };
  const CURSOR_COLOR = "#ff6ec7";

  /** What each canvas last drew, to turn a click back into an x value. */
  const drawn = new WeakMap();

  /** Draws `chart` into `canvas`: `{title, series: [{color, points:
   *  [[x, y], ...]}], xMin, xMax, xLabel, cursorX, signed, empty}`. */
  function draw(canvas, chart) {
    const wrap = canvas.parentElement;
    const { ctx, width, height } = Chart.prepare(canvas, wrap);
    const plot = {
      left: PLOT_MARGIN.left,
      top: PLOT_MARGIN.top,
      right: width - PLOT_MARGIN.right,
      bottom: height - PLOT_MARGIN.bottom,
    };
    const ys = chart.series.flatMap((s) => s.points.map((p) => p[1]));
    if (ys.length === 0 || !(chart.xMax > chart.xMin)) {
      ctx.font = "12px system-ui, sans-serif";
      ctx.fillStyle = Chart.AXIS_COLOR;
      ctx.textAlign = "left";
      ctx.textBaseline = "top";
      ctx.fillText(chart.title, 6, 2);
      Chart.placeholder(ctx, plot, chart.empty ?? "No data");
      drawn.delete(canvas);
      return;
    }
    let yMin = chart.signed ? Math.min(0, ...ys) : 0;
    let yMax = Math.max(...ys);
    if (chart.signed) {
      const extent = Math.max(Math.abs(yMin), Math.abs(yMax)) || 1;
      yMin = -extent;
      yMax = extent;
    }
    if (!(yMax > yMin)) yMax = yMin + 1;
    // Whole ticks at both ends, so none lands outside the plot.
    const yStep = Chart.niceStep(yMax - yMin, Math.max(2, Math.floor((plot.bottom - plot.top) / 40)));
    yMax = Math.ceil(yMax / yStep - 1e-9) * yStep;
    yMin = Math.floor(yMin / yStep + 1e-9) * yStep;
    const { toX, toY } = Chart.axes(ctx, plot, {
      xMin: chart.xMin,
      xMax: chart.xMax,
      xStep: Chart.niceStep(chart.xMax - chart.xMin, Math.max(2, Math.floor((plot.right - plot.left) / 70))),
      yMin,
      yMax,
      yStep,
      xLabel: chart.xLabel,
      yLabel: chart.title,
      signedY: chart.signed,
    });

    ctx.save();
    ctx.beginPath();
    ctx.rect(plot.left, plot.top, plot.right - plot.left, plot.bottom - plot.top);
    ctx.clip();
    ctx.lineWidth = 1.5;
    for (const series of chart.series) {
      ctx.strokeStyle = series.color;
      ctx.beginPath();
      let previousX = null;
      for (const [x, y] of series.points) {
        const px = toX(x);
        const py = toY(y);
        // A jump back (a new lap in the whole-session view's wrap) or a
        // gap starts a new stroke.
        if (previousX === null || x < previousX) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
        previousX = x;
      }
      ctx.stroke();
    }
    if (chart.cursorX !== null && chart.cursorX !== undefined) {
      const px = Math.round(toX(chart.cursorX)) + 0.5;
      ctx.strokeStyle = CURSOR_COLOR;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(px, plot.top);
      ctx.lineTo(px, plot.bottom);
      ctx.stroke();
    }
    ctx.restore();
    drawn.set(canvas, { plot, xMin: chart.xMin, xMax: chart.xMax });
  }

  /** The x value under `event` on `canvas`, or null off its plot. */
  function xAt(canvas, event) {
    const last = drawn.get(canvas);
    if (!last) return null;
    const rect = canvas.getBoundingClientRect();
    const px = event.clientX - rect.left;
    const { plot, xMin, xMax } = last;
    if (px < plot.left || px > plot.right) return null;
    return xMin + ((px - plot.left) / (plot.right - plot.left)) * (xMax - xMin);
  }

  return { draw, xAt };
})();
