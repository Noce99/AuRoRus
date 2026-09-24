"use strict";

// ---------------------------------------------------------------------
// MapView - the map canvas shared by every web UI in this project.
//
// Both frontends (`web_gui`, for driving live, and `debug_web_interface`,
// for replaying a recording) show the same thing: an occupancy raster, its
// start/finish line, and a vehicle on top, pannable and zoomable. All of
// that lives here, once.
//
// What they don't share is where the vehicle pose comes from - a live topic
// poll in one, a pre-fetched timeline in the other - so that is the hook a
// host supplies to `MapView.init`. Everything else (canvas sizing, world
// <-> screen transforms, wheel/drag/slider zoom and pan, the sidebar
// toggle, the redraw loop) is identical and handled here.
//
// Served at /map_view.js by both binaries, and loaded before their own
// app.js, which then talks to the `MapView` global.
// ---------------------------------------------------------------------

window.MapView = (() => {
  const MIN_VERTICAL_SIZE_M = 0.01;
  const DEFAULT_VERTICAL_SIZE_M = 10;
  const SLIDER_STEPS = 1000;
  const VEHICLE_LENGTH_M = 0.45;
  const VEHICLE_WIDTH_M = 0.25;
  const LIDAR_POINT_RADIUS_PX = 2.5;

  /** World-space view: how many meters of world height are visible, and
   *  which world point (in meters, same frame as MapInfo) is centered. */
  const view = { verticalSizeM: DEFAULT_VERTICAL_SIZE_M, centerX: 0, centerY: 0 };

  let canvas = null;
  let ctx = null;
  /** @type {{name:string|null, info:object|null, offscreen:HTMLCanvasElement}|null} */
  let currentMap = null;

  // Host hooks, filled in by init().
  let vehiclePoseAt = () => null;
  let lidarPointsAt = () => [];
  let isAnimating = () => false;
  let onFrame = () => {};

  let statusName = null;
  let statusVerticalSize = null;
  let statusSpeed = null;
  let zoomSlider = null;

  // -------------------------------------------------------------------
  // Small helpers
  // -------------------------------------------------------------------

  function clamp(value, min, max) {
    return Math.min(max, Math.max(min, value));
  }

  /** Writes `text` only when it differs from what the element already
   *  shows. The render loop calls this every frame, and an unconditional
   *  `textContent` write dirties layout even for an identical string. */
  function setText(element, text) {
    if (element && element.textContent !== text) element.textContent = text;
  }

  function maxVerticalSizeM() {
    if (!currentMap || !currentMap.info) return 100;
    return currentMap.info.height_px * currentMap.info.resolution_m_per_px;
  }

  // -------------------------------------------------------------------
  // Canvas sizing (device-pixel aware)
  // -------------------------------------------------------------------

  function resizeCanvasToDisplaySize() {
    const rect = canvas.parentElement.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    const width = Math.max(1, Math.round(rect.width * dpr));
    const height = Math.max(1, Math.round(rect.height * dpr));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
  }

  // -------------------------------------------------------------------
  // World <-> screen transforms (device pixels)
  // -------------------------------------------------------------------

  function scalePxPerMeter() {
    return canvas.height / view.verticalSizeM;
  }

  function screenToWorld(screenX, screenY) {
    const scale = scalePxPerMeter();
    return {
      x: (screenX - canvas.width / 2) / scale + view.centerX,
      y: (screenY - canvas.height / 2) / scale + view.centerY,
    };
  }

  function worldToScreen(worldX, worldY) {
    const scale = scalePxPerMeter();
    return {
      x: (worldX - view.centerX) * scale + canvas.width / 2,
      y: (worldY - view.centerY) * scale + canvas.height / 2,
    };
  }

  // Client (CSS) pixels -> device pixels, for mouse/wheel event coordinates.
  function clientToDevice(clientX, clientY) {
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    return { x: (clientX - rect.left) * dpr, y: (clientY - rect.top) * dpr };
  }

  // -------------------------------------------------------------------
  // Rendering
  // -------------------------------------------------------------------

  function drawVehicle(nowMs) {
    const pose = vehiclePoseAt(nowMs);
    if (!pose) return;

    const { x, y } = worldToScreen(pose.x_m, pose.y_m);
    const scale = scalePxPerMeter();
    const lengthPx = VEHICLE_LENGTH_M * scale;
    const widthPx = VEHICLE_WIDTH_M * scale;

    ctx.save();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.translate(x, y);
    ctx.rotate(pose.heading_rad);

    ctx.fillStyle = "#ffb020";
    ctx.fillRect(-lengthPx / 2, -widthPx / 2, lengthPx, widthPx);
    ctx.strokeStyle = "#101418";
    ctx.lineWidth = 1.5 * (window.devicePixelRatio || 1);
    ctx.strokeRect(-lengthPx / 2, -widthPx / 2, lengthPx, widthPx);

    // Small triangle marking the front, so heading is visible at a glance.
    ctx.beginPath();
    ctx.moveTo(lengthPx / 2, 0);
    ctx.lineTo(lengthPx / 2 - widthPx * 0.4, -widthPx * 0.35);
    ctx.lineTo(lengthPx / 2 - widthPx * 0.4, widthPx * 0.35);
    ctx.closePath();
    ctx.fillStyle = "#101418";
    ctx.fill();

    ctx.restore();
  }

  /** Draws each live LIDAR hit (world-frame `{x_m, y_m}`, as returned by
   *  `lidarPointsAt`) as a small red dot - a fixed device-pixel radius, so
   *  points stay legible at any zoom level rather than shrinking to nothing
   *  when zoomed out. */
  function drawLidarPoints(nowMs) {
    const points = lidarPointsAt(nowMs);
    if (!points || points.length === 0) return;

    ctx.setTransform(1, 0, 0, 1, 0, 0);
    const radius = LIDAR_POINT_RADIUS_PX * (window.devicePixelRatio || 1);
    ctx.fillStyle = "#ff3b3b";
    for (const point of points) {
      const { x, y } = worldToScreen(point.x_m, point.y_m);
      ctx.beginPath();
      ctx.arc(x, y, radius, 0, 2 * Math.PI);
      ctx.fill();
    }
  }

  function draw(nowMs) {
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = "#008080";
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    if (!currentMap || !currentMap.info) return;

    const info = currentMap.info;
    const scale = scalePxPerMeter();
    const pixelScale = scale * info.resolution_m_per_px;
    const e = (info.origin.x - view.centerX) * scale + canvas.width / 2;
    const f = (info.origin.y - view.centerY) * scale + canvas.height / 2;

    ctx.imageSmoothingEnabled = false;
    ctx.setTransform(pixelScale, 0, 0, pixelScale, e, f);
    ctx.drawImage(currentMap.offscreen, 0, 0);

    ctx.setTransform(1, 0, 0, 1, 0, 0);
    const a = worldToScreen(info.start_finish_line.a.x, info.start_finish_line.a.y);
    const b = worldToScreen(info.start_finish_line.b.x, info.start_finish_line.b.y);
    ctx.strokeStyle = "#ff3b3b";
    ctx.lineWidth = 2 * (window.devicePixelRatio || 1);
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();

    drawLidarPoints(nowMs);
    drawVehicle(nowMs);
  }

  function updateStatusBar(nowMs) {
    const pose = vehiclePoseAt(nowMs);
    setText(statusName, currentMap && currentMap.name ? currentMap.name : "No map loaded");
    setText(statusVerticalSize, `Vertical size: ${view.verticalSizeM.toFixed(2)} m`);
    setText(statusSpeed, pose ? `Speed: ${pose.speed_mps.toFixed(2)} m/s` : "");
  }

  // -------------------------------------------------------------------
  // Render loop
  //
  // Everything that wants a repaint calls `requestRedraw()`; this loop is
  // the only thing that ever paints. A burst of `mousemove` events (which
  // arrive faster than the display refreshes) therefore collapses into one
  // paint per frame instead of one each, and an animating host - a driving
  // vehicle, a playing recording - gets a smooth repaint every frame
  // without having to ask.
  // -------------------------------------------------------------------

  let needsRedraw = true;

  function requestRedraw() {
    needsRedraw = true;
  }

  function renderLoop(nowMs) {
    requestAnimationFrame(renderLoop);
    if (!needsRedraw && !isAnimating()) return;
    needsRedraw = false;

    resizeCanvasToDisplaySize();
    draw(nowMs);
    updateStatusBar(nowMs);
    onFrame(nowMs);
  }

  // -------------------------------------------------------------------
  // Zoom (mouse wheel + slider) and pan (left-button drag)
  // -------------------------------------------------------------------

  let syncingSlider = false;

  function sliderFromVerticalSize(sizeM) {
    const logMin = Math.log(MIN_VERTICAL_SIZE_M);
    const logMax = Math.log(maxVerticalSizeM());
    const t = (Math.log(sizeM) - logMin) / (logMax - logMin);
    return Math.round(clamp(t, 0, 1) * SLIDER_STEPS);
  }

  function verticalSizeFromSlider(value) {
    const logMin = Math.log(MIN_VERTICAL_SIZE_M);
    const logMax = Math.log(maxVerticalSizeM());
    const t = value / SLIDER_STEPS;
    return Math.exp(logMin + t * (logMax - logMin));
  }

  function syncZoomSlider() {
    if (!zoomSlider) return;
    syncingSlider = true;
    zoomSlider.value = String(sliderFromVerticalSize(view.verticalSizeM));
    syncingSlider = false;
  }

  function setVerticalSize(size) {
    view.verticalSizeM = clamp(size, MIN_VERTICAL_SIZE_M, maxVerticalSizeM());
    // Pushed from here, where the zoom actually changes, rather than from
    // the render loop - the slider is a DOM write and has no business
    // running once per frame.
    syncZoomSlider();
  }

  function zoomAt(deviceX, deviceY, factor) {
    const before = screenToWorld(deviceX, deviceY);
    setVerticalSize(view.verticalSizeM * factor);
    const after = screenToWorld(deviceX, deviceY);
    view.centerX += before.x - after.x;
    view.centerY += before.y - after.y;
    requestRedraw();
  }

  /** Centers the view on the start/finish line at the default zoom. */
  function homeToStartFinish() {
    if (!currentMap || !currentMap.info) return;
    const line = currentMap.info.start_finish_line;
    view.centerX = (line.a.x + line.b.x) / 2;
    view.centerY = (line.a.y + line.b.y) / 2;
    setVerticalSize(DEFAULT_VERTICAL_SIZE_M);
    requestRedraw();
  }

  function installInteractions() {
    canvas.addEventListener(
      "wheel",
      (event) => {
        event.preventDefault();
        const { x, y } = clientToDevice(event.clientX, event.clientY);
        zoomAt(x, y, Math.exp(event.deltaY * 0.0015));
      },
      { passive: false }
    );

    let dragging = false;
    let lastDevice = { x: 0, y: 0 };

    canvas.addEventListener("mousedown", (event) => {
      if (event.button !== 0) return;
      dragging = true;
      canvas.classList.add("panning");
      lastDevice = clientToDevice(event.clientX, event.clientY);
    });

    window.addEventListener("mousemove", (event) => {
      if (!dragging) return;
      const device = clientToDevice(event.clientX, event.clientY);
      const scale = scalePxPerMeter();
      view.centerX -= (device.x - lastDevice.x) / scale;
      view.centerY -= (device.y - lastDevice.y) / scale;
      lastDevice = device;
      requestRedraw();
    });

    window.addEventListener("mouseup", () => {
      dragging = false;
      canvas.classList.remove("panning");
    });

    if (zoomSlider) {
      zoomSlider.addEventListener("input", () => {
        if (syncingSlider || !currentMap) return;
        setVerticalSize(verticalSizeFromSlider(Number(zoomSlider.value)));
        requestRedraw();
      });
    }

    const homeBtn = document.getElementById("home-btn");
    if (homeBtn) homeBtn.addEventListener("click", homeToStartFinish);

    const sidebar = document.getElementById("sidebar");
    const sidebarToggle = document.getElementById("sidebar-toggle-btn");
    if (sidebar && sidebarToggle) {
      sidebarToggle.addEventListener("click", () => sidebar.classList.toggle("collapsed"));
    }

    // The ResizeObserver already fires for every size change of the
    // canvas's container, including the ones a window resize causes - a
    // `resize` listener on top of it would only buy a second redraw for the
    // same event.
    new ResizeObserver(requestRedraw).observe(canvas.parentElement);
  }

  // -------------------------------------------------------------------
  // Raster -> ImageData
  // -------------------------------------------------------------------

  /** Turns the one-byte-per-pixel occupancy raster both APIs serve into an
   *  `ImageData`. Written through a `Uint32Array` view, which fills a whole
   *  pixel per iteration instead of four separate byte stores - about 40%
   *  faster on a 1200x1200 map. */
  function buildImageData(bytes, width, height) {
    const rgba = new Uint8ClampedArray(width * height * 4);
    const pixels = new Uint32Array(rgba.buffer);
    // Little-endian byte order in memory is R,G,B,A, so these read as
    // 0xAABBGGRR: #ebebeb drivable, #281e22-ish wall, both fully opaque.
    const DRIVABLE = 0xffebebeb;
    const WALL = 0xff281e1e;
    for (let i = 0; i < pixels.length; i++) {
      pixels[i] = bytes[i] === 255 ? DRIVABLE : WALL;
    }
    return new ImageData(rgba, width, height);
  }

  /** Decodes a raster into an offscreen canvas of its own, ready to be
   *  handed to `setMap`. */
  function offscreenFromRaster(bytes, widthPx, heightPx) {
    const offscreen = document.createElement("canvas");
    offscreen.width = widthPx;
    offscreen.height = heightPx;
    offscreen.getContext("2d").putImageData(buildImageData(bytes, widthPx, heightPx), 0, 0);
    return offscreen;
  }

  // -------------------------------------------------------------------
  // Public surface
  // -------------------------------------------------------------------

  return {
    view,

    /** Wires the shared map view to the page and starts its render loop.
     *  `vehiclePoseAt(nowMs)` returns `{x_m, y_m, heading_rad, speed_mps}`
     *  or null; `lidarPointsAt(nowMs)` returns an array of world-frame
     *  `{x_m, y_m}` LIDAR hits to draw, or an empty array; `isAnimating()`
     *  says whether to repaint every frame even with no input; `onFrame(nowMs)`
     *  lets the host update its own status-bar extras from inside the same
     *  frame. */
    init({ vehiclePoseAt: poseFn, lidarPointsAt: lidarFn, isAnimating: animFn, onFrame: frameFn } = {}) {
      canvas = document.getElementById("map-canvas");
      ctx = canvas.getContext("2d");
      statusName = document.getElementById("status-map-name");
      statusVerticalSize = document.getElementById("status-vertical-size");
      statusSpeed = document.getElementById("status-speed");
      zoomSlider = document.getElementById("zoom-slider");

      if (poseFn) vehiclePoseAt = poseFn;
      if (lidarFn) lidarPointsAt = lidarFn;
      if (animFn) isAnimating = animFn;
      if (frameFn) onFrame = frameFn;

      installInteractions();
      requestAnimationFrame(renderLoop);
    },

    /** Replaces the displayed map (or clears it with `null`) and recenters
     *  on its start/finish line. */
    setMap(map) {
      currentMap = map;
      if (map && map.info) homeToStartFinish();
      requestRedraw();
    },

    currentMap: () => currentMap,
    requestRedraw,
    homeToStartFinish,
    scalePxPerMeter,
    screenToWorld,
    worldToScreen,
    buildImageData,
    offscreenFromRaster,
    clamp,
    setText,
  };
})();

/** Fetches `url` and parses it as JSON, turning a non-2xx response into a
 *  thrown `Error` carrying the API's own `{"error": "..."}` message when it
 *  sent one. Shared by both frontends. */
async function fetchJSON(url, options) {
  const response = await fetch(url, options);
  const body = await response.json();
  if (!response.ok) {
    throw new Error(body && body.error ? body.error : `request failed (${response.status})`);
  }
  return body;
}

/** Runs `poll` every `intervalMs`, but only ever with one request in
 *  flight: the next wait starts when the last response lands. `setInterval`
 *  would instead keep firing into a slow or stalled server and pile
 *  requests up behind each other. */
function startPolling(poll, intervalMs) {
  const tick = async () => {
    const startedMs = performance.now();
    try {
      await poll();
    } catch (err) {
      console.error(err);
    }
    setTimeout(tick, Math.max(0, intervalMs - (performance.now() - startedMs)));
  };
  tick();
}
