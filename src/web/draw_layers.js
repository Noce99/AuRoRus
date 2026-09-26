"use strict";

// ---------------------------------------------------------------------
// DrawLayers - the client side of the drawing protocol (see
// `src/web/draw.rs`), shared by every web UI in this project.
//
// Everything on the map canvas comes from drawing topics
// (`draw/<executor>`, see `src/topics/drawing.rs`): one layer per topic,
// polled together from `POST /api/draw`. This knows how to keep those
// layers current, fade the stale ones, and dead-reckon vehicles between
// samples - and nothing about which executor drew what.
//
// The only thing a host changes is its clock: `web_gui` ages layers
// against the wall clock (`performance.now()`), `replay_web_gui`
// against the playback time, so a paused recording stays still and a
// recorded executor that stopped publishing fades out on replay.
//
// Served at /draw_layers.js by both binaries, after /map_view.js.
// ---------------------------------------------------------------------

window.DrawLayers = (() => {
  /** A layer older than its drawing's `stale_after_ms` fades out over this
   *  long, down to `STALE_OPACITY` - kept faintly visible rather than
   *  hidden, so the last thing a crashed executor drew is still there to
   *  look at. */
  const FADE_DURATION_MS = 1000;
  const STALE_OPACITY = 0.2;

  function shapeKind(shape) {
    return Object.keys(shape)[0];
  }

  /** What shapes the ones no element covers are listed as. */
  const UNNAMED_ELEMENT = "Other";

  /** `drawing`'s elements as `{name, start, count, visibleByDefault}` -
   *  `start` the index of their first shape - with any shapes left over
   *  after the last one gathered into a shown-by-default
   *  `UNNAMED_ELEMENT`. */
  function elementsOf(drawing) {
    const elements = [];
    let start = 0;
    for (const { name, shape_count: count, visible_by_default: visibleByDefault } of drawing.elements ?? []) {
      elements.push({ name, start, count, visibleByDefault: visibleByDefault ?? true });
      start += count;
    }
    if (start < drawing.shapes.length) {
      elements.push({ name: UNNAMED_ELEMENT, start, count: drawing.shapes.length - start, visibleByDefault: true });
    }
    return elements;
  }

  /** `drawing`'s distinct elements as `{name, visibleByDefault}`, in order
   *  - the first element of each name setting its default. */
  function distinctElements(drawing) {
    const byName = new Map();
    for (const { name, visibleByDefault } of elementsOf(drawing)) {
      if (!byName.has(name)) byName.set(name, { name, visibleByDefault });
    }
    return [...byName.values()];
  }

  function elementKey(topic, name) {
    return `${topic}\n${name}`;
  }

  /** Creates the layers of one page. Options:
   *  - `clock()`: the "now", in ms, layers are aged and dead-reckoned
   *    against;
   *  - `requestExtras()`: extra fields for each `POST /api/draw` body
   *    (e.g. the playback time);
   *  - `maxExtrapolationMs()`: never dead-reckon a vehicle further than
   *    this past its sample - if samples stall, park it a little behind
   *    rather than fling it across the map;
   *  - `listEl`: a `<ul>` to render the layers into: one collapsible
   *    entry per layer, with a show/hide checkbox and how old it is, that
   *    opens onto a show/hide checkbox per element of its drawing;
   *  - `homeOnEveryNewRaster`: whether to home the view every time a new
   *    base-map raster (the bottom-most one) arrives (live, where that
   *    means the map was switched), or
   *    only the first time (replay, where seeking back and forth across the
   *    moment the map was loaded brings the same raster back again - and the
   *    view is the user's to move, not the timeline's). */
  function create({
    clock,
    requestExtras = () => ({}),
    maxExtrapolationMs = () => 150,
    listEl = null,
    homeOnEveryNewRaster = true,
  }) {
    /** The source's epoch the layers below were read under - a different
     *  one in a response means the source changed (e.g. a live restart),
     *  and every cached layer is stale. */
    let epoch = null;

    /** Topic name -> layer:
     *  `{topic, writer, writeCount, drawing, sampledAtMs, rasters}` -
     *  `drawing` as last received (`{shapes, stale_after_ms, z_index}`),
     *  `sampledAtMs` the `clock()` time it was written (or null if nothing
     *  was drawn yet), and `rasters` the decoded offscreen canvas of each
     *  raster shape, by shape index. */
    const layers = new Map();

    /** Topics the user unticked - kept by name, so a layer stays hidden
     *  across an epoch change. */
    const hidden = new Set();

    /** Whether the user ticked (`true`) or unticked (`false`) an element,
     *  by `elementKey` - elements missing here show as their drawing's
     *  `visible_by_default` says. Kept by name too, so a choice survives
     *  rewrites of the drawing, even ones the element is missing from for a
     *  while. Independent of `hidden`: unticking a whole layer and ticking
     *  it back keeps which of its elements were shown. */
    const elementChoices = new Map();

    /** Topics whose elements all show until the user unticks one,
     *  whatever their drawing's `visible_by_default` says - see `focus`. */
    const shownByDefault = new Set();

    function isElementShown(topic, name, visibleByDefault) {
      return elementChoices.get(elementKey(topic, name)) ?? (shownByDefault.has(topic) || visibleByDefault);
    }

    /** Shows `topic` (every element of it, until the user unticks one) and
     *  hides every other topic of `group` - e.g. the drawings of every
     *  autonomous algorithm but the selected one. Forgets what the user
     *  ticked or unticked in `group` before. `topic` may be null (hide the
     *  whole group), and a topic needn't be drawing yet: the choice is kept
     *  by name, like the user's. */
    function focus(topic, group) {
      for (const member of group) {
        for (const key of [...elementChoices.keys()]) {
          if (key.startsWith(`${member}\n`)) elementChoices.delete(key);
        }
        if (member === topic) {
          hidden.delete(member);
          shownByDefault.add(member);
        } else {
          hidden.add(member);
          shownByDefault.delete(member);
        }
      }
      if (listEl) renderList();
      MapView.requestRedraw();
    }

    /** Topics whose entry in the list is open. */
    const openTopics = new Set();

    /** Set when a new raster arrives (i.e. the map changed) to the `clock()`
     *  time it was drawn at: the view is homed once some vehicle drawing
     *  written at or after that time arrives, so it centers on where the
     *  vehicle was placed on the new map - which happens a few milliseconds
     *  after the map is written - not where it last was on the old one. */
    let pendingHomeAfterMs = null;
    /** Whether the view was homed yet - see `homeOnEveryNewRaster`. */
    let homedOnce = false;

    function ageMs(layer, nowMs) {
      return layer.sampledAtMs === null ? null : nowMs - layer.sampledAtMs;
    }

    function opacity(layer, nowMs) {
      const staleAfterMs = layer.drawing.stale_after_ms;
      const age = ageMs(layer, nowMs);
      if (staleAfterMs === null || age === null || age <= staleAfterMs) return 1;
      return Math.max(STALE_OPACITY, 1 - (age - staleAfterMs) / FADE_DURATION_MS);
    }

    function isFading(layer, nowMs) {
      const staleAfterMs = layer.drawing.stale_after_ms;
      const age = ageMs(layer, nowMs);
      return staleAfterMs !== null && age !== null && age > staleAfterMs && age < staleAfterMs + FADE_DURATION_MS;
    }

    /** Visible layers with a drawing, bottom first: by `z_index`, then
     *  topic. */
    function paintOrder() {
      return [...layers.values()]
        .filter((layer) => layer.drawing && !hidden.has(layer.topic))
        .sort((a, b) => a.drawing.z_index - b.drawing.z_index || a.topic.localeCompare(b.topic));
    }

    /** The shapes of `layer` whose element isn't hidden, as
     *  `{shape, index}` - `index` into `layer.drawing.shapes`. */
    function visibleShapes(layer) {
      const visible = [];
      // Elements sharing a name follow the first one's default.
      const defaults = new Map(distinctElements(layer.drawing).map((e) => [e.name, e.visibleByDefault]));
      for (const { name, start, count } of elementsOf(layer.drawing)) {
        if (!isElementShown(layer.topic, name, defaults.get(name))) continue;
        for (let index = start; index < start + count; index++) {
          visible.push({ shape: layer.drawing.shapes[index], index });
        }
      }
      return visible;
    }

    /** The bottom-most layer drawing a raster - the base map - whether
     *  it's shown or not. */
    function baseRasterLayer() {
      return [...layers.values()]
        .filter((layer) => layer.drawing?.shapes.some((shape) => shapeKind(shape) === "raster"))
        .sort((a, b) => a.drawing.z_index - b.drawing.z_index || a.topic.localeCompare(b.topic))[0];
    }

    /** `vehicle` advanced to `nowMs` along its own heading at its own
     *  speed - the same straight-line motion the simulator integrates
     *  between ticks. Steering curvature within one sample period is not
     *  modelled, which at a 33 ms period and 8 m/s is a few millimetres. */
    function extrapolatedVehicle(vehicle, layer, nowMs) {
      if (layer.sampledAtMs === null || vehicle.speed_mps === 0) return vehicle;
      const dtS = Math.min(Math.max(nowMs - layer.sampledAtMs, 0), maxExtrapolationMs()) / 1000;
      return {
        ...vehicle,
        x_m: vehicle.x_m + vehicle.speed_mps * Math.cos(vehicle.heading_rad) * dtS,
        y_m: vehicle.y_m + vehicle.speed_mps * Math.sin(vehicle.heading_rad) * dtS,
      };
    }

    /** A layer's visible shapes as they should be painted at `nowMs`:
     *  vehicles dead-reckoned forward, rasters with their decoded image
     *  attached. */
    function shapesAt(layer, nowMs) {
      return visibleShapes(layer).map(({ shape, index }) => {
        const kind = shapeKind(shape);
        if (kind === "vehicle") return { vehicle: extrapolatedVehicle(shape.vehicle, layer, nowMs) };
        if (kind === "raster") return { raster: { ...shape.raster, offscreen: layer.rasters[index] } };
        return shape;
      });
    }

    /** What `MapView`'s `layersAt` hook paints: every visible layer,
     *  bottom first, as `{opacity, shapes}`. */
    function layersAt() {
      const nowMs = clock();
      return paintOrder().map((layer) => ({ opacity: opacity(layer, nowMs), shapes: shapesAt(layer, nowMs) }));
    }

    /** The first vehicle any visible layer draws, dead-reckoned to now. */
    function firstVehicle() {
      const nowMs = clock();
      for (const layer of paintOrder()) {
        for (const { shape } of visibleShapes(layer)) {
          if (shapeKind(shape) === "vehicle") return extrapolatedVehicle(shape.vehicle, layer, nowMs);
        }
      }
      return null;
    }

    /** Union of every visible raster's extent - `MapView`'s `worldBounds`. */
    function worldBounds() {
      let bounds = null;
      for (const layer of paintOrder()) {
        for (const { shape } of visibleShapes(layer)) {
          if (shapeKind(shape) !== "raster") continue;
          const r = shape.raster;
          const minX = r.origin_x_m;
          const minY = r.origin_y_m;
          const maxX = minX + r.width_px * r.resolution_m_per_px;
          const maxY = minY + r.height_px * r.resolution_m_per_px;
          bounds = bounds
            ? {
                minX: Math.min(bounds.minX, minX),
                minY: Math.min(bounds.minY, minY),
                maxX: Math.max(bounds.maxX, maxX),
                maxY: Math.max(bounds.maxY, maxY),
              }
            : { minX, minY, maxX, maxY };
        }
      }
      return bounds;
    }

    /** The vehicle if one is drawn, else the middle of the drawn world -
     *  `MapView`'s `homeTarget`. */
    function homeTarget() {
      const vehicle = firstVehicle();
      if (vehicle) return { x: vehicle.x_m, y: vehicle.y_m };
      const bounds = worldBounds();
      return bounds ? { x: (bounds.minX + bounds.maxX) / 2, y: (bounds.minY + bounds.maxY) / 2 } : null;
    }

    function speedMps() {
      const vehicle = firstVehicle();
      return vehicle ? vehicle.speed_mps : null;
    }

    /** Whether some visible layer drew a vehicle at or after `sinceMs` - or
     *  none draws a vehicle at all, so there's nothing to wait for before
     *  homing. */
    function vehiclePlacedSince(sinceMs) {
      let anyVehicle = false;
      for (const layer of paintOrder()) {
        if (!visibleShapes(layer).some(({ shape }) => shapeKind(shape) === "vehicle")) continue;
        anyVehicle = true;
        if (layer.sampledAtMs !== null && layer.sampledAtMs >= sinceMs) return true;
      }
      return !anyVehicle;
    }

    /** Whether the picture changes by itself - a moving vehicle, or a
     *  layer mid-fade - so `MapView` should repaint every frame. */
    function isAnimating() {
      const vehicle = firstVehicle();
      if (vehicle && Math.abs(vehicle.speed_mps) > 1e-3) return true;
      const nowMs = clock();
      return paintOrder().some((layer) => isFading(layer, nowMs));
    }

    /** Fetches and decodes every raster shape of `drawing`, or returns
     *  null if any of them couldn't be - e.g. a `409` because the drawing
     *  was rewritten in the meantime - so the caller keeps its previous
     *  copy of the layer and picks the newer drawing up on the next poll. */
    async function loadRasters(topic, writeCount, drawing) {
      const rasters = [];
      for (const [i, shape] of drawing.shapes.entries()) {
        if (shapeKind(shape) !== "raster") continue;
        const r = shape.raster;
        const query = new URLSearchParams({ topic, shape: i, epoch, write_count: writeCount });
        const response = await fetch(`/api/draw/raster?${query}`);
        if (!response.ok) return null;
        const bytes = new Uint8Array(await response.arrayBuffer());
        if (bytes.length !== r.width_px * r.height_px) return null;
        rasters[i] = MapView.offscreenFromRaster(bytes, r.width_px, r.height_px);
      }
      return rasters;
    }

    /** Applies one `/api/draw` layer entry. Returns the shape kinds a newly
     *  received drawing contained. */
    async function applyLayer(entry, requestedAtMs) {
      let layer = layers.get(entry.topic);
      if (!layer) {
        layer = { topic: entry.topic, writer: null, writeCount: null, drawing: null, sampledAtMs: null, rasters: [] };
        layers.set(entry.topic, layer);
      }
      layer.writer = entry.writer;
      const sampledAtMs = entry.age_ms === null ? null : requestedAtMs - entry.age_ms;
      if (!entry.drawing) {
        // Unchanged since our copy - only its age moved on.
        layer.sampledAtMs = sampledAtMs;
        return new Set();
      }

      const hasRaster = entry.drawing.shapes.some((shape) => shapeKind(shape) === "raster");
      const rasters = hasRaster ? await loadRasters(entry.topic, entry.write_count, entry.drawing) : [];
      if (!rasters) return new Set();

      layer.drawing = entry.drawing;
      layer.writeCount = entry.write_count;
      layer.rasters = rasters;
      layer.sampledAtMs = sampledAtMs;
      return new Set(entry.drawing.shapes.map(shapeKind));
    }

    /** One `POST /api/draw` round: brings every layer up to date. Meant to
     *  be driven by `startPolling`. */
    async function poll() {
      const known = {};
      for (const layer of layers.values()) {
        if (layer.writeCount !== null) known[layer.topic] = layer.writeCount;
      }
      // Ages come back relative to the moment asked about, so they're
      // anchored to the clock as it was when asking.
      const requestedAtMs = clock();
      const response = await fetchJSON("/api/draw", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ epoch, known, ...requestExtras() }),
      });

      if (response.epoch !== epoch) {
        // A new source: every cached layer belongs to the old one, and the
        // server ignored `known`, sending everything in full.
        layers.clear();
        epoch = response.epoch;
      }

      const listedBefore = listSignature();
      const present = new Set(response.layers.map((entry) => entry.topic));
      for (const topic of [...layers.keys()]) {
        if (!present.has(topic)) layers.delete(topic);
      }

      const newRasterTopics = new Set();
      for (const entry of response.layers) {
        const kinds = await applyLayer(entry, requestedAtMs);
        if (kinds.has("raster")) newRasterTopics.add(entry.topic);
      }
      // Only the base map moves the view: an overlay raster republished as
      // it grows (e.g. SLAM's map) would otherwise re-home it every time.
      const base = baseRasterLayer();
      if (base && newRasterTopics.has(base.topic) && (homeOnEveryNewRaster || !homedOnce)) {
        pendingHomeAfterMs = Math.max(pendingHomeAfterMs ?? -Infinity, base.sampledAtMs ?? -Infinity);
      }
      if (pendingHomeAfterMs !== null && vehiclePlacedSince(pendingHomeAfterMs)) {
        pendingHomeAfterMs = null;
        homedOnce = true;
        MapView.home();
      }

      if (listEl && listSignature() !== listedBefore) renderList();
      MapView.requestRedraw();
    }

    // -----------------------------------------------------------------
    // Layer list
    // -----------------------------------------------------------------

    /** What the list shows - every layer and its elements - so it's only
     *  rebuilt when that changes, not on every poll. */
    function listSignature() {
      return [...layers.values()]
        .map((layer) =>
          [
            layer.topic,
            ...(layer.drawing ? distinctElements(layer.drawing) : []).map((e) => `${e.name}\t${e.visibleByDefault}`),
          ].join("\n"),
        )
        .join("\n\n");
    }

    function checkbox(checked, onChange) {
      const input = document.createElement("input");
      input.type = "checkbox";
      input.checked = checked;
      input.addEventListener("change", () => {
        onChange(input.checked);
        MapView.requestRedraw();
      });
      return input;
    }

    function renderList() {
      listEl.innerHTML = "";
      if (layers.size === 0) {
        const li = document.createElement("li");
        li.className = "empty";
        li.textContent = "Nothing is drawing yet";
        listEl.appendChild(li);
        return;
      }
      for (const topic of [...layers.keys()].sort()) {
        const layer = layers.get(topic);
        const li = document.createElement("li");
        const details = document.createElement("details");
        details.open = openTopics.has(topic);
        details.classList.toggle("layer-hidden", hidden.has(topic));
        details.addEventListener("toggle", () => {
          if (details.open) openTopics.add(topic);
          else openTopics.delete(topic);
        });

        // The checkbox sits right in the summary: clicking it toggles the
        // layer, clicking anywhere else opens or closes the entry.
        const summary = document.createElement("summary");
        const layerCheckbox = checkbox(!hidden.has(topic), (checked) => {
          if (checked) hidden.delete(topic);
          else hidden.add(topic);
          details.classList.toggle("layer-hidden", !checked);
        });
        const name = document.createElement("span");
        name.className = "layer-name";
        name.textContent = topic;
        const freshness = document.createElement("span");
        freshness.className = "layer-freshness";
        freshness.dataset.topic = topic;
        summary.append(layerCheckbox, name, freshness);
        details.appendChild(summary);

        const elements = document.createElement("ul");
        elements.className = "layer-elements";
        const drawnElements = layer.drawing ? distinctElements(layer.drawing) : [];
        if (drawnElements.length === 0) {
          const empty = document.createElement("li");
          empty.className = "empty";
          empty.textContent = "Nothing drawn";
          elements.appendChild(empty);
        }
        for (const { name: elementName, visibleByDefault } of drawnElements) {
          const key = elementKey(topic, elementName);
          const item = document.createElement("li");
          const label = document.createElement("label");
          const text = document.createElement("span");
          text.className = "layer-name";
          text.textContent = elementName;
          label.append(
            checkbox(isElementShown(topic, elementName, visibleByDefault), (checked) => {
              elementChoices.set(key, checked);
            }),
            text,
          );
          item.appendChild(label);
          elements.appendChild(item);
        }
        details.appendChild(elements);

        li.appendChild(details);
        listEl.appendChild(li);
      }
      renderFreshness();
    }

    /** Refreshes how old each listed layer is - call it periodically while
     *  the list is visible. */
    function renderFreshness() {
      if (!listEl) return;
      const nowMs = clock();
      for (const el of listEl.querySelectorAll(".layer-freshness")) {
        const layer = layers.get(el.dataset.topic);
        if (!layer || !layer.drawing) continue;
        const age = ageMs(layer, nowMs);
        const staleAfterMs = layer.drawing.stale_after_ms;
        MapView.setText(el, age === null ? "never drawn" : `${formatAge(Math.max(age, 0))} ago`);
        el.classList.toggle("stale", age === null || (staleAfterMs !== null && age > staleAfterMs));
      }
    }

    if (listEl) renderList();

    return { poll, layersAt, worldBounds, homeTarget, speedMps, isAnimating, renderFreshness, focus };
  }

  return { create };
})();
