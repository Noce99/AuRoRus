"use strict";

// ---------------------------------------------------------------------
// Benchmark panel - an autonomous algorithm driving the ego vehicle alone
// for a fixed number of laps, on every map x vehicle model x race line
// picked (`/api/benchmark`). The benchmark is the server's, not this tab's:
// every tab shows the same one, and the server refuses anything that would
// change the setup while it runs. Loaded after app.js, whose `countDown`,
// `formatDuration` and `isTypingTarget` it uses.
// ---------------------------------------------------------------------

const benchNavBtn = document.querySelector('.panel-nav-btn[data-panel="benchmark"]');
const benchLapsEl = document.getElementById("bench-laps");
const benchFolderEl = document.getElementById("bench-folder");
const benchSetupEl = document.getElementById("bench-setup");
const benchAlgorithmEl = document.getElementById("bench-algorithm");
const benchMapsEl = document.getElementById("bench-maps");
const benchModelsEl = document.getElementById("bench-models");
const benchLinesEl = document.getElementById("bench-lines");
const benchPlanEl = document.getElementById("bench-plan");
const benchStartBtn = document.getElementById("bench-start-btn");
const benchProgressEl = document.getElementById("bench-progress");
const benchStateEl = document.getElementById("bench-state");
const benchProgressDetailsEl = document.getElementById("bench-progress-details");
const benchBarFillEl = document.getElementById("bench-bar-fill");
const benchAbortBtn = document.getElementById("bench-abort-btn");
const benchErrorEl = document.getElementById("bench-error");
const benchResultsEl = document.getElementById("bench-results");

/** The race line file an algorithm that follows none is timed against. */
const BENCH_TIMING_LINE = "centerline.csv";

const BENCH_STATUS_LABELS = {
  completed: "Completed",
  timeout: "Timeout",
  aborted_by_user: "Aborted",
};

/** `GET /api/benchmark/options`, once fetched. */
let benchOptions = null;
/** What's picked. Kept across refreshes of the options: an item seen
 *  before keeps its tick, a new one gets its default. */
const benchPicks = {
  maps: new Set(),
  models: new Set(),
  /** "<map>/<file>" keys. */
  lines: new Set(),
  /** "<group>:<key>" of every item ever offered. */
  seen: new Set(),
};
/** The latest `GET /api/benchmark`. */
let benchStatus = null;
/** The "Go!" last counted down to, so each run's is counted down once. */
let benchCountedGoAt = null;

function showBenchError(message) {
  benchErrorEl.textContent = message ?? "";
  benchErrorEl.hidden = message === null;
}

function lineKey(map, file) {
  return `${map}/${file}`;
}

function methodLabel(method) {
  return method.replace(/_/g, " ");
}

/** A race line as listed: its file stem, and how it was computed unless
 *  that's its name already (the centerline). */
function lineLabel(line) {
  const stem = line.file.replace(/\.csv$/, "");
  const method = methodLabel(line.method);
  return stem === method ? stem : `${stem} · ${method}`;
}

function benchAlgorithm() {
  return benchOptions?.algorithms.find((algorithm) => algorithm.name === benchAlgorithmEl.value);
}

/** Ticks `key` in `benchPicks[group]` the first time it's seen, if
 *  `byDefault`. */
function seed(group, key, byDefault) {
  const seenKey = `${group}:${key}`;
  if (benchPicks.seen.has(seenKey)) return;
  benchPicks.seen.add(seenKey);
  if (byDefault) benchPicks[group].add(key);
}

async function refreshBenchOptions() {
  const options = await fetchJSON("/api/benchmark/options");
  const firstTime = benchOptions === null;
  benchOptions = options;
  benchLapsEl.textContent = options.laps;

  for (const map of options.maps) {
    seed("maps", map.name, map.lap_timeout_s !== null);
    for (const line of map.race_lines) seed("lines", lineKey(map.name, line.file), true);
  }
  // Only the model running now: every model multiplies the runs.
  for (const model of options.models) {
    seed("models", model.kind, model.kind === options.current_model);
  }

  const previous = benchAlgorithmEl.value;
  benchAlgorithmEl.replaceChildren(
    ...options.algorithms.map((algorithm) => {
      const option = document.createElement("option");
      option.value = algorithm.name;
      option.textContent = algorithm.label;
      return option;
    }),
  );
  const wanted = firstTime ? options.selected_algorithm : previous;
  if (wanted && options.algorithms.some((algorithm) => algorithm.name === wanted)) {
    benchAlgorithmEl.value = wanted;
  }
  renderBenchSetup();
}

function checkboxRow(label, checked, disabled, onChange, title = "") {
  const li = document.createElement("li");
  const input = document.createElement("input");
  input.type = "checkbox";
  input.checked = checked;
  input.disabled = disabled;
  input.addEventListener("change", () => onChange(input.checked));
  const labelEl = document.createElement("label");
  labelEl.append(input, document.createTextNode(label));
  if (title) labelEl.title = title;
  li.append(labelEl);
  return li;
}

function toggle(set, key, on) {
  if (on) set.add(key);
  else set.delete(key);
  renderBenchSetup();
}

function setCount(groupEl, picked, total) {
  groupEl.querySelector(".bench-group-count").textContent = `(${picked}/${total})`;
}

/** The runs the picks make, as the server expands them. */
function benchRuns() {
  const algorithm = benchAlgorithm();
  if (!benchOptions || !algorithm) return [];
  const runs = [];
  for (const map of benchOptions.maps) {
    if (!benchPicks.maps.has(map.name) || map.lap_timeout_s === null) continue;
    const lines = algorithm.uses_race_line
      ? map.race_lines
          .filter((line) => benchPicks.lines.has(lineKey(map.name, line.file)))
          .map((line) => line.file)
      : [BENCH_TIMING_LINE];
    for (const model of benchOptions.models) {
      if (!benchPicks.models.has(model.kind)) continue;
      for (const line of lines) runs.push({ map, model: model.kind, line });
    }
  }
  return runs;
}

function renderBenchSetup() {
  if (!benchOptions) return;
  const algorithm = benchAlgorithm();
  const usesLine = algorithm?.uses_race_line ?? true;

  const mapList = benchMapsEl.querySelector(".bench-options");
  mapList.replaceChildren(
    ...benchOptions.maps.map((map) => {
      const usable = map.lap_timeout_s !== null;
      return checkboxRow(
        map.name,
        usable && benchPicks.maps.has(map.name),
        !usable,
        (on) => toggle(benchPicks.maps, map.name, on),
        usable ? "" : "No centerline - laps can't be timed on this map.",
      );
    }),
  );
  const usableMaps = benchOptions.maps.filter((map) => map.lap_timeout_s !== null);
  setCount(
    benchMapsEl,
    usableMaps.filter((map) => benchPicks.maps.has(map.name)).length,
    usableMaps.length,
  );

  const modelList = benchModelsEl.querySelector(".bench-options");
  modelList.replaceChildren(
    ...benchOptions.models.map((model) =>
      checkboxRow(model.label, benchPicks.models.has(model.kind), false, (on) =>
        toggle(benchPicks.models, model.kind, on),
      ),
    ),
  );
  setCount(
    benchModelsEl,
    benchOptions.models.filter((model) => benchPicks.models.has(model.kind)).length,
    benchOptions.models.length,
  );

  // One block per map, greyed out while the map isn't picked; the whole
  // group while the algorithm follows no race line.
  benchLinesEl.classList.toggle("disabled", !usesLine);
  benchLinesEl.title = usesLine
    ? ""
    : "This algorithm follows no race line: it runs once per map and model, timed against the centerline.";
  const lineBlocks = benchOptions.maps.map((map) => {
    const block = document.createElement("div");
    block.className = "bench-line-map";
    const mapPicked = benchPicks.maps.has(map.name) && map.lap_timeout_s !== null;
    block.classList.toggle("disabled", !mapPicked);
    const heading = document.createElement("h3");
    heading.textContent = map.name;
    const list = document.createElement("ul");
    list.replaceChildren(
      ...map.race_lines.map((line) => {
        const key = lineKey(map.name, line.file);
        return checkboxRow(
          lineLabel(line),
          benchPicks.lines.has(key),
          !usesLine || !mapPicked,
          (on) => toggle(benchPicks.lines, key, on),
          `${line.lap_length_m.toFixed(1)} m`,
        );
      }),
    );
    if (map.race_lines.length === 0) {
      const empty = document.createElement("li");
      empty.className = "parameter-description";
      empty.textContent = "No race lines.";
      list.append(empty);
    }
    block.append(heading, list);
    return block;
  });
  benchLinesEl.querySelector(".bench-options").replaceChildren(...lineBlocks);
  const allLines = benchOptions.maps.flatMap((map) =>
    map.race_lines.map((line) => lineKey(map.name, line.file)),
  );
  setCount(benchLinesEl, allLines.filter((key) => benchPicks.lines.has(key)).length, allLines.length);

  const runs = benchRuns();
  const worstS = runs.reduce(
    (total, run) => total + benchOptions.countdown_s + run.map.lap_timeout_s * benchOptions.laps,
    0,
  );
  benchPlanEl.textContent =
    runs.length === 0
      ? "Nothing to run - pick at least one map, vehicle model and race line."
      : `${runs.length} run${runs.length === 1 ? "" : "s"} × ${benchOptions.laps} laps · worst case ~${formatDuration(worstS)} (every lap timing out).`;
  benchStartBtn.disabled = runs.length === 0 || benchStatus?.running === true;
}

/** Wires a group's All/None buttons: they sit in its <summary>, so they
 *  mustn't also open or close it. */
function wireAllNone(groupEl, keys, set) {
  groupEl.querySelector(".bench-all").addEventListener("click", (event) => {
    event.preventDefault();
    for (const key of keys()) set.add(key);
    renderBenchSetup();
  });
  groupEl.querySelector(".bench-none").addEventListener("click", (event) => {
    event.preventDefault();
    for (const key of keys()) set.delete(key);
    renderBenchSetup();
  });
}

wireAllNone(
  benchMapsEl,
  () =>
    (benchOptions?.maps ?? []).filter((map) => map.lap_timeout_s !== null).map((map) => map.name),
  benchPicks.maps,
);
wireAllNone(benchModelsEl, () => (benchOptions?.models ?? []).map((model) => model.kind), benchPicks.models);
wireAllNone(
  benchLinesEl,
  () =>
    (benchOptions?.maps ?? []).flatMap((map) =>
      map.race_lines.map((line) => lineKey(map.name, line.file)),
    ),
  benchPicks.lines,
);

benchAlgorithmEl.addEventListener("change", renderBenchSetup);

benchStartBtn.addEventListener("click", async () => {
  showBenchError(null);
  const runs = benchRuns();
  const maps = [...new Set(runs.map((run) => run.map.name))];
  const raceLines = {};
  for (const run of runs) {
    const lines = (raceLines[run.map.name] ??= []);
    if (!lines.includes(run.line)) lines.push(run.line);
  }
  benchStartBtn.disabled = true;
  try {
    await fetchJSON("/api/benchmark/start", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        algorithm: benchAlgorithmEl.value,
        maps,
        models: [...new Set(runs.map((run) => run.model))],
        race_lines: raceLines,
      }),
    });
    await pollBenchmark();
  } catch (err) {
    showBenchError(`Couldn't start: ${err.message}`);
    renderBenchSetup();
  }
});

function abortBenchmark() {
  fetchJSON("/api/benchmark/abort", { method: "POST" }).catch((err) =>
    showBenchError(`Couldn't abort: ${err.message}`),
  );
}

benchAbortBtn.addEventListener("click", abortBenchmark);

function modelLabel(kind) {
  return benchOptions?.models.find((model) => model.kind === kind)?.label ?? kind;
}

function describeRun(run) {
  const parts = [run.map, modelLabel(run.model)];
  if (benchAlgorithmUsesLine(benchStatus?.algorithm)) parts.push(run.race_line.replace(/\.csv$/, ""));
  return parts.join(" · ");
}

function benchAlgorithmUsesLine(name) {
  return benchOptions?.algorithms.find((algorithm) => algorithm.name === name)?.uses_race_line ?? true;
}

function renderBenchProgress(status) {
  const progress = status.progress;
  benchAbortBtn.disabled = status.aborting;
  if (status.aborting) {
    benchStateEl.textContent = "Aborting...";
  } else if (progress === null) {
    benchStateEl.textContent = "Starting...";
  } else {
    const stage = {
      preparing: "Preparing",
      countdown: "Countdown",
      driving: progress.lap === 0 ? "Crossing the line" : `Lap ${progress.lap}/${status.laps}`,
    }[progress.stage];
    benchStateEl.textContent = `Run ${progress.run}/${status.runs} · ${stage}`;
  }
  if (progress === null) {
    benchProgressDetailsEl.textContent = "";
    benchBarFillEl.style.width = "0%";
    return;
  }
  const lines = [`${status.algorithm} · ${describeRun(progress)}`];
  if (progress.stage === "driving") {
    lines.push(
      `Elapsed ${formatDuration(progress.elapsed_s)} · this lap ${formatDuration(progress.lap_elapsed_s)} of ${formatDuration(progress.lap_timeout_s)} max`,
    );
  }
  benchProgressDetailsEl.textContent = lines.join("\n");
  const lapsDone = Math.max(progress.lap - 1, 0);
  const fraction = (progress.run - 1 + lapsDone / status.laps) / status.runs;
  benchBarFillEl.style.width = `${(100 * fraction).toFixed(1)}%`;
}

function renderBenchResults(status) {
  benchResultsEl.replaceChildren(
    ...status.outcomes.map((outcome) => {
      const li = document.createElement("li");
      const badge = document.createElement("span");
      badge.className = `bench-badge ${outcome.status ?? "skipped"}`;
      badge.textContent = outcome.status ? BENCH_STATUS_LABELS[outcome.status] : "Skipped";
      const what = document.createElement("span");
      what.className = "bench-result-run";
      what.textContent = describeRun(outcome);
      const numbers = document.createElement("span");
      numbers.className = "bench-result-numbers";
      if (outcome.error !== null) {
        numbers.textContent = outcome.error;
      } else {
        const parts = [`${outcome.laps}/${status.laps} laps`];
        if (outcome.total_time_s !== null) parts.push(`total ${outcome.total_time_s.toFixed(2)} s`);
        if (outcome.best_lap_s !== null) parts.push(`best ${outcome.best_lap_s.toFixed(2)} s`);
        numbers.textContent = parts.join(" · ");
      }
      if (outcome.folder !== null) li.title = outcome.folder;
      li.append(badge, what, numbers);
      return li;
    }),
  );
}

async function pollBenchmark() {
  const status = await fetchJSON("/api/benchmark");
  const wasRunning = benchStatus?.running === true;
  benchStatus = status;
  benchFolderEl.textContent = `${status.root}/<map>/`;

  document.body.classList.toggle("benchmark-active", status.running);
  benchNavBtn.classList.toggle("recording", status.running);
  benchSetupEl.hidden = status.running;
  benchProgressEl.hidden = !status.running;
  if (status.running) renderBenchProgress(status);
  renderBenchResults(status);

  const goAt = status.progress?.go_at_ms ?? null;
  if (goAt !== null && goAt !== benchCountedGoAt && status.go_in_ms !== null) {
    benchCountedGoAt = goAt;
    countDown(status.go_in_ms);
  }
  // Back from a benchmark: the selections it made are the new normal.
  if (wasRunning && !status.running) refreshBenchOptions().catch((err) => console.error(err));
  renderBenchSetup();
}

// Any WASD key aborts a running benchmark (the server would anyway, on the
// command the key sends); R and P, which the server refuses meanwhile, do
// nothing - R would otherwise reload the page for no restart.
window.addEventListener(
  "keydown",
  (event) => {
    if (!benchStatus?.running || isTypingTarget(event.target)) return;
    const key = event.key.toLowerCase();
    if (["w", "a", "s", "d"].includes(key)) {
      if (!event.repeat) abortBenchmark();
    } else if (["r", "p"].includes(key) && !event.ctrlKey && !event.metaKey && !event.altKey) {
      event.preventDefault();
      event.stopImmediatePropagation();
    }
  },
  { capture: true },
);

benchNavBtn.addEventListener("click", () => {
  if (!benchStatus?.running) refreshBenchOptions().catch((err) => showBenchError(err.message));
});

/** How often the benchmark's status is re-read - often enough for the
 *  countdown and lap counter, from any tab. */
const BENCH_POLL_MS = 250;

refreshBenchOptions().catch((err) => console.error(err));
startPolling(pollBenchmark, BENCH_POLL_MS);
