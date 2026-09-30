// car_calibration's page: polls /api/state, renders every step from it, and
// posts each step's action - every POST answers with the whole state too.
//
// Motor safety: a HOLD button sends a request every HOLD_EVERY_MS while
// pressed (the first one marked `start`); the server stops the motor as
// soon as they stop coming. Letting go, leaving the button, switching tabs
// or any error stops it at once.

"use strict";

const POLL_MS = 250;
const HOLD_EVERY_MS = 100;

const $ = (id) => document.getElementById(id);
let state = null;
// The car whose values the forms were last filled from.
let filledFor = null;

// ---------------------------------------------------------------------
// Talking to the server
// ---------------------------------------------------------------------

async function api(path, body) {
  const response = await fetch(path, {
    method: body === undefined ? "GET" : "POST",
    headers: body === undefined ? {} : { "Content-Type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  const text = await response.text();
  if (!response.ok) {
    let message = text;
    try {
      message = JSON.parse(text).error ?? text;
    } catch (_) {
      // Not JSON: the text itself.
    }
    throw new Error(message);
  }
  return JSON.parse(text);
}

/** Runs a step's POST, showing what went wrong - or `done` if it went through. */
async function step(path, body, done) {
  try {
    render(await api(path, body));
    showMessage(done ?? null, "info");
    return true;
  } catch (err) {
    showMessage(err.message);
    return false;
  }
}

function showMessage(text, kind) {
  const el = $("message");
  el.hidden = !text;
  el.textContent = text ?? "";
  el.classList.toggle("info", kind === "info");
}

async function poll() {
  try {
    render(await api("/api/state"));
  } catch (err) {
    $("chip-vesc").className = "chip bad";
    $("chip-vesc").textContent = "car_calibration unreachable";
  }
  setTimeout(poll, POLL_MS);
}

// ---------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------

const fmt = (value, digits = 3) => (value == null ? "-" : Number(value).toFixed(digits));
const deg = (rad) => (rad * 180) / Math.PI;

function chip(id, text, ok) {
  const el = $(id);
  el.textContent = text;
  el.className = "chip " + (ok ? "ok" : "bad");
}

function render(next) {
  state = next;
  const { car, vesc, lidar } = state;
  const draft = car.draft;

  // Header.
  $("car-title").textContent = car.name ? `- ${car.name}${car.is_new ? " (new)" : ""}` : "";
  chip(
    "chip-vesc",
    vesc.connected
      ? `VESC ${vesc.firmware?.hardware ?? ""} fw ${vesc.firmware?.version ?? ""}${vesc.fault && vesc.fault !== "none" ? " - fault " + vesc.fault : ""}`
      : `VESC: ${vesc.error ?? "connecting..."}`,
    vesc.connected,
  );
  chip("chip-lidar", lidar.connected ? `Lidar: ${lidar.readings} readings` : "Lidar: no scan", lidar.connected);
  chip("chip-battery", vesc.voltage_v == null ? "Battery -" : `Battery ${fmt(vesc.voltage_v, 2)} V`, vesc.voltage_v != null);
  const motor = $("chip-motor");
  motor.textContent = vesc.motor_running ? "MOTOR RUNNING" : "Motor stopped";
  motor.className = "chip " + (vesc.motor_running ? "running" : "ok");

  // Car choice.
  const known = $("known-cars");
  if (known.childElementCount !== state.known_cars.length) {
    known.replaceChildren(...state.known_cars.map((name) => new Option(name, name)));
  }
  $("car-note").textContent = car.name
    ? car.is_new
      ? `Calibrating the new car "${car.name}", starting from the template.`
      : `Calibrating "${car.name}", starting from its current calibration.`
    : state.car_name_file
      ? `This machine's CAR_NAME says "${state.car_name_file}".`
      : "This machine has no CAR_NAME yet.";
  $("steps").hidden = !draft;
  if (!draft) return;
  if (filledFor !== car.name) {
    filledFor = car.name;
    $("car-name").value = car.name;
    fillForms(draft);
  }
  for (const section of document.querySelectorAll("[data-step]")) {
    section.querySelector(".badge").textContent = state.done.includes(section.dataset.step) ? "done" : "";
  }

  // Battery.
  $("battery-voltage").textContent = vesc.voltage_v == null ? "-" : `${fmt(vesc.voltage_v, 2)} V`;
  const options = vesc.possible_cells ?? [];
  const optionsEl = $("battery-options");
  const optionsKey = options.join(",");
  if (optionsEl.dataset.key !== optionsKey) {
    optionsEl.dataset.key = optionsKey;
    optionsEl.replaceChildren(
      ...(options.length ? options : [2, 3, 4, 5, 6]).map((cells) => {
        const button = document.createElement("button");
        button.type = "button";
        button.textContent = `${cells}S`;
        button.onclick = () => step("/api/battery", { cells }, `${cells} cells.`);
        return button;
      }),
    );
  }
  $("battery-cells").textContent = draft.battery.cells;

  // IMU.
  const imu = vesc.imu;
  $("imu-live").textContent = imu
    ? `accel ${imu.accel_g.map((v) => fmt(v, 2)).join(", ")} g   gyro ${imu.gyro_deg_s.map((v) => fmt(v, 1)).join(", ")} deg/s`
    : "no IMU readings";
  const still = $("imu-still");
  still.textContent = vesc.still == null ? "-" : vesc.still ? "still" : "moving";
  still.className = "pill " + (vesc.still ? "ok" : "bad");
  $("imu-flat").textContent = state.imu_steps.flat ? "✓" : "";
  $("imu-nose_up").textContent = state.imu_steps.nose_up ? "✓" : "";
  $("imu-left_up").textContent = state.imu_steps.left_up_agrees ? "✓ agrees" : "";
  $("imu-x").textContent = draft.imu.x;
  $("imu-y").textContent = draft.imu.y;
  $("imu-z").textContent = draft.imu.z;

  // Lidar.
  $("lidar-baseline").textContent = state.lidar_baseline ? "✓" : "";
  $("lidar-upside").textContent = draft.lidar.upside_down ? "yes" : "no";

  // Steering.
  $("servo-value").textContent = fmt(vesc.servo);
  for (const mark of ["left", "straight", "right"]) {
    const value = state.steering_marks[mark];
    $(`mark-${mark}`).textContent = value == null ? "" : `(${fmt(value)})`;
  }
  $("steering-table").textContent = draft.steering.points
    .map((p) => `servo ${fmt(p.servo)} → ${fmt(deg(p.angle_rad), 1)}°`)
    .join(",  ");

  // Motor.
  $("erpm-live").textContent = vesc.erpm == null ? "-" : `${Math.round(vesc.erpm)} ERPM`;
  $("counted").textContent = vesc.counted_steps == null ? "- (zero the counter)" : fmt(Math.abs(vesc.counted_steps) / 6, 1);
  $("gain").textContent = fmt(draft.motor.speed_to_erpm_gain, 0);
  $("ramp-erpms").textContent = state.limits.ramp_erpm.join(", ");
  $("ramp-seconds").textContent = Math.round(state.limits.ramp_erpm.length * 1.8);
  renderRamp(state.ramp);
  $("compensation").textContent = fmt(draft.motor.speed_compensation, 3);
  $("min-speed").textContent = fmt(draft.motor.min_speed_mps, 2);

  // Floor.
  renderFloor(state.floor, draft);

  // Review.
  renderChanges(car.changes);
  const carNameBox = $("write-car-name");
  if (!carNameBox.dataset.touched) carNameBox.checked = !state.car_name_file || state.car_name_file === car.name;
  $("saved").textContent = car.saved_to ? `Saved to ${car.saved_to}.` : "";
}

function renderFloor(floor, draft) {
  $("ahead").textContent = floor.ahead_m == null ? "no wall seen" : `${fmt(floor.ahead_m, 2)} m`;
  const drive = floor.drive;
  const perMeter = (draft.motor.speed_to_erpm_gain / 60) * 6;
  $("drive-status").textContent = drive
    ? `Last drive: ${drive.test.kind === "straight" ? "straight" : "arc " + (drive.test.index + 1)}, ` +
      `${fmt((drive.driven_steps ?? 0) / perMeter, 2)} m` +
      (drive.stop_reason ? ` - ${drive.stop_reason}` : " - driving...")
    : "";
  const result = floor.straight_result;
  $("straight-result").hidden = !result;
  if (result) {
    $("sr-distance").textContent = fmt(result.distance_m, 2);
    $("sr-gain").textContent = fmt(result.speed_to_erpm_gain, 0);
    $("sr-gain-was").textContent = fmt(draft.motor.speed_to_erpm_gain, 0);
    const k = result.curvature_per_m;
    $("sr-curve").textContent =
      Math.abs(k) < 0.01 ? "hardly at all" : `${k > 0 ? "right" : "left"}, radius ${fmt(1 / Math.abs(k), 1)} m`;
    $("sr-straight").textContent = fmt(result.straight_servo);
    const straight = draft.steering.points.find((p) => p.angle_rad === 0);
    $("sr-straight-was").textContent = straight ? fmt(straight.servo) : "-";
  }

  const body = $("arcs").querySelector("tbody");
  const key = floor.arcs.map((arc) => arc.servo).join(",");
  if (body.dataset.key !== key) {
    body.dataset.key = key;
    body.replaceChildren(
      ...floor.arcs.map((arc, index) => {
        const row = document.createElement("tr");
        const hold = document.createElement("button");
        hold.type = "button";
        hold.className = "hold";
        hold.textContent = "HOLD to drive";
        holdToRun(hold, () => ({ kind: "arc", index }), "floor-check");
        const analyze = document.createElement("button");
        analyze.type = "button";
        analyze.textContent = "Work it out";
        analyze.onclick = () => step("/api/floor/arc", { index });
        const cells = [
          `${arc.side} ${Math.round(arc.fraction * 100)}%`,
          fmt(arc.servo),
          hold,
          analyze,
          "",
          "",
        ];
        for (const content of cells) {
          const cell = document.createElement("td");
          cell.append(content);
          row.append(cell);
        }
        return row;
      }),
    );
  }
  floor.arcs.forEach((arc, index) => {
    const cells = body.children[index].children;
    const angle = arc.angle_rad;
    cells[4].textContent = angle == null ? "-" : `${fmt(deg(angle), 1)}°`;
    cells[5].textContent =
      angle == null || Math.abs(angle) < 1e-3 ? "-" : `${fmt(floor.wheelbase_m / Math.tan(Math.abs(angle)), 2)} m`;
  });
  $("steering-table-floor").textContent = $("steering-table").textContent;
}

function renderRamp(ramp) {
  const body = $("ramp-table").querySelector("tbody");
  const rows = ramp.steps.map((s) => {
    const smooth = s.measured_erpm >= 0.6 * s.commanded_erpm && s.measured_std_erpm <= 0.1 * s.commanded_erpm;
    const row = document.createElement("tr");
    row.className = smooth ? "" : "rough";
    for (const text of [s.commanded_erpm, Math.round(s.measured_erpm), Math.round(s.measured_std_erpm), smooth ? "smooth" : "rough"]) {
      const cell = document.createElement("td");
      cell.className = "num";
      cell.textContent = text;
      row.append(cell);
    }
    return row;
  });
  if (body.childElementCount !== rows.length) body.replaceChildren(...rows);
  $("ramp-status").textContent = ramp.finished
    ? "Ramp complete."
    : ramp.aborted
      ? `Ramp stopped (${ramp.aborted}) - hold again to restart it.`
      : ramp.steps.length
        ? "Running..."
        : "";
  $("ramp-apply-btn").disabled = !ramp.finished;
}

function renderChanges(changes) {
  const body = $("changes").querySelector("tbody");
  const show = (value) =>
    Array.isArray(value)
      ? value.map((p) => (p.servo == null ? JSON.stringify(p) : `${fmt(p.servo)}→${fmt(deg(p.angle_rad), 1)}°`)).join(" ")
      : typeof value === "number"
        ? String(Math.round(value * 10000) / 10000)
        : value == null
          ? "-"
          : String(value);
  const key = JSON.stringify(changes);
  if (body.dataset.key === key) return;
  body.dataset.key = key;
  body.replaceChildren(
    ...changes
      .filter((c) => c.field !== "calibrated_at")
      .map((c) => {
        const row = document.createElement("tr");
        for (const text of [c.field, show(c.old), show(c.new)]) {
          const cell = document.createElement("td");
          cell.textContent = text;
          row.append(cell);
        }
        return row;
      }),
  );
  $("changes-empty").hidden = body.childElementCount > 0;
}

/** Fills every form from the draft - once per chosen car, so typing is never overwritten. */
function fillForms(draft) {
  const g = draft.geometry;
  const values = {
    wheelbase_m: g.wheelbase_m,
    track_width_m: g.track_width_m,
    body_length_m: g.body_length_m,
    body_width_m: g.body_width_m,
    rear_axle_to_cg_m: g.rear_axle_to_cg_m,
    mass_kg: g.mass_kg,
    lidar_x_from_rear_axle_m: draft.lidar.x_from_rear_axle_m,
    lidar_y_m: draft.lidar.y_m,
    front_axle_kg: "",
    rear_axle_kg: "",
  };
  for (const [name, value] of Object.entries(values)) $(`g-${name}`).value = value;
  const points = draft.steering.points;
  const ends = [points[0], points[points.length - 1]];
  const left = ends.find((p) => p.angle_rad < 0);
  const right = ends.find((p) => p.angle_rad > 0);
  $("lock-left").value = left ? fmt(-deg(left.angle_rad), 1) : "";
  $("lock-right").value = right ? fmt(deg(right.angle_rad), 1) : "";
  const straight = points.find((p) => p.angle_rad === 0);
  if (straight) $("servo").value = straight.servo;
  $("floor-speed").value = state.floor.speed_mps;
  $("floor-stop").value = state.floor.stop_m;
}

// ---------------------------------------------------------------------
// Actions
// ---------------------------------------------------------------------

$("car-btn").onclick = () => {
  filledFor = null;
  step("/api/car", { name: $("car-name").value.trim() });
};

$("geometry-btn").onclick = () => {
  const number = (name) => {
    const text = $(`g-${name}`).value.trim();
    return text === "" ? null : Number(text);
  };
  const body = Object.fromEntries(
    [
      "wheelbase_m",
      "track_width_m",
      "body_length_m",
      "body_width_m",
      "front_axle_kg",
      "rear_axle_kg",
      "rear_axle_to_cg_m",
      "mass_kg",
      "lidar_x_from_rear_axle_m",
      "lidar_y_m",
    ].map((name) => [name, number(name)]),
  );
  // Weighed: the axle loads decide, not the fields they replace.
  if (body.front_axle_kg != null || body.rear_axle_kg != null) {
    body.rear_axle_to_cg_m = null;
    body.mass_kg = null;
  }
  step("/api/geometry", body, "Measurements taken.");
};

for (const button of document.querySelectorAll("[data-imu]")) {
  button.onclick = () => step("/api/imu/capture", { which: button.dataset.imu }, "Captured.");
}
for (const button of document.querySelectorAll("[data-lidar]")) {
  button.onclick = () => step("/api/lidar/capture", { which: button.dataset.lidar }, "Captured.");
}

// The servo follows the slider, a request at most every 80 ms.
let servoTimer = null;
function sendServo() {
  if (servoTimer) return;
  servoTimer = setTimeout(() => {
    servoTimer = null;
    api("/api/servo", { position: Number($("servo").value) }).then(render, (err) => showMessage(err.message));
  }, 80);
}
$("servo").oninput = sendServo;
for (const button of document.querySelectorAll("[data-nudge]")) {
  button.onclick = () => {
    const slider = $("servo");
    slider.value = Math.min(1, Math.max(0, Number(slider.value) + Number(button.dataset.nudge))).toFixed(3);
    sendServo();
  };
}
for (const button of document.querySelectorAll("[data-mark]")) {
  button.onclick = () => step("/api/steering/mark", { which: button.dataset.mark });
}
$("steering-btn").onclick = () =>
  step(
    "/api/steering/table",
    { left_deg: Number($("lock-left").value), right_deg: Number($("lock-right").value) },
    "Steering range set.",
  );

for (const button of document.querySelectorAll("[data-direction]")) {
  button.onclick = () => step("/api/motor/direction", { forward: button.dataset.direction === "true" }, "Direction confirmed.");
}
$("counter-btn").onclick = () => step("/api/motor/counter_zero", {}, "Counter zeroed - now spin the wheel and count.");
$("gain-btn").onclick = () =>
  step(
    "/api/motor/gain",
    { wheel_turns: Number($("wheel-turns").value), wheel_diameter_m: Number($("wheel-diameter").value) },
    "Speed per ERPM set.",
  );
$("ramp-apply-btn").onclick = () => step("/api/motor/apply_ramp", {}, "Minimum speed and compensation set.");

$("floor-settings-btn").onclick = () =>
  step(
    "/api/floor/settings",
    { speed_mps: Number($("floor-speed").value), stop_m: Number($("floor-stop").value) },
    "Set.",
  );
$("straight-analyze-btn").onclick = () => step("/api/floor/straight", {});
$("straight-apply-btn").onclick = () => step("/api/floor/apply_straight", {}, "Speed per ERPM and straight set.");
$("arcs-apply-btn").onclick = () => step("/api/floor/apply_arcs", {}, "Steering table set from the arcs.");

$("write-car-name").onchange = (event) => {
  event.target.dataset.touched = "1";
};
$("save-btn").onclick = () => step("/api/save", { write_car_name: $("write-car-name").checked }, "Saved.");

// ---------------------------------------------------------------------
// Hold to run, and stopping
// ---------------------------------------------------------------------

const releases = [];

function stopMotor() {
  api("/api/motor/stop", {}).then(render, () => {});
}

/**
 * Makes `button` hold-to-run: `request()` is what the motor is asked, and
 * only while the checkbox `gate` is ticked.
 */
function holdToRun(button, request, gate = "off-ground-check") {
  let timer = null;
  const send = (start) =>
    api("/api/motor/hold", { ...request(), start }).then(render, (err) => {
      release();
      // The ramp or a drive finishing while held isn't a problem - the
      // page says why it stopped.
      if (!err.message.startsWith("stopped - press again")) showMessage(err.message);
    });
  const press = (event) => {
    event.preventDefault();
    if (timer || !$(gate).checked) return;
    showMessage(null);
    button.classList.add("holding");
    button.setPointerCapture?.(event.pointerId);
    send(true);
    timer = setInterval(() => send(false), HOLD_EVERY_MS);
  };
  const release = () => {
    if (!timer) return;
    clearInterval(timer);
    timer = null;
    button.classList.remove("holding");
    stopMotor();
  };
  button.addEventListener("pointerdown", press);
  for (const kind of ["pointerup", "pointercancel", "lostpointercapture"]) button.addEventListener(kind, release);
  button.addEventListener("contextmenu", (event) => event.preventDefault());
  releases.push(release);
}

const spinErpm = () => Math.round(Number($("spin-erpm").value));
holdToRun($("spin-btn"), () => ({ kind: "spin", erpm: spinErpm() }));
holdToRun($("count-spin-btn"), () => ({ kind: "spin", erpm: spinErpm() }));
holdToRun($("ramp-btn"), () => ({ kind: "ramp" }));
holdToRun($("straight-btn"), () => ({ kind: "straight" }), "floor-check");

const releaseAll = () => releases.forEach((release) => release());
$("stop-btn").onclick = () => {
  releaseAll();
  stopMotor();
};
window.addEventListener("blur", releaseAll);
document.addEventListener("visibilitychange", () => {
  if (document.hidden) releaseAll();
});

// The steps that move the car stay locked until it's on a stand - or, for
// the floor tests, on the floor: never both at once.
function lockSteps(changed) {
  const bench = $("off-ground-check");
  const floor = $("floor-check");
  if (changed === bench && bench.checked) floor.checked = false;
  if (changed === floor && floor.checked) bench.checked = false;
  for (const section of document.querySelectorAll(".bench")) section.classList.toggle("locked", !bench.checked);
  for (const section of document.querySelectorAll(".floor")) section.classList.toggle("locked", !floor.checked);
  releaseAll();
}
$("off-ground-check").onchange = (event) => lockSteps(event.target);
$("floor-check").onchange = (event) => lockSteps(event.target);
lockSteps(null);

poll();
