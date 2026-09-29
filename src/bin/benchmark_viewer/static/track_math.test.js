"use strict";

// Tests of track_math.js, against a run whose every answer is known: a
// circle of radius R driven at a constant speed V for two laps. Plain
// `node` with no dependencies - run by `cargo test` (see the viewer's
// `track_math_js` test), or directly: `node track_math.test.js`. Not
// served to the browser.

const assert = require("assert");
const T = require("./track_math.js");

const R = 5;
const V = 2;
const DT = 0.025;
const LAP = 2 * Math.PI * R;

/** Two laps round the circle, sampled every DT like trajectory.csv. */
function circleRun() {
  const run = { t_s: [], lap: [], x_m: [], y_m: [], heading_rad: [], speed_mps: [], steering_cmd_rad: [], lateral_m: [] };
  for (let i = 0; i * DT <= (2 * LAP) / V; i++) {
    const t = i * DT;
    const angle = (V * t) / R;
    run.t_s.push(t);
    run.lap.push(Math.floor((V * t) / LAP) + 1);
    run.x_m.push(R * Math.cos(angle));
    run.y_m.push(R * Math.sin(angle));
    run.heading_rad.push(T.wrapAngle(angle + Math.PI / 2));
    run.speed_mps.push(V);
    run.steering_cmd_rad.push(0.1);
    run.lateral_m.push(0);
  }
  return run;
}

/** The same circle as a closed polyline starting 1 rad along - so its own
 *  `s = 0` is mid-lap, and a lap's distance has to unwrap across it. */
function circleLine() {
  const points = [...Array(400).keys()].map((i) => {
    const angle = 1 + (2 * Math.PI * i) / 400;
    return [R * Math.cos(angle), R * Math.sin(angle)];
  });
  return T.referenceLine(points);
}

function near(actual, expected, tolerance, what) {
  assert(Math.abs(actual - expected) <= tolerance, `${what}: ${actual}, expected ${expected} ± ${tolerance}`);
}

const tests = {
  "poseAt interpolates between samples and parks past the end"() {
    const run = circleRun();
    const pose = T.poseAt(run, 1.0125);
    near(Math.hypot(pose.x, pose.y), R, 1e-3, "distance from the center");
    near(pose.speed, V, 1e-9, "speed");
    assert(!pose.ended);
    assert(T.poseAt(run, 1e9).ended);
  },

  "poseAt turns the heading the short way round"() {
    const run = { t_s: [0, 1], lap: [1, 1], x_m: [0, 0], y_m: [0, 0], heading_rad: [3.1, -3.1], speed_mps: [0, 0], steering_cmd_rad: [0, 0] };
    near(Math.abs(T.wrapAngle(T.poseAt(run, 0.5).heading)), Math.PI, 1e-9, "heading halfway");
  },

  "lapStarts finds the first sample of each lap"() {
    const starts = T.lapStarts(circleRun());
    near(starts[1], 0, 1e-9, "lap 1");
    near(starts[2], LAP / V, DT, "lap 2");
    assert.strictEqual(starts[3], undefined);
  },

  "a lap's distance unwraps across the reference line's own start"() {
    const line = circleLine();
    near(line.length, LAP, 0.01, "reference length");
    const profile = T.lapProfile(circleRun(), 2, line);
    near(profile.d[profile.d.length - 1], LAP, 0.1, "distance over one lap");
    near(T.atDistance(profile, "t", LAP / 2), LAP / 2 / V, 0.03, "time at half a lap");
    assert.strictEqual(T.atDistance(profile, "t", 2 * LAP), null);
  },

  "a run 10 % slower is 10 % of the time behind"() {
    const profile = T.lapProfile(circleRun(), 2, circleLine());
    const slow = { ...profile, t: profile.t.map((t) => t * 1.1) };
    const delta = T.deltaTime(slow, profile);
    const half = Math.floor(delta.length / 2);
    near(delta[half], 0.1 * profile.t[half], 0.03, "Δ time halfway");
  },

  "a run against itself is exactly 0, even where its distance wobbles back"() {
    const profile = T.lapProfile(circleRun(), 2, circleLine());
    assert(T.deltaTime(profile, profile).every((delta) => delta === 0));
    const wobbly = { d: [0, 1, 2, 1.9, 3, 4], t: [0, 1, 2, 2.1, 3, 4] };
    assert.deepStrictEqual(T.deltaTime(wobbly, wobbly), [0, 0, 0, 0, 0, 0]);
  },
};

let failed = 0;
for (const [name, test] of Object.entries(tests)) {
  try {
    test();
    console.log(`ok - ${name}`);
  } catch (err) {
    failed++;
    console.log(`FAILED - ${name}\n  ${err.message}`);
  }
}
console.log(`${Object.keys(tests).length - failed} passed, ${failed} failed`);
process.exit(failed === 0 ? 0 : 1);
