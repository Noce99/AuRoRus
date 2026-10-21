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

  "localization stats measure the estimate against the truth"() {
    const run = circleRun();
    // Localization from lap 1's second sample on, always 0.1 m outward
    // and 0.05 rad ahead, taking 2 ms per scan - but on lap 2 an outlier.
    run.est_x_m = run.x_m.map((x, i) => (i === 0 ? null : x * (R + 0.1) / R));
    run.est_y_m = run.y_m.map((y, i) => (i === 0 ? null : y * (R + 0.1) / R));
    run.est_heading_rad = run.heading_rad.map((h, i) => (i === 0 ? null : T.wrapAngle(h + 0.05)));
    run.loc_ms = run.t_s.map((_, i) => (i === 0 ? null : 2));
    run.loc_quality = run.t_s.map(() => null);
    const lap2 = run.lap.indexOf(2);
    run.est_x_m[lap2] += 1;

    const lap1 = T.localizationStats(run, 1);
    near(lap1.meanM, 0.1, 1e-9, "lap 1 mean error");
    near(lap1.maxM, 0.1, 1e-9, "lap 1 max error");
    near(lap1.maxHeadingRad, 0.05, 1e-9, "lap 1 heading error");
    near(lap1.coverage, 1 - 1 / lap1.samples, 1e-12, "lap 1 coverage");
    near(lap1.meanMs, 2, 1e-12, "lap 1 time");
    assert(T.localizationStats(run, 2).maxM > 0.9, "the outlier is lap 2's");
    const all = T.localizationStats(run);
    assert.strictEqual(all.samples, run.t_s.length);
    assert(all.rmsM > all.meanM, "the outlier weighs more in the RMS");
    assert.strictEqual(T.localizationStats(run, 3), null);
    assert.strictEqual(T.localizationStats(circleRun(), 1), null);

    const profile = T.lapProfile(run, 1, circleLine());
    assert.strictEqual(profile.posErr[0], null);
    near(profile.posErr[5], 0.1, 1e-9, "profile error");
    near(profile.headErr[5], 0.05, 1e-9, "profile heading error");
    assert.strictEqual(T.estimateAt(run, 0), null);
    near(T.estimateAt(run, 0.03).x, run.est_x_m[1], 1e-12, "estimate at a time");
  },

  "quantile picks by nearest rank"() {
    assert.strictEqual(T.quantile([], 0.5), null);
    assert.strictEqual(T.quantile([5, 1, 3, 2, 4], 0.5), 3);
    assert.strictEqual(T.quantile([1, 2, 3, 4, 5, 6, 7, 8, 9, 10], 0.95), 10);
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
