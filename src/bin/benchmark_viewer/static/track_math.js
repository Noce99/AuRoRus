"use strict";

// ---------------------------------------------------------------------
// TrackMath - the pure math under benchmark_viewer's replay and charts,
// with no DOM, so it runs (and is tested) under plain `node` too:
// interpolating a recorded trajectory at any time, finding where each lap
// starts, projecting poses onto a reference line, and turning a lap into
// profiles over the distance driven along that line.
//
// A trajectory is `/api/runs/<id>/trajectory`'s columns: parallel arrays
// `t_s`, `lap`, `x_m`, `y_m`, `heading_rad`, `speed_mps`,
// `steering_cmd_rad`, `lateral_m`, ... sorted by `t_s` - plus, for a run
// localization ran in, its estimate `est_x_m`, `est_y_m`, `est_heading_rad`
// and its `loc_ms` and `loc_quality`, `null` where it had none.
// ---------------------------------------------------------------------

const TrackMath = (() => {
  function wrapAngle(angle) {
    return angle - 2 * Math.PI * Math.floor((angle + Math.PI) / (2 * Math.PI));
  }

  /** The last index `i` of the ascending `values` with `values[i] <= x` -
   *  `-1` if there's none. */
  function lastAtOrBefore(values, x) {
    let low = 0;
    let high = values.length - 1;
    let found = -1;
    while (low <= high) {
      const mid = (low + high) >> 1;
      if (values[mid] <= x) {
        found = mid;
        low = mid + 1;
      } else {
        high = mid - 1;
      }
    }
    return found;
  }

  /** `ys` at `x`, linearly interpolated over the ascending `xs` - clamped
   *  to the ends. */
  function interpolate(xs, ys, x) {
    const i = lastAtOrBefore(xs, x);
    if (i < 0) return ys[0];
    if (i >= xs.length - 1) return ys[xs.length - 1];
    const span = xs[i + 1] - xs[i];
    const f = span > 0 ? (x - xs[i]) / span : 0;
    return ys[i] + f * (ys[i + 1] - ys[i]);
  }

  /** The vehicle at time `t` - interpolated between the two samples
   *  around it, heading the short way round - as `{x, y, heading, speed,
   *  steering, lap, ended}`; held at the first/last sample outside the
   *  recording (`ended` past its end). `null` for an empty trajectory. */
  function poseAt(trajectory, t) {
    const times = trajectory.t_s;
    const n = times.length;
    if (n === 0) return null;
    const i = Math.max(0, lastAtOrBefore(times, t));
    const j = Math.min(n - 1, i + 1);
    const span = times[j] - times[i];
    const f = j > i && span > 0 ? Math.min(1, Math.max(0, (t - times[i]) / span)) : 0;
    const lerp = (column) => column[i] + f * (column[j] - column[i]);
    return {
      x: lerp(trajectory.x_m),
      y: lerp(trajectory.y_m),
      heading: trajectory.heading_rad[i] + f * wrapAngle(trajectory.heading_rad[j] - trajectory.heading_rad[i]),
      speed: lerp(trajectory.speed_mps),
      steering: lerp(trajectory.steering_cmd_rad),
      lap: trajectory.lap[i],
      ended: t > times[n - 1],
    };
  }

  /** When each lap starts: `starts[k]` is the time of the first sample of
   *  lap `k` (`k >= 1`), `undefined` for a lap never reached. */
  function lapStarts(trajectory) {
    const starts = [];
    trajectory.lap.forEach((lap, i) => {
      if (lap >= 1 && starts[lap] === undefined) starts[lap] = trajectory.t_s[i];
    });
    return starts;
  }

  /** A closed polyline (`[[x, y, ...], ...]`) to measure distances along. */
  function referenceLine(points) {
    const n = points.length;
    const cumulative = [0];
    for (let i = 0; i < n; i++) {
      const [ax, ay] = points[i];
      const [bx, by] = points[(i + 1) % n];
      cumulative.push(cumulative[i] + Math.hypot(bx - ax, by - ay));
    }
    return { points, cumulative, length: cumulative[n] };
  }

  /** Where `(x, y)` projects onto `line`, as `{s, segment}` - onto every
   *  segment if `hint` is null, else only onto those within `windowM` of
   *  arc length around segment `hint`, so the projection never jumps to
   *  another stretch of track running close by. */
  function project(line, x, y, hint = null, windowM = 3) {
    const n = line.points.length;
    let segments;
    if (hint === null) {
      segments = [...Array(n).keys()];
    } else {
      segments = [hint];
      for (const direction of [1, -1]) {
        let travelled = 0;
        let segment = hint;
        while (travelled < windowM && segments.length < n) {
          segment = (segment + direction + n) % n;
          if (segments.includes(segment)) break;
          segments.push(segment);
          travelled += line.cumulative[segment + 1] - line.cumulative[segment];
        }
      }
    }
    let best = null;
    for (const segment of segments) {
      const [ax, ay] = line.points[segment];
      const [bx, by] = line.points[(segment + 1) % n];
      const dx = bx - ax;
      const dy = by - ay;
      const lengthSq = dx * dx + dy * dy;
      const f = lengthSq > 0 ? Math.min(1, Math.max(0, ((x - ax) * dx + (y - ay) * dy) / lengthSq)) : 0;
      const distance = Math.hypot(ax + f * dx - x, ay + f * dy - y);
      if (!best || distance < best.distance) {
        best = { segment, distance, s: line.cumulative[segment] + f * Math.sqrt(lengthSq) };
      }
    }
    return { s: best.s, segment: best.segment };
  }

  /** Whether localization ran in `trajectory`'s run. */
  function hasEstimate(trajectory) {
    return Array.isArray(trajectory.est_x_m);
  }

  /** How far localization's estimate was off at sample `i`, as
   *  `{position, heading}` (meters, radians) - `null` without an estimate
   *  there. */
  function estimateError(trajectory, i) {
    if (!hasEstimate(trajectory)) return null;
    const x = trajectory.est_x_m[i];
    const y = trajectory.est_y_m[i];
    const heading = trajectory.est_heading_rad[i];
    if (x == null || y == null || heading == null) return null;
    return {
      position: Math.hypot(x - trajectory.x_m[i], y - trajectory.y_m[i]),
      heading: wrapAngle(heading - trajectory.heading_rad[i]),
    };
  }

  /** Localization's estimate at time `t`, as `{x, y, heading}` - the
   *  latest sample at or before `t` (an estimate is never interpolated
   *  across a gap) - or `null` without one there. */
  function estimateAt(trajectory, t) {
    if (!hasEstimate(trajectory)) return null;
    const i = lastAtOrBefore(trajectory.t_s, t);
    if (i < 0 || trajectory.est_x_m[i] == null) return null;
    return { x: trajectory.est_x_m[i], y: trajectory.est_y_m[i], heading: trajectory.est_heading_rad[i] };
  }

  /** The `p`-quantile (0..1) of `values`, nearest rank - `null` if empty. */
  function quantile(values, p) {
    if (values.length === 0) return null;
    const sorted = [...values].sort((a, b) => a - b);
    return sorted[Math.min(sorted.length - 1, Math.max(0, Math.ceil(p * sorted.length) - 1))];
  }

  /** How localization did over lap `lap` of `trajectory` - every lap
   *  driven (`lap >= 1`) when `lap` is null: `{samples, coverage,
   *  meanM, rmsM, maxM, maxHeadingRad, meanMs, p95Ms}`, with `coverage` the
   *  fraction of the samples that had an estimate. `null` if localization
   *  never ran, or the lap was never driven. */
  function localizationStats(trajectory, lap = null) {
    if (!hasEstimate(trajectory)) return null;
    const errors = [];
    const headings = [];
    const ms = [];
    let samples = 0;
    for (let i = 0; i < trajectory.t_s.length; i++) {
      const sampleLap = trajectory.lap[i];
      if (lap === null ? sampleLap < 1 : sampleLap !== lap) continue;
      samples += 1;
      const error = estimateError(trajectory, i);
      if (error) {
        errors.push(error.position);
        headings.push(Math.abs(error.heading));
      }
      const time = trajectory.loc_ms?.[i];
      if (time != null) ms.push(time);
    }
    if (samples === 0) return null;
    const mean = (values) => (values.length ? values.reduce((a, b) => a + b, 0) / values.length : null);
    return {
      samples,
      coverage: errors.length / samples,
      meanM: mean(errors),
      rmsM: errors.length ? Math.sqrt(mean(errors.map((e) => e * e))) : null,
      maxM: errors.length ? Math.max(...errors) : null,
      maxHeadingRad: headings.length ? Math.max(...headings) : null,
      meanMs: mean(ms),
      p95Ms: quantile(ms, 0.95),
    };
  }

  /** Lap `lap` of `trajectory` over the distance driven along `line`:
   *  `d` (meters from where the lap started, unwrapped across the line's
   *  own start) and, per sample, `t` (seconds into the lap), `speed`,
   *  `lateral`, `steering`, `x`, `y` - and localization's `posErr`
   *  (meters), `headErr` (radians, signed), `locMs` and `locQuality`,
   *  `null` where it had none. Empty arrays if the lap was never driven. */
  function lapProfile(trajectory, lap, line) {
    const profile = {
      d: [], t: [], speed: [], lateral: [], steering: [], x: [], y: [],
      posErr: [], headErr: [], locMs: [], locQuality: [],
    };
    let hint = null;
    let startS = 0;
    let lastS = 0;
    let d = 0;
    let startT = 0;
    for (let i = 0; i < trajectory.t_s.length; i++) {
      if (trajectory.lap[i] !== lap) continue;
      const x = trajectory.x_m[i];
      const y = trajectory.y_m[i];
      const projection = project(line, x, y, hint);
      hint = projection.segment;
      if (profile.d.length === 0) {
        startS = projection.s;
        lastS = startS;
        startT = trajectory.t_s[i];
      } else {
        // Across the line's own start the raw `s` wraps: take the short way.
        let step = projection.s - lastS;
        if (step > line.length / 2) step -= line.length;
        if (step < -line.length / 2) step += line.length;
        d += step;
        lastS = projection.s;
      }
      profile.d.push(d);
      profile.t.push(trajectory.t_s[i] - startT);
      profile.speed.push(trajectory.speed_mps[i]);
      profile.lateral.push(trajectory.lateral_m[i]);
      profile.steering.push(trajectory.steering_cmd_rad[i]);
      profile.x.push(x);
      profile.y.push(y);
      const error = estimateError(trajectory, i);
      profile.posErr.push(error ? error.position : null);
      profile.headErr.push(error ? error.heading : null);
      profile.locMs.push(trajectory.loc_ms?.[i] ?? null);
      profile.locQuality.push(trajectory.loc_quality?.[i] ?? null);
    }
    return profile;
  }

  /** `values` made non-decreasing - a lap's distance steps back a few
   *  centimeters now and then (a wobble across the line), which would
   *  otherwise break interpolating over it. */
  function monotone(values) {
    const out = [];
    let max = -Infinity;
    for (const value of values) {
      max = Math.max(max, value);
      out.push(max);
    }
    return out;
  }

  /** `profile`'s time over its distance as a strictly increasing map:
   *  only the samples that moved it further along, so every distance has
   *  exactly one time. */
  function timeByDistance(profile) {
    const d = [];
    const t = [];
    profile.d.forEach((distance, i) => {
      if (d.length === 0 || distance > d[d.length - 1]) {
        d.push(distance);
        t.push(profile.t[i]);
      }
    });
    return { d, t };
  }

  /** How far behind `reference` `profile` is at each of its samples, in
   *  seconds - both over the same distance, positive when slower. `null`
   *  past the end of the reference's lap. Both are read through
   *  `timeByDistance`, so a run against itself is exactly 0 everywhere. */
  function deltaTime(profile, reference) {
    const own = timeByDistance(profile);
    const other = timeByDistance(reference);
    const end = other.d[other.d.length - 1];
    return monotone(profile.d).map((d) =>
      d > end ? null : interpolate(own.d, own.t, d) - interpolate(other.d, other.t, d),
    );
  }

  /** `profile`'s `column` at distance `d` - `null` outside the lap. */
  function atDistance(profile, column, d) {
    const distances = monotone(profile.d);
    if (distances.length === 0 || d < distances[0] || d > distances[distances.length - 1]) return null;
    const values = profile[column];
    const i = lastAtOrBefore(distances, d);
    // A gap (no value) on either side leaves nothing to interpolate.
    if (values[i] == null || values[Math.min(i + 1, values.length - 1)] == null) return values[i] ?? null;
    return interpolate(distances, values, d);
  }

  return {
    wrapAngle, lastAtOrBefore, interpolate, poseAt, lapStarts, referenceLine, project, lapProfile, monotone,
    timeByDistance, deltaTime, atDistance, hasEstimate, estimateError, estimateAt, quantile, localizationStats,
  };
})();

if (typeof module !== "undefined") module.exports = TrackMath;
