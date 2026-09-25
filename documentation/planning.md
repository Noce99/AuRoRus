# Planning

How the `Planner` executor plans a race line for the selected map, and how
it's driven from `web_gui`'s **Planning** panel.

The race line is the **minimum-curvature** line through the track: the
closed curve, kept half the vehicle's width plus a safety margin away from
both walls, whose squared curvature summed over the lap is smallest. A speed
profile goes on top. It replaces UBM's CasADi/IPOPT "worm" optimizer
(`other_repos/ubm-f1tenth/web_interface/web_interface/mpc/mpc_worm_race_line.py`)
with pure Rust. The solver is OpEn's PANOC (the `optimization_engine`
crate), with hand-written gradients.

## Architecture

```mermaid
flowchart LR
    W["WebGui"] -- "planning_parameters, planning_request" --> P["Planner"]
    P -- "planning_status" --> W
    P -- "planning_status" --> M["MapServer"]
    M -- "map" --> P
    P -- "race_lines/race_line.csv" --> F[("maps/<map>/")]
    F --> M
    M -- "race_line" --> X["(any consumer)"]
    M -- "draw/MapServer" --> W
    P -- "draw/Planner (progress)" --> W
```

- **`planning_parameters`:** the parameter values `web_gui` wants, the whole
  set at once. The planner applies them whenever it's idle, so they take
  effect **before** Start is pressed.
- **`planning_request`:** a counter. Each bump asks for a race line for the
  map currently on `map`.
- **`planning_status`:** idle or computing, the current step (e.g.
  `Optimizing - iteration 3/30, moved up to 0.300 m`), every parameter with
  the value in effect, and the latest outcome. The outcome holds the file
  written or the error, the lap length and time, the maximum curvature of
  the race line and of the centerline, and how long planning took. The
  planner only writes it when something changes.
- **`race_line`:** published by `MapServer` for the selected map. It holds
  the planned race line if the map has one, its centerline otherwise, as
  closed `x,y,speed` points. `MapServer` reloads it when `planning_status`
  reports a line saved for that map.
- **Drawing:** `MapServer` draws the line over the map, colored by speed
  (blue at the slowest point, green, then red at the fastest). A planned
  race line is drawn thicker than a centerline. While computing, the
  planner draws the line it's optimizing around (thin, white) and the
  latest solution (amber).

## The pipeline

1. **Track** (`planning/track.rs`). The drivable pixels 4-connected to the
   start/finish line's midpoint. Exactly two walls must touch them: the
   outer one and one inner island. Anything else fails with a message
   saying how many walls were found. For example, an obstacle on the
   track makes three: clean the map up.
2. **Reference line.**
   - A map with a `centerline.csv` (generated maps) uses it.
   - A map without one (e.g. saved by SLAM) gets one computed from its
     walls (`planning/centerline.rs`): the zero line of
     `distance to inner wall − distance to outer wall`. Both distances come
     from an exact Euclidean distance transform. The line is traced with
     marching squares, giving a single closed curve with sub-pixel accuracy
     and none of a skeleton's side branches. It's resampled, smoothed with
     a moving average (`centerline_smoothing_window`), and saved as the
     map's `centerline.csv` too.

   Either way, the line is turned to run in the direction of travel given
   by the start/finish line, starts at its point nearest that line, and is
   resampled every `spacing_m`.
3. **Minimum curvature** (`planning/min_curvature.rs`).
   - Each point may only move sideways along the reference's normal:
     `p_i = c_i + α_i·n_i`.
   - Each offset is bounded by the free space either side of the point,
     raycast in the map: `−w_right + m ≤ α_i ≤ w_left − m`, where
     `m = vehicle_width_m/2 + safety_margin_m`. These are box constraints,
     which PANOC projects onto exactly.
   - The discrete curvature `κ_i = (d_i × s_i)/|d_i|³` (central first and
     second differences) is linearized in the offsets (a Gauss-Newton
     step). The cost `Σκ_i² + λ·Σ(α_{i+1}−α_i)²` becomes a quadratic whose
     gradient only couples neighbors, so each evaluation costs O(n).
   - Each iteration also caps every move at `max_step_m`, so the
     linearization stays accurate. The problem is then solved again around
     the new line: the iterative scheme of Heilmeier et al., which TUM's
     global race trajectory optimization uses too. It stops once no point
     moves more than `tolerance_m`, or after `iterations`.
   - Linearizing only the second difference, with the point spacing held
     fixed, is simpler but wrong. Moving points toward the inside of a
     corner shrinks their second difference, so that model takes the inside
     of every corner for the flattest line, which is a shortest path. On a
     ring it hugs the inner wall instead of the outer one.
4. **Speed profile** (`planning/speed_profile.rs`).
   - Every point gets `v = min(max_speed_mps, √(max_lateral_accel_mps2/|κ|))`.
   - Then backward (braking, `max_decel_mps2`) and forward (acceleration,
     `max_accel_mps2`) passes around the closed loop, until nothing
     changes.
   - The lateral and longitudinal limits are applied separately, not as a
     combined friction ellipse.

The race line is saved as the map's `race_lines/race_line.csv`, in the same
`x,y,speed` format as `centerline.csv` and replacing any previous one.
`s`, heading and curvature aren't stored: they follow from the points.

### About PANOC here

The curvature terms make this problem stiff: second differences, squared,
give a Hessian whose largest eigenvalue grows like `1/spacing⁴`. PANOC
typically needs thousands of iterations per solve, which is still fast
because each one is O(n). A full plan of a generated track takes about 7 s.

When PANOC hits `solver_max_iterations` without converging, OpEn returns
its iterate with the smallest fixed-point residual, which can be one of its
very first. The planner therefore fails with a clear message rather than
use that. If you see it, raise `solver_max_iterations`, or `spacing_m`.

## Driving it: the Planning panel

- **Start computation** plans for the selected map. It's disabled while
  computing and with no map selected. The status line shows the current
  step. Once done, the outcome line shows the file written and the lap's
  numbers, or why it failed.
- **Parameters:** one slider per parameter. A change is applied as soon as
  the planner is idle, so tune them before starting. **Save parameters**
  writes the values in effect into `config/planning/race_line.toml`,
  keeping its comments. **Load from file** goes back to that file's values.

## Parameters (`config/planning/race_line.toml`)

| Parameter | Meaning |
|---|---|
| `spacing_m` | Distance between the race line's points. |
| `centerline_smoothing_window` | Points averaged to smooth a centerline computed from the walls. |
| `vehicle_width_m`, `safety_margin_m` | The line keeps `vehicle_width_m/2 + safety_margin_m` from either wall. |
| `smoothness_weight` | `λ`: penalty on neighboring offsets differing. `0` is pure minimum curvature. |
| `max_step_m` | Farthest any point may move in one iteration. |
| `iterations`, `tolerance_m` | Most outer iterations, and the move below which the line has converged. |
| `solver_max_iterations` | PANOC's iteration budget per solve. |
| `max_speed_mps`, `max_lateral_accel_mps2`, `max_accel_mps2`, `max_decel_mps2` | Speed profile limits. |
| `solver_tolerance`, `poll_interval_ms` | Not tunable from the panel: PANOC's tolerance, and how often the planner polls. |

## Not done yet

- **Minimum lap time.** UBM's disabled `_problem_definition_new` optimizes
  the path and speeds together for a point mass. OpEn's augmented
  Lagrangian could solve it, warm-started from this race line.
- **A curvature limit.** Nothing stops the line from turning tighter than
  the vehicle's steering allows, apart from the track's shape.
- **A combined friction ellipse** in the speed profile.
