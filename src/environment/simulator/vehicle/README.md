# Vehicle models

This module exists to answer a concrete question: *how do we advance a
simulated vehicle's state forward in time, given basic control inputs and
known geometry, without a real vehicle or a full physics engine?* It is a
growing collection of physical models — it starts with one, the kinematic
bicycle model below, with more expected to join it over time (e.g. a
dynamic model with tire forces, once the kinematic model's limitations
start to matter).

Every model here follows the same shape: a plain, `Copy` state struct, a
params struct describing the vehicle's fixed geometry, and a pure `step`
function that advances the state by one control input and one time step.
None of them own a clock, a thread, or a loop — see
[Contract with the simulation environment](#contract-with-the-simulation-environment).

## Kinematic bicycle model

Implemented in [`bicycle.rs`](bicycle.rs).

### State ([`BicycleState`](bicycle.rs))

- `x_m`, `y_m` — position of the vehicle's center of gravity (CG) in world
  coordinates, in meters.
- `heading_rad` — heading of the vehicle body relative to the world X axis,
  in radians, kept wrapped to `(-pi, pi]`.
- `speed_mps` — forward speed along the body's heading, in meters/second.

### Params ([`BicycleParams`](bicycle.rs))

- `lf_m` — distance from the CG to the front axle, in meters.
- `lr_m` — distance from the CG to the rear axle, in meters.
- Wheelbase is `lf_m + lr_m`. There is deliberately no `Default`: this
  describes a specific vehicle's real geometry, and silently defaulting it
  would silently produce a physically wrong trajectory with no signal that
  anything is off.

### Control inputs

Supplied externally by the caller on every call to `step` — this module
does not own or store them:

- `steering_angle_rad` — front-wheel steering angle (`delta`), in radians.
- `acceleration_mps2` — longitudinal acceleration (`a`), in meters/second^2.

### Equations

```text
beta        = atan((lr_m / (lf_m + lr_m)) * tan(steering_angle_rad))
dx/dt       = speed_mps * cos(heading_rad + beta)
dy/dt       = speed_mps * sin(heading_rad + beta)
dheading/dt = (speed_mps / lr_m) * sin(beta)
dspeed/dt   = acceleration_mps2
```

`beta` is the slip angle: the angle between the vehicle's heading and its
actual CG velocity direction. It exists because this model tracks the
motion of the CG rather than the rear axle — the front wheel steers, but
the point being integrated sits somewhere between the two axles, so its
velocity direction leads the heading by `beta`. A simpler rear-axle-only
variant of the kinematic bicycle model exists (state at the rear axle
instead of the CG) and has no `beta` term at all (`dheading/dt = (v /
wheelbase) * tan(delta)` directly) — this module uses the CG-referenced
form instead, since tracking the CG is what will matter once a dynamic
(tire-force) model is added later and its state needs to be comparable.

### RK4 integration

`step` integrates one `dt_s` interval with classical 4th-order Runge-Kutta
(RK4) rather than explicit (forward) Euler: RK4's local truncation error is
`O(dt_s^5)` per step versus Euler's `O(dt_s^2)`, which lets a caller use a
larger tick period for the same accuracy. The cost is four derivative
evaluations per step instead of one — cheap here, since each evaluation is
a handful of trig calls with no allocation.

All four RK4 stages evaluate the derivative at the *same* control input
(`steering_angle_rad`, `acceleration_mps2`) — the control is "frozen" across
the whole step. This matches a tick-based simulation environment that
samples one control value per tick and expects the model to integrate
across it, rather than a continuous-time controller.

`heading_rad` is wrapped to `(-pi, pi]` after the step completes. This is
safe because the equations above only ever consume `heading_rad` through
`sin`/`cos`, which are periodic — wrapping never changes the dynamics, and
it keeps the value from growing unbounded over a long-running simulation.

## Contract with the simulation environment

This module owns no timing loop, thread, or scheduler. A (not yet built)
simulation environment is expected to hold a `BicycleState` and
`BicycleParams`, and call `step(state, params, steering_angle_rad,
acceleration_mps2, dt_s)` once per tick with that tick's `dt_s` and the
current control sample, replacing its stored state with the result.

No actuator limits or delay are modeled — clamping `steering_angle_rad` and
`acceleration_mps2` to values the real vehicle could actually achieve is
the caller's responsibility, not this module's.

## Limitations

- Purely kinematic: the only lateral effect modeled is the geometric slip
  angle `beta`, not real tire slip from lateral force. Not valid at high
  lateral acceleration or on low-friction surfaces, where actual tire slip
  diverges from this model's prediction.
- No default vehicle geometry is provided (see [Params](#params-bicycleparams)).
- More models — most likely a dynamic model with tire forces — are
  expected to land in this same directory and get their own section here
  as they're added.
