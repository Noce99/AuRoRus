# Vehicle models

This module exists to answer a concrete question: *how do we advance a
simulated vehicle's state forward in time, given basic control inputs and
known geometry, without a real vehicle or a full physics engine?* It is a
growing collection of physical models of increasing complexity: a kinematic
bicycle model, and a dynamic model that adds lateral tire forces, with more
expected to join them over time as each model's limitations start to matter
for a given use case. Which model actually drives the simulation at runtime
is chosen in [`crate::actuators::simulated_vehicle::VehicleModel`] - see that
module's doc comment - and selectable live from `web_gui`.

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

### Limitations

- Purely kinematic: the only lateral effect modeled is the geometric slip
  angle `beta`, not real tire slip from lateral force. Not valid at high
  lateral acceleration or on low-friction surfaces, where actual tire slip
  diverges from this model's prediction - see the dynamic bicycle model
  below for a model that accounts for it.
- No default vehicle geometry is provided (see [Params](#params-bicycleparams)).

## Dynamic bicycle model (tire forces)

Implemented in [`dynamic_bicycle.rs`](dynamic_bicycle.rs).

### State ([`DynamicState`](dynamic_bicycle.rs))

- `x_m`, `y_m`, `heading_rad` — same meaning as [`BicycleState`](bicycle.rs).
- `vx_mps`, `vy_mps` — body-frame longitudinal and lateral velocity of the
  CG, in meters/second.
- `yaw_rate_rad_s` — rate of change of `heading_rad`, in radians/second.

Unlike the kinematic model, forward speed isn't a single number: `vx_mps`
and `vy_mps` are tracked separately because real tire slip means the CG's
velocity direction no longer has to align with the body's heading by a fixed
geometric relationship.

### Params ([`DynamicParams`](dynamic_bicycle.rs))

- `mass_kg` — vehicle mass.
- `yaw_inertia_kgm2` — yaw moment of inertia about the vertical axis through
  the CG.
- `lf_m`, `lr_m` — same meaning as [`BicycleParams`](bicycle.rs).
- `cf_n_per_rad`, `cr_n_per_rad` — front/rear tire cornering stiffness
  (lateral force per radian of slip angle). As with `lf_m`/`lr_m`, there is
  deliberately no `Default`.

### Control inputs

Same as the kinematic bicycle model: `steering_angle_rad` and
`acceleration_mps2`, supplied externally on every call to `step`.

### Equations

```text
alpha_f = atan2(vy_mps + lf_m*yaw_rate_rad_s, vx_mps) - steering_angle_rad
alpha_r = atan2(vy_mps - lr_m*yaw_rate_rad_s, vx_mps)
Fyf     = -cf_n_per_rad * alpha_f
Fyr     = -cr_n_per_rad * alpha_r

dx/dt        = vx_mps*cos(heading_rad) - vy_mps*sin(heading_rad)
dy/dt        = vx_mps*sin(heading_rad) + vy_mps*cos(heading_rad)
dheading/dt  = yaw_rate_rad_s
dvx/dt       = acceleration_mps2 + vy_mps*yaw_rate_rad_s
dvy/dt       = (Fyf*cos(steering_angle_rad) + Fyr)/mass_kg - vx_mps*yaw_rate_rad_s
dyaw_rate/dt = (lf_m*Fyf*cos(steering_angle_rad) - lr_m*Fyr)/yaw_inertia_kgm2
```

`alpha_f`/`alpha_r` are the front/rear tire slip angles, computed from the
vehicle's actual body-frame velocity rather than assumed from geometry alone
- this is what lets the model produce a real lateral force (`Fyf`, `Fyr`) via
a linear tire model (force proportional to slip angle, via the cornering
stiffness), and so capture effects the kinematic model can't: understeer/
oversteer, and the tires' lateral force saturating at high slip (though the
*linear* tire model used here doesn't itself cap that force - see
Limitations below).

Integrated with the same RK4 scheme as the kinematic model (control frozen
across the step, `heading_rad` wrapped to `(-pi, pi]` at the end).

### Limitations

- The linear tire model has no saturation: `Fyf`/`Fyr` grow without bound as
  slip angle grows, whereas a real tire's lateral force saturates (and then
  falls off) at large slip. Valid only within the tires' linear region -
  roughly small slip angles, i.e. moderate lateral acceleration - not at the
  limit of grip or beyond it.
- Not valid near zero forward speed: `alpha_f`/`alpha_r` use `atan2` against
  `vx_mps`, so as `vx_mps` approaches zero, tiny lateral motion produces slip
  angles approaching ±90°, and the resulting forces no longer represent real
  tire behavior at a stop or in a very slow maneuver. A low-speed blend with
  the kinematic model (which has no such singularity) would be a natural
  future improvement, not implemented here.
- No load transfer, no combined longitudinal/lateral tire force limit (a
  "friction circle"), and no aerodynamic or rolling-resistance forces - all
  candidates for a further, even more complex model down the line.

## Contract with the simulation environment

This module owns no timing loop, thread, or scheduler. A simulation
environment (`crate::actuators::simulated_vehicle`) holds whichever model's
state and params are currently active, and calls that model's `step(state,
params, steering_angle_rad, acceleration_mps2, dt_s)` once per tick with
that tick's `dt_s` and the current control sample, replacing its stored
state with the result.

No actuator limits or delay are modeled — clamping `steering_angle_rad` and
`acceleration_mps2` to values the real vehicle could actually achieve is
the caller's responsibility, not this module's (see
`crate::actuators::simulated_vehicle::ActuatorLimits`).

More models are expected to land in this same directory and get their own
section here as they're added.
