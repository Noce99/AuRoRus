# Vehicle models

This module exists to answer a concrete question: *how do we advance a
simulated vehicle's state forward in time, given basic control inputs and
known geometry, without a real vehicle or a full physics engine?* It is a
growing collection of physical models of increasing complexity: a kinematic
bicycle model, a dynamic model that adds a linear tire model, a further
model that adds tire saturation/load transfer/combined slip via a
simplified Pacejka-style curve, and one more that upgrades that curve to the
full Pacejka Magic Formula - with more expected to join them over time as
each model's limitations start to matter for a given use case. Which model
actually drives the simulation at runtime is chosen in
[`crate::actuators::simulated_vehicle::VehicleModel`] - see that module's
doc comment - and selectable live from `web_gui`.

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
  "friction circle"), and no aerodynamic or rolling-resistance forces - see
  the nonlinear bicycle model below for one that adds the first two.

## Nonlinear bicycle model (tire saturation, load transfer, combined slip)

Implemented in [`nonlinear_bicycle.rs`](nonlinear_bicycle.rs). Builds on the
dynamic bicycle model above, replacing its linear tire model with a
saturating, load-dependent one, and coupling longitudinal and lateral force
through a shared per-axle grip budget.

### State ([`NonlinearBicycleState`](nonlinear_bicycle.rs))

Identical fields to [`DynamicState`](dynamic_bicycle.rs) - `x_m`, `y_m`,
`heading_rad`, `vx_mps`, `vy_mps`, `yaw_rate_rad_s` - kept as its own type
rather than reused, matching this module's convention that every model owns
its full state independently.

### Params ([`NonlinearTireParams`](nonlinear_bicycle.rs))

- `mass_kg`, `yaw_inertia_kgm2`, `lf_m`, `lr_m` — same meaning as
  [`DynamicParams`](dynamic_bicycle.rs).
- `cg_height_m` — height of the CG above the ground, in meters. Drives how
  much longitudinal acceleration shifts load between the front and rear
  axles.
- `tire_mu` — peak tire/road friction coefficient, shared front and rear (a
  deliberate simplification - see Limitations).
- `pacejka_b`, `pacejka_c` — stiffness and shape factors of the simplified
  Pacejka lateral force curve (see Equations), also shared front and rear.
- `front_drive_fraction` — fraction (`0.0..=1.0`) of the commanded
  longitudinal force delivered through the front axle; the rest goes to the
  rear. `0.0` is pure rear-wheel drive, `1.0` pure front-wheel drive,
  anything in between an all-wheel-drive split. As with the other geometry
  fields, there is deliberately no `Default`.

### Control inputs

Same as the other two models: `steering_angle_rad` and `acceleration_mps2`,
supplied externally on every call to `step`.

### Equations

```text
// Quasi-static longitudinal load transfer (front/rear only - this is still
// a single-track model, so there is no left/right split to transfer across).
static_fz_f = mass_kg*g*lr_m/(lf_m+lr_m)
static_fz_r = mass_kg*g*lf_m/(lf_m+lr_m)
transfer    = mass_kg*acceleration_mps2*cg_height_m/(lf_m+lr_m)
Fz_f = max(static_fz_f - transfer, 0)   // clamped: load can't go negative
Fz_r = max(static_fz_r + transfer, 0)

// Slip angles - same as the dynamic bicycle model.
alpha_f = atan2(vy_mps + lf_m*yaw_rate_rad_s, vx_mps) - steering_angle_rad
alpha_r = atan2(vy_mps - lr_m*yaw_rate_rad_s, vx_mps)

// Simplified-Pacejka lateral force: saturating and load-dependent, instead
// of growing without bound like the linear tire model's Fy = -c*alpha.
Fyf_raw = -tire_mu*Fz_f * sin(pacejka_c * atan(pacejka_b * alpha_f))
Fyr_raw = -tire_mu*Fz_r * sin(pacejka_c * atan(pacejka_b * alpha_r))

// Commanded longitudinal force, split by front_drive_fraction, each axle
// clamped to what its current normal load can support, then a friction
// ellipse derates that axle's lateral force by how much of its longitudinal
// budget is in use. An axle with zero load has zero grip in any direction.
Fx_total = mass_kg * acceleration_mps2
Fx_f = clamp(front_drive_fraction * Fx_total, -tire_mu*Fz_f, tire_mu*Fz_f)
Fx_r = clamp((1-front_drive_fraction) * Fx_total, -tire_mu*Fz_r, tire_mu*Fz_r)
remaining_f = sqrt(max(1 - (Fx_f/(tire_mu*Fz_f))^2, 0))
remaining_r = sqrt(max(1 - (Fx_r/(tire_mu*Fz_r))^2, 0))
Fyf = Fyf_raw * remaining_f
Fyr = Fyr_raw * remaining_r
ax_achieved = (Fx_f + Fx_r) / mass_kg

dx/dt        = vx_mps*cos(heading_rad) - vy_mps*sin(heading_rad)
dy/dt        = vx_mps*sin(heading_rad) + vy_mps*cos(heading_rad)
dheading/dt  = yaw_rate_rad_s
dvx/dt       = ax_achieved + vy_mps*yaw_rate_rad_s
dvy/dt       = (Fyf*cos(steering_angle_rad) + Fyr)/mass_kg - vx_mps*yaw_rate_rad_s
dyaw_rate/dt = (lf_m*Fyf*cos(steering_angle_rad) - lr_m*Fyr)/yaw_inertia_kgm2
```

`sin(C*atan(B*alpha))` is the simplified Pacejka curve: the same smooth,
naturally-bounded family as the industry-standard "Magic Formula" tire
model, but with only two shape parameters (`B`, `C`) rather than the full
4-6-coefficient version - it's linear near `alpha = 0` (initial slope
`B*C`) and saturates toward `±1` (so lateral force saturates toward
`±tire_mu*Fz`) as `alpha` grows, with no singularity for any input.

The friction-ellipse step (`remaining_f`/`remaining_r`) is what makes this a
*combined*-slip model: using more of an axle's grip for acceleration or
braking leaves correspondingly less available for cornering, and vice
versa - the effect that produces understeer under power or a slide under
heavy braking, which the dynamic bicycle model's fully decoupled
longitudinal/lateral forces can't reproduce.

Integrated with the same RK4 scheme as the other two models (control frozen
across the step, `heading_rad` wrapped to `(-pi, pi]` at the end).

### Limitations

- Load transfer is quasi-static (algebraic in the current tick's
  acceleration) - there's no suspension mass/damping dynamics, so it
  responds instantly rather than with the roll/pitch transient a real
  chassis has.
- Front and rear tires share one `tire_mu`/`pacejka_b`/`pacejka_c` - real
  vehicles often run different compounds, or the same tire simply behaves
  differently front vs rear under different loads. See the Pacejka bicycle
  model below for one that tunes each axle independently.
- Still a single-track (no left/right) model, so only longitudinal
  (front/rear) load transfer is modeled - there's no lateral load transfer
  or per-wheel asymmetry, which a full four-wheel model would add.
- The simplified two-parameter Pacejka curve is less general than the full
  Magic Formula (no curvature factor, so it's symmetric about `alpha = 0` in
  a way a real tire curve often isn't) - see the Pacejka bicycle model below
  for one that adds it.

## Pacejka bicycle model (full Magic Formula)

Implemented in [`pacejka_bicycle.rs`](pacejka_bicycle.rs). Builds on the
nonlinear bicycle model above, replacing its simplified tire curve with the
full ("similarity") Pacejka Magic Formula, tuned independently per axle, and
replacing its friction-ellipse combined slip with Pacejka's own weighting-
function shape. Longitudinal force/load-transfer handling is otherwise
unchanged from the nonlinear model.

### State ([`PacejkaBicycleState`](pacejka_bicycle.rs))

Identical fields to [`NonlinearBicycleState`](nonlinear_bicycle.rs) - `x_m`,
`y_m`, `heading_rad`, `vx_mps`, `vy_mps`, `yaw_rate_rad_s` - kept as its own
type per this module's convention that every model owns its full state
independently.

### Params ([`PacejkaTireParams`](pacejka_bicycle.rs))

- `mass_kg`, `yaw_inertia_kgm2`, `lf_m`, `lr_m`, `cg_height_m` — same meaning
  and role (quasi-static load transfer) as
  [`NonlinearTireParams`](nonlinear_bicycle.rs).
- `front_b`, `front_c`, `front_d_mu`, `front_e` and `rear_b`, `rear_c`,
  `rear_d_mu`, `rear_e` — the front/rear Magic Formula stiffness, shape,
  peak-friction, and curvature factors. `*_d_mu` plays the same role as the
  nonlinear model's single `tire_mu`, but tunable per axle; `*_e` is new -
  the nonlinear model's curve is what you get at `*_e = 0.0`.
- `combined_slip_b`, `combined_slip_c` — stiffness and shape factors of the
  combined-slip weighting-function curve (see Equations), shared front and
  rear - the same simplification choice the nonlinear model already made for
  its own shared parameters.
- `front_drive_fraction` — same meaning as
  [`NonlinearTireParams`](nonlinear_bicycle.rs)'s field. As with the other
  geometry fields, there is deliberately no `Default`.

### Control inputs

Same as the other three models: `steering_angle_rad` and
`acceleration_mps2`, supplied externally on every call to `step`.

### Equations

```text
// Load transfer and slip angles - identical to the nonlinear bicycle model.
Fz_f, Fz_r = quasi-static front/rear normal load from mass_kg, cg_height_m,
             lf_m, lr_m, and acceleration_mps2 (clamped to >= 0)
alpha_f = atan2(vy_mps + lf_m*yaw_rate_rad_s, vx_mps) - steering_angle_rad
alpha_r = atan2(vy_mps - lr_m*yaw_rate_rad_s, vx_mps)

// Full ("similarity") Magic Formula per axle - D is the axle's peak force,
// scaled by its current normal load; E adds curvature the simplified curve
// (front_e = rear_e = 0 reduces exactly to it) doesn't have.
Dx_f = front_d_mu * Fz_f
Dx_r = rear_d_mu  * Fz_r
Fyf_raw = -Dx_f * sin(front_c * atan(front_b*alpha_f - front_e*(front_b*alpha_f - atan(front_b*alpha_f))))
Fyr_raw = -Dx_r * sin(rear_c  * atan(rear_b*alpha_r  - rear_e *(rear_b*alpha_r  - atan(rear_b*alpha_r))))

// Longitudinal force, split and clamped exactly as in the nonlinear model.
Fx_total = mass_kg * acceleration_mps2
Fx_f = clamp(front_drive_fraction * Fx_total, -Dx_f, Dx_f)
Fx_r = clamp((1-front_drive_fraction) * Fx_total, -Dx_r, Dx_r)
ax_achieved = (Fx_f + Fx_r) / mass_kg

// Combined slip: Pacejka's own weighting-function shape in place of the
// nonlinear model's friction ellipse, driven by each axle's force-usage
// ratio (not a true slip ratio - see Limitations).
Gyk_f = cos(combined_slip_c * atan(combined_slip_b * (Fx_f / Dx_f)))
Gyk_r = cos(combined_slip_c * atan(combined_slip_b * (Fx_r / Dx_r)))
Fyf = Fyf_raw * Gyk_f
Fyr = Fyr_raw * Gyk_r

dx/dt        = vx_mps*cos(heading_rad) - vy_mps*sin(heading_rad)
dy/dt        = vx_mps*sin(heading_rad) + vy_mps*cos(heading_rad)
dheading/dt  = yaw_rate_rad_s
dvx/dt       = ax_achieved + vy_mps*yaw_rate_rad_s
dvy/dt       = (Fyf*cos(steering_angle_rad) + Fyr)/mass_kg - vx_mps*yaw_rate_rad_s
dyaw_rate/dt = (lf_m*Fyf*cos(steering_angle_rad) - lr_m*Fyr)/yaw_inertia_kgm2
```

An axle with `Dx <= 0` (lifted, zero load) gets `Fx = 0` and `Fy = 0`.

Integrated with the same RK4 scheme as the other three models (control
frozen across the step, `heading_rad` wrapped to `(-pi, pi]` at the end).

### Limitations

- Same quasi-static (no suspension dynamics), single-track (no left/right),
  and no-aerodynamic-forces limitations as the nonlinear bicycle model.
- Combined slip is still driven by each axle's *force* usage ratio, not a
  true independently-evolving slip ratio from wheel rotational dynamics -
  there's still no wheelspin or lockup. Modeling that would mean adding
  per-wheel state (angular velocity), wheel inertia, and a torque-tracking
  control loop - a substantially bigger model, and a deliberate scope
  boundary for this one.
- The curvature factor `E` and the combined-slip weighting shape are both
  free-tuned rather than fit to any measured tire data, same caveat as every
  other placeholder parameter in this codebase's vehicle models.

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
