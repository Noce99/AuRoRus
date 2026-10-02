# Car calibration

Status: **v1 complete** (2026-10-01): the car file (`src/hardware.rs`,
`config/hardware/<car>.toml`, `config/car_template.toml`), `CAR_NAME`,
`web_gui --sim`/`--car`, the `vehicle_geometry` topic everything reads (the
simulation simulates the named car, else the template), and every
`car_calibration` step below, floor tests included. The car `tom` is
calibrated with it.

`car_calibration` measures every hardware parameter of a car, so the stack
can be installed on a new car (different servo, motor, VESC or lidar) and
calibrated without editing TOMLs by hand.

## Using it

```
cargo run --release --bin car_calibration        # then open http://<car>:1996
```

Calibrates the car `--car NAME` or `CAR_NAME` names, or one named on the
page. Stop `web_gui` first: both need the VESC's port. The page walks
through: car, battery cells, tape and scale measurements, IMU mounting
(flat / nose up / left side up), lidar direction (empty / object on the
left), then - on a stand, wheels off the ground - the servo range, motor
direction, speed per ERPM (counting wheel turns) and the ERPM ramp
(minimum speed and compensation), then - on the floor, 4 x 4 m clear - a drive
straight at a wall (the lidar's distance vs the tachometer: speed per ERPM;
the gyro's yaw while cruising: the straight servo position) and eight 2.5 m
arcs at a quarter, half, three quarters and full lock each way (yaw rate /
speed = curvature, `atan(wheelbase * curvature)` = the steering angle: the
steering table), and finally a review of every changed value and Save.
Floor drives also stop by themselves: at the stop distance from anything
the lidar sees within 35 degrees ahead, if the lidar goes quiet, at their
distance, or after 20 s. The motor only turns while a HOLD button is held (a
request every 100 ms; the server brakes 300 ms after the last), a lapse
never restarts it, and the big STOP button stops it at once. Code:
`src/bin/car_calibration/` - `analysis.rs` (the maths, tested), `bench.rs`
(the VESC thread), `session.rs` (the draft and the API).

## Storage

```
config/
├── car_template.toml             # tracked: the starting point of a new car
└── hardware/
    ├── tom.toml                  # tracked: the latest accepted calibration of "tom"
    ├── <other_car>.toml          # tracked: one file per calibrated car
    └── history/                  # gitignored: every older calibration, never edited
        └── tom/
            ├── 2026-09-30_15-18-02.toml
            └── ...
```

`CAR_NAME` (repo root, gitignored) holds the name of the car this machine
drives, e.g. `tom`.

- The car's name is asked on the page's first step. A new name starts from
  `config/car_template.toml` (a roughly 1/10-scale car); an existing one
  starts from its last calibration, so a partial re-calibration (e.g. only
  the steering) is just running the steps wanted and saving.
- Nothing is written until Save. Saving first moves the current
  `<car>.toml` into `history/<car>/`, named after when it was calibrated,
  and then writes the new one. Reverting a bad calibration means copying a
  history file back, and `git diff` shows exactly what a calibration
  changed.
- Save can also write `CAR_NAME` ("This machine drives this car").
- `CAR_NAME` is per machine, so the repo can hold several cars. With no
  `CAR_NAME`, binaries run in simulation.
- Each file stores `schema_version`, the car's name and when it was
  calibrated. A file with another `schema_version` is refused rather than
  misread.

## What the car file holds (single source of truth)

Physical facts only. Positive y and angles are the car's right everywhere.

| Section | Values |
|---|---|
| `[geometry]` | `wheelbase_m`, `rear_axle_to_cg_m`, `track_width_m`, `body_length_m`, `body_width_m`, `mass_kg` |
| `[steering]` | `points`: the servo position -> steering angle lookup table (see [below](#the-steering-table)) |
| `[motor]` | `speed_to_erpm_gain`, `speed_compensation`, `min_speed_mps` |
| `[battery]` | `cells` |
| `[imu]` | `x`, `y`, `z`: which of the VESC's IMU axes, and which way, is each of the car's |
| `[lidar]` | `upside_down`, `x_from_rear_axle_m`, `y_m` |

Operating policy stays in the module configs: the VESC's port, rates,
timeouts, `brake_current_a`, the low-battery voltage and the limits
(`max_speed_mps`, acceleration caps, steering rate) are in
`config/actuators/vesc.toml`.

Who reads it:

- **`Vesc`**: the steering table (angle -> servo), the motor's gain,
  compensation and minimum speed, the battery's cells, the IMU's axes.
- **`HokuyoLidar`**: the lidar's mounting.
- **The algorithms and the planner**, through the `vehicle_geometry` and
  `vehicle_limits` topics: wheelbase, rear-axle-to-CG distance, body size,
  and the largest steering angle. The algorithm configs keep only their own
  margins (e.g. `safety_margin_m` added to the calibrated width) instead of
  their own copy of the car's size.
- **The simulation**: geometry, mass, `lf/lr`, the steering limit, the
  lidar's mount. The simulated car then behaves like the real one, which
  makes tuning controllers in simulation more faithful.

## Web GUI modes

- `CAR_NAME` exists → `web_gui` starts on the hardware with that car's file.
  A bad `CAR_NAME` or car file stops it at startup rather than falling back
  to simulation.
- `--sim` → simulation even on the car (still simulating that car).
- `--car <name>` picks a car instead of the one `CAR_NAME` names.
- No `CAR_NAME` and no `--car` → simulation of the template car.
- On hardware the GUI shows the car's name in the bottom right corner and a
  different canvas color, so hardware and simulation can't be confused.
- The VESC and the lidar are retried until they appear; `web_gui` doesn't
  need them at startup.
- The VESC panel doesn't edit calibrated values; policy values (limits)
  stay editable there.

## The guided process

A separate binary with its own web UI, used from a phone or laptop standing
next to the car. It drives the VESC and reads the lidar directly, with none
of the stack's executors. Every step that turns the motor is
**hold-to-run** (see [Using it](#using-it)), and each moving part asks for
an explicit "wheels off the ground" or "space is clear" confirmation. Each
step is optional and re-runnable.

1. **Car.** Its name: a new one or an existing one.
2. **Battery.** The voltage proposes a cell count, which the user confirms
   (a full 3S looks like an empty 4S).
3. **Measurements.** Tape measure: wheelbase, track width, body length and
   width, the lidar's position from the rear axle and the centerline. A
   kitchen scale under each axle gives the mass and where the center of
   gravity is; or both are typed in directly.
4. **IMU mounting.** Three captures with the car held still: flat on the
   floor (gravity gives z), nose lifted (gives x), and left side lifted as a
   check.
5. **Lidar.** One capture with a meter clear around the car and one with an
   object on the car's left set `upside_down`.

   On a stand, wheels off the ground:

6. **Steering range.** The servo is nudged to full left, straight and full
   right, just short of the end stops. The angle at full lock is measured
   with a protractor or guessed: the floor test measures it for real.
7. **Motor direction.** A short spin; the user says which way the wheels
   turned. If backward, the motor must be inverted in VESC Tool.
8. **Speed per ERPM.** The user counts the turns of a marked wheel against
   the tachometer; with the wheel's diameter that gives a first
   `speed_to_erpm_gain`.
9. **Minimum speed and compensation.** An automatic ERPM ramp (commanded vs
   measured) gives where the motor runs smoothly (`min_speed_mps`) and how
   short of the command it settles (`speed_compensation`).

   On the floor, 4 x 4 m clear:

10. **Straight at a wall.** The lidar's distance to the wall against the
    tachometer gives `speed_to_erpm_gain` including tyre squash and slip.
    The gyro's yaw while cruising gives the straight servo position; the
    drive is repeated until it hardly curves.
11. **Steering angles.** Eight 2.5 m arcs with the servo fixed, at a
    quarter, half, three quarters and full lock each way. Yaw rate / speed
    is the curvature, and `atan(wheelbase * curvature)` the steering angle.
    Both full-lock arcs are needed; the others refine the table.
12. **Review and save.** Every changed value, old and new side by side.

### The steering table

A servo -> angle lookup table (piecewise linear between the measured arc
points) rather than an offset and a gain.

- The table is simply the measured points, so it needs no fit. Left and
  right are measured separately.
- It's validated as monotonic: servo positions strictly increase, and the
  angles strictly increase or strictly decrease along them and span
  straight ahead.
- The VESC needs angle -> servo, the simulation needs servo -> angle: both
  are the same table read either way.
- Its first and last points are the servo positions never exceeded.
- `vehicle_limits` keeps one `max_steering_angle_rad`: the smaller of the
  two sides (per-side limits would touch every algorithm).
- A table can't be tuned with the panel's scalar sliders. Trim adjustments
  go through a re-calibration.

## Later (not done)

- **More in the car file:** the git commit, hostname, VESC firmware and
  hardware name, lidar model and serial, and the VESC Tool motor config, so
  a later calibration can flag that something changed. How each value was
  measured, its uncertainty and the raw samples, so the data can be
  re-fitted without driving again.
- **IMU biases and noise:** gyro and accelerometer bias from a longer
  capture standing still, and `yaw_rate_std`. `speed_std_mps` and
  `yaw_rate_std_rad_s` are still set by hand in
  `config/localization/dead_reckoning.toml`.
- **Lidar yaw offset:** square to a wall at a taped distance, which also
  cross-checks the lidar's x.
- **WASD limits:** `human_max_*` in `config/sensors/web_gui.toml` should
  derive from the calibrated limits.
- **A check drive** before saving (e.g. a figure 8, dead reckoning vs
  SLAM).
- **A table for the motor too:** commanded -> actual ERPM, in place of the
  single `speed_compensation`; then speed-dependent maps (over speed and
  battery voltage).
- **Understeer/cornering stiffness:** repeat the arcs at a second speed;
  the change in radius gives the understeer gradient, hence cornering
  stiffness for `dynamic_bicycle` and friends.
- **Accel/brake limits:** step commands, measure the ERPM response. Gives
  real `max_accel_mps2`, and `max_decel_mps2` for the given
  `brake_current_a`.
- **Steering rate:** servo steps, estimate `max_steering_rate_rad_s` from the
  yaw-rate response (the servo has no feedback).
- **Gyro scale:** drive N full circles returning to the start (lidar/scan
  match confirms the pose) and compare the integrated yaw to N x 360 degrees.
- **Yaw inertia**, tyre/Pacejka parameters for the nonlinear models.
- **Backups:** `history/` is gitignored, so an "export" button in the GUI to
  copy calibrations off the car.
