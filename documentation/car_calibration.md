# Car calibration (design notes)

Status: **being built** (discussed 2026-09-30). Done: the car file (`src/hardware.rs`, `config/hardware/tom.toml`, `config/car_template.toml`), `CAR_NAME`, the Vesc (steering lookup table) and lidar reading it, `web_gui --sim`/`--car`; the `vehicle_geometry` topic every algorithm, dead reckoning, SLAM, the planner and the simulation read (the simulation simulates the named car, else the template). Next: the `car_calibration` binary.

The goal is a guided `car_calibration` binary that measures every hardware
parameter of a car, so the stack can be installed on a new car (different
servo, motor, VESC or lidar) and calibrated without editing TOMLs by hand.

## Storage

```
config/hardware/
├── tom.toml                  # tracked: the latest accepted calibration of "tom"
├── <other_car>.toml          # tracked: one file per calibrated car
└── history/                  # gitignored: every older calibration, never edited
    └── tom/
        ├── 2026-09-30_15-18-02.toml
        └── ...
```

`CAR_NAME` (repo root, gitignored) holds the name of the car this machine
drives, e.g. `tom`.

- The car's name is asked during calibration.
- Accepting a calibration first moves the current `<car>.toml` into
  `history/<car>/` and then writes the new one. Reverting a bad calibration
  means copying a history file back, and `git diff` shows exactly what a
  calibration changed.
- `CAR_NAME` is per machine, so the repo can hold several cars. With no
  `CAR_NAME`, binaries run in simulation.
- Each file stores `schema_version`, the date, the git commit, the hostname,
  VESC firmware and hardware name, lidar model and serial, and the VESC Tool
  motor config (read-only, recorded so a later calibration can flag that it
  changed; never written). For each value it also stores how it was measured
  and its uncertainty, ideally with the raw samples so the data can be
  re-fitted without driving again.
- Partial re-calibration (e.g. only steering) copies the current file and
  replaces one section; each section records which run it came from.
- A tracked template (e.g. `config/hardware/example.toml`) is the starting
  point for a new car's first calibration.

## What moves into the car file (single source of truth)

Physical facts only. Operating policy (port, rates, timeouts,
`max_speed_mps`, accel caps, battery warning voltage) stays in the module
configs.

| Value | Today |
|---|---|
| Steering map (offset, gains or lookup table, servo min/max, max angle per side) | `actuators/vesc.toml` |
| Speed map (`speed_to_erpm_gain`, compensation, `min_speed_mps`), `brake_current_a` | `actuators/vesc.toml` |
| `battery_cells`, IMU axis mapping (`imu_x/y/z`), gyro/accel bias | `actuators/vesc.toml` |
| Lidar `upside_down`, mount x/y/yaw, valid range | `sensors/hokuyo_lidar.toml` (+ `simulated_lidar.toml` mount kept equal by hand) |
| Wheelbase | `wheelbase_m` x4: pure_pursuit, mpc, path_follower, frenet_overtaking (0.32, measured 0.325) |
| Body width | disparity_extender 0.6 (includes margin), potential_field/pursuit 0.34, race_line 0.3 |
| Body length/width drawn | `VEHICLE_BODY_*` in `topics.rs`, `DRAWN_BODY_LENGTH_M` in slam.rs, dead_reckoning.rs |
| `lf/lr`, mass, track width | `actuators/simulated_vehicle.toml` placeholders |
| `rear_axle_to_cg_m`, `speed_std_mps`, `yaw_rate_std_rad_s` | `localization/dead_reckoning.toml` |
| WASD limits | `sensors/web_gui.toml` `human_max_*` should derive from the calibrated limits |

The algorithm configs keep only their own margins (e.g. `safety_margin_m`
added to the calibrated width) instead of their own copy of the car's size.

**The simulation uses the car file too:** geometry, mass, `lf/lr`, steering
limits (per side), minimum speed, lidar mount. The simulated car then behaves
like the real one, which makes tuning controllers in sim more faithful.

## Web GUI modes

- `CAR_NAME` exists → web_gui starts on the hardware with that car's file.
- `--sim` → simulation even on the car (still using the car's geometry).
- `--hardware` is dropped. `--car <name>` picks a car instead of the one
  `CAR_NAME` names.
- The GUI shows a clear HARDWARE / SIM badge. If the devices never appear on
  hardware, it fails loudly at startup.
- The VESC panel no longer edits calibrated values; policy values (limits)
  stay editable.

## The guided process (v1)

A separate binary with its own web UI, used from a phone or laptop standing
next to the car. It runs only the hardware executors plus dead reckoning, no
autonomy. Every step that moves the car is **hold-to-run**: the browser sends
a heartbeat while the button is held, and the VESC's `command_timeout_s`
stops the car when the heartbeat stops. Speeds are capped low, and each
moving step asks for an explicit "wheels off the ground" or "space is clear"
confirmation. Each step is optional/re-runnable, so a partial re-calibration
is possible.

0. **Automatic checks.** VESC firmware and hardware name; lidar model,
   range and steps (query the lidar for its info); battery voltage, then a
   proposed cell count the user confirms (a full 3S looks like an empty 4S);
   read the VESC Tool motor config for the record.
1. **Tape measure and scale.** Wheelbase, front and rear track, body length
   and width including overhangs, lidar x/y relative to the rear axle, wheel
   diameter (only as a first guess). A kitchen scale under each axle gives
   `lf/lr` and total mass.
2. **IMU, standing still.** Gravity gives the z axis; the user lifts the
   nose, which gives x; 10 s of standing still gives gyro and accelerometer
   bias and noise (`yaw_rate_std`).
3. **Lidar.** An object held on the car's left sets `upside_down`; square to
   a wall at a taped distance gives the lidar's yaw offset and cross-checks
   `mount_x`.
4. **Wheels off the ground.** Slider to the servo end stops, then back off a
   margin; the user confirms steering and motor direction; straight-ahead
   offset by eye as a first guess; automatic ERPM ramp (commanded vs
   measured) gives the stall speed (`min_speed_mps`) and the tracking ratio.
5. **On the floor, free space ~4x4 m next to a wall.**
   - Straight line: the lidar measures the distance travelled against the
     wall, which gives `speed_to_erpm_gain` including tyre squash and slip.
     While driving straight the gyro should read zero yaw; adjust the servo
     offset until it does.
   - Circles at 5-7 servo values per side, low constant speed:
     R = v / yaw rate (SLAM path as a cross-check), effective angle
     delta = atan(L / R). Fitted **separately for left and right**.
6. **Check and save.** Old and new values side by side with uncertainties,
   a check drive (e.g. a figure 8, dead reckoning vs SLAM), save only when
   the user accepts.

### Steering and speed maps: lookup tables

**Decided:** a servo -> angle lookup table (piecewise linear between the
measured circle points) replaces offset + gain. The same approach can be used for commanded
-> actual ERPM.

- The table is simply the measured points, so it is easier to fit than a
  line. The mapping is ~20 lines of interpolation.
- Enforce/validate that it is monotonic: sort the points, check each step
  goes the same way, reject or smooth outliers. Measure enough points (5-7
  per side) so noise isn't followed.
- The VESC needs angle -> servo, the simulation needs servo -> angle: both
  are the same table read either way, valid only when it is monotonic.
- `vehicle_limits` keeps one `max_steering_angle_rad`: **decided** - the
  smaller of the two sides (per-side limits would touch every algorithm).
- A table can't be tuned with the panel's scalar sliders. That's fine, since
  calibrated values stop being editable there. Trim adjustments go through a
  re-calibration.

## Later (not v1)

- **Understeer/cornering stiffness:** repeat the circles at a second speed;
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
- **Speed-dependent maps** (speed compensation as a table over speed and
  battery voltage).
- **Backups:** `history/` is gitignored, so an "export" button in the GUI to
  copy calibrations off the car.
