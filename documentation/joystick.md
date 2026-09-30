# Joystick

Status: **working on the real car** (2026-09-30), with a wired Xbox 360 pad
(`Microsoft X-Box 360 pad`, kernel `xpad` driver).

A gamepad drives the car - or the simulator - the same way `web_gui`'s WASD
keys do, but with analog steering and speed. Code: `src/sensors/joystick.rs`
(the `Joystick` executor); settings: `config/sensors/joystick.toml`.

## Using it

1. Plug the pad into the car's computer. It shows up as `/dev/input/js0`:

   ```sh
   ls /dev/input/js*
   ```

2. Start `web_gui` as usual - on the car or in simulation, the joystick is
   always part of it:

   ```sh
   cargo run --release --bin web_gui
   ```

   It prints `Joystick: driving with /dev/input/js0` once it has the pad. If
   the pad isn't there it prints `Joystick: no joystick at ...` once and
   keeps looking every second, so the pad can be plugged in at any time.

3. Drive:

   | Control          | Does                                                  |
   |------------------|-------------------------------------------------------|
   | Left stick, x    | Steering. Full stick is the car's steering lock.      |
   | Right trigger    | Forward. Fully pulled is `max_speed_mps` (3 m/s).     |
   | Left trigger     | Reverse, same scale.                                  |
   | Both triggers    | They cancel out: right minus left.                    |

   Let go of everything and the pad hands control back (see below).

## Who drives

Three sources can command the car. Each tick the car (`Vesc` on the real
car, `SimulatedVehicle` in simulation) acts on the first of these that is
**fresh** and **asks for anything** (non-zero steering or speed):

1. the joystick (`joystick_vesc_command` topic),
2. WASD in `web_gui` (`human_vesc_command` topic),
3. the selected autonomous algorithm (`autonomous_vesc_command` topic),

and stops, steering straight, if none is. So:

- Touching the pad takes over from both WASD and the autonomous algorithm,
  immediately.
- Releasing the pad (stick centered, triggers released) gives control back -
  to WASD if a key is held, else to the autonomous algorithm if one is
  running, else the car stops.
- **Releasing the pad does not stop the car while an algorithm is
  running.** There is no "stop" button yet: to stop the car, pause the
  algorithm in `web_gui` (or stop `web_gui`).
- Moving the pad aborts a running benchmark, like a WASD key.

## Safety

- **Unplugging the pad, or losing it, is safe**: the executor notices, logs
  `Joystick: lost the joystick`, publishes a stationary, centered command
  (which hands control back as releasing it does) and reconnects when the
  pad comes back.
- **If the joystick executor itself stalls**, its last command goes stale
  and is ignored after `command_timeout_s` (0.25 s, in
  `config/actuators/vesc.toml`) on the car, 1 s in simulation.
- **Deadzones**: the stick ignores its first 10% of travel and the triggers
  their first 5%, so a stick that doesn't center exactly never keeps
  overriding the autonomous algorithm.
- **Speed and steering are capped by the car**: full stick is the
  calibrated steering lock (`vehicle_limits`, see
  [car_calibration.md](car_calibration.md)), and the top speed is the lower
  of `max_speed_mps` and the car's own `max_speed_mps` limit.
- **Dead man's switch (optional)**: set `deadman_button = 4` in
  `config/sensors/joystick.toml` and the pad only drives while the left
  bumper is held. Recommended with a **wireless** pad: out of range, a
  wireless receiver can keep reporting the last stick and trigger positions,
  and nothing on the car can tell.

## Configuration

`config/sensors/joystick.toml` - read at startup, so restart `web_gui` after
editing it.

| Key                 | Default          | What                                                               |
|---------------------|------------------|--------------------------------------------------------------------|
| `device`            | `/dev/input/js0` | The pad's joystick device.                                         |
| `rate_hz`           | 50               | How often the pad's state is published.                            |
| `reconnect_delay_s` | 1.0              | Wait between attempts to (re)open the pad.                         |
| `max_speed_mps`     | 3.0              | Speed of a fully pulled trigger, capped by the car's limit.        |
| `speed_exponent`    | 2.0              | Trigger travel is raised to this: 1 linear, 2 finer at low speed.  |
| `steering_axis`     | 0                | Left stick x.                                                      |
| `throttle_axis`     | 5                | Right trigger.                                                     |
| `reverse_axis`      | 2                | Left trigger.                                                      |
| `invert_steering`   | false            | For a pad whose stick reads left positive.                         |
| `stick_deadzone`    | 0.1              | Stick travel ignored around the center (fraction of full travel).  |
| `trigger_deadzone`  | 0.05             | Trigger travel ignored at rest.                                    |
| `deadman_button`    | unset            | Button that must be held to drive, e.g. 4 (left bumper).           |

With `speed_exponent = 2` and `max_speed_mps = 3`, a quarter-pulled trigger
asks for about 0.13 m/s, half for 0.67 m/s, full for 3 m/s. On the car, the
VESC driver then brakes below its `stop_speed_mps` and raises anything above
it to the car's calibrated minimum speed (`motor.min_speed_mps`), so the
first bit of trigger travel jumps straight to that minimum.

### Xbox 360 pad numbering (`xpad`)

| Axis | Control         | Rest    |   | Button | Control     |
|------|-----------------|---------|---|--------|-------------|
| 0    | Left stick x    | 0       |   | 0      | A           |
| 1    | Left stick y    | 0       |   | 1      | B           |
| 2    | Left trigger    | -32767  |   | 2      | X           |
| 3    | Right stick x   | 0       |   | 3      | Y           |
| 4    | Right stick y   | 0       |   | 4      | Left bumper |
| 5    | Right trigger   | -32767  |   | 5      | Right bumper|
| 6    | D-pad x         | 0       |   | 6      | Back        |
| 7    | D-pad y         | 0       |   | 7      | Start       |
|      |                 |         |   | 8      | Guide       |

### Another pad

Find its axis and button numbers with `jstest` (package `joystick`):

```sh
jstest /dev/input/js0
```

Move each control and note which number changes, then set `steering_axis`,
`throttle_axis`, `reverse_axis` (and `deadman_button`) to match. The
triggers must rest at the axis's minimum (-32767) and read the maximum
(32767) when fully pulled; a pad whose throttle is a stick instead would
need code changes. If steering goes the wrong way, set
`invert_steering = true`.

With more than one joystick plugged in, `js0` is whichever came first. Pin
one pad by its stable name instead:

```sh
ls /dev/input/by-id/*-joystick
# device = "/dev/input/by-id/usb-_USB_Controller_5E7CFDB2-joystick"
```

## Troubleshooting

- **`no joystick at /dev/input/js0 (Permission denied)`**: `/dev/input/js*`
  belongs to the `input` group, and a logged-in desktop user only gets access
  through the seat's ACL. Over SSH or as a service, add the user to the
  group, then log in again:

  ```sh
  sudo usermod -aG input $USER
  ```

- **`no joystick at /dev/input/js0 (No such file or directory)`**: the pad
  isn't seen by the kernel. Check `cat /proc/bus/input/devices` for it and
  `dmesg` for the driver; some clones need `xpad` loaded
  (`sudo modprobe xpad`).
- **The car doesn't move but the pad is found**: check that nothing else
  holds control first - it's the joystick's turn only while the stick or a
  trigger is past its deadzone - and that the VESC is connected
  (`Vesc: VESC firmware ...` in the log). A trigger that doesn't rest at
  -32767 in `jstest` reads as always pulled; raise `trigger_deadzone` or
  swap the pad.
- **The autonomous algorithm never gets control back**: a stick drifting
  past `stick_deadzone` keeps overriding it. Check the stick's resting value
  in `jstest` and raise `stick_deadzone`.
