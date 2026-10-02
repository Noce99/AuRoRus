# AuRoRus

**Au**tonomous **Ro**boracer **Rus**t - an autonomous driving stack for
[RoboRacer](https://roboracer.ai/) (formerly F1TENTH) cars, written in Rust.

It covers the whole pipeline in one crate (`aurorus`): a vehicle and LIDAR
simulator with a random track generator, SLAM mapping and localization,
race-line planning, opponent detection, a set of autonomous driving
algorithms, and the drivers for the real car (VESC, Hokuyo LIDAR, joystick).
Everything is driven from a browser GUI, and the same code runs in simulation
on a laptop and on the car.

## Installation

1. Install Rust with [rustup](https://rustup.rs/) (see the official
   [install page](https://www.rust-lang.org/tools/install) for other
   platforms and options):

   ```sh
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   ```

   The crate uses the 2024 edition, so it needs Rust 1.85 or newer -
   `rustup update` brings an older install up to date. No other system
   library is required.

2. Build everything from the repository root:

   ```sh
   cargo build --release
   ```

   The executables are placed in `target/release/`. The release profile
   trades compile time for speed (LTO, a single codegen unit), so the first
   build takes a few minutes.

3. Run the main application and open <http://localhost:1999>:

   ```sh
   ./target/release/web_gui
   ```

   On a machine that isn't set up as a car this runs the simulation. Run the
   executables from the repository root: they look for `config/`, `maps/` and
   the other folders below relative to the current directory.

## Executables

`cargo build --release` builds every binary below into `target/release/`.
Each one prints its full list of options with `--help`.

| Executable | What it does | Web page |
|---|---|---|
| [`web_gui`](#web_gui) | The main application: drive the simulation or the real car | <http://localhost:1999> |
| [`replay_web_gui`](#replay_web_gui) | Play back a recorded debug session | <http://localhost:1998> |
| [`benchmark_viewer`](#benchmark_viewer) | Compare and replay recorded benchmark runs | <http://localhost:1997> |
| [`car_calibration`](#car_calibration) | Guided calibration of a car's hardware | <http://localhost:1996> |
| [`benchmark_communication_time`](#benchmark_communication_time-and-benchmark_data_freshness) | Measure topic read/write latency | - |
| [`benchmark_data_freshness`](#benchmark_communication_time-and-benchmark_data_freshness) | Measure how stale the data readers see is | - |

The web pages are served on every network interface, so they can also be
opened from another device, e.g. a phone or laptop next to the car, at
`http://<address of the machine>:<port>`.

### `web_gui`

The main application. It starts every part of the stack - sensors,
localization, planning, perception, the driving algorithms - and serves the
GUI to control them: pick or generate a map, plan a race line, map a track
with SLAM, select and tune a driving algorithm live, race against simulated
opponents, run benchmarks and record debug sessions.

It simulates the car unless the machine is set up as one: with a `CAR_NAME`
file naming a calibrated car (see [Car calibration](documentation/car_calibration.md))
it drives the real hardware instead.

```sh
./target/release/web_gui                # simulation, or the car CAR_NAME names
./target/release/web_gui --sim          # simulate, even on a car
./target/release/web_gui --car tom      # run as the car "tom"
./target/release/web_gui --debug        # record a debug session from the start
```

See [Web GUI](documentation/web_gui.md) for every part of the page.

### `replay_web_gui`

Read-only playback of a `.debug` session file recorded by `web_gui` (from its
Debug panel, or with `--debug`): the same map view, drawing whatever the
recorded executors drew, with a timeline to scrub through. It only needs the
file itself. See [Replay web GUI](documentation/replay_web_gui.md).

```sh
./target/release/replay_web_gui --file debugs/<session>.debug
```

### `benchmark_viewer`

Read-only web UI over the benchmark runs recorded from `web_gui`'s Benchmark
panel: filter them, compare their lap times, parameters and telemetry, and
replay several of them together on the map. See
[Benchmark viewer](documentation/benchmark_viewer.md).

```sh
./target/release/benchmark_viewer
```

### `car_calibration`

A guided calibration of a real car, served as a web page: its size and
weight, how its IMU and LIDAR are mounted, its steering range and its motor's
speed gain. Saving writes `config/calibration/<car>.toml`, which every other
executable then uses. It needs the car's VESC and can't run at the same time
as `web_gui`. See [Car calibration](documentation/car_calibration.md).

```sh
./target/release/car_calibration --car tom
```

### `benchmark_communication_time` and `benchmark_data_freshness`

Two terminal benchmarks of the executor/topic framework everything is built
on, with one writer and several readers at different rates. The first
measures how long the read and write calls take, the second how old the data
is when a reader sees it. Both are explained in
[Core framework](documentation/core_framework.md).

```sh
./target/release/benchmark_communication_time --duration 30
```

### Hardware probes

A few small bring-up tools for the real car live in [examples/](examples/)
(`vesc_probe`, `servo_probe`, `motor_probe`, `hokuyo_probe`). They are built
with `cargo build --release --examples` into `target/release/examples/`; each
file's header explains its usage.

## Maps

Maps live in `maps/`, one folder per map. A fresh clone has none: create one
from `web_gui` - generate a random track, import an image, or map a real track
with SLAM.

<!-- TODO: describe the maps shipped with the repository once they are added. -->

## Files not in the repository

These files and folders are listed in `.gitignore`: they are created locally
as the executables run and are specific to one machine.

| Path | What it holds |
|---|---|
| `target/` | Cargo's build output, including the executables in `target/release/` |
| `maps/` | The maps: generated, imported, or built with SLAM (see [Maps](#maps)) |
| `debugs/` | `.debug` session files recorded by `web_gui`, played back by `replay_web_gui` |
| `benchmarks/` | Benchmark runs recorded by `web_gui`, read by `benchmark_viewer` |
| `CAR_NAME` | One line naming the car this machine drives; without it, `web_gui` simulates |
| `config/calibration/history/` | Each car's older calibrations, kept when `car_calibration` saves a new one |
| `other_repos/` | Local clones of third-party projects kept for reference |

Everything else under `config/` is tracked: one TOML file per component,
loaded at startup.

## Documentation

| Document | What it covers |
|---|---|
| [Core framework](documentation/core_framework.md) | The executor/topic framework, its design decisions, and the two communication benchmarks |
| [Web GUI](documentation/web_gui.md) | Every part of `web_gui`'s page, with screenshots, and how it works underneath |
| [Replay web GUI](documentation/replay_web_gui.md) | Playing back a recorded debug session: the map, the timeline, and how it works |
| [Benchmark viewer](documentation/benchmark_viewer.md) | Filtering, comparing and replaying benchmark runs |
| [Autonomous algorithms](documentation/autonomous_algorithms.md) | How driving algorithms are structured, selected and tuned live, and how to add one |
| [Tutorials](tutorial/README.md) | Writing an autonomous algorithm from an empty file, step by step: a gap follower and a pure pursuit |
| [Planning](documentation/planning.md) | How the race line is planned for a map |
| [SLAM](documentation/slam.md) | Mapping a track and localizing on a known map |
| [Vehicle models](src/simulation/vehicle_models/README.md) | The simulator's vehicle dynamics models |
| [Car calibration](documentation/car_calibration.md) | Calibrating a real car and the car file it produces |
| [Joystick](documentation/joystick.md) | Driving the car with a gamepad |

The API documentation is generated from the code:

```sh
cargo doc --no-deps --open
```
