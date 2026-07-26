# Road Intersection

A real-time traffic intersection simulation built with Rust and SDL2.

The application models four incoming lanes, immutable random routes, safe vehicle following, adaptive traffic lights, congestion-aware fairness, animated BMP sprites, and deterministic fixed-timestep movement.

· [Русская версия](README.md)  
· [01-edu subject](https://github.com/01-edu/public/tree/master/subjects/road_intersection)

## 📋 TOC

- [🚀 Quick start](#-quick-start)
- [🎮 Controls](#-controls)
- [📝 About](#-about)
- [✨ Features](#-features)
- [🏗️ Architecture](#️-architecture)
- [🚦 Traffic control](#-traffic-control)
- [🚗 Vehicle model](#-vehicle-model)
- [🧰 Technology stack](#-technology-stack)
- [🎨 Assets and rendering](#-assets-and-rendering)
- [🧪 Tests and CI](#-tests-and-ci)
- [📁 Project structure](#-project-structure)
- [⚠️ Notes](#️-notes)
- [🧑‍💻 Author](#-author)

## 🚀 Quick start

### Prerequisites

- Rust toolchain with Cargo
- Windows x64, Linux, or macOS
- system SDL2 package on Linux and macOS

Windows x64 SDL2 runtime and import libraries are included in the repository.

### Clone and run

```bash
git clone https://01.tomorrow-school.ai/git/nyestaye/road_intersection
cd road_intersection
cargo run --release
```

### Linux

```bash
sudo apt update
sudo apt install libsdl2-dev
cargo run --release
```

### macOS

```bash
brew install sdl2
cargo run --release
```

No CMake, SDL2_image, SDL2_ttf, or SDL2_gfx installation is required.

The renderer uses VSync by default. Set
`ROAD_INTERSECTION_FORCE_RENDER_FALLBACK=1` to exercise the frame-limited
fallback mode.

## 🎮 Controls

| Key   | Action                         |
|-------|--------------------------------|
| `↑`   | spawn a vehicle from the south |
| `↓`   | spawn a vehicle from the north |
| `←`   | spawn a vehicle from the east  |
| `→`   | spawn a vehicle from the west  |
| `R`   | spawn from a random direction  |
| `Space` | pause / resume                 |
| `Backspace` | reset the simulation       |
| `Esc` | close the simulation           |

The direction pad, `R`, `Pause`, and `Reset` controls in the side panel are also clickable.

Every spawned vehicle receives one random immutable route:

- straight;
- left;
- right.

Keyboard autorepeat is ignored. A spawn request is also rejected when the entry lane has no safe space or has reached its calculated capacity.

## 📝 About

Road Intersection is an SDL2 simulation of a two-road, four-direction junction.

The project separates simulation state from rendering. Vehicle movement, route progress, following distance, queue capacity, traffic-light phases, conflict-zone occupancy, and fairness are updated inside a deterministic fixed-timestep model. The renderer only reads the resulting state and draws the scene.

The traffic-light controller deliberately allows only one incoming direction at a time. Every green phase is followed by an all-red clearing phase before another direction may enter the intersection.

## ✨ Features

### Roads and routes

- two crossing roads;
- one incoming and one outgoing lane per direction;
- straight, left-turn, and right-turn paths;
- immutable route selection at spawn;
- path-specific conflict entry and exit points;
- smooth heading changes along polyline and arc-based paths.

### Vehicles

- one fixed movement speed for all vehicles;
- explicit lifecycle:
  - `Approaching`;
  - `Waiting`;
  - `Committed`;
  - `Crossing`;
  - `Leaving`;
- stop-line compliance on red;
- uninterrupted exit after entering the conflict zone;
- minimum `CAR_LEN + GAP` following distance;
- safe spawn rejection;
- route-specific color and sprite row;
- two-frame visual animation;
- automatic removal and passed-vehicle accounting.

### Traffic lights

- red and green states;
- one active incoming direction;
- all-red conflict clearing;
- minimum and maximum green duration;
- dynamic queue-capacity awareness;
- critical-queue priority;
- overdue waiting priority;
- longest-wait selection;
- round-robin tie-breaking;
- starvation prevention.

### Interface

- queue load indicators;
- lane-capacity visualization;
- current signal phase;
- spawned, passed, and active vehicle counters;
- control hints;
- permanent route-color legend.

## 🏗️ Architecture

```text
keyboard input
      |
      v
+-------------+
|    Sim      |
|-------------|
| vehicles    |
| paths       |
| queues      |
| lights      |
| statistics  |
+-------------+
      |
      | fixed 60 Hz update
      v
+-------------+
|  Renderer   |
|-------------|
| roads       |
| markings    |
| BMP sprites |
| side panel  |
+-------------+
      |
      v
 SDL2 window
```

The simulation uses a fixed timestep:

```text
60 updates per model second
```

Rendering may happen more often, but model movement and animation ticks advance only with simulation steps. A passive side panel displays the active signal, queues and capacity, route colors, statistics, and controls. The panel only reads `Sim` state.

## 🚦 Traffic control

The controller uses the following phase sequence:

```text
one green direction
        |
        v
all-red clearing
        |
        v
next green direction
```

A green phase remains active for at least the minimum green time. It may be extended for a critical queue, but it yields after the maximum green time when another direction is waiting.

The next direction is selected by priority class:

1. overdue queues;
2. critical queues;
3. longest waiting queue;
4. round-robin order for equal waits.

A direction becomes overdue after 15 model seconds of waiting. The previous green direction is not selected again while another non-empty direction is waiting.

The all-red phase ends only when:

- the minimum clearing time has elapsed;
- no vehicle remains inside the conflict zone.

### Lane capacity

Capacity follows the subject formula:

```text
capacity = floor(lane_length / (vehicle_length + safety_gap))
```

A vehicle is not created when the lane has reached this capacity.

## 🚗 Vehicle model

Every vehicle stores:

```text
origin
route
progress
position
angle
phase
```

The route is chosen randomly when the vehicle is spawned and never changes.

| Route    | Vehicle and legend color |
|----------|--------------------------|
| Straight | purple (`#9971E4`)       |
| Left     | orange (`#F0A33A`)       |
| Right    | teal (`#44C7B3`)         |

Movement is calculated from path progress rather than frame-dependent pixel displacement. Position and heading are derived from the selected path on every fixed simulation step.

Following logic is evaluated front-to-back for each incoming lane. Vehicles on the same route maintain progress spacing, while vehicles whose routes have physically separated are released from unnecessary coupling.

## 🧰 Technology stack

| Area                        | Technology                      |
|-----------------------------|---------------------------------|
| Language                    | Rust 2021                       |
| Window and input            | SDL2                            |
| 2D rendering                | core SDL2 renderer              |
| Font rasterization          | `fontdue`                       |
| Random route generation     | `rand`                          |
| Vehicle and signal graphics | BMP sprite sheets               |
| Timing                      | fixed timestep with accumulator |
| CI                          | GitHub Actions                  |
| Supported CI platforms      | Ubuntu and Windows MSVC         |

## 🎨 Assets and rendering

The project uses two BMP sprite sheets:

```text
assets/cars.bmp
assets/traffic_lights.bmp
```

`cars.bmp` contains a `3 × 2` layout:

- three route rows;
- two animation frames per route.

`traffic_lights.bmp` contains two frames:

- red;
- green.

Both sheets use a chroma-key background and are loaded through core SDL2. Exact dimensions are validated during startup. A missing, corrupted, or incorrectly sized asset terminates the application with a readable error.

The side-panel font is stored in:

```text
assets/font.ttf
```

## 🧪 Tests and CI

Run all local checks:

```bash
cargo fmt -- --check
cargo test
cargo clippy --all-targets -- -D warnings
cargo build --release
```

The test suite covers:

- required keyboard mappings;
- red-light stopping;
- movement on green;
- safe following distance;
- spawn spam rejection;
- immutable routes;
- capacity calculation;
- vehicle removal and passed count;
- conflict-zone clearing;
- critical and overdue priority;
- round-robin fairness;
- starvation prevention;
- multiple deterministic 60-second stress runs;
- collision and queue-overflow checks.

GitHub Actions runs:

### Ubuntu

```text
cargo fmt -- --check
cargo test
cargo clippy --all-targets -- -D warnings
cargo build --release
```

### Windows MSVC

```text
cargo test
cargo build --release
```

## 📁 Project structure

```text
road_intersection/
├── .github/
│   └── workflows/
│       └── ci.yml
├── assets/
│   ├── cars.bmp
│   ├── font.ttf
│   └── traffic_lights.bmp
├── src/
│   ├── drawing.rs
│   ├── geometry.rs
│   ├── lights.rs
│   ├── main.rs
│   ├── render.rs
│   ├── sprites.rs
│   └── vehicle.rs
├── vendor/
│   └── sdl2/
├── build.rs
├── Cargo.lock
├── Cargo.toml
├── README.md
└── README_EN.md
```

## ⚠️ Notes

- Run the application from the repository root so relative asset paths remain valid.
- Windows x64 uses the SDL2 files included under `vendor/sdl2`.
- Linux and macOS use the system SDL2 installation.
- Rendering does not modify simulation state.
- Sprite animation is visual only and does not affect vehicle speed or traffic logic.
- The controller favors safety and fairness over maximum intersection throughput.

## 🧑‍💻 Author
- Nazar Yestayev (@nyestaye)
- Daniyar Shadykhanov (@dshadykh)
- Sanzhar Serikbayev (@sserikba)
- Maksat Kapan (@mkapan)
