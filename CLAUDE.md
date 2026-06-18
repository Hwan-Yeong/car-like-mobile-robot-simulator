# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A bird's-eye-view 2D car-like (Ackermann/bicycle model) mobile robot simulator, built as a learning/portfolio
project for autonomous-driving software engineering. A vehicle follows a reference path of waypoints; the
vehicle model and path-tracking controller are swappable at runtime, physical parameters are tunable via
sliders, and telemetry is plotted live. Correctness of the kinematics/control math and clean OOP architecture
(explicit design patterns) matter more than visual fidelity here.

The project is built in phases, each one compiling and running before the next starts. Track current phase
progress against the list below rather than assuming everything is implemented.

- **Phase 1 (done):** `KinematicBicycleModel` + `PurePursuitController` + SFML render, car follows an oval path.
- **Phase 2 (done):** ImGui panels (`ParamPanel` sliders, `ControlPanel` runtime model/controller/path
  selection, play/pause/reset, target speed). Oval and figure-eight paths are both selectable.
- **Phase 3:** `DynamicBicycleModel` (2-DOF lateral dynamics) + ImPlot telemetry.
- **Phase 4:** `StanleyController` (compare against Pure Pursuit).
- **Phase 5:** MPC controller (Eigen-based QP).
- **Phase 6:** YAML config loading, README with screenshots/GIFs.

Dependencies are wired into `CMakeLists.txt` only when the phase that needs them starts — e.g. ImPlot isn't
fetched until Phase 3, Eigen3 isn't linked until Phase 5, yaml-cpp until Phase 6. Don't add a dependency to
the build before the code that uses it exists.

## Build / run

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release   # configure (first run fetches+builds SFML from source)
cmake --build build -j"$(nproc)"                 # build
./build/car_sim                                   # run
```

There is no test suite yet. There's no separate lint step — treat compiler warnings as errors to fix, not
suppress.

## Environment notes (WSL2 / Ubuntu 22.04)

- Runs under WSLg (`DISPLAY=:0`, X11 socket at `/tmp/.X11-unix`, GL rendering via the D3D12/Mesa software
  path). SFML windows display fine; `"Setting vertical sync not supported"` on startup is a benign WSLg/Mesa
  warning, not an error.
- Ubuntu 22.04's apt only ships SFML 2.5.1; the project needs 2.6, so `CMakeLists.txt` fetches and builds
  SFML 2.6.1 from source via `FetchContent` rather than `find_package`. Audio/network SFML modules are
  disabled (`SFML_BUILD_AUDIO`/`SFML_BUILD_NETWORK` OFF) since this project has no sound and no networking.
- Eigen3 (3.4.0) and yaml-cpp (0.7.0) are already available as system packages (`libeigen3-dev`,
  `libyaml-cpp-dev`) — use `find_package`, not `FetchContent`, when those phases wire them in.
- ImGui (`v1.91.6`) and imgui-sfml (`v2.6.1` — the SFML-2.x compatible release; `master` now targets SFML 3
  and won't link) are fetched via `FetchContent` in `CMakeLists.txt`. ImGui has no CMakeLists.txt of its own,
  so it's only `FetchContent_Populate`d (not `MakeAvailable`d) and its source dir is pointed to via the
  `IMGUI_DIR` cache variable that imgui-sfml's own build expects. `IMGUI_SFML_FIND_SFML` is forced `OFF` so it
  links the `sfml-graphics` target our own FetchContent already built, instead of re-running `find_package`.
  When Phase 3 adds ImPlot, pin it to `v0.17`.
- ImGui persists window positions/sizes to an `imgui.ini` it writes next to the working directory at exit —
  it's gitignored. Delete it if a panel seems to have the wrong position/size after a layout code change;
  `ImGuiCond_FirstUseEver` is silently ignored once a window has a saved entry in that file.

## Architecture

Header/impl split: public interfaces and types live under `include/<module>/`, implementations under
`src/<module>/`. One executable target, `car_sim`, built from `src/app/main.cpp` plus everything under `src/`.

Modules, in dependency order:

- **`core/`** — value types (`VehicleState` = `[X, Y, psi, vx]`, `VehicleParams`, `Waypoint`/`Path`) and the
  `IVehicleModel` **Strategy** interface with its implementations (`KinematicBicycleModel` now,
  `DynamicBicycleModel` in Phase 3).
- **`controller/`** — the `IController` **Strategy** interface and implementations (`PurePursuitController`
  now, `StanleyController`/MPC controller later). Controllers consume a `VehicleState` + `Path` and return a
  steering command; they don't know which model produced the state.
- **`engine/`** — `SimulationEngine` owns the current model, controller, path, and state, and drives the
  fixed-timestep update loop (`step(dt)`: ask the controller for steering, advance the model, notify
  observers). Renderer/GUI panels implement `ISimulationObserver` (**Observer** pattern) and register with
  `addObserver()` instead of the engine depending on them. `ModelFactory` builds models/controllers from a
  string type name (**Factory** pattern) so swapping implementations at runtime doesn't require the engine or
  `main.cpp` to know concrete types.
- **`renderer/`** — `Camera` handles the world(meters, Y-up)↔screen(pixels, Y-down) transform, pan/zoom, and
  follow-cam (recenters on the vehicle each frame). `SFMLRenderer` implements `ISimulationObserver`, drawing
  the path and vehicle each frame using the camera's current `pixelsPerMeter()` — vehicle/path sizes must be
  computed from that, not a hardcoded pixel scale, or they desync from zoom.
- **`gui/`** — ImGui panels. `ParamPanel` draws sliders for whichever `VehicleParams` fields exist today
  (`wheelbase`, `max_steer`; Phase 3 adds `Cf`/`Cr`/`m`/`Iz` once `DynamicBicycleModel` needs them) bound
  directly to a live `VehicleParams&`. `ControlPanel` draws play/pause/reset, target speed, and path/model/
  controller selectors, but is deliberately ignorant of `engine::SimulationEngine` — it only mutates a plain
  `ControlPanelState` struct that `main.cpp` reads each frame and acts on (calling `setPath`/`setModel`/
  `setController`/`setSpeed`/`reset`). This keeps `gui/` decoupled from `engine/`; it depends only on
  `core/` types and primitives. `TelemetryPanel` (Phase 3) will follow the same pattern with ImPlot.
  Combo box contents come from `ModelFactory::vehicleModelNames()`/`controllerNames()`, not hardcoded lists,
  so registering a new model/controller in the factory is enough to make it selectable.

**Live parameter tuning:** `KinematicBicycleModel` and `PurePursuitController` hold a `const VehicleParams*`
rather than a copy, and `ModelFactory::createVehicleModel`/`createController` take a pointer too. `main.cpp`
owns the one `VehicleParams` instance and passes its address through; `ParamPanel` mutates that same instance
directly via ImGui sliders, so changes apply on the very next `step()` with no extra wiring. Any future
model/controller that reads tunable params needs to follow the same by-pointer pattern, not take a by-value
copy at construction — otherwise sliders will silently do nothing.

### Vehicle model math (get this right — it's the point of the project)

- **`KinematicBicycleModel`** — rear-axle reference, state `[X, Y, psi, vx]`:
  `Xdot = vx*cos(psi)`, `Ydot = vx*sin(psi)`, `psidot = vx*tan(delta)/L`. Integrated with RK4
  (`vx` and `delta` held constant across the integration step — there's no throttle input yet).
- **`DynamicBicycleModel`** (Phase 3) — 2-DOF lateral dynamics, state `[vy` (or `beta`), `r]`, linear tire
  model `Fy = C*alpha`, Newton's law for lateral force + yaw moment, with longitudinal velocity `vx` treated
  as a slowly-varying parameter rather than a state. Also integrated with RK4.

### Controllers

- **Pure Pursuit** — geometric, adaptive lookahead `Ld = clamp(k*vx, min, max)`,
  `delta = atan2(2*L*sin(alpha), Ld)` where `alpha` is the bearing to the lookahead target relative to
  heading. `findLookaheadTarget` finds the closest path point, then walks forward (wrapping, since paths are
  closed loops) until a point at least `Ld` away is found.
- **Stanley** (Phase 4) — front-axle reference, cross-track error + heading error.
- **MPC** (Phase 5) — linearized lateral error model, QP solved each step with Eigen.
