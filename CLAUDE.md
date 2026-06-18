# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A bird's-eye-view 2D car-like (Ackermann/bicycle model) mobile robot simulator, built as a learning/portfolio
project for autonomous-driving software engineering. A vehicle follows a reference path of waypoints; the
vehicle model and path-tracking controller are swappable at runtime, physical parameters are tunable via
sliders, and telemetry is plotted live. Correctness of the kinematics/control math and clean OOP architecture
(explicit design patterns) matter more than visual fidelity here.

`README.md` covers the same project at a glance for a portfolio audience (features, screenshots, build/run);
this file is the deeper architecture/math reference.

The project is built in phases, each one compiling and running before the next starts. Track current phase
progress against the list below rather than assuming everything is implemented.

- **Phase 1 (done):** `KinematicBicycleModel` + `PurePursuitController` + SFML render, car follows an oval path.
- **Phase 2 (done):** ImGui panels (`ParamPanel` sliders, `ControlPanel` runtime model/controller/path
  selection, play/pause/reset, target speed). Oval and figure-eight paths are both selectable.
- **Phase 3 (done):** `DynamicBicycleModel` (2-DOF lateral dynamics, selectable alongside the kinematic
  model) + `TelemetryPanel` (ImPlot time-series of side-slip, yaw rate, cross-track error, steering, speed).
- **Phase 4 (done):** `StanleyController`, selectable alongside Pure Pursuit via the same `ControlPanel` combo.
- **Phase 5 (done):** `MPCController` (condensed linear MPC solved via Eigen), selectable alongside Pure
  Pursuit and Stanley.
- **Phase 6 (done):** YAML startup config (`config/config.yaml` + `core::loadConfig`) and `README.md` with
  screenshots. No animated GIFs — ffmpeg/imagemagick weren't available in the dev environment and installing
  them needed a sudo password that wasn't available either, so the user chose static PNGs only.

All six planned phases are complete. Further work (new models/controllers/paths, tests, packaging) isn't
scoped yet — ask before assuming a "Phase 7."

**Post-Phase-6 additions:** five stylized real-circuit paths (`core/Circuits.hpp`: Suzuka, Monaco,
Silverstone, Spa-Francorchamps, Monza) and a richer renderer (asphalt road ribbon with curbs instead of a
thin line; a procedural vector car silhouette instead of a plain rectangle) — see the `core::Circuits` and
`renderer::SFMLRenderer` notes under Architecture below. No textures/sprites/downloaded images were used
anywhere (avoids licensing risk); everything is vector geometry drawn with SFML shapes.

Dependencies are wired into `CMakeLists.txt` only when the phase that needs them starts. Don't add a
dependency to the build before the code that uses it exists.

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
- Eigen3 (3.4.0) and yaml-cpp (0.7.0) are available as system packages (`libeigen3-dev`, `libyaml-cpp-dev`),
  both wired in via `find_package` (no FetchContent needed since both ship proper CMake configs):
  `Eigen3::Eigen` and the plain (non-namespaced) `yaml-cpp` imported target.
- ImGui (`v1.91.6`) and imgui-sfml (`v2.6.1` — the SFML-2.x compatible release; `master` now targets SFML 3
  and won't link) are fetched via `FetchContent` in `CMakeLists.txt`. ImGui has no CMakeLists.txt of its own,
  so it's only `FetchContent_Populate`d (not `MakeAvailable`d) and its source dir is pointed to via the
  `IMGUI_DIR` cache variable that imgui-sfml's own build expects. `IMGUI_SFML_FIND_SFML` is forced `OFF` so it
  links the `sfml-graphics` target our own FetchContent already built, instead of re-running `find_package`.
- ImPlot (`v0.17`) is fetched the same populate-only way as ImGui (no CMakeLists.txt of its own). Its two
  source files (`implot.cpp`, `implot_items.cpp`) are appended directly to `car_sim`'s sources in
  `CMakeLists.txt` rather than built as a separate library target — there's no consumer besides `car_sim`,
  so a dedicated static lib would just add indirection.
- ImGui persists window positions/sizes to an `imgui.ini` it writes next to the working directory at exit —
  it's gitignored. Delete it if a panel seems to have the wrong position/size after a layout code change;
  `ImGuiCond_FirstUseEver` is silently ignored once a window has a saved entry in that file.

## Architecture

Header/impl split: public interfaces and types live under `include/<module>/`, implementations under
`src/<module>/`. One executable target, `car_sim`, built from `src/app/main.cpp` plus everything under `src/`.

Modules, in dependency order:

- **`core/`** — value types (`VehicleState` = `[X, Y, psi, vx, vy, r]`, `VehicleParams`, `Waypoint`/`Path`,
  plus path-geometry helpers `initialHeading`/`crossTrackError`/`smoothClosedPath`), the `IVehicleModel`
  **Strategy** interface with its implementations (`KinematicBicycleModel`, `DynamicBicycleModel`), and
  `SimulationConfig`/`loadConfig` for startup YAML config. `core/Circuits.hpp` adds five real-circuit path
  generators (`makeSuzukaPath`, `makeMonacoPath`, `makeSilverstonePath`, `makeSpaPath`, `makeMonzaPath`).
  Suzuka is a closed-form cosine-modulated lemniscate (the natural shape for its figure-8 crossover); the
  other four are small hand-placed anchor-point outlines (10-11 points each) smoothed into a dense drivable
  path via `smoothClosedPath`'s uniform Catmull-Rom interpolation. **These are stylized approximations of
  each circuit's recognizable silhouette and corner sequence, built from general knowledge, not surveyed or
  GPS-accurate coordinates** — say so if asked whether they're precise. Anchors are scaled (bounding box
  ~100-160m) to stay in the same order of magnitude as the original oval/figure-eight so the existing
  controller gains (Pure Pursuit lookahead, Stanley/MPC gains) keep working without retuning. Monaco and Spa
  each have one anchor pulled inward to evoke a tight hairpin (Grand Hotel Hairpin, La Source) without
  causing an actual self-intersection; if a future track needs a tighter pinch, check the resulting
  cross-track error in `TelemetryPanel` empirically (analytically bounding Catmull-Rom curvature isn't done
  here) rather than assuming it's drivable.
- **`controller/`** — the `IController` **Strategy** interface and implementations (`PurePursuitController`,
  `StanleyController`, `MPCController`). Controllers consume a `VehicleState` + `Path` and return a steering
  command; they don't know which model produced the state.
- **`engine/`** — `SimulationEngine` owns the current model, controller, path, and state, and drives the
  fixed-timestep update loop (`step(dt)`: ask the controller for steering, advance the model, notify
  observers, cache the steering value as `lastSteering()` for the telemetry panel to read). Renderer/GUI
  panels implement `ISimulationObserver` (**Observer** pattern) and register with `addObserver()` instead of
  the engine depending on them. `ModelFactory` builds models/controllers from a string type name (**Factory**
  pattern) so swapping implementations at runtime doesn't require the engine or `main.cpp` to know concrete
  types.
- **`renderer/`** — `Camera` handles the world(meters, Y-up)↔screen(pixels, Y-down) transform, pan/zoom, and
  follow-cam (recenters on the vehicle each frame). `SFMLRenderer` implements `ISimulationObserver` and draws
  two things each frame, both recomputed from `path_`/`latest_state_` rather than cached (cheap at this
  path size, and trivially correct whenever the path/state changes):
  - `drawRoad()` — a filled asphalt ribbon: at each waypoint, offsets left/right by `kRoadHalfWidth` along
    the *averaged* incoming/outgoing segment normal (not just the outgoing one, or corners would gap), drawn
    as one closed `TriangleStrip`; plus a second `Quads` pass just outside that for alternating red/white
    curb segments. All offset points go through `camera_.worldToScreen()` per vertex per frame, same as the
    old single-line path drawing this replaced.
  - `drawCar()` — a procedural vector silhouette (`car_body_` tapered `ConvexShape`, `car_cabin_`, two
    `CircleShape` headlights, four wheel `RectangleShape`s), built once in the constructor in **car-local
    meters** (origin = vehicle reference point, +x = forward) and redrawn via one shared
    `sf::Transform{translate(screen_pos); rotate(-psi_deg); scale(ppm, ppm)}` passed as `sf::RenderStates` —
    deliberately *not* via each shape's own `setPosition`/`setRotation`, so all parts stay rigidly attached
    in one transform. The `rotate(-psi...)` sign (negating world heading) is required, not arbitrary: world
    is Y-up but screen is Y-down, so a plain rotation by `+psi` would place the nose on the wrong side. The
    fix only works cleanly because the car shape is bilaterally symmetric (headlights, wheels, body all
    mirror across the centerline) — a symmetric shape can't visually distinguish "rotated by ψ in Y-up" from
    "rotated by ψ with the lateral axis flipped," which is exactly what plain `rotate(-psi)` + uniform
    `scale(ppm, ppm)` produces. **If an asymmetric car-shape detail is ever added (e.g. a left-only mirror or
    turn signal), this transform needs revisiting** — it would render mirrored left-right.
- **`gui/`** — ImGui panels. `ParamPanel` draws sliders for `VehicleParams`, grouped by which model uses them
  (kinematic: `wheelbase`/`max_steer`; dynamic: `Cf`/`Cr`/`m`/`Iz`/`a`/`b`) bound directly to a live
  `VehicleParams&` — all sliders are always visible regardless of which model is active, since switching
  models at runtime is the point. `ControlPanel` draws play/pause/reset, target speed, and path/model/
  controller selectors, but is deliberately ignorant of `engine::SimulationEngine` — it only mutates a plain
  `ControlPanelState` struct that `main.cpp` reads each frame and acts on (calling `setPath`/`setModel`/
  `setController`/`setSpeed`/`reset`). This keeps `gui/` decoupled from `engine/`; it depends only on
  `core/` types and primitives. `TelemetryPanel` follows the same pattern: a `record(time, beta, yaw_rate,
  cross_track_error, steering, speed)` call once per *executed* simulation step (not per render frame, so the
  time axis tracks simulated time, not frame rate) feeds rolling buffers that `draw()` renders as five
  stacked ImPlot line plots. Combo box contents come from `ModelFactory::vehicleModelNames()`/
  `controllerNames()`, not hardcoded lists, so registering a new model/controller in the factory is enough to
  make it selectable.
- ImGui and ImPlot contexts are created/destroyed in `main.cpp` around the event loop: `ImGui::SFML::Init`
  creates the ImGui context implicitly, so `ImPlot::CreateContext()` only needs to run after that; on
  shutdown, destroy in the reverse order (`ImPlot::DestroyContext()` before `ImGui::SFML::Shutdown()`).

**Live parameter tuning:** every model and controller (`KinematicBicycleModel`, `DynamicBicycleModel`,
`PurePursuitController`, `StanleyController`, `MPCController`) holds a `const VehicleParams*` rather than a
copy, and `ModelFactory::createVehicleModel`/`createController` take a pointer too. `main.cpp` owns the one
`VehicleParams` instance (seeded from `config/config.yaml` at startup) and passes its address through;
`ParamPanel` mutates that same instance directly via ImGui sliders, so changes apply on the very next
`step()` with no extra wiring. Any future model/controller that reads tunable params needs to follow the
same by-pointer pattern, not take a by-value copy at construction — otherwise sliders will silently do
nothing.

**Startup config:** `core::loadConfig("config/config.yaml")` returns a `SimulationConfig` (vehicle params +
initial path/model/controller names + target speed), falling back field-by-field to hardcoded defaults if
the file is missing, partially filled in, or fails to parse — a bad config should never prevent the
simulator from starting. `main.cpp` resolves the config's path/model/controller *names* to `ControlPanelState`
indices via `indexOrDefault`, which looks them up in the same `ModelFactory::vehicleModelNames()`/
`controllerNames()` lists the GUI combos use, so the config file and the GUI can never disagree about what
names are valid. The binary expects to be run from the repo root (`./build/car_sim`) since the config path
is relative.

### Vehicle model math (get this right — it's the point of the project)

- **`KinematicBicycleModel`** — rear-axle reference, integrates `[X, Y, psi]`:
  `Xdot = vx*cos(psi)`, `Ydot = vx*sin(psi)`, `psidot = vx*tan(delta)/L`. Integrated with RK4
  (`vx` and `delta` held constant across the integration step — there's no throttle input yet). Writes
  `vy = 0` and `r = (psi_next - psi) / dt` back into the output state purely so `TelemetryPanel`'s yaw-rate
  plot stays meaningful when this model is active — the kinematic model has no notion of side-slip.
- **`DynamicBicycleModel`** — 2-DOF lateral dynamics, integrates `[X, Y, psi, vy, r]` with `vx` held constant
  as a parameter (same "slowly varying, not a state" treatment as the kinematic model's `vx`). Linear tire
  model: `alpha_f = delta - (vy + a*r)/vx`, `alpha_r = (b*r - vy)/vx`, `Fyf = Cf*alpha_f`, `Fyr = Cr*alpha_r`;
  Newton's law gives `vy_dot = -vx*r + (Fyf+Fyr)/m` and `r_dot = (a*Fyf - b*Fyr)/Iz`; global position updates
  via `Xdot = vx*cos(psi) - vy*sin(psi)`, `Ydot = vx*sin(psi) + vy*cos(psi)`, `psidot = r`. Integrated with
  RK4. The slip-angle division guards `vx` with a small floor (`kMinVx = 0.5`) since the linear bicycle model
  is singular at zero speed — this is a sandbox simplification, not a real low-speed model. Default params
  (`a=1.2`, `b=1.5`, so `a+b` equals the default `wheelbase=2.7`; `m=1500`, `Iz=2250`, `Cf=Cr=80000`) are
  representative sedan-ish values, not measured from a real vehicle.

### Controllers

- **Pure Pursuit** — geometric, adaptive lookahead `Ld = clamp(k*vx, min, max)`,
  `delta = atan2(2*L*sin(alpha), Ld)` where `alpha` is the bearing to the lookahead target relative to
  heading. `findLookaheadTarget` finds the closest path point, then walks forward (wrapping, since paths are
  closed loops) until a point at least `Ld` away is found.
- **Stanley** — front-axle reference: `front = (x,y) + wheelbase*(cos(psi), sin(psi))`. Finds the nearest
  path point to the front axle, then computes `delta = psi_e + atan2(k*e, k_soft + vx)` where `psi_e` is the
  path tangent heading minus vehicle heading, and `e` is the cross-track error projected onto the *vehicle's*
  heading (`dx*sin(psi) - dy*cos(psi)`, not the path's tangent normal) — this matches the standard Hoffmann et
  al. / PythonRobotics formulation; projecting onto the path tangent instead is a common but subtly different
  mistake. `k_soft` prevents a singular/huge steering command at `vx` near 0. Like Pure Pursuit, it assumes
  `state.x/y` is close enough to the rear axle that `wheelbase` is the right front-axle offset even when
  `DynamicBicycleModel` (CG-referenced) is active — a known simplification, not exact for that model.
- **MPC** — re-linearizes a small lateral error model every control step around the current speed and the
  path's local curvature: state `x = [e_y, e_psi]` (lateral error, heading error vs. the path tangent at the
  nearest point), with `e_y_dot = vx*e_psi` and `e_psi_dot = (vx/L)*delta - vx*kappa` (small-angle Frenet
  kinematics; `kappa` estimated from the heading change between the path segments before/after the nearest
  point, divided by segment length). `e_y` is the position-error vector projected onto the path's *left*
  normal (`-dx*sin(path_heading) + dy*cos(path_heading)`) — note this is the opposite sign convention from
  Stanley's cross-track projection, which is fine since each controller only needs to be internally
  consistent with its own model derivation, not match another controller's convention.
  Discretized (`dt_mpc`) and stacked over `horizon` steps into condensed prediction matrices (`Sx`, `Su`,
  `Sd`, built iteratively: `Sx[k] = A*Sx[k-1]`, `Su[k] = A*Su[k-1]` with `B` placed in column `k`, `Sd[k] =
  A*Sd[k-1] + Bd`). The cost `sum_k x_k^T Q x_k + r*delta_k^2` has no inequality constraints, so the "QP"
  reduces to one dense linear solve, `U = -(Su^T*Qbar*Su + Rbar).ldlt().solve(Su^T*Qbar*(Sx*x0 + Sd*kappa))`
  — this is the intended use of Eigen here (dense linear algebra), not a general-purpose inequality-
  constrained QP solver; nothing like OSQP/qpOASES was added. Only `U(0)` is applied (receding horizon),
  clamped to `+-max_steer`. Default tuning in `ModelFactory` (`horizon=15`, `dt_mpc=0.1` → 1.5s lookahead,
  `q_lateral=5`, `q_heading=3`, `r_steering=1`) tracks both the oval and figure-eight with cross-track error
  bounded under ~0.15m at the default 5 m/s target speed — verified visually, not unit tested.
