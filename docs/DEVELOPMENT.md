<!-- generated-by: gsd-doc-writer -->
# Development

This guide covers the day-to-day developer workflow for Pegasus Simulator: how the
repository is laid out, how to run and iterate on example scripts, the coding
conventions enforced by pre-commit, and how to extend the framework with new vehicles,
backends, and sensors. See [README.md](../README.md) for installation instructions and
[docs/ARCHITECTURE.md](ARCHITECTURE.md) for a system-level overview, and
[docs/CONFIGURATION.md](CONFIGURATION.md) for the full configuration reference.

## Local setup

Pegasus Simulator is distributed as an **Isaac Sim extension**, not a standalone Python
package — there is no `package.json`/`pip install .` step for local development. Instead:

1. Isaac Sim itself must be installed, and the `isaac_run` shell function defined (see
   the [Installation Instructions](https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html)
   linked from the README). `isaac_run` resolves to
   [`scripts/isaac_run.sh`](../scripts/isaac_run.sh), which launches
   `$ISAACSIM_PATH/python.sh` after stripping `/opt/ros` and `ros2_ws` entries from
   `PYTHONPATH`/`LD_LIBRARY_PATH` so Isaac's bundled py3.12 `rclpy` (from the
   `isaacsim.ros2.bridge` extension) is used instead of a conflicting system ROS 2
   install.
2. `link_app.sh` (or `link_app.bat` on Windows) creates the `app` symlink Isaac Sim
   extensions typically expect, pointing at your local Isaac Sim install.
3. `extensions/pegasus.simulator/extension.toml` declares the extension's runtime pip
   dependencies under `[python.pipapi]`: `numpy`, `scipy`, `pymavlink`, `pyyaml`,
   `toml`. These are installed automatically by Isaac Sim's extension manager
   (`use_online_index = true`) the first time the extension loads — no manual `pip
   install` step is required for a normal workflow.
4. No `.env` file is used. Runtime configuration is a mix of the
   `extensions/pegasus.simulator/config/configs.yaml` file, Python config objects
   passed at script-authoring time, and (for the DPV integration stack under
   `examples/dpv_sim/`) shell environment/`source`d ROS 2 setup scripts. See
   [docs/CONFIGURATION.md](CONFIGURATION.md) for details.

Once set up, run any example with:

```bash
isaac_run examples/1_px4_single_vehicle.py
```

This is also the fastest edit-run loop for iterating on framework code: edit files
under `extensions/pegasus.simulator/pegasus/simulator/`, then re-run the example script
— there is no separate build/compile step for the Python extension code.

## Repository layout

| Path | Purpose |
|---|---|
| `extensions/pegasus.simulator/` | The core Isaac Sim extension (`extension.toml` + `pegasus/simulator` Python package). This is where framework code — vehicles, backends, sensors, thrusters, dynamics, graphs, people — lives. |
| `extensions/pegasus.simulator/pegasus/simulator/logic/` | The extensible core: `vehicles/`, `backends/`, `sensors/`, `graphical_sensors/`, `thrusters/`, `dynamics/`, `graphs/`, `people/`, `people_backends/`, `interface/` (the `PegasusInterface` singleton), `parser/`. |
| `extensions/pegasus.simulator/pegasus/simulator/assets/` | USD robot and world assets (`Robots/V1`, `Robots/Iris`, `Robots/Pegasus`, `Robots/Flying Cube`, `Worlds/Box`, `Worlds/BoxWithCylinders`, `Worlds/Empty`). |
| `extensions/pegasus.simulator/pegasus/simulator/tests/` | `omni.kit.test`-based extension tests (`test_hello_world.py`). |
| `extensions/dpv-build/`, `extensions/dpv-install/` | A colcon (ROS 2) build/install tree for this fork's "DPV" (warehouse delivery vehicle) autonomy stack — Cartographer SLAM, PX4↔ROS2 bridge, warehouse mission/path-planning nodes, pose correctors, ZED wrapper. Gitignored (`build/`, `install/`, `log/` in `.gitignore`) — these are local build artifacts, not source under version control. |
| `examples/` | Standalone runnable scripts, numbered roughly by complexity: `0_template_app.py`, `1_px4_single_vehicle.py`, `2_px4_multi_vehicle.py`, `3_ros2_single_vehicle.py`, `4_python_single_vehicle.py`, `5_python_multi_vehicle.py`, `6_paper_results.py`, `8_camera_vehicle.py`, `9_people.py`, `10_graphs.py`, `11_ardupilot_multi_vehicle.py`, `12_px4_v1_vehicle.py`. |
| `examples/dpv_sim/` | ROS 2 launch files, `start.sh`/`stop.sh` (tmux-based orchestration), and setup scripts for running the DPV stack against Isaac Sim. See [examples/dpv_sim/README.md](../examples/dpv_sim/README.md). |
| `examples/slam_v1/` | GPS-denied SLAM pipeline example (slam_toolbox + PX4 external vision) for the V1 vehicle. |
| `scripts/isaac_run.sh` | The `isaac_run` launcher — see Local setup above. |
| `docs/` | Sphinx documentation sources published to [pegasussimulator.github.io/PegasusSimulator](https://pegasussimulator.github.io/PegasusSimulator/), plus this GSD-generated doc set (`README.md`, `ARCHITECTURE.md`, `CONFIGURATION.md`, `DEVELOPMENT.md`). |

## Adding a new example

Examples are plain Python scripts under `examples/`, each a self-contained Isaac Sim
app. Follow the numbering convention (next available integer prefix) and the pattern
used by existing scripts: construct a `PegasusInterface`, add a `World`, spawn one or
more vehicles (with sensors/backends configured), then call `world.reset()` /
`simulation_app.run()`. Use `0_template_app.py` as the minimal starting skeleton. Run
new/edited examples with `isaac_run examples/<your_script>.py`.

## Coding conventions

Enforced by [`.pre-commit-config.yaml`](../.pre-commit-config.yaml) and run in CI via
[`.github/workflows/pre-commit.yaml`](../.github/workflows/pre-commit.yaml) on every
pull request:

| Tool | Version | What it does |
|---|---|---|
| [black](https://github.com/python/black) | 22.12.0 | Formats Python code, `--line-length 120`. `geo_mag_utils.py` is excluded. |
| [flake8](https://github.com/pycqa/flake8) | 6.0.0 | Lints Python code, with `flake8-simplify` and `flake8-return` plugins. |
| [pyupgrade](https://github.com/asottile/pyupgrade) | v3.3.1 | Upgrades syntax to Python 3.7+ idioms (`--py37-plus`). |
| [pre-commit-hooks](https://github.com/pre-commit/pre-commit-hooks) | v4.4.0 | Trailing whitespace, symlink checks, YAML validity, merge-conflict markers, case conflicts, executable shebang checks, end-of-file fixer, private-key detection, debug-statement detection. |

Install and run locally with:

```bash
pip install pre-commit
pre-commit install        # runs automatically on `git commit`
pre-commit run --all-files
```

**Docstring style:** the codebase uses Google-style docstrings throughout (`Args:`,
`Returns:`, `Note:` sections), plus a standard file-header comment block. Match the
existing header format when adding new files, e.g.:

```python
"""
| File: my_new_module.py
| Author: <your name> (<your email>)
| License: BSD-3-Clause. Copyright (c) <year>, <your name>. All rights reserved.
| Description: <what this file does>
"""
__all__ = ["MyNewClass"]
```

## Branch and commit conventions

There is no `CONTRIBUTING.md` in this repository and no `.github/PULL_REQUEST_TEMPLATE.md`
— no branch-naming convention is formally documented. The repository's `main` branch is
the default/only long-lived branch. Recent commit history uses a loose
[Conventional Commits](https://www.conventionalcommits.org/)-style prefix
(`feat:`, `fix:`, `chore:`) followed by a short imperative summary, e.g.:

```
feat: add guidance mode (GPS-fused localization, no SLAM/EV) for DPV sim
fix: set laser_link→imu_base TF to identity (Cartographer requires colocated IMU/tracking frames)
```

Follow this style for new commits.

## PR process

- CI runs two GitHub Actions workflows on every pull request targeting `main`:
  [`pre-commit.yaml`](../.github/workflows/pre-commit.yaml) (lint/format checks via
  `pre-commit/action`) and [`doc.yaml`](../.github/workflows/doc.yaml) (builds the
  Sphinx docs with `make html`; on push to `main` this also deploys to GitHub Pages,
  gated to the upstream `PegasusSimulator/PegasusSimulator` repository).
- Ensure `pre-commit run --all-files` passes before opening a PR.
- No formal PR template or issue template exists in `.github/` — describe the change,
  the example script(s) or vehicle/backend/sensor affected, and how it was tested
  (e.g. which example script you ran with `isaac_run`).

## Extending the framework

The `logic/` package is built around small abstract/base classes that new vehicles,
backends, and sensors subclass. All of the following live under
`extensions/pegasus.simulator/pegasus/simulator/logic/`.

### Adding a new backend

Backends are the communication/control bridge between the simulated vehicle and an
external interface (PX4 MAVLink, ArduPilot MAVLink, ROS 2, or a custom Python
controller). Subclass `Backend` and `BackendConfig` from
`logic/backends/backend.py`:

```python
from pegasus.simulator.logic.backends.backend import Backend, BackendConfig

class MyBackendConfig(BackendConfig):
    def __init__(self):
        pass

class MyBackend(Backend):
    def update_sensor(self, sensor_type: str, data): ...
    def update_graphical_sensor(self, sensor_type: str, data): ...
    def update_state(self, state): ...
    def input_reference(self): ...   # return list of desired rotor angular velocities
    def update(self, dt: float): ... # called every physics step
    def start(self): ...
    def stop(self): ...
    def reset(self): ...
```

`Backend.initialize(vehicle)` is called by the simulation to give the backend a
reference to its owning `Vehicle`. Existing backends to use as references:
`px4_mavlink_backend.py`, `ardupilot_mavlink_backend.py`, `ros2_backend.py` (publishes
ZED camera + RPLIDAR + TF data over ROS 2 using `omni.graph.core` OmniGraphs and
`rclpy`). A vehicle can carry more than one backend at once (`MultirotorConfig.backends`
is a list) — e.g. PX4 MAVLink plus a ROS2 telemetry backend simultaneously.

### Adding a new sensor

Non-graphical sensors (barometer, IMU, magnetometer, GPS) subclass `Sensor` from
`logic/sensors/sensor.py`. Implement `update(self, state, dt)` decorated with
`@Sensor.update_at_rate` so it only computes a new reading once per `update_period`
(`1 / update_rate` seconds):

```python
from pegasus.simulator.logic.sensors.sensor import Sensor
from pegasus.simulator.logic.state import State

class MySensor(Sensor):
    def __init__(self, config={}):
        super().__init__(sensor_type="MySensor", update_rate=250.0)

    @Sensor.update_at_rate
    def update(self, state: State, dt: float):
        # compute and return a dict of the simulated measurement
        return {"my_value": 0.0}
```

Reference implementations: `barometer.py`, `imu.py`, `magnetometer.py`, `gps.py`
(`geo_mag_utils.py` holds shared geomagnetic math and is excluded from `black`
formatting). Graphical sensors (camera, lidar) follow a parallel pattern under
`logic/graphical_sensors/` — see `monocular_camera.py` and `lidar.py`, subclassing
`GraphicalSensor` in `graphical_sensor.py`.

### Adding a new thrust curve / dynamics model

`logic/thrusters/thrust_curve.py` defines the `ThrustCurve` base: implement
`set_input_reference(self, input_reference)`, `update(self, state, dt)`, and the
`force`/`velocity` properties (per-rotor thrust in Newtons and angular velocity in
rad/s). `quadratic_thrust_curve.py` (`QuadraticThrustCurve`, `T = k * omega^2`) is the
only concrete implementation currently in the framework and is the default in
`MultirotorConfig`. Drag models follow the same pattern under `logic/dynamics/`
(`drag.py` base, `linear_drag.py` implementation).

### Adding a new vehicle

Vehicles subclass `Multirotor` (in `logic/vehicles/multirotor.py`, itself a subclass of
`Vehicle` in `logic/vehicles/vehicle.py`) via a `<Name>Config(MultirotorConfig)` +
`<Name>(Multirotor)` pair. `extensions/pegasus.simulator/pegasus/simulator/logic/vehicles/multirotors/`
holds the two current examples:

- `iris.py` — the default `Iris` quadrotor.
- `v1.py` — a full-scale custom vehicle (`V1Config`/`V1`) with a real thrust curve
  derived from bench-measured motor data, a ZED 2i RGB-D camera
  (`MonocularCamera`), an RPLIDAR-class `Lidar`, and both a `PX4MavlinkBackend` and a
  `ROS2Backend` attached simultaneously.

A minimal new vehicle config overrides `stage_prefix`, `usd_file` (an asset path from
`ROBOTS` in `logic/../params.py`), `thrust_curve`, `drag`, `sensors`,
`graphical_sensors`, `graphs`, and `backends` — all optional, all defaulting to the
`MultirotorConfig` base values (`QuadraticThrustCurve()`, `LinearDrag([0.5, 0.3, 0.0])`,
`[Barometer(), IMU(), Magnetometer(), GPS()]`, `[PX4MavlinkBackend(...)]`). Register the
vehicle's USD asset path in `ROBOTS` (`params.py`) and add a corresponding example
script under `examples/` (following the `12_px4_v1_vehicle.py` pattern for a
custom-vehicle example).

### Adding a person/actor controller

Simulated pedestrian/actor behavior lives under `logic/people/` (`Person`,
`PersonController` base, `LineController` implementation) and `logic/people_backends/`
(`PeopleBackend` base, `ROS2PeopleBackend`). See `examples/9_people.py` for usage.

## Testing

`extensions/pegasus.simulator/pegasus/simulator/tests/test_hello_world.py` is an
`omni.kit.test`-based test using `omni.kit.test.AsyncTestCase` (async
`unittest`-compatible test cases, runnable through Isaac Sim's extension test runner).
It is currently a template/scaffold — no functional unit tests exist for the vehicle,
backend, or sensor logic in this repository. There is no `npm test`/`pytest`-style
top-level test command; `omni.kit.test`-based extension tests are run through Isaac
Sim's own extension test tooling. When adding new framework code, prefer validating it
by running the relevant `examples/*.py` script end-to-end with `isaac_run`.

## The DPV integration stack (ROS 2 workspace)

This fork additionally carries a colcon-based ROS 2 workspace integrating a real drone
autonomy stack (Cartographer SLAM, PX4↔ROS 2 bridge, warehouse mission/path-planning,
pose correctors, ZED wrapper) so it can be driven from Isaac Sim. Source packages are
not vendored in this repository's tracked history — `extensions/dpv-build/` (colcon
build tree) and `extensions/dpv-install/` (colcon install tree, sourced via its
`setup.bash`) are both gitignored local artifacts. Day-to-day workflow for this stack —
prerequisites, phase-by-phase bringup, `start.sh`/`stop.sh` tmux orchestration — is
documented in [examples/dpv_sim/README.md](../examples/dpv_sim/README.md); do not
duplicate that workflow here.
