<!-- generated-by: gsd-doc-writer -->
# Getting Started

This guide gets you from a clean Ubuntu machine to a flying PX4 SITL drone inside Isaac
Sim, and (optionally) to the full DPV ROS 2 autonomy stack running against it. It reflects
the versions this fork has been validated against — see `README.md` "Tested
Configuration" for the authoritative matrix.

## Prerequisites

| Requirement | Version | Notes |
|---|---|---|
| Ubuntu | 22.04 LTS | Other distros/Windows are untested (`docs/source/setup/installation.rst`). |
| NVIDIA Isaac Sim | **6.0.0** | Imports throughout this repo use the `isaacsim.*` namespace (e.g. `isaacsim.core.api`), which is specific to Isaac Sim 6.0 — older Isaac Sim releases will not work with this fork. |
| PX4-Autopilot | **v1.16.2** (SITL) | Built with `make px4_sitl_default none`. PX4 v1.16+ always uses a TCP HIL link — see the `⚠️ Notes for PX4 v1.16+` callout in `README.md`. |
| Python | 3.12 (bundled with Isaac Sim) | Do not use a separate virtualenv Python for running examples — scripts must run under Isaac Sim's own `python.sh` (via `isaac_run`). |
| NVIDIA GPU driver | Compatible with Isaac Sim 6.0 | See NVIDIA's Isaac Sim system requirements; this repo does not pin a specific driver version. <!-- VERIFY: minimum driver version for Isaac Sim 6.0.0 --> |
| `git`, `make`, `cmake`, `python3-pip` | — | Required to compile PX4-Autopilot (`docs/source/setup/installation.rst`). |
| PX4 Python build deps | — | `pip install kconfiglib jinja2 empy jsonschema pyros-genmsg packaging toml numpy future` (`docs/source/setup/installation.rst`). |

Optional, only if you intend to run the DPV ROS 2 autonomy stack under
`examples/dpv_sim/` (see [DPV autonomy stack](#optional-dpv-autonomy-stack) below):

| Requirement | Notes |
|---|---|
| ROS 2 Humble (system install) | `examples/dpv_sim/` launch files and `start.sh` source `/opt/ros/humble/setup.bash`. |
| `ros-humble-cartographer-ros` | `sudo apt install ros-humble-cartographer-ros` (`examples/dpv_sim/README.md`). |
| `tmux` | Required by `examples/dpv_sim/start.sh`, which orchestrates all processes in a `tmux` session named `dpv-sim`. |
| `MicroXRCEAgent` | Bridges PX4's uXRCE-DDS to ROS 2; launched with `udp4 -p 8888`. |
| Built `extensions/dpv-install/` workspace | A colcon install space with a `setup.bash` — this is a separate ROS 2 workspace built from the DPV vehicle's source packages; see `docs/ARCHITECTURE.md` for how it fits together. |

## Installation

### 1. Install NVIDIA Isaac Sim

Download and unpack the Isaac Sim standalone installer (see
`docs/source/setup/installation.rst` for the full walkthrough, including the version
badge shown there is older — use the **6.0.0** release for this fork):

```bash
cd ~
mkdir -p isaacsim
cd isaacsim
wget https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-6.0.0-linux-x86_64.zip
unzip isaac-sim-standalone-6.0.0-linux-x86_64.zip
./post_install.sh
./isaac-sim.selector.sh
rm isaac-sim-standalone-6.0.0-linux-x86_64.zip
```
<!-- VERIFY: exact Isaac Sim 6.0.0 download URL — docs/source/setup/installation.rst only documents the 5.1.0 URL pattern; confirm the filename NVIDIA publishes for 6.0.0 -->

### 2. Add the `isaac_run` shell function

Isaac Sim bundles its own Python 3.12 interpreter and rclpy. Running examples with a
system Python (or a shell that has system ROS 2 sourced) causes import crashes. This
repo ships `scripts/isaac_run.sh`, which strips `/opt/ros` and `ros2_ws` entries from
`PYTHONPATH`/`LD_LIBRARY_PATH` before launching Isaac Sim's `python.sh`.

Add this function to your `~/.bashrc` or `~/.zshrc`:

```bash
isaac_run() { "$HOME/PegasusSimulator/scripts/isaac_run.sh" "$@"; }
```

Then reload your shell (`source ~/.bashrc`) and verify Isaac Sim launches:

```bash
isaac_run --help
```

### 3. Clone this repository

```bash
git clone https://github.com/PegasusSimulator/PegasusSimulator.git
cd PegasusSimulator
```

(If you already have this checkout, skip this step — the rest of this guide assumes
your working directory is the repository root, `~/PegasusSimulator`.)

### 4. Install the `pegasus.simulator` extension as a library

For the standalone Python scripts under `examples/` to `import pegasus.simulator...`,
install the extension into Isaac Sim's bundled Python as an editable package:

```bash
cd extensions
ISAACSIM_PYTHON="$HOME/isaacsim/python.sh" $ISAACSIM_PYTHON -m pip install --editable pegasus.simulator
```

This pulls in the extension's declared Python dependencies (`extensions/pegasus.simulator/setup.py`:
`numpy`, `pymavlink`, `scipy`, `pyyaml`).

### 5. Install PX4-Autopilot (SITL)

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.16.2
git submodule update --init --recursive
make px4_sitl_default none
```

By default, `extensions/pegasus.simulator/config/configs.yaml` expects PX4-Autopilot at
`~/PX4-Autopilot` (`px4_dir` key). If you cloned it elsewhere, edit that file's `px4_dir`
value — see `docs/CONFIGURATION.md` for the full key reference.

## First Run

With Isaac Sim, the `pegasus.simulator` extension, and PX4-Autopilot installed, launch
the simplest example — a single PX4-controlled multirotor:

```bash
cd ~/PegasusSimulator
isaac_run examples/1_px4_single_vehicle.py
```

This starts Isaac Sim, loads the Pegasus extension, spawns a multirotor, and
auto-launches PX4 SITL as a background process (`PX4LaunchTool`), which connects back to
the simulator over a TCP MAVLink link on port 4560. Watch the terminal for PX4's
`Ready for takeoff!` message — that confirms the HIL sensor link, EKF2 initialization,
and home-position lock all succeeded.

QGroundControl (if running on the same machine) connects automatically on UDP 14550;
no extra configuration is needed (`README.md` "Tested Configuration").

To try this fork's custom "V1" vehicle (the real DPV drone model, with ZED camera and
RPLIDAR sensors) instead of the stock Iris multirotor, run:

```bash
isaac_run examples/12_px4_v1_vehicle.py
```

Browse `examples/` for other scenarios — multi-vehicle (`2_px4_multi_vehicle.py`), ROS 2
(`3_ros2_single_vehicle.py`), pure-Python control without PX4 (`4_python_single_vehicle.py`),
camera sensors (`8_camera_vehicle.py`), and the experimental ArduPilot backend
(`11_ardupilot_multi_vehicle.py`).

## Optional: DPV autonomy stack

`examples/dpv_sim/` runs the real DPV drone's ROS 2 autonomy stack (Cartographer SLAM,
`px4_ros2_bridge`, mission/path-planning/trajectory nodes) against the PX4 SITL instance
running inside Isaac Sim, instead of Gazebo. This is optional and only needed if you are
working on navigation/localization, not for basic vehicle-dynamics or MAVLink work.

Full step-by-step instructions (including per-phase verification commands) live in
`examples/dpv_sim/README.md`. The short version, using the bundled `tmux` launcher:

```bash
cd ~/PegasusSimulator/examples/dpv_sim
./start.sh 1     # phase 1: localization only (Cartographer -> PX4 external vision)
# or: ./start.sh 2   (+ mission/navigation), ./start.sh 3  (+ vision corrector),
#     ./start.sh g   (guidance mode: GPS-fused, no SLAM/EV)
```

Tear everything down with:

```bash
./stop.sh
```

See `docs/CONFIGURATION.md` for the full set of `PX4_PARAM_*` environment variables and
hardcoded paths (`REPO`, `DPV_INSTALL`, `PX4_HOME`, `QGC`) `start.sh` relies on, and
`docs/ARCHITECTURE.md` for how the DPV ROS 2 graph connects to the Isaac Sim / PX4 loop.

## Common Setup Issues

- **`ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`** — a system ROS 2
  install (e.g. Humble on Python 3.10) leaked onto `PYTHONPATH`/`LD_LIBRARY_PATH` and
  shadowed Isaac Sim's bundled Python 3.12 rclpy. Always launch Isaac Sim through
  `isaac_run` (`scripts/isaac_run.sh`), which strips ROS 2 entries from the environment
  first — see the "ROS 2 environment note" in `README.md`.
- **PX4 connects but the drone never leaves the ground, altitude stays at 0** — this was
  a real bug in earlier Isaac Sim 6.0 ports of this fork (forces written to an
  unapplied `physxForce:force` USD attribute were silently discarded by PhysX). It is
  fixed in `extensions/pegasus.simulator/pegasus/simulator/logic/vehicles/vehicle.py`
  (`apply_force`/`apply_torque` now use the `RigidPrim` tensor API) — see
  `README_ISAACSIM6_FIXES.md` for the full root-cause writeup if you hit something
  similar after modifying vehicle/backend code.
- **Kit log flooded with `Could not send groundtruth through mavlink`** — a MAVLink
  field overflow (e.g. negative ground-jitter velocity packed into a `uint16` airspeed
  field). Already clamped in `px4_mavlink_backend.py`'s `update_state()` /
  `update_gps_data()`; if you see this again after editing sensor code, check the field
  ranges documented in `README_ISAACSIM6_FIXES.md`.
- **`Address already in use` on TCP port 4560 when relaunching** — a stale PX4 or Isaac
  Sim process still holds the `tcpin` socket. Kill lingering processes before
  relaunching; `examples/dpv_sim/stop.sh` does this for the DPV stack (`bin/px4`,
  `kit/python`, `MicroXRCEAgent`, `cartographer`, `px4_ros2_bridge`, `warehouse_*`,
  `QGroundControl`) and reports whether port 4560 was freed.
- **Pegasus extension fails to load in the Isaac Sim GUI with no visible panel** — Isaac
  Sim 6.0 moved `isaacsim.core.api` to `extsDeprecated/` and stopped auto-loading it;
  it must be declared as an explicit dependency in
  `extensions/pegasus.simulator/config/extension.toml` (already fixed in this fork —
  see `README_ISAACSIM6_FIXES.md` root cause 3 if you see this after editing
  `extension.toml`).

## Next Steps

- `docs/ARCHITECTURE.md` — how the vehicle/sensor/backend framework and the optional DPV
  ROS 2 stack fit together, with a component diagram and data-flow walkthrough.
- `docs/CONFIGURATION.md` — every configuration surface: `configs.yaml`, per-backend
  config dicts (`PX4MavlinkBackendConfig`, `ROS2Backend`), and the `PX4_PARAM_*`
  environment variables used by `examples/dpv_sim/start.sh`.
- `README_ISAACSIM6_FIXES.md` — detailed root-cause log for the Isaac Sim 5.1 -> 6.0
  migration issues, useful background if you are debugging similar force-application or
  MAVLink field-range problems.
- `examples/dpv_sim/README.md` — full phase-by-phase DPV bringup instructions with
  per-phase ROS 2 topic verification commands.
