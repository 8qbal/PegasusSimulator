# Pegasus Simulator

![IsaacSim 6.0](https://img.shields.io/badge/IsaacSim-6.0.0-brightgreen.svg)
![PX4-Autopilot 1.16.2](https://img.shields.io/badge/PX4--Autopilot-1.16.2-brightgreen.svg)
![ArduPilot-Copter 4.4](https://img.shields.io/badge/ArduPilot--Copter-4.4.0-brightgreen.svg)
![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04LTS-brightgreen.svg)
[![](https://dcbadge.limes.pink/api/server/[INVITE](https://discord.gg/AjCxw2QUmt?style=flat))](https://discord.gg/AjCxw2QUmt)

**Pegasus Simulator** is a framework built on top of [NVIDIA Omniverse](https://docs.omniverse.nvidia.com/) and [IsaacSim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html). It is designed to provide an easy yet powerful way of simulating the dynamics of vehicles. It provides a simulation interface for [PX4](https://px4.io/) and [ArduPilot](https://ardupilot.org/) integration, as well as a custom python control interface. At the moment, only multirotor vehicles are supported, with support for other vehicle topologies planned for future versions.

<p align = "center">
<a href="https://youtu.be/_11OCFwf_GE" target="_blank"><img src="docs/_static/pegasus_cover.png" alt="Pegasus Simulator image" height="300"/></a>
<a href="https://youtu.be/_11OCFwf_GE" target="_blank"><img src="docs/_static/mini demo.gif" alt="Pegasus Simulator gif" height="300"/></a>
</p>

Check the provided documentation [here](https://pegasussimulator.github.io/PegasusSimulator/) to discover how to install and use this framework.

## Tested Configuration (this fork)

This fork has been ported to and validated end-to-end (spawn → PX4 boot → EKF init → arm → takeoff) with:

| Component | Version |
|---|---|
| NVIDIA Isaac Sim | **6.0.0** (imports use `isaacsim.*`; physics via `isaacsim.core.experimental.prims`) |
| PX4-Autopilot | **v1.16.2** (SITL, TCP HIL link on port 4560 — the simulator is the `tcpin` server) |
| Ubuntu | 22.04 LTS |
| Python (Isaac bundled) | 3.12 |

⚠️ Notes for PX4 v1.16+: the `PX4_SIM_PROTOCOL` environment variable is no longer used by
PX4 — the HIL link is always TCP, so the default `connection_type` in
`PX4MavlinkBackendConfig` must remain `tcpin`. QGroundControl connects automatically on
UDP 14550 when running on the same machine.

## Latest Updates

⚠️ For users of versions prior to v5.1.0:
A new command line tool named `isaac_run` is now used to launch Isaac Sim. **This is a function that should be added to your .bashrc or .zshrc file during the installation of Isaac Sim.** See [Installation Instructions](https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html) for more details.

This was done to simplify the launching of Isaac Sim from the terminal with ROS2 support. All previous instructions that mentioned launching Isaac Sim examples from the examples folder using the `ISAACSIM_PYTHON` command should now use `isaac_run` instead.

Please refer to the updated documentation for more details.

> **ROS 2 environment note.** Isaac Sim bundles its own rclpy (built for Isaac's
> Python) via the `isaacsim.ros2.bridge` extension. If your shell sources a **system
> ROS 2** whose Python version differs from Isaac's (e.g. system Humble on py3.10 vs
> Isaac's py3.12), that system rclpy leaks onto `PYTHONPATH`/`LD_LIBRARY_PATH` and
> shadows Isaac's, crashing any example that imports rclpy with
> `ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`. Launch Isaac via
> [`scripts/isaac_run.sh`](scripts/isaac_run.sh), which strips `/opt/ros` and
> `ros2_ws` entries from the environment before starting Isaac (define
> `isaac_run() { "$HOME/PegasusSimulator/scripts/isaac_run.sh" "$@"; }` in your rc
> file). The system-side ROS 2 nodes — e.g. `examples/slam_v1/run_slam.sh` — still
> source system ROS 2 directly and are unaffected.

* **2025-10-26**: Pegasus Simulator v5.1.0 is released for Isaac 5.1.0. This version is **NOT** compatible with older versions of Isaac Sim. The Ardupilot experimental interface was not tested in this version. This update had an initial open-source contribution from [Victor Kallenbach](https://github.com/HO4X).
* **2026-06-23**: Pegasus Simulator v6.0.0 is released for Isaac 6.0. Updated all `omni.isaac.*` imports to `isaacsim.*` equivalents and removed deprecated Kit dependencies. The Ardupilot experimental interface was not tested in this version.
* **2025-10-25**: Pegasus Simulator v4.5.1 is released for Isaac 4.5.0. This version is **NOT** compatible with older versions of Isaac Sim. The Ardupilot experimental interface was fixed and improved by [Seunghwan Jo](https://github.com/SwiftGust) and [Tomer Tiplitsky](https://github.com/TomerTip).
* **2025-07-20**: Pegasus Simulator v4.5.0 is released for Isaac 4.5.0. This version is **NOT** compatible with older versions of Isaac Sim. The Ardupilot experimental interface was not tested in this version.
* **2024-11-01**: Pegasus Simulator v4.2.0 is released for Isaac 4.2.0. This version is **NOT** compatible with older versions of Isaac Sim. This version includes a new experimental interface for Ardupilot integration, provided by open-source contributor [Tomer Tiplitsky](https://github.com/TomerTip).
* **2024-08-02**: Pegasus Simulator v4.1.0 is released for Isaac 4.1.0. This version is **NOT** compatible with older versions of Isaac Sim.

## Citation

If you find Pegasus Simulator useful in your academic work, please cite the paper below. It is also available [here](https://doi.org/10.1109/ICUAS60882.2024.10556959).
```
@INPROCEEDINGS{10556959,
  author={Jacinto, Marcelo and Pinto, João and Patrikar, Jay and Keller, John and Cunha, Rita and Scherer, Sebastian and Pascoal, António},
  booktitle={2024 International Conference on Unmanned Aircraft Systems (ICUAS)}, 
  title={Pegasus Simulator: An Isaac Sim Framework for Multiple Aerial Vehicles Simulation}, 
  year={2024},
  volume={},
  number={},
  pages={917-922},
  keywords={Simulation;Robot sensing systems;Real-time systems;Sensor systems;Sensors;Task analysis},
  doi={10.1109/ICUAS60882.2024.10556959}}
```

## Main Developer Team

This simulation framework is an open-source effort, started by me, Marcelo Jacinto in January/2023. It is a tool that was created with the original purpose of serving my Ph.D. workplan for the next 4 years, which means that you can expect this repository to be mantained, hopefully at least until 2027.

* Project Founder
	* [Marcelo Jacinto](https://github.com/MarceloJacinto), under the supervision of <u>Prof. Rita Cunha</u> and <u>Prof. Antonio Pascoal</u> (IST/ISR-Lisbon)
* Architecture
  * [Marcelo Jacinto](https://github.com/MarceloJacinto)
  * [João Pinto](https://github.com/jschpinto)
* Multirotor Dynamic Simulation and Control
  * [Marcelo Jacinto](https://github.com/MarceloJacinto)
* Example Applications
	* [Marcelo Jacinto](https://github.com/MarceloJacinto)
	* [João Pinto](https://github.com/jschpinto)
* Ardupilot Integration (Experimental)
  * [Tomer Tiplitsky](https://github.com/TomerTip)
  * [Tanner Gilbert](https://github.com/TannerGilbert)
  * [Seunghwan Jo](https://github.com/SwiftGust)

Also check the always up-to-date [Github contributors list](https://github.com/PegasusSimulator/PegasusSimulator/graphs/contributors) with all the open-source contributors.

## Guidance, Control and Navigation Project

In parallel to this project, the Pegasus (GNC) guidance, control, and navigation project serves as the foundation control code for performing real-world experiments for my Ph.D. More information can be found at this link:
[Pegasus GNC](https://pegasusresearch.github.io/pegasus/)

## Project Roadmap

An high level project roadmap is available [here](https://pegasussimulator.github.io/PegasusSimulator/source/references/roadmap.html).

## Support and Contributing

We welcome new contributions from the community to improve this work. Please check the [Contributing](https://pegasussimulator.github.io/PegasusSimulator/source/references/contributing.html) section in the documentation for the guidelines on how to help improve and support this project.

* Use [Discussions](https://github.com/PegasusSimulator/PegasusSimulator/discussions) for discussing ideas, asking questions, and requests features.
* Use [Issues](https://github.com/PegasusSimulator/PegasusSimulator/issues) to track work in development, bugs and documentation issues.
* Use [Pull Requests](https://github.com/PegasusSimulator/PegasusSimulator/pulls) to fix bugs or contribute directly with your own ideas, code, examples or improve documentation.

## Licenses

Pegasus Simulator is released under [BSD-3 License](LICENSE). The license files of its dependencies and assets are present in the [`docs/licenses`](docs/licenses) directory.

NVIDIA Isaac Sim is available freely under [individual license](https://www.nvidia.com/en-us/omniverse/download/). 

PX4-Autopilot is available as an open-source project under [BSD-3 License](https://github.com/PX4/PX4-Autopilot).

## Project Sponsors
- Dynamics Systems and Ocean Robotics (DSOR) group of the Institute for Systems and Robotics (ISR), a research unit of the Laboratory of Robotics and Engineering Systems (LARSyS).
- Instituto Superior Técnico, Universidade de Lisboa

The work developed by Marcelo Jacinto and João Pinto was supported by Ph.D. grants funded by Fundação para a Ciência e Tecnologia (FCT).

<p float="left" align="center">
  <img src="docs/_static/dsor_logo.png" width="90" align="center" />
  <img src="docs/_static/logo_isr.png" width="200" align="center"/> 
  <img src="docs/_static/larsys_logo.png" width="200" align="center"/> 
  <img src="docs/_static/ist_logo.png" width="200" align="center"/> 
  <img src="docs/_static/logo_fct.png" width="200" align="center"/> 
</p>

## Repository Structure

* [`extensions/pegasus.simulator`](extensions/pegasus.simulator) — the core Isaac Sim extension. Its `pegasus/simulator/logic` package contains the vehicle dynamics, control `backends` (PX4/ArduPilot MAVLink, python control), `sensors`, `graphical_sensors`, `thrusters`, `graphs`, `people`/`people_backends`, and `vehicles` submodules that make up the simulation framework's public API.
* [`examples`](examples) — standalone runnable scripts demonstrating the framework, numbered roughly in order of complexity (e.g. `1_px4_single_vehicle.py`, `2_px4_multi_vehicle.py`, `3_ros2_single_vehicle.py`, `4_python_single_vehicle.py`, `8_camera_vehicle.py`, `9_people.py`, `11_ardupilot_multi_vehicle.py`, `12_px4_v1_vehicle.py`), plus `dpv_sim` and `slam_v1` subdirectories with more elaborate ROS 2 / navigation bringup setups.
* [`scripts/isaac_run.sh`](scripts/isaac_run.sh) — the launcher used by the `isaac_run` shell function to start Isaac Sim with a ROS-free environment so its bundled rclpy is used instead of any system ROS 2 install.
* [`docs`](docs) — Sphinx documentation sources, published at [pegasussimulator.github.io/PegasusSimulator](https://pegasussimulator.github.io/PegasusSimulator/).

## Quick Start

Once Isaac Sim and the `isaac_run` shell function are installed (see the [Installation Instructions](https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html)), run any example script with:

```bash
isaac_run examples/1_px4_single_vehicle.py
```

This launches Isaac Sim, loads the Pegasus Simulator extension, and spawns a single multirotor vehicle controlled through the PX4 MAVLink backend. Browse the [`examples`](examples) directory for other scenarios (multi-vehicle, ROS 2, camera sensors, ArduPilot, and the custom V1 vehicle).
