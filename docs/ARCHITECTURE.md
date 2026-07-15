<!-- generated-by: gsd-doc-writer -->

# Architecture

## System Overview

Pegasus Simulator is a Python framework built on top of NVIDIA Omniverse / Isaac Sim that
simulates the flight dynamics of multirotor aerial vehicles and bridges them to real
autopilot firmware (PX4 or ArduPilot SITL) over MAVLink, and/or to ROS 2. At its core, a
`Vehicle` object owns a set of sensors, graphical sensors, OmniGraph graphs, and
communication **backends**; every physics step the vehicle reads its simulated state,
feeds it to each backend, and applies the forces/torques the active backend's rotor
commands imply.

This fork (`PegasusSimulator` at `/home/web-scientia/PegasusSimulator`) extends the
upstream framework with a full integration of a real drone autonomy stack — the "DPV"
(warehouse delivery vehicle) stack — so that PX4 SITL flying inside Isaac Sim can drive
the same Cartographer SLAM, PX4↔ROS 2 bridge, warehouse mission/path-planning/trajectory
nodes, and pose-correction pipeline that runs on the physical vehicle, instead of Gazebo.
That stack lives as a colcon workspace under `extensions/dpv-build` (build artifacts) and
`extensions/dpv-install` (installed ROS 2 packages), orchestrated by launch files and a
`tmux`-based `start.sh` under `examples/dpv_sim/`.

## Component Diagram

```
                        ┌─────────────────────────────────────────────┐
                        │              Isaac Sim process               │
                        │  (examples/12_px4_v1_vehicle.py entry point)  │
                        │                                               │
                        │  PegasusInterface (singleton) ── World        │
                        │        │                                     │
                        │        ▼                                     │
                        │  Multirotor "V1" (extensions/pegasus.simulator)│
                        │   ├─ sensors: Barometer, IMU, Magnetometer,   │
                        │   │            GPS (guidance mode only)       │
                        │   ├─ graphical_sensors: MonocularCamera(ZED), │
                        │   │            Lidar(RPLIDAR C1, "/scan")     │
                        │   ├─ thrusters: QuadraticThrustCurve          │
                        │   ├─ dynamics: LinearDrag                     │
                        │   └─ backends:                                │
                        │        ├─ PX4MavlinkBackend ──────────┐       │
                        │        └─ ROS2Backend ("v1_" ns) ──┐  │       │
                        └──────────────────────────────────┼──┼───────┘
                                                             │  │ MAVLink (HIL_SENSOR,
                                                             │  │  HIL_GPS, HIL_STATE_QUATERNION)
                                                             │  │  TCP :4560 (tcpin)
                        camera / lidar / TF / state topics   │  ▼
                                    ▼                        │  PX4 SITL process
                        ┌───────────────────────┐            │  (PX4-Autopilot, autolaunched
                        │   ROS 2 graph          │            │   by PX4LaunchTool)
                        │  (Humble, system-wide) │            │        │
                        │                        │            │        │ uORB / uXRCE-DDS
                        │  px4_ros2_bridge nodes │◄───────────┘        ▼
                        │  (extensions/dpv-install)│           MicroXRCEAgent (udp4 :8888)
                        │                        │
                        │  cartographer_slam_    │
                        │  wrapper (Phase 1)     │
                        │                        │
                        │  warehouse_auto_mission│
                        │  warehouse_path_planner│
                        │  trajectory_generator  │  (Phase 2 / guidance mode)
                        │                        │
                        │  warehouse_pose_       │
                        │  corrector_{laser,     │  (Phase 3, optional)
                        │  vision}_based         │
                        └───────────────────────┘
```

Data flows from Isaac Sim (vehicle physics + sensors) out through two independent
channels — MAVLink to PX4 SITL, and ROS 2 topics to the DPV autonomy stack — which are
then cross-connected by `px4_ros2_bridge` nodes that translate PX4 uORB topics (received
via uXRCE-DDS/MicroXRCEAgent) to/from standard ROS 2 messages the rest of the stack
consumes.

## Data Flow

There are two data paths in the system, selected by which `examples/dpv_sim/*.launch.py`
file (or `start.sh` phase) is running.

**1. Core simulation → PX4 loop (always active, `examples/12_px4_v1_vehicle.py`):**
1. `Vehicle._update_sensors_safe` (physics callback, `extensions/pegasus.simulator/.../logic/vehicles/vehicle.py`)
   calls each sensor's `update()` every physics step and forwards new data to every
   backend's `update_sensor()`.
2. `PX4MavlinkBackend` (`logic/backends/px4_mavlink_backend.py`) accumulates sensor data
   into a `SensorMsg` and streams `HIL_SENSOR` / `HIL_GPS` / `HIL_STATE_QUATERNION`
   MAVLink messages to the PX4 SITL process it auto-launched via `PX4LaunchTool`
   (`logic/backends/tools/px4_launch_tool.py`), which listens as a `tcpin` server on port
   4560 (see `README.md` "Tested Configuration").
3. PX4 SITL computes actuator outputs and returns them over the same MAVLink link;
   `PX4MavlinkBackend.input_reference()` supplies the resulting per-rotor angular
   velocities to `Multirotor.update()` (`logic/vehicles/multirotor.py`), which converts
   them to thrust via `QuadraticThrustCurve` and applies forces/torques through
   `Vehicle.apply_force()` / `apply_torque()` (PhysX `RigidPrim` tensor API).
4. In parallel, `ROS2Backend` (`logic/backends/ros2_backend.py`) publishes the vehicle's
   ZED camera (`/zed/image_raw`, `/zed/depth`, `/zed/color/camera_info`) and RPLIDAR scan
   (`/scan`) topics, plus optional pose/twist/accel/TF state topics, for consumption by
   the DPV stack or any other ROS 2 node.

**2. DPV localization/navigation loop (Phase 1–3 or guidance mode, driven by
`examples/dpv_sim/isaac_nav_bringup*.launch.py`):**
- **Phase 1 (localization):** `/scan` (lidar) feeds `cartographer_slam_wrapper`, which
  publishes `/cartographer/laser_odom_at_fcu`. The `px4_ros2_bridge`
  `to_fcu_vehicle_visual_odometry_node` republishes this as PX4 external-vision odometry
  (`/fmu/in/vehicle_visual_odometry`), while `from_fcu_vehicle_odometry_node` and
  `from_fcu_vehicle_local_position_node` pull PX4's own local-position/odometry estimate
  back out (`/fmu/out/...`) for Cartographer's odom prior. PX4's EKF2 fuses the external
  vision instead of GPS (GPS-denied EKF2 params are set by
  `examples/dpv_sim/set_px4_gps_denied_params_onboard.py` or via `PX4_PARAM_*` env vars
  in `start.sh`).
- **Phase 2 (mission + navigation):** adds `warehouse_auto_mission` (mission state
  machine), `warehouse_path_planner`, and `trajectory_generator`, wired to PX4 through
  additional `px4_ros2_bridge` relay nodes (`from_fcu_status_relay_node`,
  `to_fcu_command_relay_node`, `to_fcu_trajectory_relay_node`).
- **Phase 3 (vision correction, optional):** the ZED camera topics are consumed by
  `warehouse_pose_corrector_vision_based`, with `examples/dpv_sim/zed_vio_stub.py`
  standing in for a full ZED VIO node (relaying Cartographer odom into the expected VIO
  topic) since the Stereolabs Isaac Sim extension is not installed.
- **Guidance mode (`start.sh g`, `DPV_GUIDANCE_MODE=1`):** bypasses SLAM/EV entirely.
  `examples/12_px4_v1_vehicle.py` adds a `GPS` sensor so `PX4MavlinkBackend` sends
  `HIL_GPS`, PX4's EKF2 fuses GPS directly (`EKF2_GPS_CTRL=7`, `EKF2_EV_CTRL=0`), and the
  same mission/planner/trajectory nodes run against `px4_ros2_bridge`'s
  GPS-derived odometry topics (`fcu_pose_at_imu`, `fcu_odom_flu`) instead of vision.

## Key Abstractions

| Abstraction | Location | Purpose |
|---|---|---|
| `PegasusInterface` | `logic/interface/pegasus_interface.py` | Singleton entry point; owns the Isaac Sim `World`, reads `pegasus.simulator` extension config (PX4/ArduPilot paths, default airframe, sim-origin lat/lon/alt). |
| `VehicleManager` | `logic/vehicle_manager.py` | Singleton registry of all spawned `Vehicle` instances, keyed by stage prefix. |
| `Vehicle` | `logic/vehicles/vehicle.py` | Base class for anything spawned into the stage as a controllable robot. Owns sensors, graphical sensors, graphs, and backends; drives their lifecycle via Isaac Sim physics/render/timeline callbacks (`sim_start_stop`, `_update_state_safe`, etc.). |
| `Multirotor` / `MultirotorConfig` | `logic/vehicles/multirotor.py` | Vehicle specialization for quadrotor-style aircraft; implements rotor force allocation (`force_and_torques_to_velocities`) and per-step thrust/drag application. |
| `V1` / `V1Config` | `logic/vehicles/multirotors/v1.py` | This fork's concrete vehicle: the real DPV drone (0.85 m span, 2.0 kg, 2.8 kgf/motor) with ZED 2i camera + RPLIDAR C1 sensors and a `[PX4MavlinkBackend, ROS2Backend]` backend pair. |
| `Backend` / `BackendConfig` (ABC) | `logic/backends/backend.py` | Contract every communication/control backend implements: `update_sensor`, `update_graphical_sensor`, `update_state`, `input_reference`, `update`, `start`/`stop`/`reset`. |
| `PX4MavlinkBackend` | `logic/backends/px4_mavlink_backend.py` | Streams simulated sensor data to PX4 SITL as MAVLink `HIL_*` messages and reads back rotor commands; can auto-launch the PX4 process via `PX4LaunchTool`. |
| `ROS2Backend` | `logic/backends/ros2_backend.py` | Publishes vehicle state, sensors, and graphical sensor data (camera/lidar) as ROS 2 topics/TF for consumption by external ROS 2 stacks such as DPV. |
| `ArduPilotMavlinkBackend` | `logic/backends/ardupilot_mavlink_backend.py` | Experimental equivalent of `PX4MavlinkBackend` for ArduPilot Copter SITL. |
| `State` | `logic/state.py` | Plain data holder for a vehicle's current position/attitude/velocity/acceleration, updated once per physics step and read by every backend. |
| `Sensor` implementations | `logic/sensors/{barometer,gps,imu,magnetometer}.py` | Simulated flight sensors that convert `State` into MAVLink-ready sensor readings. |
| Graphical sensors | `logic/graphical_sensors/{lidar,monocular_camera}.py` | RTX-rendered sensors (camera, lidar) wired to Isaac Sim's Replicator/OmniGraph pipeline and published over ROS 2. |
| `QuadraticThrustCurve` | `logic/thrusters/quadratic_thrust_curve.py` | Converts rotor angular velocity to thrust/torque per rotor, configured per-vehicle (V1's real thrust curve is set in `V1Config`). |
| `LinearDrag` | `logic/dynamics/linear_drag.py` | Simple body-frame linear drag model applied every physics step. |

**DPV ROS 2 stack abstractions** (`extensions/dpv-install/`, installed colcon packages —
built from source outside this repo, per project memory):

| Package | Role |
|---|---|
| `px4_ros2_bridge` | DDS bridge translating PX4 uORB topics to/from ROS 2 (`from_fcu_*_node`, `to_fcu_*_node` executables — local position, odometry, status relay, command relay, trajectory relay, visual/laser odometry, payload/vehicle correction). |
| `cartographer_slam_wrapper` | Wraps `cartographer_ros` for 2D laser SLAM against the lidar `/scan` topic. |
| `warehouse_auto_mission` | Mission state machine (`INIT → IDLE → READY → ... → AUTO_MISSION → MISSION_COMPLETED`) driven by `MissionCommand` messages. |
| `warehouse_path_planner` | Converts mission waypoints into a planned path. |
| `trajectory_generator` | Converts a planned path into a time-parameterized trajectory sent to PX4. |
| `warehouse_pose_corrector_laser_based` / `warehouse_pose_corrector_vision_based` | Optional pose-correction filters (Phase 3) refining localization with lidar or ZED VIO. |
| `warehouse_ros2_msgs` | Custom message definitions (e.g. `MissionCommand`) shared across the stack. |
| `px4_msgs` | PX4 uORB message definitions generated for ROS 2. |
| `zed_wrapper` / `zed_components` / `zed_ros2` | ZED camera ROS 2 driver packages (VIO normally provided by the Stereolabs Isaac Sim extension, which is not installed — see `examples/dpv_sim/zed_vio_stub.py`). |
| `rplidar_ros`, `rviz2_plugin`, `log_recorder`, `trajectory_generator`, `wsc_launch_utils` | Supporting drivers, visualization, logging, and launch utilities for the real hardware stack, reused unmodified against the simulated topics. |

## Directory Structure Rationale

```
PegasusSimulator/
├── extensions/
│   ├── pegasus.simulator/     # The core Isaac Sim extension (Python package "pegasus")
│   │   └── pegasus/simulator/
│   │       ├── logic/         # Vehicles, sensors, backends, dynamics, thrusters, graphs
│   │       ├── assets/        # Backend/Robot/World USD and config assets
│   │       ├── ui/            # Isaac Sim extension UI panel
│   │       ├── parser/        # Config/robot description parsing
│   │       └── config/        # Extension-level config (PX4/ArduPilot paths, etc.)
│   ├── dpv-build/             # colcon build artifacts for the DPV ROS 2 workspace
│   └── dpv-install/           # colcon install space (setup.bash sourced by DPV bringup)
├── examples/                  # Standalone Isaac Sim entry-point scripts (0_template_app.py … 12_px4_v1_vehicle.py)
│   ├── dpv_sim/                # DPV bringup: launch files per phase, start.sh/stop.sh, PX4 param scripts
│   └── slam_v1/                 # Earlier SLAM-only pipeline scripts/bridges predating the DPV integration
├── models/                    # Source V1 vehicle model assets (V1.blend, V1.glb)
├── scripts/                   # Shell helpers, notably isaac_run.sh (launches Isaac with system ROS 2 stripped from env)
├── docs/                      # Upstream Sphinx documentation source (docs/source/*) plus this file
└── README.md                  # Project overview, tested-configuration matrix, and quick links
```

- **`extensions/pegasus.simulator/`** is the reusable simulation framework: vehicle
  physics, sensor models, and MAVLink/ROS 2 backends, independent of any specific
  autonomy stack.
- **`extensions/dpv-build/` and `extensions/dpv-install/`** are a colcon ROS 2 workspace
  that was built from the real DPV vehicle's source packages (kept elsewhere on disk per
  project notes) so the exact same autonomy nodes that run on hardware can run against
  Isaac Sim's simulated sensor topics instead of Gazebo's.
- **`examples/`** hosts runnable entry points; `examples/12_px4_v1_vehicle.py` is the one
  used for DPV work — it spawns the `V1` vehicle with either GPS-denied sensors (Phases
  1–3) or GPS enabled (guidance mode), selected via the `DPV_GUIDANCE_MODE` environment
  variable.
- **`examples/dpv_sim/`** contains only ROS 2 launch/orchestration code (no Isaac Sim
  imports) — it assumes Isaac Sim and PX4 SITL are already running and wires up the DPV
  ROS 2 graph on top of them.
- **`models/`** holds the source V1 mesh (used to author the USD robot referenced by
  `V1Config.usd_file`), kept separate from the generated USD assets under
  `extensions/pegasus.simulator/pegasus/simulator/assets/Robots`.
