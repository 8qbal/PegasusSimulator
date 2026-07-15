<!-- generated-by: gsd-doc-writer -->
# Configuration

Pegasus Simulator is an Isaac Sim extension, so most configuration is done through Python
config objects passed to backends/vehicles/sensors at simulation-script authoring time,
plus one YAML file for install-wide defaults and a handful of shell environment variables
used by the `examples/dpv_sim/` DPV integration stack. There is no `.env` file convention
in this repository — configuration is expressed as Python dicts, YAML, and shell exports.

## Extension configuration file

`extensions/pegasus.simulator/config/configs.yaml` holds the extension-wide defaults that
`PegasusInterface` (`extensions/pegasus.simulator/pegasus/simulator/logic/interface/pegasus_interface.py`)
reads at startup. Its path is resolved as `CONFIG_FILE` in
`extensions/pegasus.simulator/pegasus/simulator/params.py` (`{extension_root}/pegasus.simulator/config/configs.yaml`).

Current contents:

```yaml
ardupilot_default_airframe: gazebo-iris
ardupilot_dir: ~/ardupilot
global_coordinates:
  altitude: 90.0
  latitude: -6.3653130531311035
  longitude: 106.82479858398438
px4_default_airframe: gazebo-classic_iris
px4_dir: ~/PX4-Autopilot
```

| Key | Required | Default (if file/key missing) | Description |
|---|---|---|---|
| `px4_dir` | Optional | `""` (empty string, logged as warning) | Path to a local PX4-Autopilot checkout. Expanded with `os.path.expanduser`. Read by `PegasusInterface.px4_path`, consumed by `PX4MavlinkBackendConfig` as `px4_dir` and by `PX4LaunchTool` when autolaunching PX4 SITL. |
| `ardupilot_dir` | Optional | `""` | Path to a local ArduPilot checkout, same resolution pattern, consumed by `ArduPilotMavlinkBackendConfig`. |
| `px4_default_airframe` | Optional | `""` | The PX4 SITL airframe/model string passed to `simulator_mavlink start` (e.g. `gazebo-classic_iris`). |
| `ardupilot_default_airframe` | Optional | `""` | The equivalent airframe string for ArduPilot SITL (e.g. `gazebo-iris`). |
| `global_coordinates.latitude` / `.longitude` / `.altitude` | Optional | Read via `_get_global_coordinates_from_config()` in `pegasus_interface.py` | The lat/lon/alt (degrees, degrees, meters) that the simulated world origin maps to — used to seed GPS/EKF origin. |

If `configs.yaml` cannot be parsed, each getter (`_get_px4_path_from_config`,
`_get_ardupilot_path_from_config`, etc. in `pegasus_interface.py`) logs a `carb.log_warn`
and falls back to an empty string rather than raising.

## World / physics settings

`WORLD_SETTINGS` and `DEFAULT_WORLD_SETTINGS` in
`extensions/pegasus.simulator/pegasus/simulator/params.py` define the physics/rendering
step sizes used when a simulation script constructs `World(**self.pg._world_settings)`.
Three named presets exist and `DEFAULT_WORLD_SETTINGS` points at the `px4` preset:

| Preset | `physics_dt` | `rendering_dt` | `stage_units_in_meters` | `device` |
|---|---|---|---|---|
| `px4` (default) | 1/250 s | 1/60 s | 1.0 | `cpu` |
| `ardupilot` | 1/800 s | 1/120 s | 1.0 | `cpu` |
| `ros2` | 1/250 s | 1/60 s | 1.0 | `cpu` |

The `ardupilot` preset runs physics at 800 Hz specifically to reach the 250 Hz
communication rate ArduPilot SITL expects (comment in `params.py`). These can be
overridden per-script via `PegasusInterface().set_world_settings(physics_dt=..., stage_units_in_meters=..., rendering_dt=..., device=...)`,
defined in `pegasus_interface.py`.

## PX4 MAVLink backend configuration (`PX4MavlinkBackendConfig`)

Defined in `extensions/pegasus.simulator/pegasus/simulator/logic/backends/px4_mavlink_backend.py`.
Passed as a dict to `PX4MavlinkBackendConfig(config={...})` when constructing a vehicle's backend list
(see `examples/12_px4_v1_vehicle.py`).

| Key | Default | Description |
|---|---|---|
| `vehicle_id` | `0` | Added to `connection_baseport` to derive the per-vehicle MAVLink port. |
| `connection_type` | `"tcpin"` | **Must stay `tcpin`.** PX4 SITL (>= v1.16) connects to the simulator as a TCP client running `simulator_mavlink start -c <port>`; the simulator must be the TCP server. `PX4_SIM_PROTOCOL` has no effect on PX4 >= v1.16 — using `udp`/`udpout` here silently prevents the HIL sensor link from ever connecting (see the in-code comment on lines 218-222). |
| `connection_ip` | `"localhost"` | Bind/connect address for the MAVLink TCP server. |
| `connection_baseport` | `4560` | Base TCP port; actual port is `connection_baseport + vehicle_id`. |
| `px4_autolaunch` | `True` | Whether Pegasus launches PX4 SITL as a background process automatically via `PX4LaunchTool`. |
| `px4_dir` | `PegasusInterface().px4_path` | PX4-Autopilot checkout directory; defaults to the `px4_dir` value from `configs.yaml`. |
| `px4_vehicle_model` | `"gazebo-classic_iris"` | PX4 SITL airframe/model name. |
| `enable_lockstep` | `True` | Whether sensor/actuator messages are exchanged in lockstep with PX4. |
| `num_rotors` | `4` | Number of rotors the backend expects control data for. |
| `input_offset` | `[0.0, 0.0, 0.0, 0.0]` | Per-rotor offset applied to incoming PX4 actuator control values. |
| `input_scaling` | `[1000.0, 1000.0, 1000.0, 1000.0]` | Per-rotor scale applied to incoming PX4 actuator control values. |
| `zero_position_armed` | `[100.0, 100.0, 100.0, 100.0]` | Per-rotor offset added after scaling (rest position when armed). |
| `update_rate` | `250.0` (Hz) | Rate at which sensor/heartbeat/ground-truth MAVLink messages are sent. |

The port-leak/reconnect handling in `PX4MavlinkBackend.update()` explicitly closes the
`tcpin` socket before dropping the connection reference on a failed heartbeat send —
otherwise reconnects fail with `Address already in use` on port 4560 until the whole
process restarts (see the comment above `send_heartbeat` error handling in
`px4_mavlink_backend.py`).

## ArduPilot MAVLink backend configuration (`ArduPilotMavlinkBackendConfig`)

Defined in `extensions/pegasus.simulator/pegasus/simulator/logic/backends/ardupilot_mavlink_backend.py`.
Mirrors the PX4 backend's config surface with ArduPilot-specific values:

| Key | Default | Notes |
|---|---|---|
| `connection_baseport` | `14550` | Different default port scheme than PX4's `4560`; alternates of `14551` / `5760` appear commented out in source. |
| `connection_type`, `connection_ip`, `ardupilot_autolaunch`, `ardupilot_dir`, `ardupilot_vehicle_model`, `enable_lockstep`, `num_rotors`, `input_offset`, `input_scaling`, `zero_position_armed`, `update_rate` | — | Same role as the equivalent PX4 keys, scoped to ArduPilot SITL. |

The final connection port is computed as `connection_baseport + vehicle_id * 10` (note the
`* 10` multiplier, unlike the PX4 backend's `+ vehicle_id`).

## ROS 2 backend configuration (`ROS2Backend`)

Defined in `extensions/pegasus.simulator/pegasus/simulator/logic/backends/ros2_backend.py`.
Constructed as `ROS2Backend(vehicle_id, num_rotors=4, config={...})`. Key options (see the
class docstring for the full dict):

| Key | Default | Description |
|---|---|---|
| `namespace` | `"drone" + str(vehicle_id)` | Topic namespace prefix. |
| `pub_pose`, `pub_twist`, `pub_twist_inertial`, `pub_accel`, `pub_imu`, `pub_mag`, `pub_gps`, `pub_gps_vel` | `True` | Toggle publishing of each ground-truth/sensor topic. |
| `pose_topic`, `twist_topic`, `twist_inertial_topic`, `accel_topic`, `imu_topic`, `mag_topic`, `gps_topic`, `gps_vel_topic` | `"state/pose"`, `"state/twist"`, `"state/twist_inertial"`, `"state/accel"`, `"sensors/imu"`, `"sensors/mag"`, `"sensors/gps"`, `"sensors/gps_twist"` | Topic names, relative to `namespace`. |
| `pub_graphical_sensors` | `True` | Publish camera/lidar data configured on the vehicle. |
| `pub_sensors` | `True` | Publish flight sensor data. |
| `pub_state` | `True` | Publish vehicle state topics. |
| `pub_tf` | `False` | Publish the ground-truth TF tree (`map -> {namespace}_base_link`); requires `tf2_ros` to be importable, otherwise silently forced `False`. |
| `sub_control` | `True` | Subscribe to control-input topics. |

## Vehicle / sensor configuration (V1 example)

`extensions/pegasus.simulator/pegasus/simulator/logic/vehicles/multirotors/v1.py`
(`V1Config`) shows the pattern used to configure a vehicle's thrust curve, drag, sensors,
and graphical sensors in Python:

- `V1Config(enable_zed_camera: bool = True)` — toggles whether the ZED2i `MonocularCamera`
  graphical sensor is attached. Disabling it (used for localization-only testing in
  `examples/dpv_sim/`) removes the most expensive per-frame render cost and improves
  real-time factor (RTF) and EKF2 convergence stability, at the cost of no `/zed/*` topics.
- `MonocularCamera("zed2i", config={...})` accepts `position`, `resolution`, `frequency`,
  `intrinsics`, `depth`, `color_topic` (`/zed/image_raw`), `depth_topic` (`/zed/depth`),
  `camera_info_topic` (`/zed/color/camera_info`), `camera_frame_id` (`zed2i_camera_link`).
- `Lidar("rplidar_c1", config={...})` accepts `position`, `sensor_configuration`
  (`"Example_Rotary_2D"` — the RTX lidar profile `RPLIDAR_S2E` produces zero output in
  Isaac Sim 6.0, per the in-code comment), `frequency`, `show_render`, `scan_topic`
  (`/scan`), `scan_frame_id` (`laser_link`).
- `QuadraticThrustCurve(config={...})` accepts `num_rotors`, `rotor_constant`,
  `rolling_moment_coefficient`, `rot_dir`, `min_rotor_velocity`, `max_rotor_velocity` — the
  V1 preset models 2.8 kgf max thrust per motor at 1100 rad/s.

## Environment variables (DPV simulation stack, `examples/dpv_sim/`)

The DPV integration scripts and launch files pass configuration through shell
environment variables rather than a `.env` file. All are consumed by
`examples/12_px4_v1_vehicle.py` or the PX4 SITL process (`px4_launch_tool.py` passes
`os.environ` through when spawning PX4).

| Variable | Set by | Consumed by | Description |
|---|---|---|---|
| `DPV_GUIDANCE_MODE` | `examples/dpv_sim/start.sh` (`export DPV_GUIDANCE_MODE=1` for phase `g`, `=0` otherwise) | `examples/12_px4_v1_vehicle.py` (`GUIDANCE_MODE = os.environ.get("DPV_GUIDANCE_MODE") == "1"`) | When `1`, attaches a `GPS()` sensor to the vehicle so `HIL_GPS` reaches PX4 for guidance-mode (GPS-fused) flight. When `0`/unset, GPS-denied phases 1-3 omit the GPS sensor so only vision/laser can feed PX4's position estimate. |
| `PX4_PARAM_EKF2_GPS_CTRL` | `start.sh` | PX4 SITL `rcS` boot script (`param set`) | EKF2 GPS control bitmask. GPS-denied phases: `0`. Guidance mode: `7` (PX4 firmware default: 2D pos + vel + hgt fusion). |
| `PX4_PARAM_EKF2_EV_CTRL` | `start.sh` | PX4 SITL `rcS` boot script | EKF2 external-vision control bitmask (bit 0=horiz pos, 1=vert pos, 2=velocity, 3=yaw). GPS-denied phases: `1` (horizontal position only — vertical/yaw EV are unreliable with the 2D laser SLAM). Guidance mode: `0`. |
| `PX4_PARAM_EKF2_HGT_REF` | `start.sh` | PX4 SITL `rcS` boot script | Height reference source; `0` (baro) in both modes. |
| `PX4_PARAM_EKF2_EV_DELAY` | `start.sh` (GPS-denied phases only) | PX4 SITL `rcS` boot script | External-vision fusion delay compensation, set to `50` (ms). |
| `PX4_PARAM_EKF2_MAG_CHECK` | `start.sh` | PX4 SITL `rcS` boot script | Magnetometer field-strength/inclination gate. GPS-denied phases: `0` (disabled — the sim's magnetic field model differs from the hardcoded validation average). Guidance mode: `1` (enabled). |

Any `PX4_PARAM_<NAME>=<value>` environment variable is applied by the PX4 SITL `rcS`
startup script as a `param set` before EKF2 starts, so no manual parameter script or PX4
reboot is required when launched via `start.sh` (comment in `start.sh` lines 45-47).

### Onboard-link parameter script (manual fallback)

`examples/dpv_sim/set_px4_gps_denied_params_onboard.py` sets the same GPS-denied EKF2
parameters manually over the PX4 **onboard** MAVLink link, for use when not launching via
`start.sh` (e.g. running Phase 1 bringup by hand, per `examples/dpv_sim/README.md`).

| Constant | Value | Notes |
|---|---|---|
| `ONBOARD_URL` | `udpin:127.0.0.1:14540` | QGroundControl latches the GCS link (UDP 18570), so parameter changes must go through the separate onboard link instead. <!-- VERIFY: 18570 GCS port is referenced only in a code comment, not read from a config file in this repo --> |
| `PARAMS['EKF2_GPS_CTRL']` | `0` | Disable GPS fusion. |
| `PARAMS['EKF2_EV_CTRL']` | `1` | Horizontal position only. |
| `PARAMS['EKF2_HGT_REF']` | `0` | Baro height reference. |
| `PARAMS['EKF2_EV_DELAY']` | `50` | EV fusion delay (ms). |
| `PARAMS['EKF2_MAG_CHECK']` | `0` | Disable mag field-strength gate. |

Parameters persist in PX4's EEPROM and require a PX4 reboot to take effect; the script
sends `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` automatically after setting all parameters.

## ROS 2 launch file arguments (`examples/dpv_sim/*.launch.py`)

Each bringup launch file declares its own `launch.actions.DeclareLaunchArgument`s:

| File | Argument | Default | Description |
|---|---|---|---|
| `isaac_nav_bringup.launch.py` | `use_sim_time` | `true` | Passed to every node so ROS 2 uses Isaac's published `/clock`. |
| `isaac_nav_bringup_phase2.launch.py` | `use_sim_time` | `true` | Same as above. |
| `isaac_nav_bringup_phase2.launch.py` | `mission_file` | `~/ros2_ws/src/warehouse_gz_sim_ws/mission_files/0001_0001_0001.json` | Path to the mission JSON consumed by `warehouse_auto_mission`. |
| `isaac_nav_bringup_phase3.launch.py` | `use_sim_time` | `true` | Same as above. |
| `isaac_nav_bringup_phase3.launch.py` | `launch_vision` | `false` | When `true`, adds the `zed_vio_stub` node and `warehouse_pose_corrector_vision_based`; when `false` (default), the vision corrector is skipped entirely, matching Gazebo sim behavior. |
| `isaac_nav_bringup_phase3.launch.py` | `mission_file` | Same default as phase 2 | Same role as phase 2's `mission_file`. |
| `isaac_nav_bringup_guidance.launch.py` | `use_sim_time` | `true` | Same as above. |
| `isaac_nav_bringup_guidance.launch.py` | `mission_file` | Same default as phase 2/3 | Same role. |

The `px4_ros2_bridge` node parameters are loaded from `bridge_params.yaml`, resolved via
`get_package_share_directory('px4_ros2_bridge')` at launch time — this is a ROS 2 package
install artifact, not a file tracked in this repository.
<!-- VERIFY: exact contents/tunable keys of bridge_params.yaml — the file lives in the built ROS 2 workspace (e.g. ~/ros2_ws/install/.../share/px4_ros2_bridge/config/bridge_params.yaml), not in this repo -->

## `start.sh` / `stop.sh` configuration

`examples/dpv_sim/start.sh` takes a single positional argument selecting which bringup to
launch in a `tmux` session named `dpv-sim`:

| Argument | Bringup launched |
|---|---|
| `1` (default) | `isaac_nav_bringup.launch.py` — localization only |
| `2` | `isaac_nav_bringup_phase2.launch.py` — + mission/navigation |
| `3` | `isaac_nav_bringup_phase3.launch.py` — + vision corrector |
| `g` | `isaac_nav_bringup_guidance.launch.py` — GPS-fused guidance mode |

Hardcoded paths inside `start.sh` (edit these variables directly if your install differs
from the author's local layout):

| Variable | Value |
|---|---|
| `REPO` | `$HOME/PegasusSimulator` |
| `DPV_INSTALL` | `$REPO/extensions/dpv-install` |
| `PX4_HOME` | `$HOME/PX4-Autopilot` |
| `QGC` | `$HOME/Downloads/QGroundControl-x86_64.AppImage` — QGroundControl launch is skipped with a warning if this AppImage is not found. |

`examples/dpv_sim/stop.sh` takes no arguments; it kills all Pegasus/DPV-related
background processes by name pattern (`bin/px4`, `kit/python`, `MicroXRCEAgent`,
`cartographer`, `px4_ros2_bridge`, `warehouse_*`, `QGroundControl`, etc.) and reports
whether TCP port `4560` was freed.

## Required external tooling (not configured via files in this repo)

The following must be installed/available on `$PATH` or at the hardcoded paths above; none
are pinned by a lockfile or manifest in this repository:

- `tmux` — required by `start.sh`.
- `MicroXRCEAgent` — required for the PX4 ROS 2 uXRCE-DDS bridge; launched with `udp4 -p 8888`.
- `ros-humble-cartographer-ros` — installed via `apt`, per `examples/dpv_sim/README.md`.
- A local PX4-Autopilot checkout with `dds_topics.yaml` patched to export `/fmu/out/vehicle_imu` (per `examples/dpv_sim/README.md` prerequisites). <!-- VERIFY: exact patch/diff not present in this repository -->
- The `extensions/dpv-install/setup.bash` DPV ROS 2 workspace overlay, sourced after `/opt/ros/humble/setup.bash`.
