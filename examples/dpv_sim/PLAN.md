# Plan: Run the real DPV warehouse autonomy stack against Isaac Sim (V1 vehicle)

**Goal:** Replace Gazebo with Isaac Sim as the simulator underneath the company's real
drone software (Cartographer SLAM, warehouse pose correctors, path planner,
px4_ros2_bridge, auto-mission), so the V1 drone localizes and navigates GPS-denied in the
Isaac warehouse using the SAME packages that fly the real drone. The autonomy packages
must NOT be modified — Isaac must produce the same topic/frame contract Gazebo did.

**Executor notes:** every "verify" step matters — this pipeline fails silently at many
layers. Read the *Environment gotchas* section BEFORE running anything. All paths are
absolute for this machine (`web-scientia`).

---

## 1. Ground truth — where everything lives

| Thing | Path | Notes |
|---|---|---|
| Pegasus/Isaac repo | `~/PegasusSimulator` | working tree has UNCOMMITTED sensor fixes (see §3) — commit them first |
| Isaac entry example | `~/PegasusSimulator/examples/12_px4_v1_vehicle.py` | V1 in warehouse, PX4 autolaunch |
| Isaac launcher (mandatory) | `~/PegasusSimulator/scripts/isaac_run.sh` | scrubs system ROS from env; NEVER launch Isaac another way |
| **Real stack (usable install)** | `~/Downloads/install_nosymlink2/install/` | non-symlink colcon install; `setup.bash` works. Call it `$DPV` |
| Real stack (2nd copy) | `~/Downloads/Install wo sylink/` | backup copy of the same |
| `~/ros2_ws/install` | **DEAD** for these pkgs | symlink-install pointing at `/home/novian/...` which does not exist here. Do NOT rely on it for the warehouse packages |
| `extensions/dpv-build/` | build artifacts only | CMake cache, no launch files, has COLCON_IGNORE. Ignore it |
| **Gazebo sim reference** | `~/ros2_ws/src/warehouse_gz_sim_ws/` | THE contract spec: `components/*/config.env` + `components/*/launcher.sh` define exactly what the sim must publish and how the stack is brought up |
| PX4 | `~/PX4-Autopilot` (v1.16, vanilla) | SITL, autolaunched by example 12 via tcpin:4560 |
| QGroundControl | may already be running | owns UDP 14550 + latches PX4's GCS link 18570 (see gotchas) |

Source environment for every stack terminal:
```bash
source /opt/ros/humble/setup.bash
source "$HOME/Downloads/install_nosymlink2/install/setup.bash"   # $DPV
```
(Isaac terminal uses NONE of this — `scripts/isaac_run.sh` handles its env.)

---

## 2. The real stack's architecture (as verified from the installs)

Localization data flow on the real drone:

```
RPLIDAR S3 driver (rplidar_ros, frame laser_link, topic /scan, 20 Hz UltraDense)
     └─ its launch (rplidar_s3_launch_lan.py) ALSO publishes static TFs and includes Cartographer:
         static TF: base_link_fcu -> laser_link           (0.21 0 0.13  -3.1415926 0 0)   # positional args = x y z YAW PITCH ROLL
         static TF: laser_link -> fcu_imu_base_link_for_laser (0.21 0 -0.13  3.1415926 0 0)
PX4 --uXRCE-DDS--> /fmu/out/vehicle_imu --px4_imu_converter--> /imu (frame fcu_imu_base_link_for_laser)
/scan + /imu --> cartographer_node  (cartographer_2d.lua: tracking_frame=laser_link,
                 published_frame=laser_link, odom_frame=odom_laser, provide_odom_frame=true,
                 use_odometry=false  <-- laser+IMU only, NO external odom needed)
cartographer --> cartographer_pose_transformer --> /cartographer/odom, /cartographer/laser_odom_at_fcu,
                 /cartographer/laser_pose_at_fcu, /cartographer/laser_raw_pose
px4_ros2_bridge (bridge.launch.py):
    static TF map -> odom_fcu (identity)
    from_fcu_vehicle_local_position: /fmu/out/{vehicle_local_position,vehicle_attitude}
        -> /px4_ros2_bridge/pose/fcu, /px4_ros2_bridge/odometry/fcu_odom_flu, .../fcu_pose_at_imu
    to_fcu_vehicle_visual_odometry: INPUT /zed/zed_node/odom_zed_to_fcu (ZED VIO!)
        -> /fmu/in/vehicle_visual_odometry            <-- PX4 EV input
    to_fcu_vehicle_laser_odometry: INPUT /cartographer/laser_odom_at_fcu
        -> /fmu/in/vehicle_laser_odometry_raw          <-- CUSTOM topic; only their PX4 FORK consumes it
correctors / navigation (Phase 2): warehouse_pose_corrector_laser_based,
    warehouse_pose_correction_filter, warehouse_path_planner, trajectory_generator,
    warehouse_auto_mission (+ payload/bin/mission nodes)
```

**Two critical consequences for the sim:**
1. On the real drone, PX4's external-vision comes from **ZED VIO**, and Cartographer's laser
   odom goes to a **fork-only** PX4 topic. Our SITL is **vanilla v1.16**: `/fmu/in/vehicle_laser_odometry_raw`
   does not exist in its `dds_topics.yaml`, and there is no ZED VIO in Phase 1.
   → **Phase 1 must route Cartographer's `/cartographer/laser_odom_at_fcu` into
   `/fmu/in/vehicle_visual_odometry`** (standard, consumed by EKF2 with `EKF2_EV_CTRL=11`).
   Do it with a sim-side launch that runs `to_fcu_vehicle_visual_odometry_node` with an
   overridden input topic param — do NOT edit `$DPV`'s `bridge_params.yaml`.
2. Vanilla `~/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml` does **NOT**
   export `/fmu/out/vehicle_imu` (verified). Cartographer's IMU feed needs it → patch + rebuild
   (Phase 0). The message `VehicleImu.idl` exists in `$DPV/px4_msgs` ✓.

### The sim contract (from `warehouse_gz_sim_ws/components/*/config.env`)

| Topic | Type | Frame | Producer in gz | Phase |
|---|---|---|---|---|
| `/clock` | rosgraph_msgs/Clock | — | gz bridge | ✅ Isaac ROS2Backend already publishes it |
| `/scan` | sensor_msgs/LaserScan | `laser_link` | gz RPLIDAR_S3 bridge | **1** |
| static TF `base_link_fcu→laser_link` | — | — | gz laser component | **1** (sim-geometry values) |
| static TF `laser_link→fcu_imu_base_link_for_laser` | — | — | gz laser component | **1** (sim-geometry values) |
| `/fmu/out/vehicle_imu` | px4_msgs/VehicleImu | — | PX4 (via agent) | **1** (needs dds_topics patch) |
| `/height/range` | LaserScan | — | gz LIDAR_Lite_3 (downward altimeter) | optional; V1 has no rangefinder — add later only if height problems appear |
| `/sim/gt_odom` | nav_msgs/Odometry | — | gz ground-truth odom | **2** (debug/rviz/QR ground truth) |
| `/battery_remaining_time_s` | std_msgs/Float32, 1 Hz, value 100 | — | `ros2 topic pub` loop | **2** (warehouse_auto_mission consumes it) |
| `/zed/image_raw`, `/zed/depth`, `/zed/points`, `/siyi/image_raw` | Image/PointCloud2 | zed frames | gz cameras bridge | **3** |

Bringup commands the gz flow uses (replicate the same commands/order):
- `MicroXRCEAgent udp4 -p 8888`
- `ros2 launch px4_ros2_bridge bridge.launch.py use_sim_time:=true`
- laser: `ros2 launch warehouse_pose_corrector_laser_based laser_scan_processing.launch.py use_sim_time:=true`
- mission session: `warehouse_auto_mission.launch.py`, `path_planner.launch.py`,
  `trajectory_generator.launch.py`, `perception_fusion.launch.py` (all `use_sim_time:=true`)
- `integrated_nav_correction.launch.py` orchestrates mission→(2s)→planner→(6s)→laser→(8s)→filter with staggers
- NOTE: gz mission launcher also references `warehouse_edt_mapping` and
  `warehouse_obstacle_avoidance` — **NOT present in `$DPV`**. Skip them (do not launch);
  if the mission controller hard-requires their topics, stub or ask the user.
- gz vision pane's `CMD_VISION` is EMPTY by default → they run sim WITHOUT the vision
  corrector. Phase 1–2 need no cameras.

---

## 3. Current Isaac/Pegasus state (already fixed earlier today — do not redo, do not lose)

Uncommitted working-tree changes (commit as first action, they are verified-working):
- `extensions/.../logic/vehicles/vehicle.py` — render callback now registered post-play in
  `sim_start_stop` via `_register_render_callback()` (was: registered in `__init__`,
  invalidated by `world.reset()` → lidar/camera ROS2 writers never created). Per-sensor
  try/except added.
- `extensions/.../logic/vehicles/multirotors/v1.py` — `config=None` default (was
  `config=V1Config()` evaluated at import → duplicate orphan rclpy node corrupting /tf);
  `pub_tf: False` (ground-truth TF disabled).
- `examples/12_px4_v1_vehicle.py` — `config_multirotor.backends[0] = PX4MavlinkBackend(...)`
  (was reassigning the whole list, dropping the ROS2Backend → no sensor topics at all).
- `extensions/.../logic/backends/ros2_backend.py` — comment cleanups.
- `examples/slam_v1/*` — old slam_toolbox pipeline (+ px4_odom_tf_bridge.py). **Superseded
  by this plan** (Cartographer needs no external odom). Leave slam_v1 as-is; do not launch it.

What Isaac publishes today (verified live): `/clock`, `/v1_0/rplidar_c1/laserscan`
(frame `rplidar_c1`, ~10 Hz), `/v1_0/rplidar_c1/pointcloud`, `/v1_0/zed2i/{color/image_raw,color/camera_info,depth}`,
`/v1_0/sensors/{imu,mag}`, `/v1_0/state/*`, rotor refs. Lidar profile is `Example_Rotary_2D`
(do NOT switch to `RPLIDAR_S2E` — produces zero output in Isaac 6.0).

EKF2 GPS-denied params in SITL eeprom at recon time were the REAL-DRONE ZED-VIO
config: `EKF2_GPS_CTRL=0, EKF2_EV_CTRL=11, EKF2_HGT_REF=3, EKF2_EV_DELAY=50`.
Those values are only valid when the EV source has a sane Z (real ZED VIO). With
cartographer 2D laser odom as EV they diverge the estimator (fusing EV vertical
injected ~6.8 km height -> roll flip + accel-bias runaway), so the sim now runs
`EKF2_EV_CTRL=1, EKF2_HGT_REF=0` (see start.sh / set_px4_gps_denied_params_onboard.py).
Phase 4 (real ZED VIO) stages back toward 11/3 — see set_px4 `--profile zed`.

---

## 4. Phase 0 — prerequisites (blockers found during recon)

1. **Commit the working tree** (Pegasus fixes above) so later experiments can be diffed/reverted.
2. **Install cartographer_ros** — the SLAM engine is NOT on this machine (verified:
   `ros2 pkg prefix cartographer_ros` → not found; wrapper only ships
   `px4_imu_converter` + `cartographer_pose_transformer`):
   ```bash
   sudo apt install ros-humble-cartographer-ros
   ```
   Verify: `ros2 pkg prefix cartographer_ros` and `ros2 pkg executables cartographer_ros`
   (needs `cartographer_node`, `cartographer_occupancy_grid_node`).
3. **Export `/fmu/out/vehicle_imu` from PX4** — add to
   `~/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml` under `publications:`
   (copy the style of the `/fmu/out/vehicle_odometry` entry):
   ```yaml
     - topic: /fmu/out/vehicle_imu
       type: px4_msgs::msg::VehicleImu
   ```
   Then rebuild SITL: `cd ~/PX4-Autopilot && make px4_sitl_default` (dds_topics.yaml is a
   codegen input; incremental rebuild is enough). Verify after next sim boot:
   `ros2 topic list | grep vehicle_imu` then `ros2 topic hz /fmu/out/vehicle_imu`
   (FROM AN INTERACTIVE TERMINAL — see gotchas).
   *Fallback if rebuild is unacceptable:* write a small sim node converting Isaac's
   `/v1_0/sensors/imu` (sensor_msgs/Imu, FRD) to `/imu` with
   `frame_id=fcu_imu_base_link_for_laser`, bypassing `px4_imu_converter` — but prefer the
   real path (their converter handles PX4 FRD conventions; don't re-derive them).
4. **px4_msgs ABI**: the stack must run against `$DPV`'s own `px4_msgs`. Sanity-check at
   runtime: `ros2 topic echo /fmu/out/vehicle_odometry --once` from a `$DPV`-sourced
   INTERACTIVE terminal. If message CDR mismatch errors appear, the fork's px4_msgs differs
   from v1.16 — stop and report.
5. **Teardown before every run**:
   ```bash
   pgrep -af "bin/px4|kit/python|MicroXRCEAgent|slam_toolbox|cartographer|px4_ros2_bridge|vision_bridge"
   # kill everything found (kill -9 if needed); then verify:
   ss -tlnp | grep 4560   # must be empty ("4560 FREE")
   ```

---

## 5. Phase 1 — localization slice (Isaac lidar+IMU → Cartographer → PX4 EV → Position mode)

### 5.1 Make Isaac publish the lidar contract (`/scan`, frame `laser_link`)

Modify Pegasus (sim side — allowed):
- `extensions/.../logic/backends/ros2_backend.py` `add_lidar_writter` (~line 507): topic and
  frame are currently hardcoded (`topicName=data["lidar_name"]+"/laserscan"`,
  `frameId=data["lidar_name"]`, `nodeNamespace=self._namespace+str(self._id)`). Extend the
  `data` dict plumbing so the Lidar sensor config can override them: in
  `graphical_sensors/lidar.py`, pass optional `config` keys (e.g. `"scan_topic"`,
  `"scan_frame_id"`) through `self._state`; in `add_lidar_writter`, use them when present.
  For an absolute topic (`/scan`) set `nodeNamespace=""` and `topicName="scan"`.
  Apply the same override to the PointCloud writer for consistency (optional).
- `extensions/.../logic/vehicles/multirotors/v1.py` Lidar config: add
  `"scan_topic": "/scan", "scan_frame_id": "laser_link"`. Keep profile `Example_Rotary_2D`,
  frequency 10.0. (Real S3 runs 20 Hz; 10 Hz works for cartographer — raising it is a tuning
  knob, not a blocker. The prim attrs drive LaserScan writer metadata automatically.)

### 5.2 Sim bringup launch (new file, `examples/dpv_sim/isaac_nav_bringup.launch.py`)

A single launch (run from a `$DPV`-sourced terminal) that starts, all with
`use_sim_time:=true`:
1. Static TF `base_link_fcu → laser_link`: **sim geometry**, i.e. `0 0 0.135 0 0 0`
   (V1 lidar mount = [0,0,0.135], no rotation — do NOT copy the real drone's
   `0.21 0 0.13 -3.1415926 0 0`; positional static_transform_publisher args are
   `x y z YAW PITCH ROLL`).
2. Static TF `laser_link → fcu_imu_base_link_for_laser`: sim geometry `0 0 -0.135 0 0 0`
   (IMU at body origin).
   *Note:* the real launch's TF pair encodes the real mounting (incl. 180° yaw). We keep the
   frame NAMES identical and adjust numbers to V1's actual USD geometry. If scans appear
   mirrored/rotated vs. motion in rviz, revisit these two TFs first.
3. `IncludeLaunchDescription` of `$DPV` `cartographer_slam_wrapper/launch/cartographer_slam.launch.py`
   with `use_sim_time:=true` (defaults are right: px4_imu_topic=/fmu/out/vehicle_imu,
   scan remapped to `/scan`).
4. The needed `px4_ros2_bridge` nodes (mirror `bridge.launch.py`, but as individual `Node`
   actions so the EV input can be overridden without touching `$DPV`):
   - static TF `map → odom_fcu` (identity) — as in their launch.
   - `from_fcu_vehicle_local_position_node` (params: `$DPV/px4_ros2_bridge/share/px4_ros2_bridge/config/bridge_params.yaml` + `use_sim_time`).
   - `to_fcu_vehicle_visual_odometry_node` with params `[bridge_params.yaml, {use_sim_time: true},
     {'to_fcu.input_topics.odometry': '/cartographer/laser_odom_at_fcu'}]`.
     ⚠ The exact param name must be confirmed at runtime (`ros2 param list /to_fcu_vehicle_visual_odometry`);
     nested YAML keys usually flatten to `to_fcu.input_topics.odometry`. If param override
     doesn't take, copy bridge_params.yaml into examples/dpv_sim/, edit the copy, pass it instead.
   - (skip laser_odometry/correction/payload/trajectory relay nodes in Phase 1 — they publish
     to fork-only or mission topics.)
5. TF connectivity note (observed quirk, replicate as-is): Cartographer with
   `provide_odom_frame=true` publishes `map→odom_laser→laser_link` while the static TF gives
   `laser_link` the parent `base_link_fcu`. This dual-parent arrangement exists on the real
   drone and works for them — do not "fix" it; just replicate. If TF errors flood, report
   rather than redesign.

### 5.3 Run order (3 terminals + checks)

```
T1: cd ~/PegasusSimulator && scripts/isaac_run.sh examples/12_px4_v1_vehicle.py
    # wait until PX4 preflight messages appear in output
T2: MicroXRCEAgent udp4 -p 8888
T3: (source $DPV) ros2 launch <repo>/examples/dpv_sim/isaac_nav_bringup.launch.py
```
EKF2 params: already in eeprom on this machine. To verify/re-set: QGC latches the GCS link
(udp 18570) — use PX4's ONBOARD link instead: pymavlink `udpin:127.0.0.1:14540`
(PX4 sends to 14540 from 14580). A working script existed at the previous session's
scratchpad (`ekf2_params_onboard.py`); recreate it in `examples/dpv_sim/` (params:
`EKF2_GPS_CTRL=0, EKF2_EV_CTRL=11, EKF2_HGT_REF=3, EKF2_EV_DELAY=50`; param changes need a
PX4 reboot to take effect).

### 5.4 Phase 1 acceptance (all from an INTERACTIVE terminal, `$DPV` sourced)

1. `ros2 topic hz /scan` ≈ 10 Hz, `ros2 topic echo /scan --once | grep frame_id` → `laser_link`.
2. `ros2 topic hz /fmu/out/vehicle_imu` > 0 and `/imu` > 0 (px4_imu_converter output).
3. Cartographer alive: `/cartographer/odom` and `/cartographer/laser_odom_at_fcu` publishing;
   occupancy grid `/map` grows in rviz2; no "queue is full" spam in cartographer output.
4. `ros2 topic hz /fmu/in/vehicle_visual_odometry` > 0 (to_fcu node forwarding).
5. PX4 (T1 output): NO repeating `[timesync] time jump detected`; eventually
   `EKF2 commencing external vision fusion` (or equivalent); preflight
   "height estimate not stable" clears.
6. QGC: valid local position; **flight test**: Stabilized → Altitude → Position mode hold,
   then a small stick translation — cartographer map stays consistent.

---

## 6. Phase 2 — correction + navigation stack

Prereqs: Phase 1 green. Add to the bringup (staggered like `integrated_nav_correction.launch.py`):
1. `laser_scan_processing.launch.py` (laser corrector; input default `/scan` ✓).
2. `perception_fusion.launch.py` (warehouse_pose_correction_filter).
3. `path_planner.launch.py`, `trajectory_generator.launch.py`.
4. `warehouse_auto_mission.launch.py` + battery stub:
   `ros2 topic pub -r 1 /battery_remaining_time_s std_msgs/msg/Float32 '{data: 100.0}'`.
5. Ground-truth odom for rviz/debug: small sim node `examples/dpv_sim/gt_odom_pub.py`
   subscribing `/v1_0/state/pose` + `/v1_0/state/twist_inertial` → nav_msgs/Odometry on `/sim/gt_odom`.
6. Mission needs a mission file: gz used `$SIM_WS/mission_files/0001_0001_0001.json`
   (`~/ros2_ws/src/warehouse_gz_sim_ws/mission_files/`). integrated_nav's `load_mission_cmd`
   shows how it is loaded (TimerAction @2 s) — replicate.
7. Watch for topics from the MISSING `warehouse_edt_mapping`/`warehouse_obstacle_avoidance`
   packages; if the mission state machine blocks on them, report to user (decision needed:
   obtain packages or disable those features in mission params —
   `$DPV/warehouse_auto_mission/share/warehouse_auto_mission/config/warehouse_auto_mission_params.yaml`).

Acceptance: corrected pose stream stays sane while flying Position-mode moves; planner
produces a path for a mission waypoint; (if wired) trajectory setpoints reach PX4 via the
bridge's trajectory relay (add `to_fcu_trajectory_relay_node`/`to_fcu_command_relay_node`
to the bringup for this — vanilla PX4 consumes `/fmu/in/trajectory_setpoint` etc. — verify
which `/fmu/in/*` topics the relays use and that they exist in vanilla dds_topics.yaml
before expecting offboard motion).

## 7. Phase 3 — cameras / vision corrector / QR

### 7.1 Ground-truth (verified on this machine)

| Thing | Status |
|---|---|
| ZED SDK | **Installed** at `/usr/local/zed/` (libsl_zed.so, ZED_Explorer, ZED_Diagnostic) |
| zed-msgs (Python) | **Installed** v5.2.1 |
| `ros-humble-zed-msgs` | **Installed** |
| Stereolabs Isaac Sim extension | **AVAILABLE since 2026-06** — zed-isaac-sim v5.1.0 supports Isaac Sim 6.0; pinned clone at `extensions/zed-isaac-sim/` (Phase 4). Virtual cameras are the ZED X family (12 cm baseline like the ZED 2i); zed_wrapper sim_mode must use `camera_model:=zedx` |
| Isaac ZED camera output | Publishes `/v1_0/zed2i/color/image_raw`, `/v1_0/zed2i/depth`, `/v1_0/zed2i/color/camera_info` |
| SVI (SIYI camera) | Not on V1 vehicle; Isaac has no SIYI sensor |

**Critical consequence (RESOLVED by Phase 4):** at recon time no Stereolabs Isaac
extension existed for this Isaac version, so `zed_wrapper` could not run in `sim_mode`
and the ZED VIO odometry (`/zed/zed_node/odom_zed_to_fcu`) had to be faked
(`zed_vio_stub.py`, Option B below). zed-isaac-sim **v5.1.0** (June 2026) added Isaac
Sim 6.0 support: the virtual ZED streams to ZED SDK 5.0.7 (IPC, port 30000) and the
real `zed_wrapper` runs `sim_mode:=true` — see Phase 4 (`start.sh` default) and
`isaac_nav_bringup_zed.launch.py`.

### 7.2 Camera topic remapping (3.1)

Same pattern as the lidar override in Phase 1. Extend `MonocularCamera` + `add_monocular_camera_writter`
to accept optional topic/frame overrides from camera config.

**7.2.1 `monocular_camera.py`** — pass through `color_topic`, `depth_topic`, `camera_info_topic`, `camera_frame_id`:

```python
# In __init__, add:
self._color_topic = config.get("color_topic", None)
self._depth_topic = config.get("depth_topic", None)
self._camera_info_topic = config.get("camera_info_topic", None)
self._camera_frame_id = config.get("camera_frame_id", None)

# In update(), add to self._state:
self._state["color_topic"] = self._color_topic
self._state["depth_topic"] = self._depth_topic
self._state["camera_info_topic"] = self._camera_info_topic
self._state["camera_frame_id"] = self._camera_frame_id
```

**7.2.2 `ros2_backend.py`** — in `add_monocular_camera_writter`, honor overrides (same namespace logic as lidar: absolute topic → empty namespace):

```python
# Resolve topic overrides
color_override = data.get("color_topic")
depth_override = data.get("depth_topic")
info_override = data.get("camera_info_topic")
frame_id = data.get("camera_frame_id", data["camera_name"])

if color_override and color_override.startswith("/"):
    ns, color_topic = "", color_override[1:]
elif color_override:
    ns, color_topic = self._namespace + str(self._id), color_override
else:
    ns, color_topic = self._namespace + str(self._id), data["camera_name"] + "/color/image_raw"

# Same pattern for depth_topic, camera_info_topic...
# Use `ns` and resolved topics in writer.initialize() calls.
# Set frameId=frame_id in all three writers.
```

**7.2.3 `v1.py`** — ZED camera config (no SIYI on V1):

```python
MonocularCamera("zed2i", config={
    "position": [0.355, 0.0, 0.0],
    "resolution": (1920, 1200),
    "frequency": 30,
    "intrinsics": [[777.0, 0.0, 960.0], [0.0, 777.0, 600.0], [0.0, 0.0, 1.0]],
    "depth": True,
    "color_topic": "/zed/image_raw",
    "depth_topic": "/zed/depth",
    "camera_info_topic": "/zed/color/camera_info",
    "camera_frame_id": "zed2i_camera_link",
}),
```

Note: Isaac publishes `/v1_0/zed2i/depth` as a depth image but can produce point clouds
via the `DistanceToImagePlane` writer (`depth=True` in camera config already enables this).
The `/zed/points` topic from the Gazebo contract is a PointCloud2 — the existing
`add_monocular_camera_writter` already has a depth writer; we may need to add a
PointCloud writer as well.

### 7.3 Vision corrector (3.2)

The real vision corrector consumes:
- `/zed/zed_node/odom_zed_to_fcu` (ZED VIO odometry — nav_msgs/Odometry, produced by zed_wrapper)
- Camera images for beam detection

**Problem:** zed_wrapper's `sim_mode` needs the Stereolabs Isaac extension to stream
simulated stereo images from Isaac → ZED SDK → zed_wrapper → VIO. Without the extension:

**Option A (matching gz sim behavior):** Skip the vision corrector entirely — gz sim
runs with `CMD_VISION` empty and the corrector stays off. Cartographer's laser-based
localization handles odometry; the vision pipeline is only for QR/beam pose refinement.

**Option B (fake ZED VIO):** Create `zed_vio_stub.py` that converts
`/cartographer/laser_odom_at_fcu` → `/zed/zed_node/odom_zed_to_fcu`, mirroring the
frame conventions (FLU → NED etc.). This lets the vision corrector "see" odometry,
but the beam inference still needs real camera images for pillar detection.

**Recommendation:** Implement Option A first (skip corrector, just remap camera topics
for visualization/rviz). Option B is a separate investigation if vision-behavior is
specifically needed.

**Phase 4 supersedes both options:** the real `zed_wrapper` VIO now runs against the
Isaac stream (zed-isaac-sim v5.1.0 + ZED SDK sim_mode), publishing the true
`/zed/zed_node/*` topics — including `depth/depth_registered` + `depth/camera_info`
that the vision corrector expects, with no remapping. `zed_vio_stub.py` remains only
for phase-3 back-compat; phase 4 uses `zed_odom_to_fcu.py` (re-stamp glue on the real
wrapper odom).

### 7.3a Phase 4 BLOCKED — ZED SDK version gap (measured 2026-07-16)

Phase 4 is implemented (7.3b) but **cannot run on this machine's SDK**. Measured by
calling `sl::Camera::getSDKVersion()` directly on each library:

| Component | Bundled/linked ZED SDK | Kit target |
|---|---|---|
| zed-isaac-sim **v5.1.0** streamer (sender) — the ONLY release supporting Isaac Sim 6.0 | **5.2.0** | 110.0 (Isaac 6.0) |
| zed-isaac-sim **v4.2.1** (tested as a downgrade) | **5.2.0** — identical | 107.3 (Isaac 5.0) |
| `/usr/local/zed` + `zed_wrapper` 5.0.0 in dpv-install (receiver) | **5.0.7** | — |

**The extension version is not the problem — the system SDK is too old.** Stereolabs
ships the same SDK 5.2.0 runtime in both the 4.x and 5.x extension lines, so
downgrading the extension does not close the gap (and 4.x additionally targets kit
107.3, so it will not even load in Isaac Sim 6.0). v5.1.0 is the correct choice here.

Symptom chain: `Backward compatibility required.` -> `Metadata timeout. the size is
equal to 196 instead of 21960.` -> `Invalid calibration file` -> `zed_node` segfault
(exit -11). The calibration metadata struct changed between 5.0.x and 5.2.x.

Both workarounds are closed, by measurement not guesswork:
- **Sender -> 5.0.7** (symlink the system lib into the extension's `bin/`): the plugin
  has an `isZEDSDKCompatible` gate and refuses it — `[ZED] Error while loading ZED SDK`,
  so nothing streams at all (metadata size `-1`).
- **Receiver -> 5.2.0** (`LD_LIBRARY_PATH` the wrapper at the bundled lib): ABI break.
  Of the 173 `sl::` symbols `libzed_camera_component.so` needs, **4 are absent** from
  5.2.0 (`InputType::setFromSerialNumber`, `InputType::setFromCameraID`,
  `InputType` copy ctor, `PositionalTrackingParameters` ctor) — undefined-symbol crash.
- **Older extension**: no v5.0.x release/tag exists (the CHANGELOG's 5.0.0/5.0.1 were
  never published); 4.X.X targets Isaac Sim 5.0 and predates 6.0 support.

**Unblocking requires ZED SDK 5.2.0 + a `zed_wrapper` rebuilt against it** — i.e.
upgrading dpv-install, which the §1 constraint forbids and which would diverge the sim
from the drone's actual flight software (SDK 5.0.x / wrapper 5.0.0). That is a product
decision: it only makes sense **once the drone itself moves to SDK 5.2**. Until then
`./start.sh` defaults to phase 2 (laser EV + native Isaac ZED 2i RGB-D), which is
unaffected — phase 4 only changes where PX4's EV comes from.

### 7.3b Phase 4 — real ZED VIO (implemented 2026-07)

The full real EV chain now runs; see `examples/dpv_sim/README.md` "Phase 4" for
the run procedure. Pieces: `extensions/zed-isaac-sim/` (pinned v5.1.0, needs
one-time `./build.sh`), `zed_sim_camera.py` (virtual ZED X on the V1 nose,
SVGA@30 IPC stream), the real `zed_wrapper` in sim_mode (start.sh window "zed",
`zed_sim_overrides.yaml`), `zed_odom_to_fcu.py` (glue), and
`isaac_nav_bringup_zed.launch.py` (to_fcu on its real default EV input,
cartographer in shadow mode). EKF2 staging: D-1 `EV_CTRL=1/HGT_REF=0` (boot
default) → D-2 `EV_CTRL=3` → D-3 `EV_CTRL=11/HGT_REF=3` (drone eeprom config)
via `set_px4_gps_denied_params_onboard.py --profile zed`.

### 7.4 Isaac ZED warmup

Camera topics (`/v1_0/zed2i/*`) appear ~100 rendered frames after sim play starts
(§8.7). Any bringup launch that includes vision nodes must account for this delay —
use TimerActions with sufficient stagger.

### 7.5 QR pipeline (3.3) — parked

Needs Sony camera (SIYI) — not on V1, no Isaac SIYI sensor — plus QR texture assets
on cargo in the world model. This is a separate Isaac content-authoring task. Document
in PLAN but do not implement in this phase.

### 7.6 Phase 3 deliverables inventory

| File | Action |
|---|---|
| `extensions/.../monocular_camera.py` | add color_topic/depth_topic/camera_info_topic/camera_frame_id config passthrough |
| `extensions/.../ros2_backend.py` | honor camera topic/frame overrides in `add_monocular_camera_writter` |
| `extensions/.../v1.py` | ZED camera config → `/zed/image_raw`, `/zed/depth`, etc. |
| `examples/dpv_sim/zed_vio_stub.py` | NEW (optional) — cartographer odom → ZED VIO relay for vision corrector |
| `examples/dpv_sim/isaac_nav_bringup_phase3.launch.py` | NEW — Phase 2 + camera bringup + vision corrector (optional) |

---

## 8. Environment gotchas (hard-won; violating these wastes hours)

1. **Isaac must be launched via `scripts/isaac_run.sh`** — it strips `/opt/ros` + `~/ros2_ws`
   from PYTHONPATH/LD_LIBRARY_PATH and points the ROS2 bridge at Isaac's internal humble libs.
   A bare `python.sh` run crashes rclpy (py3.10 vs 3.12) → NO topics at all.
2. **Stale processes are the #1 source of phantom bugs.** Old `bin/px4` holding state, or a
   leaked 4560 listener → infinite `[Errno 98] Address already in use` storm from
   PX4MavlinkBackend. Always do the §4.5 teardown; verify `4560 FREE`.
3. **This machine's non-interactive shells (agent Bash) have a broken ROS2 data plane**:
   `ros2 topic list/info` (discovery) work; `ros2 topic echo/hz` receive NOTHING even from
   healthy publishers. Never conclude "no data" from an agent shell — verify rates in the
   user's interactive terminal, or read node log files.
4. **`use_sim_time:=true` on EVERY stack node.** Isaac publishes `/clock` (sim time).
   A single wall-clock-stamped publisher into PX4 (`vehicle_visual_odometry.timestamp`)
   causes PX4 `[timesync] time jump detected` flapping and EKF2 rejects everything.
   The px4_ros2_bridge nodes take `use_sim_time` from the launch — pass it everywhere.
5. **QGC latches PX4's GCS mavlink (18570/14550).** Param scripts must use the onboard link
   `udpin:127.0.0.1:14540`. EKF2 params persist in eeprom; they apply on NEXT PX4 boot.
6. **Isaac RTX lidar**: profile `RPLIDAR_S2E` yields zero output in Isaac 6.0 — keep
   `Example_Rotary_2D`. The LaserScan writer needs the scan-geometry kwargs already wired in
   `add_lidar_writter` (don't remove them).
7. **Camera warmup**: `/v1_0/zed2i/*` appear ~100 rendered frames after play.
8. **PX4 preflight fails until vision fuses** ("height estimate not stable",
   "Attitude failure", "High Accelerometer Bias") — EXPECTED with no GPS; not a bug. It
   clears when EV data flows + EKF2 converges (~30 s). Don't chase it before step 5.4.5.
9. First `Preflight Fail: Attitude failure (roll)` lines can also appear pre-play/early —
   only judge after the sim has been playing for ≥30 s with EV flowing.

---

## 9. Risks / open questions (report, don't silently work around)

- **Param override name** for `to_fcu_vehicle_visual_odometry` input (§5.2.4) — confirm at runtime.
- **`laser_odom_at_fcu` message type** assumed `nav_msgs/Odometry` (matches to_fcu's ZED input);
  confirm with `ros2 topic info`.
- **px4_msgs fork drift** (§4.4).
- **Dual-parent TF on `laser_link`** (cartographer vs static TF) — replicate; report if it breaks.
- **EV frame semantics**: `laser_odom_at_fcu` was built to be consumed at the FCU/IMU position —
  designed for their fork's laser-odometry input. Feeding it to `vehicle_visual_odometry`
  (Phase 1 shortcut) should be equivalent (same NED/FRD conventions handled by to_fcu node),
  but if EKF2 innovations look huge, compare against `/px4_ros2_bridge/odometry/fcu_odom_flu`.
- **Missing packages**: `warehouse_edt_mapping`, `warehouse_obstacle_avoidance` (§6.7).
- **Downward rangefinder** (`/height/range`): real drone has LIDAR-Lite → PX4 distance_sensor.
  V1 sim relies on vision height (`EKF2_HGT_REF=3`). If mission behavior needs AGL height,
  add a rangefinder to V1 later (Pegasus has no stock one — would need an RTX lidar in
  single-beam config or a fake publisher from ground-truth z).

## 10. Deliverables inventory

| File | Action |
|---|---|
| `extensions/.../graphical_sensors/lidar.py` | add scan_topic/scan_frame_id config passthrough |
| `extensions/.../backends/ros2_backend.py` | honor topic/frame overrides in `add_lidar_writter` |
| `extensions/.../vehicles/multirotors/v1.py` | lidar config → `/scan`, `laser_link` |
| `examples/dpv_sim/isaac_nav_bringup.launch.py` | NEW — Phase 1 bringup (TFs + cartographer + bridge nodes) |
| `examples/dpv_sim/set_px4_gps_denied_params_onboard.py` | NEW — onboard-link param setter |
| `examples/dpv_sim/gt_odom_pub.py` | NEW (Phase 2) — `/sim/gt_odom` |
| `examples/dpv_sim/README.md` | NEW — run procedure (condense §5.3) |
| `~/PX4-Autopilot/.../dds_topics.yaml` | add `/fmu/out/vehicle_imu` + rebuild (outside repo — note in README) |
| git | commit existing working-tree fixes FIRST, then per-phase commits |
