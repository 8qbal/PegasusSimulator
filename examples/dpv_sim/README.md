# DPV Autonomy Stack on Isaac Sim

Run the real DPV drone software (Cartographer SLAM, warehouse correctors, path planner,
px4_ros2_bridge, auto-mission) against Isaac Sim instead of Gazebo.

## Prerequisites

1. **PX4 dds_topics.yaml** must export `/fmu/out/vehicle_imu` (patched; rebuild required).
2. **`ros-humble-cartographer-ros`** installed: `sudo apt install ros-humble-cartographer-ros`.
3. **DPV install** at `extensions/dpv-install/` — source its `setup.bash`:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/PegasusSimulator/extensions/dpv-install/setup.bash
   ```
4. **EKF2 params** set via onboard link (see below).
5. **Phase 4 only:** the Stereolabs streamer plugin, built once:
   `cd ~/PegasusSimulator/extensions/zed-isaac-sim && ./build.sh`
   (plus ZED SDK 5.x at `/usr/local/zed`, already installed).

## Quick start

Everything below can be driven by tmux instead of manual terminals:

```bash
./start.sh          # phase 4 (default): REAL ZED VIO as the EV source
./start.sh 1|2|3    # laser-EV phases (localization / +mission / +vision stub)
./start.sh g        # guidance mode (GPS-fused, no SLAM/EV)
./stop.sh           # kill everything
```

## Phase 1 — Localization (Lidar + Cartographer → PX4 EV)

### 1. Set EKF2 GPS-denied params (once, they persist in EEPROM)

```bash
python3 ~/PegasusSimulator/examples/dpv_sim/set_px4_gps_denied_params_onboard.py
```
(PX4 must be running; script connects to onboard link `udpin:127.0.0.1:14540`.)

### 2. Teardown before every run

```bash
pkill -f "bin/px4|kit/python|MicroXRCEAgent|slam_toolbox|cartographer|px4_ros2_bridge"
ss -tlnp | grep 4560   # must be empty
```

### 3. Launch Isaac Sim

```bash
# Terminal 1
cd ~/PegasusSimulator
scripts/isaac_run.sh examples/12_px4_v1_vehicle.py
# Wait until PX4 preflight messages appear
```

### 4. Launch MicroXRCEAgent

```bash
# Terminal 2
MicroXRCEAgent udp4 -p 8888
```

### 5. Launch the DPV bringup

```bash
# Terminal 3 (source DPV + /opt/ros/humble first)
source /opt/ros/humble/setup.bash
source ~/PegasusSimulator/extensions/dpv-install/setup.bash
ros2 launch ~/PegasusSimulator/examples/dpv_sim/isaac_nav_bringup.launch.py
```

### 6. Verify (from interactive terminal, DPV sourced)

```bash
ros2 topic hz /scan                           # ≈ 10 Hz, frame laser_link
ros2 topic hz /fmu/out/vehicle_imu            # > 0
ros2 topic hz /imu                            # px4_imu_converter output
ros2 topic hz /cartographer/odom              # Cartographer running
ros2 topic hz /cartographer/laser_odom_at_fcu # Cartographer odom
ros2 topic hz /fmu/in/vehicle_visual_odometry # EV flowing to PX4
```

PX4 should eventually show "EKF2 commencing external vision fusion" and clear
preflight failures (~30 s after EV data flows).

## Environment Gotchas

- **Isaac must launch via `scripts/isaac_run.sh`** — it strips system ROS from the env.
- **Stale processes** cause phantom bugs — always do the teardown step.
- **Non-interactive shells** (CI/scripts) cannot `ros2 topic echo/hz` — verify in a user terminal.
- **`use_sim_time:=true`** on every stack node — Isaac publishes `/clock`.
- **QGC latches PX4 GCS link** — use onboard link `udpin:127.0.0.1:14540` for param scripts.
- **PX4 preflight fails** until EV fuses — expected with no GPS, clears when EV data flows.
- **RTX lidar profile** `RPLIDAR_S2E` produces zero output in Isaac 6.0 — we use `Example_Rotary_2D`.

## Phase 2 — Correction + Navigation

After Phase 1 is green:

```bash
# Terminal 3 (source DPV + /opt/ros/humble first)
source /opt/ros/humble/setup.bash
source ~/PegasusSimulator/extensions/dpv-install/setup.bash
ros2 launch ~/PegasusSimulator/examples/dpv_sim/isaac_nav_bringup_phase2.launch.py \
  mission_file:=~/ros2_ws/src/warehouse_gz_sim_ws/mission_files/0001_0001_0001.json
```

This adds (staggered):
- `warehouse_auto_mission` (0 s) + battery stub
- Mission file load (2 s)
- `warehouse_path_planner` (4 s)
- `trajectory_generator` (6 s)
- `laser_scan_processing` / corrector (8 s)
- `perception_fusion` (10 s)

Plus relay nodes: `from_fcu_status_relay`, `to_fcu_command_relay`, `to_fcu_trajectory_relay`,
and the `fcu_pose_to_odom_relay` (pose→odometry for path planner).

### Verify Phase 2

```bash
ros2 topic hz /warehouse_path_planner/state
ros2 topic hz /trajectory_generator/state
ros2 topic hz /warehouse_auto_mission/mission_state
```

**Note:** The mission bypasses battery (`bypass_battery_for_testing: true` in
`warehouse_auto_mission_params.yaml`), so `/battery_remaining_time_s` is published
but not strictly required. Missing packages `warehouse_edt_mapping` /
`warehouse_obstacle_avoidance` are skipped — if the mission state machine blocks
on their topics, report the error.

## Phase 3 — Cameras + Vision Corrector

Camera topics are remapped via v1.py config — Isaac now publishes to the Gazebo
contract topics regardless of which launch is used:

| Isaac writer | Topic |
|---|---|
| ZED color | `/zed/image_raw` |
| ZED depth | `/zed/depth` |
| ZED camera info | `/zed/color/camera_info` |
| ZED frame ID | `zed2i_camera_link` |

### Vision corrector (experimental, off by default)

The vision corrector normally needs ZED VIO from zed_wrapper. In phase 3 that is
faked with a stub (Phase 4 below runs the real thing). Two options:

**Option A (default):** Skip it — matches Gazebo sim behavior (`CMD_VISION` empty).

**Option B:** Use the ZED VIO stub that relays cartographer odom into the ZED VIO
topic, then launch the vision corrector:

```bash
ros2 launch ~/PegasusSimulator/examples/dpv_sim/isaac_nav_bringup_phase3.launch.py \
  launch_vision:=true
```

This adds (at +12 s): `zed_vio_stub` (cartographer odom → `/zed/zed_node/odom_zed_to_fcu`)
then `warehouse_pose_corrector_vision_based`.

### Verify Phase 3

```bash
ros2 topic hz /zed/image_raw          # ZED color images
ros2 topic hz /zed/depth              # ZED depth images
ros2 topic hz /zed/color/camera_info  # Camera intrinsics
# If launch_vision:=true:
ros2 topic hz /zed/zed_node/odom_zed_to_fcu  # VIO stub relay
```

## Phase 4 — Real ZED VIO (default `./start.sh`)

The real drone's external-vision chain, end to end, with no stubs:

```
Isaac (virtual ZED X stereo + IMU, zed-isaac-sim streamer, IPC :30000)
  -> ZED SDK 5.x (/usr/local/zed)
  -> real zed_wrapper  camera_model:=zedx camera_name:=zed sim_mode:=true   (tmux window "zed")
  -> /zed/zed_node/odom -> zed_odom_to_fcu.py -> /zed/zed_node/odom_zed_to_fcu
  -> to_fcu_vehicle_visual_odometry (its REAL bridge_params.yaml default input)
  -> /fmu/in/vehicle_visual_odometry -> EKF2
```

Cartographer still runs in **shadow mode** (feeds the laser corrector and gives an
A/B reference in `diagnose_ev_chain.py`) but nothing routes it into PX4 anymore —
same as the drone, where "cartographer is retired from the stack". The full
phase-2 mission chain (auto-mission, path planner, trajectory generator,
correctors) is included. The virtual camera is the ZED X family (the only models
the streamer supports): same 12 cm baseline as the ZED 2i.

### One-time setup

```bash
cd ~/PegasusSimulator/extensions/zed-isaac-sim && ./build.sh
```

Downloads Stereolabs' `libsl_zed` streaming runtime and builds the OGN streamer
plugin into `exts/sl.sensor.camera/bin/`. `start.sh` refuses to start phase 4
until this exists.

### Run

```bash
./start.sh            # phase 4 is the default
# wait for Isaac to boot + the zed window to connect (it sleeps 45 s first;
# if it errored because Isaac wasn't streaming yet: Up+Enter in that window)
./ev_ready.sh         # blocks until EKF2 fuses EV position + attitude level
./load_mission.sh load ../mission/mission_file_sim_warehouse.json
./load_mission.sh start
./load_mission.sh state   # INIT -> ... -> AUTO_MISSION -> MISSION_COMPLETED
```

### Verify Phase 4

```bash
ros2 topic hz /zed/zed_node/rgb/image_rect_color      # wrapper connected to the stream
ros2 topic hz /zed/zed_node/depth/depth_registered    # real depth (steady, no "blinking")
ros2 topic hz /zed/zed_node/odom                      # real VIO
ros2 topic hz /zed/zed_node/odom_zed_to_fcu           # glue output = EV input
python3 diagnose_ev_chain.py                          # chain + cs_ev_pos + level attitude
```

QGC checklist after ~60 s: no "High Accelerometer Bias" / "Preflight Fail:
Attitude failure", Position mode accepted, arming OK.

### EKF2 staging (D-1 → D-3)

`start.sh` boots phase 4 with the conservative laser-era params
(`EKF2_EV_CTRL=1`, `EKF2_HGT_REF=0`). Once `diagnose_ev_chain.py` shows clean EV
innovations from the real VIO, step toward the drone's eeprom config — either by
editing the `PX4_GPS_DENIED_PARAMS` line in `start.sh`, or live:

```bash
python3 set_px4_gps_denied_params_onboard.py --profile zed --ev-ctrl 3            # D-2: +EV height
python3 set_px4_gps_denied_params_onboard.py --profile zed --ev-ctrl 11 --hgt-ref 3  # D-3: drone config
```

(each needs the PX4 reboot the script triggers).

### Phase 4 troubleshooting

- **Wrapper exits with a connection error**: Isaac wasn't streaming yet
  (asset loading takes a while). Re-run with Up+Enter in the `zed` window.
- **`odom_zed_to_fcu` stamps look far from now**: `zed_odom_to_fcu.py` logs the
  `now - msg.stamp` offset every 5 s. If it is large or drifting, relaunch the
  bringup glue with `restamp:=true` (edit `isaac_nav_bringup_zed.launch.py`) and
  re-check `EKF2_EV_DELAY`.
- **RTF too low**: the streamer runs SVGA@30 over IPC and the wrapper depth mode
  is PERFORMANCE (`zed_sim_overrides.yaml`). Only raise
  `DPV_ZED_RES=HD1080|HD1200` / `DPV_ZED_FPS` if the vision corrector needs it.
- **EV innovations yaw-dependent**: the sim sets `to_fcu.lever_arm.x=0.355`
  (camera ahead of the FCU). The drone ships zeros — zero it in
  `isaac_nav_bringup_zed.launch.py` to match and accept the constant offset.
- **Vision corrector (stretch)**: `ros2 launch isaac_nav_bringup_zed.launch.py
  launch_vision:=true` — it consumes the *real* wrapper depth
  (`/zed/zed_node/depth/depth_registered` + `depth/camera_info`), no remaps.

## Guidance mode — GPS localization + mission/planner/trajectory stack

GPS is acceptable for flight testing when only "guidance mode" (arm → offboard →
auto-mission waypoint flight) needs to work, not full GPS-denied localization. This
mode drops Cartographer, the external-vision reroute, and the pose correctors
entirely, and lets PX4's EKF2 fuse its own simulated GPS instead. The real
`px4_ros2_bridge` + `warehouse_auto_mission` + `warehouse_path_planner` +
`trajectory_generator` stack runs unmodified — those packages consume PX4's fused
local position (`/px4_ros2_bridge/odometry/fcu_pose_at_imu` /
`fcu_odom_flu`) and don't care whether it came from GPS or vision.

Run it:

```bash
./start.sh g
```

Differs from phases 1–3:
- No TF publishers for the lidar/Cartographer frame chain, no Cartographer bringup.
- No `to_fcu_vehicle_visual_odometry_node` (EV reroute) — nothing feeds vision/laser
  odometry to PX4.
- No pose-corrector / correction-filter stages, no ZED VIO stub.
- `start.sh g` exports GPS-enabled EKF2 params (`EKF2_GPS_CTRL=7`, `EKF2_EV_CTRL=0`,
  `EKF2_MAG_CHECK=1`) explicitly, overriding any GPS-denied values a prior phase run
  may have persisted to the PX4 SITL eeprom.

### Verify guidance mode

```bash
# QGC: confirm 3D GPS fix and EKF2 preflight checks pass — no "no vision" warning.
ros2 topic echo /fmu/out/vehicle_gps_position --once
ros2 topic echo /fmu/out/estimator_status_flags --once   # cs_gps set, cs_ev_pos unset
ros2 topic hz /px4_ros2_bridge/odometry/fcu_pose_at_imu
ros2 topic hz /px4_ros2_bridge/odometry/fcu_odom_flu
# warehouse_auto_mission logs: MissionCommand received, WaypointList published
# state machine: INIT->IDLE->READY->CHANGE_TO_POSITION_MODE->ARMING->
#   CHANGE_TO_OFFBOARD_MODE->TAKING_OFF->AUTO_MISSION->MISSION_COMPLETED
```
