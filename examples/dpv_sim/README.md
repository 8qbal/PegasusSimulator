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

(To be implemented — see PLAN.md §6.)
