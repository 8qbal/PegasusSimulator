# V1 GPS-Denied SLAM Pipeline (Isaac Sim + PX4 + slam_toolbox)

Localizes the V1 drone in the warehouse **without GPS**, using only its onboard
sensors: the RPLIDAR 2D scans feed `slam_toolbox`, and the SLAM pose estimate is
fed back to PX4's EKF2 as external vision odometry over uXRCE-DDS.

```
Isaac Sim (V1 + Full Warehouse)                       system ROS 2 Humble
┌───────────────────────────────┐                    ┌──────────────────────┐
│ RPLIDAR ──► /v1_0/rplidar_c1/laserscan ───────────►│ slam_toolbox         │
│ ZED 2i  ──► /v1_0/zed2i/*  (RGB-D, for later)      │   └─► /pose          │
│ ground truth ─► TF map -> v1__base_link (odom prior)│        │             │
│                                                    │        ▼             │
│ PX4 SITL ◄── /fmu/in/vehicle_visual_odometry ◄─────│ px4_vision_bridge    │
│  (EKF2 fuses vision instead of GPS)                └──────────────────────┘
└───────────────────────────────┘   (uXRCE-DDS client 8888 ◄─► MicroXRCEAgent)
```

## Run it (3 terminals)

**Terminal 1 — the simulator** (GPS already removed from this example):
```bash
cd ~/PegasusSimulator
isaac_run examples/12_px4_v1_vehicle.py
```
> `isaac_run` must be the env-scrubbing launcher (`scripts/isaac_run.sh`) — it strips
> system ROS 2 from the environment so Isaac's bundled rclpy is used. Otherwise this
> terminal crashes with `No module named 'rclpy._rclpy_pybind11'` (system py3.10 rclpy
> vs Isaac py3.12). Terminals 2–3 below keep system ROS 2 and are unaffected.

**Terminal 2 — the SLAM pipeline** (agent + static TF + slam_toolbox + bridge):
```bash
~/PegasusSimulator/examples/slam_v1/run_slam.sh
```

**Terminal 3 — one-time PX4 EKF2 configuration** (after PX4 has booted):
```bash
~/PegasusSimulator/.venv/bin/python ~/PegasusSimulator/examples/slam_v1/set_px4_gps_denied_params.py
```
This sets `EKF2_GPS_CTRL=0`, `EKF2_EV_CTRL=11` (hpos+vpos+yaw), `EKF2_HGT_REF=3`
(vision height), `EKF2_EV_DELAY=50ms`. Parameters persist in the SITL eeprom, so
after the first run you only need it again if you reset PX4's parameters.
EKF2 picks up vision fusion cleanly on the next PX4 restart — restart the sim
after the first configuration.

## Verify

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 topic hz  /v1_0/rplidar_c1/laserscan       # ~10 Hz scans from Isaac
ros2 topic echo /pose --once                     # slam_toolbox pose (slam_map frame)
ros2 topic hz  /fmu/in/vehicle_visual_odometry   # vision odom flowing to PX4
```
In QGC the drone should report a valid local position with GPS disabled;
Position mode and takeoff become available once EKF2 converges on vision.

## Frames

| Frame | Published by | Meaning |
|---|---|---|
| `map` | Pegasus ROS2 backend (ground truth) | odometry prior for SLAM (`odom_frame`) |
| `v1__base_link` | Pegasus ROS2 backend | drone body (FLU) |
| `rplidar_c1` | static TF (run_slam.sh) | lidar mount, 0.135 m above body origin |
| `slam_map` | slam_toolbox | SLAM's world frame; `/pose` lives here and is what PX4 receives |

## Notes & known limitations

- The lidar uses the generic `Example_Rotary_2D` RTX profile: the bundled
  `RPLIDAR_S2E` profile produces **zero output** in this Isaac Sim build
  (verified with an isolated repro). Range is clamped to 12 m in
  `slam_toolbox_params.yaml` to match the real RPLIDAR C1.
- The ground-truth TF (`map -> v1__base_link`) currently serves as a *perfect*
  odometry prior. For a fully realistic setup, replace it with PX4's own EKF2
  odometry (`/fmu/out/vehicle_odometry` via the bridge, inverted back to TF).
- The ZED 2i RGB-D topics (`/v1_0/zed2i/*`) are published but unused here —
  they are ready for RTAB-Map or a visual-odometry frontend as a next step.
- 2D SLAM assumes roughly constant altitude; large altitude changes will smear
  the map since the lidar plane sweeps different warehouse cross-sections.
