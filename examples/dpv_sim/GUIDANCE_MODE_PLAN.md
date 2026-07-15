# Guidance-mode (GPS-allowed) DPV simulation — implementation plan

## Context

The DPV sim work so far (`PLAN.md`, phases 1–3) assumed GPS-denied flight: Isaac lidar → Cartographer → external-vision reroute into PX4's EKF2, with `EKF2_GPS_CTRL=0` forcing PX4 to ignore its GPS sensor. The mentor has since confirmed GPS is acceptable for flight testing, as long as the real DPV packages are still exercised, and only "guidance mode" (arm → offboard → auto-mission waypoint flight) needs to work — not full GPS-denied localization.

This removes the need for Cartographer, the EV reroute, and the pose correctors entirely. Investigation confirmed this is a clean simplification, not a rework:

- **GPS already reaches PX4.** `PX4MavlinkBackend.send_gps_msgs()` (`extensions/pegasus.simulator/pegasus/simulator/logic/backends/px4_mavlink_backend.py:747`) sends `HIL_GPS` from the vehicle's `GPS()` sensor unconditionally — this is independent of the ROS2Backend's `pub_gps: False` (that flag only gates the ROS topic, not MAVLink). The only thing suppressing GPS use was `EKF2_GPS_CTRL=0`.
- **The guidance/mission packages are position-source-agnostic.** `warehouse_auto_mission`, `warehouse_path_planner`, and `trajectory_generator` all subscribe to `/px4_ros2_bridge/odometry/fcu_pose_at_imu` / `fcu_odom_flu` — PX4's own fused local-position output relayed by `px4_ros2_bridge`. They don't care whether EKF2 derived that position from GPS or external vision. So no changes are needed to the real packages themselves — only to which upstream bringup feeds EKF2, and which EKF2 params are set.
- **Mission waypoints are local-frame**, not georeferenced (`~/ros2_ws/src/warehouse_gz_sim_ws/mission_files/0001_0001_0001.json` waypoints are plain local x/y/z/yaw, e.g. `{"type":20,"x":10.0,"y":0.0,...}`), so switching the position source doesn't require touching the mission file.
- **Risk: stale eeprom.** PX4 SITL persists params to `~/PX4-Autopilot/build/px4_sitl_default/rootfs/eeprom`. A prior phase-1/2/3 run could leave `EKF2_GPS_CTRL=0` etc. baked into eeprom; if guidance mode doesn't explicitly override them, it could silently inherit GPS-denied config. `start.sh` already relies on exporting `PX4_PARAM_*` env vars that Pegasus's px4 launch tool passes through to the rcS boot script (applied via `param set` before EKF2 starts), so the fix is to have guidance mode explicitly export the stock/GPS-enabled values rather than omitting them.

## Plan

### 1. New launch file: `isaac_nav_bringup_guidance.launch.py`

Model on `isaac_nav_bringup_phase2.launch.py`, with these removals:
- Drop the 3 static TF publishers (`base_link_fcu→laser_link`, `laser_link→imu_base`, `map→odom_fcu`) — they exist only for the lidar/Cartographer frame chain, which guidance mode doesn't use.
- Drop `to_fcu_vehicle_visual_odometry_node` (the EV reroute) entirely — no vision/laser odometry is being fed to PX4.
- Drop Cartographer bringup include and the pose-corrector/correction-filter TimerActions (the 8s/10s stages in phase2).
- Keep: `from_fcu_vehicle_local_position_node`, `from_fcu_vehicle_odometry_node`, `from_fcu_status_relay_node`, `to_fcu_command_relay_node`, `to_fcu_trajectory_relay_node` (the bridge nodes the mission stack needs).
- Keep the staggered TimerAction chain: `warehouse_auto_mission` (0s) → mission-file load via `MissionCommand` pub to `/onboard_command` (2s) → `warehouse_path_planner` (4s) → `trajectory_generator` (6s).
- Keep the battery stub (`/battery_remaining_time_s` constant pub) — `warehouse_auto_mission` config has `bypass_battery_for_testing` but the stub is cheap insurance.
- Mission file arg: default to the real path `~/ros2_ws/src/warehouse_gz_sim_ws/mission_files/0001_0001_0001.json` (confirmed to exist, unlike the dead symlinks noted for other tooling elsewhere) — keep it as a launch arg so it's overridable.

### 2. `start.sh`: add a `g` mode

- Extend the phase `case` statement to accept `g` → `BRINGUP=.../isaac_nav_bringup_guidance.launch.py`.
- For window 1 (Isaac Sim), branch the exported `PX4_PARAM_*` block: when `PHASE=g`, export the guidance/GPS-enabled set instead of the GPS-denied set, explicitly (to overwrite anything stale in eeprom from a prior phase run):
  ```
  PX4_PARAM_EKF2_GPS_CTRL=<PX4 default, verify in PX4 firmware source under ~/PX4-Autopilot — typically 1 = 2D fusion>
  PX4_PARAM_EKF2_EV_CTRL=0
  PX4_PARAM_EKF2_HGT_REF=0   (baro height, same as before)
  PX4_PARAM_EKF2_MAG_CHECK=1 (re-enable, since mag environment is nominal here)
  ```
  Verify PX4's actual default `EKF2_GPS_CTRL` value before hardcoding — grep the firmware source rather than assuming.
- Update the usage comment header (currently documents phases 1–3 only).

### 3. No changes needed to:
- `extensions/pegasus.simulator/.../v1.py` — GPS sensor + PX4MavlinkBackend already deliver GPS to PX4.
- The real DPV ROS2 packages (`warehouse_auto_mission`, `warehouse_path_planner`, `trajectory_generator`, `px4_ros2_bridge`) — unmodified, matching the mentor's requirement to keep using the provided packages as-is.

### 4. `README.md`

Add a "Guidance mode" section: what it covers (GPS localization + real mission/planner/trajectory stack, no SLAM/EV/vision), how to run (`./start.sh g`), and how it differs from phases 1–3.

### 5. Verification

1. `./start.sh g`, attach via `tmux attach -t dpv-sim`.
2. In QGC: confirm a 3D GPS fix and that EKF2 preflight checks pass (unlike GPS-denied phases, this should NOT show the "no vision" preflight warning).
3. `ros2 topic echo /fmu/out/vehicle_gps_position --once` and `/fmu/out/estimator_status_flags` — confirm `cs_gps` fusion flag set, `cs_ev_pos` unset.
4. Confirm bridge topics alive: `/px4_ros2_bridge/odometry/fcu_pose_at_imu`, `/px4_ros2_bridge/odometry/fcu_odom_flu`.
5. Confirm mission load: check `warehouse_auto_mission` logs for `MissionCommand` received and `WaypointList` published.
6. Watch the state machine progress: `INIT→IDLE→READY→CHANGE_TO_POSITION_MODE→ARMING→CHANGE_TO_OFFBOARD_MODE→TAKING_OFF→AUTO_MISSION` (via `/warehouse_auto_mission` state topic or logs).
7. Confirm the drone actually flies the waypoints in Isaac Sim (visually) and reaches `MISSION_COMPLETED` or lands per the mission's `type:5` (`prepare_landing`) waypoint.
8. Run `stop.sh` after, confirm clean teardown (same as existing phases).

## Scope decisions (already made, don't re-litigate)

- Package scope: **minimal guidance chain only** — `px4_ros2_bridge` + `warehouse_auto_mission` + `warehouse_path_planner` + `trajectory_generator` + battery stub + mission load. No Cartographer, no EV reroute, no pose correctors, no ZED stub.
- Mode layout: **added as a new mode**, not a replacement — phases 1–3 stay untouched as GPS-denied fallback/reference.
