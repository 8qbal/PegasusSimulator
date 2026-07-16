# Changelog

Notable changes to this fork of Pegasus Simulator, which ports the framework to
Isaac Sim 6.0 and integrates a real drone ("DPV") autonomy stack — Cartographer
SLAM, PX4↔ROS 2 bridge, warehouse mission/path-planning, pose correctors, ZED
VIO — against it in place of Gazebo. No version tags exist yet; entries are
grouped by work session instead of release number. Format loosely follows
[Keep a Changelog](https://keepachangelog.com/).

## 2026-07-16 — Cleanup

### Fixed
- `isaac_nav_bringup_phase2.launch.py` / `isaac_nav_bringup_phase3.launch.py`: the
  declared `mission_file` launch argument was never actually used — the mission-load
  command concatenated the Python-time default path instead, so
  `mission_file:=/other.json` on the command line was silently ignored (the same bug
  already fixed in the guidance and Phase 4 launches). Now wired through as a real
  launch substitution, with the `-w 2` DDS-discovery-race fix applied for consistency
  with `load_mission.sh`.

### Removed
- Unused imports left over from recent edits: `State` and `MultirotorConfig` in
  `examples/12_px4_v1_vehicle.py`, `TransformStamped` in `examples/dpv_sim/zed_vio_stub.py`.

## 2026-07-15 — Phase 4: real ZED VIO

### Added
- `extensions/zed-isaac-sim/` — Stereolabs' `zed-isaac-sim` v5.1.0, pinned (the June
  2026 release that added Isaac Sim 6.0 support, unblocking real ZED SDK sim-mode
  streaming that PLAN.md had marked unavailable).
- `examples/dpv_sim/zed_sim_camera.py` — mounts a virtual ZED X on the V1 nose and
  streams stereo + IMU to the ZED SDK (IPC, port 30000) via the Stereolabs extension.
- `examples/dpv_sim/zed_odom_to_fcu.py` — adapts the real `zed_wrapper` VIO output to
  the drone's EV input topics (successor to `zed_vio_stub.py`, which faked ZED VIO
  from Cartographer laser odometry).
- `examples/dpv_sim/isaac_nav_bringup_zed.launch.py` — full mission stack with
  `to_fcu_vehicle_visual_odometry` on its real default input (real zed_wrapper VIO
  instead of the Cartographer reroute); Cartographer kept running in shadow mode for
  A/B comparison and to keep feeding the laser corrector.
- `examples/dpv_sim/ev_ready.sh` / `diagnose_ev_chain.py --wait` — readiness gate that
  blocks until EKF2 is fusing EV position with a level attitude, to run before handing
  off to the mission state machine.
- `DPV_ZED_MODE` (`native`/`wrapper`/`off`) in `examples/12_px4_v1_vehicle.py` to
  switch between the Isaac-native RGB-D camera and the real ZED SDK stream.
- `start.sh` tmux window `zed` running the real `zed_wrapper` in `sim_mode`; plain
  `./start.sh` (no argument) now defaults to this phase.

### Fixed
- Depth-camera "blinking": the RGB/depth/camera-info writers shared one render
  product gated at a hardcoded `int(60 / frequency)`, while `Camera.set_frequency`
  applied a second, independent throttle — the two gates drifted out of phase and
  produced an irregular publish cadence. The writer gate is now pass-through
  (`step=1`) with the camera's own frequency as the single rate authority.
- Native ZED camera resolution dropped from 1920×1200 to HD720 (1280×720), matching
  what the real drone's `zed_wrapper` config actually grabs — substantially reduces
  per-frame render cost and RTF jitter.
- EKF2 parameter drift across files: `diagnose_ev_chain.py` and `PLAN.md` still
  referenced the old `EKF2_EV_CTRL=11` eeprom value (valid only for real ZED-VIO
  height, not 2D laser odometry). Reconciled to the working `EKF2_EV_CTRL=1` /
  `EKF2_HGT_REF=0` set, with `set_px4_gps_denied_params_onboard.py --profile
  laser|zed` plus `--ev-ctrl`/`--hgt-ref` overrides for staging back toward the
  real-drone config (`11`/`3`) once the ZED VIO chain is verified stable.

## 2026-07-14 — GPS-denied stack hardening

### Fixed
- Spawn-height tilt-init bug: spawning the vehicle above its resting height made
  EKF2's very first IMU samples a drop-and-bounce, which could initialize tilt
  180° flipped (`Ekf::initialiseTilt`'s degenerate case) — surfaced as a permanent
  `Preflight Fail: Attitude failure (roll)`. Fixed by spawning at the measured
  resting height (`[-9.0, 0.0, 0.1124]`).
- External-vision chain silently never initializing: `cartographer_pose_transformer`
  waits for a message on `/px4_ros2_bridge/odometry/fcu_odom_flu` before it will
  publish `laser_odom_at_fcu`, but no bringup launched the node that publishes it —
  EV never reached PX4 and EKF2 dead-reckoned into an accelerometer-bias failure.
  Added `from_fcu_vehicle_odometry_node` to all GPS-denied bringups.
- uXRCE-DDS "time jump detected" churn: `uxrce_dds_client` timesync compares
  Agent-wall-clock against PX4 sim-time, which structurally drifts at any RTF ≠ 1.0
  and is unrelated to render load. Disabled with `PX4_PARAM_UXRCE_DDS_SYNCT=0`.
- EV-vertical fusion from 2D laser SLAM (which has no valid Z) injected a ~6.8 km
  height estimate and diverged the estimator. `EKF2_EV_CTRL` narrowed to horizontal
  position only (`=1`), height from baro (`EKF2_HGT_REF=0`).

### Added
- `enable_zed_camera` flag on `V1Config` so localization-only phases can drop the
  camera render cost entirely.
- `examples/dpv_sim/diagnose_ev_chain.py` — link-by-link EV chain diagnostic
  (publisher census, EKF2 fusion flags, attitude/EV sample dump).
- Warehouse aisle built procedurally from measured site geometry
  (`examples/dpv_sim/aisle_spec.json`, `warehouse_aisle.py`).

## 2026-07-13/14 — Guidance mode

### Added
- `isaac_nav_bringup_guidance.launch.py` and `./start.sh g` — a GPS-fused
  localization mode that drops Cartographer/EV/pose-correctors entirely and lets
  EKF2 fuse simulated GPS, so arm → offboard → auto-mission flight can be validated
  independent of the SLAM/vision pipeline.
- QGroundControl auto-launch in `start.sh`, matching teardown in `stop.sh`.
- `examples/dpv_sim/load_mission.sh` — CLI mission-state-machine driver
  (load/start/terminate/reset/state), with the `-w 2` fix for a DDS-discovery race
  that could otherwise drop the mission-load message before any subscriber matched.

## 2026-07-13 — DPV warehouse autonomy integration (Phases 1–3)

### Added
- Phase 1: Isaac lidar (`/scan`, frame `laser_link`) → Cartographer SLAM →
  `to_fcu_vehicle_visual_odometry` → PX4 external vision, replacing GPS entirely.
  `isaac_nav_bringup.launch.py` plus the static TF chain
  (`base_link_fcu → laser_link → fcu_imu_base_link_for_laser`, `map → odom_fcu`).
- Phase 2: full mission/navigation chain —
  `warehouse_auto_mission`, `warehouse_path_planner`, `trajectory_generator`,
  `warehouse_pose_corrector_laser_based`, `warehouse_pose_correction_filter`,
  staggered via `TimerAction`s; `gt_odom_pub.py` ground-truth odometry for
  debugging/RViz.
- Phase 3: camera topics remapped to the Gazebo sim contract
  (`/zed/image_raw`, `/zed/depth`, `/zed/color/camera_info`); `zed_vio_stub.py`
  (Cartographer odom relayed as fake ZED VIO) and the vision corrector bringup,
  both opt-in via `launch_vision:=true`.
- `set_px4_gps_denied_params_onboard.py` — sets GPS-denied EKF2 parameters over the
  PX4 onboard MAVLink link (QGroundControl latches the GCS link, so param changes
  must go through the onboard link instead).
- `start.sh` / `stop.sh` — tmux-orchestrated bring-up/teardown across Isaac Sim,
  MicroXRCEAgent, the DPV bringup, and QGroundControl.

## 2026-07-06/13 — V1 vehicle + GPS-denied SLAM pipeline

### Added
- `V1` vehicle (`extensions/pegasus.simulator/.../vehicles/multirotors/v1.py`) — a
  full-scale custom quadrotor with a thrust curve derived from bench-measured motor
  data, a ZED 2i-class RGB-D camera, an RPLIDAR-class 2D lidar, and both a
  `PX4MavlinkBackend` and a `ROS2Backend` attached simultaneously.
- `examples/12_px4_v1_vehicle.py`, `examples/slam_v1/` — GPS-denied SLAM pipeline
  example (slam_toolbox + PX4 external vision) for V1, later superseded by the
  Cartographer-based DPV integration above.

### Fixed
- Render callback registered post-play, corrected `PX4MavlinkBackend` config
  assignment, `V1(config=None)` default (a mutable default that constructed
  `V1Config()` at import time created an orphaned `ROS2Backend`/rclpy node as an
  import side effect, corrupting the `/tf` tree).

## 2026-07-03 — Isaac Sim 6.0 port

Ported the framework from Isaac Sim 5.1 to 6.0 for PX4 v1.16.2 (see
`README_ISAACSIM6_FIXES.md` for the full debug log).

### Fixed
- Rotor thrust, drag, and rolling-moment torque were silently discarded: writing a
  bare `physxForce:force` USD attribute does nothing unless the `PhysxForceAPI`
  schema is applied to the prim. Replaced with the Isaac Sim 6.0
  `isaacsim.core.experimental.prims.RigidPrim` tensor API
  (`apply_forces`/`apply_forces_and_torques_at_pos`, `local_frame=True`).
- `HIL_STATE_QUATERNION`/`HIL_GPS` MAVLink sends crashed on every physics step once
  ground contact made body velocity slightly negative: `ind_airspeed`/`true_airspeed`
  are `uint16`, so `struct.error` fired on any negative or out-of-range value.
  Clamped all `uint16`/`int16` fields and fixed the accel units bug (was sending
  mm/s² instead of mG). Bare `except:` blocks in the send path now log the actual
  exception instead of swallowing it.
- Pegasus extension silently failed to load in the Isaac Sim GUI: `isaacsim.core.api`
  moved to `extsDeprecated/` in 6.0 and is no longer auto-loaded — added as an
  explicit `extension.toml` dependency.
- MAVLink HIL connection type: PX4 v1.16 runs `simulator_mavlink` as a TCP *client*,
  so the simulator must be the TCP *server* (`tcpin`, not `udpin`/`PX4_SIM_PROTOCOL`,
  which is a no-op on this PX4 version).
