<!-- generated-by: gsd-doc-writer -->
# Testing

Pegasus Simulator (and this fork's DPV integration) has **no automated unit/integration
test suite** (no `pytest`, `unittest`, or `ros2 test` targets exist anywhere in this
repository). Verification is done by **running the simulation interactively** and
checking that specific ROS 2 topics/MAVLink state reach expected rates and values, plus
one Python diagnostic script that automates part of that check. This document describes
that manual verification workflow as it actually exists in the repo, rather than
describing a test framework that isn't present.

## Verification approach and setup

There are three layers of verification used across the codebase:

1. **Interactive example smoke-runs** — launching an `examples/*.py` script and confirming
   the vehicle spawns, connects to PX4/ArduPilot, and behaves as expected in the Isaac Sim
   viewport (e.g. arms, takes off, follows a trajectory).
2. **`ros2 topic hz` / `ros2 topic echo` checks** — confirming that specific topics in the
   DPV ROS 2 graph (`examples/dpv_sim/`, `examples/slam_v1/`) are publishing at the
   expected rate, used as pass/fail gates for each bringup phase.
3. **`examples/dpv_sim/diagnose_ev_chain.py`** — a standalone rclpy diagnostic script that
   automates step 2 for the external-vision (EV) fusion chain, printing per-link
   publish rates and a diagnosis of the first broken link.

Setup required before any of the above:

- Isaac Sim launched via the `isaac_run` shell function (wraps
  [`scripts/isaac_run.sh`](../scripts/isaac_run.sh)), which strips system ROS 2 from the
  environment so Isaac's bundled `rclpy` is used instead of a conflicting system install.
- For DPV/ROS 2 checks: `source /opt/ros/humble/setup.bash` then
  `source ~/PegasusSimulator/extensions/dpv-install/setup.bash` (or
  `~/ros2_ws/install/setup.bash` for the older `examples/slam_v1/` pipeline) in the
  terminal used to run `ros2 topic ...` commands or `diagnose_ev_chain.py`.
- A clean process state — stale background processes from a previous run are a common
  source of false failures. See "Teardown before every run" below.

No coverage tool, coverage threshold, or CI test job is configured — see
[CI integration](#ci-integration).

## Running verification checks

### 1. Base vehicle/backend examples (no DPV stack)

Run any numbered example directly and confirm it starts without error and the vehicle
behaves as described in [`README.md`](../README.md#quick-start):

```bash
isaac_run examples/1_px4_single_vehicle.py    # single PX4 vehicle spawns, arms, flies
isaac_run examples/3_ros2_single_vehicle.py   # ROS 2 backend topics publish
isaac_run examples/12_px4_v1_vehicle.py       # this fork's V1 vehicle (used by DPV work)
```

`examples/6_paper_results.py` + `examples/7_paper_plots.ipynb` are a reproducibility
check rather than a pass/fail test: the script logs trajectory-tracking error to
`examples/results/statistics_*.npz`, and the notebook plots position/error curves for
visual comparison against the published paper results — there is no automated
assertion or threshold.

### 2. DPV localization stack (Phase 1 — Cartographer + external vision)

Per [`examples/dpv_sim/README.md`](../examples/dpv_sim/README.md), after launching Isaac
Sim, `MicroXRCEAgent`, and the Phase 1 bringup (`isaac_nav_bringup.launch.py`), verify
from an interactive terminal with DPV sourced:

```bash
ros2 topic hz /scan                           # ~10 Hz, frame laser_link
ros2 topic hz /fmu/out/vehicle_imu            # > 0 Hz
ros2 topic hz /imu                            # px4_imu_converter output
ros2 topic hz /cartographer/odom              # Cartographer running
ros2 topic hz /cartographer/laser_odom_at_fcu # Cartographer odom
ros2 topic hz /fmu/in/vehicle_visual_odometry # EV flowing to PX4
```

Pass condition: PX4 logs "EKF2 commencing external vision fusion" and preflight failures
clear (~30 s after EV data starts flowing).

### 3. `diagnose_ev_chain.py` — automated EV chain diagnostic

[`examples/dpv_sim/diagnose_ev_chain.py`](../examples/dpv_sim/diagnose_ev_chain.py)
listens on the same topics as the manual checks above (plus
`/fmu/out/estimator_status_flags` and `/fmu/out/vehicle_attitude`) for a fixed 12-second
window, then prints:

- A **publisher census** per topic (flags a stale/duplicate publisher, e.g. a leftover
  `slam_v1` bridge still writing to a topic in the wrong frame).
- **Per-link Hz** for each stage of the chain (`from_fcu` → `laser_odom` →
  `vehicle_visual_odometry` → `estimator_status_flags`), with the first zero-rate link
  reported as the break point.
- **EKF2 fusion intent flags** (`cs_ev_pos`, `cs_ev_yaw`, `cs_ev_hgt`, `cs_ev_vel`,
  `reject_hor_pos`, `reject_ver_pos`, `reject_yaw`) read from the live
  `estimator_status_flags` topic, with a diagnosis (e.g. "`cs_ev_pos` FALSE: EKF2 is NOT
  fusing EV position — check `param show EKF2_EV_CTRL`" or "fails the innovation gate").
- Sample EV pose/orientation and Cartographer laser-odom pose for a quick frame/sign
  sanity check.

Run it while the sim and DPV bringup are already up:

```bash
source /opt/ros/humble/setup.bash
source ~/PegasusSimulator/extensions/dpv-install/setup.bash
python3 ~/PegasusSimulator/examples/dpv_sim/diagnose_ev_chain.py
```

### 4. Phase 2 — mission/planner/trajectory stack

```bash
ros2 topic hz /warehouse_path_planner/state
ros2 topic hz /trajectory_generator/state
ros2 topic hz /warehouse_auto_mission/mission_state
```

### 5. Phase 3 — camera + vision corrector

```bash
ros2 topic hz /zed/image_raw          # ZED color images
ros2 topic hz /zed/depth              # ZED depth images
ros2 topic hz /zed/color/camera_info  # Camera intrinsics
ros2 topic hz /zed/zed_node/odom_zed_to_fcu  # only if launch_vision:=true
```

### 6. Guidance mode (GPS-fused, no SLAM/EV)

```bash
# QGC: confirm 3D GPS fix and EKF2 preflight checks pass — no "no vision" warning.
ros2 topic echo /fmu/out/vehicle_gps_position --once
ros2 topic echo /fmu/out/estimator_status_flags --once   # cs_gps set, cs_ev_pos unset
ros2 topic hz /px4_ros2_bridge/odometry/fcu_pose_at_imu
ros2 topic hz /px4_ros2_bridge/odometry/fcu_odom_flu
# warehouse_auto_mission logs: MissionCommand received, WaypointList published
# state machine progression: INIT -> IDLE -> READY -> CHANGE_TO_POSITION_MODE ->
#   ARMING -> CHANGE_TO_OFFBOARD_MODE -> TAKING_OFF -> AUTO_MISSION -> MISSION_COMPLETED
```

`./start.sh g` (see [`examples/dpv_sim/start.sh`](../examples/dpv_sim/start.sh)) launches
this mode directly in a `tmux` session, with a dedicated `status` window pre-populated
with several of the `ros2 topic hz` commands above.

### 7. Earlier `slam_v1` pipeline (predates the DPV integration)

Per [`examples/slam_v1/README.md`](../examples/slam_v1/README.md):

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 topic hz  /v1_0/rplidar_c1/laserscan       # ~10 Hz scans from Isaac
ros2 topic echo /pose --once                     # slam_toolbox pose (slam_map frame)
ros2 topic hz  /fmu/in/vehicle_visual_odometry   # vision odom flowing to PX4
```
Pass condition: QGC reports a valid local position with GPS disabled; Position mode and
takeoff become available once EKF2 converges on vision.

### Teardown before every run

Stale background processes are called out as a recurring source of false failures in
`examples/dpv_sim/README.md`. Before re-running any DPV verification:

```bash
pkill -f "bin/px4|kit/python|MicroXRCEAgent|slam_toolbox|cartographer|px4_ros2_bridge"
ss -tlnp | grep 4560   # must be empty before starting a new run
```

[`examples/dpv_sim/stop.sh`](../examples/dpv_sim/stop.sh) automates this — it kills all
Pegasus/DPV-related processes by name pattern and reports whether TCP port `4560` was
freed.

## Writing new verification checks

There is no test file naming convention or test helper library, since there is no test
framework. When adding a new verification check, follow the existing patterns in the
repo:

- **Topic-rate smoke checks** — add the relevant `ros2 topic hz <topic>` /
  `ros2 topic echo <topic> --once` command to the "Verify" section of the relevant
  `examples/dpv_sim/README.md` phase, next to the launch instructions for that phase.
- **Standalone diagnostic scripts** — follow the pattern in
  `examples/dpv_sim/diagnose_ev_chain.py`: a single-purpose rclpy `Node` script that
  subscribes with a maximally-compatible QoS profile (`BEST_EFFORT` / `VOLATILE`,
  `KEEP_LAST` depth 10) to the topics under test, listens for a fixed duration, and
  prints publisher census + per-topic rate + a plain-language diagnosis rather than an
  exit code. Save new scripts under `examples/dpv_sim/` alongside the existing ones and
  document the run command in its module docstring (matching
  `diagnose_ev_chain.py`'s docstring style).
- **Config/param scripts used as verification setup** — e.g.
  `examples/dpv_sim/set_px4_gps_denied_params_onboard.py` — are one-shot MAVLink
  parameter setters, not tests; keep new scripts of this kind similarly single-purpose.

## Coverage requirements

No coverage tool or threshold is configured. The closest equivalent to a coverage/exit
criterion is the **"Tested Configuration"** table in the root
[`README.md`](../README.md#tested-configuration-this-fork), which states the specific
software versions this fork has been "validated end-to-end (spawn → PX4 boot → EKF init
→ arm → takeoff)" against — that end-to-end sequence is the informal acceptance
criterion for a working setup, verified manually rather than by an automated suite.

## CI integration

`.github/workflows/` contains two workflows, neither of which runs tests:

| Workflow | File | Trigger | What it runs |
|---|---|---|---|
| `Run code linters` | [`.github/workflows/pre-commit.yaml`](../.github/workflows/pre-commit.yaml) | `pull_request` | `pre-commit/action@v3.0.0`, running the hooks defined in [`.pre-commit-config.yaml`](../.pre-commit-config.yaml) (`black`, `flake8` with `flake8-simplify`/`flake8-return`, `pyupgrade --py37-plus`, plus generic hygiene hooks — trailing whitespace, YAML validity, merge-conflict markers, symlink checks, private-key detection). |
| `Build docs` | [`.github/workflows/doc.yaml`](../.github/workflows/doc.yaml) | `push`/`pull_request` to `main`, gated to `github.repository == 'PegasusSimulator/PegasusSimulator'` | Installs `docs/requirements.txt`, runs `make html` in `docs/`, and deploys the built Sphinx site to GitHub Pages. |

Since these workflows are scoped to the upstream `PegasusSimulator/PegasusSimulator`
repository, the `Build docs` deploy step will not run on a fork; the lint workflow runs
on any pull request regardless of fork.
