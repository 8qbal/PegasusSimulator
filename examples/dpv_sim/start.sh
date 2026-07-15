#!/usr/bin/env bash
# start.sh — launch Pegasus DPV simulation in tmux
#
# Usage:  ./start.sh [1|2|3|4|g]
#   phase 4 = REAL ZED VIO as the EV source (default): Stereolabs zed-isaac-sim
#             streamer in Isaac -> ZED SDK -> real zed_wrapper (sim_mode, tmux
#             window "zed") -> zed_odom_to_fcu.py -> PX4. Full mission stack,
#             cartographer in shadow mode. One-time setup:
#               cd extensions/zed-isaac-sim && ./build.sh
#   phase 1 = localization only (laser EV)
#   phase 2 = + mission/navigation (laser EV)
#   phase 3 = + vision corrector on the stub (off by default; launch_vision:=true)
#   phase g = guidance mode: GPS-fused localization (no SLAM/EV/vision), real
#             mission/planner/trajectory stack, arm->offboard->auto-mission flight
#
# Requires: tmux
# The session is named 'dpv-sim'. Attach:  tmux attach -t dpv-sim
# Detach from session: Ctrl+B, D
set -euo pipefail

PHASE="${1:-4}"
SESSION="dpv-sim"
REPO="$HOME/PegasusSimulator"
DPV_INSTALL="$REPO/extensions/dpv-install"
PX4_HOME="$HOME/PX4-Autopilot"
QGC="$HOME/Downloads/QGroundControl-x86_64.AppImage"

case "$PHASE" in
    1) BRINGUP="$REPO/examples/dpv_sim/isaac_nav_bringup.launch.py" ;;
    2) BRINGUP="$REPO/examples/dpv_sim/isaac_nav_bringup_phase2.launch.py" ;;
    3) BRINGUP="$REPO/examples/dpv_sim/isaac_nav_bringup_phase3.launch.py" ;;
    4) BRINGUP="$REPO/examples/dpv_sim/isaac_nav_bringup_zed.launch.py" ;;
    g) BRINGUP="$REPO/examples/dpv_sim/isaac_nav_bringup_guidance.launch.py" ;;
    *) echo "Invalid phase: $PHASE (use 1, 2, 3, 4, or g)"; exit 1 ;;
esac

# Phase 4 hard prerequisite: the Stereolabs streamer plugin must have been built once.
ZED_PLUGIN="$REPO/extensions/zed-isaac-sim/exts/sl.sensor.camera/bin/libsl.sensor.camera.plugin.so"
if [ "$PHASE" = "4" ] && [ ! -f "$ZED_PLUGIN" ]; then
    echo "Phase 4 needs the zed-isaac-sim streamer plugin (not built yet):"
    echo "  cd $REPO/extensions/zed-isaac-sim && ./build.sh"
    exit 1
fi

# Kill existing session if any
tmux kill-session -t "$SESSION" 2>/dev/null || true

echo "Starting Phase $PHASE in tmux session '$SESSION'..."
echo "Attach:  tmux attach -t $SESSION"
echo "Detach:  Ctrl+B, D"
echo ""

# --- Window 0: bringup (bottom pane, where user watches output) ---
tmux new-session -d -s "$SESSION" -n bringup
tmux send-keys -t "$SESSION:0" \
    "source /opt/ros/humble/setup.bash && source $DPV_INSTALL/setup.bash && echo '=== DPV Bringup (Phase $PHASE) ===' && sleep 2 && ros2 launch $BRINGUP" Enter

# --- Window 1: Isaac Sim ---
# PX4_PARAM_* env vars are applied by the SITL rcS at boot (param set), before EKF2
# starts, so the GPS-denied/external-vision config needs no manual script or reboot.
# Pegasus's px4_launch_tool passes os.environ through to the px4 process.
#
# EKF2_EV_CTRL bits: 0=horiz pos, 1=vert pos, 2=velocity, 3=yaw.
#   =1 -> horizontal position ONLY. The 2D laser SLAM has no real Z, so fusing EV
#   vertical (bit 1) injected a ~6.8 km height and blew up the estimator (roll flip,
#   accel-bias runaway). EV yaw (bit 3) reads 90 deg off in NED and is gated out, so
#   it's left off too; mag provides heading. Height comes from baro (EKF2_HGT_REF=0),
#   which example 12 enables. To re-enable laser heading later, sort the map-frame yaw
#   convention first, then set EKF2_EV_CTRL=9 (horiz+yaw).
# EKF2_MAG_CHECK=0: disable the mag field-strength/inclination gate. GPS-denied, it
#   validates against a hardcoded average field (not the real Jakarta field from the sim
#   mag model), so in a clean sim (no hard-iron) it only risks spurious mag rejections
#   that stall heading convergence. Safe to disable in sim; heading is mag-only here.
# UXRCE_DDS_SYNCT=0: disable uxrce_dds_client's timestamp synchronisation. It measures
#   the offset between the *Agent OS (wall) clock* and *PX4 time*, but PX4 SITL here runs
#   on Isaac-driven sim time at RTF ~0.5, so that offset genuinely drifts ~500 ms per wall
#   second. Timesync (src/lib/timesync/Timesync.hpp) rejects samples >100 ms off its
#   estimate and resets the filter after 10 consecutive ones, producing an endless
#   "time jump detected" -> "sync no longer converged" -> "converged" loop. This is
#   structural at any RTF != 1.0 and is NOT caused by render load (the ZED camera only
#   moves the RTF value, never to exactly 1.0). Disabling it makes the /fmu/out/* stamps
#   pass through as raw PX4 sim time, which is what the ROS 2 side already expects:
#   every node runs use_sim_time:=true against the /clock that Pegasus's ROS2Backend
#   publishes from the same sim clock. Applied at rcS:134 (param set), well before
#   uxrce_dds_client start at rcS:320, so the param's reboot_required is satisfied.
PX4_SIM_TIME_PARAMS="PX4_PARAM_UXRCE_DDS_SYNCT=0"
PX4_GPS_DENIED_PARAMS="export $PX4_SIM_TIME_PARAMS PX4_PARAM_EKF2_GPS_CTRL=0 PX4_PARAM_EKF2_EV_CTRL=1 PX4_PARAM_EKF2_HGT_REF=0 PX4_PARAM_EKF2_EV_DELAY=50 PX4_PARAM_EKF2_MAG_CHECK=0"
# Guidance mode (phase g): GPS is the position source, so explicitly restore the
# stock/GPS-enabled EKF2 params rather than omitting them — SITL persists params to
# eeprom (build/px4_sitl_default/rootfs/eeprom), so a prior GPS-denied phase run could
# otherwise leave EKF2_GPS_CTRL=0 baked in and silently starve the estimator of GPS.
# EKF2_GPS_CTRL=7 is PX4's firmware default (2D pos + vel + hgt fusion, verified via
# PARAM_DEFINE_INT32(EKF2_GPS_CTRL, 7) in the built module_params.c).
PX4_GUIDANCE_PARAMS="export $PX4_SIM_TIME_PARAMS PX4_PARAM_EKF2_GPS_CTRL=7 PX4_PARAM_EKF2_EV_CTRL=0 PX4_PARAM_EKF2_HGT_REF=0 PX4_PARAM_EKF2_MAG_CHECK=1"
# Phase 4 (ZED VIO) stage D-1 uses the same conservative EKF2 set as the laser
# phases: EV horizontal position only + baro height. Once diagnose_ev_chain.py
# shows clean EV innovations from the real ZED VIO, stage the real-drone config
# back in (it has a sane Z, unlike the 2D laser odom):
#   D-2: PX4_PARAM_EKF2_EV_CTRL=3            (h+v position)
#   D-3: PX4_PARAM_EKF2_EV_CTRL=11 PX4_PARAM_EKF2_HGT_REF=3   (drone eeprom config)
# (edit here, or at runtime: set_px4_gps_denied_params_onboard.py --profile zed
#  --ev-ctrl 3|11 [--hgt-ref 3] + PX4 reboot)
if [ "$PHASE" = "g" ]; then
    PX4_PARAMS="$PX4_GUIDANCE_PARAMS"
    # Tells 12_px4_v1_vehicle.py to attach the GPS() sensor (guidance mode needs HIL_GPS
    # reaching PX4); GPS-denied phases 1-3 must NOT set this or EKF2 gets a GPS position
    # source in addition to whatever SLAM/EV is under test.
    GUIDANCE_ENV="export DPV_GUIDANCE_MODE=1"
else
    PX4_PARAMS="$PX4_GPS_DENIED_PARAMS"
    GUIDANCE_ENV="export DPV_GUIDANCE_MODE=0"
fi

# How example 12 provides the ZED (see DPV_ZED_MODE in 12_px4_v1_vehicle.py):
# phase 4 streams the virtual ZED X to the ZED SDK for the real zed_wrapper;
# every other phase keeps the Isaac-native RGB-D camera (/zed/image_raw etc.).
if [ "$PHASE" = "4" ]; then
    ZED_ENV="export DPV_ZED_MODE=wrapper"
else
    ZED_ENV="export DPV_ZED_MODE=native"
fi

tmux new-window -t "$SESSION" -n isaac
tmux send-keys -t "$SESSION:1" \
    "cd $REPO && $PX4_PARAMS && $GUIDANCE_ENV && $ZED_ENV && echo '=== Isaac Sim (Phase $PHASE) ===' && scripts/isaac_run.sh examples/12_px4_v1_vehicle.py" Enter

# --- Window 2: MicroXRCEAgent ---
tmux new-window -t "$SESSION" -n agent
tmux send-keys -t "$SESSION:2" \
    "echo '=== MicroXRCEAgent ===' && sleep 3 && MicroXRCEAgent udp4 -p 8888" Enter

# --- Window 3: zed_wrapper (phase 4 only; placeholder window otherwise so the
# ---           window numbering below stays fixed) ---
# The REAL Stereolabs node, from the same dpv-install the drone runs, consuming
# the Isaac stream via the ZED SDK. camera_model MUST be zedx (the streamer's
# virtual model family; same 12 cm baseline as the drone's zed2i) while
# camera_name:=zed keeps the real /zed/zed_node/* topic namespace. All TF/URDF
# publishing is off: the DPV stack owns the TF tree (base_link_fcu chain), and
# ZED VIO reaches PX4 through topics, not TF. The sleep gives Isaac time to
# boot and start streaming; if the wrapper still came up too early and exited,
# re-run it with Up+Enter in this window.
tmux new-window -t "$SESSION" -n zed
if [ "$PHASE" = "4" ]; then
    tmux send-keys -t "$SESSION:3" \
        "source /opt/ros/humble/setup.bash && source $DPV_INSTALL/setup.bash && echo '=== zed_wrapper (sim_mode, ZED SDK stream from Isaac) ===' && sleep 45 && ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx camera_name:=zed sim_mode:=true sim_address:=127.0.0.1 sim_port:=30000 publish_tf:=false publish_map_tf:=false publish_urdf:=false publish_imu_tf:=false use_sim_time:=true ros_params_override_path:=$REPO/examples/dpv_sim/zed_sim_overrides.yaml" Enter
else
    tmux send-keys -t "$SESSION:3" \
        "echo 'zed_wrapper window unused in phase $PHASE (phase 4 only)'" Enter
fi

# --- Window 4: status ---
tmux new-window -t "$SESSION" -n status
tmux send-keys -t "$SESSION:4" \
    "source /opt/ros/humble/setup.bash && source $DPV_INSTALL/setup.bash && echo '=== Status Commands ===' && echo '' && echo 'ros2 topic hz /scan' && echo 'ros2 topic hz /fmu/in/vehicle_visual_odometry' && echo 'ros2 topic hz /cartographer/laser_odom_at_fcu' && echo 'ros2 topic hz /zed/zed_node/odom                  # phase 4: real ZED VIO' && echo 'ros2 topic hz /zed/zed_node/odom_zed_to_fcu       # phase 4: EV input to to_fcu' && echo 'ros2 topic hz /zed/zed_node/depth/depth_registered # phase 4: real wrapper depth' && echo 'ros2 topic hz /zed/image_raw                      # phases 1-3: native camera' && echo '' && echo '# Mission gate: $REPO/examples/dpv_sim/ev_ready.sh   (blocks until EV fused + level)' && echo '# Full chain report: python3 $REPO/examples/dpv_sim/diagnose_ev_chain.py' && echo '# EKF2 params are auto-injected at PX4 boot (PX4_PARAM_* env, isaac window).' && echo '# Manual fallback: python3 $REPO/examples/dpv_sim/set_px4_gps_denied_params_onboard.py --profile zed'" Enter

# --- Window 5: QGroundControl (requires AppImage) ---
tmux new-window -t "$SESSION" -n qgc
if [ -x "$QGC" ]; then
    tmux send-keys -t "$SESSION:5" \
        "echo '=== QGroundControl ===' && sleep 5 && $QGC" Enter
else
    tmux send-keys -t "$SESSION:5" \
        "echo 'QGroundControl AppImage not found at $QGC'" Enter
fi

# Select the Isaac window (it's the first to start)
tmux select-window -t "$SESSION:1"

# Attach if in a terminal, otherwise print instructions
if [ -t 0 ]; then
    tmux attach -t "$SESSION"
fi
