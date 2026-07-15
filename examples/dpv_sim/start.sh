#!/usr/bin/env bash
# start.sh — launch Pegasus DPV simulation in tmux
#
# Usage:  ./start.sh [1|2|3|g]
#   phase 1 = localization only (default)
#   phase 2 = + mission/navigation
#   phase 3 = + vision corrector (off by default, use launch_vision:=true for on)
#   phase g = guidance mode: GPS-fused localization (no SLAM/EV/vision), real
#             mission/planner/trajectory stack, arm->offboard->auto-mission flight
#
# Requires: tmux
# The session is named 'dpv-sim'. Attach:  tmux attach -t dpv-sim
# Detach from session: Ctrl+B, D
set -euo pipefail

PHASE="${1:-1}"
SESSION="dpv-sim"
REPO="$HOME/PegasusSimulator"
DPV_INSTALL="$REPO/extensions/dpv-install"
PX4_HOME="$HOME/PX4-Autopilot"
QGC="$HOME/Downloads/QGroundControl-x86_64.AppImage"

case "$PHASE" in
    1) BRINGUP="$REPO/examples/dpv_sim/isaac_nav_bringup.launch.py" ;;
    2) BRINGUP="$REPO/examples/dpv_sim/isaac_nav_bringup_phase2.launch.py" ;;
    3) BRINGUP="$REPO/examples/dpv_sim/isaac_nav_bringup_phase3.launch.py" ;;
    g) BRINGUP="$REPO/examples/dpv_sim/isaac_nav_bringup_guidance.launch.py" ;;
    *) echo "Invalid phase: $PHASE (use 1, 2, 3, or g)"; exit 1 ;;
esac

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
tmux new-window -t "$SESSION" -n isaac
tmux send-keys -t "$SESSION:1" \
    "cd $REPO && $PX4_PARAMS && $GUIDANCE_ENV && echo '=== Isaac Sim (Phase $PHASE) ===' && scripts/isaac_run.sh examples/12_px4_v1_vehicle.py" Enter

# --- Window 2: MicroXRCEAgent ---
tmux new-window -t "$SESSION" -n agent
tmux send-keys -t "$SESSION:2" \
    "echo '=== MicroXRCEAgent ===' && sleep 3 && MicroXRCEAgent udp4 -p 8888" Enter

# --- Window 3: status ---
tmux new-window -t "$SESSION" -n status
tmux send-keys -t "$SESSION:3" \
    "source /opt/ros/humble/setup.bash && source $DPV_INSTALL/setup.bash && echo '=== Status Commands ===' && echo '' && echo 'ros2 topic hz /scan' && echo 'ros2 topic hz /fmu/in/vehicle_visual_odometry' && echo 'ros2 topic hz /cartographer/laser_odom_at_fcu' && echo 'ros2 topic hz /zed/image_raw' && echo '' && echo '# EKF2 GPS-denied params are auto-injected at PX4 boot (PX4_PARAM_* env, isaac window).' && echo '# Manual fallback if needed: python3 $REPO/examples/dpv_sim/set_px4_gps_denied_params_onboard.py'" Enter

# --- Window 4: QGroundControl (requires AppImage) ---
tmux new-window -t "$SESSION" -n qgc
if [ -x "$QGC" ]; then
    tmux send-keys -t "$SESSION:4" \
        "echo '=== QGroundControl ===' && sleep 5 && $QGC" Enter
else
    tmux send-keys -t "$SESSION:4" \
        "echo 'QGroundControl AppImage not found at $QGC'" Enter
fi

# Select the Isaac window (it's the first to start)
tmux select-window -t "$SESSION:1"

# Attach if in a terminal, otherwise print instructions
if [ -t 0 ]; then
    tmux attach -t "$SESSION"
fi
