#!/usr/bin/env bash
# start.sh — launch Pegasus DPV simulation in tmux
#
# Usage:  ./start.sh [1|2|3]
#   phase 1 = localization only (default)
#   phase 2 = + mission/navigation
#   phase 3 = + vision corrector (off by default, use launch_vision:=true for on)
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
    *) echo "Invalid phase: $PHASE (use 1, 2, or 3)"; exit 1 ;;
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
tmux new-window -t "$SESSION" -n isaac
tmux send-keys -t "$SESSION:1" \
    "cd $REPO && echo '=== Isaac Sim (Phase $PHASE) ===' && scripts/isaac_run.sh examples/12_px4_v1_vehicle.py" Enter

# --- Window 2: MicroXRCEAgent ---
tmux new-window -t "$SESSION" -n agent
tmux send-keys -t "$SESSION:2" \
    "echo '=== MicroXRCEAgent ===' && sleep 3 && MicroXRCEAgent udp4 -p 8888" Enter

# --- Window 3: status ---
tmux new-window -t "$SESSION" -n status
tmux send-keys -t "$SESSION:3" \
    "source /opt/ros/humble/setup.bash && source $DPV_INSTALL/setup.bash && echo '=== Status Commands ===' && echo '' && echo 'ros2 topic hz /scan' && echo 'ros2 topic hz /fmu/in/vehicle_visual_odometry' && echo 'ros2 topic hz /cartographer/odom' && echo 'ros2 topic hz /zed/image_raw' && echo '' && echo '# Set EKF2 params (run once, PX4 must be running):' && echo 'python3 $REPO/examples/dpv_sim/set_px4_gps_denied_params_onboard.py'" Enter

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
