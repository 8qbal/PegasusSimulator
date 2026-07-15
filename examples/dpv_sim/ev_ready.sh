#!/bin/bash
# | File: examples/dpv_sim/ev_ready.sh
# | Description: Readiness gate for GPS-denied mission ops. Blocks until EKF2 is
# |   fusing EV position (cs_ev_pos, not rejected) AND the attitude is level, i.e.
# |   PX4 will accept Position/Offboard mode instead of throwing "no valid position"
# |   / "High Accelerometer Bias". Run this BEFORE load_mission.sh load/start.
# | Usage: ./ev_ready.sh [--timeout 120]
# | Exit: 0 = ready, 1 = timed out.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/../../extensions/dpv-install/setup.bash"
exec python3 "$SCRIPT_DIR/diagnose_ev_chain.py" --wait "$@"
