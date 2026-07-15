#!/usr/bin/env bash
# ==============================================================================
# load_mission.sh — drive a mission file through the real DPV mission stack from
# the terminal, without RViz.
#
# Same effect as the RViz WarehouseCommanderPanel buttons: both publish
# warehouse_ros2_msgs/MissionCommand on /onboard_command, which
# mission_state_controller (warehouse_auto_mission) consumes.
#
# Modelled on warehouse_gz_sim_ws/scripts/load_mission.sh, but sourcing this
# repo's extensions/dpv-install instead of that workspace's paths.config profile.
#
# Usage:
#   ./load_mission.sh load  <mission.json>   # IDLE  -> READY   (cmd_int 0)
#   ./load_mission.sh start                  # READY -> flying  (cmd_int 2)
#   ./load_mission.sh terminate              # abort            (cmd_int 3)
#   ./load_mission.sh reset                  # back to IDLE     (cmd_int 4)
#   ./load_mission.sh state                  # print current mission state
#
# Loading only gets the state machine to READY - it does NOT fly. Send `start`
# once PX4 reports "Ready for takeoff!" (i.e. preflight checks pass).
# ==============================================================================
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DPV_INSTALL="$REPO/extensions/dpv-install"

# /opt/ros setup.bash reads several vars as $VAR rather than ${VAR:-}, which trips
# `set -u`; disable it just around the sourcing.
set +u
source /opt/ros/humble/setup.bash
source "$DPV_INSTALL/setup.bash"
set -u

# cmd_int values from warehouse_ros2_msgs/msg/MissionCommand:
#   0 GCS_CMD_LOAD_MISSION   1 GCS_CMD_CONTINUE_MISSION   2 GCS_CMD_START_MISSION
#   3 GCS_CMD_TERMINATE_MISSION                           4 GCS_CMD_RESET_MISSION
send() {
  local cmd_int="$1" cmd_string="${2:-}"
  # -w 2: wait until at least 2 subscribers have been discovered before publishing.
  # Without it, `ros2 topic pub --once` creates a publisher, sends, and exits before DDS
  # has finished matching the subscribers, and the message is silently dropped - the
  # mission just never loads and mission_state_controller sits on
  # "Waiting for : [mission_waypoints]" with no error anywhere. /onboard_command has 3
  # subscribers when the guidance bringup is fully up (mission_parser,
  # mission_state_controller, payload_control); 2 is a safe floor that still proves
  # discovery has run. The reference warehouse_gz_sim_ws/scripts/load_mission.sh has the
  # same race and loses commands the same way.
  ros2 topic pub --once -w 2 /onboard_command warehouse_ros2_msgs/msg/MissionCommand \
    "{cmd_type: 0, cmd_int: $cmd_int, cmd_string: '$cmd_string', cmd_param1_int: 0}"
}

case "${1:-}" in
  load)
    MISSION="${2:-}"
    [ -n "$MISSION" ] || { echo "usage: $0 load <mission.json>" >&2; exit 2; }
    [ -f "$MISSION" ] || { echo "Mission file not found: $MISSION" >&2; exit 1; }
    # mission_state_controller resolves relative paths against its own CWD, not this
    # shell's, so always hand it an absolute path.
    ABS="$(cd "$(dirname "$MISSION")" && pwd)/$(basename "$MISSION")"
    echo "LOAD_MISSION: $ABS"
    send 0 "$ABS"
    echo "Loaded. Check state with: $0 state   then fly with: $0 start"
    ;;
  start)     echo "START_MISSION";     send 2 ;;
  terminate) echo "TERMINATE_MISSION"; send 3 ;;
  reset)     echo "RESET_MISSION";     send 4 ;;
  state)
    timeout 10 ros2 topic echo /warehouse_auto_mission/mission_state --once \
      | grep -E "state:|state_name:|current_waypoint_id:|total_waypoints:" || \
      echo "no mission_state yet - is the guidance bringup running?"
    ;;
  *)
    sed -n '2,25p' "$0" | sed 's/^# \?//'
    exit 2
    ;;
esac
