#!/bin/bash
# | File: scripts/isaac_run.sh
# | Description: Launch Isaac Sim with a ROS-free environment so its bundled
# |   py3.12 rclpy (provided by the isaacsim.ros2.bridge extension) wins over a
# |   system py3.10 ROS 2 that the shell may have sourced. Without this, an Isaac
# |   example that imports rclpy crashes with:
# |     ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
# |   because a py3.10 rclpy on PYTHONPATH loads into Isaac's py3.12 interpreter.
# | Usage: scripts/isaac_run.sh examples/12_px4_v1_vehicle.py
export ISAACSIM_PATH="${ISAACSIM_PATH:-$HOME/isaacsim}"
_strip() { echo "${1:-}" | tr ':' '\n' | grep -vE '/opt/ros/|/ros2_ws/' | paste -sd ':'; }
export PYTHONPATH="$(_strip "$PYTHONPATH")"
export LD_LIBRARY_PATH="$(_strip "$LD_LIBRARY_PATH")"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH   # cheap insurance vs transitive ROS discovery

# Point the ROS 2 bridge at Isaac's *internal* (bundled) ROS 2 libraries, so it does
# not need a system ROS 2 install. Without this the bridge fails to load librcutils.so
# and never injects its py3.12 rclpy -> `ModuleNotFoundError: No module named 'rclpy'`.
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$ISAACSIM_PATH/exts/isaacsim.ros2.core/humble/lib"

exec "$ISAACSIM_PATH/python.sh" "$@"
