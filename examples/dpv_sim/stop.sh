#!/usr/bin/env bash
# stop.sh — kill all Pegasus/DPV simulation processes
set -euo pipefail

echo "Stopping all Pegasus DPV processes..."

pkill -9 -f "bin/px4" 2>/dev/null || true
pkill -9 -f "kit/python" 2>/dev/null || true
pkill -9 -f "isaac_run.sh" 2>/dev/null || true
pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
pkill -9 -f "slam_toolbox" 2>/dev/null || true
pkill -9 -f "cartographer" 2>/dev/null || true
pkill -9 -f "px4_ros2_bridge" 2>/dev/null || true
pkill -9 -f "warehouse_" 2>/dev/null || true
pkill -9 -f "path_planner" 2>/dev/null || true
pkill -9 -f "trajectory_generator" 2>/dev/null || true
pkill -9 -f "QGroundControl" 2>/dev/null || true
pkill -9 -f "qgroundcontrol" 2>/dev/null || true

sleep 1

if ss -tlnp 2>/dev/null | grep -q 4560; then
    echo "WARNING: port 4560 still in use!"
    ss -tlnp | grep 4560
else
    echo "Port 4560: FREE"
fi

echo "Done."
