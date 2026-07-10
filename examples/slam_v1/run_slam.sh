#!/bin/bash
# | File: run_slam.sh
# | Description: Launches the system-side half of the V1 GPS-denied SLAM pipeline:
# |   1. MicroXRCEAgent           (PX4 uXRCE-DDS <-> ROS 2, udp 8888)
# |   2. static TF                v1__base_link -> rplidar_c1 (lidar mount, z=0.135)
# |   3. slam_toolbox             (async online mapping on /v1_0/rplidar_c1/laserscan)
# |   4. px4_vision_bridge        (/pose -> /fmu/in/vehicle_visual_odometry)
# |
# | Start the simulator FIRST in another terminal:
# |   isaac_run examples/12_px4_v1_vehicle.py
# | Then run this script. Ctrl-C stops everything it started.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
LOG_DIR="/tmp/v1_slam_logs"
mkdir -p "$LOG_DIR"

# System ROS 2 Humble + px4_msgs overlay (must be sourced before `set -u`:
# the ROS setup scripts reference unset variables)
source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi
set -u

PIDS=()
cleanup() {
    echo ""
    echo "shutting down slam pipeline..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    wait 2>/dev/null
    exit 0
}
trap cleanup INT TERM

echo "[1/4] MicroXRCEAgent (udp 8888) -> $LOG_DIR/xrce_agent.log"
MicroXRCEAgent udp4 -p 8888 > "$LOG_DIR/xrce_agent.log" 2>&1 &
PIDS+=($!)

echo "[2/4] static TF v1__base_link -> rplidar_c1 (0, 0, 0.135)"
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0.135 --roll 0 --pitch 0 --yaw 0 \
    --frame-id v1__base_link --child-frame-id rplidar_c1 \
    --ros-args -p use_sim_time:=true > "$LOG_DIR/static_tf.log" 2>&1 &
PIDS+=($!)

echo "[3/4] slam_toolbox (async online, sim time) -> $LOG_DIR/slam_toolbox.log"
ros2 run slam_toolbox async_slam_toolbox_node \
    --ros-args --params-file "$SCRIPT_DIR/slam_toolbox_params.yaml" \
    -p use_sim_time:=true > "$LOG_DIR/slam_toolbox.log" 2>&1 &
PIDS+=($!)

echo "[4/4] px4_vision_bridge -> $LOG_DIR/vision_bridge.log"
python3 "$SCRIPT_DIR/px4_vision_bridge.py" > "$LOG_DIR/vision_bridge.log" 2>&1 &
PIDS+=($!)

sleep 2
echo ""
echo "pipeline up. Useful checks:"
echo "  ros2 topic hz /v1_0/rplidar_c1/laserscan     # scans arriving from Isaac"
echo "  ros2 topic echo /pose --once                  # slam_toolbox pose estimate"
echo "  ros2 topic hz /fmu/in/vehicle_visual_odometry # vision odom flowing to PX4"
echo "  ros2 run rviz2 rviz2                          # visualize slam_map / scan"
echo ""
echo "to make PX4 fuse it (one-time, while PX4 runs):"
echo "  $REPO_DIR/.venv/bin/python $SCRIPT_DIR/set_px4_gps_denied_params.py"
echo ""
echo "Ctrl-C to stop."
wait
