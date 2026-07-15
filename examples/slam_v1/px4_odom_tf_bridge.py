#!/usr/bin/env python3
"""
| File: px4_odom_tf_bridge.py
| Description: Publishes the odom -> base_link TF that slam_toolbox needs as its motion
|   prior, sourced from PX4's EKF2 odometry (so slam is not fed ground-truth pose).
|   Subscribes:  /fmu/out/vehicle_odometry (px4_msgs/VehicleOdometry, NED/FRD)
|   Broadcasts:  TF  odom (ENU) -> v1__base_link (FLU)
|
| Why this exists: slam_toolbox's message filter drops every scan unless it can look up
| the transform from the scan frame to its odom_frame. With no wheel odometry on the
| drone, that transform has to come from somewhere - here, PX4's own state estimate.
|
| Caveat (feedback path): with EKF2_EV_CTRL=11, PX4's odometry is partly derived from the
| vision estimate that slam produces (slam -> px4_vision_bridge -> EKF2 -> here -> slam).
| slam only uses odom as a *prior* and corrects it via scan matching (the map->odom TF
| absorbs the drift), and attitude/height (the dominant terms for de-rotating 2D scans)
| come from IMU/baro independently of vision, so this is stable in practice. If it ever
| locks in drift, feed a vision-independent odometry instead.
|
| Run with the system ROS 2 Humble + the px4_msgs overlay (see run_slam.sh), with
| use_sim_time:=true so the TF stamps share the sim clock with the scans.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def quat_mul(a, b):
    """Hamilton product of quaternions in [w, x, y, z] order."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


# NED<->ENU (swap x/y, flip z) and FRD<->FLU (flip y/z) are both 180-degree rotations and
# their own inverse, so the same quaternions convert PX4 (NED/FRD) back to ROS (ENU/FLU).
Q_NED_TO_ENU = np.array([0.0, np.sqrt(0.5), np.sqrt(0.5), 0.0])  # 180 deg about (1,1,0)/sqrt2
Q_FRD_TO_FLU = np.array([0.0, 1.0, 0.0, 0.0])                    # 180 deg about body X

ODOM_FRAME = "odom"
BASE_FRAME = "v1__base_link"


class PX4OdomTFBridge(Node):

    def __init__(self):
        super().__init__("px4_odom_tf_bridge")

        # PX4 uXRCE-DDS publishers use best-effort / transient-local QoS
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._tf = TransformBroadcaster(self)
        self._sub = self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self._on_odom, px4_qos)
        self._n = 0
        self.get_logger().info(
            f"px4_odom_tf_bridge up: /fmu/out/vehicle_odometry (NED) -> TF {ODOM_FRAME}->{BASE_FRAME} (ENU)")

    def _on_odom(self, msg: VehicleOdometry):
        # IMPORTANT: PX4's odometry POSITION is NOT usable here. GPS-denied and before vision
        # converges, EKF2 has no horizontal/height aiding, so its position dead-reckons on
        # accelerometer bias and diverges to hundreds/thousands of metres within seconds
        # (observed: -358 m -> -2552 m while the drone sat still). Feeding that to slam_toolbox
        # as a prior is worse than useless. So we publish ATTITUDE ONLY (valid from IMU/mag,
        # independent of vision) with ZERO translation, and let slam_toolbox's scan matcher
        # estimate translation. The map->odom transform then carries the full position; /pose
        # (in slam_map) is the real, scan-matched estimate fed back to PX4 as external vision,
        # which is what finally gives EKF2 a bounded position. (If a lidar-odometry front-end
        # such as rf2o_laser_odometry is installed later, prefer that as the odom source.)
        east, north, up = 0.0, 0.0, 0.0

        # Orientation: q_enu_flu = q_ned2enu * q_ned_frd * q_frd2flu
        q_ned = np.array([float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3])])
        # PX4 sends NaN quaternion before the estimator has a valid attitude; skip those.
        if not np.all(np.isfinite(q_ned)) or np.allclose(q_ned, 0.0):
            return
        q_enu = quat_mul(quat_mul(Q_NED_TO_ENU, q_ned), Q_FRD_TO_FLU)
        q_enu = q_enu / np.linalg.norm(q_enu)

        if not (np.isfinite(east) and np.isfinite(north) and np.isfinite(up)):
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()  # sim time (use_sim_time)
        t.header.frame_id = ODOM_FRAME
        t.child_frame_id = BASE_FRAME
        t.transform.translation.x = east
        t.transform.translation.y = north
        t.transform.translation.z = up
        t.transform.rotation.w = float(q_enu[0])
        t.transform.rotation.x = float(q_enu[1])
        t.transform.rotation.y = float(q_enu[2])
        t.transform.rotation.z = float(q_enu[3])
        self._tf.sendTransform(t)

        self._n += 1
        if self._n % 200 == 1:
            self.get_logger().info(
                f"odom TF #{self._n}: enu=({east:+.2f}, {north:+.2f}, {up:+.2f})")


def main():
    rclpy.init()
    node = PX4OdomTFBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
