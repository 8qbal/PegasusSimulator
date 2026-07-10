#!/usr/bin/env python3
"""
| File: px4_vision_bridge.py
| Description: Bridges the slam_toolbox pose estimate into PX4's external-vision input.
|   Subscribes:  /pose (geometry_msgs/PoseWithCovarianceStamped, ENU/FLU, slam_map frame)
|   Publishes:   /fmu/in/vehicle_visual_odometry (px4_msgs/VehicleOdometry, NED/FRD)
| PX4 side: uxrce_dds_client (autostarted by SITL on udp 8888) + MicroXRCEAgent must be running.
| Run with the system ROS 2 Humble + the px4_msgs overlay (see run_slam.sh).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_msgs.msg import VehicleOdometry


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


# World rotation ENU -> NED and body rotation FLU -> FRD are the same quaternion:
# a 180-degree rotation about the (1,1,0)/sqrt(2) axis: q = [0, sqrt(2)/2, sqrt(2)/2, 0] (w,x,y,z)
Q_ENU_TO_NED = np.array([0.0, np.sqrt(0.5), np.sqrt(0.5), 0.0])
Q_FLU_TO_FRD = np.array([0.0, 1.0, 0.0, 0.0])  # 180 deg about body X


class PX4VisionBridge(Node):

    def __init__(self):
        super().__init__("px4_vision_bridge")

        # PX4 uXRCE-DDS subscribers expect best-effort QoS
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._odom_pub = self.create_publisher(VehicleOdometry, "/fmu/in/vehicle_visual_odometry", px4_qos)
        self._pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/pose", self._on_pose, 10)
        self._n_sent = 0
        self.get_logger().info("px4_vision_bridge up: /pose (ENU) -> /fmu/in/vehicle_visual_odometry (NED)")

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation

        # Position ENU -> NED
        pos_ned = np.array([p.y, p.x, -p.z], dtype=np.float32)

        # Orientation: q_ned_frd = q_enu2ned * q_enu_flu * q_flu2frd
        q_enu = np.array([o.w, o.x, o.y, o.z])
        q_ned = quat_mul(quat_mul(Q_ENU_TO_NED, q_enu), Q_FLU_TO_FRD)

        out = VehicleOdometry()
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        out.timestamp = now_us
        out.timestamp_sample = now_us
        out.pose_frame = VehicleOdometry.POSE_FRAME_NED
        out.position = pos_ned
        out.q = np.array([q_ned[0], q_ned[1], q_ned[2], q_ned[3]], dtype=np.float32)
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        out.velocity = np.array([np.nan] * 3, dtype=np.float32)          # not estimated by 2D SLAM
        out.angular_velocity = np.array([np.nan] * 3, dtype=np.float32)  # not estimated
        # Conservative fixed variances from the pose covariance diagonal (fall back to 0.05 m / 0.05 rad)
        cov = np.array(msg.pose.covariance).reshape(6, 6)
        pvar = np.clip(np.array([cov[1, 1], cov[0, 0], cov[2, 2]]), 1e-4, 1.0)
        out.position_variance = pvar.astype(np.float32)
        out.orientation_variance = np.array([0.05, 0.05, max(1e-4, min(1.0, cov[5, 5]))], dtype=np.float32)
        out.velocity_variance = np.array([np.nan] * 3, dtype=np.float32)
        out.reset_counter = 0
        out.quality = 0

        self._odom_pub.publish(out)
        self._n_sent += 1
        if self._n_sent % 100 == 1:
            self.get_logger().info(
                f"vision odom #{self._n_sent}: ned=({pos_ned[0]:+.2f}, {pos_ned[1]:+.2f}, {pos_ned[2]:+.2f})"
            )


def main():
    rclpy.init()
    node = PX4VisionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
