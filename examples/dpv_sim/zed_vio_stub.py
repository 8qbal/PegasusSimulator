#!/usr/bin/env python3
"""Relay cartographer laser odom → /zed/zed_node/odom_zed_to_fcu.

The vision corrector consumes ZED VIO odometry at /zed/zed_node/odom_zed_to_fcu,
which is normally produced by zed_wrapper from StereoLabs ZED SDK. Without the
Stereolabs Isaac Sim extension, zed_wrapper cannot run in sim_mode. This node
bridges the gap by republishing Cartographer's laser-based odometry as a
ZED-VIO-compatible Odometry message.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class ZedVioStub(Node):
    def __init__(self):
        super().__init__('zed_vio_stub')
        self.declare_parameter('use_sim_time', False)

        self._sub = self.create_subscription(
            Odometry, '/cartographer/laser_odom_at_fcu',
            self._odom_cb, 10)
        self._pub = self.create_publisher(
            Odometry, '/zed/zed_node/odom_zed_to_fcu', 10)
        self._pose_pub = self.create_publisher(
            PoseStamped, '/zed/zed_node/odom_original', 10)

        self.get_logger().info(
            'ZED VIO stub: /cartographer/laser_odom_at_fcu -> '
            '/zed/zed_node/odom_zed_to_fcu'
        )

    def _odom_cb(self, msg):
        self._pub.publish(msg)

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._pose_pub.publish(pose)


def main():
    rclpy.init()
    node = ZedVioStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
