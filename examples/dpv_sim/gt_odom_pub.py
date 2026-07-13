#!/usr/bin/env python3
"""Publish ground-truth odometry from Isaac Sim state for rviz/debug.

Subscribes to Isaac's sim-state pose and inertial twist, republishes as
nav_msgs/Odometry on /sim/gt_odom (frame map -> v1_0_base_link).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry


class GtOdomPublisher(Node):
    def __init__(self):
        super().__init__('gt_odom_publisher')
        self.declare_parameter('use_sim_time', False)
        self._pose_sub = self.create_subscription(
            PoseStamped, '/v1_0/state/pose', self._pose_cb, 10)
        self._twist_sub = self.create_subscription(
            TwistStamped, '/v1_0/state/twist_inertial', self._twist_cb, 10)
        self._odom_pub = self.create_publisher(Odometry, '/sim/gt_odom', 10)
        self._latest_pose = None
        self._latest_twist = None

    def _pose_cb(self, msg):
        self._latest_pose = msg
        self._try_publish()

    def _twist_cb(self, msg):
        self._latest_twist = msg
        self._try_publish()

    def _try_publish(self):
        if self._latest_pose is None or self._latest_twist is None:
            return
        odom = Odometry()
        odom.header.stamp = self._latest_pose.header.stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'v1_0_base_link'
        odom.pose.pose = self._latest_pose.pose
        odom.twist.twist = self._latest_twist.twist
        self._odom_pub.publish(odom)


def main():
    rclpy.init()
    node = GtOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
