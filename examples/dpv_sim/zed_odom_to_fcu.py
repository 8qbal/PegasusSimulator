#!/usr/bin/env python3
"""Phase-4 glue: real zed_wrapper VIO -> the DPV EV input topic.

Successor of zed_vio_stub.py (which faked ZED VIO from cartographer laser odom).
With the Stereolabs zed-isaac-sim streamer running (Isaac side) and the real
zed_wrapper in sim_mode (tmux window "zed"), this node adapts the wrapper's
odometry to the exact topic pair the real drone's px4_ros2_bridge consumes
(bridge_params.yaml to_fcu.input_topics — no overrides needed in phase 4):

  /zed/zed_node/odom  (nav_msgs/Odometry, frame odom_zed, ENU/FLU)
    -> /zed/zed_node/odom_zed_to_fcu  (nav_msgs/Odometry)
    -> /zed/zed_node/odom_original    (geometry_msgs/PoseStamped)

Pose/twist/covariance pass through untouched — the camera->FCU lever arm is
applied by the real to_fcu_vehicle_visual_odometry node via its lever_arm
params (overridden to the sim mount x=0.355 in isaac_nav_bringup_zed.launch.py),
exactly like on the drone.

Timestamps: the wrapper stamps frames from the ZED SDK grab clock, which in
sim mode may not be the wall clock the DPV stack runs on (use_sim_time:=False
everywhere). The node logs the (now - msg.stamp) offset every few seconds —
use it to sanity-check EKF2_EV_DELAY. If the offset is large/drifting, set
`restamp:=true` to overwrite stamps with this node's clock at arrival (loses
true latency, but keeps EKF2's delay bookkeeping bounded).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# Matches both RELIABLE and BEST_EFFORT publishers (wrapper QoS varies by config).
PERMISSIVE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

LOG_PERIOD_S = 5.0


class ZedOdomToFcu(Node):
    def __init__(self):
        super().__init__('zed_odom_to_fcu')
        self.declare_parameter('input_topic', '/zed/zed_node/odom')
        self.declare_parameter('restamp', False)

        input_topic = self.get_parameter('input_topic').value
        self._restamp = bool(self.get_parameter('restamp').value)

        self._sub = self.create_subscription(
            Odometry, input_topic, self._odom_cb, PERMISSIVE_QOS)
        self._pub = self.create_publisher(
            Odometry, '/zed/zed_node/odom_zed_to_fcu', 10)
        self._pose_pub = self.create_publisher(
            PoseStamped, '/zed/zed_node/odom_original', 10)

        self._n = 0
        self._last_log = self.get_clock().now()

        self.get_logger().info(
            f'ZED VIO glue: {input_topic} -> /zed/zed_node/odom_zed_to_fcu '
            f'(restamp={self._restamp})')

    def _odom_cb(self, msg):
        now = self.get_clock().now()
        stamp_offset_s = (now.nanoseconds
                          - (msg.header.stamp.sec * 1_000_000_000
                             + msg.header.stamp.nanosec)) * 1e-9

        if self._restamp:
            msg.header.stamp = now.to_msg()

        self._pub.publish(msg)

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._pose_pub.publish(pose)

        self._n += 1
        if (now - self._last_log).nanoseconds * 1e-9 >= LOG_PERIOD_S:
            self._last_log = now
            p = msg.pose.pose.position
            self.get_logger().info(
                f'{self._n} msgs | pos=({p.x:+.2f},{p.y:+.2f},{p.z:+.2f}) '
                f'frame={msg.header.frame_id} | stamp offset now-msg='
                f'{stamp_offset_s:+.3f}s (feed into EKF2_EV_DELAY check)')


def main():
    rclpy.init()
    node = ZedOdomToFcu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
