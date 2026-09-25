"""pose_check -- how far is the ArUco UGV pose from Gazebo's ground truth?

Validation only; nothing in the mission uses it. Compares /ugv/pose (map
frame) with /ugv/ground_truth (gz world frame, bridged by the rover's
spawn launch) through the world->map transform sim.launch.py publishes,
and prints the error every 5 s. The mean heading error is the correction
for yaw_offset_rad in guidance.yaml.

    ros2 run guidance pose_check --ros-args -p use_sim_time:=true
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class PoseCheck(Node):

    def __init__(self):
        super().__init__('pose_check')
        self.truth = deque(maxlen=100)     # (t, x, y, yaw) in world
        self.err = []                      # (dxy, dyaw)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(Odometry, '/ugv/ground_truth', self.on_truth, 10)
        self.create_subscription(PoseStamped, '/ugv/pose', self.on_pose, 10)
        self.create_timer(5.0, self.report)

    def on_truth(self, msg):
        p = msg.pose.pose
        self.truth.append((stamp(msg), p.position.x, p.position.y, yaw_of(p.orientation)))

    def on_pose(self, msg):
        if not self.truth:
            return
        t = stamp(msg)
        gt = min(self.truth, key=lambda e: abs(e[0] - t))
        if abs(gt[0] - t) > 0.05:
            return
        try:
            # map is world shifted by the UAV spawn point, axes unchanged
            off = self.tf.lookup_transform('world', 'map', Time()).transform.translation
        except Exception:  # noqa: BLE001 - tf not up yet
            self.get_logger().warn('no world->map transform yet', throttle_duration_sec=5.0)
            return
        dx = msg.pose.position.x + off.x - gt[1]
        dy = msg.pose.position.y + off.y - gt[2]
        dyaw = yaw_of(msg.pose.orientation) - gt[3]
        self.err.append((math.hypot(dx, dy), math.atan2(math.sin(dyaw), math.cos(dyaw))))

    def report(self):
        if not self.err:
            self.get_logger().info('no matched poses in the last 5 s')
            return
        e = np.array(self.err)
        yaw = math.atan2(np.sin(e[:, 1]).mean(), np.cos(e[:, 1]).mean())
        self.get_logger().info(
            f'{len(e)} poses: xy error median {np.median(e[:, 0]):.3f} m, '
            f'p95 {np.percentile(e[:, 0], 95):.3f} m, max {e[:, 0].max():.3f} m; '
            f'mean heading error {math.degrees(yaw):+.1f} deg '
            f'(subtract {yaw:+.3f} rad from yaw_offset_rad)')
        self.err = []


def main(args=None):
    rclpy.init(args=args)
    node = PoseCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
