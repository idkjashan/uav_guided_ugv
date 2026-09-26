"""pose_check -- how good is the run, measured against Gazebo's ground truth?

Validation only; nothing in the mission uses it. Every 5 s it prints:

* the ArUco pose error: /ugv/pose (map frame) against /ugv/ground_truth
  (gz world frame) through the world->map transform sim.launch.py
  publishes. The mean heading error is the correction for yaw_offset_rad.
* the centre-line deviation so far: how far the *true* UGV position is
  from the centre line it was told to drive (/ugv/path), counted only
  while it is moving. This is the number the rubric scores; the
  follower's own /ugv/cross_track_error uses the estimated pose instead.

    ros2 run guidance pose_check --ros-args -p use_sim_time:=true
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from .pursuit import Path

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class PoseCheck(Node):

    def __init__(self):
        super().__init__('pose_check')
        self.truth = deque(maxlen=100)     # (t, x, y, yaw) in world
        self.err = []                      # (dxy, dyaw)
        self.path = None
        self.s = None                      # progress along the path, for the windowed search
        self.dev = []                      # true distance from the centre line, whole run
        self.driven = 0.0
        self.last_xy = None
        self.off = None
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(Odometry, '/ugv/ground_truth', self.on_truth, 10)
        self.create_subscription(PoseStamped, '/ugv/pose', self.on_pose, 10)
        self.create_subscription(PathMsg, '/ugv/path', self.on_path, LATCHED)
        self.create_timer(5.0, self.report)

    def offset(self):
        """map origin in the world frame (the UAV spawn point), or None."""
        if self.off is None:
            try:
                self.off = self.tf.lookup_transform('world', 'map', Time()).transform.translation
            except Exception:  # noqa: BLE001 - tf not up yet
                self.get_logger().warn('no world->map transform yet', throttle_duration_sec=5.0)
        return self.off

    def on_path(self, msg):
        xy = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        if len(xy) >= 2:
            self.path, self.s, self.dev, self.driven = Path(xy), None, [], 0.0

    def on_truth(self, msg):
        p = msg.pose.pose
        self.truth.append((stamp(msg), p.position.x, p.position.y, yaw_of(p.orientation)))
        v = msg.twist.twist.linear
        off = self.offset()
        if self.path is None or off is None or math.hypot(v.x, v.y) < 0.05:
            self.last_xy = None
            return
        xy = np.array([p.position.x - off.x, p.position.y - off.y])
        self.s, d = self.path.project(xy, self.s, 6.0)
        self.dev.append(d)
        if self.last_xy is not None:
            self.driven += float(np.hypot(*(xy - self.last_xy)))
        self.last_xy = xy

    def on_pose(self, msg):
        if not self.truth:
            return
        t = stamp(msg)
        gt = min(self.truth, key=lambda e: abs(e[0] - t))
        if abs(gt[0] - t) > 0.05:
            return
        off = self.offset()          # map is world shifted by the UAV spawn, axes unchanged
        if off is None:
            return
        dx = msg.pose.position.x + off.x - gt[1]
        dy = msg.pose.position.y + off.y - gt[2]
        dyaw = yaw_of(msg.pose.orientation) - gt[3]
        self.err.append((math.hypot(dx, dy), math.atan2(math.sin(dyaw), math.cos(dyaw))))

    def report(self):
        if self.dev:
            d = np.array(self.dev)
            self.get_logger().info(
                f'centre-line deviation over {self.driven:.1f} m driven: median '
                f'{np.median(d):.3f} m, p95 {np.percentile(d, 95):.3f} m, max {d.max():.3f} m')
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
