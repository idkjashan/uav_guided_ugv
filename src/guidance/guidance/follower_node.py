"""ugv_follower -- drive the UGV along the road centre line with pure pursuit.

Waits for the mission to reach TRACK (the UAV is overhead and sees the
marker), builds the centre line from the latest road costmap once, then
publishes /cmd_vel from /ugv/pose. The UGV only moves while it is being
seen: a pose older than ``pose_timeout_s`` stops it, and so does any mission
state other than TRACK. The gz DiffDrive plugin has no command timeout, so
the stop is an explicit zero Twist, not silence.

    ros2 run guidance ugv_follower --ros-args --params-file config/guidance.yaml
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32, String

from .pursuit import PursuitParams, centerline_path, command

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class UgvFollower(Node):

    def __init__(self):
        super().__init__('ugv_follower')
        defaults = PursuitParams()
        p = self.declare_parameters('', [
            ('pose_topic', '/ugv/pose'),
            ('costmap_topic', '/road/costmap'),
            ('state_topic', '/mission/state'),
            ('cmd_vel_topic', '/cmd_vel'),
            ('pose_timeout_s', 0.5),
            ('rate_hz', 20.0),
            ('path_spacing_m', 0.5),
        ] + [(f, getattr(defaults, f)) for f in defaults.__dataclass_fields__])
        self.p = {d.name: d.value for d in p}
        self.pp = PursuitParams(**{f: float(self.p[f]) for f in defaults.__dataclass_fields__})

        self.pose = None          # (t, x, y, yaw)
        self.costmap = None
        self.state = ''
        self.path = None
        self.s = None
        self.done = False
        self.moving = True        # so the first tick sends a stop

        self.create_subscription(PoseStamped, self.p['pose_topic'], self.on_pose, 10)
        self.create_subscription(OccupancyGrid, self.p['costmap_topic'], self.on_costmap, LATCHED)
        self.create_subscription(String, self.p['state_topic'], self.on_state, LATCHED)
        self.pub_cmd = self.create_publisher(Twist, self.p['cmd_vel_topic'], 10)
        self.pub_path = self.create_publisher(PathMsg, '/ugv/path', LATCHED)
        self.pub_cte = self.create_publisher(Float32, '/ugv/cross_track_error', 10)
        self.create_timer(1.0 / float(self.p['rate_hz']), self.tick)

    def on_pose(self, msg: PoseStamped):
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pose = (t, msg.pose.position.x, msg.pose.position.y, yaw)

    def on_costmap(self, msg: OccupancyGrid):
        occ = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.costmap = (occ, msg.info.origin.position.x, msg.info.origin.position.y,
                        msg.info.resolution, msg.header.frame_id)

    def on_state(self, msg: String):
        if msg.data != self.state:
            self.get_logger().info(f'mission state {msg.data}')
        self.state = msg.data

    def stop(self):
        if self.moving:
            self.pub_cmd.publish(Twist())
            self.moving = False

    def build_path(self):
        occ, ox, oy, res, frame = self.costmap
        path = centerline_path(occ, ox, oy, res, self.pose[1:3], float(self.p['path_spacing_m']))
        if path is None:
            self.get_logger().error('no centre line in the costmap; is the road mapped?',
                                    throttle_duration_sec=5.0)
            return
        self.path = path
        msg = PathMsg()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in path.xy:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)
        end = path.xy[-1]
        self.get_logger().info(f'centre line {path.length:.1f} m, goal ({end[0]:.1f}, {end[1]:.1f})')

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        fresh = self.pose is not None and now - self.pose[0] < float(self.p['pose_timeout_s'])
        if self.state != 'TRACK' or not fresh or self.done:
            self.stop()
            return
        if self.path is None:
            if self.costmap is None:
                self.get_logger().warn('waiting for the road costmap', throttle_duration_sec=5.0)
                return
            self.build_path()
            if self.path is None:
                return
        _, x, y, yaw = self.pose
        cmd = command(x, y, yaw, self.path, self.s, self.pp)
        self.s = cmd.s
        self.pub_cte.publish(Float32(data=float(cmd.cross_track)))
        if cmd.done:
            self.done = True
            self.stop()
            self.get_logger().info('goal reached: end of the road')
            return
        out = Twist()
        out.linear.x = float(cmd.v)
        out.angular.z = float(cmd.w)
        self.pub_cmd.publish(out)
        self.moving = True


def main(args=None):
    # rclpy's own SIGINT handler shuts the context down before `finally`
    # runs, and the stop below would then fail. With Python's handler the
    # KeyboardInterrupt arrives while the publisher still works.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = UgvFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())     # DiffDrive would keep the last command forever
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
