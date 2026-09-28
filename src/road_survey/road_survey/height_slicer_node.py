"""height_slicer -- make the flat costmap honest on a switchback.

A single 2-D grid cannot say "there is road here at 40 m *and* road here at
55 m".  On a spiral hill road the upper and lower legs can project onto the
same XY cells, and a 2-D planner will happily cut between them through thin
air.

This node keeps the map 2-D (so plain Nav2 still works) but makes it a *slice*:
cells whose surveyed elevation is more than ``band_m`` away from the UGV's
current height are marked lethal.  As the rover climbs, the slice follows it,
and the leg of the road it is not on disappears.  Run it only if the mapper
reports ``multi_level_cells > 0``.

    ros2 run road_survey height_slicer --ros-args \
        -p map_npz:=~/uav_guided_ugv/maps/road_map.npz \
        -p pose_topic:=/ugv/pose -p use_sim_time:=true
"""

from __future__ import annotations

import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

from .map_publisher_node import LATCHED, occupancy_msg
from .mapio import load_npz


class HeightSlicer(Node):

    def __init__(self):
        super().__init__('height_slicer')
        self.declare_parameter('map_npz', '~/uav_guided_ugv/maps/road_map.npz')
        self.declare_parameter('pose_topic', '/ugv/pose')
        self.declare_parameter('out_topic', '/road/costmap')
        self.declare_parameter('band_m', 3.0)
        self.declare_parameter('period_s', 1.0)
        # Below this many multi-level cells the slice is pointless overhead.
        self.declare_parameter('min_multi_level_cells', 1)

        path = os.path.expanduser(self.get_parameter('map_npz').value)
        if not os.path.exists(path):
            cwd_cand = os.path.join(os.getcwd(), 'maps', os.path.basename(path))
            if os.path.exists(cwd_cand):
                path = cwd_cand
        self.data = load_npz(path)
        self.band = float(self.get_parameter('band_m').value)
        self.frame = self.data['frame_id'] or 'map'
        self.elev = self.data['elevation']
        self.occ = self.data['occ']
        self.z = None

        self.pub = self.create_publisher(
            OccupancyGrid, self.get_parameter('out_topic').value, LATCHED)
        self.create_subscription(PoseStamped,
                                 self.get_parameter('pose_topic').value,
                                 self.on_pose, 10)
        self.create_timer(float(self.get_parameter('period_s').value), self.tick)
        self.get_logger().info(
            f'slicing {path} to +/-{self.band} m around the UGV height')

    def on_pose(self, msg: PoseStamped):
        self.z = float(msg.pose.position.z)

    def tick(self):
        occ = self.occ
        if self.z is not None:
            far = np.isfinite(self.elev) & (np.abs(self.elev - self.z) > self.band)
            occ = np.where(far, np.int8(100), self.occ).astype(np.int8)
        self.pub.publish(occupancy_msg(
            occ, self.data['origin_x'], self.data['origin_y'],
            self.data['resolution'], self.frame,
            self.get_clock().now().to_msg()))


def main(args=None):
    rclpy.init(args=args)
    node = HeightSlicer()
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
