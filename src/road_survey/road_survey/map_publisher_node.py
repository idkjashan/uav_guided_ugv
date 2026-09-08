"""map_publisher -- replay a saved survey as a latched OccupancyGrid.

Stage 2 (Nav2 + ArUco) does not need the UAV or the mapper running; it needs
the map.  This publishes the ``.npz`` a survey produced, on a transient-local
topic so Nav2's static layer picks it up whenever it starts.

    ros2 run road_survey map_publisher --ros-args \
        -p map_npz:=~/uav_guided_ugv/maps/road_map.npz -p use_sim_time:=true
"""

from __future__ import annotations

import array
import os

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from .mapio import load_npz

LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST,
                     depth=1)


def occupancy_msg(occ: np.ndarray, origin_x: float, origin_y: float,
                  resolution: float, frame_id: str, stamp) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.info.resolution = float(resolution)
    msg.info.width = int(occ.shape[1])
    msg.info.height = int(occ.shape[0])
    msg.info.origin.position.x = float(origin_x)
    msg.info.origin.position.y = float(origin_y)
    msg.info.origin.orientation.w = 1.0
    msg.data = array.array('b', occ.astype(np.int8).tobytes())
    return msg


class MapPublisher(Node):

    def __init__(self):
        super().__init__('map_publisher')
        self.declare_parameter('map_npz', '~/uav_guided_ugv/maps/road_map.npz')
        self.declare_parameter('topic', '/road/costmap')
        self.declare_parameter('frame_id', '')
        self.declare_parameter('republish_period_s', 0.0)

        path = os.path.expanduser(self.get_parameter('map_npz').value)
        self.data = load_npz(path)
        self.frame = (self.get_parameter('frame_id').value
                      or self.data['frame_id'] or 'map')
        self.pub = self.create_publisher(
            OccupancyGrid, self.get_parameter('topic').value, LATCHED)
        self.publish_once()
        period = float(self.get_parameter('republish_period_s').value)
        if period > 0.0:
            self.create_timer(period, self.publish_once)
        self.get_logger().info(
            f"published {self.data['occ'].shape[1]}x{self.data['occ'].shape[0]} "
            f"@ {self.data['resolution']} m from {path}")

    def publish_once(self):
        self.pub.publish(occupancy_msg(
            self.data['occ'], self.data['origin_x'], self.data['origin_y'],
            self.data['resolution'], self.frame,
            self.get_clock().now().to_msg()))


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
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
