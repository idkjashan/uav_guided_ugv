"""ugv_localizer -- ArUco marker on the UGV roof -> /ugv/pose in the map frame.

Subscribes to the UAV's nadir RGB camera and PX4 pose. For every frame with
the marker in it: solvePnP gives the marker in the camera, the camera
extrinsics and the PX4 pose at the frame's timestamp carry it into the map
frame (PX4 local ENU, the costmap's frame). No wheel odometry, no depth.

    ros2 run guidance ugv_localizer --ros-args --params-file config/guidance.yaml
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from road_survey.depth import intrinsics_from_hfov
from road_survey.frames import camera_extrinsics, px4_pose_to_map

from .aruco import MarkerDetector, image_to_gray, marker_in_camera, ugv_pose_in_map
from .px4 import subscribe_versioned


def nearest(buf, t):
    buf = tuple(buf)      # the PX4 thread appends while we search
    if not buf:
        return None, math.inf
    best = min(buf, key=lambda e: abs(e[0] - t))
    return best[1], abs(best[0] - t)


class UgvLocalizer(Node):

    def __init__(self):
        super().__init__('ugv_localizer')
        p = self.declare_parameters('', [
            ('image_topic', '/uav/rgb'),
            ('camera_info_topic', '/uav/camera_info'),
            ('position_topic', '/fmu/out/vehicle_local_position'),
            ('attitude_topic', '/fmu/out/vehicle_attitude'),
            ('pose_topic', '/ugv/pose'),
            ('map_frame', 'map'),
            # IMX214 on the OakD-Lite, used only until camera_info arrives
            ('image_width', 1920),
            ('image_height', 1080),
            ('image_hfov_rad', 1.57),
            ('cam_pose_in_body', [0.12, 0.03, 0.242, 0.0, 1.5708, 0.0]),
            ('marker_id', 0),
            ('marker_length_m', 0.446),   # black square, not the white plate
            ('marker_height_m', 0.127),   # marker face above the UGV base_link
            ('yaw_offset_rad', 0.0),      # marker +x -> UGV forward; calibrate once
            ('max_reproj_err_px', 2.0),
            ('max_pose_dt_s', 0.08),
            ('max_rate_hz', 15.0),
        ])
        self.p = {d.name: d.value for d in p}
        self.k = intrinsics_from_hfov(int(self.p['image_width']),
                                      int(self.p['image_height']),
                                      float(self.p['image_hfov_rad']))
        self.k_from_info = False
        self.t_cam, self.r_body_opt = camera_extrinsics(self.p['cam_pose_in_body'])
        self.detect = MarkerDetector(marker_id=int(self.p['marker_id']))
        self.pos_buf = deque(maxlen=200)
        self.att_buf = deque(maxlen=200)
        self.t_last = -1e9
        self.n = {'frames': 0, 'seen': 0, 'published': 0}

        # Three threads: images, PX4, and the default group, which holds
        # rclpy's own /clock subscription. PX4 poses are stamped on arrival
        # with a clock that keeps ticking while a frame is being decoded.
        px4_group = MutuallyExclusiveCallbackGroup()
        image_group = MutuallyExclusiveCallbackGroup()
        subscribe_versioned(self, 'VehicleLocalPosition', self.p['position_topic'],
                            self.on_position, px4_group)
        subscribe_versioned(self, 'VehicleAttitude', self.p['attitude_topic'],
                            self.on_attitude, px4_group)
        self.create_subscription(Image, self.p['image_topic'], self.on_image,
                                 qos_profile_sensor_data, callback_group=image_group)
        self.create_subscription(CameraInfo, self.p['camera_info_topic'], self.on_info,
                                 qos_profile_sensor_data, callback_group=image_group)
        self.pub = self.create_publisher(PoseStamped, self.p['pose_topic'], 10)
        self.create_timer(5.0, self.report)
        if not self.get_parameter('use_sim_time').value:
            self.get_logger().warn('use_sim_time is false; image and PX4 times will not match')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_position(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.pos_buf.append((self._now(), np.array([msg.x, msg.y, msg.z])))

    def on_attitude(self, msg):
        self.att_buf.append((self._now(), np.asarray(msg.q, dtype=float)))

    def on_info(self, msg):
        if not self.k_from_info and msg.k[0] > 0.0:
            self.k = np.asarray(msg.k, dtype=float).reshape(3, 3)
            self.k_from_info = True
            self.get_logger().info(f'intrinsics from camera_info: fx={self.k[0, 0]:.1f}')

    def on_image(self, msg: Image):
        self.n['frames'] += 1
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t - self.t_last < 1.0 / float(self.p['max_rate_hz']):
            return
        self.t_last = t
        try:
            corners = self.detect(image_to_gray(msg))
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return
        if corners is None:
            return
        self.n['seen'] += 1
        sol = marker_in_camera(corners, self.k, float(self.p['marker_length_m']))
        if sol is None or sol[2] > float(self.p['max_reproj_err_px']):
            return
        pos, dt_p = nearest(self.pos_buf, t)
        q, dt_a = nearest(self.att_buf, t)
        if pos is None or q is None or max(dt_p, dt_a) > float(self.p['max_pose_dt_s']):
            self.get_logger().warn('no PX4 pose near the image time; skipping',
                                   throttle_duration_sec=5.0)
            return
        p_map, r_map_body = px4_pose_to_map(pos, q)
        x, y, z, yaw = ugv_pose_in_map(sol[0], sol[1], self.r_body_opt, self.t_cam,
                                       r_map_body, p_map,
                                       float(self.p['yaw_offset_rad']),
                                       float(self.p['marker_height_m']))
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.p['map_frame']
        out.pose.position.x, out.pose.position.y, out.pose.position.z = x, y, z
        out.pose.orientation.z = math.sin(yaw / 2.0)
        out.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub.publish(out)
        self.n['published'] += 1

    def report(self):
        self.get_logger().info(
            f"frames {self.n['frames']}, marker seen {self.n['seen']}, "
            f"poses published {self.n['published']} (last 5 s)")
        self.n = dict.fromkeys(self.n, 0)


def main(args=None):
    rclpy.init(args=args)
    node = UgvLocalizer()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
