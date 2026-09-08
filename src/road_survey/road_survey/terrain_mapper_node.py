"""terrain_mapper -- the stage-1 survey node.

Subscribes to the UAV's downward depth camera and PX4 local pose, accumulates
every depth frame into a 2.5-D elevation grid in the map (ENU) frame,
classifies road vs. not-road geometrically, and publishes the result as a
nav_msgs/OccupancyGrid that Nav2's static layer can consume directly.

You fly the survey manually.  This node never commands the UAV.

    ros2 run road_survey terrain_mapper --ros-args \
        --params-file src/road_survey/config/road_survey.yaml

    ros2 service call /terrain_mapper/save std_srvs/srv/Trigger
"""

from __future__ import annotations

import array
import json
import math
import os
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from . import costmap as cm
from . import mapio
from . import risk as rk
from .depth import Unprojector, decode_image, intrinsics_from_hfov, k_from_camera_info
from .frames import camera_extrinsics, optical_to_map, px4_pose_to_map, yaw_from_rot
from .grid import TerrainGrid, known_bbox

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST,
                     depth=1)

LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST,
                     depth=1)


class TerrainMapper(Node):

    def __init__(self):
        super().__init__('terrain_mapper')
        p = self.declare_parameters('', [
            ('depth_topic', '/uav/depth'),
            ('camera_info_topic', '/uav/depth_camera_info'),
            ('position_topic', '/fmu/out/vehicle_local_position'),
            ('attitude_topic', '/fmu/out/vehicle_attitude'),
            ('costmap_topic', '/road/costmap'),
            ('map_frame', 'map'),

            # Depth camera fallback intrinsics (StereoOV7251 on the OakD-Lite).
            ('depth_width', 640),
            ('depth_height', 480),
            ('depth_hfov_rad', 1.274),
            # The SDF <pose> of camera_link in base_link: x y z r p y.
            ('cam_pose_in_body', [0.12, 0.03, 0.242, 0.0, 1.5708, 0.0]),
            # Stay inside the sensor's <clip>: near 0.2, far 19.1.
            ('depth_min_m', 0.4),
            ('depth_max_m', 18.0),
            ('stride', 2),

            ('resolution', 0.25),
            ('map_size_m', 400.0),
            ('auto_center', True),
            ('center_x', 0.0),
            ('center_y', 0.0),
            ('track_extrema', True),
            ('multi_level_gap_m', 2.0),

            ('max_integrate_hz', 5.0),
            ('min_translation_m', 0.25),
            ('min_rotation_deg', 3.0),
            ('max_pose_dt_s', 0.08),
            ('max_tilt_deg', 25.0),
            ('min_count', 3),

            ('classify_period_s', 2.0),
            ('publish_elevation_image', False),
            ('seed_at_start_pose', True),
            ('output_dir', '~/uav_guided_ugv/maps'),
            ('output_name', 'road_map'),
            ('save_centerline', True),
            ('autosave_on_shutdown', True),

            # risk.RiskParams
            ('window_cells', 5),
            ('smooth_cells', 3),
            ('slope_crit_deg', 15.0),
            ('step_crit_m', 0.25),
            ('rough_crit_m', 0.12),
            ('risk_thresh', 0.35),
            # costmap.CostmapParams
            ('open_radius_m', 0.4),
            ('close_radius_m', 0.8),
            ('min_area_m2', 25.0),
            ('keep_components', 1),
            ('edge_cost_ref_m', 1.5),
            ('unknown_is_lethal', True),
        ])
        self.p = {d.name: d.value for d in p}

        self.risk_params = rk.RiskParams(
            window_cells=int(self.p['window_cells']),
            smooth_cells=int(self.p['smooth_cells']),
            slope_crit_deg=float(self.p['slope_crit_deg']),
            step_crit_m=float(self.p['step_crit_m']),
            rough_crit_m=float(self.p['rough_crit_m']),
            risk_thresh=float(self.p['risk_thresh']),
            min_count=int(self.p['min_count']))
        self.cost_params = cm.CostmapParams(
            open_radius_m=float(self.p['open_radius_m']),
            close_radius_m=float(self.p['close_radius_m']),
            min_area_m2=float(self.p['min_area_m2']),
            keep_components=int(self.p['keep_components']),
            edge_cost_ref_m=float(self.p['edge_cost_ref_m']),
            unknown_is_lethal=bool(self.p['unknown_is_lethal']))

        self.t_cam, self.r_body_opt = camera_extrinsics(self.p['cam_pose_in_body'])
        self.k = intrinsics_from_hfov(int(self.p['depth_width']),
                                      int(self.p['depth_height']),
                                      float(self.p['depth_hfov_rad']))
        self.k_from_info = False
        self.unproj = None

        self.grid = None
        self.seed_cell = None
        self.pos_buf = deque(maxlen=200)   # (t, np.array NED xyz)
        self.att_buf = deque(maxlen=200)   # (t, np.array wxyz)
        self.last_integrated = None        # (pos_map, yaw)
        self.n_frames = 0
        self.n_used = 0
        self.n_points = 0
        self.t_last_frame = -1e9
        self.last_stats = {}

        self.create_subscription(Image, self.p['depth_topic'],
                                 self.on_depth, qos_profile_sensor_data)
        if self.p['camera_info_topic']:
            self.create_subscription(CameraInfo, self.p['camera_info_topic'],
                                     self.on_info, qos_profile_sensor_data)
        self._subscribe_px4()

        self.pub_map = self.create_publisher(OccupancyGrid,
                                             self.p['costmap_topic'], LATCHED)
        self.pub_stats = self.create_publisher(String, '/road/stats', 10)
        self.pub_elev = None
        if self.p['publish_elevation_image']:
            self.pub_elev = self.create_publisher(Image, '/terrain/elevation', 1)

        self.create_service(Trigger, '~/save', self.on_save)
        self.create_service(Trigger, '~/reset', self.on_reset)
        self.create_timer(float(self.p['classify_period_s']), self.on_classify)

        if not self.get_parameter('use_sim_time').value:
            self.get_logger().warn(
                'use_sim_time is false. Run with -p use_sim_time:=true or the '
                'depth stamps and PX4 poses will not line up.')
        self.get_logger().info(
            f"waiting for depth on {self.p['depth_topic']} and pose on "
            f"{self.p['position_topic']}")

    # -- PX4 plumbing ---------------------------------------------------------

    def _subscribe_px4(self):
        try:
            from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
        except ImportError as exc:  # pragma: no cover - environment issue
            raise SystemExit(
                'px4_msgs not found. source ~/px4_ros_ws/install/setup.bash '
                'before running this node.') from exc
        self.create_subscription(VehicleLocalPosition, self.p['position_topic'],
                                 self.on_position, PX4_QOS)
        self.create_subscription(VehicleAttitude, self.p['attitude_topic'],
                                 self.on_attitude, PX4_QOS)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_position(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.pos_buf.append((self._now(), np.array([msg.x, msg.y, msg.z])))

    def on_attitude(self, msg):
        self.att_buf.append((self._now(), np.asarray(msg.q, dtype=float)))

    def on_info(self, msg):
        if not self.k_from_info:
            self.k = k_from_camera_info(msg)
            self.k_from_info = True
            self.unproj = None
            self.get_logger().info(
                f'intrinsics from camera_info: fx={self.k[0, 0]:.1f} '
                f'cx={self.k[0, 2]:.1f} cy={self.k[1, 2]:.1f}')

    @staticmethod
    def _nearest(buf, t):
        if not buf:
            return None, math.inf
        best = min(buf, key=lambda e: abs(e[0] - t))
        return best[1], abs(best[0] - t)

    # -- the hot path ---------------------------------------------------------

    def on_depth(self, msg: Image):
        self.n_frames += 1
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        min_dt = 1.0 / max(float(self.p['max_integrate_hz']), 1e-3)
        if t - self.t_last_frame < min_dt:
            return

        pos_ned, dt_p = self._nearest(self.pos_buf, t)
        quat, dt_a = self._nearest(self.att_buf, t)
        if pos_ned is None or quat is None:
            return
        if max(dt_p, dt_a) > float(self.p['max_pose_dt_s']):
            self._throttle_warn(f'pose is {max(dt_p, dt_a) * 1e3:.0f} ms from '
                                'the depth frame; skipping')
            return

        p_map, r_map_body = px4_pose_to_map(pos_ned, quat)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, r_map_body[2, 2]))))
        if tilt > float(self.p['max_tilt_deg']):
            return

        if self.grid is None:
            self._init_grid(p_map)
        yaw = yaw_from_rot(r_map_body)
        if self.last_integrated is not None:
            dp = float(np.linalg.norm(p_map[:2] - self.last_integrated[0][:2]))
            dy = abs(math.atan2(math.sin(yaw - self.last_integrated[1]),
                                math.cos(yaw - self.last_integrated[1])))
            if (dp < float(self.p['min_translation_m'])
                    and math.degrees(dy) < float(self.p['min_rotation_deg'])):
                return

        try:
            depth = decode_image(msg)
        except ValueError as exc:
            self._throttle_warn(str(exc))
            return
        if (self.unproj is None
                or not self.unproj.matches(self.k, depth.shape, self.p['stride'])):
            self.unproj = Unprojector(self.k, depth.shape, int(self.p['stride']))
        pts_opt = self.unproj(depth, float(self.p['depth_min_m']),
                              float(self.p['depth_max_m']))
        if pts_opt.shape[0] == 0:
            self._throttle_warn('depth frame had no valid returns -- too high? '
                                'the sensor far clip is 19.1 m')
            return

        pts_map = optical_to_map(pts_opt.astype(np.float64), self.r_body_opt,
                                 self.t_cam, r_map_body, p_map)
        self.n_points += self.grid.add_points(pts_map)
        self.n_used += 1
        self.t_last_frame = t
        self.last_integrated = (p_map, yaw)

    def _init_grid(self, p_map):
        if self.p['auto_center']:
            cx, cy = float(p_map[0]), float(p_map[1])
        else:
            cx, cy = float(self.p['center_x']), float(self.p['center_y'])
        self.grid = TerrainGrid.centered_on(cx, cy, float(self.p['map_size_m']),
                                            float(self.p['resolution']),
                                            bool(self.p['track_extrema']))
        if self.p['seed_at_start_pose']:
            self.seed_cell = self.grid.world_to_cell(float(p_map[0]), float(p_map[1]))
        self.get_logger().info(
            f'grid {self.grid.width}x{self.grid.height} @ '
            f'{self.grid.res} m, origin ({self.grid.origin_x:.1f}, '
            f'{self.grid.origin_y:.1f}) in {self.p["map_frame"]}')

    def _throttle_warn(self, text):
        self.get_logger().warn(text, throttle_duration_sec=5.0)

    # -- classification -------------------------------------------------------

    def _classify(self):
        """-> dict of full-size layers plus the crop box, or None."""
        if self.grid is None:
            return None
        known = self.grid.known(self.risk_params.min_count)
        box = known_bbox(known, margin=max(8, 2 * int(self.p['window_cells'])))
        if box is None:
            return None
        r0, r1, c0, c1 = box
        elev = self.grid.elevation(self.risk_params.min_count)[r0:r1, c0:c1]
        sub_known = known[r0:r1, c0:c1]
        res = self.grid.res

        risk, road, valid, _ = rk.classify(elev, sub_known, res, self.risk_params)
        seed = None
        if self.seed_cell is not None:
            seed = (self.seed_cell[0] - c0, self.seed_cell[1] - r0)
        road, info = cm.clean_mask(road, res, self.cost_params, seed)
        occ = cm.to_occupancy(road, valid, res, self.cost_params)

        extent = self.grid.vertical_extent(self.risk_params.min_count)[r0:r1, c0:c1]
        n_multi = int(np.count_nonzero(
            rk.multi_level_cells(extent, road, float(self.p['multi_level_gap_m']))))
        info.update({
            'frames_seen': self.n_frames,
            'frames_used': self.n_used,
            'points': self.n_points,
            'points_outside_grid': self.grid.dropped,
            'surveyed_m2': round(self.grid.coverage_m2(self.risk_params.min_count), 1),
            'multi_level_cells': n_multi,
        })
        return {'occ': occ, 'road': road, 'risk': risk, 'valid': valid,
                'elev': elev, 'extent': extent, 'box': box, 'info': info}

    def on_classify(self):
        out = self._classify()
        if out is None:
            return
        self.last_stats = out['info']
        self.pub_map.publish(self._to_grid_msg(out['occ'], out['box']))
        self.pub_stats.publish(String(data=json.dumps(out['info'])))
        if out['info']['multi_level_cells'] > 0:
            self._throttle_warn(
                f"{out['info']['multi_level_cells']} road cells stack two levels "
                '>%.1f m apart -- run height_slicer for the Nav2 stage'
                % float(self.p['multi_level_gap_m']))
        if self.pub_elev is not None:
            self.pub_elev.publish(self._to_image_msg(out['elev']))

    def _to_grid_msg(self, occ: np.ndarray, box) -> OccupancyGrid:
        r0, _, c0, _ = box
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p['map_frame']
        msg.info.resolution = self.grid.res
        msg.info.width = int(occ.shape[1])
        msg.info.height = int(occ.shape[0])
        msg.info.origin.position.x = self.grid.origin_x + c0 * self.grid.res
        msg.info.origin.position.y = self.grid.origin_y + r0 * self.grid.res
        msg.info.origin.orientation.w = 1.0
        msg.data = array.array('b', occ.astype(np.int8).tobytes())
        return msg

    def _to_image_msg(self, elev: np.ndarray) -> Image:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p['map_frame']
        msg.height, msg.width = elev.shape
        msg.encoding = '32FC1'
        msg.is_bigendian = 0
        msg.step = msg.width * 4
        msg.data = elev.astype(np.float32).tobytes()
        return msg

    # -- services -------------------------------------------------------------

    def on_save(self, _req, resp):
        try:
            paths = self.save()
        except Exception as exc:  # noqa: BLE001 - a failed save must not kill the survey
            resp.success = False
            resp.message = f'save failed: {exc}'
            self.get_logger().error(resp.message)
            return resp
        resp.success = True
        resp.message = ' '.join(paths)
        self.get_logger().info(f'saved: {resp.message}')
        return resp

    def on_reset(self, _req, resp):
        self.grid = None
        self.last_integrated = None
        self.n_frames = self.n_used = self.n_points = 0
        resp.success = True
        resp.message = 'grid cleared'
        return resp

    def save(self):
        out = self._classify()
        if out is None:
            raise RuntimeError('nothing surveyed yet')
        r0, _, c0, _ = out['box']
        ox = self.grid.origin_x + c0 * self.grid.res
        oy = self.grid.origin_y + r0 * self.grid.res
        d = os.path.expanduser(str(self.p['output_dir']))
        os.makedirs(d, exist_ok=True)
        base = os.path.join(d, str(self.p['output_name']))

        cl_rc = None
        if self.p['save_centerline']:
            try:
                from .centerline import longest_skeleton_path  # noqa: PLC0415
                cl_rc = longest_skeleton_path(out['road'])
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'centre line skipped: {exc}')

        params = {'risk': self.risk_params.as_dict(),
                  'costmap': self.cost_params.as_dict(),
                  'stats': out['info']}
        r0_, r1_, c0_, c1_ = out['box']
        paths = [mapio.save_npz(
            base + '.npz', occ=out['occ'], elevation=out['elev'],
            risk=out['risk'], road=out['road'], valid=out['valid'],
            count=self.grid.count[r0_:r1_, c0_:c1_], extent=out['extent'],
            origin_x=ox, origin_y=oy, resolution=self.grid.res,
            frame_id=str(self.p['map_frame']), params=params)]
        paths.append(mapio.save_map_server(base, out['occ'], self.grid.res, ox, oy))
        paths.append(mapio.save_debug_png(
            base + '.png', elevation=out['elev'], road=out['road'],
            risk=out['risk'], resolution=self.grid.res, centerline_rc=cl_rc))
        return paths


def main(args=None):
    rclpy.init(args=args)
    node = TerrainMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.p['autosave_on_shutdown'] and node.grid is not None:
            try:
                node.get_logger().info(f'autosaved: {node.save()}')
            except Exception as exc:  # noqa: BLE001
                node.get_logger().warn(f'autosave failed: {exc}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
