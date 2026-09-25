"""mission -- PX4 offboard wrapper around guidance.mission.Mission.

Handles the PX4 side: stream setpoints, switch to offboard, arm, then hand
every tick to the mission logic and publish the setpoint it returns. Feeds the
logic the three facts it needs: ground height under the UAV (median of the
depth image centre), the latest road costmap, and the UGV pose.

It never force-arms. If PX4 refuses to arm, the preflight checks are failing
and flying anyway is how a UAV ends up toilet-bowling; run `commander check`
in the PX4 shell to see why.

If PX4 leaves offboard mode after the mission started (RC takeover, failsafe,
QGC), the node stops publishing setpoints and reports ABORTED, which also
stops the UGV. Restart the node to fly again.

    ros2 run guidance mission --ros-args --params-file config/guidance.yaml
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from road_survey.depth import decode_image

from .explore import ExploreParams
from .mission import Mission, MissionParams
from .px4 import PX4_QOS, px4_msgs, subscribe_versioned

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
RATE_HZ = 20.0


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission')
        mp, ep = MissionParams(), ExploreParams()
        names = [f for f in mp.__dataclass_fields__ if f != 'explore']
        p = self.declare_parameters('', [(f, getattr(mp, f)) for f in names] + [
            ('lookahead_m', ep.lookahead_m),
            ('max_turn_deg', ep.max_turn_deg),
            ('ugv_timeout_s', 0.5),
            ('depth_topic', '/uav/depth'),
            ('costmap_topic', '/road/costmap'),
            ('ugv_pose_topic', '/ugv/pose'),
            ('save_service', '/terrain_mapper/save'),
            ('cam_height_in_body_m', 0.242),
        ])
        self.p = {d.name: d.value for d in p}
        self.params = MissionParams(
            **{f: type(getattr(mp, f))(self.p[f]) for f in names},
            explore=ExploreParams(lookahead_m=float(self.p['lookahead_m']),
                                  max_turn_deg=float(self.p['max_turn_deg'])))

        self.status = None
        self.lpos = None
        self.ground = None        # (t, NED z of the ground)
        self.costmap = None       # newest costmap not yet handed to the mission
        self.ugv = None           # (t, x, y, z) ENU
        self.mission = None
        self.ticks = 0
        self.started = False
        self.state = ''

        subscribe_versioned(self, 'VehicleStatus', '/fmu/out/vehicle_status', self.on_status)
        subscribe_versioned(self, 'VehicleLocalPosition', '/fmu/out/vehicle_local_position',
                            self.on_lpos)
        self.create_subscription(Image, self.p['depth_topic'], self.on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, self.p['costmap_topic'], self.on_costmap, LATCHED)
        self.create_subscription(PoseStamped, self.p['ugv_pose_topic'], self.on_ugv, 10)
        self.pub_ocm = self.create_publisher(px4_msgs.OffboardControlMode,
                                             '/fmu/in/offboard_control_mode', PX4_QOS)
        self.pub_sp = self.create_publisher(px4_msgs.TrajectorySetpoint,
                                            '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.pub_cmd = self.create_publisher(px4_msgs.VehicleCommand,
                                             '/fmu/in/vehicle_command', PX4_QOS)
        self.pub_state = self.create_publisher(String, '/mission/state', LATCHED)
        self.save = self.create_client(Trigger, self.p['save_service'])
        self.create_timer(1.0 / RATE_HZ, self.tick)
        self.set_state('WAIT_PX4')

    # -- inputs ----------------------------------------------------------------

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, msg):
        self.status = msg

    def on_lpos(self, msg):
        self.lpos = msg

    def on_depth(self, msg):
        if self.lpos is None:
            return
        try:
            depth = decode_image(msg)
        except ValueError:
            return
        h, w = depth.shape
        centre = depth[h // 2 - 30:h // 2 + 30, w // 2 - 30:w // 2 + 30]
        centre = centre[np.isfinite(centre) & (centre > 0.3) & (centre < 19.0)]
        if centre.size > 100:
            # the camera sits cam_height_in_body_m above the body origin (NED: minus)
            z = self.lpos.z - float(self.p['cam_height_in_body_m']) + float(np.median(centre))
            self.ground = (self._now(), z)

    def on_costmap(self, msg):
        occ = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.costmap = (occ, msg.info.origin.position.x, msg.info.origin.position.y,
                        msg.info.resolution)

    def on_ugv(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = msg.pose.position
        self.ugv = (t, pos.x, pos.y, pos.z)

    # -- PX4 -------------------------------------------------------------------

    def set_state(self, s):
        if s != self.state:
            self.state = s
            self.pub_state.publish(String(data=s))
            self.get_logger().info(f'state {s}')

    def command(self, cmd, p1=0.0, p2=0.0):
        m = px4_msgs.VehicleCommand()
        m.timestamp = 0               # 0 = PX4 stamps it on arrival, see publish_setpoint
        m.command = int(cmd)
        m.param1, m.param2 = float(p1), float(p2)
        m.target_system = m.target_component = 1
        m.source_system = m.source_component = 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def publish_setpoint(self, pos, vel, yaw):
        # timestamp 0: PX4 replaces it with its own clock on arrival. Any
        # other value goes through the uXRCE time sync, which assumes the
        # sender uses the OS clock; these nodes run on sim time.
        ocm = px4_msgs.OffboardControlMode()
        ocm.timestamp = 0
        ocm.position = True           # finite velocity below acts as feed-forward
        self.pub_ocm.publish(ocm)
        sp = px4_msgs.TrajectorySetpoint()
        sp.timestamp = 0
        sp.position = [float(v) for v in pos]
        sp.velocity = [float(v) for v in vel]
        # PX4 uses every finite field and Python fills unset ones with 0.0
        sp.acceleration = [math.nan] * 3
        sp.jerk = [math.nan] * 3
        sp.yaw = float(yaw)
        sp.yawspeed = math.nan
        self.pub_sp.publish(sp)

    @property
    def armed(self):
        return (self.status is not None and
                self.status.arming_state == px4_msgs.VehicleStatus.ARMING_STATE_ARMED)

    @property
    def offboard(self):
        return (self.status is not None and
                self.status.nav_state == px4_msgs.VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    # -- the tick --------------------------------------------------------------

    def tick(self):
        if self.state == 'ABORTED':
            return
        lp = self.lpos
        if self.status is None:
            self.get_logger().info('waiting for /fmu/out/vehicle_status(_v1)',
                                   throttle_duration_sec=5.0)
            return
        if lp is None or not (lp.xy_valid and lp.z_valid):
            self.get_logger().info('waiting for a valid PX4 local position',
                                   throttle_duration_sec=5.0)
            return
        t = self._now()
        if self.mission is None:
            self.mission = Mission(self.params, [lp.x, lp.y, lp.z], lp.heading, t)
            self.get_logger().info(
                f'home ({lp.x:.1f}, {lp.y:.1f}, {lp.z:.1f}) NED, heading {lp.heading:.2f} rad')

        if not self.started:
            self.set_state('ARMING')
            self.publish_setpoint(self.mission.home, np.zeros(3), self.mission.yaw)
            self.ticks += 1
            if self.armed and self.offboard:
                self.started = True
                self.mission.t_state = t
            elif self.ticks >= RATE_HZ and self.ticks % int(RATE_HZ) == 0:
                if not self.offboard:
                    self.command(px4_msgs.VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                else:
                    self.command(px4_msgs.VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                if self.ticks % int(10 * RATE_HZ) == 0:
                    self.get_logger().warn('PX4 has not armed in offboard yet; if this '
                                           'persists run `commander check` in the PX4 shell')
            return

        if not (self.armed and self.offboard):
            self.get_logger().error('PX4 left offboard/armed; mission aborted')
            self.set_state('ABORTED')
            return

        ground = self.ground[1] if self.ground and t - self.ground[0] < 0.5 else None
        ugv = (self.ugv[1:] if self.ugv and t - self.ugv[0] < float(self.p['ugv_timeout_s'])
               else None)
        costmap, self.costmap = self.costmap, None
        sp = self.mission.step(t, 1.0 / RATE_HZ, [lp.x, lp.y, lp.z], ground, costmap, ugv)
        self.publish_setpoint(sp.pos, sp.vel, sp.yaw)
        self.set_state(self.mission.state)

        if self.mission.save_requested:
            self.mission.save_requested = False
            self.get_logger().info(f'survey finished ({self.mission.end_reason}); saving map')
            if self.save.service_is_ready():
                self.save.call_async(Trigger.Request()).add_done_callback(self.on_saved)
            else:
                self.get_logger().error(f"{self.p['save_service']} not available; "
                                        'the live /road/costmap is still used')

    def on_saved(self, future):
        res = future.result()
        log = self.get_logger().info if res.success else self.get_logger().error
        log(f'map save: {res.message}')


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
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
