"""mission -- PX4 offboard wrapper around guidance.mission.Mission.

Handles the PX4 side: stream setpoints, switch to offboard, arm, then hand
every tick to the mission logic and publish the setpoint it returns. Feeds the
logic the three facts it needs: ground height under the UAV (median of the
depth image centre), the latest road costmap, and the UGV pose.

Provides an interactive CLI interface on startup to choose between:
  1. Road Survey (Autonomous or Manual Teleop)
     - Allows ending survey early at any time and saving the current map status
     - Automatically transitions to Guidance after map save
  2. Guidance (Direct execution using existing map)

    ros2 run guidance mission --ros-args --params-file config/guidance.yaml
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from road_survey.depth import decode_image

from .explore import ExploreParams
from .mission import Mission, MissionParams
from .px4 import PX4_QOS, px4_msgs, subscribe_versioned

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
RATE_HZ = 20.0

_tty_lock = threading.Lock()


def _prompt(text: str, default: str = '') -> str:
    """Prompt user via /dev/tty if available, otherwise fallback to stdin."""
    with _tty_lock:
        try:
            with open('/dev/tty', 'w') as out_f, open('/dev/tty', 'r') as in_f:
                out_f.write(text)
                out_f.flush()
                line = in_f.readline()
                val = line.strip()
                return val if val else default
        except Exception:
            pass
        try:
            val = input(text).strip()
            return val if val else default
        except Exception:
            return default


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
            ('load_service', '/terrain_mapper/load'),
            ('cam_height_in_body_m', 0.242),
            ('interactive', True),
            ('manual_survey', False),
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
        self.goal_reached = False
        self.mission = None
        self.ticks = 0
        self.started = False
        self.state = ''
        self.manual_survey = bool(self.p.get('manual_survey', False))
        self._load_timer = None

        subscribe_versioned(self, 'VehicleStatus', '/fmu/out/vehicle_status', self.on_status)
        subscribe_versioned(self, 'VehicleLocalPosition', '/fmu/out/vehicle_local_position',
                            self.on_lpos)
        self.create_subscription(Image, self.p['depth_topic'], self.on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, self.p['costmap_topic'], self.on_costmap, LATCHED)
        self.create_subscription(PoseStamped, self.p['ugv_pose_topic'], self.on_ugv, 10)
        self.create_subscription(Bool, '/ugv/goal_reached', self.on_goal_reached, LATCHED)
        self.pub_ocm = self.create_publisher(px4_msgs.OffboardControlMode,
                                             '/fmu/in/offboard_control_mode', PX4_QOS)
        self.pub_sp = self.create_publisher(px4_msgs.TrajectorySetpoint,
                                             '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.pub_cmd = self.create_publisher(px4_msgs.VehicleCommand,
                                             '/fmu/in/vehicle_command', PX4_QOS)
        self.pub_state = self.create_publisher(String, '/mission/state', LATCHED)
        self.save = self.create_client(Trigger, self.p['save_service'])
        self.load_client = self.create_client(Trigger, str(self.p['load_service']))

        if bool(self.p.get('interactive', True)):
            self._run_interactive_menu()
        elif bool(self.params.survey):
            self._start_survey_listener_thread()

        self.create_timer(1.0 / RATE_HZ, self.tick)
        self.set_state('WAIT_PX4' if not self.manual_survey else 'MANUAL_SURVEY')

    # -- Interactive Menu & Survey Controls ------------------------------------

    def _run_interactive_menu(self):
        banner = (
            "\n"
            "============================================================\n"
            "           DRDO UAV-GUIDED UGV MISSION CONTROLLER           \n"
            "============================================================\n"
            "Select Mission Mode:\n"
            "  [1] Survey   - Map the road corridor from the air\n"
            "  [2] Guidance - Escort UGV along mapped road to goal\n"
        )
        choice = _prompt(banner + "Choice [1/2] (default: 1): ", default="1")
        if choice == "2":
            self.get_logger().info("Mission Mode: GUIDANCE (using existing map)")
            self.params.survey = False
            self.manual_survey = False
            self._request_map_load()
        else:
            self.get_logger().info("Mission Mode: SURVEY")
            survey_banner = (
                "\nSelect Survey Method:\n"
                "  [1] Autonomous - UAV automatically explores and maps road\n"
                "  [2] Manual     - Pilot UAV manually (teleop / joystick / QGC)\n"
            )
            s_choice = _prompt(survey_banner + "Choice [1/2] (default: 1): ", default="1")
            if s_choice == "2":
                self.get_logger().info("Survey Method: MANUAL (pilot via teleop / QGC)")
                self.manual_survey = True
                self.params.survey = True
                self._start_manual_survey_thread()
            else:
                self.get_logger().info("Survey Method: AUTONOMOUS")
                self.manual_survey = False
                self.params.survey = True
                self._start_survey_listener_thread()

    def _request_map_load(self):
        if self.load_client.service_is_ready():
            self.load_client.call_async(Trigger.Request()).add_done_callback(self._on_map_loaded)
        else:
            self.get_logger().info("Waiting for terrain_mapper ~/load service...")
            self._load_timer = self.create_timer(1.0, self._try_load_timer)

    def _try_load_timer(self):
        if self.load_client.service_is_ready():
            self.load_client.call_async(Trigger.Request()).add_done_callback(self._on_map_loaded)
            if self._load_timer is not None:
                self._load_timer.cancel()
                self._load_timer = None

    def _on_map_loaded(self, future):
        res = future.result()
        if res.success:
            self.get_logger().info(f"[MISSION] Costmap loaded: {res.message}")
        else:
            self.get_logger().warn(f"[MISSION] Map load: {res.message} (will wait for /road/costmap)")

    def _start_survey_listener_thread(self):
        def _listen():
            while rclpy.ok() and self.state not in ('SURVEY', 'RETURN', 'ACQUIRE', 'TRACK', 'DONE', 'ABORTED'):
                time.sleep(0.2)
            if self.state == 'SURVEY':
                msg = (
                    "\n"
                    ">>> Autonomous survey in progress...\n"
                    ">>> Press [Enter] or type 'end' / 'save' at any time to finish survey early & save map: "
                )
                _prompt(msg)
                if self.state == 'SURVEY' or (self.mission and self.mission.state == 'SURVEY'):
                    self.get_logger().info("User requested early end of autonomous survey.")
                    self.end_survey_early()
        t = threading.Thread(target=_listen, daemon=True)
        t.start()

    def _start_manual_survey_thread(self):
        def _listen_manual():
            banner = (
                "\n"
                ">>> Manual Survey Active!\n"
                ">>> Pilot the UAV over the road corridor (via QGC, joystick, or RC).\n"
                ">>> Press [Enter] or type 'end' / 'save' when finished to save map: "
            )
            _prompt(banner)
            self.get_logger().info("Manual survey finished by user. Saving map...")
            self.save_manual_map()
        t = threading.Thread(target=_listen_manual, daemon=True)
        t.start()

    def end_survey_early(self):
        t = self._now()
        if self.manual_survey:
            self.save_manual_map()
            return
        if self.mission is not None and self.mission.state in ('SURVEY', 'TAKEOFF'):
            if self.mission.request_early_end(t):
                self.get_logger().info("Autonomous survey ended early. Saving map and returning to base...")
                if self.save.service_is_ready():
                    self.save.call_async(Trigger.Request()).add_done_callback(self.on_saved)

    def save_manual_map(self):
        if self.save.service_is_ready():
            self.save.call_async(Trigger.Request()).add_done_callback(self.on_saved_manual)
        else:
            self.get_logger().warn("Terrain mapper save service not ready; retrying...")
            time.sleep(1.0)
            if self.save.service_is_ready():
                self.save.call_async(Trigger.Request()).add_done_callback(self.on_saved_manual)

    def on_saved_manual(self, future):
        res = future.result()
        if res.success:
            self.get_logger().info(f"[MISSION] Manual survey map saved: {res.message}")
        else:
            self.get_logger().error(f"[MISSION] Manual survey map save failed: {res.message}")

        ans = _prompt("\n[MISSION] Proceed to Guidance system now? [Y/n] (default: Y): ", default="y")
        if ans.lower() in ('', 'y', 'yes'):
            self.get_logger().info("Transitioning to Guidance system (Offboard escort mode)...")
            self.manual_survey = False
            self.params.survey = False
            self.mission = None
            self.started = False
            self.ticks = 0
            self.set_state('WAIT_PX4')
        else:
            self.get_logger().info("Mission finished. UAV hovering in manual mode.")

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

    def on_goal_reached(self, msg):
        self.goal_reached = self.goal_reached or msg.data

    # -- PX4 -------------------------------------------------------------------

    def set_state(self, s):
        if s != self.state:
            self.state = s
            self.pub_state.publish(String(data=s))
            self.get_logger().info(f'state {s}')

    def command(self, cmd, p1=0.0, p2=0.0):
        m = px4_msgs.VehicleCommand()
        m.timestamp = 0
        m.command = int(cmd)
        m.param1, m.param2 = float(p1), float(p2)
        m.target_system = m.target_component = 1
        m.source_system = m.source_component = 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def publish_setpoint(self, pos, vel, yaw):
        ocm = px4_msgs.OffboardControlMode()
        ocm.timestamp = 0
        ocm.position = True
        self.pub_ocm.publish(ocm)
        sp = px4_msgs.TrajectorySetpoint()
        sp.timestamp = 0
        sp.position = [float(v) for v in pos]
        sp.velocity = [float(v) for v in vel]
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
        if self.manual_survey:
            self.set_state('MANUAL_SURVEY')
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
        sp = self.mission.step(t, 1.0 / RATE_HZ, [lp.x, lp.y, lp.z], ground, costmap, ugv,
                               self.goal_reached)
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
        self.get_logger().info(
            '[MISSION] Map saved successfully! UAV returning to base to begin UGV Guidance.')


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
