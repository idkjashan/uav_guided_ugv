#!/usr/bin/env python3
"""Automated UAV arming, takeoff, road survey flight, and map verification.

Flies the UAV in OFFBOARD mode at 12 m AGL, surveys the road with the downward
camera, monitors real-time costmap generation from road_survey/terrain_mapper,
and triggers the map save service to verify end-to-end functionality.
"""

import math
import os
import subprocess
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import OccupancyGrid

try:
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
        VehicleStatus,
        VehicleLocalPosition,
    )
except ImportError:
    pass

PX4_SUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

PX4_PUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class UAVSurveyFlightTester(Node):
    def __init__(self):
        super().__init__('survey_mission')

        self.declare_parameter('altitude', 12.0)       # metres AGL (sensor clip 0.2..19.1m)
        self.declare_parameter('forward_dist', 18.0)    # metres along heading
        self.declare_parameter('speed', 1.5)           # m/s
        self.declare_parameter('survey_time_s', 25.0)  # seconds to fly / survey
        self.declare_parameter('maps_dir', os.path.expanduser('~/uav_guided_ugv/maps'))

        self.target_alt = float(self.get_parameter('altitude').value)
        self.forward_dist = float(self.get_parameter('forward_dist').value)
        self.speed = float(self.get_parameter('speed').value)
        self.survey_time_s = float(self.get_parameter('survey_time_s').value)
        self.maps_dir = self.get_parameter('maps_dir').value

        # Vehicle state
        self.status = None
        self.lpos = None
        self.home_pos = None
        self.target_pos = [0.0, 0.0, -self.target_alt]
        self.target_yaw = 0.0
        self.flight_start_time = None
        self.takeoff_complete = False
        self.survey_complete = False

        # Map monitoring
        self.latest_costmap = None
        self.latest_stats = ""
        self.frames_integrated = 0
        self.road_cells_detected = 0

        # Subscriptions
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self._on_status, PX4_SUB_QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self._on_status, PX4_SUB_QOS)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._on_lpos, PX4_SUB_QOS)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self._on_lpos, PX4_SUB_QOS)
        self.create_subscription(OccupancyGrid, '/road/costmap', self._on_costmap, LATCHED_QOS)
        self.create_subscription(String, '/road/stats', self._on_stats, 10)

        # Publishers
        self.pub_ocm = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_PUB_QOS)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_PUB_QOS)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', PX4_PUB_QOS)

        # Service client for saving map
        self.save_client = self.create_client(Trigger, '/terrain_mapper/save')

        # Control loop @ 20 Hz
        self.setpoints_sent = 0
        self.timer = self.create_timer(0.05, self._control_loop)
        self.get_logger().info('SurveyMission initialized. Waiting for telemetry...')

    def _on_status(self, msg):
        self.status = msg

    def _on_lpos(self, msg):
        self.lpos = msg
        if self.home_pos is None:
            self.home_pos = [msg.x, msg.y, msg.z]
            self.target_yaw = msg.heading
            self.target_pos = [msg.x, msg.y, msg.z - self.target_alt]
            self.get_logger().info(
                f'Home pose captured: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}, heading={msg.heading:.2f} rad'
            )

    def _on_costmap(self, msg: OccupancyGrid):
        self.latest_costmap = msg
        data = np.array(msg.data, dtype=np.int8)
        road_cells = np.count_nonzero((data >= 0) & (data < 100))
        self.road_cells_detected = road_cells
        self.get_logger().info(
            f'[/road/costmap] size={msg.info.width}x{msg.info.height}, road_cells={road_cells}, res={msg.info.resolution}m'
        )

    def _on_stats(self, msg: String):
        self.latest_stats = msg.data

    def _cmd(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        m = VehicleCommand()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.command = int(command)
        m.param1 = float(param1)
        m.param2 = float(param2)
        m.param3 = float(param3)
        m.param4 = float(param4)
        m.param5 = float(param5)
        m.param6 = float(param6)
        m.param7 = float(param7)
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def _heartbeat(self):
        m = OffboardControlMode()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.position = True
        m.velocity = False
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        self.pub_ocm.publish(m)

    def _setpoint(self, pos, yaw=0.0):
        m = TrajectorySetpoint()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.position = [float(pos[0]), float(pos[1]), float(pos[2])]
        m.velocity = [float('nan')] * 3
        m.acceleration = [float('nan')] * 3
        m.yaw = float(yaw)
        self.pub_sp.publish(m)

    @property
    def is_armed(self):
        return self.status is not None and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    @property
    def is_offboard(self):
        return self.status is not None and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def _control_loop(self):
        self._heartbeat()
        self._setpoint(self.target_pos, self.target_yaw)
        self.setpoints_sent += 1

        if self.home_pos is None or self.lpos is None:
            return

        # Phase 1: Stream setpoints before switching mode (PX4 requirement >= 10 msgs)
        if self.setpoints_sent < 30:
            return

        # Phase 2: Switch to Offboard
        if not self.is_offboard:
            self.mode_attempts = getattr(self, 'mode_attempts', 0) + 1
            if self.mode_attempts % 10 == 1:
                self.get_logger().info('Requesting OFFBOARD mode...')
                self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            if self.mode_attempts >= 15 and self.mode_attempts % 10 == 5:
                px4_cmd = os.path.expanduser('~/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander')
                if os.path.isfile(px4_cmd):
                    subprocess.run([px4_cmd, 'mode', 'offboard'],
                                   check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return

        # Phase 3: Arm motors
        if not self.is_armed:
            self.arm_attempts = getattr(self, 'arm_attempts', 0) + 1
            if self.arm_attempts % 10 == 1:
                self.get_logger().info('Vehicle in OFFBOARD. Arming motors...')
                self._cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            if self.arm_attempts >= 15 and self.arm_attempts % 10 == 5:
                px4_cmd = os.path.expanduser('~/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander')
                if os.path.isfile(px4_cmd):
                    subprocess.run([px4_cmd, 'arm', '-f'],
                                   check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return

        # Phase 4: Climb to target altitude
        current_alt = self.home_pos[2] - self.lpos.z
        if not self.takeoff_complete:
            if current_alt >= (self.target_alt - 0.8):
                self.takeoff_complete = True
                self.flight_start_time = self.get_clock().now()
                self.get_logger().info(f'Takeoff complete! Altitude={current_alt:.2f} m. Starting forward survey flight...')
            return

        # Phase 5: Forward survey flight
        elapsed = (self.get_clock().now() - self.flight_start_time).nanoseconds * 1e-9
        if elapsed < self.survey_time_s:
            # Advance waypoint smoothly along heading
            prog = min(1.0, elapsed / (self.survey_time_s * 0.7))
            d = self.forward_dist * prog
            dx = d * math.cos(self.target_yaw)
            dy = d * math.sin(self.target_yaw)

            self.target_pos[0] = self.home_pos[0] + dx
            self.target_pos[1] = self.home_pos[1] + dy
            self.target_pos[2] = self.home_pos[2] - self.target_alt

            if int(elapsed * 2) % 4 == 0:
                self.get_logger().info(
                    f'Surveying: t={elapsed:.1f}s/{self.survey_time_s:.0f}s | '
                    f'UAV pos=({self.lpos.x:.1f}, {self.lpos.y:.1f}, {-self.lpos.z:.1f}) | '
                    f'Road cells={self.road_cells_detected} | {self.latest_stats}'
                )
        else:
            if not self.survey_complete:
                self.survey_complete = True
                self.get_logger().info('Survey flight complete! Requesting terrain map save...')
                self._trigger_map_save()

    def _trigger_map_save(self):
        if not self.save_client.service_is_ready():
            self.get_logger().warn('/terrain_mapper/save service not ready yet. Retrying...')
            return
        req = Trigger.Request()
        future = self.save_client.call_async(req)
        future.add_done_callback(self._on_save_response)

    def _on_save_response(self, future):
        try:
            res = future.result()
            self.get_logger().info(f'Save service response: success={res.success}, message="{res.message}"')
        except Exception as e:
            self.get_logger().error(f'Failed to call save service: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UAVSurveyFlightTester()
    try:
        while rclpy.ok() and not node.survey_complete:
            rclpy.spin_once(node, timeout_sec=0.1)
        # Spin a little longer to let the save callback execute
        for _ in range(50):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
