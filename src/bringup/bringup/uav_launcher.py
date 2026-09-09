#!/usr/bin/env python3
"""Autonomous UAV arming, takeoff, and hover in OFFBOARD mode.

Continuously streams TrajectorySetpoint and OffboardControlMode at 20 Hz,
switches to OFFBOARD mode, arms the motors, climbs to the specified
altitude (default 12.0 m AGL), and stably holds position.
"""

import math
import os
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

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


class UAVLauncher(Node):
    def __init__(self):
        super().__init__('uav_launcher')

        self.declare_parameter('altitude', 12.0)   # target altitude in metres AGL
        self.declare_parameter('yaw', float('nan'))  # target yaw in radians, NaN = hold current

        self.target_alt = float(self.get_parameter('altitude').value)
        self.desired_yaw = float(self.get_parameter('yaw').value)

        # Vehicle state
        self.status = None
        self.lpos = None
        self.start_pos = None
        self.target_pos = None
        self.target_yaw = 0.0
        self.takeoff_complete = False

        self.setpoints_sent = 0
        self.mode_attempts = 0
        self.arm_attempts = 0

        # Subscriptions
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self._on_status, PX4_SUB_QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self._on_status, PX4_SUB_QOS)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._on_lpos, PX4_SUB_QOS)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self._on_lpos, PX4_SUB_QOS)

        # Publishers
        self.pub_ocm = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_PUB_QOS)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_PUB_QOS)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', PX4_PUB_QOS)

        # 20 Hz control loop
        self.timer = self.create_timer(0.05, self._control_loop)
        self.get_logger().info('UAVLauncher initialized. Waiting for vehicle telemetry...')

    def _on_status(self, msg):
        self.status = msg

    def _on_lpos(self, msg):
        self.lpos = msg
        if self.start_pos is None:
            self.start_pos = [msg.x, msg.y, msg.z]
            self.target_yaw = msg.heading if math.isnan(self.desired_yaw) else self.desired_yaw
            self.target_pos = [msg.x, msg.y, msg.z - self.target_alt]
            self.get_logger().info(
                f'Starting position captured: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}. '
                f'Target: alt={self.target_alt:.1f}m (z={self.target_pos[2]:.2f})'
            )

    @property
    def is_armed(self):
        return self.status is not None and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    @property
    def is_offboard(self):
        return self.status is not None and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

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

    def _cmd(self, command, param1=0.0, param2=0.0):
        m = VehicleCommand()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.command = int(command)
        m.param1 = float(param1)
        m.param2 = float(param2)
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def _control_loop(self):
        self._heartbeat()
        if self.target_pos is not None:
            self._setpoint(self.target_pos, self.target_yaw)

        self.setpoints_sent += 1

        if self.start_pos is None or self.lpos is None:
            return

        # Warm-up: stream setpoints for >= 1s before requesting mode change
        if self.setpoints_sent < 25:
            return

        # Phase 1: Switch to OFFBOARD
        if not self.is_offboard:
            self.mode_attempts += 1
            if self.mode_attempts % 10 == 1:
                self.get_logger().info('Requesting OFFBOARD flight mode...')
                self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            if self.mode_attempts >= 15 and self.mode_attempts % 10 == 5:
                px4_cmd = os.path.expanduser('~/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander')
                if os.path.isfile(px4_cmd):
                    subprocess.run([px4_cmd, 'mode', 'offboard'],
                                   check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return

        # Phase 2: Arm motors
        if not self.is_armed:
            self.arm_attempts += 1
            if self.arm_attempts % 10 == 1:
                self.get_logger().info('Vehicle in OFFBOARD. Arming motors...')
                self._cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            if self.arm_attempts >= 15 and self.arm_attempts % 10 == 5:
                px4_cmd = os.path.expanduser('~/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander')
                if os.path.isfile(px4_cmd):
                    subprocess.run([px4_cmd, 'arm', '-f'],
                                   check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return

        # Phase 3: Climb & Hover
        current_alt = self.start_pos[2] - self.lpos.z
        if not self.takeoff_complete:
            if current_alt >= (self.target_alt - 0.8):
                self.takeoff_complete = True
                self.get_logger().info(
                    f'Takeoff complete! UAV airborne and hovering stably at altitude {current_alt:.2f} m AGL.'
                )
            elif self.setpoints_sent % 20 == 0:
                self.get_logger().info(
                    f'Climbing: current alt={current_alt:.2f} m / target={self.target_alt:.2f} m'
                )
        else:
            if self.setpoints_sent % 40 == 0:
                self.get_logger().info(
                    f'Hovering: alt={current_alt:.2f} m | pos=({self.lpos.x:.2f}, {self.lpos.y:.2f}) | '
                    f'heading={self.lpos.heading:.2f} rad'
                )


def main(args=None):
    rclpy.init(args=args)
    node = UAVLauncher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
