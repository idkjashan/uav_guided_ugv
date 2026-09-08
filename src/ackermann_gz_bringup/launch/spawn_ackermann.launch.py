#!/usr/bin/env python3
"""
Spawns the ackermann_bot model (Ackermann-steered vehicle with an ArUco
marker on its roof) into a Gazebo Harmonic (gz-sim) world that is ALREADY
RUNNING in a separate terminal, e.g.:

    gz sim -r my_world.sdf

This launch file does NOT start gz sim itself. It only:
  1. Spawns the robot model into the named world via `ros_gz_sim create`.
  2. Starts a ros_gz_bridge parameter_bridge so cmd_vel / odom / tf /
     joint_states / clock are usable from ROS 2.

Usage:
    ros2 launch ackermann_gz_bringup spawn_ackermann.launch.py world:=my_world

Arguments:
    world       Name of the already-running gz world (as declared by
                <world name="..."> in the SDF the other terminal loaded).
                Default: "empty"
    robot_name  Name to give the spawned model / its ROS topic namespace.
                Default: "ackermann_bot"
    x, y, z, yaw  Spawn pose in the world. Defaults: 0 0 0.1 0
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    roll = LaunchConfiguration('roll').perform(context)
    pitch = LaunchConfiguration('pitch').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    pkg_share = get_package_share_directory('ackermann_gz_bringup')
    model_sdf_path = os.path.join(
        pkg_share, 'models', 'ackermann_bot', 'model.sdf'
    )

    # Spawn the model into the world that is already running elsewhere.
    # ros_gz_sim's `create` node talks to the /world/<world>/create service,
    # so it attaches to whatever gz sim instance owns that world name -- it
    # does not launch or require gz sim itself to be started here.
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', world,
            '-file', model_sdf_path,
            '-name', robot_name,
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw,
            '-allow_renaming', 'true',
        ],
    )

    # ROS <-> GZ topic bridge, namespaced to the spawned robot / world.
    bridge_args = [
        # Sim clock -> ROS clock
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        # ROS geometry_msgs/Twist -> gz cmd_vel consumed by AckermannSteering
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        # gz odometry -> ROS nav_msgs/Odometry
        '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        # gz tf -> ROS tf2
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        # gz joint states -> ROS sensor_msgs/JointState
        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
    ]

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ackermann_bot_gz_bridge',
        output='screen',
        arguments=bridge_args,
        remappings=[
            ('/odometry', '/odom'),
        ],
    )

    return [spawn_node, bridge_node]


def generate_launch_description():
    pkg_share = get_package_share_directory('ackermann_gz_bringup')

    # Make the model (and its ArUco texture) discoverable by gz-sim via
    # model:// URIs, in addition to the absolute -file path used to spawn it.
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_share, 'models')
        + os.pathsep
        + os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='empty',
            description='Name of the already-running gz sim world to spawn into.'
        ),
        DeclareLaunchArgument(
            'robot_name', default_value='ackermann_bot',
            description='Model name / ROS topic namespace for the spawned vehicle.'
        ),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position.'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y position.'),
        DeclareLaunchArgument('z', default_value='0.1', description='Spawn Z position.'),
        DeclareLaunchArgument('roll', default_value='0.0', description='Spawn roll (rad).'),
        DeclareLaunchArgument('pitch', default_value='0.0', description='Spawn pitch (rad).'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw (rad).'),

        set_resource_path,
        OpaqueFunction(function=launch_setup),
    ])
