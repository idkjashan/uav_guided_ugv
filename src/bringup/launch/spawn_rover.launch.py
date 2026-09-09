#!/usr/bin/env python3
"""Spawn Ackermann Rover with URDF and Robot State Publisher."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    roll = LaunchConfiguration('roll').perform(context)
    pitch = LaunchConfiguration('pitch').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    ackermann_pkg = get_package_share_directory('ackermann_gz_bringup')
    bringup_pkg = get_package_share_directory('bringup')

    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ackermann_pkg, 'launch', 'spawn_ackermann.launch.py')),
        launch_arguments={
            'world': world,
            'x': x,
            'y': y,
            'z': z,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
        }.items(),
    )

    urdf_path = os.path.join(bringup_pkg, 'urdf', 'ackermann_bot.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    )

    return [spawn_launch, rsp_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='drdo_world2', description='World name'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn X'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y'),
        DeclareLaunchArgument('z', default_value='0.2', description='Spawn Z'),
        DeclareLaunchArgument('roll', default_value='0.0', description='Spawn Roll'),
        DeclareLaunchArgument('pitch', default_value='0.0', description='Spawn Pitch'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn Yaw'),
        OpaqueFunction(function=launch_setup),
    ])
