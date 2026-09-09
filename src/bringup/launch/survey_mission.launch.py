#!/usr/bin/env python3
"""Launch Autonomous UAV Survey Flight Mission."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('altitude', default_value='12.0', description='Target survey altitude in metres AGL'),
        DeclareLaunchArgument('forward_dist', default_value='18.0', description='Survey distance in metres along heading'),
        DeclareLaunchArgument('speed', default_value='1.5', description='UAV flight speed in m/s'),
        DeclareLaunchArgument('survey_time_s', default_value='25.0', description='Survey duration in seconds'),

        Node(
            package='bringup',
            executable='survey_mission',
            name='survey_mission',
            output='screen',
            parameters=[{
                'altitude': LaunchConfiguration('altitude'),
                'forward_dist': LaunchConfiguration('forward_dist'),
                'speed': LaunchConfiguration('speed'),
                'survey_time_s': LaunchConfiguration('survey_time_s'),
                'use_sim_time': True,
            }],
        ),
    ])
