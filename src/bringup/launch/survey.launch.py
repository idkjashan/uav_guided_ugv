#!/usr/bin/env python3
"""Launch Road Survey Mapping and Visualization.

Launches:
  - road_survey terrain_mapper_node
  - RViz visualization for real-time costmap and UAV telemetry
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    road_survey_pkg = get_package_share_directory('road_survey')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='drdo_world2',
                              description='World name for camera and clock bridging'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Whether to start RViz visualization'),
        DeclareLaunchArgument('use_bridge', default_value='false',
                              description='Whether to launch depth bridge (usually handled by sim bringup)'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(road_survey_pkg, 'launch', 'survey.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'rviz': LaunchConfiguration('rviz'),
                'use_bridge': LaunchConfiguration('use_bridge'),
            }.items(),
        ),
    ])
