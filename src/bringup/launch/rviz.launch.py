#!/usr/bin/env python3
"""Launch RViz2 for Road Survey & UGV/UAV Visualization."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_road_survey = get_package_share_directory('road_survey')
    rviz_config = os.path.join(pkg_road_survey, 'rviz', 'survey.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
