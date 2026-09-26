"""Launch pre-configured RViz2 visualization for UAV-guided UGV.

Displays:
  - UAV downward RGB camera (/uav/rgb)
  - UAV downward depth camera (/uav/depth) with full 0-20m dynamic range
  - 3D Terrain PointCloud (/terrain/pointcloud)
  - Road costmap (/road/costmap)
  - UGV pure pursuit path & centreline (/ugv/path)
  - Estimated UGV pose (/ugv/pose)
  - Ground-truth UGV pose (/ugv/ground_truth)
  - UAV pose (/uav/pose)
  - TF tree (world, map, uav_base_link, etc.)

Usage:
    ros2 launch bringup rviz.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_cfg = os.path.join(
        get_package_share_directory('road_survey'), 'rviz', 'survey.rviz')

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', os.environ.get('ROS_DOMAIN_ID', '42')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', os.environ.get('ROS_LOCALHOST_ONLY', '0')),
        DeclareLaunchArgument('rviz_config', default_value=rviz_cfg,
                              description='Path to RViz configuration file'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
