"""Mission: map the road, bring the UAV back, guide the UGV to the end of it.

    ros2 launch bringup mission.launch.py                  # survey, then guide
    ros2 launch bringup mission.launch.py survey:=false    # reuse maps/road_map.npz

Needs sim.launch.py (or the real vehicles) already running. Nodes:
  terrain_mapper   depth + PX4 pose -> /road/costmap      (survey:=true)
  map_publisher    saved road_map.npz -> /road/costmap    (survey:=false)
  ugv_localizer    UAV RGB + PX4 pose -> /ugv/pose
  mission          PX4 offboard: takeoff, survey, return, track the UGV
  ugv_follower     /ugv/pose + /road/costmap -> /cmd_vel
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    survey = LaunchConfiguration('survey').perform(context).lower() == 'true'
    params = LaunchConfiguration('params').perform(context)
    sim = {'use_sim_time': True}
    road_params = os.path.join(get_package_share_directory('road_survey'),
                               'config', 'road_survey.yaml')
    rviz_cfg = os.path.join(get_package_share_directory('road_survey'), 'rviz', 'survey.rviz')

    if survey:
        road = Node(package='road_survey', executable='terrain_mapper', name='terrain_mapper',
                    parameters=[road_params, sim], output='screen')
    else:
        road = Node(package='road_survey', executable='map_publisher', name='map_publisher',
                    parameters=[{'map_npz': LaunchConfiguration('map_npz').perform(context)}, sim],
                    output='screen')
    return [
        road,
        Node(package='guidance', executable='ugv_localizer', name='ugv_localizer',
             parameters=[params, sim], output='screen'),
        Node(package='guidance', executable='ugv_follower', name='ugv_follower',
             parameters=[params, sim], output='screen'),
        Node(package='guidance', executable='mission', name='mission',
             parameters=[params, sim, {'survey': survey}], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg],
             parameters=[sim], condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('survey', default_value='true',
                              description='false: skip the survey and load map_npz'),
        DeclareLaunchArgument('map_npz', default_value='~/uav_guided_ugv/maps/road_map.npz'),
        DeclareLaunchArgument('params', default_value=os.path.join(
            get_package_share_directory('guidance'), 'config', 'guidance.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
