"""Bring up the road survey and visualization.

    ros2 launch road_survey survey.launch.py world:=drdo_world2
    ros2 launch road_survey survey.launch.py world:=drdo_world1 rviz:=true
    ros2 launch road_survey survey.launch.py bridge:=true    # if uav_bridge not already running

Assumes PX4 SITL + Gazebo + MicroXRCEAgent are already running.
Launches:
  1. terrain_mapper (road detection, elevation grid, costmap publisher)
  2. static_transform_publisher (unifies map -> odom for the chosen world)
  3. survey_bridge (optional Gazebo depth camera bridge if not already running)
  4. rviz2 (pre-configured visualization for road costmap, cameras, UGV & UAV)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# World-specific spawn offsets of UGV relative to UAV in ENU map frame:
# dx = ugv_x - uav_x, dy = ugv_y - uav_y, dz = ugv_z - uav_z
WORLD_UGV_OFFSETS = {
    'drdo_world1': (-1.994, -2.854, -0.567, 0.0),
    'drdo_world1_overlay': (-1.994, -2.854, -0.567, 0.0),
    'drdo_world2': (0.965, -0.428, -1.589, 0.0),
    'drdo_world2_overlay': (0.965, -0.428, -1.589, 0.0),
    'drdo_world3': (0.227, 2.927, -0.273, 0.0),
    'drdo_world3_overlay': (0.227, 2.927, -0.273, 0.0),
}


def launch_setup(context, *args, **kwargs):
    world_val = LaunchConfiguration('world').perform(context)
    params_val = LaunchConfiguration('params').perform(context)
    use_bridge = LaunchConfiguration('bridge')
    use_rviz = LaunchConfiguration('rviz')

    clock_topic = f'/world/{world_val}/clock'

    # Optional standalone bridge for depth sensor
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='survey_bridge',
        condition=IfCondition(use_bridge),
        arguments=[
            '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            f'{clock_topic}@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[
            ('/depth_camera', '/uav/depth'),
            ('/camera_info', '/uav/depth_camera_info'),
            (clock_topic, '/clock'),
        ],
        output='screen',
    )

    # Road survey terrain mapper node
    mapper = Node(
        package='road_survey',
        executable='terrain_mapper',
        name='terrain_mapper',
        parameters=[params_val, {'use_sim_time': True}],
        output='screen',
    )

    # Static transform: map -> odom (connects UGV odom frame to survey map frame)
    offset = WORLD_UGV_OFFSETS.get(world_val, (0.0, 0.0, 0.0, 0.0))
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=[
            '--x', str(offset[0]),
            '--y', str(offset[1]),
            '--z', str(offset[2]),
            '--yaw', str(offset[3]),
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
        output='screen',
    )

    # RViz2 with pre-configured displays
    rviz_config = os.path.join(
        get_package_share_directory('road_survey'), 'rviz', 'survey.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return [bridge, mapper, map_to_odom_tf, rviz_node]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('road_survey'), 'config', 'road_survey.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='drdo_world2',
                              description='Gazebo world name (drdo_world1, drdo_world2, drdo_world3, etc.)'),
        DeclareLaunchArgument('params', default_value=default_params,
                              description='Path to road_survey YAML parameter file'),
        DeclareLaunchArgument('bridge', default_value='false',
                              description='Launch camera bridge if not already running'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2 with road survey configuration'),
        OpaqueFunction(function=launch_setup),
    ])
