"""Bring up the road survey.

    ros2 launch road_survey survey.launch.py world:=drdo_world1
    ros2 launch road_survey survey.launch.py bridge:=false   # you bridge yourself

Assumes PX4 SITL + Gazebo + MicroXRCEAgent are already running.  The bridge
here only carries what the mapper needs (depth, its camera_info, clock); the
RGB stream is not required for stage 1.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    world = LaunchConfiguration('world')
    params = LaunchConfiguration('params')
    use_bridge = LaunchConfiguration('bridge')
    default_params = os.path.join(
        get_package_share_directory('road_survey'), 'config', 'road_survey.yaml')

    clock_topic = PythonExpression(["'/world/' + '", world, "' + '/clock'"])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='survey_bridge',
        condition=IfCondition(use_bridge),
        arguments=[
            '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            [clock_topic, '@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        ],
        remappings=[
            ('/depth_camera', '/uav/depth'),
            ('/depth_camera/camera_info', '/uav/depth_camera_info'),
            (clock_topic, '/clock'),
        ],
        output='screen',
    )

    mapper = Node(
        package='road_survey',
        executable='terrain_mapper',
        name='terrain_mapper',
        parameters=[params, {'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='drdo_world1'),
        DeclareLaunchArgument('params', default_value=default_params),
        DeclareLaunchArgument('bridge', default_value='true'),
        bridge,
        mapper,
    ])
