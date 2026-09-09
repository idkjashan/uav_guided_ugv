"""
Bridge UAV downward camera topics from Gazebo Sim (Harmonic) to ROS 2 Humble.

Bridged topics:
  - Gazebo RGB image       -> /uav/rgb          (sensor_msgs/msg/Image)
  - Gazebo CameraInfo      -> /uav/camera_info  (sensor_msgs/msg/CameraInfo)
  - Gazebo Depth image     -> /uav/depth        (sensor_msgs/msg/Image)
  - Gazebo PointCloud2     -> /uav/points       (sensor_msgs/msg/PointCloud2)
  - Gazebo sim clock       -> /clock            (rosgraph_msgs/msg/Clock)

Usage:
  ros2 launch drdo_gz_worlds uav_bridge.launch.py world:=drdo_world2
  ros2 launch drdo_gz_worlds uav_bridge.launch.py world:=drdo_world1 model_name:=x500_depth_down_0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    model_name = LaunchConfiguration('model_name').perform(context)

    # Gazebo topics for downward OakD-Lite camera
    gz_rgb_image = f'/world/{world}/model/{model_name}/link/camera_link/sensor/IMX214/image'
    gz_rgb_info = f'/world/{world}/model/{model_name}/link/camera_link/sensor/IMX214/camera_info'
    gz_depth_image = '/depth_camera'
    gz_depth_info = '/camera_info'
    gz_depth_points = '/depth_camera/points'
    gz_clock = f'/world/{world}/clock'

    bridge_args = [
        f'{gz_rgb_image}@sensor_msgs/msg/Image@gz.msgs.Image',
        f'{gz_rgb_info}@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        f'{gz_depth_image}@sensor_msgs/msg/Image@gz.msgs.Image',
        f'{gz_depth_info}@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        f'{gz_depth_points}@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        f'{gz_clock}@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    ]

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='uav_camera_bridge',
        output='screen',
        arguments=bridge_args,
        remappings=[
            (gz_rgb_image, '/uav/rgb'),
            (gz_rgb_info, '/uav/camera_info'),
            (gz_depth_image, '/uav/depth'),
            (gz_depth_info, '/uav/depth_camera_info'),
            (gz_depth_points, '/uav/points'),
            (gz_clock, '/clock'),
        ],
    )

    return [bridge_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='drdo_world2',
            description='World name containing the UAV model (e.g. drdo_world1, drdo_world2, drdo_world3)'
        ),
        DeclareLaunchArgument(
            'model_name',
            default_value='x500_depth_down_0',
            description='Spawned UAV model instance name (e.g. x500_depth_down_0 or x500_depth_down)'
        ),
        OpaqueFunction(function=launch_setup),
    ])
