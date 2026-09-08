"""
Launch a DRDO terrain world in Gazebo Harmonic (gz-sim) under ROS 2 Humble.

Replaces: interiit22/launch/drdo_world{1,2,3}.launch (ROS1 + Gazebo Classic)

The UAV and UGV are intentionally NOT spawned here — the original world
files baked the "iris" drone directly into the .world SDF, but this port
spawns vehicles as separate models via `ros_gz_sim create`, so each can be
swapped/iterated on independently (e.g. ArduPilot's ardupilot_gz iris model
for the UAV, your converted prius model for the UGV).

Usage:
  ros2 launch drdo_gz_worlds world.launch.py world:=drdo_world1
  ros2 launch drdo_gz_worlds world.launch.py world:=drdo_world1_overlay

  ros2 launch drdo_gz_worlds world.launch.py world:=drdo_world1 \
      uav_sdf:=/path/to/iris_with_gimbal.sdf \
      uav_x:=-10.226 uav_y:=311.831 uav_z:=22.863 \
      uav_roll:=0.011338 uav_pitch:=0.135709 uav_yaw:=-2.161422 \
      ugv_sdf:=/path/to/prius.sdf \
      ugv_x:=-12.220319 ugv_y:=308.976703 ugv_z:=22.295580 \
      ugv_roll:=0.011338 ugv_pitch:=0.135709 ugv_yaw:=-2.161422

Reference spawn poses carried over from the original ROS1 launch/world
files, for when you're ready to wire up the UAV/UGV models:

  world          UAV x,y,z,R,P,Y                                    UGV x,y,z,R,P,Y
  drdo_world1    -10.226 311.831 22.863 0.011338 0.135709 -2.161422       -12.220319 308.976703 22.295580 0.011338 0.135709 -2.161422
  drdo_world2    103.776917 -101.472992 17.318562 -0.054656 0.032451 2.460081   104.742386 -101.9010777 15.730011 -0.054656 0.032451 2.460081
  drdo_world3    108.849 -265.663 49.4752 0.045161 0.003268 1.588               109.076 -262.736 49.2026 0.005846 -0.047033 1.56611
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('drdo_gz_worlds')
    models_path = os.path.join(pkg_share, 'models')

    # gz-sim needs to find `model://world1`, `model://world1_mesh`, etc.
    # Gazebo Classic used GAZEBO_MODEL_PATH for this; gz-sim uses
    # GZ_SIM_RESOURCE_PATH. Chain onto whatever's already set (e.g. from
    # ardupilot_gz or your UGV package) rather than clobbering it.
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = (
        models_path + os.pathsep + existing_resource_path
        if existing_resource_path else models_path
    )

    args = [
        DeclareLaunchArgument(
            'world', default_value='drdo_world2',
            description='drdo_world1 | drdo_world1_overlay | '
                        'drdo_world2 | drdo_world2_overlay | '
                        'drdo_world3 | drdo_world3_overlay'),
        DeclareLaunchArgument('gui', default_value='true'),

        # Leave *_sdf empty to skip spawning that vehicle.
        DeclareLaunchArgument('uav_sdf', default_value=''),
        DeclareLaunchArgument('uav_name', default_value='iris'),
        DeclareLaunchArgument('uav_x', default_value='0'),
        DeclareLaunchArgument('uav_y', default_value='0'),
        DeclareLaunchArgument('uav_z', default_value='0'),
        DeclareLaunchArgument('uav_roll', default_value='0'),
        DeclareLaunchArgument('uav_pitch', default_value='0'),
        DeclareLaunchArgument('uav_yaw', default_value='0'),

        DeclareLaunchArgument('ugv_sdf', default_value=''),
        DeclareLaunchArgument('ugv_name', default_value='prius'),
        DeclareLaunchArgument('ugv_x', default_value='0'),
        DeclareLaunchArgument('ugv_y', default_value='0'),
        DeclareLaunchArgument('ugv_z', default_value='0'),
        DeclareLaunchArgument('ugv_roll', default_value='0'),
        DeclareLaunchArgument('ugv_pitch', default_value='0'),
        DeclareLaunchArgument('ugv_yaw', default_value='0'),
    ]

    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', value=resource_path
    )

    world_file = PathJoinSubstitution([
        pkg_share, 'worlds',
        PythonExpression(["'", LaunchConfiguration('world'), "' + '.sdf'"]),
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': [world_file, ' -r']}.items(),
    )

    spawn_uav = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', LaunchConfiguration('uav_sdf'),
            '-name', LaunchConfiguration('uav_name'),
            '-x', LaunchConfiguration('uav_x'),
            '-y', LaunchConfiguration('uav_y'),
            '-z', LaunchConfiguration('uav_z'),
            '-R', LaunchConfiguration('uav_roll'),
            '-P', LaunchConfiguration('uav_pitch'),
            '-Y', LaunchConfiguration('uav_yaw'),
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('uav_sdf'), "' != ''"])
        ),
        output='screen',
    )

    spawn_ugv = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', LaunchConfiguration('ugv_sdf'),
            '-name', LaunchConfiguration('ugv_name'),
            '-x', LaunchConfiguration('ugv_x'),
            '-y', LaunchConfiguration('ugv_y'),
            '-z', LaunchConfiguration('ugv_z'),
            '-R', LaunchConfiguration('ugv_roll'),
            '-P', LaunchConfiguration('ugv_pitch'),
            '-Y', LaunchConfiguration('ugv_yaw'),
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('ugv_sdf'), "' != ''"])
        ),
        output='screen',
    )

    return LaunchDescription(args + [set_resource_path, gz_sim, spawn_uav, spawn_ugv])
