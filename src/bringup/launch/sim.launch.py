"""Simulation: Gazebo world, UGV, MicroXRCEAgent, PX4 SITL UAV, camera bridge.

    ros2 launch bringup sim.launch.py world:=drdo_world2

Spawn poses come from config/world_poses.yaml. Start the mission in a second
terminal once PX4 prints "Ready for takeoff!":

    ros2 launch bringup mission.launch.py
"""

import os
import shutil

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    world = LaunchConfiguration('world').perform(context)
    px4_dir = os.path.expanduser(LaunchConfiguration('px4_dir').perform(context))
    bringup = get_package_share_directory('bringup')
    worlds_pkg = get_package_share_directory('drdo_gz_worlds')
    ugv_pkg = get_package_share_directory('ackermann_gz_bringup')

    with open(os.path.join(bringup, 'config', 'world_poses.yaml'), encoding='utf-8') as fh:
        poses = yaml.safe_load(fh)
    key = world.replace('_overlay', '')
    if key not in poses:
        raise RuntimeError(f'no spawn poses for {key!r} in world_poses.yaml')
    uav, ugv = poses[key]['uav'], poses[key]['ugv']

    resource_path = os.pathsep.join([
        os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models'),
        os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'worlds'),
        os.path.join(worlds_pkg, 'models'),
        os.path.join(ugv_pkg, 'models'),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ])
    actions = [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path)]

    nvidia_opt = LaunchConfiguration('nvidia_gpu').perform(context).lower()
    has_nvidia = shutil.which('nvidia-smi') is not None
    if nvidia_opt == 'true' or (nvidia_opt == 'auto' and has_nvidia):
        actions.append(SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'))
        actions.append(SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(worlds_pkg, 'launch', 'world.launch.py')),
        launch_arguments={'world': world}.items()))

    actions.append(TimerAction(period=4.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ugv_pkg, 'launch', 'spawn_ackermann.launch.py')),
        launch_arguments={'world': world, **{k: str(v) for k, v in ugv.items()}}.items())]))

    domain_id = str(os.environ.get('ROS_DOMAIN_ID', '42'))

    agent = shutil.which('MicroXRCEAgent')
    if agent is None:
        raise RuntimeError('MicroXRCEAgent not on PATH')
    actions.append(ExecuteProcess(cmd=[agent, 'udp4', '-p', '8888'],
                                  additional_env={'ROS_DOMAIN_ID': domain_id},
                                  name='micro_xrce_agent', output='screen'))

    px4_bin = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4')
    if not os.path.isfile(px4_bin):
        raise RuntimeError(f'{px4_bin} not found; build PX4 with make px4_sitl_default')
    pose = ','.join(str(uav[k]) for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'))
    actions.append(TimerAction(period=8.0, actions=[ExecuteProcess(
        cmd=[px4_bin, '-d'],
        cwd=os.path.join(px4_dir, 'build', 'px4_sitl_default', 'rootfs'),
        additional_env={'PX4_GZ_STANDALONE': '1',
                        'PX4_SIM_MODEL': LaunchConfiguration('uav_model').perform(context),
                        'PX4_GZ_WORLD': world,
                        'PX4_GZ_MODEL_POSE': pose,
                        'ROS_DOMAIN_ID': domain_id},
        name='px4_sitl', output='screen')]))

    # PX4's local frame (our 'map') is the gz world shifted to the UAV spawn
    # point, axes unchanged. Validation only: it lets RViz and pose_check put
    # /ugv/ground_truth (world frame) next to /ugv/pose (map frame).
    actions.append(Node(
        package='tf2_ros', executable='static_transform_publisher', name='world_to_map',
        arguments=['--x', str(uav['x']), '--y', str(uav['y']), '--z', str(uav['z']),
                   '--frame-id', 'world', '--child-frame-id', 'map']))

    actions.append(TimerAction(period=10.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(worlds_pkg, 'launch', 'uav_bridge.launch.py')),
        launch_arguments={'world': world}.items())]))
    return actions


def generate_launch_description():
    px4_default = os.environ.get('PX4_AUTOPILOT_DIR', os.path.expanduser('~/PX4-Autopilot'))
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', os.environ.get('ROS_DOMAIN_ID', '42')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', os.environ.get('ROS_LOCALHOST_ONLY', '0')),
        SetEnvironmentVariable('GZ_PARTITION', os.environ.get('GZ_PARTITION', '')),
        DeclareLaunchArgument('world', default_value='drdo_world2'),
        DeclareLaunchArgument('uav_model', default_value='gz_x500_depth_down'),
        DeclareLaunchArgument('px4_dir', default_value=px4_default),
        DeclareLaunchArgument('nvidia_gpu', default_value='auto',
                              description='Enable NVIDIA GPU offload: auto, true, or false'),
        OpaqueFunction(function=launch_setup),
    ])
