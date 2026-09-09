#!/usr/bin/env python3
"""Unified Simulation Bringup Launch File.

Launches:
  1. Gazebo Harmonic world (drdo_world1, drdo_world2, drdo_world3, etc.)
  2. UGV (Ackermann rover) spawned at world-specific road coordinates
  3. Robot State Publisher with URDF for RViz visualization
  4. Micro-XRCE-DDS-Agent for PX4 SITL ROS 2 communication
  5. PX4 SITL UAV (x500_depth_down) spawned at world-specific coordinates
  6. UAV Downward Camera ROS-Gazebo bridges (/uav/rgb, /uav/depth, /uav/camera_info)

Usage:
  ros2 launch bringup sim.launch.py world:=drdo_world2 gui:=true
"""

import os
import shutil
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


WORLD_POSES = {
    'drdo_world1': {
        'uav': '-10.226,311.831,22.863,0.011338,0.135709,-2.161422',
        'ugv': {'x': '-12.220319', 'y': '308.976703', 'z': '22.295580', 'roll': '0.011338', 'pitch': '0.135709', 'yaw': '-2.161422'}
    },
    'drdo_world2': {
        'uav': '103.776917,-101.472992,17.318562,-0.054656,0.032451,2.460081',
        'ugv': {'x': '104.742386', 'y': '-101.9010777', 'z': '15.730011', 'roll': '-0.054656', 'pitch': '0.032451', 'yaw': '2.460081'}
    },
    'drdo_world3': {
        'uav': '108.849,-265.663,49.4752,0.045161,0.003268,1.588',
        'ugv': {'x': '109.076', 'y': '-262.736', 'z': '49.2026', 'roll': '0.005846', 'pitch': '-0.047033', 'yaw': '1.56611'}
    },
}


def launch_setup(context, *args, **kwargs):
    world_raw = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    spawn_ugv = LaunchConfiguration('spawn_ugv').perform(context).lower() == 'true'
    spawn_uav = LaunchConfiguration('spawn_uav').perform(context).lower() == 'true'
    run_agent = LaunchConfiguration('run_agent').perform(context).lower() == 'true'
    uav_model = LaunchConfiguration('uav_model').perform(context)
    px4_dir_cfg = LaunchConfiguration('px4_dir').perform(context)

    world_key = world_raw.replace('_overlay', '')
    poses = WORLD_POSES.get(world_key, WORLD_POSES['drdo_world2'])
    ugv_pose = poses['ugv']
    uav_pose_str = poses['uav']

    # Locate PX4 directory
    px4_dir = px4_dir_cfg
    if not os.path.isdir(px4_dir):
        for candidate in [os.environ.get('PX4_AUTOPILOT_DIR', ''),
                          os.path.expanduser('~/PX4-Autopilot'),
                          '/home/jashan/PX4-Autopilot']:
            if candidate and os.path.isdir(candidate):
                px4_dir = candidate
                break

    actions = []

    # 1. Launch World (Gazebo Harmonic)
    drdo_worlds_pkg = get_package_share_directory('drdo_gz_worlds')
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(drdo_worlds_pkg, 'launch', 'world.launch.py')),
        launch_arguments={'world': world_raw, 'gui': gui}.items(),
    )
    actions.append(world_launch)

    # 2. Spawn UGV (Ackermann rover) after Gazebo initializes (4s delay)
    if spawn_ugv:
        ackermann_pkg = get_package_share_directory('ackermann_gz_bringup')
        ugv_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ackermann_pkg, 'launch', 'spawn_ackermann.launch.py')),
            launch_arguments={
                'world': world_raw,
                'x': ugv_pose['x'],
                'y': ugv_pose['y'],
                'z': ugv_pose['z'],
                'roll': ugv_pose['roll'],
                'pitch': ugv_pose['pitch'],
                'yaw': ugv_pose['yaw'],
            }.items(),
        )

        # Robot State Publisher with URDF for RViz visualization
        bringup_pkg = get_package_share_directory('bringup')
        urdf_path = os.path.join(bringup_pkg, 'urdf', 'ackermann_bot.urdf')
        with open(urdf_path, 'r') as f:
            robot_desc = f.read()

        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        )

        actions.append(TimerAction(period=4.0, actions=[ugv_launch, rsp_node]))

    # 3. MicroXRCEAgent
    if run_agent:
        xrce_agent_bin = shutil.which('MicroXRCEAgent')
        if not xrce_agent_bin:
            candidate = os.path.expanduser('~/px4_ros_ws/install/microxrcedds_agent/bin/MicroXRCEAgent')
            if os.path.isfile(candidate):
                xrce_agent_bin = candidate

        if xrce_agent_bin:
            agent_cmd = ExecuteProcess(
                cmd=[xrce_agent_bin, 'udp4', '-p', '8888'],
                name='micro_xrce_agent',
                output='screen',
            )
            actions.append(TimerAction(period=2.0, actions=[agent_cmd]))

    # 4. PX4 SITL UAV Spawn
    if spawn_uav and os.path.isdir(px4_dir):
        px4_bin = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4')
        px4_rootfs = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'rootfs')
        if os.path.isfile(px4_bin):
            px4_env = dict(os.environ)
            px4_env['PX4_GZ_STANDALONE'] = '1'
            px4_env['PX4_SIM_MODEL'] = uav_model
            px4_env['PX4_GZ_WORLD'] = world_raw
            px4_env['PX4_GZ_MODEL_POSE'] = uav_pose_str

            px4_proc = ExecuteProcess(
                cmd=[px4_bin, '-d'],
                cwd=px4_rootfs,
                additional_env=px4_env,
                name='px4_sitl',
                output='screen',
            )
            actions.append(TimerAction(period=8.0, actions=[px4_proc]))

    # 5. UAV Downward Camera ROS-Gazebo Bridge
    uav_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(drdo_worlds_pkg, 'launch', 'uav_bridge.launch.py')),
        launch_arguments={'world': world_raw}.items(),
    )
    actions.append(TimerAction(period=10.0, actions=[uav_bridge_launch]))

    return actions


def generate_launch_description():
    pkg_bringup = get_package_share_directory('bringup')
    pkg_drdo = get_package_share_directory('drdo_gz_worlds')
    pkg_ackermann = get_package_share_directory('ackermann_gz_bringup')
    px4_models = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/models')
    px4_worlds = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/worlds')

    resource_path = (
        f"{px4_models}:{px4_worlds}:"
        f"{os.path.join(pkg_drdo, 'models')}:"
        f"{os.path.join(pkg_ackermann, 'models')}:"
        f"{os.path.join(pkg_bringup, 'urdf')}:"
        f"{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    )

    set_res_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path)
    set_partition = SetEnvironmentVariable(name='GZ_PARTITION', value=os.environ.get('GZ_PARTITION', 'uav_ugv'))
    set_gpu_render = SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1')
    set_glx_vendor = SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia')

    return LaunchDescription([
        set_res_path,
        set_partition,
        set_gpu_render,
        set_glx_vendor,

        DeclareLaunchArgument('world', default_value='drdo_world2',
                              description='Gazebo world: drdo_world1, drdo_world2, drdo_world3, etc.'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Whether to launch Gazebo GUI'),
        DeclareLaunchArgument('spawn_ugv', default_value='true',
                              description='Whether to spawn the Ackermann rover'),
        DeclareLaunchArgument('spawn_uav', default_value='true',
                              description='Whether to spawn the PX4 SITL UAV'),
        DeclareLaunchArgument('run_agent', default_value='true',
                              description='Whether to run MicroXRCEAgent'),
        DeclareLaunchArgument('uav_model', default_value='gz_x500_depth_down',
                              description='PX4 UAV simulation model'),
        DeclareLaunchArgument('px4_dir', default_value=os.path.expanduser('~/PX4-Autopilot'),
                              description='Path to PX4-Autopilot repository'),

        OpaqueFunction(function=launch_setup),
    ])
