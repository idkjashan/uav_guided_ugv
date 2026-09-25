#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /home/jashan/px4_ros_ws/install/setup.bash
source /home/jashan/uav_guided_ugv/install/setup.bash

WORLD=${1:-drdo_world2}
echo "Starting simulation in world: $WORLD..."
ros2 launch bringup sim.launch.py world:="$WORLD"
