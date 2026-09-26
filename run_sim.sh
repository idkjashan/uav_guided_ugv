#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source "$HOME/px4_ros_ws/install/setup.bash"
source "$(dirname "$(readlink -f "$0")")/install/setup.bash"

WORLD=${1:-drdo_world2}
echo "Starting simulation in world: $WORLD..."
ros2 launch bringup sim.launch.py world:="$WORLD"
