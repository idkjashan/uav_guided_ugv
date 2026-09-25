#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /home/jashan/px4_ros_ws/install/setup.bash
source /home/jashan/uav_guided_ugv/install/setup.bash

echo "Starting pose_check in background..."
ros2 run guidance pose_check --ros-args -p use_sim_time:=true &
PID_CHK=$!

echo "Starting mission.launch.py with args: $@..."
ros2 launch bringup mission.launch.py "$@" &
PID_MIS=$!

trap "kill -TERM $PID_CHK $PID_MIS 2>/dev/null || true" SIGINT SIGTERM EXIT

wait
