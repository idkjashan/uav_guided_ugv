#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /home/jashan/px4_ros_ws/install/setup.bash
source /home/jashan/uav_guided_ugv/install/setup.bash

P=$(ros2 pkg prefix guidance)/share/guidance/config/guidance.yaml

echo "Starting ugv_localizer..."
ros2 run guidance ugv_localizer --ros-args --params-file "$P" -p use_sim_time:=true &
PID_LOC=$!

echo "Starting pose_check..."
ros2 run guidance pose_check --ros-args -p use_sim_time:=true &
PID_CHK=$!

echo "Starting mission (survey:=false)..."
ros2 run guidance mission --ros-args --params-file "$P" -p use_sim_time:=true -p survey:=false &
PID_MIS=$!

trap "kill -TERM $PID_LOC $PID_CHK $PID_MIS 2>/dev/null || true" SIGINT SIGTERM EXIT

wait
