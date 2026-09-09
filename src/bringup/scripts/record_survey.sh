#!/usr/bin/env bash
# Record one survey flight so it can be replayed against different thresholds
# without re-flying. Everything the mapper consumes, nothing else.
#
#   ./record_survey.sh drdo_world1
#   ros2 bag play ~/uav_guided_ugv/bags/drdo_world1 --clock   # then run the mapper
set -euo pipefail

NAME="${1:-survey}"
OUT="${HOME}/uav_guided_ugv/bags/${NAME}"
mkdir -p "$(dirname "${OUT}")"

echo "recording to ${OUT} -- Ctrl-C to stop"
exec ros2 bag record -o "${OUT}" \
  /uav/depth \
  /uav/camera_info \
  /uav/depth_camera_info \
  /fmu/out/vehicle_local_position \
  /fmu/out/vehicle_local_position_v1 \
  /fmu/out/vehicle_attitude \
  /clock
