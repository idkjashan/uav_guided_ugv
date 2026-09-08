#!/usr/bin/env bash
# Record one survey flight so it can be replayed against different thresholds
# without re-flying.  Everything the mapper consumes, nothing else.
#
#   ./scripts/record_survey.sh world1
#   ros2 bag play ~/uav_guided_ugv/bags/world1 --clock   # then run the mapper
set -euo pipefail

NAME="${1:-survey}"
OUT="${HOME}/uav_guided_ugv/bags/${NAME}"
mkdir -p "$(dirname "${OUT}")"

echo "recording to ${OUT} -- Ctrl-C to stop"
exec ros2 bag record -o "${OUT}" \
  /uav/depth \
  /uav/depth_camera_info \
  /fmu/out/vehicle_local_position \
  /fmu/out/vehicle_attitude \
  /clock
