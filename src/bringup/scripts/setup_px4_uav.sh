#!/usr/bin/env bash
set -e

# setup_px4_uav.sh
# Automates setting up the x500_depth_down (downward-facing OakD-Lite camera)
# model and airframe inside PX4-Autopilot for DRDO terrain world simulations.

PX4_DIR="${PX4_AUTOPILOT_DIR:-$HOME/PX4-Autopilot}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

echo "============================================================"
echo " Setting up UAV Downward-Camera (x500_depth_down) for PX4   "
echo "============================================================"
echo "Target PX4 directory: $PX4_DIR"

if [ ! -d "$PX4_DIR" ]; then
  echo "ERROR: PX4 directory not found at $PX4_DIR"
  echo "Please set PX4_AUTOPILOT_DIR if installed in a non-default location:"
  echo "  export PX4_AUTOPILOT_DIR=/path/to/PX4-Autopilot"
  exit 1
fi

# 1. Install x500_depth_down model
GZ_MODELS_DIR="$PX4_DIR/Tools/simulation/gz/models"
if [ -d "$GZ_MODELS_DIR" ]; then
  echo "[1/4] Installing x500_depth_down model into $GZ_MODELS_DIR..."
  cp -r "$REPO_DIR/src/drdo_gz_worlds/models/x500_depth_down" "$GZ_MODELS_DIR/"
else
  echo "WARNING: $GZ_MODELS_DIR not found. Skipping model copy."
fi

# 2. Install airframe
AIRFRAMES_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
if [ -d "$AIRFRAMES_DIR" ]; then
  echo "[2/4] Installing 4022_gz_x500_depth_down airframe into $AIRFRAMES_DIR..."
  cp "$REPO_DIR/src/drdo_gz_worlds/px4_airframes/4022_gz_x500_depth_down" "$AIRFRAMES_DIR/"

  # 3. Register in CMakeLists.txt
  CMAKELISTS="$AIRFRAMES_DIR/CMakeLists.txt"
  if [ -f "$CMAKELISTS" ]; then
    if ! grep -q "4022_gz_x500_depth_down" "$CMAKELISTS"; then
      echo "[3/4] Registering 4022_gz_x500_depth_down in $CMAKELISTS..."
      sed -i '/4021_gz_x500_flow/a \\t4022_gz_x500_depth_down' "$CMAKELISTS"
    else
      echo "[3/4] 4022_gz_x500_depth_down already registered in $CMAKELISTS."
    fi
  fi
else
  echo "WARNING: $AIRFRAMES_DIR not found. Skipping airframe copy."
fi

# 4. Build PX4 SITL target
if [ "$1" != "--no-build" ] && [ "$1" != "-n" ]; then
  echo "[4/4] Building PX4 SITL default target..."
  cd "$PX4_DIR"
  make px4_sitl_default
else
  echo "[4/4] Skipping build step (--no-build specified)."
fi

echo "============================================================"
echo " Setup complete! UAV is ready for simulation.               "
echo "============================================================"
