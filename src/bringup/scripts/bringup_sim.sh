#!/usr/bin/env bash
#
# bringup_sim.sh: Automated bringup for Gazebo world, UGV (Ackermann rover),
# PX4 SITL UAV (downward-facing camera), MicroXRCEAgent, and ROS-GZ bridges.
#
# Usage:
#   ./bringup_sim.sh [world_name] [uav_model]
#
# Examples:
#   ./bringup_sim.sh drdo_world2
#   ./bringup_sim.sh drdo_world1
#   ./bringup_sim.sh drdo_world3_overlay
#

WORLD=${1:-drdo_world2}
UAV_MODEL=${2:-gz_x500_depth_down}
if [ -n "$PX4_AUTOPILOT_DIR" ]; then
  PX4_DIR="$PX4_AUTOPILOT_DIR"
elif [ -d "/home/jashan/PX4-Autopilot" ]; then
  PX4_DIR="/home/jashan/PX4-Autopilot"
elif [ -d "$HOME/PX4-Autopilot" ]; then
  PX4_DIR="$HOME/PX4-Autopilot"
fi
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
LOG_DIR="$REPO_DIR/log/sim_bringup"
mkdir -p "$LOG_DIR"

# Enable discrete GPU acceleration if available
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only

# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
if [ -f "$REPO_DIR/install/setup.bash" ]; then
  source "$REPO_DIR/install/setup.bash"
fi
if [ -f "$HOME/px4_ros_ws/install/setup.bash" ]; then
  source "$HOME/px4_ros_ws/install/setup.bash"
fi

# Resource paths for Gazebo Harmonic
export GZ_PARTITION="${GZ_PARTITION:-uav_ugv}"
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:$REPO_DIR/install/drdo_gz_worlds/share/drdo_gz_worlds/models:$REPO_DIR/install/ackermann_gz_bringup/share/ackermann_gz_bringup/models:$REPO_DIR/install/bringup/share/bringup/urdf:${GZ_SIM_RESOURCE_PATH:-}"

# World-specific terrain spawn offsets
case ${WORLD%_overlay} in
  drdo_world1)
    UAV_POSE="-10.226,311.831,22.863,0.011338,0.135709,-2.161422"
    UGV_ARGS="x:=-12.220319 y:=308.976703 z:=22.295580 roll:=0.011338 pitch:=0.135709 yaw:=-2.161422"
    ;;
  drdo_world2)
    UAV_POSE="103.776917,-101.472992,17.318562,-0.054656,0.032451,2.460081"
    UGV_ARGS="x:=104.742386 y:=-101.9010777 z:=15.730011 roll:=-0.054656 pitch:=0.032451 yaw:=2.460081"
    ;;
  drdo_world3)
    UAV_POSE="108.849,-265.663,49.4752,0.045161,0.003268,1.588"
    UGV_ARGS="x:=109.076 y:=-262.736 z:=49.2026 roll:=0.005846 pitch:=-0.047033 yaw:=1.56611"
    ;;
  *)
    echo "ERROR: Unknown world '$WORLD'. Supported: drdo_world1, drdo_world1_overlay, drdo_world2, drdo_world2_overlay, drdo_world3, drdo_world3_overlay"
    exit 1
    ;;
esac

echo "============================================================"
echo " Launching Simulation Bringup: $WORLD"
echo " UAV Model: $UAV_MODEL"
echo " Logs directory: $LOG_DIR"
echo "============================================================"

# Kill previous instances if requested or clean stale processes of this simulation
echo "[0/5] Cleaning up existing simulation processes for $WORLD..."
if [ -f "$LOG_DIR/px4.pid" ]; then
  kill -TERM "$(cat "$LOG_DIR/px4.pid")" 2>/dev/null || true
  rm -f "$LOG_DIR/px4.pid"
fi
pkill -f "world.launch.py.*world:=$WORLD" 2>/dev/null || true
pkill -f "spawn_ackermann.launch.py.*world:=$WORLD" 2>/dev/null || true
pkill -f "uav_bridge.launch.py.*world:=$WORLD" 2>/dev/null || true
pkill -f "PX4_GZ_WORLD=$WORLD" 2>/dev/null || true
if [ -f /tmp/px4_lock-0 ] && ! fuser /tmp/px4_lock-0 >/dev/null 2>&1; then
  rm -f /tmp/px4_lock-0
fi
sleep 1

# 1. Launch Gazebo Harmonic world
echo "[1/5] Launching Gazebo world ($WORLD)..."
nohup setsid ros2 launch drdo_gz_worlds world.launch.py world:="$WORLD" gui:="${GUI:-true}" </dev/null > "$LOG_DIR/gz.log" 2>&1 &
disown
sleep 12

# 2. Spawn UGV (Ackermann rover with ArUco marker)
echo "[2/5] Spawning UGV (ackermann_bot) at terrain offset..."
nohup setsid ros2 launch ackermann_gz_bringup spawn_ackermann.launch.py world:="$WORLD" $UGV_ARGS </dev/null > "$LOG_DIR/rover.log" 2>&1 &
disown
sleep 4

# 3. Start MicroXRCEAgent (for PX4 ROS 2 communication)
if command -v MicroXRCEAgent &> /dev/null || [ -f "$HOME/px4_ros_ws/install/microxrcedds_agent/bin/MicroXRCEAgent" ]; then
  if ! pgrep -f "MicroXRCEAgent" > /dev/null; then
    echo "[3/5] Starting Micro-XRCE-DDS-Agent..."
    nohup setsid MicroXRCEAgent udp4 -p 8888 </dev/null > "$LOG_DIR/xrce_agent.log" 2>&1 &
    disown
    sleep 2
  else
    echo "[3/5] MicroXRCEAgent is already running."
  fi
else
  echo "[3/5] MicroXRCEAgent not found; skipping DDS agent."
fi

# 4. Launch PX4 SITL UAV (standalone spawn into running Gazebo)
if [ -d "$PX4_DIR" ]; then
  echo "[4/5] Launching PX4 SITL UAV ($UAV_MODEL) at terrain offset..."
  (
    cd "$PX4_DIR/build/px4_sitl_default/rootfs"
    PX4_GZ_STANDALONE=1 PX4_SIM_MODEL="$UAV_MODEL" PX4_GZ_WORLD="$WORLD" PX4_GZ_MODEL_POSE="$UAV_POSE" \
      nohup setsid ../bin/px4 -d </dev/null > "$LOG_DIR/px4.log" 2>&1 &
    echo $! > "$LOG_DIR/px4.pid"
    disown
  )
  sleep 12
else
  echo "[4/5] PX4-Autopilot directory not found at $PX4_DIR; skipping PX4 SITL."
fi

# 5. Launch UAV downward camera ROS-Gazebo bridge
echo "[5/5] Launching UAV camera bridge (/uav/rgb, /uav/depth, /uav/camera_info)..."
nohup setsid ros2 launch drdo_gz_worlds uav_bridge.launch.py world:="$WORLD" </dev/null > "$LOG_DIR/uav_bridge.log" 2>&1 &
disown

echo "============================================================"
echo " Simulation Bringup Complete!"
echo " ROS Topics ready:"
echo "   - UGV Control:      /cmd_vel"
echo "   - UGV Odometry:     /odom"
echo "   - UAV Downward RGB: /uav/rgb"
echo "   - UAV Depth:        /uav/depth"
echo "   - UAV CameraInfo:   /uav/camera_info"
echo " View logs with: tail -f $LOG_DIR/*.log"
echo "============================================================"
