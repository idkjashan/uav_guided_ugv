# uav_guided_ugv

Colcon workspace for the DRDO Inter-IIT "UAV-guided UGV" problem statement.  
ROS 2 Humble · Gazebo Harmonic · PX4 Autopilot v1.16.

The UGV carries no sensors. The UAV surveys the road from the air with a downward-facing depth camera, builds a 2.5-D traversability costmap, and guides the UGV along the traversable route.

```
uav_guided_ugv/
├── src/
│   ├── bringup/                # System bringup, orchestration launch files, URDFs, mission nodes
│   ├── road_survey/            # Stage 1: Road detection, 2.5-D elevation accumulation, costmap
│   ├── drdo_gz_worlds/         # DRDO Gazebo Harmonic worlds, mesh terrains, UAV bridge & models
│   └── ackermann_gz_bringup/   # UGV Ackermann rover model, SDF/URDF, and ROS-GZ bridges
├── docs/
│   ├── 01_road_survey_plan.md  # How road detection works and theoretical rationale
│   ├── HANDOFF.md              # Integration notes and cross-package workflows
│   └── SYSTEM_ENVIRONMENT.md   # Exact machine specifications, versions, and topics
└── maps/                       # Exported costmaps (.npz, .pgm, .yaml, .png)
```

## Packages Overview

1. **`bringup`**:
   - Master orchestration package.
   - Contains launch files:
     - `sim.launch.py`: launches Gazebo world, spawns Ackermann rover, publishes rover URDF TF, starts MicroXRCEAgent, launches PX4 SITL UAV, and starts sensor bridges.
     - `survey.launch.py`: launches `road_survey`'s `terrain_mapper_node` and RViz.
     - `survey_mission.launch.py`: autonomous UAV offboard takeoff and survey flight node.
     - `full_system.launch.py`: 1-command complete system bringup.
     - `spawn_rover.launch.py`: spawns Ackermann rover with URDF and `robot_state_publisher`.
     - `rviz.launch.py`: RViz visualization.
   - Includes standalone URDFs for both the Ackermann rover (`ackermann_bot.urdf`) and quadrotor UAV (`x500_depth_down.urdf`).

2. **`road_survey`**:
   - Stage 1 road survey and traversability costmap generator.
   - Accumulates downward depth returns into a 2.5-D elevation grid.
   - Evaluates geometry-based risk (slope, step height, surface roughness).
   - Generates and publishes `/road/costmap` (`nav_msgs/OccupancyGrid`) for Nav2 planning.
   - Exports lossless `.npz`, Nav2 map server `.pgm`/`.yaml`, and visualization `.png`.

3. **`drdo_gz_worlds`**:
   - High-fidelity Gazebo Harmonic simulation environments (`drdo_world1`, `drdo_world2`, `drdo_world3`).
   - Downward-facing OakD-Lite depth/RGB camera UAV model (`x500_depth_down`).
   - ROS-Gazebo bridge for UAV sensor feeds (`/uav/depth`, `/uav/camera_info`, `/uav/rgb`).

4. **`ackermann_gz_bringup`**:
   - 4-wheel Ackermann-steered / skid-steer UGV rover model (`ackermann_bot`) equipped with roof-mounted ArUco marker (DICT_4X4_50, ID 0).
   - URDF and SDF models, ROS-GZ bridges for `/cmd_vel`, `/odom`, `/joint_states`, and `/tf`.

---

## Build & Test

```bash
cd ~/uav_guided_ugv
source /opt/ros/humble/setup.bash
source ~/px4_ros_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run unit tests:
```bash
PYTHONPATH=src/road_survey python3 -m pytest src/road_survey/test -q
```
*(47 passed in ~2s, including synthetic terrain surveys and geometric validation)*.

---

## Running the System

### 1. Unified Simulation Bringup
```bash
ros2 launch bringup sim.launch.py world:=drdo_world2 gui:=true
```
Supported worlds: `drdo_world1`, `drdo_world2`, `drdo_world3`.

### 2. Start Road Survey Mapping & RViz
```bash
ros2 launch bringup survey.launch.py world:=drdo_world2
```

### 3. Fly the UAV Autonomous Survey Mission
```bash
ros2 run bringup survey_mission
# Or via launch file:
ros2 launch bringup survey_mission.launch.py altitude:=12.0 forward_dist:=18.0
```

### 4. Or Launch Everything in One Command
```bash
ros2 launch bringup full_system.launch.py world:=drdo_world2
```

### 5. Manual Map Save & Replay
Trigger map save on demand:
```bash
ros2 service call /terrain_mapper/save std_srvs/srv/Trigger
```
Replay saved survey map for the UGV navigation stage:
```bash
ros2 run road_survey map_publisher --ros-args -p map_npz:=~/uav_guided_ugv/maps/road_map.npz -p use_sim_time:=true
```
Sweep classification thresholds offline:
```bash
ros2 run road_survey tune_offline ~/uav_guided_ugv/maps/road_map.npz --sweep slope 8 10 12 15 20
```
