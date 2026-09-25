# uav_guided_ugv

Colcon workspace for the DRDO Inter-IIT "UAV-guided UGV" problem statement.
ROS 2 Humble, Gazebo Harmonic, PX4 Autopilot v1.16.

The UGV carries no sensors. The UAV surveys the road from the air with a
downward depth camera and builds a traversability costmap (stage 1). It then
flies back to the UGV, localises it from the ArUco marker on its roof and
holds station above it while the UGV follows the road centre line to the end
of the road (stage 2).

```
uav_guided_ugv/
├── src/
│   ├── road_survey/            # stage 1: depth -> 2.5-D elevation -> road costmap
│   ├── guidance/               # stage 2: UAV mission, ArUco UGV pose, UGV path following
│   ├── bringup/                # launch files and world spawn poses
│   ├── drdo_gz_worlds/         # DRDO Gazebo worlds, x500_depth_down UAV model, camera bridge
│   └── ackermann_gz_bringup/   # skid-steer UGV model with its roof marker
├── docs/
│   ├── 01_road_survey_plan.md  # how the road is detected, and why
│   ├── 02_guidance.md          # survey, return, localise and guide the UGV
│   ├── HANDOFF.md              # bring-up checklist for the simulation machine
│   └── SYSTEM_ENVIRONMENT.md   # machine, versions, frames
└── maps/                       # survey output (.npz, .pgm, .yaml, .png), not tracked
```

## Build and test

```bash
cd ~/uav_guided_ugv
source /opt/ros/humble/setup.bash
source ~/px4_ros_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash

PYTHONPATH=src/road_survey:src/guidance python3 -m pytest src/road_survey/test src/guidance/test -q
```

The tests need no ROS or simulator. `src/guidance/test/test_mission_sim.py`
flies the whole mission (survey, return, lock-on, guided drive) against a toy
world. The centre line needs scikit-image (`pip install "scikit-image<0.25"`).

## Run

```bash
# terminal 1
ros2 launch bringup sim.launch.py world:=drdo_world2

# terminal 2, once PX4 prints "Ready for takeoff!"
ros2 launch bringup mission.launch.py                 # survey, then guide the UGV
ros2 launch bringup mission.launch.py survey:=false   # reuse maps/road_map.npz
```

First time on a machine, install the UAV model and airframe into PX4 with
`src/bringup/scripts/setup_px4_uav.sh`.

## Tools

```bash
ros2 run guidance pose_check --ros-args -p use_sim_time:=true      # ArUco pose vs Gazebo truth
ros2 service call /terrain_mapper/save std_srvs/srv/Trigger          # save the map now
ros2 run road_survey tune_offline ~/uav_guided_ugv/maps/road_map.npz --sweep slope 8 10 12 15 20
```
