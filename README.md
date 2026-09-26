# uav_guided_ugv

Colcon workspace for the DRDO Inter-IIT "UAV-guided UGV" problem statement.
ROS 2 Humble, Gazebo Harmonic, PX4 Autopilot v1.16.

The UGV carries no sensors and uses no wheel odometry. The UAV maps the road
from the air with a downward depth camera, flies back to the UGV, localises
it from the ArUco marker on its roof and holds station above it while the
UGV drives the road centre line to the end of the road.

```
TAKEOFF -> SURVEY -> RETURN -> ACQUIRE -> TRACK -> DONE
           map the    retrace   find the   UGV drives the centre line,
           road       the way   marker     UAV stays 10 m above it
```

How the pieces fit: [docs/ARCHITECTURE_AND_APPROACH.md](docs/ARCHITECTURE_AND_APPROACH.md).

```
uav_guided_ugv/
├── run_sim.sh, run_mission.sh, test_phase1.sh   # launch helpers
├── src/
│   ├── road_survey/            # stage 1: depth -> 2.5-D elevation -> road costmap
│   ├── guidance/               # stage 2: UAV mission, ArUco UGV pose, UGV path following
│   ├── bringup/                # launch files and world spawn poses
│   ├── drdo_gz_worlds/         # DRDO Gazebo worlds, x500_depth_down UAV model, camera bridge
│   └── ackermann_gz_bringup/   # skid-steer UGV model with its roof marker
├── docs/
│   ├── ARCHITECTURE_AND_APPROACH.md   # nodes, topics, states, frames, data
│   ├── 01_road_survey_plan.md         # how the road is detected, and why
│   ├── 02_guidance.md                 # survey, return, localise and guide, and why
│   ├── HANDOFF.md                     # bring-up checklist and troubleshooting
│   └── SYSTEM_ENVIRONMENT.md          # machine, versions, frames
└── maps/                              # survey output (.npz, .pgm, .yaml, .png), not tracked
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
flies the whole mission against a toy world. The centre line needs
scikit-image (`pip install "scikit-image<0.25"`).

First time on a machine, install the UAV model and airframe into PX4 with
`src/bringup/scripts/setup_px4_uav.sh`.

## Run

```bash
./run_sim.sh                     # terminal 1; add a world: ./run_sim.sh drdo_world1
./run_mission.sh                 # terminal 2, once PX4 prints "Ready for takeoff!"
./run_mission.sh survey:=false   # reuse maps/road_map.npz instead of surveying again
./test_phase1.sh                 # UAV and localiser only, UGV parked: checks the marker pose
```

`run_mission.sh` also runs `pose_check`, which compares the ArUco pose with
Gazebo's ground truth and, while the UGV drives, prints how far the true UGV
is from the centre line:

```
centre-line deviation over <L> m driven: median <d> m, p95 <d> m, max <d> m
<n> poses: xy error median <e> m, p95 <e> m, max <e> m; mean heading error <h> deg (...)
```

Without the helpers:

```bash
ros2 launch bringup sim.launch.py world:=drdo_world2
ros2 launch bringup mission.launch.py [survey:=false]
ros2 run guidance pose_check --ros-args -p use_sim_time:=true
```

## Tools

```bash
ros2 service call /terrain_mapper/save std_srvs/srv/Trigger          # save the map now
ros2 run road_survey tune_offline ~/uav_guided_ugv/maps/road_map.npz --sweep slope 8 10 12 15 20
```

## Notes

- PX4 keeps parameters between runs. If the same PX4 tree has been used with
  GPS fusion turned off (for example for GPS-denied work), clear its saved
  parameters before flying this: `rm -f ~/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters*.bson ~/PX4-Autopilot/build/px4_sitl_default/rootfs/eeprom/parameters*`.
- The UGV wheel joints use `<axis><xyz>0 0 -1</xyz></axis>` because the wheel
  links are rolled 90 degrees; with `0 0 1` a forward `/cmd_vel` drives it
  backwards.
- The marker's black square is 0.446 m (`marker_length_m`), not the 0.55 m
  plate size; passing the plate size scales every range by 1.23.
