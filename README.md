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

Technical specification & architecture: [docs/ARCHITECTURE_AND_APPROACH.md](docs/ARCHITECTURE_AND_APPROACH.md).

```
uav_guided_ugv/
├── src/
│   ├── road_survey/            # stage 1: depth -> 2.5-D elevation -> road costmap & 3D cloud
│   ├── guidance/               # stage 2: interactive mission, ArUco UGV pose, UGV path follower
│   ├── bringup/                # sim.launch.py, mission.launch.py, rviz.launch.py, world configs
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

## ROS Domain ID Setup

The workspace is configured with a unified global ROS Domain ID (`42`) across all components (ROS 2 CLI, PX4 SITL, MicroXRCEAgent, Gazebo bridges, RViz, and mission nodes):

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

*(This is exported in `~/.bashrc` and passed automatically to PX4 SITL and MicroXRCEAgent).*

## Build and test

```bash
cd ~/uav_guided_ugv
source /opt/ros/humble/setup.bash
source ~/px4_ros_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash

PYTHONPATH=src/road_survey:src/guidance python3 -m pytest src/road_survey/test src/guidance/test -q
```

The tests need no ROS or simulator (68 tests passing). `src/guidance/test/test_mission_sim.py`
flies the whole mission against a toy world. The centre line needs
scikit-image (`pip install "scikit-image<0.25"`).

First time on a machine, install the UAV model and airframe into PX4 with
`src/bringup/scripts/setup_px4_uav.sh`.

## Running the System

Launch the system in two steps: first start the simulation, then start the interactive mission.

### Step 1: Launch Simulation
In **Terminal 1**:

```bash
source /opt/ros/humble/setup.bash
source ~/px4_ros_ws/install/setup.bash
source ~/uav_guided_ugv/install/setup.bash

ros2 launch bringup sim.launch.py world:=drdo_world2
```
*(Optionally select another world: `world:=drdo_world1` or `world:=drdo_world3`).*

Wait for Gazebo to load and PX4 to print:
```text
INFO  [commander] Ready for takeoff!
```

### Step 2: Launch Interactive Mission Node
In **Terminal 2**:

```bash
source /opt/ros/humble/setup.bash
source ~/px4_ros_ws/install/setup.bash
source ~/uav_guided_ugv/install/setup.bash

ros2 launch bringup mission.launch.py
```

### Interactive Mission Workflow

When launched, the mission node provides an interactive terminal menu:

1. **Select Mission Mode:**
   - **`[1] Survey`**: Map the road corridor from the air.
   - **`[2] Guidance`**: Directly escort the UGV using an existing saved map (`maps/road_map.npz`).

2. **Select Survey Method (if Survey chosen):**
   - **`[1] Autonomous Survey`**: UAV takes off, follows road corridor, accumulates 2.5D elevation & costmap, logs breadcrumbs, and automatically detects when the road ends.
   - **`[2] Manual Survey`**: Pilot the UAV manually (via QGroundControl, joystick, or RC transmitter) while the terrain mapper builds the costmap live.

3. **Ending Survey Early & Saving Map Status:**
   - During autonomous or manual survey, press `[Enter]` or type `end` / `save` in the terminal at **any time** to terminate the survey immediately.
   - The current map status is saved to `maps/road_map.npz`, `maps/road_map.png`, `.yaml`, and `.pgm`.
   - The prompt will ask:
     ```text
     >>> Proceed to Guidance system now? [Y/n] (default: Y):
     ```
   - On confirmation, the UAV retraces its breadcrumbs back to the spawn point, descends to 10 m, acquires the UGV roof marker, and guides the UGV along the centre line to the end of the road.

4. **Real-time Pose Accuracy & Centre-line Deviation (`pose_check`):**
   - `pose_check` runs automatically in `mission.launch.py` and reports real-time statistics:
     ```text
     centre-line deviation over <L> m driven: median <d> m, p95 <d> m, max <d> m
     <n> poses: xy error median <e> m, p95 <e> m, max <e> m; mean heading error <h> deg (...)
     ```

### Separate RViz Visualization

RViz2 opens automatically with `mission.launch.py`. You can also launch or re-open RViz separately at any time in its own window:

```bash
source /opt/ros/humble/setup.bash
source ~/uav_guided_ugv/install/setup.bash

ros2 launch bringup rviz.launch.py
```

Pre-configured displays include:
- **UAV Depth Camera (`/uav/depth`)**: Configured with `Best Effort` QoS and 0–20 m normalized dynamic range.
- **UAV Downward RGB Camera (`/uav/rgb`)**: Stream from down-facing IMX214 sensor.
- **3D Terrain PointCloud (`/terrain/pointcloud`)**: Real-time 3D ground point cloud accumulated under the UAV in `map` frame.
- **Road Costmap (`/road/costmap`)**: Occupancy grid (driveable road shelf vs lethal drop/slope).
- **UGV Centre Line & Path (`/ugv/path`)**: Pure pursuit reference trajectory.
- **UGV Pose (ArUco) (`/ugv/pose`)**: Estimated rover pose from camera PnP.
- **UGV Ground Truth (`/ugv/ground_truth`)**: Validation pose from Gazebo.
- **TF Tree**: Transforms between `world`, `map`, and `uav_base_link`.

## Tools

```bash
ros2 service call /terrain_mapper/save std_srvs/srv/Trigger          # Save map manually
ros2 service call /terrain_mapper/load std_srvs/srv/Trigger          # Reload map from maps/road_map.npz
ros2 run road_survey tune_offline ~/uav_guided_ugv/maps/road_map.npz --sweep slope 8 10 12 15 20
```

## Implementation Notes

- **PX4 Rootfs Cache:** PX4 caches parameters between runs. If the airframe parameters were modified or GPS disabled in earlier setups, clear its parameter cache before launching:
  ```bash
  rm -f ~/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters*.bson ~/PX4-Autopilot/build/px4_sitl_default/rootfs/eeprom/parameters*
  ```
- **UGV Wheel Joint Axes:** Child wheel links have roll $+90^\circ$; joint rotation axes are configured as `<axis><xyz>0 0 -1</xyz></axis>` in `model.sdf` to ensure standard forward `linear.x` and counter-clockwise `angular.z`.
- **Roof Marker Sizing:** The ArUco marker black square is 0.446 m (`marker_length_m`), not the 0.55 m plate size.
- **Sensor Range Limit:** Downward depth camera clips at 19.1 m; survey flight altitude is maintained at 12 m AGL with terrain distance feedback.
