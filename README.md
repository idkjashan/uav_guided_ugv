# UAV-Guided UGV (DRDO Inter-IIT Tech Meet)

Autonomous aerial mapping, visual localization, and lead-escort path tracking for a sensorless ground rover using a quadrotor UAV.

Developed for **ROS 2 Humble**, **Gazebo Harmonic**, and **PX4 Autopilot v1.16**.

---

## 1. System Overview & Approach

In this operational scenario, the **Unmanned Ground Vehicle (UGV)** operates in unknown, GPS-denied or sensor-constrained mountain terrain. The UGV is completely blind: it carries **no onboard perception sensors** (no LiDAR, cameras, or radar) and uses **no wheel odometry**. It carries only an ArUco fiducial marker on its roof.

The **Unmanned Aerial Vehicle (UAV)** serves as the "eye in the sky", equipped with:
- A downward-facing Intel RealSense D435 depth camera (clipped at 19.1 m)
- An IMX214 high-resolution RGB camera
- PX4 Autopilot running in Offboard mode

### Operational Stages

```
                                  [ START ]
                                      │
                                      ▼
             ┌─────────────────────────────────────────────────┐
             │       STAGE 1: AUTONOMOUS ROAD SURVEY           │
             │ - UAV climbs to 12 m AGL                        │
             │ - Frontier exploration along the road corridor  │
             │ - Downward depth -> 2.5D elevation grid         │
             │ - Slope / Step / Roughness traversability risk  │
             │ - Medial-axis centreline extraction & smoothing │
             │ - 3D terrain breadcrumb logging                 │
             │ - Auto-saves map when road end is reached       │
             └────────────────────────┬────────────────────────┘
                                      │
                                      ▼
             ┌─────────────────────────────────────────────────┐
             │       STAGE 2: RETURN & ARUCO ACQUISITION       │
             │ - Retraces 3D breadcrumbs back to spawn point   │
             │ - Descends safely to guidance altitude (10 m)   │
             │ - Detects 40 cm roof ArUco marker (ID 0)        │
             │ - Computes camera-to-marker PnP pose            │
             │ - Transforms into global map frame at 30 Hz     │
             └────────────────────────┬────────────────────────┘
                                      │
                                      ▼
             ┌─────────────────────────────────────────────────┐
             │       STAGE 3: GUIDANCE & PURE PURSUIT ESCORT   │
             │ - UGV Pure Pursuit controller tracks centreline │
             │ - Dynamic UAV station-keeping above moving UGV  │
             │ - Failsafe zero-velocity timeout on marker loss │
             │ - Reaches goal at end of road with cm accuracy  │
             └─────────────────────────────────────────────────┘
```

For the detailed mathematical formulation, coordinate frames, and algorithms, see [docs/ARCHITECTURE_AND_APPROACH.md](file:///home/jashan/uav_guided_ugv/docs/ARCHITECTURE_AND_APPROACH.md).

---

## 2. Workspace Directory & File Manifest

```
uav_guided_ugv/
├── run_sim.sh                  # Helper script to launch Gazebo, PX4 SITL, and bridges
├── run_mission.sh              # Helper script to launch pose_check and mission stack
├── test_phase1.sh              # Helper script to verify ArUco localization & station keeping
├── maps/                       # Generated costmaps and centrelines (.npz, .png, .pgm, .yaml)
├── docs/                       # Technical specifications, environment, and checklists
└── src/
    ├── road_survey/            # Stage 1: Depth pointcloud to 2.5D elevation & costmap
    ├── guidance/               # Stage 2: Mission FSM, ArUco PnP localizer, Pure Pursuit follower
    ├── bringup/                # Top-level launch files, world spawn configs, PX4 setup
    ├── drdo_gz_worlds/         # Gazebo Harmonic worlds, x500_depth_down model, airframe
    └── ackermann_gz_bringup/   # 4-wheel UGV model with roof ArUco marker and diff-drive
```

### Detailed File Responsibilities

#### Root Helper Scripts
- [`run_sim.sh`](file:///home/jashan/uav_guided_ugv/run_sim.sh): Sources workspaces and launches the Gazebo simulation environment (`drdo_world2` by default).
- [`run_mission.sh`](file:///home/jashan/uav_guided_ugv/run_mission.sh): Launches the entire mission pipeline (`bringup/mission.launch.py`) along with the realtime ground-truth pose validation tool (`pose_check`). Accepts arguments like `survey:=false`.
- [`test_phase1.sh`](file:///home/jashan/uav_guided_ugv/test_phase1.sh): Runs Phase 1 testing (UAV takeoff, hovering over stationary UGV, ArUco localization accuracy check) without driving the UGV.

#### `src/road_survey/` (Stage 1: Mapping & Terrain Analysis)
- [`road_survey/depth.py`](file:///home/jashan/uav_guided_ugv/src/road_survey/road_survey/depth.py): Reprojects 2D depth images into 3D camera-frame pointclouds using camera intrinsics.
- [`road_survey/frames.py`](file:///home/jashan/uav_guided_ugv/src/road_survey/road_survey/frames.py): Mathematical transformation utilities connecting Camera Optical, Camera Link, UAV Body (FRD), PX4 Local (NED), and ROS Map (ENU) frames.
- [`road_survey/grid.py`](file:///home/jashan/uav_guided_ugv/src/road_survey/road_survey/grid.py): Fixed-resolution 2.5D elevation grid data structure ($0.25\text{ m/cell}$) that maintains running elevation statistics ($z_{\min}, z_{\max}, \bar{z}, N$).
- [`road_survey/risk.py`](file:///home/jashan/uav_guided_ugv/src/road_survey/road_survey/risk.py): Terrain feature extraction: computes slope angle, step height, and roughness about the locally fitted plane via separable box filters, returning the normalized traversability risk.
- [`road_survey/costmap.py`](file:///home/jashan/uav_guided_ugv/src/road_survey/road_survey/costmap.py): Converts continuous terrain risk into ROS standard `nav_msgs/OccupancyGrid` messages (cost 0 = road centreline, 100 = lethal terrain).
- [`road_survey/centerline.py`](file:///home/jashan/uav_guided_ugv/src/road_survey/road_survey/centerline.py): Extracts the topological road centreline via morphological closing, medial axis skeletonization, spline smoothing ($s = 0.01N$), and corner fork pruning.
- [`road_survey/terrain_mapper_node.py`](file:///home/jashan/uav_guided_ugv/src/road_survey/road_survey/terrain_mapper_node.py): ROS 2 node subscribing to `/uav/depth` and PX4 `VehicleOdometry`, dynamically integrating depth frames and exposing the `/terrain_mapper/save` service.
- [`road_survey/map_publisher_node.py`](file:///home/jashan/uav_guided_ugv/src/road_survey/road_survey/map_publisher_node.py): ROS 2 node used when `survey:=false`, loading previously saved maps from `maps/road_map.npz` and republishing `/road/costmap` and `/road/centerline`.
- [`road_survey/mapio.py`](file:///home/jashan/uav_guided_ugv/src/road_survey/road_survey/mapio.py): Serialization utility saving costmaps to `.npz`, `.pgm`, `.yaml`, and human-viewable `.png`.
- [`road_survey/tune_offline.py`](file:///home/jashan/uav_guided_ugv/src/road_survey/road_survey/tune_offline.py): Standalone CLI tool to evaluate and tune slope, step, and roughness thresholds offline from recorded `.npz` data without flying.
- [`road_survey/height_slicer_node.py`](file:///home/jashan/uav_guided_ugv/src/road_survey/road_survey/height_slicer_node.py): Utility for multi-tier mountain switchback roads.

#### `src/guidance/` (Stage 2: Guidance, Localization & Control)
- [`guidance/px4.py`](file:///home/jashan/uav_guided_ugv/src/guidance/guidance/px4.py): PX4 Offboard protocol interface: manages arming, mode switching, Offboard heartbeat, coordinate conversions (ENU $\leftrightarrow$ NED), and trajectory setpoint publishing.
- [`guidance/explore.py`](file:///home/jashan/uav_guided_ugv/src/guidance/guidance/explore.py): Frontier road exploration logic: samples a 5 m circular forward ring on the costmap, clusters driveable road cells within an $80^\circ$ cone, and determines the next survey waypoint.
- [`guidance/aruco.py`](file:///home/jashan/uav_guided_ugv/src/guidance/guidance/aruco.py): OpenCV ArUco detector (Dictionary `4x4_50`, Marker ID 0) with sub-pixel corner refinement, PnP solver, and camera-to-world pose estimation.
- [`guidance/pursuit.py`](file:///home/jashan/uav_guided_ugv/src/guidance/guidance/pursuit.py): Pure Pursuit path tracking controller with adaptive lookahead distance ($1.5\text{ m}$), in-place rotation for large heading errors ($>45^\circ$), curve speed damping, and goal arrival thresholding.
- [`guidance/mission.py`](file:///home/jashan/uav_guided_ugv/src/guidance/guidance/mission.py): High-level finite-state machine (TAKEOFF $\to$ SURVEY $\to$ RETURN $\to$ ACQUIRE $\to$ TRACK $\to$ GOAL_REACHED) with carrot setpoint filtering and altitude terrain tracking.
- [`guidance/mission_node.py`](file:///home/jashan/uav_guided_ugv/src/guidance/guidance/mission_node.py): ROS 2 node executing the mission state machine and streaming trajectory setpoints to PX4.
- [`guidance/localizer_node.py`](file:///home/jashan/uav_guided_ugv/src/guidance/guidance/localizer_node.py): ROS 2 node subscribing to `/uav/rgb`, `/uav/camera_info`, and vehicle odometry, publishing estimated `/ugv/pose` in the world `map` frame at 30 Hz.
- [`guidance/follower_node.py`](file:///home/jashan/uav_guided_ugv/src/guidance/guidance/follower_node.py): ROS 2 node executing Pure Pursuit path following, subscribing to `/ugv/pose` and `/road/costmap`, and publishing `/cmd_vel` to steer the UGV.
- [`guidance/pose_check_node.py`](file:///home/jashan/uav_guided_ugv/src/guidance/guidance/pose_check_node.py): Realtime validation tool comparing estimated `/ugv/pose` against Gazebo ground-truth `/ugv/ground_truth`, printing position error, heading error, and cross-track error.

#### `src/bringup/` (Launch & Integration)
- [`bringup/launch/sim.launch.py`](file:///home/jashan/uav_guided_ugv/src/bringup/launch/sim.launch.py): Starts Gazebo Harmonic, PX4 SITL daemon, MicroXRCEAgent, static TF (`world` to `map`), spawns UAV and UGV models, and bridges camera/clock topics.
- [`bringup/launch/mission.launch.py`](file:///home/jashan/uav_guided_ugv/src/bringup/launch/mission.launch.py): Launches the full mapping, localization, guidance, and follower stack along with RViz2. Supports `survey:=false` to reuse previously saved maps.
- [`bringup/config/world_poses.yaml`](file:///home/jashan/uav_guided_ugv/src/bringup/config/world_poses.yaml): Spawn poses ($x, y, z, \text{yaw}$) for UAV and UGV across competition worlds (`drdo_world1`, `drdo_world2`, `drdo_world3`).
- [`bringup/scripts/setup_px4_uav.sh`](file:///home/jashan/uav_guided_ugv/src/bringup/scripts/setup_px4_uav.sh): Setup script to install the custom `x500_depth_down` model and airframe into PX4 Autopilot.

#### `src/drdo_gz_worlds/` (Environments & UAV Model)
- `worlds/drdo_world{1,2,3}.sdf`: High-fidelity simulation worlds featuring winding mountain roads, slopes, ditches, and obstacles.
- `models/x500_depth_down/`: Custom quadrotor model with downward Intel RealSense D435 depth camera and IMX214 RGB camera.
- `px4_airframes/4022_gz_x500_depth_down`: Custom PX4 airframe definition with standard GPS and EKF2 configuration.

#### `src/ackermann_gz_bringup/` (UGV Model)
- [`models/ackermann_bot/model.sdf`](file:///home/jashan/uav_guided_ugv/src/ackermann_gz_bringup/models/ackermann_bot/model.sdf): 4-wheel skid-steer rover model with roof-mounted 40 cm ArUco marker plate. Joint axes oriented as `<xyz>0 0 -1</xyz>` in rolled wheel child links ensuring standard forward/left ROS kinematics.
- `launch/spawn_ackermann.launch.py`: Spawns the UGV at target coordinates and bridges `/cmd_vel` and ground truth odometry.

#### `docs/` (Documentation)
- [`docs/ARCHITECTURE_AND_APPROACH.md`](file:///home/jashan/uav_guided_ugv/docs/ARCHITECTURE_AND_APPROACH.md): In-depth system architecture, risk combination formulas, PnP chain, and pure pursuit dynamics.
- [`docs/01_road_survey_plan.md`](file:///home/jashan/uav_guided_ugv/docs/01_road_survey_plan.md): Detailed design for Stage 1 slope/step/roughness elevation mapping.
- [`docs/02_guidance.md`](file:///home/jashan/uav_guided_ugv/docs/02_guidance.md): Design spec for Stage 2 Offboard guidance, marker sizing, and pure pursuit.
- [`docs/SYSTEM_ENVIRONMENT.md`](file:///home/jashan/uav_guided_ugv/docs/SYSTEM_ENVIRONMENT.md): Versions, coordinate frame definitions, and sensor specs.
- [`docs/HANDOFF.md`](file:///home/jashan/uav_guided_ugv/docs/HANDOFF.md): Bring-up checklist and validation milestones.

---

## 3. Build & Setup

### Prerequisites
- ROS 2 Humble
- Gazebo Harmonic
- PX4-Autopilot v1.16 built for SITL (`~/PX4-Autopilot`)
- MicroXRCEAgent (`MicroXRCEAgent udp4 -p 8888`)
- Python packages: `scikit-image<0.25`, `scipy`, `numpy`, `opencv-python`

### First-Time Machine Setup
Install the custom UAV model and airframe into PX4:
```bash
~/uav_guided_ugv/src/bringup/scripts/setup_px4_uav.sh
```

### Building the Workspace
```bash
cd ~/uav_guided_ugv
source /opt/ros/humble/setup.bash
source ~/px4_ros_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Running Unit Tests (No Simulator Needed)
All mapping, PnP geometry, pure pursuit, and mission state machine tests run offline in seconds:
```bash
PYTHONPATH=src/road_survey:src/guidance python3 -m pytest src/road_survey/test src/guidance/test -q
```
*(All 66 tests should pass cleanly).*

---

## 4. How to Run the Pipeline

You can run the full system using either the automated **Helper Scripts** (Option A) or directly using **Raw ROS 2 CLI Commands** (Option B).

---

### Option A: Running with Helper Scripts (Recommended)

Three executable scripts are provided in the workspace root:

#### Step 1: Start Simulation
Open **Terminal 1**:
```bash
cd ~/uav_guided_ugv
./run_sim.sh
```
*Note: To specify another world, pass it as an argument: `./run_sim.sh drdo_world1` or `./run_sim.sh drdo_world3`. Default is `drdo_world2`.*

Wait until PX4 prints:
```text
INFO  [commander] Ready for takeoff!
```

#### Step 2: Start Autonomous Mission
Open **Terminal 2**:
```bash
cd ~/uav_guided_ugv
./run_mission.sh
```
*This launches the entire Stage 1 (Survey) + Stage 2 (Return, Localize, Escort) pipeline and streams real-time pose validation statistics via `pose_check`.*

#### Fast Re-run (Skipping Survey):
If you already have a saved map (`maps/road_map.npz`) and want to test guidance and path following directly without re-flying the survey:
```bash
./run_mission.sh survey:=false
```

#### Phase 1 Verification (Station-Keeping & Localization Only):
To test UAV takeoff, hovering above the stationary UGV, and ArUco marker localization without driving the rover:
```bash
./test_phase1.sh
```

---

### Option B: Running Manually Without Helper Scripts (Raw ROS 2 Commands)

If you prefer full control over each terminal and process:

#### Step 1: Launch the Simulator
In **Terminal 1**:
```bash
source /opt/ros/humble/setup.bash
source ~/px4_ros_ws/install/setup.bash
source ~/uav_guided_ugv/install/setup.bash

ros2 launch bringup sim.launch.py world:=drdo_world2
```
Wait for Gazebo to load and PX4 to print `Ready for takeoff!`.

#### Step 2: Launch the Mission
In **Terminal 2**:
```bash
source /opt/ros/humble/setup.bash
source ~/px4_ros_ws/install/setup.bash
source ~/uav_guided_ugv/install/setup.bash

# Full mission (Survey -> Return -> Escort)
ros2 launch bringup mission.launch.py

# OR skip survey and reuse previously saved map
ros2 launch bringup mission.launch.py survey:=false
```

#### Step 3 (Optional): Real-Time Ground-Truth Accuracy Checker
In **Terminal 3**:
```bash
source /opt/ros/humble/setup.bash
source ~/uav_guided_ugv/install/setup.bash

ros2 run guidance pose_check --ros-args -p use_sim_time:=true
```
*Outputs real-time comparison between ArUco estimated `/ugv/pose` and Gazebo ground truth `/ugv/ground_truth`:*
```text
[pose_check]: xy error = 0.072 m (7.2 cm), heading error = 0.1 deg, CTE = 0.021 m
```

---

### Running Individual Nodes Separately (For Step-by-Step Debugging)

If you want to debug individual nodes without using `mission.launch.py`:

```bash
# Common setup for all terminals:
source /opt/ros/humble/setup.bash
source ~/px4_ros_ws/install/setup.bash
source ~/uav_guided_ugv/install/setup.bash
P=$(ros2 pkg prefix guidance)/share/guidance/config/guidance.yaml
R=$(ros2 pkg prefix road_survey)/share/road_survey/config/road_survey.yaml

# Terminal A: Terrain Mapper (Stage 1)
ros2 run road_survey terrain_mapper --ros-args --params-file "$R" -p use_sim_time:=true

# Terminal B: UGV ArUco Localizer (Stage 2)
ros2 run guidance ugv_localizer --ros-args --params-file "$P" -p use_sim_time:=true

# Terminal C: UGV Pure Pursuit Follower (Stage 2)
ros2 run guidance ugv_follower --ros-args --params-file "$P" -p use_sim_time:=true

# Terminal D: Mission State Machine
ros2 run guidance mission --ros-args --params-file "$P" -p use_sim_time:=true -p survey:=true

# Terminal E: Pose Check Diagnostics
ros2 run guidance pose_check --ros-args -p use_sim_time:=true
```

---

## 5. Offline Tools & Diagnostics

### Manually Save the Costmap
If flying manually or triggering an early map save:
```bash
ros2 service call /terrain_mapper/save std_srvs/srv/Trigger
```
Outputs are written to `~/uav_guided_ugv/maps/road_map.npz`, `.png`, `.yaml`, and `.pgm`.

### Offline Parameter Tuning
To evaluate different slope or step thresholds against saved map data without running simulation:
```bash
ros2 run road_survey tune_offline ~/uav_guided_ugv/maps/road_map.npz --sweep slope 8 10 12 15 20
```

---

## 6. Key Implementation & Troubleshooting Notes

1. **UGV Wheel Joint Axes Fix:**
   - In `src/ackermann_gz_bringup/models/ackermann_bot/model.sdf`, wheel joints are configured with `<xyz>0 0 -1</xyz>` to account for the $+90^\circ$ child link roll, ensuring standard forward/left ROS kinematics.
2. **PX4 Airframe & GPS:**
   - Airframe `4022_gz_x500_depth_down` uses standard GPS fusion (`EKF2_GPS_CTRL 7`). If changing airframe settings, clear PX4 rootfs parameters:
     ```bash
     rm -f ~/PX4-Autopilot/build/px4_sitl_default/rootfs/eeprom/parameters*
     ```
3. **Sensor Range & Altitude Management:**
   - The depth camera has an upper far clip of $19.1\text{ m}$. Flying above $18\text{ m}$ results in empty depth images. The survey state dynamically maintains $12\text{ m}$ Above Ground Level (AGL) based on central depth returns to climb and descend with mountain gradients.
4. **ArUco Marker Size:**
   - Marker plate is $0.55\text{ m}$ with a black square of $0.446\text{ m}$ (effective size $0.40\text{ m}$ used in `guidance.yaml`). This ensures $>7\text{ px/cell}$ at $10\text{ m}$ altitude for reliable single-frame PnP detection.
