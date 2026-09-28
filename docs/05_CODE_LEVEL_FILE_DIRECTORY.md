# Comprehensive Code-Level File Directory

This document provides an exhaustive, code-level walkthrough of every source file, node, configuration, and launch script across all five packages in `uav_guided_ugv`.

---

# Table of Contents
1. [Package: `drdo_gz_worlds`](#1-package-drdo_gz_worlds)
2. [Package: `ackermann_gz_bringup`](#2-package-ackermann_gz_bringup)
3. [Package: `road_survey`](#3-package-road_survey)
4. [Package: `guidance`](#4-package-guidance)
5. [Package: `bringup`](#5-package-bringup)

---

# 1. Package: `drdo_gz_worlds`

### 1.1 `src/drdo_gz_worlds/launch/world.launch.py`
- **Location:** [`src/drdo_gz_worlds/launch/world.launch.py`](../src/drdo_gz_worlds/launch/world.launch.py)
- **Role:** Launches the Gazebo Harmonic (`gz sim`) simulation server and graphical client loaded with a chosen DRDO terrain environment.
- **Inputs:**
  - Launch Argument `world`: Name of the world SDF to load (`drdo_world1`, `drdo_world2`, `drdo_world3`).
  - Optional vehicle SDF arguments (`uav_sdf`, `ugv_sdf`) and coordinates.
- **Code Logic:**
  - Dynamically locates `drdo_gz_worlds` package share using `ament_index_python`.
  - Configures `GZ_SIM_RESOURCE_PATH` to include the package's internal `models/` directory so that `<uri>model://world1</uri>` and terrain meshes resolve properly.
  - Formulates execution command for `ros_gz_sim`'s `gz_sim.launch.py` with flags `gz_args:="<world_file> -r"`.
- **Outputs:**
  - Active Gazebo Sim engine with rendered terrain meshes, lighting, and simulation clock.

---

### 1.2 `src/drdo_gz_worlds/launch/uav_bridge.launch.py`
- **Location:** [`src/drdo_gz_worlds/launch/uav_bridge.launch.py`](../src/drdo_gz_worlds/launch/uav_bridge.launch.py)
- **Role:** Bridges camera sensor buffers and simulation time from Gazebo Harmonic to ROS 2.
- **Inputs:**
  - Gazebo topic: `/world/{world}/model/{model_name}/link/camera_link/sensor/IMX214/image`
  - Gazebo topic: `/world/{world}/model/{model_name}/link/camera_link/sensor/IMX214/camera_info`
  - Gazebo topic: `/depth_camera`
  - Gazebo topic: `/camera_info`
  - Gazebo topic: `/world/{world}/clock`
- **Code Logic:**
  - Spawns a `ros_gz_bridge::parameter_bridge` Node.
  - Deliberately bridges only the 2D depth image and excludes point clouds to save ~110 MB/s of DDS serialization overhead.
- **Outputs:**
  - `/uav/rgb` (`sensor_msgs/msg/Image`)
  - `/uav/camera_info` (`sensor_msgs/msg/CameraInfo`)
  - `/uav/depth` (`sensor_msgs/msg/Image`, encoding `32FC1`)
  - `/uav/depth_camera_info` (`sensor_msgs/msg/CameraInfo`)
  - `/clock` (`rosgraph_msgs/msg/Clock`)

---

### 1.3 `src/drdo_gz_worlds/urdf/x500_depth_down.urdf`
- **Location:** [`src/drdo_gz_worlds/urdf/x500_depth_down.urdf`](../src/drdo_gz_worlds/urdf/x500_depth_down.urdf)
- **Role:** Pure kinematic and visual description of the quadrotor with its downward OakD-Lite camera for TF and RViz2.
- **Inputs:** None (static URDF XML).
- **Code Logic:**
  - Declares `uav_base_link` and 4 continuous rotor joints.
  - Defines fixed joint `camera_joint` mounting `camera_link` at `xyz="0.12 0.03 0.242"` with `rpy="0 1.57079632679 0"` (nadir downward pitch).
  - Defines fixed joint `camera_optical_joint` mounting `camera_optical_frame` with `rpy="-1.57079632679 0 -1.57079632679"`, aligning with standard CV optical coordinates (+X right, +Y down, +Z along boresight).
- **Outputs:**
  - Consumed by `robot_state_publisher` and RViz2 to construct the complete UAV TF tree.

---

### 1.4 `src/drdo_gz_worlds/models/x500_depth_down/model.sdf`
- **Location:** [`src/drdo_gz_worlds/models/x500_depth_down/model.sdf`](../src/drdo_gz_worlds/models/x500_depth_down/model.sdf)
- **Role:** Gazebo Harmonic simulation model combining the standard PX4 `x500` quadrotor airframe with the `OakD-Lite` sensor module.
- **Inputs:** Gazebo physics ticks.
- **Code Logic:**
  - Includes `<uri>x500</uri>` with `<include merge='true'>`.
  - Includes `<uri>model://OakD-Lite</uri>` with pose `<pose>.12 .03 .242 0 1.5708 0</pose>`.
  - Establishes fixed joint `<joint name="CameraJoint" type="fixed">` attaching `camera_link` to `base_link`.
- **Outputs:**
  - Spawns the physical, flying multirotor entity inside Gazebo Sim.

---

### 1.5 `src/drdo_gz_worlds/px4_airframes/4022_gz_x500_depth_down`
- **Location:** [`src/drdo_gz_worlds/px4_airframes/4022_gz_x500_depth_down`](../src/drdo_gz_worlds/px4_airframes/4022_gz_x500_depth_down)
- **Role:** PX4 POSIX airframe startup configuration script.
- **Inputs:** Evaluated during PX4 SITL boot.
- **Code Logic:**
  - Sets `PX4_SIM_MODEL=x500_depth_down`.
  - Sources base airframe `init.d-posix/airframes/4001_gz_x500`.
  - Disables data link loss action (`param set-default NAV_DLL_ACT 0`) and RC loss failsafe (`COM_RCL_EXCEPT 7`) for autonomous offboard control.
- **Outputs:**
  - Boots PX4 SITL ready to connect to Gazebo with model `x500_depth_down`.

---

# 2. Package: `ackermann_gz_bringup`

### 2.1 `src/ackermann_gz_bringup/launch/spawn_ackermann.launch.py`
- **Location:** [`src/ackermann_gz_bringup/launch/spawn_ackermann.launch.py`](../src/ackermann_gz_bringup/launch/spawn_ackermann.launch.py)
- **Role:** Spawns the Ackermann ground vehicle into an active Gazebo world and sets up communication bridges.
- **Inputs:**
  - Launch arguments: `world`, `robot_name`, `x`, `y`, `z`, `roll`, `pitch`, `yaw`.
- **Code Logic:**
  - Exports `GZ_SIM_RESOURCE_PATH` to include UGV textures (`aruco_marker_0.png`).
  - Calls `ros_gz_sim create` to insert the vehicle SDF at the given spawn coordinates.
  - Launches `ros_gz_bridge parameter_bridge` mapping `/cmd_vel` (ROS $\to$ Gz) and `/ugv/ground_truth` (Gz $\to$ ROS).
- **Outputs:**
  - Spawned UGV entity in Gazebo and active control topic bridges.

---

### 2.2 `src/ackermann_gz_bringup/models/ackermann_bot/model.sdf`
- **Location:** [`src/ackermann_gz_bringup/models/ackermann_bot/model.sdf`](../src/ackermann_gz_bringup/models/ackermann_bot/model.sdf)
- **Role:** Physical simulation model of the 4-wheel Ackermann ground vehicle.
- **Inputs:** Gazebo physics ticks.
- **Code Logic:**
  - Defines 20 kg chassis box ($1.0 \times 0.6 \times 0.25\text{ m}$) with realistic inertia.
  - Maps ArUco DICT_4X4_50 ID 0 texture onto the roof plate.
  - Implements `gz::sim::systems::DiffDrive` plugin configured with $0.7\text{ m}$ track separation and $0.15\text{ m}$ radius wheels, subscribed to `/cmd_vel`.
  - Implements `gz::sim::systems::OdometryPublisher` plugin publishing ground-truth position on `/ugv/ground_truth`.
- **Outputs:**
  - Dynamic vehicle simulation responding to velocity commands and publishing ground-truth poses.

---

### 2.3 `src/ackermann_gz_bringup/urdf/ackermann_bot.urdf`
- **Location:** [`src/ackermann_gz_bringup/urdf/ackermann_bot.urdf`](../src/ackermann_gz_bringup/urdf/ackermann_bot.urdf)
- **Role:** Kinematic model of the UGV for RViz2 and ROS TF.
- **Inputs:** None (static XML).
- **Code Logic:**
  - Models `base_footprint`, `base_link`, 4 wheel links/joints, and `aruco_marker_link` ($Z = +0.1275\text{ m}$ above chassis).
- **Outputs:**
  - Parses into the RViz RobotModel display.

---

# 3. Package: `road_survey`

### 3.1 `src/road_survey/road_survey/depth.py`
- **Location:** [`src/road_survey/road_survey/depth.py`](../src/road_survey/road_survey/depth.py)
- **Role:** Low-level mathematical module for decoding depth buffers and 3D back-projection.
- **Functions:**
  - `decode_image(msg, depth_min, depth_max)`: Converts `32FC1` image bytes into a 2D float32 array via `np.frombuffer().view(np.float32)`. Replaces NaNs, infinities, and out-of-range depths with NaN.
  - `intrinsics_from_hfov(w, h, hfov)`: Derives focal lengths $f_x, f_y$ and principal point $c_x, c_y$ from horizontal FOV.
  - `k_from_camera_info(info)`: Extracts the $3 \times 3$ intrinsic matrix from `sensor_msgs/CameraInfo`.
  - `Unprojector.unproject(depth)`: Applies pinhole inverse equations $X_c = (u - c_x) Z / f_x$, $Y_c = (v - c_y) Z / f_y$, $Z_c = Z$ to produce an $N \times 3$ array of 3D optical vectors.
- **Outputs:**
  - Filtered 2D metric depth arrays and unprojected 3D camera coordinates.

---

### 3.2 `src/road_survey/road_survey/frames.py`
- **Location:** [`src/road_survey/road_survey/frames.py`](../src/road_survey/road_survey/frames.py)
- **Role:** Coordinate transformation math converting between PX4 (NED), ROS (ENU), and OpenCV (Optical).
- **Functions:**
  - `ned_to_enu_position(p)`: Maps $(x, y, z)_{\text{ned}} \to (y, x, -z)_{\text{enu}}$.
  - `ned_to_enu_quaternion(q)`: Transforms attitude quaternions from NED body to ENU body.
  - `optical_to_body_transform()`: Generates the fixed $4 \times 4$ transform matrix converting optical vectors (+Z forward) to vehicle body frame (+X forward, +Y left, +Z up).
  - `camera_extrinsics(cam_pose_in_body)`: Computes the static rigid transform from camera link to UAV body link.
- **Outputs:**
  - $4 \times 4$ homogenous transform matrices and 3D position vectors in ROS standard ENU coordinates.

---

### 3.3 `src/road_survey/road_survey/grid.py`
- **Location:** [`src/road_survey/road_survey/grid.py`](../src/road_survey/road_survey/grid.py)
- **Role:** 2.5-D cumulative elevation map accumulator ([`ElevationGrid`](../src/road_survey/road_survey/grid.py#L20-L100) class).
- **Inputs:** 3D world points $(X_w, Y_w, Z_w)$ in map frame.
- **Code Logic:**
  - Allocates 2D NumPy arrays for mean elevation $\bar{z}$, sample count $N$, minimum elevation $z_{\min}$, and maximum elevation $z_{\max}$ across a $1600 \times 1600$ grid ($400\text{ m} \times 400\text{ m}$ at $0.25\text{ m}$ resolution).
  - Implements `integrate_points(points)`: Discretizes points into row/column indices and performs incremental running updates.
  - Computes multi-level gap metrics to identify overpasses or tiered switchbacks.
- **Outputs:**
  - 2D elevation, sample count, and vertical spread matrices.

---

### 3.4 `src/road_survey/road_survey/risk.py`
- **Location:** [`src/road_survey/road_survey/risk.py`](../src/road_survey/road_survey/risk.py)
- **Role:** Purely geometric terrain hazard classifier.
- **Inputs:** 2D elevation grid and sample count mask.
- **Code Logic:**
  - Fits local least-squares planes $z = p \cdot x + q \cdot y + c$ across a $5 \times 5$ window ($1.25\text{ m}$).
  - Evaluates slope angle $\theta = \arctan\sqrt{p^2 + q^2}$ (critical: $15^\circ$).
  - Evaluates step hazard $\Delta h = \max_{3 \times 3}(z) - \min_{3 \times 3}(z)$ (critical: $0.25\text{ m}$).
  - Evaluates roughness $\sigma = \text{RMS residual}$ (critical: $0.12\text{ m}$).
  - Computes composite risk $R = \max(R_{\text{slope}}, R_{\text{step}}, R_{\text{rough}})$ and thresholding ($R \le 0.35$).
- **Outputs:**
  - Boolean road mask array and continuous metric risk array.

---

### 3.5 `src/road_survey/road_survey/costmap.py`
- **Location:** [`src/road_survey/road_survey/costmap.py`](../src/road_survey/road_survey/costmap.py)
- **Role:** Converts raw geometric classifications into Nav2-compatible costmaps.
- **Inputs:** Binary road mask from `risk.py`.
- **Code Logic:**
  - Applies morphological opening (radius $0.4\text{ m}$) and closing (radius $0.8\text{ m}$) to remove isolated noise and seal cracks.
  - Runs connected component labeling (`scipy.ndimage.label`) to extract the main continuous road corridor ($>25\text{ m}^2$) seeded under vehicle spawn.
  - Applies Exact Euclidean Distance Transform (`scipy.ndimage.distance_transform_edt`) from road edges inwards: assigns cost 0 at centerline, ramping to 99 at edges, and 100/254 for lethal terrain.
- **Outputs:**
  - 2D `int8` costmap array $[0, 100]$.

---

### 3.6 `src/road_survey/road_survey/centerline.py`
- **Location:** [`src/road_survey/road_survey/centerline.py`](../src/road_survey/road_survey/centerline.py)
- **Role:** Extracts a smooth, ordered 2D reference path along the road ribbon.
- **Inputs:** Binary road mask and start position.
- **Code Logic:**
  - Applies medial axis thinning via `skimage.morphology.skeletonize`.
  - Converts skeleton pixels into an adjacency graph, computes the longest shortest-path starting from the vehicle pose, and fits a cubic smoothing spline.
- **Outputs:**
  - Ordered sequence of $(x, y)$ world coordinates spaced at $0.25\text{ m}$.

---

### 3.7 `src/road_survey/road_survey/mapio.py`
- **Location:** [`src/road_survey/road_survey/mapio.py`](../src/road_survey/road_survey/mapio.py)
- **Role:** File serialization and deserialization for surveyed maps.
- **Functions:**
  - `save_npz(path, costmap, elevation, risk, road_mask, metadata)`: Packages all arrays and metadata into a compressed `.npz` archive.
  - `save_nav2(prefix, costmap, resolution, origin)`: Exports standard ROS Nav2 `.pgm` raster image and `.yaml` map metadata.
  - `load_npz(path)`: Loads and validates a saved `.npz` archive.
- **Outputs:**
  - Saved map files on disk.

---

### 3.8 `src/road_survey/road_survey/terrain_mapper_node.py`
- **Location:** [`src/road_survey/road_survey/terrain_mapper_node.py`](../src/road_survey/road_survey/terrain_mapper_node.py)
- **Role:** Active ROS 2 node executing the real-time aerial survey pipeline.
- **Inputs:**
  - `/uav/depth` (`sensor_msgs/msg/Image`)
  - `/uav/depth_camera_info` (`sensor_msgs/msg/CameraInfo`)
  - `/fmu/out/vehicle_local_position` (`px4_msgs/msg/VehicleLocalPosition`)
  - `/fmu/out/vehicle_attitude` (`px4_msgs/msg/VehicleAttitude`)
- **Code Logic:**
  - Filters frames by update rate ($5\text{ Hz}$ throttle), drone bank tilt ($<25^\circ$), and pose timestamp alignment ($<80\text{ ms}$).
  - Back-projects depth frames into 3D world points and integrates into `ElevationGrid`.
  - Runs classification (`risk.py`) and costmap shaping (`costmap.py`) every $2.0\text{ seconds}$.
  - Publishes `/road/costmap` (`nav_msgs/OccupancyGrid`) and `/terrain/pointcloud` (`sensor_msgs/PointCloud2`).
  - Broadcasts TF `map` $\to$ `uav_base_link`.
  - Exposes services `/terrain_mapper/save` and `/terrain_mapper/load` (`std_srvs/srv/Trigger`).
- **Outputs:**
  - `/road/costmap`, `/terrain/pointcloud`, `/tf`, and saved map files.

---

### 3.9 `src/road_survey/road_survey/map_publisher_node.py`
- **Location:** [`src/road_survey/road_survey/map_publisher_node.py`](../src/road_survey/road_survey/map_publisher_node.py)
- **Role:** Publishes an existing pre-surveyed `.npz` costmap without flying the survey.
- **Inputs:**
  - Parameter `map_npz`: Path to `.npz` file (with fallback to `./maps/road_map.npz`).
- **Code Logic:**
  - Loads `.npz` via `mapio.load_npz()`.
  - Converts costmap array to `nav_msgs/msg/OccupancyGrid`.
  - Publishes as a latched (transient local) topic.
- **Outputs:**
  - `/road/costmap` (`nav_msgs/msg/OccupancyGrid`).

---

### 3.10 `src/road_survey/road_survey/height_slicer_node.py`
- **Location:** [`src/road_survey/road_survey/height_slicer_node.py`](../src/road_survey/road_survey/height_slicer_node.py)
- **Role:** Dynamically filters multi-level switchbacks based on UGV altitude.
- **Inputs:**
  - `/ugv/pose` (`geometry_msgs/msg/PoseStamped`)
  - Parameter `map_npz`
- **Code Logic:**
  - Monitors the UGV's current $Z$ coordinate.
  - Slices the 2.5-D elevation grid to retain only cells within $\pm 3.0\text{ m}$ of the vehicle.
  - Re-shapes and publishes the single-level costmap.
- **Outputs:**
  - Sliced `/road/costmap`.

---

### 3.11 `src/road_survey/road_survey/tune_offline.py`
- **Location:** [`src/road_survey/road_survey/tune_offline.py`](../src/road_survey/road_survey/tune_offline.py)
- **Role:** Offline CLI parameter tuning utility.
- **Inputs:** Path to a saved `road_map.npz` and command-line threshold flags (`--slope`, `--step`, `--rough`, `--thresh`, `--sweep`).
- **Code Logic:**
  - Re-runs geometric classification and costmap generation directly against the saved elevation grid in milliseconds without launching Gazebo or PX4.
- **Outputs:**
  - Re-classified `.npz` and costmap visual inspection images.

---

# 4. Package: `guidance`

### 4.1 `src/guidance/guidance/px4.py`
- **Location:** [`src/guidance/guidance/px4.py`](../src/guidance/guidance/px4.py)
- **Role:** Helper abstraction layer for PX4 DDS communication.
- **Functions:**
  - `enu_position(msg)`: Converts PX4 NED local position to ENU $(x, y, z)$.
  - `enu_yaw(msg)`: Extracts yaw heading in radians ENU from PX4 vehicle attitude quaternion.
  - `PX4_QOS`: Pre-configured BEST_EFFORT QoS profile matching PX4 uORB publishers.
- **Outputs:**
  - Metric ENU state vectors.

---

### 4.2 `src/guidance/guidance/aruco.py`
- **Location:** [`src/guidance/guidance/aruco.py`](../src/guidance/guidance/aruco.py)
- **Role:** High-accuracy OpenCV ArUco detector.
- **Functions:**
  - `detect_marker(bgr_image, marker_id, marker_length, k)`: Runs `cv2.aruco.detectMarkers()` using dictionary `DICT_4X4_50`. Solves PnP pose estimation to determine 3D translation vector $t$ and rotation vector $r$.
- **Outputs:**
  - Relative 3D translation $(t_x, t_y, t_z)$ and orientation matrix $R$.

---

### 4.3 `src/guidance/guidance/pursuit.py`
- **Location:** [`src/guidance/guidance/pursuit.py`](../src/guidance/guidance/pursuit.py)
- **Role:** Custom Regulated Pure Pursuit steering and speed controller (zero external dependencies).
- **Classes & Functions:**
  - `Path`: Parameterizes path waypoints by cumulative arc-length chord distance; provides `project()` (fast windowed projection) and `point_at()` (linear interpolation).
  - `command(x, y, yaw, path, s_hint, params)`: Computes heading error $\alpha$, triggers in-place pivot if $|\alpha| > 0.8\text{ rad}$, calculates curvature $\kappa = 2\sin\alpha / L_d$, commands angular velocity $\omega = \kappa \cdot v$, and regulates speed based on turn radius and goal approach.
- **Outputs:**
  - Tuple `(linear_velocity, angular_velocity, target_point, lateral_error)`.

---

### 4.4 `src/guidance/guidance/explore.py`
- **Location:** [`src/guidance/guidance/explore.py`](../src/guidance/guidance/explore.py)
- **Role:** Autonomous aerial exploration frontier planner ([`RoadExplorer`](../src/guidance/guidance/explore.py#L25-L95) class).
- **Inputs:** Current UAV pose and `/road/costmap`.
- **Code Logic:**
  - Identifies forward frontier centroids along the survey heading.
  - Emits lookahead waypoints $5.0\text{ m}$ ahead at survey altitude ($12.0\text{ m}$ AGL).
  - Detects road completion when no new frontier appears for 3 cycles.
- **Outputs:**
  - 3D flight waypoints $(x, y, z)$ for the survey mission.

---

### 4.5 `src/guidance/guidance/mission.py`
- **Location:** [`src/guidance/guidance/mission.py`](../src/guidance/guidance/mission.py)
- **Role:** High-level finite state machine governing the collaborative mission.
- **States:**
  - `INIT`: Awaiting PX4 EKF2 convergence.
  - `TAKEOFF`: Climbing to survey altitude.
  - `SURVEY`: Mapping road corridor (autonomous or teleop).
  - `RETURN_TO_UGV`: Flying back to UGV rendezvous coordinates.
  - `TRACK_UGV`: Holding $10\text{ m}$ station overhead while tracking marker.
  - `LAND`: Safe touchdown upon goal completion.
- **Outputs:**
  - State transitions and flight mode commands.

---

### 4.6 `src/guidance/guidance/localizer_node.py`
- **Location:** [`src/guidance/guidance/localizer_node.py`](../src/guidance/guidance/localizer_node.py)
- **Role:** Active ROS 2 node tracking the UGV from overhead camera frames.
- **Inputs:**
  - `/uav/rgb` (`sensor_msgs/msg/Image`)
  - `/uav/camera_info` (`sensor_msgs/msg/CameraInfo`)
  - `/fmu/out/vehicle_local_position` (`px4_msgs/msg/VehicleLocalPosition`)
  - `/fmu/out/vehicle_attitude` (`px4_msgs/msg/VehicleAttitude`)
- **Code Logic:**
  - Subscribes at $15\text{ Hz}$, executes `aruco.py` detection, projects optical coordinates through the drone's instantaneous world pose, and compensates for the $0.1275\text{ m}$ roof mount offset.
- **Outputs:**
  - `/ugv/pose` (`geometry_msgs/msg/PoseStamped`)
  - TF broadcast: `map` $\to$ `ugv_base_link`.

---

### 4.7 `src/guidance/guidance/follower_node.py`
- **Location:** [`src/guidance/guidance/follower_node.py`](../src/guidance/guidance/follower_node.py)
- **Role:** Active ROS 2 node driving the UGV along the mapped road.
- **Inputs:**
  - `/ugv/pose` (`geometry_msgs/msg/PoseStamped`)
  - `/road/costmap` (`nav_msgs/msg/OccupancyGrid`)
- **Code Logic:**
  - Extracts path centerline via `centerline.py`.
  - Runs pure pursuit control loop at $20\text{ Hz}$ via `pursuit.py`.
  - Monitors pose staleness ($0.5\text{ s}$ timeout safety halt).
- **Outputs:**
  - `/cmd_vel` (`geometry_msgs/msg/Twist`)
  - `/ugv/path` (`nav_msgs/msg/Path`).

---

### 4.8 `src/guidance/guidance/mission_node.py`
- **Location:** [`src/guidance/guidance/mission_node.py`](../src/guidance/guidance/mission_node.py)
- **Role:** Master interactive coordinator and PX4 offboard controller.
- **Inputs:**
  - Interactive operator terminal CLI.
  - `/ugv/pose`, `/road/costmap`, PX4 odometry.
- **Code Logic:**
  - Publishes offboard heartbeat setpoints (`/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command`).
  - Manages takeoff, survey translation, return, station-keeping over the UGV, map save service triggering, and landing.
- **Outputs:**
  - PX4 flight trajectory setpoints and vehicle command actions.

---

### 4.9 `src/guidance/guidance/pose_check_node.py`
- **Location:** [`src/guidance/guidance/pose_check_node.py`](../src/guidance/guidance/pose_check_node.py)
- **Role:** Real-time localization error validation node.
- **Inputs:**
  - `/ugv/pose` (ArUco visual estimate)
  - `/ugv/ground_truth` (Gazebo physics ground truth)
  - `/ugv/path`
- **Code Logic:**
  - Evaluates position error $\Delta r$, yaw error $\Delta \theta$, and lateral cross-track tracking deviation at $10\text{ Hz}$.
- **Outputs:**
  - Real-time terminal diagnostic telemetry.

---

# 5. Package: `bringup`

### 5.1 `src/bringup/launch/sim.launch.py`
- **Location:** [`src/bringup/launch/sim.launch.py`](../src/bringup/launch/sim.launch.py)
- **Role:** Main simulation launcher bringing up the entire virtual world and vehicles.
- **Inputs:**
  - Launch arguments: `world` (`drdo_world2`), `uav_model`, `px4_dir`, `nvidia_gpu`.
- **Code Logic:**
  - Loads spawn offsets from `world_poses.yaml`.
  - Auto-detects NVIDIA hardware via `shutil.which('nvidia-smi')` to configure GPU offload safely.
  - Sets unified `ROS_DOMAIN_ID=42` and dynamic `GZ_SIM_RESOURCE_PATH`.
  - Sequentially starts `world.launch.py`, `spawn_ackermann.launch.py`, `MicroXRCEAgent`, PX4 SITL default binary, static TF `world` $\to$ `map`, and `uav_bridge.launch.py`.
- **Outputs:**
  - Complete, synchronized simulation environment.

---

### 5.2 `src/bringup/launch/mission.launch.py`
- **Location:** [`src/bringup/launch/mission.launch.py`](../src/bringup/launch/mission.launch.py)
- **Role:** Autonomy stack launcher.
- **Inputs:**
  - Launch arguments: `survey` (`true`/`false`), `map_npz`, `pose_check`, `rviz`.
- **Code Logic:**
  - Launches `terrain_mapper` (or `map_publisher` if `survey:=false`).
  - Launches `ugv_localizer`, `ugv_follower`, `pose_check`, interactive `mission` node, and `rviz2`.
- **Outputs:**
  - Autonomous mission execution and 3D visualization.

---

### 5.3 `src/bringup/launch/rviz.launch.py`
- **Location:** [`src/bringup/launch/rviz.launch.py`](../src/bringup/launch/rviz.launch.py)
- **Role:** Pre-configured RViz2 visualization launcher.
- **Inputs:** `rviz_config` (defaults to `road_survey/rviz/survey.rviz`).
- **Code Logic:**
  - Starts RViz2 pre-loaded with displays for RGB camera, normalized depth camera, 3D point cloud, costmap, UGV path, and vehicle frames.
- **Outputs:**
  - RViz2 graphical user interface.

---

### 5.4 `src/bringup/scripts/setup_px4_uav.sh`
- **Location:** [`src/bringup/scripts/setup_px4_uav.sh`](../src/bringup/scripts/setup_px4_uav.sh)
- **Role:** Automated PX4 model and airframe setup script.
- **Inputs:** `$PX4_AUTOPILOT_DIR` or `$HOME/PX4-Autopilot`.
- **Code Logic:**
  - Automatically locates model files in source tree or ROS 2 install space.
  - Copies `x500_depth_down` model into PX4 `Tools/simulation/gz/models/`.
  - Copies airframe `4022_gz_x500_depth_down` into PX4 ROMFS airframes.
  - Registers airframe in PX4 CMake build and compiles `px4_sitl_default`.
- **Outputs:**
  - Ready-to-simulate PX4 build with downward camera quadrotor support.

---

### 5.5 `src/bringup/scripts/record_survey.sh`
- **Location:** [`src/bringup/scripts/record_survey.sh`](../src/bringup/scripts/record_survey.sh)
- **Role:** Minimal survey flight rosbag recorder.
- **Inputs:** Bag name argument.
- **Code Logic:**
  - Records only `/uav/depth`, `/uav/camera_info`, `/uav/depth_camera_info`, PX4 odometry, and `/clock`.
- **Outputs:**
  - Compact ROS 2 bag file in `bags/` directory for fast offline tuning.
