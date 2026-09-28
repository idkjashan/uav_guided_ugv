# Reference Repositories, Prior Art & Attribution

This document details foundational open-source repositories, architectural references, and prior art that inform the design of the **DRDO UAV-Guided UGV Navigation System**, along with how our implementation builds upon and improves them.

---

## 1. Upstream & Foundational Open-Source Repositories

### 1. PX4 Autopilot (`PX4/PX4-Autopilot`)
- **Repository:** [https://github.com/PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)
- **Role in System:** Core aerial flight stack.
- **Components Leveraged:**
  - `ROMFS/px4fmu_common/init.d-posix/`: POSIX SITL startup architecture.
  - `Tools/simulation/gz/models/`: Base simulation model definitions for the Holybro `x500` quadrotor airframe and `OakD-Lite` sensor.
  - 24-state Extended Kalman Filter (EKF2): Fuses simulated IMU, barometer, GPS, and magnetometer to produce centimeter-accurate metric odometry (`VehicleLocalPosition` and `VehicleAttitude`).
- **Our Custom Contribution:**
  - Created the dedicated airframe [`4022_gz_x500_depth_down`](file:///home/jashan/uav_guided_ugv/src/drdo_gz_worlds/px4_airframes/4022_gz_x500_depth_down).
  - Built the automated installer script [`setup_px4_uav.sh`](file:///home/jashan/uav_guided_ugv/src/bringup/scripts/setup_px4_uav.sh) to integrate our nadir-depth model seamlessly into PX4 SITL builds.

---

### 2. Micro-XRCE-DDS Agent & PX4 ROS 2 Messages (`eProsima` & `PX4/px4_msgs`)
- **Repositories:**
  - Micro-XRCE-DDS Agent: [https://github.com/eProsima/Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)
  - PX4 ROS 2 Messages: [https://github.com/PX4/px4_msgs](https://github.com/PX4/px4_msgs)
- **Role in System:** Low-latency communications middleware linking PX4's internal uORB publish-subscribe bus with the ROS 2 DDS network.
- **Topics Consumed / Commanded:**
  - `/fmu/out/vehicle_local_position`: Subscribed for real-time UAV world coordinates.
  - `/fmu/out/vehicle_attitude`: Subscribed for UAV orientation quaternions.
  - `/fmu/in/trajectory_setpoint`: Published at 20 Hz to command 3D velocity and position waypoints.
  - `/fmu/in/vehicle_command`: Published to command offboard arming, takeoff, and mode changes.

---

### 3. Gazebo Harmonic & ROS-Gz Bridge (`gazebosim/gz-sim` & `gazebosim/ros_gz`)
- **Repositories:**
  - Gazebo Sim: [https://github.com/gazebosim/gz-sim](https://github.com/gazebosim/gz-sim)
  - ROS-Gz: [https://github.com/gazebosim/ros_gz](https://github.com/gazebosim/ros_gz)
- **Role in System:** Physical simulation environment and sensor rendering.
- **Components Leveraged:**
  - `gz::sim::systems::DiffDrive`: Powers the 4-wheel Ackermann UGV chassis dynamics.
  - `gz::sim::systems::Sensors`: GPU ray-casting for both color (IMX214) and depth (StereoOV7251) cameras.
  - `ros_gz_bridge`: Birectionally forwards camera images and command velocities between Gazebo and ROS 2.

---

### 4. Original DRDO Inter-IIT Challenge Repository (`interiit22`)
- **Reference Context:** The original DRDO problem statement was distributed as `interiit22` targeted at ROS 1 Noetic and Gazebo Classic 11.
- **Limitations of Original Codebase:**
  - Coupled tightly to outdated ROS 1 Noetic (EOL) and Gazebo Classic (EOL).
  - Hardcoded drone models into the `.world` files, preventing modular vehicle swapping.
  - Fragile wheel contact friction models that caused the UGV to spin out or drift on inclines.
- **Our Modernization & Architectural Overhaul:**
  - 100% ported to **ROS 2 Humble** and **Gazebo Harmonic (gz-sim)**.
  - Converted terrain collada meshes and digital elevation models into standalone, modular models (`world1_mesh`, `world2_mesh`, `world3_mesh`).
  - Decoupled UAV and UGV models, spawning them dynamically at parameterized terrain start coordinates via launch arguments.

---

### 5. Nav2 Regulated Pure Pursuit Controller (`ros-navigation/navigation2`)
- **Repository:** [https://github.com/ros-navigation/navigation2](https://github.com/ros-navigation/navigation2)
- **Conceptual Reference:** The `nav2_regulated_pure_pursuit_controller` (Macenski et al., 2023).
- **Comparison & Our Implementation Advantage:**
  - **Standard Nav2 Approach:** Requires compiling the heavy C++ Nav2 stack, costmap layers, lifecycle managers, behavior tree navigators, and action servers.
  - **Our Approach ([`pursuit.py`](file:///home/jashan/uav_guided_ugv/src/guidance/guidance/pursuit.py)):** Built a **zero-dependency, standalone pure Python controller** based on the same kinematic regulation principles:
    - Chord-distance arc-length interpolation.
    - Curvature-based velocity scaling ($v \propto R$).
    - Goal proximity deceleration ramp.
    - Added an essential **in-place pivot safety guard** ($|\alpha| > 0.8\text{ rad} \implies v=0, \omega=\pm 0.6$) designed specifically to handle tight mountain switchbacks without wheel drift.

---

### 6. Luxonis DepthAI ROS 2 Stack (`luxonis/depthai-ros`)
- **Repository:** [https://github.com/luxonis/depthai-ros](https://github.com/luxonis/depthai-ros)
- **Role in System:** Hardware baseline and geometric specification reference for the **Oak-D Lite**.
- **Specifications Adopted:**
  - $75\text{ mm}$ stereo baseline between the OmniVision OV7251 grayscale sensors.
  - $1.274\text{ rad}$ ($73^\circ$) horizontal field of view.
  - Optical coordinate alignment adhering to ROS REP-103 conventions.

---

### 7. OpenCV Contrib ArUco Module (`opencv/opencv_contrib`)
- **Repository:** [https://github.com/opencv/opencv_contrib/tree/4.x/modules/aruco](https://github.com/opencv/opencv_contrib/tree/4.x/modules/aruco)
- **Role in System:** Visual fiducial marker tracking and 6-DOF Perspective-n-Point pose estimation.
- **Implementation:**
  - Utilizes `DICT_4X4_50` ID 0 for fast corner localization under variable lighting.
  - Employs `estimatePoseSingleMarkers()` to compute the 3D translation vector and Rodrigues rotation vector from the camera optical center to the roof marker.

---

### 8. Scikit-Image Morphology (`scikit-image/scikit-image`)
- **Repository:** [https://github.com/scikit-image/scikit-image](https://github.com/scikit-image/scikit-image)
- **Role in System:** Topological skeletonization for road centerline extraction.
- **Implementation:**
  - Implements the Lee-94 2D medial axis skeletonization algorithm in [`centerline.py`](file:///home/jashan/uav_guided_ugv/src/road_survey/road_survey/centerline.py) to thin the 2D traversable road ribbon into a 1-pixel-wide topological centerline.

---

## 2. Key Academic & Technical Citations

1. **Coulter, R. C. (1992).** *Implementation of the Pure Pursuit Path Tracking Algorithm.* Carnegie Mellon University, The Robotics Institute, Technical Report CMU-RI-TR-92-01.
2. **Macenski, S., Singh, F., Martin, F., & Gines, J. (2023).** *Regulated Pure Pursuit for Mobile Robot Path Tracking.* Autonomous Robots, Springer.
3. **Garrido-Jurado, S., Muñoz-Salinas, R., Madrid-Cuevas, F. J., & Marín-Jiménez, M. J. (2014).** *Automatic generation and detection of highly reliable fiducial markers under occlusion.* Pattern Recognition, 47(6), 2280-2292.
4. **Lee, T. C., Kashyap, R. L., & Chu, C. N. (1994).** *Building skeleton models via 3-D medial surface/axis thinning algorithms.* CVGIP: Graphical Models and Image Processing, 56(6), 462-478.
5. **Foote, T. (2010).** *REP 105: Coordinate Frames for Mobile Platforms.* ROS Enhancement Proposals.
