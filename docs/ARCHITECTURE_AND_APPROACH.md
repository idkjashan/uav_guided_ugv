# UAV-Guided UGV: System Architecture & Technical Approach

This document provides a comprehensive technical overview of the UAV-guided UGV system architecture, theoretical approach, coordinate frame transformations, algorithms, and operational pipelines.

---

## 1. Problem Statement & Core Challenge

In challenging, GPS-denied or sensor-constrained operational environments, ground rovers (UGVs) may lack onboard perception sensors (LiDAR, stereo cameras, radar) due to payload, power, cost, or vulnerability constraints. 

In this system:
- **UGV (Unmanned Ground Vehicle):** Carries **no onboard perception sensors** and utilizes **no wheel odometry**. It relies entirely on external guidance. It carries an ArUco fiducial marker on its roof.
- **UAV (Unmanned Aerial Vehicle):** Serves as an aerial perception, mapping, and guidance platform equipped with a downward-facing Intel RealSense D435 depth camera, an IMX214 high-resolution RGB camera, and PX4 Autopilot navigation.

The mission requires the UAV to:
1. **Survey the unknown environment** autonomously from the air.
2. **Construct a 2.5D elevation and traversability costmap** identifying safe, driveable road corridors while rejecting steep banks, drops, boulders, and ditches.
3. **Extract the optimal road centreline**.
4. **Return autonomously to base**, locate the blind UGV, and establish a high-accuracy visual localization link.
5. **Actively guide and escort the UGV** along the surveyed road centreline to the destination while holding station overhead.

---

## 2. End-to-End Mission Workflow

```
                                  [ START ]
                                      │
                                      ▼
                        ┌───────────────────────────┐
                        │     Phase 1: TAKEOFF      │
                        │ Climb vertically to 12m   │
                        └─────────────┬─────────────┘
                                      │
                                      ▼
                        ┌───────────────────────────┐
                        │     Phase 2: SURVEY       │
                        │ Autonomous exploration    │
                        │ 2.5D elevation & costmap  │
                        │ Breadcrumb recording      │
                        └─────────────┬─────────────┘
                                      │ (Road end detected)
                                      ▼
                        ┌───────────────────────────┐
                        │     Phase 3: RETURN       │
                        │ Retrace 3D breadcrumbs    │
                        │ Descend safely to 10m     │
                        └─────────────┬─────────────┘
                                      │ (Spawn reached)
                                      ▼
                        ┌───────────────────────────┐
                        │     Phase 4: ACQUIRE      │
                        │ Detect roof ArUco marker  │
                        │ Solve camera-to-UGV PnP   │
                        └─────────────┬─────────────┘
                                      │ (Marker locked)
                                      ▼
                        ┌───────────────────────────┐
                        │      Phase 5: TRACK       │
                        │ Pure Pursuit UGV steering │
                        │ Dynamic UAV escort flight │
                        └─────────────┬─────────────┘
                                      │ (Goal reached)
                                      ▼
                        ┌───────────────────────────┐
                        │  Phase 6: GOAL REACHED    │
                        │ UGV stops; UAV hovers/RTL │
                        └───────────────────────────┘
```

---

## 3. Stage 1: Road Survey & Costmap Generation

### 3.1 Downward Depth Perception & Projection
- The UAV carries a downward-facing depth camera (`StereoOV7251` / Intel RealSense D435 profile) with an effective range clipped at $19.1\text{ m}$.
- At an operating altitude of $12\text{ m}$ Above Ground Level (AGL), the camera footprint covers approximately $17.7\text{ m} \times 13.4\text{ m}$ on the ground with a ground sampling distance of $\approx 2.8\text{ cm/pixel}$.
- Raw depth frames (`/uav/depth`) are back-projected into 3D camera-frame coordinates using camera intrinsics ($f_x, f_y, c_x, c_y$):
  $$X_c = \frac{(u - c_x) Z_c}{f_x}, \quad Y_c = \frac{(v - c_y) Z_c}{f_y}, \quad Z_c = d(u, v)$$
- Using the UAV's vehicle odometry and sensor mounting extrinsics, the points are transformed into the world `map` frame (PX4 local ENU).

### 3.2 2.5D Elevation Grid Accumulation
- The continuous world points are binned into a discrete 2.5D elevation grid (`road_survey/grid.py`) with a resolution of $0.25\text{ m/cell}$.
- For each cell $(x, y)$, running statistics are maintained:
  - Minimum elevation $z_{\min}$
  - Maximum elevation $z_{\max}$
  - Running mean elevation $\bar{z}$
  - Sample count $N$

### 3.3 Terrain Feature Extraction & Risk Combination
Road surfaces in mountainous terrain are essentially flat shelves cut into hillside slopes. To detect this shelf reliably without confusing smooth hillsides or shallow ramps for roads, three geometric features are evaluated in a local $5 \times 5$ window ($1.25\text{ m}$):
1. **Slope Angle ($\theta$):** Computed from a locally fitted plane $z = ax + by + c$ using separable box filters:
   $$\theta = \arccos\left(\frac{1}{\sqrt{a^2 + b^2 + 1}}\right)$$
2. **Step Height ($\Delta z$):** The difference between the highest and lowest points within the local window:
   $$\Delta z = z_{\max} - z_{\min}$$
3. **Roughness ($\sigma_r$):** The root-mean-square residual of points **measured about the locally fitted plane** (not about the mean):
   $$\sigma_r = \sqrt{\frac{1}{K} \sum (z_k - (ax_k + by_k + c))^2}$$

Each metric is normalized against critical thresholds:
$$n_{\text{slope}} = \text{clip}\left(\frac{\theta}{\theta_{\text{crit}}}, 0, 1\right), \quad n_{\text{step}} = \text{clip}\left(\frac{\Delta z}{\Delta z_{\text{crit}}}, 0, 1\right), \quad n_{\text{rough}} = \text{clip}\left(\frac{\sigma_r}{\sigma_{r,\text{crit}}}, 0, 1\right)$$

The continuous risk index is formulated as:
$$\text{risk} = \begin{cases} 1.0 & \text{if } \max(n_{\text{slope}}, n_{\text{step}}, n_{\text{rough}}) \ge 1.0 \\ \frac{1}{3}(n_{\text{slope}} + n_{\text{step}} + n_{\text{rough}}) & \text{otherwise} \end{cases}$$

A cell is classified as road if $\text{risk} < \text{risk}_{\text{thresh}}$ (default $0.35$).

### 3.4 Road Centreline Extraction
Once the binary road mask is established:
1. **Hole Filling:** Morphological closing removes interior artifacts (such as the parked UGV itself, which shows up as a step discontinuity during the survey).
2. **Medial Axis Skeletonization:** Identifies the topological centre of the road corridor.
3. **Graph Tracing & Spline Smoothing:** Extracts the longest continuous graph branch and smooths it using cubic B-splines with controlled smoothing parameter ($s = 0.01 N$) to prevent cutting corners while eliminating pixel discretization noise.
4. **Fork Pruning:** Trims artificial medial axis corner forks that occur at the boundary of the surveyed corridor.

### 3.5 Frontier Exploration
The UAV explores the road without human intervention:
- A forward circular sampling ring ($5\text{ m}$ radius) is cast ahead of the UAV's current position.
- Road cells falling within an $80^\circ$ forward sector are clustered.
- The cluster centre closest to the UAV's current flight heading (weighted towards lower cost cells) is selected as the next survey waypoint.
- If three successive iterations detect no forward road candidate while hovering at the frontier, the survey concludes that the road has ended.

---

## 4. Stage 2: Guidance, Localization & Path Tracking

### 4.1 Breadcrumb Return Flight
- During the survey, the UAV logs 3D position breadcrumbs every $2\text{ m}$.
- Because the UAV maintained terrain-relative altitude ($12\text{ m}$ AGL) over climbing terrain, simply flying a straight line back to spawn could result in a collision with rising hillsides.
- The return state retraces the stored breadcrumbs in reverse order, ensuring guaranteed terrain clearance.

### 4.2 Overhead ArUco Localization
- The UGV carries a $40\text{ cm}$ ArUco marker (Dictionary `4x4_50`, ID 0) on its roof.
- When the UAV descends to guidance altitude ($10\text{ m}$), the marker is detected in the high-resolution RGB camera frame.
- **Perspective-n-Point (PnP):** `cv2.solvePnP` computes the translation vector $\mathbf{t}_{c}$ and rotation vector $\mathbf{r}_{c}$ from the camera optical frame to the marker frame.
- **Coordinate Transformations:**
  1. Optical to Camera Link: $[x, y, z]_{\text{cam}} = [z, -x, -y]_{\text{opt}}$
  2. Camera Link to UAV Body: Forward-Right-Down (FRD)
  3. UAV Body to PX4 Local Frame (NED) via vehicle attitude quaternion $\mathbf{q}_{\text{att}}$
  4. PX4 Local (NED) to ROS Map Frame (ENU) via fixed rotation $\mathbf{R}_{\text{NED}\to\text{ENU}}$
- This transformation chain resolves the UGV's full 3D position $(x, y, z)$ and planar heading $\psi$ in the world `map` frame at $\approx 30\text{ Hz}$.

### 4.3 Pure Pursuit Centreline Follower
The UGV is driven by a Pure Pursuit path tracking controller (`guidance/pursuit.py`):
1. **Lookahead Point:** Finds the path point located at lookahead distance $L_d$ (nominal $1.5\text{ m}$) ahead of the UGV along the centreline.
2. **Steering Curvature:**
   $$\kappa = \frac{2 \sin(\alpha)}{L_d}$$
   where $\alpha$ is the angle between the UGV's current heading vector and the lookahead vector.
3. **Cross-Track Error (CTE) Minimization:** Steering commands translate to angular velocity $\omega = v \cdot \kappa$.
4. **Adaptive Heading Error Handling:** If the heading error $|\alpha| > 45^\circ$, linear velocity is held at zero while the UGV executes an in-place rotation until aligned.
5. **Curvature Speed Scaling:** Linear velocity scales down on sharp curves ($R < 2\text{ m}$) and during final approach within $2\text{ m}$ of the goal.
6. **Failsafe Timeout:** If no fresh UGV pose is received for $> 0.5\text{ s}$ (e.g. marker occlusion), zero velocity is commanded immediately.

### 4.4 Dynamic UAV Escort / Station-Keeping
- The UAV operates in PX4 **Offboard Mode**.
- Rather than flying ahead or waiting at fixed waypoints, the UAV continuously sets its horizontal $(x, y)$ setpoint directly to the UGV's estimated position:
  $$\mathbf{p}_{\text{setpoint, uav}} = \begin{bmatrix} x_{\text{ugv}} \\ y_{\text{ugv}} \\ z_{\text{terrain}} + h_{\text{guide}} \end{bmatrix}$$
- Velocity feed-forward is computed to ensure smooth tracking without lag.
- The UAV naturally escorts the UGV, keeping the roof marker centered in the camera frame throughout the traversal.
- Upon reaching the goal waypoint at the end of the road, the UGV halts, and the mission signals completion.

---

## 5. Coordinate Frame Conventions

| Frame Name | Convention | Description |
|---|---|---|
| `world` | ENU | Gazebo simulation ground-truth world origin. |
| `map` | ENU | PX4 local origin at UAV takeoff location ($x, y$ aligned with world). |
| `base_link` (UAV) | FRD | UAV body frame (Forward, Right, Down). |
| `camera_optical` | EDN | Downward camera optical frame (X right, Y down, Z forward/down). |
| `base_link` (UGV) | FLU | UGV chassis frame (Forward, Left, Up). |
| `marker` | FLU | ArUco marker center on UGV roof (X forward, Y left, Z up). |

---

## 6. Critical Engineering Fixes Applied

1. **UGV Wheel Joint Rotation Axes (`model.sdf`):**
   - In Gazebo SDF, child wheel links have a default roll of $+90^\circ$ (`1.5707963 rad`).
   - Specifying joint axes as `<xyz>0 0 1</xyz>` in the child link frame resulted in wheel rotation vectors of $(0, -1, 0)$ in the parent chassis frame, causing positive torque/velocity to rotate the wheels backward.
   - The axes were inverted to `<xyz>0 0 -1</xyz>`, restoring standard positive forward motion and correct left/right differential steering.
2. **PX4 Airframe Configuration (`4022_gz_x500_depth_down`):**
   - Ensured standard GPS fusion is active in EKF2 (`EKF2_GPS_CTRL 7`) matching the simulated sensors.
   - Cleared stale rootfs parameter caches to avoid conflicting visual-odometry settings.
3. **Camera Far-Clip Altitude Enclosure:**
   - Downward sensor far clip is $19.1\text{ m}$. Survey altitude was bounded strictly to $10-15\text{ m}$ AGL with real-time ground distance feedback to prevent sensor blindness over climbing roads.
