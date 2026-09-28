# 2.5-D Mapping, Terrain Classification & Costmap Generation

## 1. 2.5-D Elevation Grid Accumulator ([`grid.py`](file:///home/jashan/uav_guided_ugv/src/road_survey/road_survey/grid.py))

Rather than accumulating an unstructured, memory-intensive 3D voxel grid, the survey module accumulates points into an efficient **2.5-D Digital Elevation Model (DEM)** represented by the [`ElevationGrid`](file:///home/jashan/uav_guided_ugv/src/road_survey/road_survey/grid.py#L20-L100) class.

```
       Map Grid (400m x 400m, 0.25m resolution = 1600 x 1600 cells)
       +-------------------------------------------------------------+
       |                                                             |
       |                   Cell (row, col)                           |
       |                   +-------------------------------+         |
       |                   | - Mean Elevation z_bar        |         |
       |                   | - Min Elevation z_min         |         |
       |                   | - Max Elevation z_max         |         |
       |                   | - Sample Count N              |         |
       |                   +-------------------------------+         |
       |                                                             |
       +-------------------------------------------------------------+
```

### Grid Parameters
- **Dimensions:** $400\text{ m} \times 400\text{ m}$, centered on the UAV's initial spawn coordinates.
- **Resolution:** $\Delta = 0.25\text{ m}$ per cell ($1600 \times 1600$ cells total).
- **Coordinate Conversion:** For any world point $(X, Y)$:
  $$\text{col} = \left\lfloor \frac{X - \text{origin}_x}{\Delta} \right\rfloor, \quad \text{row} = \left\lfloor \frac{Y - \text{origin}_y}{\Delta} \right\rfloor$$

### Numerically Stable Rolling Elevation Updates
When a batch of 3D points falls into cell $(r, c)$, its elevation statistics are updated incrementally:
- **Sample Count:** $N \leftarrow N + k$
- **Running Mean:** $\bar{z} \leftarrow \bar{z} + \frac{\sum z_i - k \cdot \bar{z}}{N}$
- **Extrema Tracking:** $z_{\min} \leftarrow \min(z_{\min}, \min(z_i)), \quad z_{\max} \leftarrow \max(z_{\max}, \max(z_i))$

Cells with $N < \text{min\_count}$ ($N < 3$) are classified as unobserved.

---

## 2. Geometric Terrain Classification ([`risk.py`](file:///home/jashan/uav_guided_ugv/src/road_survey/road_survey/risk.py))

Every $2.0\text{ seconds}$, [`terrain_mapper_node`](file:///home/jashan/uav_guided_ugv/src/road_survey/road_survey/terrain_mapper_node.py) analyzes the current elevation grid to identify traversable road surfaces versus hazardous mountain terrain.

Rather than relying on brittle semantic RGB color segmentation (which degrades under shadows, dust, and lighting shifts), classification is performed **purely geometrically** using local surface properties.

```
          Fitted Local Plane: z = p*x + q*y + c
          ^
         /|  Local normal n = [-p, -q, 1] / sqrt(p^2 + q^2 + 1)
        / |  Slope angle theta = arctan(sqrt(p^2 + q^2))
       /  v
      +-------+  <-- Step Hazard = max(z) - min(z) in 3x3 window
     /       /|
    +-------+ |  <-- Roughness = RMS residual against fitted plane
    |       | |
```

### Step 1: Local Tangent Plane Fitting
For each cell $(r, c)$, a least-squares plane $z = p \cdot x + q \cdot y + c$ is fitted across a $5 \times 5$ cell neighborhood ($1.25\text{ m} \times 1.25\text{ m}$, roughly matching the UGV wheelbase):
$$\begin{bmatrix} p \\ q \\ c \end{bmatrix} = (A^T A)^{-1} A^T Z$$
where $A$ contains local cell offsets $(\Delta x, \Delta y, 1)$ and $Z$ contains the measured elevations.

### Step 2: Risk Metrics Computation
The terrain risk is decomposed into three distinct physical hazards:

1. **Slope Risk ($R_{\text{slope}}$):**
   The terrain incline angle is:
   $$\theta = \arctan\left(\sqrt{p^2 + q^2}\right)$$
   Normalized against the critical vehicle tilt limit $\theta_{\text{crit}} = 15.0^\circ$ ($0.2618\text{ rad}$):
   $$R_{\text{slope}} = \min\left(1.0, \frac{\theta}{\theta_{\text{crit}}}\right)$$

2. **Step Hazard Risk ($R_{\text{step}}$):**
   The maximum vertical step inside a $3 \times 3$ window ($0.75\text{ m} \times 0.75\text{ m}$):
   $$\Delta h = \max_{3 \times 3}(z) - \min_{3 \times 3}(z)$$
   Normalized against the UGV bumper/wheel clearance $h_{\text{crit}} = 0.25\text{ m}$:
   $$R_{\text{step}} = \min\left(1.0, \frac{\Delta h}{h_{\text{crit}}}\right)$$

3. **Roughness / Micro-Obstacle Risk ($R_{\text{rough}}$):**
   The root-mean-square (RMS) residual error of observed cell heights about the fitted tangent plane:
   $$\sigma = \sqrt{\frac{1}{M} \sum_{i=1}^M \left(z_i - (p \cdot x_i + q \cdot y_i + c)\right)^2}$$
   Normalized against the suspension compliance threshold $\sigma_{\text{crit}} = 0.12\text{ m}$:
   $$R_{\text{rough}} = \min\left(1.0, \frac{\sigma}{\sigma_{\text{crit}}}\right)$$

### Step 3: Composite Hazard Score & Road Mask
The total terrain risk $R(r, c)$ is the maximum hazard:
$$R(r, c) = \max\left(R_{\text{slope}}, R_{\text{step}}, R_{\text{rough}}\right)$$
A cell is classified as **traversable road** if:
$$\text{is\_road}(r, c) = \left(R(r, c) \le R_{\text{thresh}}\right) \land \left(N(r, c) \ge N_{\text{min}}\right)$$
where $R_{\text{thresh}} = 0.35$ and $N_{\text{min}} = 3$.

---

## 3. Costmap Shaping & Morphology ([`costmap.py`](file:///home/jashan/uav_guided_ugv/src/road_survey/road_survey/costmap.py))

Raw geometric classification contains sensor speckle, isolated boulders, and fragmented edges. [`costmap.py`](file:///home/jashan/uav_guided_ugv/src/road_survey/road_survey/costmap.py) refines the binary mask into a navigable costmap:

1. **Morphological Filtering:**
   - **Binary Opening** (radius $0.4\text{ m}$): Removes isolated single-pixel false positives.
   - **Binary Closing** (radius $0.8\text{ m}$): Closes minor fractures and sensor shadow gaps along the road.
2. **Connected Component Extraction:**
   - Detects all connected road regions. Discards small disjoint patches ($\text{area} < 25.0\text{ m}^2$).
   - Retains the primary continuous road ribbon seeded under the vehicle spawn coordinate.
3. **Euclidean Distance Transform (EDT):**
   - For every cell within the traversable road ribbon, computes its exact metric Euclidean distance $d(r, c)$ to the nearest road boundary:
     $$\text{cost}(r, c) = \text{round}\left(99 \cdot \left(1.0 - \min\left(1.0, \frac{d(r, c)}{d_{\text{ref}}}\right)\right)\right)$$
     where $d_{\text{ref}} = 1.5\text{ m}$.
   - **Result:**
     - The centerline of the road has distance $d \ge 1.5\text{ m} \implies \mathbf{\text{cost} = 0}$ (highest preference).
     - As the vehicle approaches the cliff or boundary, cost smoothly ramps up from $0 \to 99$.
     - Unmapped, hazardous, or off-road cells are marked as **$100$ (Lethal Obstacle)** or **$-1$ (Unknown / Lethal)**.

The costmap is serialized into standard ROS 2 `nav_msgs/msg/OccupancyGrid` format on `/road/costmap`.

---

## 4. Multi-Level Road Slicing ([`height_slicer_node.py`](file:///home/jashan/uav_guided_ugv/src/road_survey/road_survey/height_slicer_node.py))

In steep mountain switchbacks, hairpin bends often pass directly above or below one another with only a few meters of horizontal separation:

```
              ====================== Upper Switchback (Z = 24m)
                  | | (cliff)
              ====================== Lower Switchback (Z = 16m)
```

In a traditional 2D costmap projection, the upper and lower roads overlap, causing global path planners to jump vertically between switchback tiers.

The [`height_slicer_node`](file:///home/jashan/uav_guided_ugv/src/road_survey/road_survey/height_slicer_node.py) solves this:
- Subscribes to the UGV's current 3D pose (`/ugv/pose`).
- Obtains the UGV's current altitude $Z_{\text{ugv}}$.
- Filters the 2.5-D elevation grid to keep only cells satisfying:
  $$|z_{\text{cell}} - Z_{\text{ugv}}| \le \Delta z_{\text{band}} \quad (\Delta z_{\text{band}} = 3.0\text{ m})$$
- Publishes an altitude-sliced costmap that isolates the active driving plane.

---

## 5. Road Centerline Extraction ([`centerline.py`](file:///home/jashan/uav_guided_ugv/src/road_survey/road_survey/centerline.py))

To generate a drivable reference trajectory for pure pursuit:
1. **Medial Axis Skeletonization:** Applies the Lee-94 topological skeletonization algorithm (`skimage.morphology.skeletonize`) to the binary road mask, thinning the road ribbon to a 1-pixel-wide topological centerline.
2. **Graph Traversal:** Converts the pixel skeleton into an adjacency graph, searches for the longest continuous path beginning at the vehicle's initial pose, and terminates at the furthest road frontier.
3. **Spline Smoothing:** Resamples the discrete pixel graph into evenly spaced metric waypoints with $0.25\text{ m}$ chord spacing, published as `nav_msgs/msg/Path` on `/ugv/path`.
