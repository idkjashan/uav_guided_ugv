# Handoff — integrating `road_survey`

For whoever (human or model) wires this into the running simulation. The
algorithms are written and tested; what is left is plumbing against the real
Gazebo topic names and a real flight. **Read `01_road_survey_plan.md` first** —
it explains why the code is shaped this way.

## What is done

- `src/road_survey/` — an `ament_python` package, four executables.
- 47 tests, all passing offline with no ROS and no simulator
  (`pytest src/road_survey/test`). They cover frame conversions, unprojection,
  grid statistics, the risk classifier, costmap encoding, and one end-to-end
  simulated survey that ray-casts a synthetic hill road and checks the
  recovered mask against ground truth (IoU 0.87).

## What is NOT done, in the order it will bite you

1. **Real Gazebo topic names.** The launch file assumes the depth sensor
   publishes on `/depth_camera` and `/depth_camera/camera_info`. Verify with
   `gz topic -l | grep -i depth` before anything else. If the SDF's `<topic>`
   differs, change the launch arguments — no code change is needed.
2. **`camera_info` may not exist** for the depth sensor. That is fine: leave
   `camera_info_topic` pointing at a topic that never publishes and the node
   uses intrinsics derived from `depth_hfov_rad` (fx = fy = 432.5 for
   640×480 @ 1.274 rad). Do check the encoding is `32FC1` and the units are
   metres — `ros2 topic echo /uav/depth --field encoding --once`.
3. **`use_sim_time`.** Must be true everywhere. If the node warns about it, or
   warns repeatedly that the pose is far from the depth frame, this is why.
4. **A real flight has never been run.** Nobody has confirmed the mapper
   produces a sane map from actual Gazebo depth data. That is the first
   integration task, not a formality.
5. **Nav2 is not configured.** See below.
6. **Stage 2** — ArUco pose, TF, Nav2 bringup — is not started.

## First integration run

```bash
cd ~/uav_guided_ugv
colcon build --packages-select road_survey --symlink-install
source install/setup.bash
source ~/px4_ros_ws/install/setup.bash        # px4_msgs
```

With PX4 SITL, Gazebo and `MicroXRCEAgent` already up:

```bash
ros2 launch road_survey survey.launch.py world:=drdo_world1
```

Then, before flying, confirm the four inputs are live:

```bash
ros2 topic hz /uav/depth                      # ~30 Hz
ros2 topic hz /fmu/out/vehicle_local_position # ~50 Hz
ros2 topic echo /road/stats --once            # after the first climb
ros2 topic echo /road/costmap --field info --once
```

`frames_used` climbing while `frames_seen` climbs faster is correct — the node
deliberately drops frames (rate limit, hover gate, tilt gate). `frames_used`
stuck at 0 while `frames_seen` climbs means a gate is rejecting everything;
the warnings say which.

## Nav2 configuration for stage 2

Two settings are not optional:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: /road/costmap
        subscribe_to_updates: false
        trinary_costmap: false      # REQUIRED: keeps the centre-seeking gradient
        lethal_cost_threshold: 100  # only 100 is lethal; 0..99 are real costs
```

With `trinary_costmap: true` (the default) every road cell collapses to free
and the rover has no reason to stay near the centre — which is exactly what the
rubric scores. Keep the inflation layer radius small (≤ 1 m); the road is only
6–10 m wide and a large inflation will close it.

If `/road/stats` reported `multi_level_cells > 0`, run `height_slicer` instead
of `map_publisher` and feed it the UGV pose.

## Interfaces

| Node | In | Out |
|---|---|---|
| `terrain_mapper` | `/uav/depth`, `/uav/depth_camera_info`, `/fmu/out/vehicle_local_position`, `/fmu/out/vehicle_attitude` | `/road/costmap`, `/road/stats`, services `~/save` `~/reset` |
| `map_publisher` | `road_map.npz` | `/road/costmap` (transient-local) |
| `height_slicer` | `road_map.npz`, `/ugv/pose` | `/road/costmap` |
| `tune_offline` | `road_map.npz` | PNG + npz per threshold set |

The `map` frame is **PX4's local ENU** — origin at the UAV's EKF init point,
i.e. the UAV spawn pose, *not* the Gazebo world origin. If you need Gazebo
world coordinates for scoring, apply the spawn pose from the world file as a
static offset; do not change the mapper.

## House rules for changes

- `frames.py`, `depth.py`, `grid.py`, `risk.py`, `costmap.py`, `centerline.py`
  are pure — numpy/scipy/cv2 only, no ROS imports. Keep them that way; that is
  what makes the tests fast and the algorithms reviewable.
- Any change to those files must keep `pytest src/road_survey/test` green.
- Thresholds belong in `config/road_survey.yaml`, not in code.
- Do not add `grid_map` / `elevation_mapping` as dependencies. They were
  considered and rejected: the ROS 2 ports are unofficial, they drag in Eigen
  and kindr, and the ~400 lines here are tested and understood.
