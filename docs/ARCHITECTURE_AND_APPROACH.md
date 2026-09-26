# Architecture

The UGV has no sensors. Everything it knows comes from the UAV: the UAV maps
the road with a downward depth camera, flies back, finds the ArUco marker on
the UGV's roof with its downward RGB camera and keeps station above it. The
UGV follows the road centre line using only the pose the UAV measures. There
is no wheel odometry anywhere in the loop.

Why each piece is built the way it is lives in `01_road_survey_plan.md`
(mapping) and `02_guidance.md` (mission, localisation, following). This file
is the map of how the pieces fit.

## 1. Vehicles and sensors

| | What | Used for |
|---|---|---|
| UAV | PX4 x500 (`gz_x500_depth_down`, airframe 4022), GPS on | flies the survey and the escort |
| Depth camera | OakD-Lite StereoOV7251, nadir, 640x480, 73 deg HFOV, far clip 19.1 m | road mapping, height above ground |
| RGB camera | OakD-Lite IMX214, nadir, 1920x1080, 90 deg HFOV | finding the UGV's marker |
| UGV | 4-wheel skid-steer, gz `DiffDrive`, driven by `/cmd_vel` | drives the road |
| Marker | ArUco DICT_4X4_50 id 0, 0.55 m plate, 0.446 m black square, on the roof | the UGV's only link to the world |

## 2. Nodes and topics

```
                 Gazebo + PX4 SITL (sim.launch.py)
   /uav/depth ─────┬──────────────────────┐        /uav/rgb, /uav/camera_info
   PX4 pose ───────┤                      │        PX4 pose
                   ▼                      │           │
           ┌───────────────┐              │           ▼
           │terrain_mapper │              │   ┌───────────────┐
           │ (road_survey) │              │   │ ugv_localizer │
           └───────┬───────┘              │   └───────┬───────┘
      /road/costmap│ (every 2 s)          │           │ /ugv/pose (map frame)
                   ├──────────────┐       │           ├──────────────────┐
                   ▼              ▼       ▼           ▼                  ▼
           ┌───────────────┐   ┌─────────────────────────┐      ┌───────────────┐
           │ ugv_follower  │◀──│        mission          │      │  pose_check   │
           │ pure pursuit  │   │ PX4 offboard state      │      │ (validation)  │
           └───────┬───────┘   │ machine                 │      └───────────────┘
                   │           └────────────┬────────────┘       ▲
     /cmd_vel ─────┘  /mission/state ◀──────┤                    │ /ugv/ground_truth
     /ugv/path, /ugv/goal_reached ─────────▶│                    │ (Gazebo, never
                                            ▼                    │  used for control)
                            /fmu/in/trajectory_setpoint (PX4)
```

| Node | Package | Reads | Writes |
|---|---|---|---|
| `terrain_mapper` | road_survey | `/uav/depth`, PX4 local position + attitude | `/road/costmap`, `/road/stats`, `/uav/pose`, `maps/road_map.*` on save |
| `map_publisher` | road_survey | `maps/road_map.npz` | `/road/costmap` (used instead of the mapper with `survey:=false`) |
| `ugv_localizer` | guidance | `/uav/rgb`, `/uav/camera_info`, PX4 local position + attitude | `/ugv/pose` |
| `mission` | guidance | PX4 status + local position, `/uav/depth`, `/road/costmap`, `/ugv/pose`, `/ugv/goal_reached` | PX4 offboard setpoints and commands, `/mission/state` |
| `ugv_follower` | guidance | `/ugv/pose`, `/road/costmap`, `/mission/state` | `/cmd_vel`, `/ugv/path`, `/ugv/goal_reached`, `/ugv/cross_track_error` |
| `pose_check` | guidance | `/ugv/pose`, `/ugv/ground_truth`, `/ugv/path` | log only |

## 3. The mission, start to end

```
TAKEOFF -> SURVEY -> RETURN -> ACQUIRE -> TRACK -> DONE
```

| State | UAV | UGV | Ends when |
|---|---|---|---|
| TAKEOFF | climbs 1 m/s to 12 m above spawn | parked | height reached |
| SURVEY | follows the mapped road at 1.5 m/s, 12 m above the ground under it, drops a breadcrumb every 2 m | parked (it shows up in the map as a hole, filled later) | no road ahead for 3 costmaps in a row; the map is saved |
| RETURN | retraces the breadcrumbs at 3 m/s | parked | back over spawn |
| ACQUIRE | hovers 10 m above spawn, climbs to 16 m after 3 s without the marker | parked | marker seen |
| TRACK | holds 10 m above the UGV | follower builds the centre line once, then drives it | follower reaches the end |
| DONE | holds over the stopped UGV | stopped | (final state) |

With `survey:=false` the mission goes TAKEOFF -> ACQUIRE and the map comes
from the last saved survey.

## 4. Stage 1: mapping the road

1. **Depth to points.** Each depth frame is unprojected with the camera
   intrinsics, then moved into the `map` frame with the camera extrinsics and
   the PX4 pose nearest the frame's timestamp. Frames are used at most 5 Hz,
   only after the UAV has moved 0.25 m or turned 3 degrees, and only when it is
   tilted less than 25 degrees.
2. **Points to a 2.5-D grid.** 0.25 m cells, each keeping count, mean
   height, min and max. A cell needs 3 samples to count as known.
3. **Grid to road.** For every cell a plane is fitted over a 5x5 window
   (1.25 m): *slope* is its angle, *roughness* is the RMS residual about the
   plane, *step* is max minus min over 3x3. Each is divided by a critical
   value (15 deg, 0.12 m, 0.25 m). If any reaches 1 the cell is not road;
   otherwise risk is their mean and cells under 0.35 are road.
4. **Road to costmap.** Morphological open and close, keep the largest piece
   (and the one under the spawn), then cost 0 in the middle of the road
   rising to 99 at the edge, 100 off the road. Published every 2 s as a
   `nav_msgs/OccupancyGrid`.

## 5. Stage 2: localising and guiding the UGV

**Where the UGV is.** OpenCV finds the marker's four corners, and
`solvePnP` (IPPE_SQUARE) with the 0.446 m side gives the marker pose in the
camera. The same frame chain the mapper uses carries it into `map`: camera
optical -> camera link -> UAV body -> PX4 local NED -> `map` ENU, using the
PX4 pose at the image time. Heading is the marker's +x axis projected onto
the ground. Up to 15 poses per second; no depth camera involved.

**How the UAV keeps over it.** The mission sets the UGV's position (10 m up)
as the goal and moves a setpoint carrot towards it at up to 3 m/s. PX4's own
position controller does the rest; there is no extra PID. The UAV trails a
moving UGV by about 0.9 m because the carrot carries no UGV velocity.

**The path.** On entering TRACK, the follower takes the latest costmap:
road cells -> fill holes -> medial axis -> longest branch -> spline (within
about 10 cm of the skeleton) -> cut the corner forks at the two ends -> the
end nearer the UGV is the start. The far end is the goal. Published on
`/ugv/path`.

**How the UGV drives it.** Pure pursuit: chase the point 1.5 m ahead on the
path, curvature `k = 2 y / d^2` in the UGV frame, `omega = v k`. Rotate in
place when that point is more than 46 deg off the nose, slow down on curves
tighter than 2 m radius and inside the last 2 m, stop within 0.5 m of the
goal. Top speed 0.8 m/s.

## 6. Frames

| Frame | Axes | Origin |
|---|---|---|
| `world` | ENU | Gazebo world origin |
| `map` | ENU | PX4 EKF origin = UAV spawn point; `world` shifted, not rotated |
| PX4 local | NED | same origin as `map` (x north, y east, z down) |
| UAV body | FRD in PX4, FLU in `road_survey.frames` | UAV centre |
| camera optical | x right, y down, z along the view (down) | 0.12 m forward, 0.24 m up on the UAV |
| marker | x = UGV forward, y = UGV left, z up | roof, 0.127 m above the UGV `base_link` |

`map` <-> PX4 local: `x_map = y_ned, y_map = x_ned, z_map = -z_ned`,
`yaw_map = pi/2 - yaw_ned`.

## 7. Data that persists

| What | Where | Written by |
|---|---|---|
| road map (costmap, elevation, risk, road mask, counts, origin) | `maps/road_map.npz` (+ `.pgm`/`.yaml` for map_server, `.png` to look at) | `terrain_mapper`, on the save the mission requests at the end of the survey, and on Ctrl-C |
| breadcrumbs | in the mission node's memory only | `mission` |
| centre line | `/ugv/path` (latched), recomputed each run | `ugv_follower` |
| tunable values | `src/guidance/config/guidance.yaml`, `src/road_survey/config/road_survey.yaml` | you |

## 8. Safety behaviours

- The mission never force-arms. If PX4 refuses, a check is failing, and the
  node says how to find out which one.
- Setpoints never jump: the carrot moves at a bounded speed, so PX4 never
  sees a far-away target.
- If PX4 leaves offboard after the mission started, the node stops sending
  setpoints and reports ABORTED, which stops the UGV.
- The UGV moves only while it is seen: a pose older than 0.5 s, any state
  other than TRACK, or the follower shutting down sends a zero `Twist`.
  The DiffDrive plugin would otherwise keep the last command forever.
- `/ugv/ground_truth` exists for `pose_check` only; nothing in the control
  loop reads it.

## 9. How it is checked

- **Offline, no ROS:** `pytest src/road_survey/test src/guidance/test`. This
  covers frames, the mapper on a ray-cast synthetic road, the marker pose
  from synthetic renders, pure pursuit, the explorer, and a whole mission
  flown against a toy world (`test_mission_sim.py`).
- **In Gazebo:** `pose_check` prints the ArUco pose error, the heading
  correction, and the true UGV's deviation from the centre line while it
  drives. That last number is what the rubric scores.

## 10. Where the code is

```
src/road_survey/road_survey/   frames, depth, grid, risk, costmap, centerline, mapio  (pure)
                               terrain_mapper_node, map_publisher_node, height_slicer_node, tune_offline
src/guidance/guidance/         aruco, explore, mission, pursuit                        (pure)
                               mission_node, localizer_node, follower_node, pose_check_node, px4
src/bringup/launch/            sim.launch.py, mission.launch.py
src/bringup/config/            world_poses.yaml
src/drdo_gz_worlds/            worlds, x500_depth_down model, airframe 4022, camera bridge
src/ackermann_gz_bringup/      UGV model with the marker, spawn + bridge
```

"Pure" modules import no ROS, which is what lets the whole mission run in
the offline tests.
