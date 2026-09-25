# Handoff: bringing stage 2 up in Gazebo

For whoever runs this on the Ubuntu machine (ROS 2 Humble, Gazebo Harmonic,
PX4 at `~/PX4-Autopilot`). Everything below was written and tested offline
without ROS; none of it has flown yet. Read `02_guidance.md` for why the code
is shaped this way, and `01_road_survey_plan.md` for the mapper.

## What changed in this round

- New package `src/guidance`: UAV mission (takeoff, autonomous road survey,
  return, track the UGV), ArUco UGV localiser, pure-pursuit UGV follower,
  `pose_check` validation tool.
- `bringup` is now launch files only: `sim.launch.py` (rewritten) and
  `mission.launch.py` (new). The old `survey_mission` / `uav_launcher` nodes
  and their launch files are gone. They force-armed PX4 past failing checks,
  jumped position setpoints tens of metres, and left unused setpoint fields at
  0.0 instead of NaN.
- UGV model: the marker is now a 0.55 m plane (black square 0.446 m);
  wheel odometry and `/tf` are no longer bridged; `/ugv/ground_truth` is
  bridged for validation only; `/clock` comes from the UAV bridge alone.
- Worlds: physics step 1 ms -> 4 ms (PX4's default; 4x less CPU).
- UAV bridge: the depth point cloud is no longer bridged (nothing used it).
- `road_survey/centerline.py`: spline smoothing fixed (it cut bends by up to
  1 m). No other stage 1 change.

## 0. Build and test offline

```bash
cd ~/uav_guided_ugv && git pull
python3 -c "import skimage" || pip install --user "scikit-image<0.25"   # centre line
rm -rf build/bringup install/bringup          # drop the old bringup python package
colcon build --symlink-install
source install/setup.bash && source ~/px4_ros_ws/install/setup.bash
PYTHONPATH=src/road_survey:src/guidance python3 -m pytest src/road_survey/test src/guidance/test -q
```

Expect `66 passed`. If `test_mission_sim.py` fails here but passed in WSL,
stop and report it; it flies the whole mission logic.

## 1. Simulation comes up

```bash
ros2 launch bringup sim.launch.py world:=drdo_world2
```

Wait for PX4's `Ready for takeoff!`, then in another terminal:

```bash
ros2 topic info /clock                     # Publisher count: 1
ros2 topic list | grep fmu/out             # note the names, see below
ros2 topic hz /uav/rgb                     # ~30 Hz (less if RTF < 1)
ros2 topic hz /uav/depth
ros2 topic echo /ugv/ground_truth --once
```

This PX4 build publishes `/fmu/out/vehicle_status_v1` and
`/fmu/out/vehicle_local_position_v1` (versioned), but `vehicle_attitude`
without a suffix. The nodes subscribe to both spellings, so either works.
If neither form of a topic exists, the agent or `px4_msgs` is the problem,
not the nodes.

Check in the Gazebo GUI that the UGV has the large marker on its roof and
note the real-time factor (bottom right).

## 2. UAV and localiser only, UGV parked

No survey, no follower: the UAV takes off, finds the UGV and holds over it.

```bash
P=$(ros2 pkg prefix guidance)/share/guidance/config/guidance.yaml
ros2 run guidance ugv_localizer --ros-args --params-file $P -p use_sim_time:=true
ros2 run guidance pose_check --ros-args -p use_sim_time:=true
ros2 run guidance mission --ros-args --params-file $P -p use_sim_time:=true -p survey:=false
```

Expected from `mission`: `state ARMING` -> `TAKEOFF` -> `ACQUIRE` ->
`TRACK` within about 20 s, a smooth 1 m/s climb, then a steady hover about
10 m above the UGV. From `ugv_localizer`, every 5 s:
`frames N, marker seen M, poses published M`, with M close to 75 (15 Hz)
once overhead. From `pose_check`: xy error in centimetres.

**Calibrate the heading once.** `pose_check` prints
`mean heading error +X deg (subtract R rad from yaw_offset_rad)`. It should
be near 0; if it is near ±90 or 180 the texture sits rotated on the plate.
Put the corrected value in `src/guidance/config/guidance.yaml`
(`yaw_offset_rad`; with `--symlink-install` no rebuild is needed) and
restart the localiser. Always write floats with a decimal point (`0.0`,
`3.0`): Humble refuses a YAML integer for a float parameter and the node
exits.

Then drive the UGV by hand and watch the UAV follow:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.1}}"
# Ctrl-C, then stop it explicitly: DiffDrive keeps the last command forever
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

## 3. Full mission

Restart the simulation (the UAV must start on the ground at its spawn), then:

```bash
ros2 launch bringup mission.launch.py
ros2 run guidance pose_check --ros-args -p use_sim_time:=true   # optional, alongside
```

Expected: `TAKEOFF`, then `SURVEY` along the road at about 1.5 m/s and 12 m
above it, `survey finished (road ended); saving map`, `RETURN` along the same
track, `ACQUIRE`, `TRACK`. Then the follower logs
`centre line L m, goal (x, y)` and the UGV drives to the end of the road with
the UAV above it, ending with `goal reached: end of the road`. RViz shows the
costmap, the magenta centre line, the red ArUco pose arrow and the green
ground-truth arrow.

To re-run stage 2 without surveying again:

```bash
ros2 launch bringup mission.launch.py survey:=false   # uses maps/road_map.npz
```

## 4. When something is wrong

| Symptom | Likely cause | Check / fix |
|---|---|---|
| `waiting for /fmu/out/vehicle_status(_v1)` forever | no `/fmu/out` topics, or `px4_msgs` mismatch | `ros2 topic list \| grep fmu`; MicroXRCEAgent running |
| `waiting for a valid PX4 local position` forever | EKF not initialised yet, or local position under another name | wait ~10 s after boot; `ros2 topic echo /fmu/out/vehicle_local_position_v1 --once` |
| stuck in `ARMING`, warning every 10 s | a PX4 preflight check fails | `cd ~/PX4-Autopilot/build/px4_sitl_default && ./bin/px4-commander check` and `./bin/px4-listener failsafe_flags`. `NAV_DLL_ACT` must be 0 (airframe 4022 sets it; a saved parameter can override: `./bin/px4-param set NAV_DLL_ACT 0`). Do not force-arm |
| `ABORTED` | PX4 left offboard (failsafe, RC, QGC) | PX4 console says why; restart the mission node |
| `marker seen 0` with the UGV in view | marker too small (old model still loaded) or badly lit | rebuild `ackermann_gz_bringup`, restart Gazebo; look at `/uav/rgb` in `rqt_image_view` |
| `no PX4 pose near the image time` | `use_sim_time` missing somewhere | every node needs `use_sim_time:=true` |
| `pose_check` xy error > 0.5 m | wrong `marker_length_m` or pose/image timing | must be 0.446; report the numbers |
| survey ends at once, `road ended` | heading points away from the road, or no costmap | `ros2 topic echo /road/stats`; set `heading_enu` (rad, ENU) |
| survey leaves the road | stage 1 classification | tune with `tune_offline` (see `01_road_survey_plan.md`) |
| UGV never moves in TRACK | no centre line (scikit-image missing) or costmap missing | follower log; `python3 -c "import skimage"` |
| UGV turns in place and never drives | heading off by about 90 or 180 degrees | calibrate `yaw_offset_rad` (step 2) |
| sim time jumps backwards, TF warnings | two `/clock` publishers | `ros2 topic info /clock` must show 1 |

## 5. What to send back

1. `ros2 topic list | grep fmu` output.
2. `pose_check` lines with the UGV parked and while driving, and the
   `yaw_offset_rad` you settled on.
3. The mission node's log from launch to `TRACK` (state lines, survey end
   reason, map save response).
4. `maps/road_map.png`.
5. A bag of the guided drive, from which the rubric metric can be computed:
   ```bash
   ros2 bag record -o ~/uav_guided_ugv/bags/guided /ugv/pose /ugv/ground_truth \
     /ugv/cross_track_error /ugv/path /mission/state /cmd_vel /road/costmap /clock
   ```
6. Gazebo's real-time factor during the run.

## Interfaces

| Topic | Type | Frame | From |
|---|---|---|---|
| `/road/costmap` | `nav_msgs/OccupancyGrid` (transient local) | `map` | `terrain_mapper` / `map_publisher` |
| `/ugv/pose` | `geometry_msgs/PoseStamped`, stamp = image time | `map` | `ugv_localizer` |
| `/mission/state` | `std_msgs/String` (transient local) | | `mission` |
| `/ugv/path` | `nav_msgs/Path` (transient local) | `map` | `ugv_follower` |
| `/ugv/cross_track_error` | `std_msgs/Float32`, m | | `ugv_follower` |
| `/cmd_vel` | `geometry_msgs/Twist` | UGV body | `ugv_follower` |
| `/ugv/ground_truth` | `nav_msgs/Odometry`, validation only | `world` | Gazebo |

`map` is PX4's local ENU with its origin at the UAV spawn; `world -> map` is
the UAV spawn position with no rotation (static transform from
`sim.launch.py`).

## House rules

- `aruco.py`, `explore.py`, `mission.py`, `pursuit.py` and the `road_survey`
  modules listed in `01_road_survey_plan.md` are pure: numpy/scipy/cv2, no
  ROS. Keep them that way; it is what lets the whole mission be tested
  offline.
- Any change to them must keep both test suites green, `test_mission_sim.py`
  above all.
- Tunable values go in `config/guidance.yaml` / `config/road_survey.yaml`.
- Nothing in the control loop may read `/ugv/ground_truth`.
