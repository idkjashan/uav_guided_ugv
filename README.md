# uav_guided_ugv

Colcon workspace for the DRDO Inter-IIT "UAV-guided UGV" problem statement.
ROS 2 Humble · Gazebo Harmonic · PX4 v1.16.

The UGV carries no sensors. The UAV surveys the road from the air, builds a
costmap, and the rover is then navigated along it.

```
uav_guided_ugv/
├── src/road_survey/        stage 1 — road detection and costmap  (implemented)
├── docs/
│   ├── 01_road_survey_plan.md   how road detection works and why
│   ├── HANDOFF.md               integration notes: read before wiring it up
│   └── SYSTEM_ENVIRONMENT.md    the exact machine, versions and topics
└── scripts/record_survey.sh     bag the topics needed to replay a survey
```

## Stages

| | | |
|---|---|---|
| 1 | **Road survey** — fly the UAV manually, accumulate depth into a 2.5-D elevation grid, classify road from slope/step/roughness, publish an OccupancyGrid | implemented, tested offline, **not yet flown** |
| 2 | **UGV localisation** — ArUco marker on the rover roof, `solvePnP` against the UAV camera, UAV pose from PX4 ⇒ rover pose in the same `map` frame | not started |
| 3 | **Navigation** — Nav2 on the stage-1 costmap, `/cmd_vel` to the rover | not started |

## Build and test

```bash
cd ~/uav_guided_ugv
colcon build --packages-select road_survey --symlink-install
source install/setup.bash
```

The algorithm modules are pure numpy/scipy/cv2 and need neither ROS nor Gazebo
to test:

```bash
python3 -m pytest src/road_survey/test -q
```

47 tests, ~10 s, including an end-to-end simulated survey over a synthetic hill
road.

## Run a survey

With PX4 SITL, Gazebo and `MicroXRCEAgent` already running:

```bash
source ~/px4_ros_ws/install/setup.bash
ros2 launch road_survey survey.launch.py world:=drdo_world1
```

Fly the UAV **10–15 m above the road** (the depth sensor's far clip is 19.1 m —
higher than that and it returns nothing) along the whole route, then:

```bash
ros2 service call /terrain_mapper/save std_srvs/srv/Trigger
```

which writes `~/uav_guided_ugv/maps/road_map.{npz,pgm,yaml,png}`. Open the PNG:
an unbroken green ribbon with red flanks means it worked.

Tune the thresholds against the saved map instead of re-flying:

```bash
ros2 run road_survey tune_offline ~/uav_guided_ugv/maps/road_map.npz \
    --sweep slope 8 10 12 15 20
```

Republish a saved map for the Nav2 stage:

```bash
ros2 run road_survey map_publisher \
    --ros-args -p map_npz:=~/uav_guided_ugv/maps/road_map.npz -p use_sim_time:=true
```

## Dependencies

Everything is in the versions listed in `docs/SYSTEM_ENVIRONMENT.md`, plus
`px4_msgs` from `~/px4_ros_ws` (source it before running the nodes) and,
optionally, `python3-skimage` for centre-line extraction.
