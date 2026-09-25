# bringup

Launch files and world configuration for the DRDO UAV-guided UGV stack. No
nodes of its own: the mapper lives in `road_survey`, the mission, UGV
localiser and UGV follower in `guidance`.

```
bringup/
├── launch/
│   ├── sim.launch.py        # Gazebo world, UGV, MicroXRCEAgent, PX4 SITL UAV, camera bridge
│   └── mission.launch.py    # mapper + localiser + follower + mission + RViz
├── config/
│   └── world_poses.yaml     # UAV and UGV spawn poses for drdo_world1/2/3
└── scripts/
    ├── setup_px4_uav.sh     # install the x500_depth_down model and airframe 4022 into PX4
    └── record_survey.sh     # record the topics the mapper consumes
```

## Run

```bash
# terminal 1: simulation (wait for PX4 to print "Ready for takeoff!")
ros2 launch bringup sim.launch.py world:=drdo_world2

# terminal 2: survey the road, fly back, guide the UGV to the end of it
ros2 launch bringup mission.launch.py

# or, with a map from an earlier survey in ~/uav_guided_ugv/maps/road_map.npz
ros2 launch bringup mission.launch.py survey:=false
```

`sim.launch.py` also publishes a static `world -> map` transform (the UAV
spawn position). It exists so RViz and `pose_check` can compare the UGV pose
with Gazebo's ground truth; nothing in the mission uses it.

Spawn poses are read from `config/world_poses.yaml`, keyed by world name
without the `_overlay` suffix. Add a world there before launching it.
