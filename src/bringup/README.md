# bringup

Unified bringup package for the DRDO UAV-Guided UGV stack.

Contains ROS 2 launch files, URDF robot models, world spawn configurations, autonomous mission nodes, and simulation orchestration utilities.

## Package Structure

```
bringup/
├── launch/
│   ├── sim.launch.py            # Complete simulation bringup (Gazebo, Rover, PX4 UAV, bridges, URDF)
│   ├── full_system.launch.py    # Simulation + road_survey mapper + RViz
│   ├── survey.launch.py         # Road survey mapping node and RViz visualization
│   ├── survey_mission.launch.py # Autonomous UAV survey flight & map auto-save
│   ├── spawn_rover.launch.py    # Spawn Ackermann rover with URDF and robot_state_publisher
│   └── rviz.launch.py           # RViz visualization
├── urdf/
│   ├── ackermann_bot.urdf       # UGV rover URDF (chassis, 4 wheels, ArUco roof marker)
│   └── x500_depth_down.urdf     # Quadrotor UAV URDF (OakD-Lite downward depth camera)
├── config/
│   └── world_poses.yaml         # World spawn coordinates (drdo_world1, drdo_world2, drdo_world3)
├── bringup/
│   ├── uav_launcher.py          # Autonomous offboard arm, takeoff, and hover node
│   └── survey_mission.py        # Autonomous survey flight and real-time map verification
└── scripts/
    ├── bringup_sim.sh           # CLI simulation bringup script
    ├── setup_px4_uav.sh         # PX4 model & airframe installation script
    └── record_survey.sh         # Rosbag recording utility for offline tuning
```

## Quick Start

### 1. Launch Complete Simulation
```bash
ros2 launch bringup sim.launch.py world:=drdo_world2 gui:=true
```

### 2. Launch Survey Mapping & Visualization
```bash
ros2 launch bringup survey.launch.py world:=drdo_world2
```

### 3. Run Autonomous Survey Flight
```bash
ros2 run bringup survey_mission
# Or via launch:
ros2 launch bringup survey_mission.launch.py altitude:=12.0 forward_dist:=18.0
```

### 4. Or Launch Everything with One Command
```bash
ros2 launch bringup full_system.launch.py world:=drdo_world2
```
