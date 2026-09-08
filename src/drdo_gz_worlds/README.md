# drdo_gz_worlds

Gazebo Harmonic (gz-sim) port of the terrain worlds from
[`interiit22`](https://github.com/phoenixrider12/DRDO-UAV-Guided-UGV-Navigation)
(originally ROS1 Noetic + Gazebo Classic 11). Built for ROS 2 Humble.

## What changed vs. the original `interiit22/world` + `interiit22/models`

- **Terrain models (`models/world1`, `world1_mesh`, `world2`, `world2_mesh`,
  `world3`) — untouched.** SDF's model/link/collision/visual/mesh elements
  parse identically across Gazebo Classic and gz-sim; there was nothing
  version-specific to change here. Same `.dae` meshes, same textures, same
  `<scale>` values, copied verbatim.
- **World files (`worlds/*.sdf`) — rewritten.** Three things Gazebo Classic
  did implicitly that gz-sim requires explicitly:
  1. `<plugin>` tags loading the gz-sim systems (Physics, UserCommands,
     SceneBroadcaster, Sensors, Imu, Contact) — Classic had these built in.
  2. The `<gui><camera>` block replaced with gz-sim's GUI plugin stack
     (`MinimalScene`, `WorldControl`, etc.), seeded with the original
     `user_camera` pose values so the default view matches.
  3. The embedded `<model name="iris"><include>...` UAV block **removed**.
     Per your direction, the UAV and UGV are spawned as separate models
     rather than baked into the world.
  - Everything else (lights, gravity, magnetic_field, physics params,
    spherical_coordinates, scene) is copied unchanged — `type="ode"` in the
    `<physics>` tag is left as-is; gz-sim ignores that attribute rather
    than erroring on it (this is also what ArduPilot's own `iris_runway.sdf`
    example world does).
  - `drdo_world1_overlay` / `drdo_world2_overlay` (the untextured variants
    used for UGV-only runs after mapping) had their terrain embedded
    inline in the original; here they just `<include>` the existing
    `world1_mesh` / `world2_mesh` models instead — same result, less
    duplication. Their saved `<state>` blocks (a simulation snapshot) were
    dropped as redundant.
  - `drdo_world3` has no untextured overlay in the source repo either.

## Not included here (by design, per your scope)

- The UAV model (iris + gimbal + depth camera) and UGV model (Prius) —
  you said these are being handled as separate models. `launch/world.launch.py`
  has spawn hooks (`uav_sdf`, `ugv_sdf`, and pose args) ready for whenever
  those exist; leave them blank to just load an empty terrain.
- ArduPilot SITL / MAVROS bring-up — pair this with
  [`ArduPilot/ardupilot_gz`](https://github.com/ArduPilot/ardupilot_gz),
  which is the officially maintained ROS 2 Humble + Gazebo Harmonic
  integration for ArduPilot.
- Sensor plugins (depth camera, IMU, GPS) that lived on the old
  `drone_with_depth_camera` / `gimbal_small_2d` models — those move with
  the UAV model itself, not the world, so they're out of scope here.

## Build

```bash
cd ~/ros2_ws/src
cp -r drdo_gz_worlds .
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select drdo_gz_worlds
source install/setup.bash
```

## Run

```bash
export GZ_VERSION=harmonic

# Textured world, mapping run
ros2 launch drdo_gz_worlds world.launch.py world:=drdo_world1

# Untextured world, UGV-only navigation run
ros2 launch drdo_gz_worlds world.launch.py world:=drdo_world1_overlay

# Once you have UAV/UGV SDF models ready:
ros2 launch drdo_gz_worlds world.launch.py world:=drdo_world1 \
    uav_sdf:=/path/to/iris.sdf \
    uav_x:=-10.226 uav_y:=311.831 uav_z:=22.863 \
    uav_roll:=0.011338 uav_pitch:=0.135709 uav_yaw:=-2.161422 \
    ugv_sdf:=/path/to/prius.sdf \
    ugv_x:=-12.220319 ugv_y:=308.976703 ugv_z:=22.295580 \
    ugv_roll:=0.011338 ugv_pitch:=0.135709 ugv_yaw:=-2.161422
```

`world2` / `world3` and their reference spawn poses are documented in the
docstring at the top of `launch/world.launch.py`.

## Caveat

This was built and XML-validated in a sandbox without Gazebo Harmonic or
ROS 2 Humble installed, so it hasn't been run in a live `gz sim` session.
The structure and plugin set follow the standard gz-sim world template
(matching ArduPilot's own Harmonic example worlds), but budget time for the
usual first-run friction — missing system plugin, a resource path typo —
when you build this for real.
