# ackermann_gz_bringup

Spawns a basic 4-wheel **skid-steer ("tank drive")** vehicle into a
**Gazebo Harmonic** (`gz-sim`) world that is already running in another
terminal, and bridges it to ROS 2. The vehicle has an **ArUco marker**
(dictionary `DICT_4X4_50`, id `0`) mounted flat on its roof.

All 4 wheels are directly driven (no steering joints) — the left pair and
right pair spin at independently commanded speeds, and turning comes from
the speed difference between the two sides, exactly like a tank. Driving
uses gz-sim's built-in `DiffDrive` system plugin, which natively supports
multiple wheels per side — no `ros2_control` setup required. It's still
commanded through `/cmd_vel` the same way an Ackermann vehicle would be;
only the kinematics under the hood differ.

> Note: the package/model are still named `ackermann_gz_bringup` /
> `ackermann_bot` from an earlier iteration of this vehicle. Rename freely
> if you'd like the naming to match the tank-drive kinematics.

## Package layout

```
ackermann_gz_bringup/
├── launch/
│   └── spawn_ackermann.launch.py   # spawns the robot + starts the ROS<->GZ bridge
├── models/
│   └── ackermann_bot/
│       ├── model.config
│       ├── model.sdf               # chassis, wheels, steering joints, plugins, marker
│       └── materials/textures/
│           └── aruco_marker_0.png  # generated ArUco marker (4x4_50, id 0)
├── CMakeLists.txt
└── package.xml
```

## Build

Drop this folder into your workspace's `src/` and build:

```bash
cd ~/ros2_ws
colcon build --packages-select ackermann_gz_bringup
source install/setup.bash
```

Dependencies (install if missing): `ros_gz_sim`, `ros_gz_bridge` for your
ROS 2 distro paired with Gazebo Harmonic, e.g.:

```bash
sudo apt install ros-<distro>-ros-gz-sim ros-<distro>-ros-gz-bridge
```

## Run

**Terminal 1** — start your world as usual (this package does not launch gz
sim itself):

```bash
gz sim -r my_world.sdf
```

Note the `<world name="...">` value inside that SDF — that's the name to
pass below.

**Terminal 2** — spawn the tank-drive vehicle into it:

```bash
ros2 launch ackermann_gz_bringup spawn_ackermann.launch.py world:=my_world
```

Optional arguments:

| Argument     | Default        | Meaning                                   |
|--------------|----------------|--------------------------------------------|
| `world`      | `empty`        | Name of the running gz world               |
| `robot_name` | `ackermann_bot`| Model name / ROS topic namespace           |
| `x, y, z`    | `0 0 0.1`      | Spawn position                             |
| `yaw`        | `0.0`          | Spawn yaw (radians)                        |

## Drive it

The bridge exposes:

- `/cmd_vel` (`geometry_msgs/Twist`) → drives the vehicle. `linear.x` sets
  forward speed, `angular.z` sets turning rate; `DiffDrive` converts these
  into independent left-side / right-side wheel speeds (skid-steer), so a
  pure `angular.z` with zero `linear.x` spins the vehicle in place, like a
  tank.
- `/ugv/ground_truth` (`nav_msgs/Odometry`, frame `world`) ← the true model
  pose from the `OdometryPublisher` system. For checking the ArUco pose
  estimate only; the UGV is never controlled from it.

Wheel odometry and its `odom -> base_link` transform are still produced by
`DiffDrive`, on `/model/ackermann_bot/odometry` and `/model/ackermann_bot/tf`,
but are deliberately not bridged: the UGV is localised from its marker alone.
`/clock` is not bridged here either; the UAV camera bridge is the one clock
source. `DiffDrive` has no command timeout, so the vehicle keeps its last
`/cmd_vel` until told otherwise.

Example:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

## The ArUco marker

- Dictionary: `cv2.aruco.DICT_4X4_50`, marker id `0`, baked into
  `materials/textures/aruco_marker_0.png` with a white quiet-zone border.
- It's a `0.55 x 0.55 m` `<plane>` visual on `base_link`, 2 mm above the roof
  (0.127 m above `base_link`). The PNG has a 60 px white border round a
  512 px marker, so the black square, which is the marker length OpenCV
  needs, is `0.55 * 512 / 632 = 0.446 m`.
- A plane maps the texture straight onto the link axes: image right is +X
  (forward), image up is +Y. The previous 0.15 m box was about 12 px across
  from 12 m up, too small to decode reliably.
- To detect it from an external/overhead camera with OpenCV:

  ```python
  import cv2
  aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
  detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
  corners, ids, _ = detector.detectMarkers(gray_image)
  ```

- To use a different marker id/dictionary, regenerate the PNG and swap the
  file at the same path (keep the filename or update the `albedo_map` path
  in `model.sdf`).

## Tuning / customizing

Vehicle geometry (wheelbase `0.9 m`, track `0.7 m`, wheel radius `0.15 m`,
chassis `1.0 x 0.6 x 0.25 m`) and drive limits are all set at the top of
`model.sdf` inside the `<link>`/`<joint>` poses and the `DiffDrive`
`<plugin>` block — edit those values to match a different vehicle size.
`wheel_separation` in the plugin should match the left/right wheel track
(`0.7 m` here) for accurate odometry.

The exact `DiffDrive` plugin tag set can vary slightly between gz-sim /
Harmonic point releases. If a parameter is rejected or ignored on your
install, run `gz sim -v 4` for plugin load warnings and cross-check against
`gz sim --versions` / the installed `ros_gz` docs for your version.

## RViz

The UGV shows up in RViz as the `/ugv/pose` arrow from the `guidance`
package's ArUco localiser, with `/ugv/ground_truth` next to it for
comparison.
