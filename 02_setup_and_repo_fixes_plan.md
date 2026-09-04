# Plan 2: Toolchain setup, Training_Pool bring-up, repo fixes

Purpose: Harmonic + bridge + PX4 v1.16.x + px4_msgs/agent + Training_Pool working; fixes B1, B4a, B2, B3, README on branch `fixes`; `bringup.sh <world>`.
Inputs: Plan 1 STATE facts if present; user at the machine.
Outputs: `~/uav_guided_ugv/setup/env.sh`, `~/uav_guided_ugv/setup/bringup.sh`, `~/uav_guided_ugv/setup/reports/02_setup_report.md`.
Orchestrator turn budget: none on Gemini CLI (1,500 requests/day); 12 if on a 50/day orchestrator. Dispatch one card at a time.

## Orchestrator

Rules
1. Never run commands; update STATE first every turn.
2. Fill every `{{PLACEHOLDER}}` from Facts/reports; never invent poses.
3. `free_gb < 15` -> stop, send user to Plan 1.
4. FAILED/BLOCKED -> ask user; never re-dispatch an unchanged card.
5. Batch open user questions into one message.
6. Long processes are backgrounded with logs in `~/uav_guided_ugv/setup/logs/`; later cards read them.
7. After each verified fix, dispatch T33 with a one-line log entry (batch with the next fix card).
8. Verify = re-dispatch T13/T14/T15/T16/T17/T19/T20/T22 with new inputs.

State keys
`free_gb gz_version bridge_pkgs ros_gz_pkg px4_tag px4_msgs_branch agent_version training_pool_commit scale_warning z_start z_end rtf_world1..3 fmu_topics camera_topics uav_model fixes_commits known_issues`

Decision tree
```
N0 facts have free_gb+gz_version? yes -> N1 ; no -> T1 -> N1
N1 free_gb<15 -> [ASK USER] run Plan 1, stop ; else -> N2
N2 gz_version 8.x -> N3 ; other/absent -> T2 (+T3 REMOVE_PKGS=old gz + bridge_pkgs, [ASK USER] first) -> N3
N3 bridge_pkgs only ros-humble-ros-gzharmonic* -> N4 ; Fortress ros-humble-ros-gz* -> [ASK USER] then T3 ; empty -> T4
N4 px4_tag v1.16.* -> N5 ; absent -> T6a ; other -> T6b (PX4_TAG=v1.16.2)
N5 batch{T5, T7, T9 (PX4_MSGS_BRANCH=release/1.16), T11 (skip if tp_present=yes)}
   T5 not 8.x or ros_gz_pkg empty -> N2 ; T9 FAILED with 'fastdds' in raw -> T10
N6 batch{T8, T12}; ready_count 0 or no drone -> [ASK USER], stop ; else N7
N7 T13 WORLD=drdo_world2 ; scale_warning=yes or no terrain -> B4a needed ; -> N8
N8 batch{T14 (world2 UGV), T16 (UAV_MODEL=gz_x500, world2 UAV), T17}
   |z_end-UGV_Z|>1 -> B1 confirmed (expected) ; ready 0 or fmu_topics 0 -> [ASK USER]
N9 batch{T15, T18, T19} ; T20 with T19 topics ; user confirms image -> N10
N10 T21 -> per world 1/2/3: T13+T14+T15+T22 ; z within 1 m, drives ; rtf_world3<0.3 -> T23, re-verify ; fail -> [ASK USER]
N11 B4a needed -> T24 -> T13 world2, user confirms terrain ; else N12
N12 T25 -> T13 WORLD=drdo_world3_overlay + T14 world3 + T22 ; plain terrain confirmed -> N13
N13 T27 -> T13 world2 + T16 (UAV_MODEL=gz_x500_depth_down) + T17 + T19 + T20 ; no ground in image -> T28, repeat ; -> N14
N14 T30 -> T31 (suffixes from T19) -> T32 ; 5 signals -> N15 ; else [ASK USER]
N15 T33 with the Report template -> Done
```

Ask the user when
- A card needs sudo; any `needs: gui` question; any BLOCKED/FAILED.
- Before T3 (show the dry-run list); free_gb < 15; T8 shows no drone.

Done when: T32 shows all five signals and 02_setup_report.md has every setup-facts key.

## Task cards

### T1 sense  |  tier: flash  |  needs: none
Goal: Inventory disk, Gazebo, bridge, PX4, workspaces.
Inputs: none
Run:
```bash
df -h / | tail -1
gz sim --versions 2>/dev/null || echo NO_GZ
ls /opt/ros
dpkg -l | grep -E 'ros-humble-ros-gz|^ii  gz-|gazebo11' | awk '{print $2}'
git -C ~/PX4-Autopilot describe --tags 2>/dev/null || echo NO_PX4
ls -d ~/training_pool/install ~/px4_ros_ws/install 2>&1
```
Expect: all lines print (NO_* is fine).
Stop if: `df` fails.
Return: free_gb, gz_version (8.x|other|absent), bridge_pkgs, px4_tag, tp_present (yes|no), ws_present (yes|no)

### T2 install-gz-harmonic  |  tier: flash  |  needs: sudo
Goal: Add OSRF repo, install gz-harmonic.
Inputs: none
Run:
```bash
sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install -y gz-harmonic
gz sim --versions
```
Expect: last line `8.x.y`.
Stop if: any line starting with `E:` from apt.
Return: gz_version

### T3 swap-bridge  |  tier: flash  |  needs: sudo
Goal: Remove conflicting packages, install the Harmonic bridge.
Inputs: {{REMOVE_PKGS}} (space-separated apt names)
Run:
```bash
sudo apt-get -s remove {{REMOVE_PKGS}} | sed -n '/REMOVED/,/^[0-9]/p'
sudo apt-get remove -y {{REMOVE_PKGS}}
sudo apt-get install -y ros-humble-ros-gzharmonic
dpkg -l | grep ros-humble-ros-gz | awk '{print $2}'
```
Expect: only ros-humble-ros-gzharmonic* packages listed.
Stop if: dry-run REMOVED list contains ros-humble-desktop or ros-humble-ros-base (stop BEFORE line 2); any `E:` line.
Return: bridge_pkgs

### T4 install-bridge  |  tier: flash  |  needs: sudo
Goal: Install the Harmonic bridge.
Inputs: none
Run:
```bash
sudo apt-get install -y ros-humble-ros-gzharmonic
dpkg -l | grep ros-humble-ros-gz | awk '{print $2}'
```
Expect: list includes ros-humble-ros-gzharmonic.
Stop if: any `E:` line.
Return: bridge_pkgs

### T5 verify-gz  |  tier: flash  |  needs: none
Goal: Confirm Harmonic + bridge visible to ROS.
Inputs: none
Run:
```bash
gz sim --versions
source /opt/ros/humble/setup.bash
ros2 pkg list | grep ros_gz
```
Expect: `8.x.y`; ros_gz_sim, ros_gz_bridge listed.
Stop if: nothing.
Return: gz_version, ros_gz_pkg (comma list)

### T6a px4-clone  |  tier: flash  |  needs: none
Goal: Clone PX4 at the release tag.
Inputs: {{PX4_TAG}}
Run:
```bash
git clone --recursive -b {{PX4_TAG}} https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
git -C ~/PX4-Autopilot describe --tags
```
Expect: last line = {{PX4_TAG}}.
Stop if: git prints `fatal`.
Return: px4_tag

### T6b px4-checkout  |  tier: flash  |  needs: none
Goal: Checkout the release tag in an existing clone.
Inputs: {{PX4_TAG}}
Run:
```bash
cd ~/PX4-Autopilot
git status --short | head
git fetch --tags
git checkout {{PX4_TAG}}
git submodule update --init --recursive
git describe --tags
```
Expect: last line = {{PX4_TAG}}.
Stop if: `git status` shows modified files (M) or checkout prints `error`.
Return: px4_tag

### T7 px4-deps  |  tier: flash  |  needs: sudo
Goal: PX4 toolchain; re-check Gazebo version.
Inputs: none
Run:
```bash
cd ~/PX4-Autopilot
bash Tools/setup/ubuntu.sh --no-nuttx
gz sim --versions
```
Expect: script ends without `E:`; last line `8.x.y`.
Stop if: any `E:` line or gz version not 8.x.
Return: gz_version

### T8 px4-build-smoke  |  tier: flash  |  needs: gui
Goal: Build SITL, run stock gz_x500 once.
Inputs: none
Run:
```bash
mkdir -p ~/uav_guided_ugv/setup/logs ~/uav_guided_ugv/setup/reports
cd ~/PX4-Autopilot
make px4_sitl > ~/uav_guided_ugv/setup/logs/px4_build.log 2>&1
tail -3 ~/uav_guided_ugv/setup/logs/px4_build.log
nohup make px4_sitl gz_x500 > ~/uav_guided_ugv/setup/logs/px4_smoke.log 2>&1 &
sleep 90
grep -c 'Ready for takeoff' ~/uav_guided_ugv/setup/logs/px4_smoke.log
```
Expect: count >= 1; Gazebo window with a quadrotor.
Stop if: build log ends with `Error`.
Return: ready_count; status NEEDS_USER "Is a drone visible in Gazebo?"

### T9 px4-ros-ws  |  tier: flash  |  needs: none
Goal: Build px4_msgs + Micro-XRCE-DDS-Agent.
Inputs: {{PX4_MSGS_BRANCH}}
Run:
```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/px4_ros_ws/src
cd ~/px4_ros_ws/src
git clone -b {{PX4_MSGS_BRANCH}} https://github.com/PX4/px4_msgs.git
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd ~/px4_ros_ws
colcon build > ~/uav_guided_ugv/setup/logs/px4_ros_ws_build.log 2>&1
tail -3 ~/uav_guided_ugv/setup/logs/px4_ros_ws_build.log
find install -name MicroXRCEAgent
```
Expect: `Summary: 2 packages finished`; MicroXRCEAgent found.
Stop if: Summary shows failed packages.
Return: px4_msgs_branch, agent_version (v2.4.2)

### T10 fastdds-tag-fix  |  tier: flash  |  needs: none
Goal: Pin the FastDDS tag and rebuild the agent.
Inputs: none
Run:
```bash
cd ~/px4_ros_ws/src/Micro-XRCE-DDS-Agent
grep -n '_fastdds_tag' CMakeLists.txt
sed -i 's/set(_fastdds_tag 2.12.x)/set(_fastdds_tag v2.12.1)/' CMakeLists.txt
grep -n '_fastdds_tag' CMakeLists.txt
cd ~/px4_ros_ws
colcon build > ~/uav_guided_ugv/setup/logs/px4_ros_ws_build2.log 2>&1
tail -3 ~/uav_guided_ugv/setup/logs/px4_ros_ws_build2.log
```
Expect: second grep shows `v2.12.1`; 2 packages finished.
Stop if: first grep is empty.
Return: agent_version

### T11 training-pool-build  |  tier: flash  |  needs: sudo
Goal: Clone and build Training_Pool (two packages).
Inputs: none
Run:
```bash
source /opt/ros/humble/setup.bash
git clone https://github.com/abhinavakalita1/Training_Pool.git ~/training_pool
cd ~/training_pool
git rev-parse --short HEAD
rosdep update
rosdep install --from-paths src --ignore-src -y --simulate --skip-keys "ros_gz_sim ros_gz_bridge ros_gz_image ros_gz_interfaces"
rosdep install --from-paths src --ignore-src -y --skip-keys "ros_gz_sim ros_gz_bridge ros_gz_image ros_gz_interfaces"
colcon build --packages-select drdo_gz_worlds ackermann_gz_bringup > ~/uav_guided_ugv/setup/logs/tp_build.log 2>&1
tail -3 ~/uav_guided_ugv/setup/logs/tp_build.log
ls install/drdo_gz_worlds/share/drdo_gz_worlds/worlds
```
Expect: Summary: 2 packages finished; 5 world files listed.
Stop if: --simulate output contains `ros-humble-ros-gz-` (stop BEFORE the real rosdep line); any `E:` line.
Return: training_pool_commit

### T12 write-env-sh  |  tier: flash  |  needs: none
Goal: Create the shared environment file.
Inputs: none
Run:
```bash
mkdir -p ~/uav_guided_ugv/setup/logs ~/uav_guided_ugv/setup/reports
cat > ~/uav_guided_ugv/setup/env.sh <<'EOF'
source /opt/ros/humble/setup.bash
source ~/px4_ros_ws/install/setup.bash
source ~/training_pool/install/setup.bash
[ -f ~/uav_guided_ugv/ws/install/setup.bash ] && source ~/uav_guided_ugv/ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds:$HOME/training_pool/install/drdo_gz_worlds/share/drdo_gz_worlds/models:${GZ_SIM_RESOURCE_PATH:-}
EOF
bash -c 'source ~/uav_guided_ugv/setup/env.sh && ros2 pkg prefix drdo_gz_worlds && ros2 pkg prefix px4_msgs'
```
Expect: two install prefixes.
Stop if: `ros2 pkg prefix` errors.
Return: none

### T13 launch-world  |  tier: flash  |  needs: gui
Goal: Kill old sim processes, launch a world in background.
Inputs: {{WORLD}}
Run:
```bash
source ~/uav_guided_ugv/setup/env.sh
pkill -f 'gz sim' ; pkill -f px4_sitl_default/bin/px4 ; pkill -f MicroXRCEAgent ; pkill -f parameter_bridge ; sleep 3
nohup ros2 launch drdo_gz_worlds world.launch.py world:={{WORLD}} > ~/uav_guided_ugv/setup/logs/gz_{{WORLD}}.log 2>&1 &
sleep 40
grep -i 'scale' ~/uav_guided_ugv/setup/logs/gz_{{WORLD}}.log | head -3
gz topic -l | grep -c '/world/{{WORLD}}/'
```
Expect: topic count > 0; Gazebo window open.
Stop if: log contains `process has died`.
Return: scale_warning (yes|no), world_topics; status NEEDS_USER "Terrain visible in {{WORLD}}?"

### T14 spawn-rover  |  tier: flash  |  needs: none
Goal: Spawn the rover, sample /odom z.
Inputs: {{WORLD}} {{UGV_X}} {{UGV_Y}} {{UGV_Z}} {{UGV_YAW}}
Run:
```bash
source ~/uav_guided_ugv/setup/env.sh
pkill -f spawn_ackermann ; sleep 1
nohup ros2 launch ackermann_gz_bringup spawn_ackermann.launch.py world:={{WORLD}} x:={{UGV_X}} y:={{UGV_Y}} z:={{UGV_Z}} yaw:={{UGV_YAW}} > ~/uav_guided_ugv/setup/logs/rover_{{WORLD}}.log 2>&1 &
sleep 10
timeout 3 ros2 topic echo /odom --field pose.pose.position.z | head -1
sleep 5
timeout 3 ros2 topic echo /odom --field pose.pose.position.z | head -1
```
Expect: two numbers within 1.0 of {{UGV_Z}}.
Stop if: no number printed.
Return: z_start, z_end

### T15 teleop-check  |  tier: flash  |  needs: gui
Goal: Drive the rover forward for 5 s via /cmd_vel.
Inputs: none
Run:
```bash
source ~/uav_guided_ugv/setup/env.sh
timeout 3 ros2 topic echo /odom --field pose.pose.position | head -3
timeout 5 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"
timeout 3 ros2 topic echo /odom --field pose.pose.position | head -3
```
Expect: x/y differ by > 1 m between samples.
Stop if: first echo prints nothing.
Return: pos_before, pos_after; status NEEDS_USER "Did the rover drive on the road?"

### T16 px4-standalone-spawn  |  tier: flash  |  needs: none
Goal: Spawn PX4 UAV into the running world.
Inputs: {{WORLD}} {{UAV_MODEL}} {{UAV_POSE}} (x,y,z,roll,pitch,yaw comma-separated)
Run:
```bash
source ~/uav_guided_ugv/setup/env.sh
pkill -f px4_sitl_default/bin/px4 ; sleep 2
cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_SIM_MODEL={{UAV_MODEL}} PX4_GZ_WORLD={{WORLD}} PX4_GZ_MODEL_POSE="{{UAV_POSE}}" nohup ./build/px4_sitl_default/bin/px4 -d > ~/uav_guided_ugv/setup/logs/px4_{{WORLD}}.log 2>&1 &
sleep 45
grep -c 'Ready for takeoff' ~/uav_guided_ugv/setup/logs/px4_{{WORLD}}.log
gz model --list | grep -i x500
```
Expect: count >= 1; model listed as `{{UAV_MODEL}}_0` (no gz_ prefix).
Stop if: log contains `ERROR`.
Return: ready_count, uav_model_name

### T17 start-agent  |  tier: flash  |  needs: none
Goal: Start the XRCE agent, count /fmu topics.
Inputs: none
Run:
```bash
source ~/uav_guided_ugv/setup/env.sh
pkill -f MicroXRCEAgent ; sleep 1
nohup MicroXRCEAgent udp4 -p 8888 > ~/uav_guided_ugv/setup/logs/agent.log 2>&1 &
sleep 15
grep -c -i 'session established' ~/uav_guided_ugv/setup/logs/agent.log
ros2 topic list | grep -c /fmu/
ros2 topic list | grep vehicle_local_position
```
Expect: session count >= 1; fmu topics > 20.
Stop if: `MicroXRCEAgent: command not found`.
Return: session_established, fmu_topics, local_position_topic

### T18 takeoff-check  |  tier: flash  |  needs: gui
Goal: Arm and take off via the PX4 client.
Inputs: {{LOCAL_POSITION_TOPIC}}
Run:
```bash
~/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander takeoff
sleep 20
source ~/uav_guided_ugv/setup/env.sh
timeout 3 ros2 topic echo {{LOCAL_POSITION_TOPIC}} --field z | head -1
```
Expect: z about -2.5 (NED, negative = above start).
Stop if: `px4-commander` prints `not running`.
Return: uav_z; status NEEDS_USER "Did the drone lift off in Gazebo?"

### T19 discover-camera-topics  |  tier: flash  |  needs: none
Goal: List Gazebo camera topics of the spawned UAV.
Inputs: none
Run:
```bash
source ~/uav_guided_ugv/setup/env.sh
gz topic -l | grep -i -E 'image|depth|camera_info|points'
```
Expect: lines like `/world/<w>/model/<m>/link/camera_link/sensor/<s>/image`.
Stop if: nothing.
Return: rgb_topic, depth_topic, info_topic, points_topic (`absent` if missing)

### T20 camera-bridge  |  tier: flash  |  needs: gui
Goal: Bridge four camera topics; view RGB.
Inputs: {{RGB_TOPIC}} {{DEPTH_TOPIC}} {{INFO_TOPIC}} {{POINTS_TOPIC}}
Run:
```bash
source ~/uav_guided_ugv/setup/env.sh
pkill -f parameter_bridge ; sleep 1
nohup ros2 run ros_gz_bridge parameter_bridge {{RGB_TOPIC}}@sensor_msgs/msg/Image@gz.msgs.Image {{DEPTH_TOPIC}}@sensor_msgs/msg/Image@gz.msgs.Image {{INFO_TOPIC}}@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo {{POINTS_TOPIC}}@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked > ~/uav_guided_ugv/setup/logs/bridge.log 2>&1 &
sleep 5
timeout 8 ros2 topic hz {{RGB_TOPIC}}
nohup ros2 run rqt_image_view rqt_image_view {{RGB_TOPIC}} > /dev/null 2>&1 &
```
Expect: average rate printed.
Stop if: bridge.log contains `Unrecognized`.
Return: rgb_hz; status NEEDS_USER "Does rqt_image_view show an image? Ground visible?"

### T21 fix-B1  |  tier: pro  |  needs: none
Goal: Mesh terrain collision in every worldN model.sdf; commit on branch fixes.
Inputs: none
Run:
```bash
cd ~/training_pool
git config user.name drdo
git config user.email drdo@local
git checkout -b fixes
grep -c '<box>' src/drdo_gz_worlds/models/world*/model.sdf
python3 - <<'EOF'
import re,glob
for f in sorted(glob.glob('src/drdo_gz_worlds/models/world*/model.sdf')):
    s=open(f).read()
    u=re.search(r"<uri>(model://\w+/meshes/[\w.]+)</uri>",s).group(1)
    sc=re.search(r"<scale>([\d. ]+)</scale>",s).group(1)
    new=f"<collision name='collision'><geometry><mesh><scale>{sc}</scale><uri>{u}</uri></mesh></geometry></collision>"
    s2=re.sub(r"<collision name='collision'>.*?</collision>",new,s,flags=re.S)
    open(f,'w').write(s2); print(f,s!=s2)
EOF
grep -c '<box>' src/drdo_gz_worlds/models/world*/model.sdf
grep '<collision' src/drdo_gz_worlds/models/world3/model.sdf
git add -A
git commit -m "B1: terrain collision uses the visual mesh"
source ~/uav_guided_ugv/setup/env.sh
colcon build --packages-select drdo_gz_worlds
```
Expect: 5 True; box counts 0; build finished.
Stop if: python prints Traceback.
Return: fixes_commits

### T22 rtf-check  |  tier: flash  |  needs: none
Goal: Read real-time factor of the running world.
Inputs: {{WORLD}}
Run:
```bash
source ~/uav_guided_ugv/setup/env.sh
timeout 6 gz topic -e -t /world/{{WORLD}}/stats | grep real_time_factor | tail -2
```
Expect: two `real_time_factor: <number>` lines.
Stop if: nothing printed.
Return: rtf

### T23 decimate-world3-collision  |  tier: pro  |  needs: sudo
Goal: 10% decimated world3 mesh for collision only.
Inputs: none
Run:
```bash
sudo apt-get install -y blender
cd ~/training_pool/src/drdo_gz_worlds/models
blender -b --python-expr "import bpy;bpy.ops.wm.read_factory_settings(use_empty=True);bpy.ops.wm.collada_import(filepath='world3/meshes/terrain_raster3.dae');[o.select_set(True) for o in bpy.data.objects];bpy.context.view_layer.objects.active=[o for o in bpy.data.objects if o.type=='MESH'][0];bpy.ops.object.join();m=bpy.context.object.modifiers.new('d','DECIMATE');m.ratio=0.1;bpy.ops.object.modifier_apply(modifier='d');bpy.ops.wm.collada_export(filepath='world3/meshes/terrain_raster3_collision.dae')"
ls -la world3/meshes
python3 -c "p='world3/model.sdf';s=open(p).read();s=s.replace('terrain_raster3.dae','terrain_raster3_collision.dae',1);open(p,'w').write(s)"
grep -n 'terrain_raster3' world3/model.sdf
cd ~/training_pool
git add -A
git commit -m "B1: decimated collision mesh for world3"
source ~/uav_guided_ugv/setup/env.sh
colcon build --packages-select drdo_gz_worlds
```
Expect: new .dae well under 40 MB; first grep hit ends `_collision.dae`.
Stop if: blender prints `Error` or the new file is missing.
Return: collision_mesh_mb, fixes_commits

### T24 fix-B4a  |  tier: flash  |  needs: none
Goal: Delete the 1e-6 include scale from all world SDFs; commit.
Inputs: none
Run:
```bash
cd ~/training_pool
grep -n '0.000001' src/drdo_gz_worlds/worlds/*.sdf
sed -i '/<scale>0.000001 0.000001 0.000001<\/scale>/d' src/drdo_gz_worlds/worlds/*.sdf
grep -c '0.000001' src/drdo_gz_worlds/worlds/*.sdf
git add -A
git commit -m "B4a: drop 1e-6 include scale"
source ~/uav_guided_ugv/setup/env.sh
colcon build --packages-select drdo_gz_worlds
```
Expect: first grep >= 5 lines; second all 0; build finished.
Stop if: first grep empty.
Return: fixes_commits

### T25 fix-B2  |  tier: flash  |  needs: none
Goal: Create world3_mesh and drdo_world3_overlay.sdf; commit.
Inputs: none
Run:
```bash
cd ~/training_pool/src/drdo_gz_worlds
mkdir -p models/world3_mesh
cp -r models/world3/meshes models/world3_mesh/
cat > models/world3_mesh/model.config <<'EOF'
<?xml version="1.0"?>
<model>
  <name>world3_mesh</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
</model>
EOF
cat > models/world3_mesh/model.sdf <<'EOF'
<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='terrain'>
    <static>1</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <mesh>
            <scale>20 20 20</scale>
            <uri>model://world3_mesh/meshes/terrain_raster3.dae</uri>
          </mesh>
        </geometry>
      </collision>
      <visual name='visual'>
        <cast_shadows>0</cast_shadows>
        <geometry>
          <mesh>
            <scale>20 20 20</scale>
            <uri>model://world3_mesh/meshes/terrain_raster3.dae</uri>
          </mesh>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.1 0.1 0.2 1</diffuse>
          <specular>0 0 0 0</specular>
          <emissive>1 1 1 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
EOF
sed -e 's/name="drdo_world3"/name="drdo_world3_overlay"/' -e 's#model://world3<#model://world3_mesh<#' worlds/drdo_world3.sdf > worlds/drdo_world3_overlay.sdf
grep -n 'world3' worlds/drdo_world3_overlay.sdf
cd ~/training_pool
git add -A
git commit -m "B2: world3_mesh model and drdo_world3_overlay world"
source ~/uav_guided_ugv/setup/env.sh
colcon build --packages-select drdo_gz_worlds
ls install/drdo_gz_worlds/share/drdo_gz_worlds/worlds install/drdo_gz_worlds/share/drdo_gz_worlds/models
```
Expect: grep shows `drdo_world3_overlay` and `model://world3_mesh`; install lists both.
Stop if: grep shows no `world3_mesh`.
Return: fixes_commits

### T27 fix-B3  |  tier: pro  |  needs: none
Goal: Author x500_depth_down + airframe 4022; build PX4; commit.
Inputs: none
Run:
```bash
cd ~/PX4-Autopilot/Tools/simulation/gz
git checkout -b drdo_depth_down
cp -r models/x500_depth models/x500_depth_down
cd models/x500_depth_down
grep -c '.12 .03 .242 0 0 0' model.sdf
sed -i "s/<model name='x500_depth'>/<model name='x500_depth_down'>/" model.sdf
sed -i 's#<name>x500_depth</name>#<name>x500_depth_down</name>#' model.config
sed -i 's#<pose>.12 .03 .242 0 0 0</pose>#<pose>.12 .03 .242 0 1.5708 0</pose>#' model.sdf
grep -n "x500_depth_down\|1.5708" model.sdf model.config
cd ~/PX4-Autopilot/Tools/simulation/gz
git add models/x500_depth_down
git commit -m "x500_depth_down: downward OAK-D-Lite"
cd ~/PX4-Autopilot
git checkout -b drdo_depth_down
cd ROMFS/px4fmu_common/init.d-posix/airframes
ls | grep -c '^4022_'
sed 's/x500_depth/x500_depth_down/g' 4002_gz_x500_depth > 4022_gz_x500_depth_down
sed -i '/4021_gz_x500_flow/a 4022_gz_x500_depth_down' CMakeLists.txt
grep -n -A1 '4021_gz_x500_flow' CMakeLists.txt
cd ~/PX4-Autopilot
make px4_sitl > ~/uav_guided_ugv/setup/logs/px4_build_b3.log 2>&1
tail -3 ~/uav_guided_ugv/setup/logs/px4_build_b3.log
ls build/px4_sitl_default/etc/init.d-posix/airframes | grep 4022
git add ROMFS Tools/simulation/gz
git commit -m "B3: gz_x500_depth_down airframe 4022"
```
Expect: 1.5708 on 2 lines (include + joint); build without Error; 4022 installed.
Stop if: first grep prints 0, or `ls | grep -c '^4022_'` prints 1.
Return: fixes_commits (PX4 hash), uav_model (gz_x500_depth_down)

### T28 flip-camera-pitch  |  tier: flash  |  needs: none
Goal: Flip the camera pitch sign and rebuild.
Inputs: none
Run:
```bash
cd ~/PX4-Autopilot/Tools/simulation/gz/models/x500_depth_down
grep -n '1.5708' model.sdf
sed -i 's/0 1.5708 0/0 -1.5708 0/' model.sdf
grep -n '1.5708' model.sdf
git commit -am "x500_depth_down: flip camera pitch"
```
Expect: grep shows `-1.5708`.
Stop if: first grep empty.
Return: fixes_commits

### T30 fix-README  |  tier: flash  |  needs: none
Goal: README PX4 example gets the world2 pose; commit.
Inputs: none
Run:
```bash
cd ~/training_pool
grep -n 'PX4_GZ_MODEL_POSE' README.md
sed -i 's/PX4_GZ_WORLD=drdo_world2 PX4_GZ_MODEL_POSE="-10.226,311.831,22.863,0.011338,0.135709,-2.161422"/PX4_GZ_WORLD=drdo_world2 PX4_GZ_MODEL_POSE="103.776917,-101.472992,17.318562,-0.054656,0.032451,2.460081"/' README.md
grep -n 'PX4_GZ_MODEL_POSE' README.md
git commit -am "README: PX4 example uses the world2 UAV pose"
git log --oneline -6
```
Expect: second grep shows the world2 pose.
Stop if: first grep empty.
Return: fixes_commits (all hashes)

### T31 write-bringup-sh  |  tier: flash  |  needs: none
Goal: Write the one-command bring-up script.
Inputs: {{RGB_SUFFIX}} {{DEPTH_SUFFIX}} {{INFO_SUFFIX}} {{POINTS_SUFFIX}} (topic part after `/world/<world>/`)
Run:
```bash
cat > ~/uav_guided_ugv/setup/bringup.sh <<'EOF'
#!/usr/bin/env bash
W=${1:?world}; M=${2:-gz_x500_depth_down}
source ~/uav_guided_ugv/setup/env.sh
case ${W%_overlay} in
  drdo_world1) UAV="-10.226,311.831,22.863,0.011338,0.135709,-2.161422"; UGV="-12.220319 308.976703 22.295580 -2.161422";;
  drdo_world2) UAV="103.776917,-101.472992,17.318562,-0.054656,0.032451,2.460081"; UGV="104.742386 -101.9010777 15.730011 2.460081";;
  drdo_world3) UAV="108.849,-265.663,49.4752,0.045161,0.003268,1.588"; UGV="109.076 -262.736 49.2026 1.56611";;
  *) echo "unknown world $W"; exit 1;;
esac
set -- $UGV
L=~/uav_guided_ugv/setup/logs; mkdir -p $L
pkill -f 'gz sim'; pkill -f px4_sitl_default/bin/px4; pkill -f MicroXRCEAgent; pkill -f parameter_bridge; sleep 3
nohup ros2 launch drdo_gz_worlds world.launch.py world:=$W > $L/gz.log 2>&1 &
sleep 30
nohup ros2 launch ackermann_gz_bringup spawn_ackermann.launch.py world:=$W x:=$1 y:=$2 z:=$3 yaw:=$4 > $L/rover.log 2>&1 &
nohup MicroXRCEAgent udp4 -p 8888 > $L/agent.log 2>&1 &
cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_SIM_MODEL=$M PX4_GZ_WORLD=$W PX4_GZ_MODEL_POSE="$UAV" nohup ./build/px4_sitl_default/bin/px4 -d > $L/px4.log 2>&1 &
sleep 40
P=/world/$W
nohup ros2 run ros_gz_bridge parameter_bridge $P/{{RGB_SUFFIX}}@sensor_msgs/msg/Image@gz.msgs.Image $P/{{DEPTH_SUFFIX}}@sensor_msgs/msg/Image@gz.msgs.Image $P/{{INFO_SUFFIX}}@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo $P/{{POINTS_SUFFIX}}@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked --ros-args -r $P/{{RGB_SUFFIX}}:=/uav/rgb -r $P/{{DEPTH_SUFFIX}}:=/uav/depth -r $P/{{INFO_SUFFIX}}:=/uav/camera_info -r $P/{{POINTS_SUFFIX}}:=/uav/points > $L/bridge.log 2>&1 &
EOF
chmod +x ~/uav_guided_ugv/setup/bringup.sh
bash -n ~/uav_guided_ugv/setup/bringup.sh && echo SYNTAX_OK
```
Expect: SYNTAX_OK.
Stop if: bash -n reports an error.
Return: none

### T32 verify-bringup  |  tier: flash  |  needs: gui
Goal: Run bringup.sh from a clean shell; check five signals.
Inputs: none
Run:
```bash
env -u ROS_DISTRO -u AMENT_PREFIX_PATH -u GZ_SIM_RESOURCE_PATH -u COLCON_PREFIX_PATH bash --norc -c '~/uav_guided_ugv/setup/bringup.sh drdo_world2'
source ~/uav_guided_ugv/setup/env.sh
grep -c 'Ready for takeoff' ~/uav_guided_ugv/setup/logs/px4.log
grep -c -i 'session established' ~/uav_guided_ugv/setup/logs/agent.log
timeout 3 ros2 topic echo /odom --field pose.pose.position.z | head -1
timeout 8 ros2 topic hz /uav/rgb
ros2 topic list | grep -c /fmu/
```
Expect: 1, >=1, z near 15.7, a rate, >20.
Stop if: bringup.sh exits non-zero.
Return: ready, session, odom_z, rgb_hz, fmu_topics; status NEEDS_USER "Terrain, rover, drone visible?"

### T33 append-report  |  tier: flash  |  needs: none
Goal: Append text to the setup report.
Inputs: {{TEXT}}
Run:
```bash
cat >> ~/uav_guided_ugv/setup/reports/02_setup_report.md <<'EOF'
{{TEXT}}
EOF
tail -3 ~/uav_guided_ugv/setup/reports/02_setup_report.md
```
Expect: appended text echoed.
Stop if: nothing.
Return: none

## Report template
Sent as T33 {{TEXT}}; earlier fix-log lines: `- <fix> <commit> verified: <signal>`.

```markdown
# 02 Setup report
## Fix log
## setup-facts
```
```
gz_version: 
ros_gz_pkg: 
px4_tag: 
px4_msgs_branch: 
agent_version: 
training_pool_commit: 
fixes_commits: B1=<h> B4a=<h|skipped> B2=<h> B3=<px4 h> README=<h>
rtf_world1: 
rtf_world2: 
rtf_world3: 
camera_topics: <4 gz topics> -> /uav/rgb /uav/depth /uav/camera_info /uav/points
known_issues: 
```

## Facts
| world | UAV pose x,y,z,roll,pitch,yaw (PX4_GZ_MODEL_POSE) | UGV x y z yaw (spawn_ackermann) |
|---|---|---|
| drdo_world1 | -10.226,311.831,22.863,0.011338,0.135709,-2.161422 | -12.220319 308.976703 22.295580 -2.161422 |
| drdo_world2 | 103.776917,-101.472992,17.318562,-0.054656,0.032451,2.460081 | 104.742386 -101.9010777 15.730011 2.460081 |
| drdo_world3 | 108.849,-265.663,49.4752,0.045161,0.003268,1.588 | 109.076 -262.736 49.2026 1.56611 |

Versions: gz-sim 8.x; ros-humble-ros-gzharmonic; PX4 v1.16.2; px4_msgs release/1.16; agent v2.4.2; airframe 4022 free; Tools/simulation/gz is a submodule.

Docs: https://gazebosim.org/docs/harmonic/ros_installation ; https://gazebosim.org/docs/harmonic/install_ubuntu ; https://docs.px4.io/main/en/sim_gazebo_gz/ ; https://docs.px4.io/main/en/ros2/user_guide.html
