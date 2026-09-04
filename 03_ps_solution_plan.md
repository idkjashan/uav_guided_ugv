# Plan 3: Solving the problem statement in stages
Purpose: simulator -> submission: UAV maps road, UGV drives centreline to end on 3 overlay worlds; one command, docs, talk.
Inputs: 02_setup_report.md setup-facts; fixes merged; `~/uav_guided_ugv/setup/bringup.sh <world>` starts world, rover, PX4, agent, /uav/* bridges; ~/uav_guided_ugv/setup/env.sh sources ~/uav_guided_ugv/ws too.
Outputs: ~/uav_guided_ugv/ws/src/drdo_uav_ugv; ~/uav_guided_ugv/setup/reports/03_solution_report.md.
Orchestrator turn budget: none on Gemini CLI (1,500 requests/day); 12 if on a 50/day orchestrator. Dispatch one card at a time.

## Orchestrator
Rules
1. Entry: 02_setup_report.md has bringup_ok: yes, uav_topics: yes, world3_overlay: yes; else ask user.
2. Accuracy over speed (deviation > 5 m = zero world): low v_max; UGV stops when marker lost or path stale.
3. Classical CV only (depth normals, skeleton, spline, ArUco); compute is scored.
4. World 3 first (half the marks): every gate from S2b runs on drdo_world3_overlay before worlds 1, 2.
5. /odom = ground truth: evaluator and validation tools only (T12 greps).
6. Value outside its gate -> rerun the card with the raw block; second failure or topic/frame change -> ASK USER.
7. pro writes nodes from the inline spec; flash scaffolds, tests, docs. Fill every {{TOKEN}}; batch independent cards.

State keys: report-template keys plus cv2_version, marker_size_m, marker_px_15m, mask_ok_w3/w1/w2, pp_max_dev_m, fsm_end_ok.

Architecture (params in config/params.yaml)
node | subscribes | publishes | rate
---|---|---|---
uav_offboard | /fmu/out/vehicle_local_position, /uav/goal | /fmu/in/offboard_control_mode, /fmu/in/trajectory_setpoint, /fmu/in/vehicle_command, /uav/local_pose, TF map->uav/base_link | 10 Hz
uav_mission_fsm | /ugv/pose, /ugv/path, /road/mask, /uav/local_pose | /uav/goal, /mission/state String, /ugv/enable Bool | 5 Hz
aruco_ugv_localizer | /uav/rgb, /uav/camera_info, TF | /ugv/pose PoseStamped, TF map->ugv/base_link | per frame
road_segmentation | /uav/depth, /uav/camera_info | /road/mask mono8 | 5 Hz
centerline_mapper | /road/mask, /uav/depth, /uav/camera_info, TF | /map/centerline Path | 2 Hz
path_manager | /map/centerline, /ugv/pose | /ugv/path Path (UGV onward) | 5 Hz
pure_pursuit | /ugv/path, /ugv/pose, /ugv/enable | /cmd_vel_raw Twist | 20 Hz
speed_governor | /cmd_vel_raw, /ugv/path, /ugv/pose | /cmd_vel | 20 Hz
evaluator | /odom, /cmd_vel, /mission/state | /eval/deviation, /eval/summary, CSV | 10 Hz

TF: map (PX4 local origin, ENU) -> uav/base_link -> uav/camera_link (static, from x500_depth_down SDF); map -> ugv/base_link.
NED->ENU: (x,y,z)_enu = (y,x,-z)_ned; yaw_enu = pi/2 - yaw_ned; FRD->FLU (x,-y,-z).
{{ALT}} = 15 m above local ground (UAV z minus centre depth). {{UAV_SPAWN_XYZ}} = UAV spawn xyz = map origin in world frame.

Decision tree (pass -> next; fail -> rule 6)
```
N0 entry facts present -> batch T1, T2 x3 worlds, T4
N1 T1 build_ok, T4 hover_ok, tests_passed -> T3
N2 T3 eval_ok -> batch T5, T7 (world3_overlay)
N3 T5 aruco_dict found -> T6 ; none -> ASK USER
N4 T6 ugv_pose_err_m <= 0.5 ; alt_m = 15 if marker_px_15m >= 20 else floor(15*px/20) ; < 10 -> ASK USER
N5 T7 mask_ok -> T8 (world1_overlay), T8 (world2_overlay)
N6 all mask_ok, S2a -> T9 ; centerline_rms_m <= 1.0, rviz_ok -> T10
N7 T10 pp_max_dev_m <= 1.5 -> T11 ; fsm_end_ok -> batch T12, T13 (world3_overlay), T14
N8 odom_hits 0, consts 0, occlusion_ok, T13 dev_max_m <= 3 -> T13 (world1_overlay), T13 (world2_overlay)
N9 all T13 finished yes, dev_max_m <= 3 -> T15 -> T16 ; rehearsal_ok -> write report
```
Mission tree (uav_mission_fsm): TAKEOFF -> ACQUIRE (marker seen) -> MAP_AHEAD (until path > 15 m past UGV) -> RELEASE (/ugv/enable true) -> LEAD (10 m ahead; marker age > 1 s -> LOST: enable false, back to last UGV pose; seen -> LEAD; 30 s -> HOLD) -> END (no road ahead 5 s, UGV within 2 m of path end).

Stage gates
S | Done when
---|---
S0 | package builds; evaluator logs deviation, elapsed, cpu, ram at 10 Hz
S1 | hover 15 +-0.5 m for 30 s; frame tests pass
S2a | ugv_pose_err_m <= 0.5; marker >= 20 px at alt_m
S2b | mask_ok on all three overlays (road_frac 0.05-0.4, band on road)
S3 | centerline_rms_m <= 1.0; rviz_ok
S4 | pp_max_dev_m <= 1.5 over 60 s
S5 | world3_overlay: END reached, dev_max_m <= 3
S6 | 3 worlds finished yes, dev_max_m <= 3, cpu/ram recorded, occlusion_ok, odom_hits 0
S7 | rehearsal_ok; docs + talk exist

Ask the user when: entry facts missing; no dictionary; second gate failure; alt_m < 10; any world dev_max_m > 3; rehearsal fails.
Done when: all three worlds finished yes, dev_max_m <= 3, report written.

## Task cards

### T1 scaffold package  |  tier: flash  |  needs: sudo
Goal: create ament_python package drdo_uav_ugv, stubs, scripts/up.sh.
Inputs: none
Run:
```bash
source ~/uav_guided_ugv/setup/env.sh
sudo apt install -y python3-opencv python3-skimage python3-scipy python3-psutil python3-transforms3d ros-humble-teleop-twist-keyboard
python3 -c "import cv2;print(cv2.__version__);cv2.aruco.ArucoDetector"
mkdir -p ~/uav_guided_ugv/ws/src ~/uav_guided_ugv/ws/ref ~/uav_guided_ugv/ws/logs
cd ~/uav_guided_ugv/ws/src
ros2 pkg create drdo_uav_ugv --build-type ament_python --dependencies rclpy px4_msgs sensor_msgs geometry_msgs nav_msgs std_msgs tf2_ros cv_bridge
mkdir -p drdo_uav_ugv/launch drdo_uav_ugv/config drdo_uav_ugv/rviz drdo_uav_ugv/test drdo_uav_ugv/scripts drdo_uav_ugv/docs
```
setup.py: console_scripts uav_offboard, uav_mission_fsm, aruco_ugv_localizer, road_segmentation, centerline_mapper, path_manager, pure_pursuit, speed_governor, evaluator, validate_pose, marker_px (`name = drdo_uav_ugv.<name>:main`, stub `def main(): pass`); data_files launch, config, rviz. scripts/up.sh (chmod +x): `cd ~/uav_guided_ugv/ws; colcon build --packages-select drdo_uav_ugv; source install/setup.bash; WORLD=$1; shift; ~/uav_guided_ugv/setup/bringup.sh $WORLD > logs/bringup.log 2>&1 & sleep 40; for n in "$@"; do ros2 run drdo_uav_ugv $n > logs/$n.log 2>&1 & sleep 5; done`.
Expect: cv2 >= 4.7, no AttributeError; up.sh prints "1 package finished".
Stop if: apt E: line; AttributeError; build error.
Return: cv2_version, build_ok

### T2 reference centreline  |  tier: flash  |  needs: gui
Goal: record the ground-truth centreline of {{WORLD}} by teleop.
Inputs: {{WORLD}}
Run:
```bash
source ~/uav_guided_ugv/setup/env.sh
~/uav_guided_ugv/ws/src/drdo_uav_ugv/scripts/up.sh {{WORLD}}
ros2 topic echo /odom --csv --field pose.pose.position > ~/uav_guided_ugv/ws/ref/{{WORLD}}.csv &
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Return NEEDS_USER: user drives the rover along the road centre to the end, Ctrl-C both; then `wc -l ~/uav_guided_ugv/ws/ref/{{WORLD}}.csv`.
Expect: > 500 lines; end xy > 50 m from start.
Stop if: rover z drops > 2 m below spawn z.
Return: world_name, csv_lines, end_xy

### T3 evaluator node  |  tier: pro  |  needs: none
Goal: write drdo_uav_ugv/evaluator.py.
Inputs: {{WORLD}}
Spec: param world (default {{WORLD}}); ref ~/uav_guided_ugv/ws/ref/<world>.csv xy resampled 0.5 m; out ~/uav_guided_ugv/ws/logs/eval_<world>.csv. Sub /odom, /cmd_vel, /mission/state. 10 Hz: dev = min distance odom xy to ref polyline; elapsed since first |cmd_vel| > 0; psutil cpu_percent and rss summed over processes px4, gz, ros2, python3; publish /eval/deviation; append time,dev,cpu,ram. On state END or dev > 5: print and publish /eval/summary "dev_max= dev_mean= time= cpu_mean= ram_max= finished=yes|no" (finished = odom within 2 m of ref end).
Run:
```bash
source ~/uav_guided_ugv/setup/env.sh
~/uav_guided_ugv/ws/src/drdo_uav_ugv/scripts/up.sh {{WORLD}}
timeout 30 ros2 run drdo_uav_ugv evaluator --ros-args -p world:={{WORLD}}
tail -3 ~/uav_guided_ugv/ws/logs/eval_{{WORLD}}.csv
```
Expect: ~300 rows, dev < 1, cpu > 0.
Stop if: build error or exception.
Return: eval_ok, dev_at_spawn

### T4 offboard node + frame tests  |  tier: pro  |  needs: none
Goal: write frames.py, test/test_frames.py, uav_offboard.py.
Inputs: {{ALT}}
Spec frames: ned_to_enu, enu_to_ned, yaw_ned_to_enu, yaw_enu_to_ned, q_frd_to_flu; tests: ned (1,2,-3) -> enu (2,1,3); yaw_ned 0 -> pi/2; round trips. Offboard: QoS BEST_EFFORT, TRANSIENT_LOCAL, KEEP_LAST 1; sub VehicleLocalPosition. 10 Hz: OffboardControlMode(position=True, rest False, timestamp = clock ns // 1000) + TrajectorySetpoint(position=[n,e,d], yaw). After 10 setpoints: VehicleCommand VEHICLE_CMD_DO_SET_MODE param1=1 param2=6, then VEHICLE_CMD_COMPONENT_ARM_DISARM param1=1 (target/source ids 1, from_external True). Default goal current xy, d = -{{ALT}}; sub /uav/goal (map ENU) -> NED. Publish /uav/local_pose ENU, TF map->uav/base_link.
Run:
```bash
source ~/uav_guided_ugv/setup/env.sh
~/uav_guided_ugv/ws/src/drdo_uav_ugv/scripts/up.sh drdo_world3_overlay uav_offboard
python3 -m pytest ~/uav_guided_ugv/ws/src/drdo_uav_ugv/test -q
sleep 40
ros2 topic echo /uav/local_pose --once
grep -c -i armed ~/uav_guided_ugv/ws/logs/bringup.log
```
Expect: passed; z = {{ALT}} +-0.5; armed >= 1.
Stop if: test failed; "Failsafe" or offboard rejected in logs.
Return: tests_passed, hover_ok, hover_z

### T5 ArUco dictionary discovery  |  tier: flash  |  needs: none
Goal: find the dictionary decoding the rover marker and its size.
Inputs: none
Run:
```bash
F=$(find ~/training_pool/src -name 'aruco_marker_0.png' | head -1)
python3 - "$F" <<'EOF'
import cv2,sys
img=cv2.imread(sys.argv[1],0)
for n in ['DICT_4X4_50','DICT_5X5_50','DICT_6X6_250','DICT_ARUCO_ORIGINAL','DICT_4X4_100','DICT_6X6_50']:
    d=cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,n))
    c,ids,_=cv2.aruco.ArucoDetector(d,cv2.aruco.DetectorParameters()).detectMarkers(img)
    print(n, None if ids is None else ids.ravel().tolist())
EOF
grep -n -A12 aruco_marker_link ~/training_pool/src/*/models/ackermann_bot/model.sdf | grep -E 'size|pose'
```
Expect: one line with [0]; a size in metres.
Stop if: none.
Return: aruco_dict, marker_size_m, marker_pose

### T6 ArUco UGV localizer + validation  |  tier: pro  |  needs: none
Goal: write aruco_ugv_localizer.py, validate_pose.py, marker_px.py; measure error and pixel size.
Inputs: {{ARUCO_DICT}}, {{MARKER_SIZE_M}}, {{UAV_SPAWN_XYZ}}
Spec localizer: ArucoDetector({{ARUCO_DICT}}) on /uav/rgb; object points (+-s/2, +-s/2, 0), s = {{MARKER_SIZE_M}}; cv2.solvePnP SOLVEPNP_IPPE_SQUARE -> tvec optical (z forward, x right, y down) -> camera_link FLU -> tf2 map->uav/camera_link (static param cam_xyz_rpy). Publish /ugv/pose (map; yaw from rvec; minus param marker_offset_z), TF map->ugv/base_link; drop if reprojection error > 2 px. validate_pose: /ugv/pose + param spawn_xyz vs /odom, print err_mean, err_max over 300 samples, exit. marker_px: one /uav/rgb frame, print side_px, exit.
Run:
```bash
source ~/uav_guided_ugv/setup/env.sh
~/uav_guided_ugv/ws/src/drdo_uav_ugv/scripts/up.sh drdo_world3_overlay uav_offboard aruco_ugv_localizer
sleep 40
ros2 topic hz /ugv/pose --window 50
timeout 30 ros2 run drdo_uav_ugv marker_px
timeout 90 ros2 run drdo_uav_ugv validate_pose --ros-args -p spawn_xyz:="[{{UAV_SPAWN_XYZ}}]"
```
Expect: /ugv/pose >= 5 Hz; side_px printed; err_mean <= 0.5.
Stop if: no /ugv/pose in 20 s.
Return: pose_hz, marker_px_15m, ugv_pose_err_m, ugv_pose_err_max_m

### T7 road segmentation  |  tier: pro  |  needs: gui
Goal: write road_segmentation.py.
Inputs: {{WORLD}}
Spec: /uav/depth (32FC1 m) + /uav/camera_info. Downsample 2x; back-project with fx,fy,cx,cy; normals = normalised cross(dX/du, dX/dv); slope = acos(|n_z|); mask1 = slope < slope_max_deg (8); roughness = 7x7 local std of Z; mask2 = roughness < rough_max_m (0.15); mask = open3, close7 of mask1 & mask2; keep components > min_area_frac (0.02); publish /road/mask mono8 full resolution 5 Hz; save ~/uav_guided_ugv/ws/logs/mask_{{WORLD}}.png every 5 s. Params declared.
Run:
```bash
source ~/uav_guided_ugv/setup/env.sh
~/uav_guided_ugv/ws/src/drdo_uav_ugv/scripts/up.sh {{WORLD}} uav_offboard
sleep 40
timeout 20 ros2 run drdo_uav_ugv road_segmentation
python3 -c "import cv2;m=cv2.imread('$HOME/uav_guided_ugv/ws/logs/mask_{{WORLD}}.png',0);print('road_frac',(m>0).mean())"
```
Expect: road_frac 0.05-0.4; user confirms the band lies on the road.
Stop if: road_frac 0 or > 0.6.
Return: mask_ok, road_frac

### T8 mask check on another world  |  tier: flash  |  needs: gui
Goal: run road_segmentation unchanged on {{WORLD}}.
Inputs: {{WORLD}}
Run:
```bash
source ~/uav_guided_ugv/setup/env.sh
~/uav_guided_ugv/ws/src/drdo_uav_ugv/scripts/up.sh {{WORLD}} uav_offboard
sleep 40
timeout 20 ros2 run drdo_uav_ugv road_segmentation
python3 -c "import cv2;m=cv2.imread('$HOME/uav_guided_ugv/ws/logs/mask_{{WORLD}}.png',0);print('road_frac',(m>0).mean())"
```
Expect: road_frac 0.05-0.4; user confirms band on road.
Stop if: road_frac 0 or > 0.6.
Return: world_name, mask_ok, road_frac

### T9 centreline mapper + path manager + RViz  |  tier: pro  |  needs: gui
Goal: write centerline_mapper.py, path_manager.py, launch/perception.launch.py, config/params.yaml, rviz/drdo.rviz.
Inputs: {{WORLD}}, {{UAV_SPAWN_XYZ}}
Spec mapper: skimage.morphology.skeletonize(mask > 0); skeleton pixels + depth -> camera XYZ -> map via TF; accumulate; voxel-downsample 1 m; chain by nearest neighbour from /ugv/pose; drop branches < 5 m; scipy.interpolate.splprep smoothing -> 0.5 m samples; publish /map/centerline 2 Hz; write ~/uav_guided_ugv/ws/logs/centerline_{{WORLD}}.csv x,y,z. path_manager: /ugv/path = centreline from nearest point to UGV onward, 5 Hz. Launch: uav_offboard, aruco_ugv_localizer, road_segmentation, centerline_mapper, path_manager + params.yaml. RViz: TF, /map/centerline green, /ugv/path blue, /ugv/pose, /road/mask, /uav/rgb.
Run:
```bash
source ~/uav_guided_ugv/setup/env.sh
~/uav_guided_ugv/ws/src/drdo_uav_ugv/scripts/up.sh {{WORLD}}
nohup ros2 launch drdo_uav_ugv perception.launch.py > ~/uav_guided_ugv/ws/logs/perc.log 2>&1 &
sleep 60
python3 - <<'EOF'
import numpy as np, os
h=os.path.expanduser('~/uav_guided_ugv/ws')
c=np.loadtxt(f'{h}/logs/centerline_{{WORLD}}.csv',delimiter=',')[:,:2]+np.array([{{UAV_SPAWN_XYZ}}])[:2]
r=np.loadtxt(f'{h}/ref/{{WORLD}}.csv',delimiter=',')[:,:2]
d=[np.min(np.linalg.norm(r-p,axis=1)) for p in c]
print('n',len(c),'rms',np.sqrt(np.mean(np.square(d))),'max',np.max(d))
EOF
rviz2 -d ~/uav_guided_ugv/ws/src/drdo_uav_ugv/rviz/drdo.rviz
```
Expect: n >= 30, rms <= 1.0; user sees the green centreline on the road.
Stop if: csv missing after 60 s.
Return: centerline_rms_m, centerline_max_m, n_points, rviz_ok

### T10 pure pursuit + speed governor  |  tier: pro  |  needs: none
Goal: write pure_pursuit.py, speed_governor.py; test on world 3.
Inputs: none
Spec pp: L = clamp(k_l*v, 1.5, 4.0); target = first /ugv/path point at distance >= L; alpha = heading error from /ugv/pose yaw; curvature = 2 sin(alpha)/L; /cmd_vel_raw linear.x = v_cmd, angular.z = v_cmd*curvature, 20 Hz; zero when /ugv/enable false or path older than 2 s. Governor: v_cmd = clip(v_max*(1 - |curvature|/curv_max), v_min, v_max), v_max 1.5, v_min 0.4, curv_max 0.5; cross-track > 2 m or path end within 1 m -> 0; publish /cmd_vel. No /odom.
Run:
```bash
source ~/uav_guided_ugv/setup/env.sh
~/uav_guided_ugv/ws/src/drdo_uav_ugv/scripts/up.sh drdo_world3_overlay evaluator pure_pursuit speed_governor
nohup ros2 launch drdo_uav_ugv perception.launch.py > ~/uav_guided_ugv/ws/logs/perc.log 2>&1 &
sleep 60
ros2 topic pub --once /ugv/enable std_msgs/msg/Bool "{data: true}"
sleep 60
ros2 topic pub --once /ugv/enable std_msgs/msg/Bool "{data: false}"
python3 -c "import numpy as np;d=np.loadtxt('$HOME/uav_guided_ugv/ws/logs/eval_drdo_world3_overlay.csv',delimiter=',',skiprows=1);print('dev_max',d[:,1].max(),'dev_mean',d[:,1].mean())"
```
Expect: dev_max <= 1.5; rover moved.
Stop if: dev_max > 5.
Return: pp_max_dev_m, pp_mean_dev_m

### T11 mission FSM + one-command run  |  tier: pro  |  needs: none
Goal: write uav_mission_fsm.py, launch/mission.launch.py (arg world: perception + pure_pursuit + speed_governor + fsm + evaluator world:=world), scripts/run.sh.
Inputs: {{ALT}}
Spec FSM (/mission/state 5 Hz): TAKEOFF: /uav/goal = spawn xy, z {{ALT}} -> ACQUIRE at |z err| < 0.5. ACQUIRE -> MAP_AHEAD when /ugv/pose age < 1 s for 2 s. MAP_AHEAD: goal = UGV + 20 m along /ugv/path tangent (UGV yaw if empty) -> RELEASE when path past UGV > 15 m. RELEASE: /ugv/enable true -> LEAD. LEAD: goal = path point 10 m ahead of UGV, z = UAV z - centre depth + {{ALT}}; marker age > 1 s -> LOST. LOST: enable false, goal = last UGV pose; marker back -> LEAD; > 30 s -> HOLD. END when far third of /road/mask < 1 % road for 5 s and UGV within 2 m of path end: enable false, hover. run.sh: copy ~/uav_guided_ugv/setup/env.sh and bringup.sh into scripts/; `set -e; WORLD=${1:-drdo_world1_overlay}; source scripts/env.sh; scripts/bringup.sh $WORLD` backgrounded with log; sleep 40; `ros2 launch drdo_uav_ugv mission.launch.py world:=$WORLD`.
Run:
```bash
source ~/uav_guided_ugv/setup/env.sh
~/uav_guided_ugv/ws/src/drdo_uav_ugv/scripts/up.sh
nohup ~/uav_guided_ugv/ws/src/drdo_uav_ugv/scripts/run.sh drdo_world3_overlay > ~/uav_guided_ugv/ws/logs/mission.log 2>&1 &
timeout 900 ros2 topic echo /eval/summary --once
grep -o "state: [A-Z_]*" ~/uav_guided_ugv/ws/logs/mission.log | uniq
```
Expect: finished=yes, dev_max <= 3, states end with END.
Stop if: dev_max > 5 or no summary in 900 s.
Return: fsm_end_ok, dev_max_m, time_s, states_seen

### T12 hardening grep  |  tier: flash  |  needs: none
Goal: find ground-truth leaks and per-world constants in control nodes.
Inputs: none
Run:
```bash
cd ~/uav_guided_ugv/ws/src/drdo_uav_ugv/drdo_uav_ugv
grep -n "/odom" *.py | grep -v -E "evaluator|validate" | wc -l
grep -n -E "world[123]|311\.8|101\.47|265\.66" *.py | grep -v -E "evaluator|validate" | wc -l
grep -n -E "^[^#]*= *[0-9]+\.[0-9]+" *.py | grep -v -E "parameter|evaluator|validate" | head -20
```
Expect: first two counts 0; hardcoded lines listed.
Stop if: never.
Return: odom_hits, consts, hardcoded_lines

### T13 full run on a world  |  tier: flash  |  needs: none
Goal: end-to-end run on {{WORLD}}; record numbers.
Inputs: {{WORLD}}
Run:
```bash
source ~/uav_guided_ugv/setup/env.sh
nohup ~/uav_guided_ugv/ws/src/drdo_uav_ugv/scripts/run.sh {{WORLD}} > ~/uav_guided_ugv/ws/logs/mission_{{WORLD}}.log 2>&1 &
timeout 900 ros2 topic echo /eval/summary --once
pkill -f mission.launch
pkill -f bringup.sh
```
Expect: finished=yes, dev_max <= 3.
Stop if: dev_max > 5.
Return: world_name, dev_max_m, dev_mean_m, time_s, cpu_mean, ram_max_mb, finished

### T14 occlusion / lost-marker test  |  tier: flash  |  needs: none
Goal: occlude the marker during LEAD; UGV must stop, then recover.
Inputs: none
Run:
```bash
source ~/uav_guided_ugv/setup/env.sh
nohup ~/uav_guided_ugv/ws/src/drdo_uav_ugv/scripts/run.sh drdo_world3_overlay > ~/uav_guided_ugv/ws/logs/mission_occ.log 2>&1 &
sleep 190
X=$(ros2 topic echo /odom --once --field pose.pose.position.x)
Y=$(ros2 topic echo /odom --once --field pose.pose.position.y)
ros2 run ros_gz_sim create -world drdo_world3_overlay -name occluder -x $X -y $Y -z 60 -string '<sdf version="1.8"><model name="occluder"><static>true</static><link name="l"><visual name="v"><geometry><box><size>4 4 0.1</size></box></geometry></visual></link></model></sdf>'
sleep 5
ros2 topic echo /cmd_vel --once
gz service -s /world/drdo_world3_overlay/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 --req 'name: "occluder" type: MODEL'
sleep 10
grep -o "state: [A-Z_]*" ~/uav_guided_ugv/ws/logs/mission_occ.log | uniq | tail -4
```
Expect: /cmd_vel zero while occluded; LEAD -> LOST -> LEAD.
Stop if: rover moving 5 s after occlusion.
Return: occlusion_ok, states_tail

### T15 docs + talk  |  tier: flash  |  needs: none
Goal: write README.md, docs/ALGORITHM.md, docs/TALK.md in ~/uav_guided_ugv/ws/src/drdo_uav_ugv.
Inputs: {{PX4_TAG}}, {{GZ_VERSION}}, {{W1}}, {{W2}}, {{W3}} (dev/time/cpu strings)
README: Requirements (Ubuntu 22.04, ROS 2 Humble, gz-sim {{GZ_VERSION}}, ros-humble-ros-gzharmonic, PX4 {{PX4_TAG}}, px4_msgs, Micro-XRCE-DDS-Agent v2.4.2, python deps); Install (exact lines from ~/uav_guided_ugv/setup/reports/02_setup_report.md); Run (`scripts/run.sh <world>`); Topics; Results {{W1}} {{W2}} {{W3}}; License. ALGORITHM.md: pipeline, per-node steps, params, frames, failure handling, compute. TALK.md, 10 slides x 1 min: problem + scoring; live RViz centreline overlay, world 3; architecture; depth segmentation (texture-free); ArUco numbers; FSM; results; compute; failures; next steps.
Run:
```bash
cd ~/uav_guided_ugv/ws/src/drdo_uav_ugv
wc -w README.md docs/ALGORITHM.md docs/TALK.md
```
Expect: 400-1200, 400-1200, 300-700 words.
Stop if: never.
Return: readme_words, algo_words, talk_words

### T16 clean-workspace rehearsal  |  tier: flash  |  needs: gui
Goal: follow README literally in a fresh workspace; run world 2.
Inputs: none
Run:
```bash
rm -rf ~/uav_guided_ugv/ws_test
mkdir -p ~/uav_guided_ugv/ws_test/src
cp -r ~/uav_guided_ugv/ws/src/drdo_uav_ugv ~/uav_guided_ugv/ws_test/src/
cd ~/uav_guided_ugv/ws_test
source /opt/ros/humble/setup.bash
source ~/px4_ros_ws/install/setup.bash
source ~/training_pool/install/setup.bash
colcon build
source install/setup.bash
src/drdo_uav_ugv/scripts/run.sh drdo_world2_overlay
```
Expect: finished=yes using README steps only.
Stop if: a README step is missing or fails (name it).
Return: rehearsal_ok, missing_steps

## Risks
risk | signal | mitigation
---|---|---
marker < 20 px at 15 m | T6 | lower alt_m (N4); larger marker in rover SDF (ask user)
depth noise on plain overlay | T7 road_frac | raise rough_max_m, downsample 4x, median filter
skeleton branches on hairpins | T9 rms > 1 | prune < 5 m, more smoothing
world 3 trimesh RTF < 0.3 | bringup log | decimated collision mesh (Plan 2)
offboard rejected | Failsafe in log | 10 setpoints before mode switch; COM_RCL_EXCEPT=4
UGV off path on slope | pp dev > 1.5 | lower v_max, longer lookahead
CPU too high | cpu_mean | mask 2 Hz, mapper 1 Hz, downsample 4x

Bag rule: record only /uav/rgb /uav/depth /uav/camera_info /odom /ugv/pose /map/centerline /cmd_vel, `--compression-mode file --compression-format zstd`; never `-a`; delete after the gate.

Per-world regression: 1 bringup RTF > 0.3; 2 marker px >= 20; 3 mask_ok; 4 centreline rms <= 1; 5 T13 finished yes, dev_max <= 3.

Submission checklist (PS): one command `scripts/run.sh`; install doc; algorithm doc; open-source deps only; per-world deviation, time, CPU/RAM; UAV <= 20 m, RGBD + IMU + GPS only; UGV sensorless (odom_hits 0); all overlay worlds finished.

## Report template
```
# 03 solution report
```setup-facts
aruco_dict:
alt_m:
ugv_pose_err_m:
centerline_rms_m:
w1: dev_max= dev_mean= time= cpu= ram= finished=
w2: dev_max= dev_mean= time= cpu= ram= finished=
w3: dev_max= dev_mean= time= cpu= ram= finished=
occlusion_ok:
rehearsal_ok:
```
open issues:
```

## Facts
Spawn (x y z r p y): world1 UAV -10.226 311.831 22.863 0.011338 0.135709 -2.161422, UGV -12.220319 308.976703 22.295580 same rpy; world2 UAV 103.776917 -101.472992 17.318562 -0.054656 0.032451 2.460081, UGV 104.742386 -101.9010777 15.730011 same rpy; world3 UAV 108.849 -265.663 49.4752 0.045161 0.003268 1.588, UGV 109.076 -262.736 49.2026 0.005846 -0.047033 1.56611.
Versions: ROS 2 Humble, gz-sim 8.x, PX4 v1.16.x, px4_msgs release/1.16, Micro-XRCE-DDS-Agent v2.4.2, opencv >= 4.7.
Docs: https://docs.px4.io/main/en/ros2/offboard_control.html ; https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html ; https://gazebosim.org/docs/harmonic/ros_installation
