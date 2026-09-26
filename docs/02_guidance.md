# Stage 2: survey, return, guide the UGV

**Goal.** Take off, survey the road on our own, come back to the UGV, find
the ArUco marker on its roof and hold station over it while the UGV drives
the road centre line to the end of the road. The UGV has no sensors and we
use no wheel odometry: its only pose is the one the UAV measures.

Status: implemented in `src/guidance` and run end to end in Gazebo on
`drdo_world2` (2026-09-26): survey, return, lock-on and the UGV following the
centre line. 20 offline tests, including a whole mission flown against a toy
world (section 6). `HANDOFF.md` is the bring-up checklist.

---

## 1. The pieces

| Node | In | Out |
|---|---|---|
| `terrain_mapper` (stage 1) | `/uav/depth`, PX4 pose | `/road/costmap` |
| `ugv_localizer` | `/uav/rgb`, `/uav/camera_info`, PX4 pose | `/ugv/pose` |
| `mission` | PX4 status and position, `/uav/depth`, `/road/costmap`, `/ugv/pose` | PX4 offboard setpoints, `/mission/state` |
| `ugv_follower` | `/ugv/pose`, `/road/costmap`, `/mission/state` | `/cmd_vel`, `/ugv/path`, `/ugv/cross_track_error` |
| `pose_check` | `/ugv/pose`, `/ugv/ground_truth` | log only (validation) |

Everything is in one frame, `map`: PX4's local ENU, origin at the UAV spawn.
The mapper already works there, and the UGV pose lands there because it is
computed from the UAV's PX4 pose. `map` is the Gazebo world shifted by the
UAV spawn position with the axes unchanged: PX4's local frame is
north-aligned whatever the spawn yaw, and Gazebo's +Y is north.

All the logic sits in pure modules with no ROS imports (`aruco.py`,
`explore.py`, `mission.py`, `pursuit.py`), the same rule `road_survey`
follows. The nodes only move messages in and out, which is why the whole
mission can be tested without a simulator.

## 2. Mission states

```
TAKEOFF -> SURVEY -> RETURN -> ACQUIRE -> TRACK -> DONE
    \_______ survey:=false _______/
```

- **TAKEOFF.** Climb straight up at 1 m/s to 12 m above the spawn point.
- **SURVEY.** Follow the road the mapper has already classified (section 3).
  Height is held 12 m above the ground under the UAV: the median of the
  depth image centre gives the range to the ground, so the UAV climbs with
  the road instead of flying into it or rising past the depth camera's
  19.1 m far clip. When the road ends, ask the mapper to save the map.
- **RETURN.** Retrace the breadcrumbs dropped every 2 m during the survey.
  Every breadcrumb was at survey height over the terrain below it, so the
  way back cannot cut through a hillside the way a straight line home could.
- **ACQUIRE.** Hover 10 m above the spawn point. The UGV spawns 1 to 3.5 m
  from the UAV in all three worlds and the RGB footprint at 10 m is about
  20 x 11 m, so the marker is normally in view straight away. If it is not
  seen for 3 s, climb towards 16 m to widen the view.
- **TRACK.** Hold 10 m above the UGV. If the marker is lost the setpoint stops
  where it is, and after 3 s the UAV climbs towards 16 m.
- **DONE.** The follower reached the end of the road and published
  `/ugv/goal_reached`; the UAV holds over the stopped UGV.

The setpoint never jumps. It is a carrot that moves towards the current goal
at a bounded speed (1.5 m/s survey, 3 m/s return and track, 1 m/s vertical),
and its velocity goes to PX4 as feed-forward. Jumping a position setpoint far
away makes PX4 fly at up to `MPC_XY_VEL_MAX` with up to 45 degrees of tilt.

We do not write a PID for the UAV. PX4's position controller is already a
tuned cascaded PID; giving it the UGV's position as the setpoint is what keeps
the marker centred.

## 3. How the survey finds its way

`explore.next_waypoint` looks at the road cells in a ring 5 m around the
UAV, keeps the ones within 80 degrees of the direction of travel, splits
them into angular clusters (one per piece of road crossing the ring) and
returns the centre of the cluster most nearly straight ahead, weighted
towards the cheap cells in the middle of the road. That point becomes the
next goal every time a new costmap arrives, every 2 s.

The ring is 5 m because the depth footprint at 12 m is 17.7 m across and
13.4 m along the body axis, so 5 m ahead is always inside what has just
been seen. Road behind the UAV sits outside the 80 degree cone, so the
survey never doubles back.

"Road ended" means three costmaps in a row with no road ahead while hovering
at the last waypoint. The first 5 m after takeoff are flown blind along the
spawn heading. That is needed because the mapper only integrates a frame
after the UAV has moved sideways, so straight after a vertical climb the map
is one small patch and would read as "no road ahead". The survey direction is
the UAV spawn heading unless `heading_enu` is set.

## 4. Finding the UGV

```
marker corners (px) --solvePnP--> marker pose in the camera optical frame
  --camera extrinsics--> UAV body --PX4 pose at the image time--> map
```

It is the same chain `terrain_mapper` uses for depth points, and it reuses
`road_survey.frames`.

**No depth camera.** The marker's known size fixes the range, and PnP also
gives heading, which depth cannot. When the marker is near the image centre a
range error moves the estimate up or down, not sideways.

**The marker had to grow.** At 10 m the IMX214 (1920 px, 90 degrees) sees
96 px per metre. The old 0.15 m plate carries a 0.12 m black square: 12 px,
2 px per marker cell, which does not decode reliably (a test shows it failing
from 12 m). The new plate is 0.55 m, the widest that fits the 0.6 m roof. The
PNG has a 60 px white border round a 512 px marker, so the black square is
0.446 m, about 43 px or 7 px per cell. **`marker_length_m` is 0.446, not
0.55.** Passing the plate size scales every range by 1.23.

The plate is a `<plane>` visual, not a box. Gazebo maps a plane's texture
straight onto the link axes (image right is +X, the UGV's forward), so
`yaw_offset_rad` should be 0. That rests on reading Gazebo's source, not a
run: `pose_check` prints the correction on the first flight.

**Accuracy** (synthetic renders through the real IMX214 pinhole, with UAV
tilt up to 9 degrees and the UGV on up to an 8 degree slope):

| UAV height | position error | heading error |
|---|---|---|
| 10 m (tracking) | about 1 cm | under 0.5 deg |
| 12 m (survey) | about 1 cm | about 1 deg |
| 16 m (search ceiling) | about 1 cm | up to 2.4 deg |

In Gazebo the dominant error will be the UAV's own attitude estimate and
image-to-pose time matching, not pixels: 1 degree of tilt error is about
17 cm on the ground at 10 m. PX4 poses are buffered by arrival time on their
own thread, so a frame being decoded does not delay them.

## 5. Driving the UGV

**Pure pursuit along the centre line, not Nav2.**

- The rubric scores lateral deviation from the road centre. Following the
  centre line optimises that directly. Nav2's planner only gets close to it
  through the cost gradient, and cuts corners.
- Nav2's recovery behaviours (spin, back up, clear costmap) exist for robots
  that sense new obstacles. Ours has no sensors and the map does not change.
  The failure that matters here is losing the marker, and Nav2 does not know
  about it.
- It is one tuning knob (lookahead, 1.5 m) instead of a Nav2 bring-up. The
  controller is Nav2 RPP's geometry: rotate in place when the heading error
  is above 46 degrees, slow down on curves tighter than 2 m radius and inside
  the last 2 m.

**The UGV only moves while it is seen.** A `/ugv/pose` older than 0.5 s, or
any mission state other than TRACK, publishes a zero `Twist`. The gz
`DiffDrive` plugin has no command timeout, so silence would not stop it.

**Centre line.** Built once, on entering TRACK, from the latest
`/road/costmap`:

1. Fill holes in the road mask. The UGV is parked on the road during the
   survey and shows up as a 0.45 m step, so without this the centre line
   loops round it.
2. `road_survey.centerline.extract`: medial axis, longest path, spline.
   Its spline smoothing was `s = N`, which lets the curve sit up to about 1 m
   from the skeleton and cut every bend. That was a stage 1 bug nobody had
   measured. It is now `s = 0.01 N` (about 10 cm, the skeleton's own noise at
   0.25 m cells), and the error went from 0.49 m to 0.06 m median.
3. Cut the forks. At a squared-off end (the end of the survey, a dead end)
   the medial axis forks into the two corners. Where the road half-width
   drops below 90 % of its median the path is cut, then continued straight
   for one half-width so the goal still reaches the end of the road.
4. Orient it: the end nearest the UGV is the start, the far end is the goal.

## 6. Offline results

`pytest src/road_survey/test src/guidance/test`: 67 tests, a few seconds.

`test_mission_sim.py` flies the real mission, explorer, centre line and pure
pursuit code against a toy world. The road is 110 m long, climbs 6 % and
curves ±10 m. The costmap only contains cells the depth footprint has
covered, refreshed every 2 s. PX4 is modelled as a P position loop plus
feed-forward. The marker is visible only inside the RGB footprint and carries
3 cm of noise.

| Measure | Result |
|---|---|
| Phase times | takeoff 12 s, survey 77 s, return 38 s, acquire 0.1 s, track 145 s |
| Survey end | "road ended" detected, map save requested |
| UGV goal | stopped 1.1 m before the end of the road |
| Height over the climbing road | median 11.89 m, range 11.55 to 12.01 m (target 12) |
| UGV in the camera frame during TRACK | 100 % of ticks |
| UAV to UGV horizontal offset while tracking | median 0.87 m (the ~v/Kp lag at 0.8 m/s) |
| UGV cross-track error once on the line | median 6.8 cm, p95 17 cm, max 29 cm |
| UGV start | 1 m off the line on purpose; joins it within 10 m |

The toy world has no wind, no pose-to-image timing error and a perfect PX4.
Treat these numbers as a ceiling to approach in Gazebo, not a prediction.

## 7. Known limits

| Limit | What happens | Upgrade path |
|---|---|---|
| Hairpin tighter than the 5 m survey ring | reads as "road ended" | smaller `lookahead_m`, wider `max_turn_deg` |
| Stacked switchback (`multi_level_cells > 0`) | the flat centre line merges the two legs | slice the mask by height as `height_slicer` does |
| No UGV-velocity feed-forward | UAV trails the UGV by about 0.9 m | alpha-beta velocity estimate from `/ugv/pose` |
| Survey direction | taken from the spawn heading | set `heading_enu` if a world spawns facing the wrong way |
| RGB at 1920x1080, 30 Hz through the bridge | about 190 MB/s of image data | lower the camera resolution if CPU-bound (the marker needs about 30 px) |
| Marker lost for good | UAV climbs to 16 m and waits; UGV waits | a wider search pattern |
