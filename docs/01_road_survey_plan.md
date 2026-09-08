# Stage 1 — road survey and costmap generation

**Goal.** Fly the UAV manually over the whole road, and end up with a saved
occupancy grid in which the road is cheap, everything else is lethal, and the
middle of the road is cheaper than its edges. Nothing else. No autonomy, no
UGV, no ArUco. Stage 2 loads that grid and runs Nav2.

Status: implemented in `src/road_survey`, 47 tests green including an
end-to-end simulated survey.

---

## 1. Why slope works, and what "slope" has to mean

Your instinct — *the road is the flat bit between two slopes* — is exactly the
method the literature uses, but it needs one correction to survive contact with
a real depth map.

A raw per-pixel gradient is not enough. Three quantities are needed, and every
serious traversability stack computes the same three:

| Quantity | What it catches | Fails alone because |
|---|---|---|
| **Slope** — angle of the locally fitted plane | the cut face and the drop | a smooth 10° hillside also looks flat-ish |
| **Step** — max−min height in a 3×3 window | the kerb, the road edge, rocks | a gentle ramp has no step |
| **Roughness** — RMS residual about that plane | rubble, scrub, depth noise | a tilted smooth face has no roughness |

This triple is the slope/step/roughness filter set in ANYbotics'
`traversability_estimation`, and it is the same formulation as the risk map in
Wang et al., *Aerial-Ground Collaborative Continuous Risk Mapping* (IEEE T-AES
2023) — the paper you sent. We use their combination rule verbatim:

```
n_i = clip(f_i / f_i_crit, 0, 1)          for i in {slope, step, roughness}
risk = 1                     if any n_i >= 1
risk = Σ w_i · n_i           otherwise      (w = 1/3 each)
road = risk < risk_thresh
```

The one thing you must get right, and the reason a naive implementation
produces mush: **roughness has to be measured about the fitted plane, not about
the local mean.** If you use the mean, a perfectly smooth 20° hillside reads as
0.09 m of roughness and the whole map turns to noise. We fit the plane in
closed form with separable box filters (`risk.py:terrain_features`), so it
costs three `uniform_filter` passes, not a per-cell least squares.
`test_risk.py::test_roughness_is_measured_about_the_plane_not_the_mean` locks
this in.

What we are *actually* doing, said plainly for the report: a hill road is a
flat shelf cut into a slope. We are not asking "can the rover climb this cell",
we are using the risk map as a **shape detector for that shelf**. That is why
the thresholds below are much tighter than the paper's off-road ones (they
allow 30° slopes; we allow 15°).

### Starting thresholds

| Parameter | Value | Reasoning |
|---|---|---|
| `slope_crit_deg` | 15° | road cross-slope is <5°, cut face is 30–50° |
| `step_crit_m` | 0.25 | bigger than depth noise, smaller than a kerb |
| `rough_crit_m` | 0.12 | ~4× the measured per-cell depth noise |
| `risk_thresh` | 0.35 | a 12° smooth ramp sits at ~0.34, a 15° one is rejected |
| `resolution` | 0.25 m | road is 6–10 m wide → 24–40 cells across |
| `window_cells` | 5 (1.25 m) | ⅓ of a road width: averages noise, does not straddle the edge |

These are starting points, not answers. Section 5 is how you tune them in
seconds instead of flights.

---

## 2. The hard constraint nobody notices until it wastes a day

The `StereoOV7251` depth sensor on `x500_depth_down` has
`<clip><far>19.1</far></clip>`.

**Above ~18 m AGL the depth image is empty.** Not degraded — empty. Survey
altitude is therefore **10–15 m above the road**, and the mapper logs a
specific warning if a frame arrives with no valid returns.

At 12 m AGL, with hfov 1.274 rad and 640×480:

- ground footprint **17.7 m across × 13.4 m along**
- ground sample distance **2.8 cm/px** — 0.25 m cells get ~80 samples per frame
- one road width (6–10 m) fits inside the footprint with margin

If you would rather fly at 25 m and cover ground faster, the only fix is to
edit `<far>` in your own `x500_depth_down/model.sdf` and rebuild. That is a
legitimate change (it is your model), but do it deliberately and record it,
because it changes what "the sensor" means in your report.

---

## 3. How to fly the survey

The mapper never commands the UAV. Fly it in Position mode from QGC or a
gamepad.

1. Take off, climb to **12 m above the road surface** (not above the valley —
   above the road you are over).
2. Fly **along the road**, keeping the road inside the footprint. The camera is
   nadir, so what you cover is roughly a 17 m strip centred under the aircraft.
3. Speed **≤ 3 m/s**. Not for coverage — at 5 Hz integration even 5 m/s gives
   93 % along-track overlap — but because pose/image time-sync error turns into
   map smear proportional to speed.
4. **Do not bank hard.** Frames taken at >25° tilt are dropped
   (`max_tilt_deg`); at high tilt the far edge of the image is beyond the far
   clip and the pose error is amplified by range.
5. On switchbacks, fly each leg separately at the right height for that leg.
   Do not cut the corner high — you will be above the far clip.
6. Watch `/road/stats`. `surveyed_m2` should climb steadily; if
   `points_outside_grid` is non-zero, raise `map_size_m`.
7. When the whole road is covered:
   `ros2 service call /terrain_mapper/save std_srvs/srv/Trigger`
   (it also autosaves on Ctrl-C).

Coverage is the thing that actually determines your score. A gap in the survey
becomes lethal cells, and Nav2 will refuse to plan through it.

---

## 4. The costmap question: 2-D, 2.5-D, or 3-D

You asked whether to keep one flat costmap, rebuild it by height as the rover
drives, or go 3-D. The answer is **2.5-D internally, 2-D on the wire, with a
height-slice escape hatch** — and here is the reasoning, because it is the part
worth defending in the presentation.

**3-D (voxels / OctoMap) is wrong for this.** Nav2 plans in 2-D. A 3-D map
would have to be flattened before planning anyway, so all the extra memory buys
you is the ability to represent overhangs — and there are no overhangs on these
roads, only stacked switchback legs, which is a much cheaper problem.

**A single flat 2-D grid is wrong too** — but only in one specific place. On a
spiral hill road, the upper and lower legs can project onto the *same* XY
cells. A flat grid says "road here", the planner draws a straight line between
two legs that are 15 m apart vertically, and the rover drives off the edge.

**So: keep the full 2.5-D state, publish 2-D.** Every cell carries mean
height, height variance, sample count and min/max height. We publish an
`OccupancyGrid` (because that is what Nav2 eats) but we keep the elevation
layer beside it in the `.npz`.

The stacking case is then handled explicitly rather than hoped away:

- The mapper flags any road cell whose samples span more than `2 m` vertically
  and reports `multi_level_cells` in `/road/stats`.
- **If that count is 0** — which it will be for world 1 and probably world 2 —
  the flat map is provably safe. Ship it, run plain Nav2, done.
- **If it is non-zero**, run `height_slicer`. It keeps the map 2-D and Nav2
  untouched, but republishes it each second with every cell whose surveyed
  elevation is more than `band_m` (default 3 m) from the UGV's *current* height
  marked lethal. The slice follows the rover up the hill and the other leg of
  the switchback simply is not there. This is the "script which updates the
  costmap according to height" you described, and it is ~60 lines because the
  height layer was kept.

The elevation layer earns its keep twice more: it gives the UGV pose a `z` for
free, and it lets you sanity-check the ArUco-derived pose in stage 2 (if the
marker says the rover is at a height the map disagrees with, the pose is bad).

### Why the costs are graded rather than binary

The rubric scores **lateral deviation from the road centre**, not "did you stay
on the road". A binary free/lethal map makes every point across a 6 m road
equally attractive and Nav2 will happily hug the inside edge of every corner —
scoring badly while being perfectly "correct".

So `to_occupancy` assigns each road cell a cost that falls from 99 at the edge
to 0 at `edge_cost_ref_m` (1.5 m) inside it, using a distance transform. The
cheapest path is then the centre of the road by construction. Nav2 must be
configured with `trinary_costmap: false` or it will throw the gradient away —
see the handoff notes.

---

## 5. Tuning loop

Do **not** re-fly to test a threshold. Fly once, save, then:

```bash
ros2 run road_survey tune_offline ~/uav_guided_ugv/maps/road_map.npz \
    --sweep slope 8 10 12 15 20
```

which re-classifies the saved elevation grid at each value in about a second
each and writes a PNG per value:

```
slope=8   area=  501.4 m2  width_p95=4.80 m  parts=1  -> sw_slope8.png
slope=12  area=  514.7 m2  width_p95=4.89 m  parts=1  -> sw_slope12.png
slope=15  area=  522.6 m2  width_p95=5.00 m  parts=1  -> sw_slope15.png
slope=20  area=  529.6 m2  width_p95=5.00 m  parts=1  -> sw_slope20.png
```

What you are looking for:

- `parts` (connected components) **= 1**. More than one means the road broke
  into fragments — loosen the thresholds or fix a coverage gap.
- `road_width_p95_m` ≈ the real road width. Much larger means you are leaking
  onto the verge; much smaller means you are over-eroding.
- The PNG. Green is road, red is over-critical risk, grey is surveyed
  non-road, near-black is unsurveyed. A correct result is an unbroken green
  ribbon with red on both flanks.

Sweep `slope` first (it dominates), then `thresh`, then `rough` only if the
depth noise in your build is worse than expected.

---

## 6. Acceptance criteria for stage 1

Stage 1 is done when, on `drdo_world1`:

1. `parts == 1` and the green ribbon in the debug PNG runs unbroken from the
   UGV spawn to the end of the road.
2. `road_width_p95_m` is within ~1 m of the road's true width.
3. `points_outside_grid == 0`.
4. `multi_level_cells == 0`, **or** `height_slicer` is in the stage-2 launch.
5. `road_map.npz`, `.pgm`, `.yaml` and `.png` exist and `map_publisher`
   republishes the grid with the same origin and resolution.

Reference numbers from the simulated survey in `test_pipeline.py` (synthetic
6 m road on a 40° cut, 12 m AGL, 1 cm depth noise) — treat these as the ceiling
a real run should approach, not as a promise:

| Metric | Result |
|---|---|
| Elevation error on the carriageway | 0.6 mm median, 2 cm p95 |
| Road mask IoU vs. ground truth | 0.87 |
| Recovered area vs. true area | 523 m² vs 605 m² (mask sits ~0.4 m inside each edge) |
| Centre-line lateral error | 0.29 m median, 0.46 m p90 |

---

## 7. Interface handed to stage 2

| | |
|---|---|
| Topic | `/road/costmap`, `nav_msgs/OccupancyGrid`, transient-local |
| Frame | `map` = ENU, origin at the **PX4 EKF local origin** (the UAV spawn) |
| Values | `0…99` road (0 = centre), `100` not road, `-1` unknown if enabled |
| File | `~/uav_guided_ugv/maps/road_map.npz` — occ, elevation, risk, road, valid, count, extent, origin, resolution |
| Replay | `ros2 run road_survey map_publisher -p map_npz:=…` |

Note the frame carefully: everything is in **PX4's local ENU**, whose origin is
where the UAV's EKF initialised, i.e. the UAV spawn pose. It is *not* the
Gazebo world origin. Stage 2's ArUco-derived UGV pose must land in this same
frame — which it will, because it is derived from the UAV's PX4 pose.

---

## 8. Known risks

| Risk | Mitigation |
|---|---|
| The depth topic's real Gazebo name differs from `/depth_camera` | check `gz topic -l` first; it is a launch argument, not a code change |
| Gazebo publishes no `camera_info` for the depth sensor | the node falls back to intrinsics from hfov; verified equal to 432.5 px |
| PX4 pose and depth stamps drift apart | frames with >80 ms pose gap are dropped and warned about; if this fires constantly, `use_sim_time` is wrong |
| Road wider than the footprint at low altitude | fly two passes; the grid accumulates across passes with no special handling |
| Depth noise worse than 1 cm in your build | raise `rough_crit_m`, or `min_count` to average more samples per cell |
| A survey gap creates a lethal band | re-fly that section; the grid is additive, just restart the node and re-survey, or keep it running |
