"""End-to-end: simulated survey flight over a synthetic hill road.

Builds a terrain that has the same shape as the DRDO worlds -- a flat shelf
cut into a hillside, curving in plan -- renders nadir depth images from it by
ray casting, and pushes them through the exact code path the node uses:

    depth image -> unproject -> PX4 pose -> map frame -> grid -> risk ->
    mask cleanup -> occupancy

then checks the recovered road against the ground truth.  If frames, signs or
thresholds are wrong this test fails; the unit tests around it say *where*.
"""

import math

import numpy as np
import pytest

from road_survey import costmap as cm
from road_survey import risk as rk
from road_survey.depth import Unprojector, intrinsics_from_hfov
from road_survey.frames import (camera_extrinsics, map_pose_to_px4,
                                optical_to_map, px4_pose_to_map, rpy_to_rot)
from road_survey.grid import TerrainGrid, known_bbox

RES = 0.25
ROAD_HALF = 3.0          # 6 m wide road
CUT_SLOPE = math.tan(math.radians(40.0))
LONG_SLOPE = 0.06        # the road itself climbs 6%
CAM_POSE = [0.12, 0.03, 0.242, 0.0, math.pi / 2, 0.0]
HFOV = 1.274
SHAPE = (240, 320)       # half-res frames keep the test quick


def centreline_y(x):
    return 25.0 + 10.0 * np.sin(x / 30.0)


def terrain(x, y):
    """Height field: a shelf of constant cross-slope cut into a hillside."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    yc = centreline_y(x)
    base = LONG_SLOPE * x
    d = y - yc
    cut = np.where(d > ROAD_HALF, (d - ROAD_HALF) * CUT_SLOPE,
                   np.where(d < -ROAD_HALF, (d + ROAD_HALF) * CUT_SLOPE, 0.0))
    return base + cut


def render_depth(cam_xyz, r_map_opt, k, shape, rng):
    """Ray cast the height field into an optical-frame depth image."""
    h, w = shape
    fx, fy = k[0, 0], k[1, 1]
    cx, cy = k[0, 2], k[1, 2]
    uu, vv = np.meshgrid(np.arange(w, dtype=float), np.arange(h, dtype=float))
    rays = np.stack([(uu - cx) / fx, (vv - cy) / fy, np.ones_like(uu)], axis=-1)
    d = rays @ r_map_opt.T                       # ray directions in map frame
    s = np.full((h, w), cam_xyz[2] - terrain(cam_xyz[0], cam_xyz[1]))
    for _ in range(6):                           # Newton, converges in 3-4
        p = cam_xyz + s[..., None] * d
        err = p[..., 2] - terrain(p[..., 0], p[..., 1])
        s = s + err / np.maximum(-d[..., 2], 1e-6)
    depth = s.astype(np.float32)
    depth += rng.normal(0.0, 0.01, size=depth.shape).astype(np.float32)
    return depth


def fly_survey(grid, altitude=12.0, step=1.5, x0=5.0, x1=95.0, seed=0):
    """Walk a nadir camera along the road and integrate every frame."""
    rng = np.random.default_rng(seed)
    k = intrinsics_from_hfov(SHAPE[1], SHAPE[0], HFOV)
    unproj = Unprojector(k, SHAPE, stride=1)
    t_cam, r_body_opt = camera_extrinsics(CAM_POSE)

    xs = np.arange(x0, x1, step)
    for x in xs:
        y = float(centreline_y(x))
        z = float(terrain(x, y)) + altitude
        # Point the nose along the road, and add the small attitude wobble a
        # hand-flown quad always has.
        dx = 1.0
        dy = float(centreline_y(x + 0.1) - centreline_y(x - 0.1)) / 0.2
        yaw = math.atan2(dy, dx)
        r_map_body = rpy_to_rot(rng.normal(0, 0.02), rng.normal(0, 0.02), yaw)

        # Round-trip through the PX4 representation so the frame conversion in
        # the node is exercised, not bypassed.
        pos_ned, quat = map_pose_to_px4([x, y, z], r_map_body)
        p_map, r_map_body2 = px4_pose_to_map(pos_ned, quat)
        assert np.allclose(p_map, [x, y, z], atol=1e-9)
        assert np.allclose(r_map_body2, r_map_body, atol=1e-9)

        r_map_opt = r_map_body2 @ r_body_opt
        cam_xyz = r_map_body2 @ t_cam + p_map
        depth = render_depth(cam_xyz, r_map_opt, k, SHAPE, rng)
        pts_opt = unproj(depth, 0.4, 18.0)
        grid.add_points(optical_to_map(pts_opt.astype(np.float64), r_body_opt,
                                       t_cam, r_map_body2, p_map))
    return len(xs)


@pytest.fixture(scope='module')
def survey():
    grid = TerrainGrid.centered_on(50.0, 25.0, 140.0, RES)
    frames = fly_survey(grid)
    return grid, frames


def test_survey_covers_the_road_corridor(survey):
    grid, frames = survey
    assert frames > 50
    assert grid.dropped == 0
    assert grid.coverage_m2(min_count=3) > 1500.0


def test_elevation_matches_the_true_height_field(survey):
    """Sub-centimetre on the road; the cut face is limited by cell size.

    A 0.25 m cell on a 40-degree face spans 0.21 m of height, so the cell mean
    legitimately differs from the height at the cell centre by that much.  The
    accuracy that matters is on the carriageway, which is where the rover
    drives and where the classifier reads slope.
    """
    grid, _ = survey
    elev = grid.elevation(min_count=3)
    known = np.isfinite(elev)
    iy, ix = np.nonzero(known)
    x, y = grid.cell_to_world(ix, iy)
    err = np.abs(elev[known] - terrain(x, y))
    on_road = np.abs(y - centreline_y(x)) < ROAD_HALF - 0.5

    assert float(np.median(err[on_road])) < 0.005
    assert float(np.percentile(err[on_road], 95)) < 0.02
    assert float(np.median(err)) < 0.02


def test_recovered_road_matches_ground_truth(survey):
    grid, _ = survey
    p = rk.RiskParams()
    known = grid.known(p.min_count)
    r0, r1, c0, c1 = known_bbox(known, margin=12)
    elev = grid.elevation(p.min_count)[r0:r1, c0:c1]

    _, road, valid, _ = rk.classify(elev, known[r0:r1, c0:c1], RES, p)
    road, info = cm.clean_mask(road, RES, cm.CostmapParams())
    occ = cm.to_occupancy(road, valid, RES, cm.CostmapParams())

    iy, ix = np.mgrid[r0:r1, c0:c1]
    x, y = grid.cell_to_world(ix, iy)
    truth = (np.abs(y - centreline_y(x)) < ROAD_HALF) & valid

    inter = np.count_nonzero(road & truth)
    union = np.count_nonzero((road | truth) & valid)
    assert inter / union > 0.75, f'IoU too low: {inter / union:.2f}'

    # No stray road on the cut face or the drop: every recovered road cell must
    # be within a metre of the true carriageway.
    stray = road & (np.abs(y - centreline_y(x)) > ROAD_HALF + 1.0)
    assert np.count_nonzero(stray) / max(np.count_nonzero(road), 1) < 0.02
    assert info['components_kept'] == 1
    assert (occ[road] >= 0).all() and (occ[~road] == 100).all()


def test_costmap_is_cheapest_near_the_road_centre(survey):
    grid, _ = survey
    p = rk.RiskParams()
    known = grid.known(p.min_count)
    r0, r1, c0, c1 = known_bbox(known, margin=12)
    elev = grid.elevation(p.min_count)[r0:r1, c0:c1]
    _, road, valid, _ = rk.classify(elev, known[r0:r1, c0:c1], RES, p)
    road, _ = cm.clean_mask(road, RES, cm.CostmapParams())
    occ = cm.to_occupancy(road, valid, RES, cm.CostmapParams())

    iy, ix = np.mgrid[r0:r1, c0:c1]
    x, y = grid.cell_to_world(ix, iy)
    off = np.abs(y - centreline_y(x))
    middle = road & (off < 1.0)
    edge = road & (off > 2.0)
    assert occ[middle].mean() < occ[edge].mean()
    assert occ[middle].mean() < 25


def test_no_multi_level_on_a_single_carriageway(survey):
    grid, _ = survey
    p = rk.RiskParams()
    extent = grid.vertical_extent(p.min_count)
    road = np.nan_to_num(extent, nan=0.0) > 0
    assert np.count_nonzero(rk.multi_level_cells(extent, road, 2.0)) == 0


def test_centreline_follows_the_road(survey):
    pytest.importorskip('skimage')
    from road_survey.centerline import extract

    grid, _ = survey
    p = rk.RiskParams()
    known = grid.known(p.min_count)
    r0, r1, c0, c1 = known_bbox(known, margin=12)
    elev = grid.elevation(p.min_count)[r0:r1, c0:c1]
    _, road, _, _ = rk.classify(elev, known[r0:r1, c0:c1], RES, p)
    road, _ = cm.clean_mask(road, RES, cm.CostmapParams())

    ox = grid.origin_x + c0 * RES
    oy = grid.origin_y + r0 * RES
    xy, half = extract(road, ox, oy, RES, spacing_m=1.0)
    assert xy.shape[0] > 40
    lateral = np.abs(xy[:, 1] - centreline_y(xy[:, 0]))
    assert float(np.median(lateral)) < 0.5
    assert float(np.percentile(lateral, 90)) < 1.0
    assert 2.0 < float(np.median(half)) < 3.5
