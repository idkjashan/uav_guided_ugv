"""Fly the whole mission offline: survey, return, lock on, drive the UGV home.

A toy world stands in for Gazebo and PX4, and the real mission, explorer,
centre-line and pure pursuit code runs against it:

* the road climbs 6 % and curves; only cells the depth camera has looked at
  appear in the costmap, refreshed every 2 s like terrain_mapper
* the UGV sits on the road during the survey and shows up as a hole
* PX4 is a P position loop plus the setpoint's velocity feed-forward
* the marker is seen only while it is inside the RGB footprint

This is the test to run after touching any of the guidance logic.
"""

import math

import numpy as np
import pytest

from guidance.mission import ACQUIRE, DONE, RETURN, SURVEY, TRACK, Mission, MissionParams
from guidance.pursuit import PursuitParams, centerline_path, command
from road_survey import costmap as cm

RES = 0.25
OX, OY = -10.0, 0.0
H, W = 240, 520                   # 60 m x 130 m
ROAD_END = 110.0
DT = 0.05


def centre_y(x):
    return 30.0 + 10.0 * np.sin(x / 18.0)


def ground(x, y):
    return 0.06 * np.asarray(x)


iy, ix = np.mgrid[0:H, 0:W]
GX = OX + (ix + 0.5) * RES
GY = OY + (iy + 0.5) * RES
ROAD = (np.abs(GY - centre_y(GX)) < 3.0) & (GX > 0.0) & (GX < ROAD_END)


def ned(e):   # ENU (x, y, z) -> NED
    return np.array([e[1], e[0], -e[2]])


def enu(n):
    return np.array([n[1], n[0], -n[2]])


def fly():
    rng = np.random.default_rng(1)
    home_e = np.array([6.0, float(centre_y(6.0)), float(ground(6.0, 0.0))])
    ugv = [3.0, float(centre_y(3.0)) - 1.0, 0.0]            # x, y, yaw (ENU)
    ugv_blob = (np.abs(GX - ugv[0]) < 1.1) & (np.abs(GY - ugv[1]) < 0.9)

    uav = ned(home_e)
    yaw_ned = math.pi / 2                                     # facing east, along the road
    m = Mission(MissionParams(), uav.copy(), yaw_ned, 0.0)
    seen = np.zeros((H, W), dtype=bool)
    occ = None
    path = None
    s = None
    log = {'state': [], 'agl': [], 'uav': [], 'ugv': [], 'visible': [], 'cte': []}
    done = False
    t_done = None

    for k in range(int(1500 / DT)):
        t = k * DT
        e = enu(uav)
        agl = e[2] - float(ground(e[0], e[1]))

        # depth camera: nadir footprint, 73 deg x 58 deg, far clip 19.1 m
        costmap = None
        if 0.4 < agl < 18.0 and k % int(2.0 / DT) == 0:
            seen |= (np.abs(GX - e[0]) < 0.55 * agl) & (np.abs(GY - e[1]) < 0.74 * agl)
            road = ROAD & seen
            if m.state in ('TAKEOFF', SURVEY):     # the UGV is parked on the road
                road &= ~ugv_blob
            occ = cm.to_occupancy(road, seen, RES, cm.CostmapParams())
            costmap = (occ, OX, OY, RES)
        ground_z = -float(ground(e[0], e[1])) if agl < 18.0 else None

        # RGB camera: 90 deg x 59 deg, the marker needs to be inside it
        gz = float(ground(ugv[0], ugv[1]))
        dx, dy = ugv[0] - e[0], ugv[1] - e[1]
        vis = (2.0 < agl < 18.0 and abs(dx) < 0.56 * (e[2] - gz)
               and abs(dy) < 1.0 * (e[2] - gz))
        seen_ugv = ((ugv[0] + rng.normal(0, 0.03), ugv[1] + rng.normal(0, 0.03), gz)
                    if vis else None)

        sp = m.step(t, DT, uav, ground_z, costmap, seen_ugv, goal_reached=done)
        v = 0.95 * (sp.pos - uav) + sp.vel
        n = np.linalg.norm(v[:2])
        if n > 8.0:
            v[:2] *= 8.0 / n
        uav = uav + v * DT

        if m.state == TRACK:
            if path is None:
                path = centerline_path(occ, OX, OY, RES, ugv[:2])
            if seen_ugv is not None:
                cmd = command(seen_ugv[0], seen_ugv[1], ugv[2], path, s, PursuitParams())
                s = cmd.s
                if cmd.done:
                    done, t_done = True, t
                    continue
                ugv[0] += cmd.v * math.cos(ugv[2]) * DT
                ugv[1] += cmd.v * math.sin(ugv[2]) * DT
                ugv[2] += cmd.w * DT
                log['cte'].append(abs(ugv[1] - float(centre_y(ugv[0]))))

        log['state'].append(m.state)
        log['agl'].append(agl)
        log['uav'].append(e)
        log['ugv'].append(list(ugv))
        log['visible'].append(vis)
        if t_done is not None and t - t_done > 10.0:     # watch the UAV hold for 10 s
            break
    log = {k: np.array(v) for k, v in log.items()}
    return m, log, done, ugv


@pytest.fixture(scope='module')
def flight():
    pytest.importorskip('skimage')
    return fly()


def test_survey_runs_to_the_end_of_the_road(flight):
    m, log, _, _ = flight
    assert m.end_reason == 'road ended'
    surveyed = log['uav'][log['state'] == SURVEY]
    assert surveyed[:, 0].max() > ROAD_END - 8.0


def test_survey_holds_height_above_the_climbing_road(flight):
    _, log, _, _ = flight
    agl = log['agl'][log['state'] == SURVEY]
    assert abs(np.median(agl) - 12.0) < 0.3
    assert np.abs(agl - 12.0).max() < 1.5


def test_returns_and_locks_on(flight):
    _, log, _, _ = flight
    states = list(dict.fromkeys(log['state']))
    assert states == ['TAKEOFF', SURVEY, RETURN, ACQUIRE, TRACK, DONE]


def test_holds_over_the_ugv_once_it_has_arrived(flight):
    _, log, done, ugv = flight
    assert done
    held = log['uav'][log['state'] == DONE]
    assert len(held) > 100
    off = np.hypot(held[-50:, 0] - ugv[0], held[-50:, 1] - ugv[1])
    assert off.max() < 0.3                          # settled over the stopped UGV
    assert np.ptp(held[-50:, 2]) < 0.05             # and holding height


def test_ugv_never_leaves_the_frame_while_tracked(flight):
    _, log, _, _ = flight
    assert log['visible'][log['state'] == TRACK].all()


def test_ugv_drives_the_centre_line_to_the_end(flight):
    _, log, done, ugv = flight
    assert done
    assert ugv[0] > ROAD_END - 3.0
    cte = log['cte']
    x = log['ugv'][log['state'] == TRACK][:len(cte), 0]
    assert cte[x < 10.0].max() < 1.1              # it starts 1 m off the line
    joined = x >= 10.0
    assert np.median(cte[joined]) < 0.1
    assert np.percentile(cte[joined], 95) < 0.3
    assert cte[joined].max() < 0.4


def test_early_survey_end():
    p = MissionParams(survey=True)
    m = Mission(p, [0.0, 0.0, 0.0], 0.0, 0.0)
    # Step through takeoff to reach survey
    for i in range(250):
        t = i * DT
        m.step(t, DT, [0.0, 0.0, -12.0])
    assert m.state == SURVEY
    # Trigger early end
    assert m.request_early_end(15.0)
    assert m.state == RETURN
    assert m.save_requested
    assert m.end_reason == 'user ended survey early'

