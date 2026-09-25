"""Road-following survey on a synthetic road the UAV reveals as it flies.

Only cells inside the camera footprint of places the UAV has been are known,
exactly like the live costmap. The explorer must follow the bends, never turn
back onto road it has already flown, and report the end of the road.
"""

import math

import numpy as np

from guidance.explore import ExploreParams, next_waypoint
from road_survey import costmap as cm

RES = 0.25
ORIGIN = (0.0, 0.0)
SHAPE = (240, 480)              # 60 m x 120 m


def centre_y(x):
    return 30.0 + 12.0 * np.sin(x / 15.0)


def road_mask():
    iy, ix = np.mgrid[0:SHAPE[0], 0:SHAPE[1]]
    x = ORIGIN[0] + (ix + 0.5) * RES
    y = ORIGIN[1] + (iy + 0.5) * RES
    return (np.abs(y - centre_y(x)) < 3.0) & (x > 5.0) & (x < 105.0), x, y


def survey(start, heading, step_m=3.0, max_steps=200):
    road, gx, gy = road_mask()
    seen = np.zeros(SHAPE, dtype=bool)
    pos = np.array(start, dtype=float)
    track = [pos.copy()]
    misses = 0
    p = ExploreParams()
    for _ in range(max_steps):
        seen |= (np.abs(gx - pos[0]) < 6.7) & (np.abs(gy - pos[1]) < 8.8)
        occ = cm.to_occupancy(road & seen, seen, RES, cm.CostmapParams())
        wp = next_waypoint(occ, *ORIGIN, RES, pos, heading, p)
        if wp is None:
            misses += 1
            if misses >= 3:
                return np.array(track), True
            continue
        misses = 0
        d = np.asarray(wp) - pos
        heading = math.atan2(d[1], d[0])
        pos = pos + d / np.linalg.norm(d) * min(step_m, np.linalg.norm(d))
        track.append(pos.copy())
    return np.array(track), False


def test_follows_the_road_to_its_end():
    track, ended = survey((8.0, float(centre_y(8.0))), heading=0.0)
    assert ended
    assert track[-1, 0] > 98.0                          # got to the far end
    off = np.abs(track[:, 1] - centre_y(track[:, 0]))
    assert off.max() < 3.0                              # never left the road
    # near the road centre until the ring starts clipping the dead end
    assert off[track[:, 0] < 100.0].max() < 0.5
    assert (np.diff(track[:, 0]) > -0.1).all()          # never doubled back


def test_starting_the_wrong_way_ends_immediately():
    track, ended = survey((8.0, float(centre_y(8.0))), heading=math.pi)
    assert ended and len(track) <= 2


def test_prefers_the_branch_straight_ahead():
    occ = np.full((200, 200), 100, dtype=np.int8)       # 50 m x 50 m
    occ[90:110, :] = 10                                  # road along x at y=25
    occ[100:200, 140:160] = 10                           # branch going +y at x=37.5
    wp = next_waypoint(occ, 0.0, 0.0, RES, (32.0, 25.0), 0.0, ExploreParams())
    assert wp is not None and abs(wp[1] - 25.0) < 1.0 and wp[0] > 35.0
