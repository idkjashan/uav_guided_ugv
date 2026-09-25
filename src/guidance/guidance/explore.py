"""Where to fly next during the survey, read off the live road costmap.

The UAV surveys by following the road it has already classified. Take the
road cells in a ring ``lookahead_m`` around the UAV, keep the ones in front
of the direction of travel, split them into angular clusters (one per piece
of road crossing the ring) and fly to the centre of the cluster that is most
nearly straight ahead. When no road crosses the ring in front, either the
road has ended or it turned harder than ``max_turn_deg`` inside the ring.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class ExploreParams:
    lookahead_m: float = 5.0      # inside the ~6.7 m half-footprint at 12 m AGL
    ring_width_m: float = 1.5
    max_turn_deg: float = 80.0    # road further off the heading than this is ignored
    cluster_gap_deg: float = 12.0


def next_waypoint(occ, origin_x, origin_y, res, pos_xy, heading, p: ExploreParams):
    """-> (x, y) of the next survey waypoint in the map frame, or None.

    ``occ`` is the (H, W) int8 costmap, row 0 at ``origin_y``; road is 0..99.
    ``heading`` is the current direction of travel (ENU, rad).
    """
    # ponytail: a hairpin tighter than the ring reads as "road ended"; widen
    # max_turn_deg or shrink lookahead_m if a world has one.
    reach = p.lookahead_m + p.ring_width_m
    c0 = max(int((pos_xy[0] - reach - origin_x) / res), 0)
    c1 = min(int((pos_xy[0] + reach - origin_x) / res) + 1, occ.shape[1])
    r0 = max(int((pos_xy[1] - reach - origin_y) / res), 0)
    r1 = min(int((pos_xy[1] + reach - origin_y) / res) + 1, occ.shape[0])
    if c0 >= c1 or r0 >= r1:
        return None
    win = occ[r0:r1, c0:c1]
    iy, ix = np.nonzero((win >= 0) & (win < 100))
    if iy.size == 0:
        return None
    cost = win[iy, ix].astype(float)
    x = origin_x + (ix + c0 + 0.5) * res
    y = origin_y + (iy + r0 + 0.5) * res
    dx, dy = x - pos_xy[0], y - pos_xy[1]
    rng = np.hypot(dx, dy)
    ang = np.arctan2(dy, dx) - heading
    ang = np.arctan2(np.sin(ang), np.cos(ang))
    keep = ((np.abs(rng - p.lookahead_m) <= 0.5 * p.ring_width_m)
            & (np.abs(ang) <= math.radians(p.max_turn_deg)))
    if not keep.any():
        return None
    x, y, ang, cost = x[keep], y[keep], ang[keep], cost[keep]

    order = np.argsort(ang)
    cuts = np.nonzero(np.diff(ang[order]) > math.radians(p.cluster_gap_deg))[0] + 1
    best = min(np.split(order, cuts), key=lambda idx: abs(float(np.mean(ang[idx]))))
    w = 100.0 - cost[best]       # the costmap is cheapest in the road centre
    return (float(np.average(x[best], weights=w)),
            float(np.average(y[best], weights=w)))
