"""Road centre line -> path -> pure pursuit velocity commands for the UGV.

The UGV has no sensors of its own. Its pose comes from the ArUco marker the
UAV sees, so the controller is a function of (pose, path) only. The geometry
is the same as Nav2's Regulated Pure Pursuit: chase a point one lookahead
ahead on the path, rotate in place when facing the wrong way, slow down on
tight curves and near the goal. The costmap-collision parts of RPP are left
out because without a sensor there is nothing new to collide with.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage


@dataclass
class PursuitParams:
    lookahead_m: float = 1.5            # the one knob that matters
    max_speed: float = 0.8              # m/s
    min_speed: float = 0.15             # floor for the curvature/approach slow-downs
    max_yaw_rate: float = 1.0           # rad/s
    rotate_threshold_rad: float = 0.8   # rotate in place above this heading error
    rotate_yaw_rate: float = 0.6        # rad/s while rotating in place
    min_turn_radius_m: float = 2.0      # slow down on curves tighter than this
    approach_dist_m: float = 2.0        # slow down inside this distance of the goal
    goal_tolerance_m: float = 0.5
    search_window_m: float = 6.0        # how far ahead along the path to look for the robot


@dataclass
class Command:
    v: float            # m/s, forward
    w: float            # rad/s, counter-clockwise
    s: float            # progress along the path, m
    cross_track: float  # distance from the path, m
    done: bool


class Path:
    """A polyline with arc length, in the map frame."""

    def __init__(self, xy):
        self.xy = np.asarray(xy, dtype=float)
        if self.xy.ndim != 2 or self.xy.shape[0] < 2:
            raise ValueError('a path needs at least two points')
        seg = np.diff(self.xy, axis=0)
        self.seg_len = np.hypot(seg[:, 0], seg[:, 1])
        self.s = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        self.length = float(self.s[-1])

    def project(self, p, s_hint=None, window=None):
        """-> (arc length of the closest path point to ``p``, distance to it).

        With ``s_hint`` the search is limited to ``[s_hint - 1, s_hint + window]``
        so that on a switchback the robot cannot snap onto the other leg.
        """
        a = self.xy[:-1]
        ab = np.diff(self.xy, axis=0)
        l2 = np.maximum(self.seg_len ** 2, 1e-12)
        t = np.clip(((p - a) * ab).sum(axis=1) / l2, 0.0, 1.0)
        q = a + t[:, None] * ab
        d = np.hypot(q[:, 0] - p[0], q[:, 1] - p[1])
        s_seg = self.s[:-1] + t * self.seg_len
        if s_hint is not None and window is not None:
            ok = (s_seg >= s_hint - 1.0) & (s_seg <= s_hint + window)
            if ok.any():
                d = np.where(ok, d, np.inf)
        i = int(np.argmin(d))
        return float(s_seg[i]), float(d[i])

    def point_at(self, s):
        s = min(max(s, 0.0), self.length)
        return np.array([np.interp(s, self.s, self.xy[:, 0]),
                         np.interp(s, self.s, self.xy[:, 1])])


def command(x, y, yaw, path: Path, s_prev, p: PursuitParams) -> Command:
    """One control step. ``s_prev`` is the last returned ``s`` (None at start)."""
    s, cte = path.project(np.array([x, y]), s_prev, p.search_window_m)
    remaining = path.length - s
    if remaining <= p.goal_tolerance_m:
        return Command(0.0, 0.0, s, cte, True)

    lx, ly = path.point_at(s + p.lookahead_m)
    dx, dy = lx - x, ly - y
    c, sn = math.cos(yaw), math.sin(yaw)
    xr = c * dx + sn * dy            # lookahead point in the robot frame
    yr = -sn * dx + c * dy
    alpha = math.atan2(yr, xr)
    if abs(alpha) > p.rotate_threshold_rad:
        return Command(0.0, math.copysign(p.rotate_yaw_rate, alpha), s, cte, False)

    k = 2.0 * yr / max(xr * xr + yr * yr, 1e-6)   # curvature of the arc to it
    v = p.max_speed
    if abs(k) > 1e-6 and 1.0 / abs(k) < p.min_turn_radius_m:
        v *= (1.0 / abs(k)) / p.min_turn_radius_m
    if remaining < p.approach_dist_m:
        v *= remaining / p.approach_dist_m
    v = max(v, p.min_speed)
    if abs(v * k) > p.max_yaw_rate:      # keep the arc, trade speed for yaw rate
        v = p.max_yaw_rate / abs(k)
    return Command(v, v * k, s, cte, False)


def centerline_path(occ, origin_x, origin_y, res, start_xy, spacing_m=0.5):
    """Road costmap -> centre-line Path, starting at the end nearest ``start_xy``.

    ``occ`` is the OccupancyGrid data as an (H, W) int8 array, row 0 at
    ``origin_y``. Road cells are 0..99. Holes inside the road are filled first:
    the UGV itself sits on the road during the survey, shows up as a 0.45 m
    step, and would otherwise leave a hole the centre line has to loop around.
    """
    from road_survey.centerline import extract  # noqa: PLC0415

    road = (occ >= 0) & (occ < 100)
    road = ndimage.binary_fill_holes(road)
    xy, half = extract(road, origin_x, origin_y, res, spacing_m)
    if xy.shape[0] < 2:
        return None

    # At a squared-off end (the end of the survey, or a dead end) the medial
    # axis forks into the two corners, so the longest path's last few metres
    # run off to a corner. Cut the fork where the road half-width collapses
    # and carry on straight for one half-width instead.
    typical = float(np.nanmedian(half))
    ok = np.nan_to_num(half) >= 0.9 * typical
    if ok.sum() >= 3:
        i0 = int(np.argmax(ok))
        i1 = len(ok) - int(np.argmax(ok[::-1]))
        xy = xy[i0:i1]
        head = xy[0] - xy[min(2, len(xy) - 1)]
        tail = xy[-1] - xy[max(-3, -len(xy))]
        xy = np.vstack([xy[0] + head / max(np.hypot(*head), 1e-9) * typical,
                        xy,
                        xy[-1] + tail / max(np.hypot(*tail), 1e-9) * typical])

    start = np.asarray(start_xy, dtype=float)
    if np.hypot(*(xy[-1] - start)) < np.hypot(*(xy[0] - start)):
        xy = xy[::-1]
    return Path(xy)
