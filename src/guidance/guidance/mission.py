"""UAV mission: take off, survey the road, fly back, lock onto the UGV.

Pure logic, no ROS, so a test can fly it end to end. The node feeds it the
vehicle position and a few sensor facts each tick and publishes the setpoint
it returns. Positions are PX4 local NED (x north, y east, z down); the road
costmap and the UGV pose come in as map-frame ENU like the rest of the
project, and are converted here.

The setpoint never jumps. It is a carrot that moves towards the current goal
at a bounded speed, and its velocity goes to PX4 as feed-forward. Jumping a
position setpoint tens of metres makes PX4 fly flat out at MPC_XY_VEL_MAX,
which is the "erratic" flight this replaces.

    TAKEOFF -> SURVEY -> RETURN -> ACQUIRE -> TRACK
       \\___(survey disabled)___/
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .explore import ExploreParams, next_waypoint

TAKEOFF, SURVEY, RETURN, ACQUIRE, TRACK = 'TAKEOFF', 'SURVEY', 'RETURN', 'ACQUIRE', 'TRACK'


def swap_xy(v):
    """NED <-> ENU in the horizontal plane: north/east swap places."""
    return np.array([v[1], v[0]])


@dataclass
class MissionParams:
    survey: bool = True              # False: a map already exists, go straight to the UGV
    survey_agl_m: float = 12.0       # depth far clip is 19.1 m
    track_agl_m: float = 10.0
    max_search_agl_m: float = 16.0   # climb to here to widen the view when the UGV is lost
    survey_speed: float = 1.5        # m/s; the costmap refreshes every 2 s
    return_speed: float = 3.0
    track_speed: float = 3.0         # max carrot speed while chasing the UGV
    climb_speed: float = 1.0
    arrive_tol_m: float = 0.5
    end_misses: int = 3              # costmaps in a row with no road ahead -> end of road
    max_survey_m: float = 1500.0
    crumb_spacing_m: float = 2.0
    search_after_s: float = 3.0      # UGV unseen this long -> start climbing
    heading_enu: float = float('nan')  # survey direction; NaN = UAV heading at takeoff
    ground_filter: float = 0.2       # low-pass on the depth ground estimate
    explore: ExploreParams = field(default_factory=ExploreParams)


@dataclass
class Setpoint:
    pos: np.ndarray    # NED
    vel: np.ndarray    # NED feed-forward
    yaw: float         # NED


class Mission:

    def __init__(self, p: MissionParams, home_ned, yaw_ned: float, t: float):
        self.p = p
        self.home = np.asarray(home_ned, dtype=float)
        self.yaw = float(yaw_ned)
        self.sp = self.home.copy()
        self.vel = np.zeros(3)
        self.state = TAKEOFF
        self.t_state = t
        self.heading = (p.heading_enu if math.isfinite(p.heading_enu)
                        else math.pi / 2 - self.yaw)
        self.goal = self.home.copy()
        self.goal[2] = self.home[2] - (p.survey_agl_m if p.survey else p.track_agl_m)
        self.crumbs = []
        self.crumb_i = 0
        self.misses = 0
        self.travelled = 0.0
        self.ground_z = None         # NED z of the ground under the UAV
        self.ugv_z = None            # ENU z of the UGV, last seen
        self.last_seen = None
        self.save_requested = False  # the node calls the mapper's save service
        self.end_reason = ''

    # -- helpers ---------------------------------------------------------------

    def _enter(self, state, t):
        self.state = state
        self.t_state = t

    def _move(self, speed_xy, dt):
        """Carrot one tick towards self.goal. -> True once it is there."""
        d = self.goal[:2] - self.sp[:2]
        n = float(np.hypot(*d))
        step = speed_xy * dt
        if n <= step:
            self.sp[:2] = self.goal[:2]
            self.vel[:2] = 0.0
        else:
            self.sp[:2] += d / n * step
            self.vel[:2] = d / n * speed_xy
        self.travelled += min(n, step)
        dz = self.goal[2] - self.sp[2]
        stepz = self.p.climb_speed * dt
        if abs(dz) <= stepz:
            self.sp[2] = self.goal[2]
            self.vel[2] = 0.0
        else:
            self.sp[2] += math.copysign(stepz, dz)
            self.vel[2] = math.copysign(self.p.climb_speed, dz)
        return n <= step and abs(dz) <= stepz

    # -- the tick --------------------------------------------------------------

    def step(self, t, dt, uav_ned, ground_z=None, costmap=None, ugv=None) -> Setpoint:
        """One tick.

        ``ground_z``  NED z of the ground below the UAV (from depth), or None.
        ``costmap``   (occ, origin_x, origin_y, res) when a new one arrived, else None.
        ``ugv``       (x, y, z) ENU of the UGV if seen within the timeout, else None.
        """
        if ground_z is not None:
            a = self.p.ground_filter
            self.ground_z = ground_z if self.ground_z is None else (
                (1 - a) * self.ground_z + a * ground_z)
        if ugv is not None:
            self.last_seen = t
            self.ugv_z = float(ugv[2])
        getattr(self, '_' + self.state.lower())(t, dt, np.asarray(uav_ned, float),
                                                costmap, ugv)
        return Setpoint(self.sp.copy(), self.vel.copy(), self.yaw)

    def _takeoff(self, t, dt, uav, costmap, ugv):
        self._move(0.0, dt)
        if abs(uav[2] - self.goal[2]) < self.p.arrive_tol_m:
            self.crumbs = [self.sp.copy()]
            if not self.p.survey:
                self._enter(ACQUIRE, t)
                return
            # Blind first leg: the mapper only integrates a frame once the UAV
            # has moved sideways, so right after a vertical climb the map is a
            # patch the size of the takeoff footprint.
            h = np.array([math.cos(self.heading), math.sin(self.heading)])
            self.goal[:2] = self.sp[:2] + swap_xy(h * self.p.explore.lookahead_m)
            self.travelled = 0.0
            self._enter(SURVEY, t)

    def _survey(self, t, dt, uav, costmap, ugv):
        p = self.p
        if self.ground_z is not None:
            self.goal[2] = self.ground_z - p.survey_agl_m
        if costmap is not None:
            occ, ox, oy, res = costmap
            here = swap_xy(self.sp)
            wp = next_waypoint(occ, ox, oy, res, here, self.heading, p.explore)
            if wp is not None:
                d = np.asarray(wp) - here
                if np.hypot(*d) > 1e-3:
                    self.heading = math.atan2(d[1], d[0])
                self.goal[:2] = swap_xy(wp)
                self.misses = 0
            elif self.travelled >= p.explore.lookahead_m:
                self.misses += 1
        arrived = self._move(p.survey_speed, dt)
        if np.hypot(*(self.sp[:2] - self.crumbs[-1][:2])) >= p.crumb_spacing_m:
            self.crumbs.append(self.sp.copy())

        ended = arrived and self.misses >= p.end_misses
        if ended or self.travelled >= p.max_survey_m:
            self.end_reason = 'road ended' if ended else 'max_survey_m reached'
            self.crumbs.append(self.sp.copy())
            self.crumb_i = len(self.crumbs) - 1
            self.save_requested = True
            self._enter(RETURN, t)

    def _return(self, t, dt, uav, costmap, ugv):
        # Retrace the survey: every breadcrumb was at survey height over the
        # terrain under it, so the way back cannot fly into a hillside.
        self.goal = self.crumbs[self.crumb_i].copy()
        if self._move(self.p.return_speed, dt):
            self.crumb_i -= 1
            if self.crumb_i < 0:
                self.goal = self.home.copy()
                self.goal[2] = self.home[2] - self.p.track_agl_m
                self._enter(ACQUIRE, t)

    def _acquire(self, t, dt, uav, costmap, ugv):
        if ugv is not None:
            self._enter(TRACK, t)
            self._track(t, dt, uav, costmap, ugv)
            return
        if t - self.t_state > self.p.search_after_s:
            self.goal[2] = self.home[2] - self.p.max_search_agl_m
        self._move(self.p.track_speed, dt)

    def _track(self, t, dt, uav, costmap, ugv):
        p = self.p
        if ugv is not None:
            self.goal[:2] = swap_xy(ugv[:2])
            self.goal[2] = -self.ugv_z - p.track_agl_m
        elif self.last_seen is not None and t - self.last_seen > p.search_after_s:
            self.goal[2] = -self.ugv_z - p.max_search_agl_m
        # ponytail: no UGV-velocity feed-forward, so the UAV trails a moving
        # UGV by ~v/MPC_XY_P (~1 m at 1 m/s). Fine at 10 m; add an
        # alpha-beta velocity estimate if the marker ever leaves the frame.
        self._move(p.track_speed, dt)
