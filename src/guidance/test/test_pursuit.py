"""Pure pursuit on a simulated skid-steer UGV with a marker-quality pose.

The pose the controller sees is what the localiser delivers: ~10 Hz, 0.1 s
late, a few cm and ~1 deg of noise. The rover itself is a unicycle with the
DiffDrive plugin's acceleration limit.
"""

import math

import numpy as np
import pytest

from guidance.pursuit import Path, PursuitParams, centerline_path, command


def s_curve(n=400):
    x = np.linspace(0.0, 60.0, n)
    return np.column_stack([x, 6.0 * np.sin(x / 8.0)])


def hairpin():
    """Two parallel legs 6 m apart joined by a 3 m radius U-turn."""
    a = np.column_stack([np.linspace(0, 30, 120), np.zeros(120)])
    t = np.linspace(-math.pi / 2, math.pi / 2, 60)
    u = np.column_stack([30 + 3 * np.cos(t), 3 + 3 * np.sin(t)])
    b = np.column_stack([np.linspace(30, 0, 120), np.full(120, 6.0)])
    return np.vstack([a, u[1:], b[1:]])


def drive(path_xy, start, p=PursuitParams(), seed=0, t_max=300.0):
    rng = np.random.default_rng(seed)
    path = Path(path_xy)
    x, y, yaw = start
    v = w = 0.0
    dt, lag = 0.05, 2                 # 20 Hz control, pose 0.1 s late
    history = []
    seen = None
    s = None
    ctes = []
    for i in range(int(t_max / dt)):
        history.append((x, y, yaw))
        if i % 2 == 0 and len(history) > lag:        # 10 Hz pose updates
            hx, hy, hyaw = history[-1 - lag]
            seen = (hx + rng.normal(0, 0.03), hy + rng.normal(0, 0.03),
                    hyaw + rng.normal(0, math.radians(1.0)))
        if seen is None:
            continue
        cmd = command(*seen, path, s, p)
        s = cmd.s
        if cmd.done:
            return True, np.array(ctes), (x, y)
        # DiffDrive accel limit is 3 m/s^2
        v += np.clip(cmd.v - v, -3 * dt, 3 * dt)
        w += np.clip(cmd.w - w, -3 * dt, 3 * dt)
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
        ctes.append(path.project(np.array([x, y]))[1])
    return False, np.array(ctes), (x, y)


def test_follows_an_s_curve_to_the_end():
    xy = s_curve()
    done, cte, end = drive(xy, (0.0, 0.0, 0.3))
    assert done
    assert math.hypot(end[0] - xy[-1, 0], end[1] - xy[-1, 1]) < 0.8
    assert np.percentile(cte, 95) < 0.15
    assert cte.max() < 0.3


def test_hairpin_does_not_jump_to_the_other_leg():
    xy = hairpin()
    done, cte, _ = drive(xy, (0.0, 0.0, 0.0))
    assert done
    assert cte.max() < 0.6


def test_rotates_in_place_when_facing_backwards():
    path = Path(s_curve())
    cmd = command(0.0, 0.0, math.pi, path, None, PursuitParams())
    assert cmd.v == 0.0 and abs(cmd.w) > 0.0
    done, _, _ = drive(s_curve(), (0.0, 0.0, math.pi))
    assert done


def test_stops_at_the_goal():
    path = Path(s_curve())
    end = path.xy[-1]
    cmd = command(end[0] - 0.1, end[1], 0.0, path, path.length - 0.2, PursuitParams())
    assert cmd.done and cmd.v == 0.0 and cmd.w == 0.0


def test_centerline_path_fills_the_ugv_hole_and_starts_at_the_ugv():
    pytest.importorskip('skimage')
    res = 0.25
    occ = np.full((80, 400), 100, dtype=np.int8)       # 20 m x 100 m
    occ[28:52, 10:390] = 20                            # 6 m wide road along x
    occ[36:44, 40:46] = 100                            # the UGV, 1.5 m x 2 m
    path = centerline_path(occ, 0.0, 0.0, res, start_xy=(11.0, 10.0))
    assert path is not None
    assert path.xy[0, 0] < path.xy[-1, 0]              # oriented away from the UGV
    assert path.length > 80.0
    assert np.abs(path.xy[:, 1] - 10.0).max() < 0.5    # no loop around the hole
