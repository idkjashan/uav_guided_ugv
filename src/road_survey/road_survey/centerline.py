"""Road centre line from the road mask (medial axis -> longest path -> spline).

Optional for stage 1 -- the cost gradient in ``costmap.py`` already biases a
Nav2 plan towards the middle of the road.  This module exists because the
scoring rubric is *defined* on distance to the centre line, so having an
explicit one lets you measure your own deviation offline, and gives stage 2 a
reference path if you decide to drive it with pure pursuit instead of Nav2.
"""

from __future__ import annotations

from collections import deque

import numpy as np

from .costmap import distance_to_edge


def _skeletonize(mask: np.ndarray) -> np.ndarray:
    try:
        from skimage.morphology import skeletonize  # noqa: PLC0415
        return skeletonize(mask.astype(bool))
    except ImportError:
        pass
    try:
        import cv2  # noqa: PLC0415
        thin = cv2.ximgproc.thinning(mask.astype(np.uint8) * 255)
        return thin > 0
    except (ImportError, AttributeError) as exc:
        raise RuntimeError(
            'centre-line extraction needs a thinning implementation: '
            'sudo apt install python3-skimage  (or pip install scikit-image)'
        ) from exc


_NEIGHBOURS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def _adjacency(skel: np.ndarray):
    idx = {}
    pts = np.argwhere(skel)
    for i, (r, c) in enumerate(pts):
        idx[(int(r), int(c))] = i
    adj = [[] for _ in range(len(pts))]
    for (r, c), i in idx.items():
        for dr, dc in _NEIGHBOURS:
            j = idx.get((r + dr, c + dc))
            if j is not None:
                adj[i].append(j)
    return pts, adj


def _farthest(adj, start):
    dist = {start: 0}
    prev = {start: None}
    q = deque([start])
    last = start
    while q:
        u = q.popleft()
        last = u
        for v in adj[u]:
            if v not in dist:
                dist[v] = dist[u] + 1
                prev[v] = u
                q.append(v)
    return last, prev, dist


def longest_skeleton_path(mask: np.ndarray) -> np.ndarray:
    """(N, 2) array of (row, col) along the longest branch of the medial axis.

    Two breadth-first passes: the vertex farthest from an arbitrary start is an
    endpoint of a longest path, and the vertex farthest from *that* is the other
    endpoint.  Exact on a tree, good enough on the near-tree skeleton of a road.
    """
    skel = _skeletonize(mask)
    pts, adj = _adjacency(skel)
    if len(pts) < 2:
        return np.empty((0, 2), dtype=int)
    a, _, _ = _farthest(adj, 0)
    b, prev, _ = _farthest(adj, a)
    path = []
    node = b
    while node is not None:
        path.append(pts[node])
        node = prev[node]
    return np.array(path[::-1], dtype=int)


def path_to_world(path_rc: np.ndarray, origin_x: float, origin_y: float,
                  res: float) -> np.ndarray:
    """(row, col) cells -> (N, 2) map-frame XY at cell centres."""
    if path_rc.size == 0:
        return np.empty((0, 2))
    xy = np.empty((path_rc.shape[0], 2), dtype=float)
    xy[:, 0] = origin_x + (path_rc[:, 1] + 0.5) * res
    xy[:, 1] = origin_y + (path_rc[:, 0] + 0.5) * res
    return xy


def smooth_path(xy: np.ndarray, spacing_m: float = 1.0,
                smoothing: float = None) -> np.ndarray:
    """Fit a cubic B-spline and resample at uniform arc length."""
    if xy.shape[0] < 4:
        return xy
    from scipy.interpolate import splev, splprep  # noqa: PLC0415
    # splprep's s bounds the sum of squared residuals (m^2). N * (0.1 m)^2
    # keeps the spline within ~10 cm of the skeleton, which is the skeleton's
    # own noise at 0.25 m cells. s = N allowed ~1 m and cut every bend.
    s = 0.01 * xy.shape[0] if smoothing is None else float(smoothing)
    tck, _ = splprep([xy[:, 0], xy[:, 1]], s=s, k=3)
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()
    n = max(int(seg / max(spacing_m, 1e-3)), 2)
    u = np.linspace(0.0, 1.0, n)
    x, y = splev(u, tck)
    return np.stack([x, y], axis=1)


def elevation_along(xy: np.ndarray, elev: np.ndarray, origin_x: float,
                    origin_y: float, res: float) -> np.ndarray:
    """Sample the elevation grid at each centre-line point (NaN off-grid)."""
    h, w = elev.shape
    ix = np.floor((xy[:, 0] - origin_x) / res).astype(int)
    iy = np.floor((xy[:, 1] - origin_y) / res).astype(int)
    ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    out = np.full(xy.shape[0], np.nan, dtype=float)
    out[ok] = elev[iy[ok], ix[ok]]
    return out


def extract(mask: np.ndarray, origin_x: float, origin_y: float, res: float,
            spacing_m: float = 1.0):
    """mask -> (xy centre line, half-width in metres at each point)."""
    path_rc = longest_skeleton_path(mask)
    if path_rc.size == 0:
        return np.empty((0, 2)), np.empty(0)
    xy = smooth_path(path_to_world(path_rc, origin_x, origin_y, res), spacing_m)
    dist = distance_to_edge(mask, res)
    half = elevation_along(xy, dist, origin_x, origin_y, res)
    return xy, half
