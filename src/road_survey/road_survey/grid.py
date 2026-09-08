"""2.5-D accumulation grid.

One cell holds running statistics over every depth sample that ever landed in
it: count, sum(z), sum(z^2), min(z), max(z).  From those we get the mean
elevation (the map), the within-cell standard deviation (sensor noise plus real
micro-roughness) and the vertical extent (which is how we notice that two
levels of a switchback road stack on the same XY cell).

Deliberately *not* a probabilistic / Kalman elevation map: with a static world,
ground-truth-quality PX4 odometry in SITL and a nadir camera, a running mean is
within a couple of centimetres of anything fancier and has no tuning knobs.
"""

from __future__ import annotations

import numpy as np

# Index used for "no data" when compressing to a saved map.
UNKNOWN = np.float32(np.nan)


class TerrainGrid:
    """Axis-aligned grid in the map (ENU) frame.

    Cell (row=iy, col=ix) covers
    ``[origin_x + ix*res, origin_x + (ix+1)*res) x [origin_y + iy*res, ...)``.
    ``origin_*`` is the *lower-left corner*, matching nav_msgs/OccupancyGrid.
    """

    def __init__(self, origin_x: float, origin_y: float,
                 width: int, height: int, resolution: float,
                 track_extrema: bool = True):
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.width = int(width)
        self.height = int(height)
        self.res = float(resolution)
        self.track_extrema = bool(track_extrema)

        shape = (self.height, self.width)
        self.count = np.zeros(shape, dtype=np.int32)
        self.z_sum = np.zeros(shape, dtype=np.float64)
        self.z_sq = np.zeros(shape, dtype=np.float64)
        if self.track_extrema:
            self.z_min = np.full(shape, np.inf, dtype=np.float32)
            self.z_max = np.full(shape, -np.inf, dtype=np.float32)
        else:
            self.z_min = self.z_max = None
        self.dropped = 0  # points that fell outside the grid

    # -- construction helpers -------------------------------------------------

    @classmethod
    def centered_on(cls, cx: float, cy: float, size_m: float,
                    resolution: float, track_extrema: bool = True) -> 'TerrainGrid':
        n = int(round(size_m / resolution))
        return cls(cx - 0.5 * n * resolution, cy - 0.5 * n * resolution,
                   n, n, resolution, track_extrema)

    # -- accumulation ---------------------------------------------------------

    def add_points(self, pts: np.ndarray) -> int:
        """Fold an (N, 3) array of map-frame points in.  Returns points used."""
        if pts.size == 0:
            return 0
        ix = np.floor((pts[:, 0] - self.origin_x) / self.res).astype(np.int64)
        iy = np.floor((pts[:, 1] - self.origin_y) / self.res).astype(np.int64)
        ok = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        n_ok = int(np.count_nonzero(ok))
        self.dropped += int(pts.shape[0] - n_ok)
        if n_ok == 0:
            return 0
        flat = (iy[ok] * self.width + ix[ok])
        z = pts[ok, 2].astype(np.float64)
        n_cells = self.width * self.height

        self.count += np.bincount(flat, minlength=n_cells).reshape(
            self.height, self.width).astype(np.int32)
        self.z_sum += np.bincount(flat, weights=z, minlength=n_cells).reshape(
            self.height, self.width)
        self.z_sq += np.bincount(flat, weights=z * z, minlength=n_cells).reshape(
            self.height, self.width)
        if self.track_extrema:
            np.minimum.at(self.z_min.reshape(-1), flat, z.astype(np.float32))
            np.maximum.at(self.z_max.reshape(-1), flat, z.astype(np.float32))
        return n_ok

    # -- readout --------------------------------------------------------------

    def known(self, min_count: int = 3) -> np.ndarray:
        return self.count >= int(min_count)

    def elevation(self, min_count: int = 3) -> np.ndarray:
        """Mean height per cell, NaN where unknown."""
        known = self.known(min_count)
        out = np.full(self.count.shape, np.nan, dtype=np.float32)
        n = np.maximum(self.count, 1)
        out[known] = (self.z_sum[known] / n[known]).astype(np.float32)
        return out

    def within_cell_std(self, min_count: int = 3) -> np.ndarray:
        """sqrt(E[z^2] - E[z]^2) per cell, NaN where unknown."""
        known = self.known(min_count)
        out = np.full(self.count.shape, np.nan, dtype=np.float32)
        n = np.maximum(self.count, 1).astype(np.float64)
        var = self.z_sq / n - (self.z_sum / n) ** 2
        out[known] = np.sqrt(np.maximum(var[known], 0.0)).astype(np.float32)
        return out

    def vertical_extent(self, min_count: int = 3) -> np.ndarray:
        """max(z) - min(z) per cell, NaN where unknown or untracked."""
        if not self.track_extrema:
            return np.full(self.count.shape, np.nan, dtype=np.float32)
        known = self.known(min_count)
        out = np.full(self.count.shape, np.nan, dtype=np.float32)
        out[known] = (self.z_max[known] - self.z_min[known])
        return out

    # -- geometry -------------------------------------------------------------

    def world_to_cell(self, x: float, y: float):
        return (int(np.floor((x - self.origin_x) / self.res)),
                int(np.floor((y - self.origin_y) / self.res)))

    def cell_to_world(self, ix: int, iy: int):
        return (self.origin_x + (ix + 0.5) * self.res,
                self.origin_y + (iy + 0.5) * self.res)

    def coverage_m2(self, min_count: int = 3) -> float:
        return float(np.count_nonzero(self.known(min_count))) * self.res ** 2


def known_bbox(known: np.ndarray, margin: int = 8):
    """Bounding box (r0, r1, c0, c1) of True cells, padded, clipped.

    Lets the classifier run on the surveyed strip instead of the whole grid --
    on a 1600x1600 grid that is the difference between 40 ms and 2 s per pass.
    Returns None if nothing is known yet.
    """
    rows = np.flatnonzero(known.any(axis=1))
    cols = np.flatnonzero(known.any(axis=0))
    if rows.size == 0 or cols.size == 0:
        return None
    r0 = max(int(rows[0]) - margin, 0)
    r1 = min(int(rows[-1]) + margin + 1, known.shape[0])
    c0 = max(int(cols[0]) - margin, 0)
    c1 = min(int(cols[-1]) + margin + 1, known.shape[1])
    return r0, r1, c0, c1
