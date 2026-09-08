"""Road mask -> cleaned mask -> nav2-compatible occupancy costs.

The occupancy values we emit are *not* trinary.  Road cells carry a cost that
falls from 99 at the road edge to 0 in the middle of the road, so a Nav2
planner that is configured with ``trinary_costmap: false`` naturally prefers
the centre line without us having to plan one explicitly.  That matters: the
problem statement scores lateral deviation from the road centre, so "cheapest
path" and "highest scoring path" have to be the same thing.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

FREE = 0
LETHAL = 100
UNKNOWN = -1


@dataclass
class CostmapParams:
    open_radius_m: float = 0.4      # bites off single-cell speckle
    close_radius_m: float = 0.8     # bridges gaps from missing depth returns
    min_area_m2: float = 25.0       # drop blobs smaller than this
    keep_components: int = 1        # keep the N largest blobs (0 = keep all)
    edge_cost_ref_m: float = 1.5    # distance from the edge where cost hits 0
    unknown_is_lethal: bool = True  # unsurveyed ground is not drivable

    def as_dict(self):
        return {f: getattr(self, f) for f in self.__dataclass_fields__}


def _disk(radius_m: float, res: float):
    r = int(round(radius_m / res))
    if r < 1:
        return None
    k = 2 * r + 1
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))


def clean_mask(road: np.ndarray, res: float, params: CostmapParams,
               seed_cell=None):
    """Morphological cleanup + connected-component filtering.

    ``seed_cell`` is an optional ``(ix, iy)`` -- normally the UGV's start cell.
    If given, the component containing it is always kept even if it is not the
    largest.  Returns ``(mask, stats_dict)``.
    """
    m = road.astype(np.uint8)
    k_open = _disk(params.open_radius_m, res)
    if k_open is not None:
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k_open)
    k_close = _disk(params.close_radius_m, res)
    if k_close is not None:
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k_close)

    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    cell_area = res * res
    min_cells = params.min_area_m2 / cell_area

    keep = set()
    if seed_cell is not None:
        ix, iy = seed_cell
        if 0 <= iy < labels.shape[0] and 0 <= ix < labels.shape[1]:
            lab = int(labels[iy, ix])
            if lab != 0:
                keep.add(lab)
    order = sorted(range(1, n_labels),
                   key=lambda i: int(stats[i, cv2.CC_STAT_AREA]), reverse=True)
    big = [i for i in order if stats[i, cv2.CC_STAT_AREA] >= min_cells]
    if params.keep_components > 0:
        keep.update(big[:params.keep_components])
    else:
        keep.update(big)

    out = np.isin(labels, list(keep)) if keep else np.zeros_like(m, dtype=bool)
    info = {
        'components_found': int(n_labels - 1),
        'components_kept': len(keep),
        'largest_area_m2': float(stats[order[0], cv2.CC_STAT_AREA] * cell_area)
        if order else 0.0,
        'road_area_m2': float(np.count_nonzero(out) * cell_area),
        'seed_in_road': bool(seed_cell is not None and keep and
                             0 <= seed_cell[1] < out.shape[0] and
                             0 <= seed_cell[0] < out.shape[1] and
                             out[seed_cell[1], seed_cell[0]]),
    }
    return out, info


def distance_to_edge(mask: np.ndarray, res: float) -> np.ndarray:
    """Metres from each road cell to the nearest non-road cell (0 off-road)."""
    return cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5) * res


def road_width_m(mask: np.ndarray, res: float) -> np.ndarray:
    """2x the distance transform -- an estimate of local road width."""
    return 2.0 * distance_to_edge(mask, res)


def to_occupancy(mask: np.ndarray, valid: np.ndarray, res: float,
                 params: CostmapParams) -> np.ndarray:
    """(H, W) int8 occupancy: 0..99 on the road, 100 off it, -1 unknown."""
    dist = distance_to_edge(mask, res)
    ref = max(params.edge_cost_ref_m, 1e-3)
    grad = np.clip(99.0 * (1.0 - dist / ref), 0.0, 99.0)

    occ = np.full(mask.shape, LETHAL, dtype=np.int16)
    occ[mask] = grad[mask].astype(np.int16)
    if not params.unknown_is_lethal:
        occ[~valid & ~mask] = UNKNOWN
    return occ.astype(np.int8)


def occupancy_to_pgm(occ: np.ndarray) -> np.ndarray:
    """int8 occupancy -> the 0..255 image map_server expects (row 0 at top).

    map_server reads image row 0 as the *top* of the map while OccupancyGrid
    row 0 is the *bottom*, hence the flip.
    """
    img = np.empty(occ.shape, dtype=np.uint8)
    unknown = occ < 0
    val = np.clip(occ.astype(np.float32), 0.0, 100.0) / 100.0
    img[:] = np.round(255.0 * (1.0 - val)).astype(np.uint8)
    img[unknown] = 205
    return np.flipud(img)
