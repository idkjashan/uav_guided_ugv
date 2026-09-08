"""Geometric road detection: elevation grid -> slope / step / roughness -> risk.

Method
------
Follows the geometry-based risk formulation of Wang et al., "Aerial-Ground
Collaborative Continuous Risk Mapping for Autonomous Driving of UGV in Off-Road
Environments" (IEEE T-AES 2023), which is itself the slope / step / roughness
filter triple used by the ANYbotics ``traversability_estimation`` stack.

For every cell we fit a plane to the elevation over a local window and take

  slope      angle of that plane from horizontal
  roughness  RMS residual of the elevation about the fitted plane
  step       max(z) - min(z) over the 3x3 neighbourhood

Each is normalised by a critical value.  If any one exceeds its critical value
the cell is maximally risky; otherwise risk is their weighted sum.  Cells below
``risk_thresh`` are road.

Why this works here: a hill road is a locally flat shelf cut into a slope.  The
shelf scores ~0 on all three terms, the cut face scores 1 on slope, and the
outer edge scores 1 on step.  We are not really measuring "can the rover climb
it" -- we are using the risk map as a shape detector for that shelf.

The plane fit is done in closed form with separable box filters, so the whole
thing is a handful of ``scipy.ndimage`` passes and runs in tens of milliseconds
on the surveyed region.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage as ndi


@dataclass
class RiskParams:
    """All thresholds live here so ``tune_offline`` can sweep them."""

    # Local plane-fit window, in cells.  5 cells @ 0.25 m = 1.25 m, about a
    # third of a road width -- big enough to average depth noise, small enough
    # not to straddle the road edge.
    window_cells: int = 5
    # Pre-smoothing of the elevation before the fit, in cells (0 disables).
    smooth_cells: int = 3

    slope_crit_deg: float = 15.0
    step_crit_m: float = 0.25
    rough_crit_m: float = 0.12

    w_slope: float = 1.0 / 3.0
    w_step: float = 1.0 / 3.0
    w_rough: float = 1.0 / 3.0

    risk_thresh: float = 0.35
    min_count: int = 3

    def as_dict(self):
        return {f: getattr(self, f) for f in self.__dataclass_fields__}


@dataclass
class Features:
    slope_deg: np.ndarray
    step_m: np.ndarray
    rough_m: np.ndarray
    valid: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=bool))


def fill_unknown(elev: np.ndarray, known: np.ndarray) -> np.ndarray:
    """Nearest-neighbour fill so the box filters never see NaN.

    Filled cells are *not* trusted: ``terrain_features`` erodes ``known`` by the
    filter footprint so no output cell depends on invented data.
    """
    if known.all():
        return elev.astype(np.float32, copy=True)
    if not known.any():
        return np.zeros_like(elev, dtype=np.float32)
    _, idx = ndi.distance_transform_edt(~known, return_indices=True)
    return elev[tuple(idx)].astype(np.float32, copy=False)


def terrain_features(elev: np.ndarray, known: np.ndarray, res: float,
                     params: RiskParams) -> Features:
    """Slope (deg), step (m) and roughness (m) from a 2.5-D elevation grid."""
    w = int(params.window_cells)
    if w % 2 == 0:  # a symmetric window keeps the plane fit unbiased
        w += 1
    z = fill_unknown(elev, known)
    if params.smooth_cells and params.smooth_cells > 1:
        z = ndi.uniform_filter(z, size=int(params.smooth_cells), mode='nearest')

    # --- local least-squares plane, in closed form -------------------------
    # Offsets within the window, in metres.  Symmetric so E[x] = E[y] = 0.
    off = (np.arange(w, dtype=np.float64) - (w - 1) / 2.0) * res
    var_off = float(np.mean(off ** 2))  # == res^2 (w^2 - 1) / 12

    z64 = z.astype(np.float64)
    m1 = ndi.uniform_filter(z64, size=w, mode='nearest')
    m2 = ndi.uniform_filter(z64 * z64, size=w, mode='nearest')
    # E[x*z] over the 2-D window: weighted mean along x, plain mean along y.
    exz = ndi.uniform_filter1d(
        ndi.correlate1d(z64, off / w, axis=1, mode='nearest'),
        size=w, axis=0, mode='nearest')
    eyz = ndi.uniform_filter1d(
        ndi.correlate1d(z64, off / w, axis=0, mode='nearest'),
        size=w, axis=1, mode='nearest')
    gx = exz / var_off
    gy = eyz / var_off

    slope_deg = np.degrees(np.arctan(np.hypot(gx, gy))).astype(np.float32)

    # Residual variance about the fitted plane (total minus what the plane
    # explains).  Clamped because float error can push it slightly negative.
    resid_var = m2 - m1 * m1 - (gx * gx + gy * gy) * var_off
    rough_m = np.sqrt(np.maximum(resid_var, 0.0)).astype(np.float32)

    step_m = (ndi.maximum_filter(z, size=3, mode='nearest')
              - ndi.minimum_filter(z, size=3, mode='nearest')).astype(np.float32)

    # A cell is only valid if its whole filter footprint was really measured.
    pad = max(w // 2, 1)
    valid = ndi.binary_erosion(known, structure=np.ones((3, 3), bool),
                               iterations=pad, border_value=0)
    return Features(slope_deg, step_m, rough_m, valid)


def risk_from_features(feat: Features, params: RiskParams) -> np.ndarray:
    """Per-cell risk in [0, 1]; 1.0 (max) wherever a critical value is passed."""
    ns = np.clip(feat.slope_deg / max(params.slope_crit_deg, 1e-6), 0.0, 1.0)
    ng = np.clip(feat.step_m / max(params.step_crit_m, 1e-6), 0.0, 1.0)
    nu = np.clip(feat.rough_m / max(params.rough_crit_m, 1e-6), 0.0, 1.0)
    wsum = max(params.w_slope + params.w_step + params.w_rough, 1e-6)
    risk = (params.w_slope * ns + params.w_step * ng + params.w_rough * nu) / wsum
    over = (ns >= 1.0) | (ng >= 1.0) | (nu >= 1.0)
    risk[over] = 1.0
    return risk.astype(np.float32)


def classify(elev: np.ndarray, known: np.ndarray, res: float,
             params: RiskParams):
    """Convenience wrapper: -> (risk, road_mask, valid, features)."""
    feat = terrain_features(elev, known, res, params)
    risk = risk_from_features(feat, params)
    road = (risk < params.risk_thresh) & feat.valid
    return risk, road, feat.valid, feat


def multi_level_cells(extent: np.ndarray, road: np.ndarray,
                      gap_m: float = 2.0) -> np.ndarray:
    """Road cells whose samples span more than ``gap_m`` vertically.

    On a switchback these are the places where two levels of the road project
    onto the same XY cell -- exactly where a single 2-D costmap would lie to
    the planner.  See ``height_slicer_node`` for what to do about them.
    """
    with np.errstate(invalid='ignore'):
        return road & (np.nan_to_num(extent, nan=0.0) > float(gap_m))
