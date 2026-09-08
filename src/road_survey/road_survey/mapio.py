"""Persistence for the survey product.

Three artefacts come out of a survey, all sharing one origin/resolution:

``road_map.npz``   every float layer, lossless.  This is the real output --
                   ``map_publisher_node`` and ``tune_offline`` both read it.
``road_map.pgm``   + ``road_map.yaml``, for ``nav2_map_server`` if you would
                   rather load the map the conventional way.  Quantised to 8
                   bits, so prefer the npz where you can.
``road_map.png``   a human-readable overlay for the report and for eyeballing
                   whether the thresholds were sane.
"""

from __future__ import annotations

import os

import cv2
import numpy as np


def save_npz(path: str, *, occ: np.ndarray, elevation: np.ndarray,
             risk: np.ndarray, road: np.ndarray, valid: np.ndarray,
             count: np.ndarray, extent: np.ndarray, origin_x: float,
             origin_y: float, resolution: float, frame_id: str,
             params: dict) -> str:
    os.makedirs(os.path.dirname(os.path.abspath(path)) or '.', exist_ok=True)
    np.savez_compressed(
        path,
        occ=occ.astype(np.int8),
        elevation=elevation.astype(np.float32),
        risk=risk.astype(np.float32),
        road=road.astype(bool),
        valid=valid.astype(bool),
        count=count.astype(np.int32),
        extent=extent.astype(np.float32),
        origin=np.array([origin_x, origin_y], dtype=np.float64),
        resolution=np.float64(resolution),
        frame_id=np.array(frame_id),
        params=np.array(repr(params)),
    )
    return path


def load_npz(path: str) -> dict:
    data = np.load(path, allow_pickle=False)
    out = {k: data[k] for k in data.files}
    out['origin_x'] = float(out['origin'][0])
    out['origin_y'] = float(out['origin'][1])
    out['resolution'] = float(out['resolution'])
    out['frame_id'] = str(out['frame_id'])
    return out


def save_map_server(prefix: str, occ: np.ndarray, resolution: float,
                    origin_x: float, origin_y: float) -> str:
    """Write ``<prefix>.pgm`` + ``<prefix>.yaml`` in nav2_map_server format."""
    from .costmap import occupancy_to_pgm  # noqa: PLC0415

    pgm = f'{prefix}.pgm'
    yaml_path = f'{prefix}.yaml'
    cv2.imwrite(pgm, occupancy_to_pgm(occ))
    with open(yaml_path, 'w', encoding='utf-8') as fh:
        fh.write(
            f'image: {os.path.basename(pgm)}\n'
            f'mode: scale\n'
            f'resolution: {resolution}\n'
            f'origin: [{origin_x}, {origin_y}, 0.0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.99\n'
            'free_thresh: 0.005\n'
        )
    return yaml_path


def _hillshade(elev: np.ndarray, resolution: float) -> np.ndarray:
    z = np.nan_to_num(elev, nan=float(np.nanmin(elev)) if np.isfinite(elev).any() else 0.0)
    gy, gx = np.gradient(z.astype(np.float32), resolution)
    shade = (gx * 0.6 + gy * 0.6 + 1.0) / 2.0
    return np.clip(shade, 0.0, 1.0)


def save_debug_png(path: str, *, elevation: np.ndarray, road: np.ndarray,
                   risk: np.ndarray, resolution: float,
                   centerline_rc: np.ndarray = None) -> str:
    """Grey hillshade, green road, red high-risk, magenta centre line."""
    shade = _hillshade(elevation, resolution)
    img = np.dstack([shade, shade, shade])
    unknown = ~np.isfinite(elevation)
    img[unknown] = 0.15
    hot = np.isfinite(risk) & (risk >= 0.9) & ~unknown
    img[hot] = np.array([0.15, 0.15, 0.75])   # BGR: red
    img[road.astype(bool)] = np.array([0.25, 0.85, 0.35])  # BGR: green
    if centerline_rc is not None and len(centerline_rc):
        rr = np.clip(centerline_rc[:, 0].astype(int), 0, img.shape[0] - 1)
        cc = np.clip(centerline_rc[:, 1].astype(int), 0, img.shape[1] - 1)
        img[rr, cc] = np.array([0.9, 0.2, 0.9])
    out = np.flipud((img * 255).astype(np.uint8))  # map y-up -> image y-down
    cv2.imwrite(path, out)
    return path
