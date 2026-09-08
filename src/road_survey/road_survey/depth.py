"""Depth image -> optical-frame point cloud.

We deliberately unproject the depth *image* ourselves instead of consuming
``/depth_camera/points``.  Gazebo's PointCloudPacked axis convention for a
depth sensor is easy to get wrong and impossible to unit test; a float depth
image plus an intrinsics matrix is unambiguous, a quarter of the bandwidth,
and lets us stride-subsample before any work happens.
"""

from __future__ import annotations

import math

import numpy as np


def intrinsics_from_hfov(width: int, height: int, hfov_rad: float) -> np.ndarray:
    """Pinhole K for a Gazebo camera specified by horizontal FOV.

    Gazebo uses square pixels and puts the principal point at the image centre,
    so ``fy == fx``.  For the StereoOV7251 (640x480, hfov 1.274 rad) this gives
    fx = fy = 432.5.
    """
    fx = (width / 2.0) / math.tan(hfov_rad / 2.0)
    return np.array([[fx, 0.0, width / 2.0],
                     [0.0, fx, height / 2.0],
                     [0.0, 0.0, 1.0]])


class Unprojector:
    """Caches the per-pixel ray directions for one (shape, stride, K)."""

    def __init__(self, k: np.ndarray, shape, stride: int = 2):
        self.k = np.asarray(k, dtype=float)
        self.stride = int(max(1, stride))
        self.shape = tuple(shape)
        h, w = self.shape
        fx, fy = self.k[0, 0], self.k[1, 1]
        cx, cy = self.k[0, 2], self.k[1, 2]
        us = np.arange(0, w, self.stride, dtype=np.float32)
        vs = np.arange(0, h, self.stride, dtype=np.float32)
        uu, vv = np.meshgrid(us, vs)
        # Unit-depth ray: multiply by z to get the metric point.
        self.ray_x = ((uu - cx) / fx).ravel()
        self.ray_y = ((vv - cy) / fy).ravel()

    def matches(self, k, shape, stride) -> bool:
        return (self.shape == tuple(shape)
                and self.stride == int(stride)
                and np.allclose(self.k, k))

    def __call__(self, depth: np.ndarray, z_min: float, z_max: float) -> np.ndarray:
        """(H, W) float depth in metres -> (N, 3) optical-frame points."""
        z = depth[::self.stride, ::self.stride].ravel().astype(np.float32, copy=False)
        good = np.isfinite(z) & (z > z_min) & (z < z_max)
        z = z[good]
        if z.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        out = np.empty((z.size, 3), dtype=np.float32)
        out[:, 0] = self.ray_x[good] * z
        out[:, 1] = self.ray_y[good] * z
        out[:, 2] = z
        return out


def decode_image(msg) -> np.ndarray:
    """sensor_msgs/Image -> float32 (H, W) depth in metres, without cv_bridge.

    Handles the two encodings a Gazebo depth camera can produce: 32FC1 (metres,
    what the OakD-Lite model emits) and 16UC1 (millimetres, RealSense style).
    """
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ('32fc1', '32f'):
        arr = buf.view(np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
        return arr.astype(np.float32, copy=False)
    if enc in ('16uc1', 'mono16'):
        arr = buf.view(np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
        return arr.astype(np.float32) * 1e-3
    raise ValueError(f'unsupported depth encoding: {msg.encoding!r}')


def k_from_camera_info(msg) -> np.ndarray:
    return np.asarray(msg.k, dtype=float).reshape(3, 3)
