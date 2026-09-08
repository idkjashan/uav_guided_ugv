"""Unprojection and image decoding."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from road_survey.depth import Unprojector, decode_image, intrinsics_from_hfov


def test_intrinsics_match_the_stereo_ov7251():
    k = intrinsics_from_hfov(640, 480, 1.274)
    assert math.isclose(k[0, 0], 432.5, abs_tol=0.5)
    assert k[0, 0] == k[1, 1]
    assert (k[0, 2], k[1, 2]) == (320.0, 240.0)


def test_flat_ground_footprint_matches_the_fov():
    """A constant-depth frame at h metres must span 2*h*tan(hfov/2) across."""
    h, hfov = 12.0, 1.274
    k = intrinsics_from_hfov(640, 480, hfov)
    pts = Unprojector(k, (480, 640), stride=1)(np.full((480, 640), h,
                                                       dtype=np.float32), 0.1, 30.0)
    assert pts.shape[0] == 480 * 640
    assert np.allclose(pts[:, 2], h)
    expected_half = h * math.tan(hfov / 2)
    assert math.isclose(pts[:, 0].max() - pts[:, 0].min(),
                        2 * expected_half, rel_tol=0.01)


def test_invalid_returns_are_dropped():
    k = intrinsics_from_hfov(8, 8, 1.0)
    depth = np.full((8, 8), 5.0, dtype=np.float32)
    depth[0, 0] = np.inf
    depth[0, 1] = np.nan
    depth[0, 2] = 0.0
    depth[0, 3] = 100.0
    pts = Unprojector(k, (8, 8), stride=1)(depth, 0.4, 18.0)
    assert pts.shape[0] == 60


def test_stride_subsamples():
    k = intrinsics_from_hfov(640, 480, 1.274)
    u = Unprojector(k, (480, 640), stride=4)
    pts = u(np.full((480, 640), 10.0, dtype=np.float32), 0.4, 18.0)
    assert pts.shape[0] == (480 // 4) * (640 // 4)
    assert u.matches(k, (480, 640), 4)
    assert not u.matches(k, (480, 640), 2)


def _img(arr, encoding):
    return SimpleNamespace(data=arr.tobytes(), encoding=encoding,
                           height=arr.shape[0], width=arr.shape[1],
                           step=arr.shape[1] * arr.dtype.itemsize)


def test_decode_32fc1_is_metres():
    arr = np.arange(12, dtype=np.float32).reshape(3, 4)
    assert np.allclose(decode_image(_img(arr, '32FC1')), arr)


def test_decode_16uc1_is_millimetres():
    arr = np.array([[1000, 2500]], dtype=np.uint16)
    assert np.allclose(decode_image(_img(arr, '16UC1')), [[1.0, 2.5]])


def test_decode_rejects_unknown_encoding():
    with pytest.raises(ValueError):
        decode_image(_img(np.zeros((2, 2), np.uint8), 'rgb8'))
