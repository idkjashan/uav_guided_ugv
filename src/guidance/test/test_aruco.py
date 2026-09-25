"""Render the roof marker into a synthetic nadir image, recover the UGV pose.

The image is made the way Gazebo makes it: a 632 px texture (512 px black
marker inside a 60 px white border, same as the model's PNG) on a flat plate,
projected through the IMX214 pinhole from a known UAV pose. If a frame, a
sign or the marker-length convention is wrong, the recovered pose is off by
metres, not centimetres.
"""

import math

import cv2
import numpy as np
import pytest

from guidance.aruco import MarkerDetector, image_to_gray, marker_in_camera, ugv_pose_in_map
from road_survey.depth import intrinsics_from_hfov
from road_survey.frames import camera_extrinsics, rpy_to_rot

K = intrinsics_from_hfov(1920, 1080, 1.57)
CAM_POSE = [0.12, 0.03, 0.242, 0.0, math.pi / 2, 0.0]
FACE_M = 0.55
BLACK_M = FACE_M * 512 / 632
MARKER_H = 0.1275           # marker face above the UGV base_link
YAW_OFFSET = math.pi / 2    # arbitrary, so the test notices if it is ignored


def texture():
    m = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 0, 512)
    return cv2.copyMakeBorder(m, 60, 60, 60, 60, cv2.BORDER_CONSTANT, value=255)


def render(ugv, uav_xyz, uav_rpy, ugv_tilt=(0.0, 0.0), face_m=FACE_M, seed=0):
    x, y, z, yaw = ugv
    t_cam, r_body_opt = camera_extrinsics(CAM_POSE)
    r_map_body = rpy_to_rot(*uav_rpy)
    r_map_opt = r_map_body @ r_body_opt
    cam = r_map_body @ t_cam + np.asarray(uav_xyz, dtype=float)

    r_map_ugv = rpy_to_rot(ugv_tilt[0], ugv_tilt[1], yaw)
    r_map_marker = r_map_ugv @ rpy_to_rot(0.0, 0.0, -YAW_OFFSET)
    centre = np.array([x, y, z]) + r_map_ugv @ np.array([0.0, 0.0, MARKER_H])

    r1 = r_map_opt.T @ r_map_marker[:, 0]
    r2 = r_map_opt.T @ r_map_marker[:, 1]
    t = r_map_opt.T @ (centre - cam)
    tex = texture()
    n = tex.shape[0]
    s = face_m / n
    a = np.array([[s, 0.0, s * (0.5 - n / 2)],
                  [0.0, -s, s * (n / 2 - 0.5)],
                  [0.0, 0.0, 1.0]])
    h = K @ np.column_stack([r1, r2, t]) @ a

    rng = np.random.default_rng(seed)
    img = rng.normal(95, 12, size=(1080, 1920)).clip(0, 255).astype(np.uint8)
    cv2.warpPerspective(tex, h, (1920, 1080), dst=img, flags=cv2.INTER_AREA,
                        borderMode=cv2.BORDER_TRANSPARENT)
    return img, r_body_opt, t_cam, r_map_body


def recover(img, r_body_opt, t_cam, r_map_body, uav_xyz):
    corners = MarkerDetector()(img)
    if corners is None:
        return None
    r, t, err = marker_in_camera(corners, K, BLACK_M)
    assert err < 1.0
    return ugv_pose_in_map(r, t, r_body_opt, t_cam, r_map_body,
                           np.asarray(uav_xyz, dtype=float), YAW_OFFSET, MARKER_H)


CASES = [
    # ugv (x, y, z, yaw), uav offset from ugv, uav roll/pitch/yaw, ugv tilt,
    # yaw tolerance. Heading is pixel-limited: < 1 deg at the 10 m tracking
    # height, ~2.5 deg at the 16 m search ceiling where the marker is 26 px.
    ((10.0, 20.0, 5.0, 0.3), (0.0, 0.0, 10.0), (0.0, 0.0, 0.0), (0.0, 0.0), 1.0),
    ((10.0, 20.0, 5.0, -2.0), (2.5, -1.5, 10.0), (0.12, -0.08, 1.1), (0.0, 0.0), 1.0),
    ((-4.0, 3.0, 40.0, 2.9), (0.5, 0.5, 16.0), (0.0, 0.05, -2.2), (0.05, -0.1), 3.0),
    ((0.0, 0.0, 0.0, 1.0), (-3.0, 2.0, 8.0), (-0.15, 0.1, 0.4), (0.0, 0.14), 1.0),
]


@pytest.mark.parametrize('ugv,off,rpy,tilt,yaw_tol', CASES)
def test_recovers_ugv_pose(ugv, off, rpy, tilt, yaw_tol):
    uav = np.add(ugv[:3], off)
    img, r_body_opt, t_cam, r_map_body = render(ugv, uav, rpy, tilt)
    est = recover(img, r_body_opt, t_cam, r_map_body, uav)
    assert est is not None, 'marker not detected'
    x, y, z, yaw = est
    assert math.hypot(x - ugv[0], y - ugv[1]) < 0.05
    assert abs(z - ugv[2]) < 0.2
    dyaw = math.atan2(math.sin(yaw - ugv[3]), math.cos(yaw - ugv[3]))
    assert abs(math.degrees(dyaw)) < yaw_tol


def test_old_15cm_marker_is_too_small_from_12m():
    ugv = (0.0, 0.0, 0.0, 0.0)
    uav = (0.0, 0.0, 12.0)
    img, *_ = render(ugv, uav, (0.0, 0.0, 0.0), face_m=0.15)
    assert MarkerDetector()(img) is None


def test_image_to_gray_handles_padded_rgb_rows():
    class Msg:
        encoding = 'rgb8'
        height, width = 2, 3
        step = 12                       # 9 bytes of pixels + 3 of padding
        data = bytes([255, 0, 0, 0, 255, 0, 0, 0, 255, 7, 7, 7] * 2)
    g = image_to_gray(Msg())
    assert g.shape == (2, 3)
    assert g[0, 1] > g[0, 0] > g[0, 2]   # green > red > blue in luma
