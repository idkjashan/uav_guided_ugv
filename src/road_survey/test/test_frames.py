"""Frame conversions. These are the bugs that silently ruin a map, so they get
the most tests."""

import math

import numpy as np
import pytest

from road_survey.frames import (camera_extrinsics, ned_to_enu, optical_to_map,
                                px4_pose_to_map, quat_to_rot, rpy_to_rot,
                                yaw_from_rot, yaw_ned_to_enu)

IDENTITY_Q = (1.0, 0.0, 0.0, 0.0)


def test_quat_to_rot_identity():
    assert np.allclose(quat_to_rot(IDENTITY_Q), np.eye(3))


def test_rotations_are_proper():
    for r in (rpy_to_rot(0.1, -0.4, 2.0),
              quat_to_rot((0.5, 0.5, 0.5, 0.5)),
              camera_extrinsics([0, 0, 0, 0, 1.5708, 0])[1]):
        assert np.isclose(np.linalg.det(r), 1.0)
        assert np.allclose(r @ r.T, np.eye(3), atol=1e-12)


def test_ned_to_enu_swaps_and_flips():
    assert np.allclose(ned_to_enu([10.0, 5.0, -20.0]), [5.0, 10.0, 20.0])


def test_yaw_ned_to_enu():
    assert math.isclose(yaw_ned_to_enu(0.0), math.pi / 2)        # north
    assert math.isclose(yaw_ned_to_enu(math.pi / 2), 0.0)        # east


def test_level_north_facing_pose():
    """Identity PX4 attitude = nose north, level. In ENU that is yaw +90 deg."""
    p, r = px4_pose_to_map([10.0, 5.0, -20.0], IDENTITY_Q)
    assert np.allclose(p, [5.0, 10.0, 20.0])
    assert np.allclose(r @ [1, 0, 0], [0, 1, 0], atol=1e-12)   # forward -> north
    assert np.allclose(r @ [0, 1, 0], [-1, 0, 0], atol=1e-12)  # left -> west
    assert np.allclose(r @ [0, 0, 1], [0, 0, 1], atol=1e-12)   # up -> up
    assert math.isclose(yaw_from_rot(r), math.pi / 2)


def test_yaw_ninety_in_ned_is_east():
    q = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))  # yaw +90 NED
    _, r = px4_pose_to_map([0, 0, 0], q)
    assert np.allclose(r @ [1, 0, 0], [1, 0, 0], atol=1e-9)  # forward -> east
    assert abs(yaw_from_rot(r)) < 1e-9


def test_downward_camera_points_optical_z_at_the_ground():
    t, r = camera_extrinsics([0.12, 0.03, 0.242, 0.0, 1.5708, 0.0])
    assert np.allclose(t, [0.12, 0.03, 0.242])
    assert np.allclose(r @ [0, 0, 1], [0, 0, -1], atol=1e-4)  # optical fwd -> down
    assert np.allclose(r @ [1, 0, 0], [0, -1, 0], atol=1e-4)  # image right -> body right


def test_ground_point_lands_at_zero_height():
    """UAV hovering 10 m up, level: a return 10 m along the optical axis is ground."""
    _, r_body_opt = camera_extrinsics([0.0, 0.0, 0.0, 0.0, math.pi / 2, 0.0])
    p_map, r_map_body = px4_pose_to_map([0.0, 0.0, -10.0], IDENTITY_Q)
    pts = optical_to_map(np.array([[0.0, 0.0, 10.0]]), r_body_opt,
                         np.zeros(3), r_map_body, p_map)
    assert np.allclose(pts[0], [0.0, 0.0, 0.0], atol=1e-9)


@pytest.mark.parametrize('offset,expect', [
    ((1.0, 0.0), (1.0, 0.0)),    # image right -> body right -> east
    ((0.0, 1.0), (0.0, -1.0)),  # image down  -> body rear  -> south
])
def test_offset_points_keep_their_sign(offset, expect):
    """A return offset in the image plane must not mirror when mapped.

    Facing north and looking down, +X_optical (image right) is the body's right,
    which is east; +Y_optical (image down) is the body's rear, which is south.
    """
    _, r_body_opt = camera_extrinsics([0.0, 0.0, 0.0, 0.0, math.pi / 2, 0.0])
    p_map, r_map_body = px4_pose_to_map([0.0, 0.0, -10.0], IDENTITY_Q)
    pts = optical_to_map(np.array([[offset[0], offset[1], 10.0]]), r_body_opt,
                         np.zeros(3), r_map_body, p_map)
    assert np.allclose(pts[0, :2], expect, atol=1e-9)
    assert np.isclose(pts[0, 2], 0.0)
