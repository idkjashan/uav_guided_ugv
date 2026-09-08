"""Coordinate frame conversions.

Frames used in this package
---------------------------
map        ENU, origin = PX4 EKF local origin (the UAV spawn point).
           +X East, +Y North, +Z Up.
base_link  FLU body frame. +X forward, +Y left, +Z up.
px4 world  NED. +X North, +Y East, +Z Down.  (VehicleLocalPosition)
px4 body   FRD. +X forward, +Y right, +Z down. (VehicleAttitude q: FRD -> NED)
gz camera  the frame the SDF <pose> of the camera link is expressed in:
           +X along the optical axis, +Y left, +Z up (Gazebo convention).
optical    REP-103 camera optical frame: +X right, +Y down, +Z forward.

Everything here is pure numpy so it can be unit tested without ROS.
"""

from __future__ import annotations

import math

import numpy as np

# NED <-> ENU.  Involutive: S @ S == I.  det(S) == +1 so it is a rotation.
S_NED_ENU = np.array([[0.0, 1.0, 0.0],
                      [1.0, 0.0, 0.0],
                      [0.0, 0.0, -1.0]])

# FLU -> FRD (and its own inverse).
D_FLU_FRD = np.diag([1.0, -1.0, -1.0])

# optical (RDF) -> gazebo camera (FLU-ish).  Maps optical +Z to camera +X.
R_GZCAM_OPT = np.array([[0.0, 0.0, 1.0],
                        [-1.0, 0.0, 0.0],
                        [0.0, -1.0, 0.0]])


def quat_to_rot(q) -> np.ndarray:
    """Rotation matrix from a (w, x, y, z) quaternion (PX4 order)."""
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat(r: np.ndarray) -> np.ndarray:
    """Rotation matrix -> (w, x, y, z), the inverse of :func:`quat_to_rot`."""
    tr = float(r[0, 0] + r[1, 1] + r[2, 2])
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = [0.25 * s, (r[2, 1] - r[1, 2]) / s,
             (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s]
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        q = [(r[2, 1] - r[1, 2]) / s, 0.25 * s,
             (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s]
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        q = [(r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s,
             0.25 * s, (r[1, 2] + r[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        q = [(r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s,
             (r[1, 2] + r[2, 1]) / s, 0.25 * s]
    return np.array(q)


def map_pose_to_px4(p_map, r_map_body):
    """Inverse of :func:`px4_pose_to_map` -- used by the simulated survey test."""
    pos_ned = S_NED_ENU @ np.asarray(p_map, dtype=float)
    return pos_ned, rot_to_quat(S_NED_ENU @ np.asarray(r_map_body) @ D_FLU_FRD)


def rpy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Rz(yaw) @ Ry(pitch) @ Rx(roll) -- the convention SDF <pose> uses."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def normalize_angle(angle: float) -> float:
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def ned_to_enu(vec) -> np.ndarray:
    return S_NED_ENU @ np.asarray(vec, dtype=float)


def yaw_ned_to_enu(yaw_ned: float) -> float:
    return normalize_angle(math.pi / 2.0 - yaw_ned)


def px4_pose_to_map(pos_ned, q_ned_frd):
    """PX4 local pose -> (position in ENU map frame, R_map_body for FLU body).

    v_map = S @ R_ned_frd @ D @ v_body   with D mapping FLU to FRD.
    """
    p_map = S_NED_ENU @ np.asarray(pos_ned, dtype=float)
    r_map_body = S_NED_ENU @ quat_to_rot(q_ned_frd) @ D_FLU_FRD
    return p_map, r_map_body


def yaw_from_rot(r: np.ndarray) -> float:
    """Yaw of an ENU/FLU rotation matrix (heading of body +X in the map XY plane)."""
    return math.atan2(r[1, 0], r[0, 0])


def camera_extrinsics(xyz_rpy):
    """SDF camera <pose> in the body frame -> (t_body_cam, R_body_optical).

    ``xyz_rpy`` is the six numbers straight out of the model SDF, e.g.
    ``[0.12, 0.03, 0.242, 0, 1.5708, 0]`` for the downward OakD-Lite.
    """
    x, y, z, roll, pitch, yaw = (float(v) for v in xyz_rpy)
    r_body_gzcam = rpy_to_rot(roll, pitch, yaw)
    return np.array([x, y, z]), r_body_gzcam @ R_GZCAM_OPT


def optical_to_map(points_opt: np.ndarray,
                   r_body_opt: np.ndarray,
                   t_body_cam: np.ndarray,
                   r_map_body: np.ndarray,
                   p_map_body: np.ndarray) -> np.ndarray:
    """Transform an (N, 3) array of optical-frame points into the map frame."""
    r = r_map_body @ r_body_opt
    t = r_map_body @ t_body_cam + p_map_body
    return points_opt @ r.T + t
