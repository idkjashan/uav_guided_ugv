"""ArUco marker on the UGV roof -> UGV pose in the map frame.

The chain is the same one ``road_survey`` uses for depth points:

    marker corners (px) --solvePnP--> marker pose in the camera optical frame
    --camera extrinsics--> UAV body --PX4 pose--> map (PX4 local ENU)

so the UGV pose lands in exactly the frame the costmap is in. No depth is
needed: the marker's known size fixes the range, and when the marker is near
the image centre a range error moves the estimate up/down, not sideways.
"""

from __future__ import annotations

import math

import cv2
import numpy as np

from road_survey.frames import optical_to_map


class MarkerDetector:
    """Finds one marker id and returns its four corners, or None."""

    def __init__(self, dictionary_id=cv2.aruco.DICT_4X4_50, marker_id=0):
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(dictionary_id), params)
        self.marker_id = int(marker_id)

    def __call__(self, gray):
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return None
        for c, i in zip(corners, ids.ravel()):
            if int(i) == self.marker_id:
                return c.reshape(4, 2).astype(np.float64)  # TL, TR, BR, BL
        return None


def marker_in_camera(corners, k, length_m, dist=None):
    """-> (R_opt_marker, t_opt, reprojection error in px), or None.

    ``length_m`` is the side of the *black* square, not of the white plate it
    is printed on. Marker frame: +x from the top-left to the top-right corner,
    +y towards the top edge, +z out of the marker face.
    """
    h = 0.5 * float(length_m)
    obj = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])
    dist = np.zeros(5) if dist is None else np.asarray(dist, dtype=float)
    ok, rvec, tvec = cv2.solvePnP(obj, corners, k, dist,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    proj, _ = cv2.projectPoints(obj, rvec, tvec, k, dist)
    err = float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - corners) ** 2, axis=1))))
    return cv2.Rodrigues(rvec)[0], tvec.ravel(), err


def ugv_pose_in_map(r_opt_marker, t_opt, r_body_opt, t_body_cam, r_map_body,
                    p_map_body, yaw_offset=0.0, marker_height_m=0.0):
    """Marker pose in the camera -> UGV (x, y, z, yaw) in the map frame.

    ``yaw_offset`` is the angle from the marker's +x axis to the UGV's
    forward axis. ``marker_height_m`` is how far the marker face sits above
    the UGV's base_link. Yaw comes from the marker's +x axis projected onto
    the ground, which is unaffected by the tilt flip IPPE can pick between.
    """
    p = optical_to_map(np.asarray(t_opt, dtype=float).reshape(1, 3),
                       r_body_opt, t_body_cam, r_map_body, p_map_body)[0]
    r_map_marker = r_map_body @ r_body_opt @ r_opt_marker
    yaw = math.atan2(r_map_marker[1, 0], r_map_marker[0, 0]) + yaw_offset
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    return float(p[0]), float(p[1]), float(p[2] - marker_height_m), yaw


def image_to_gray(msg):
    """sensor_msgs/Image -> uint8 (H, W) grey, without cv_bridge."""
    enc = msg.encoding.lower()
    ch = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(enc)
    if ch is None:
        raise ValueError(f'unsupported image encoding: {msg.encoding!r}')
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    img = buf[:, :msg.width * ch]
    if ch == 1:
        return img
    img = img.reshape(msg.height, msg.width, ch)
    code = {'rgb8': cv2.COLOR_RGB2GRAY, 'bgr8': cv2.COLOR_BGR2GRAY,
            'rgba8': cv2.COLOR_RGBA2GRAY, 'bgra8': cv2.COLOR_BGRA2GRAY}[enc]
    return cv2.cvtColor(img, code)
