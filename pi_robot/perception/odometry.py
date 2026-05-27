from __future__ import annotations

from dataclasses import dataclass
from math import atan2

import cv2
import numpy as np

from pi_robot.models import Pose2D


@dataclass(slots=True)
class VisualOdometryState:
    pose: Pose2D
    prev_gray: np.ndarray | None = None
    prev_depth: np.ndarray | None = None
    prev_keypoints: tuple[cv2.KeyPoint, ...] | None = None
    prev_descriptors: np.ndarray | None = None


class VisualOdometry:
    def __init__(self) -> None:
        self._orb = cv2.ORB_create(300)
        self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self._state = VisualOdometryState(pose=Pose2D(confidence=0.0))

    def update(self, color_bgr: np.ndarray, depth_mm: np.ndarray) -> Pose2D:
        gray = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = self._orb.detectAndCompute(gray, None)
        keypoints = tuple(keypoints or [])

        if descriptors is None or len(keypoints) < 8:
            self._state.pose.confidence = 0.0
            self._cache(gray, depth_mm, keypoints, descriptors)
            return self._state.pose

        if self._state.prev_descriptors is None or self._state.prev_keypoints is None:
            self._state.pose.confidence = 0.2
            self._cache(gray, depth_mm, keypoints, descriptors)
            return self._state.pose

        matches = self._matcher.match(self._state.prev_descriptors, descriptors)
        matches = sorted(matches, key=lambda match: match.distance)[:80]
        object_points: list[list[float]] = []
        image_points: list[list[float]] = []

        if self._state.prev_depth is None:
            self._cache(gray, depth_mm, keypoints, descriptors)
            return self._state.pose

        for match in matches:
            prev_pt = self._state.prev_keypoints[match.queryIdx].pt
            curr_pt = keypoints[match.trainIdx].pt
            z_mm = self._state.prev_depth[int(prev_pt[1]), int(prev_pt[0])]
            if z_mm <= 0:
                continue
            z = float(z_mm) / 1000.0
            x = (prev_pt[0] - gray.shape[1] / 2.0) * z / 320.0
            y = (prev_pt[1] - gray.shape[0] / 2.0) * z / 320.0
            object_points.append([x, y, z])
            image_points.append([curr_pt[0], curr_pt[1]])

        if len(object_points) < 6:
            self._state.pose.confidence = 0.0
            self._cache(gray, depth_mm, keypoints, descriptors)
            return self._state.pose

        camera_matrix = np.array(
            [[320.0, 0.0, gray.shape[1] / 2.0], [0.0, 320.0, gray.shape[0] / 2.0], [0.0, 0.0, 1.0]],
            dtype=np.float32,
        )
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            np.array(object_points, dtype=np.float32),
            np.array(image_points, dtype=np.float32),
            camera_matrix,
            None,
        )
        if not success:
            self._state.pose.confidence = 0.0
            self._cache(gray, depth_mm, keypoints, descriptors)
            return self._state.pose

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        yaw = float(atan2(rotation_matrix[1, 0], rotation_matrix[0, 0]))
        self._state.pose.x += float(tvec[0][0])
        self._state.pose.y += float(tvec[2][0])
        self._state.pose.yaw = yaw
        self._state.pose.confidence = min(1.0, (0 if inliers is None else len(inliers)) / 30.0)

        self._cache(gray, depth_mm, keypoints, descriptors)
        return self._state.pose

    def _cache(
        self,
        gray: np.ndarray,
        depth_mm: np.ndarray,
        keypoints: tuple[cv2.KeyPoint, ...],
        descriptors: np.ndarray | None,
    ) -> None:
        self._state.prev_gray = gray
        self._state.prev_depth = depth_mm
        self._state.prev_keypoints = keypoints
        self._state.prev_descriptors = descriptors
