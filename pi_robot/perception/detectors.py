from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(slots=True)
class PerceptionResult:
    near_obstacle: bool
    edge_risk: bool
    tape_risk: bool
    obstacle_distance_m: float | None
    edge_score: float
    tape_ratio: float


def _depth_to_meters(depth_roi: np.ndarray) -> np.ndarray:
    return depth_roi.astype(np.float32) / 1000.0


def detect_perception(
    color_bgr: np.ndarray,
    depth_mm: np.ndarray,
    tape_low: tuple[int, int, int],
    tape_high: tuple[int, int, int],
) -> PerceptionResult:
    height, width = depth_mm.shape
    front_roi = depth_mm[int(height * 0.25) : int(height * 0.7), int(width * 0.25) : int(width * 0.75)]
    support_roi = depth_mm[int(height * 0.72) : int(height * 0.95), int(width * 0.2) : int(width * 0.8)]

    front_m = _depth_to_meters(front_roi)
    valid_front = front_m[(front_m > 0.05) & (front_m < 2.0)]
    obstacle_distance = float(valid_front.min()) if valid_front.size else None
    near_obstacle = obstacle_distance is not None and obstacle_distance < 0.20

    support_valid = _depth_to_meters(support_roi)
    support_ratio = float(np.mean((support_valid > 0.08) & (support_valid < 1.5)))
    edge_score = 1.0 - support_ratio
    edge_risk = edge_score > 0.55

    hsv = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(tape_low, dtype=np.uint8), np.array(tape_high, dtype=np.uint8))
    tape_roi = mask[int(height * 0.70) : int(height * 0.95), int(width * 0.1) : int(width * 0.9)]
    tape_ratio = float(np.mean(tape_roi > 0))
    tape_risk = tape_ratio > 0.12

    return PerceptionResult(
        near_obstacle=near_obstacle,
        edge_risk=edge_risk,
        tape_risk=tape_risk,
        obstacle_distance_m=obstacle_distance,
        edge_score=edge_score,
        tape_ratio=tape_ratio,
    )
