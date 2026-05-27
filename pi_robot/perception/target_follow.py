from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


@dataclass(slots=True)
class TrackerResult:
    found: bool
    bbox: tuple[int, int, int, int] | None
    center: tuple[int, int] | None
    template_size: tuple[int, int] | None


def create_tracker() -> Any:
    if hasattr(cv2, "TrackerCSRT_create"):
        return cv2.TrackerCSRT_create()
    if hasattr(getattr(cv2, "legacy", None), "TrackerCSRT_create"):
        return cv2.legacy.TrackerCSRT_create()
    if hasattr(cv2, "TrackerKCF_create"):
        return cv2.TrackerKCF_create()
    if hasattr(getattr(cv2, "legacy", None), "TrackerKCF_create"):
        return cv2.legacy.TrackerKCF_create()
    return None


def sanitize_bbox(
    x: int,
    y: int,
    width: int,
    height: int,
    image_width: int,
    image_height: int,
) -> tuple[int, int, int, int]:
    x = max(0, min(image_width - 2, x))
    y = max(0, min(image_height - 2, y))
    width = max(2, min(image_width - x, width))
    height = max(2, min(image_height - y, height))
    return x, y, width, height


def center_box(
    center_x: int,
    center_y: int,
    width: int,
    height: int,
    image_width: int,
    image_height: int,
) -> tuple[int, int, int, int]:
    x = int(center_x - width / 2)
    y = int(center_y - height / 2)
    return sanitize_bbox(x, y, width, height, image_width, image_height)


def initialize_tracker(
    tracker: Any,
    frame_bgr: np.ndarray,
    bbox: tuple[int, int, int, int],
) -> TrackerResult:
    if tracker is None:
        x, y, width, height = bbox
        center = (x + width // 2, y + height // 2)
        return TrackerResult(
            found=True,
            bbox=bbox,
            center=center,
            template_size=(width, height),
        )
    ok = tracker.init(frame_bgr, tuple(bbox))
    if not ok:
      return TrackerResult(found=False, bbox=None, center=None, template_size=None)
    x, y, width, height = bbox
    center = (x + width // 2, y + height // 2)
    return TrackerResult(
        found=True,
        bbox=bbox,
        center=center,
        template_size=(width, height),
    )


def update_tracker(tracker: Any, frame_bgr: np.ndarray) -> TrackerResult:
    if tracker is None:
        return TrackerResult(found=False, bbox=None, center=None, template_size=None)
    ok, bbox = tracker.update(frame_bgr)
    if not ok:
        return TrackerResult(found=False, bbox=None, center=None, template_size=None)
    x, y, width, height = (int(value) for value in bbox)
    center = (x + width // 2, y + height // 2)
    return TrackerResult(
        found=True,
        bbox=(x, y, width, height),
        center=center,
        template_size=(width, height),
    )


def extract_template(frame_bgr: np.ndarray, bbox: tuple[int, int, int, int]) -> np.ndarray:
    x, y, width, height = bbox
    roi = frame_bgr[y : y + height, x : x + width]
    return roi.copy()


def reacquire_template(
    frame_bgr: np.ndarray,
    template_bgr: np.ndarray,
    search_region: tuple[int, int, int, int] | None = None,
) -> TrackerResult:
    if template_bgr.size == 0:
        return TrackerResult(found=False, bbox=None, center=None, template_size=None)

    region = frame_bgr
    offset_x = 0
    offset_y = 0
    if search_region is not None:
        sx, sy, sw, sh = search_region
        region = frame_bgr[sy : sy + sh, sx : sx + sw]
        offset_x = sx
        offset_y = sy

    if region.shape[0] < template_bgr.shape[0] or region.shape[1] < template_bgr.shape[1]:
        return TrackerResult(found=False, bbox=None, center=None, template_size=None)

    result = cv2.matchTemplate(region, template_bgr, cv2.TM_CCOEFF_NORMED)
    _, score, _, top_left = cv2.minMaxLoc(result)
    if score < 0.55:
        return TrackerResult(found=False, bbox=None, center=None, template_size=None)

    x = top_left[0] + offset_x
    y = top_left[1] + offset_y
    width = template_bgr.shape[1]
    height = template_bgr.shape[0]
    center = (x + width // 2, y + height // 2)
    return TrackerResult(
        found=True,
        bbox=(x, y, width, height),
        center=center,
        template_size=(width, height),
    )


def depth_median_m(depth_mm: np.ndarray, bbox: tuple[int, int, int, int]) -> float | None:
    x, y, width, height = bbox
    margin_x = max(1, int(width * 0.35))
    margin_y = max(1, int(height * 0.35))
    inner = depth_mm[
        y + margin_y : y + height - margin_y,
        x + margin_x : x + width - margin_x,
    ]
    if inner.size == 0:
        return None
    values = inner.astype(np.float32) / 1000.0
    valid = values[(values > 0.08) & (values < 4.0)]
    if valid.size == 0:
        return None
    return float(np.median(valid))
