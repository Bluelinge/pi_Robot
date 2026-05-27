from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:  # pragma: no cover - handled at runtime on target device
    rs = None  # type: ignore[assignment]


@dataclass(slots=True)
class CameraFrame:
    color: np.ndarray
    depth: np.ndarray
    timestamp_ms: float
    imu: dict[str, Any] | None


class RealSenseCamera:
    def __init__(
        self,
        width: int,
        height: int,
        fps: int,
        require_imu: bool = False,
    ) -> None:
        self._width = width
        self._height = height
        self._fps = fps
        self._require_imu = require_imu
        self._pipeline = None
        self._align = None

    def start(self) -> None:
        if rs is None:
            raise RuntimeError("pyrealsense2 is not installed")
        self._pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, self._width, self._height, rs.format.bgr8, self._fps)
        config.enable_stream(rs.stream.depth, self._width, self._height, rs.format.z16, self._fps)
        if self._require_imu:
            config.enable_stream(rs.stream.accel)
            config.enable_stream(rs.stream.gyro)
        self._pipeline.start(config)
        self._align = rs.align(rs.stream.color)

    def stop(self) -> None:
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None
            self._align = None

    def get_frame(self) -> CameraFrame:
        if self._pipeline is None or self._align is None:
            raise RuntimeError("camera pipeline not started")

        frames = self._pipeline.wait_for_frames()
        aligned = self._align.process(frames)
        color = aligned.get_color_frame()
        depth = aligned.get_depth_frame()
        if not color or not depth:
            raise RuntimeError("missing aligned frame")

        color_np = np.asanyarray(color.get_data())
        depth_np = np.asanyarray(depth.get_data())
        return CameraFrame(
            color=color_np,
            depth=depth_np,
            timestamp_ms=float(color.get_timestamp()),
            imu=None,
        )
