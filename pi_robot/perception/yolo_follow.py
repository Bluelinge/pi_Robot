from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


@dataclass(slots=True)
class YoloDetection:
    class_id: int
    class_name: str
    confidence: float
    bbox: tuple[int, int, int, int]
    center: tuple[int, int]
    area: int


class YoloFollower:
    def __init__(self, model_path: str, confidence_threshold: float = 0.35) -> None:
        from ultralytics import YOLO

        self._model_path = model_path
        self._confidence_threshold = confidence_threshold
        self._model = YOLO(model_path)
        self._class_names = self._model.names

    def detect(self, frame_bgr: np.ndarray, target_class: str) -> list[YoloDetection]:
        results = self._model(frame_bgr, verbose=False)
        detections: list[YoloDetection] = []
        if not results:
            return detections

        boxes = results[0].boxes
        if boxes is None:
            return detections

        xyxy = boxes.xyxy.cpu().numpy() if hasattr(boxes.xyxy, "cpu") else boxes.xyxy.numpy()
        confs = boxes.conf.cpu().numpy() if hasattr(boxes.conf, "cpu") else boxes.conf.numpy()
        clss = boxes.cls.cpu().numpy() if hasattr(boxes.cls, "cpu") else boxes.cls.numpy()

        for bbox_values, conf, class_id_value in zip(xyxy, confs, clss):
            class_id = int(class_id_value)
            class_name = str(self._class_names[class_id])
            if class_name != target_class or float(conf) < self._confidence_threshold:
                continue
            x1, y1, x2, y2 = (int(value) for value in bbox_values)
            width = max(2, x2 - x1)
            height = max(2, y2 - y1)
            bbox = (x1, y1, width, height)
            center = (x1 + width // 2, y1 + height // 2)
            detections.append(
                YoloDetection(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=float(conf),
                    bbox=bbox,
                    center=center,
                    area=width * height,
                )
            )
        return detections

    @property
    def class_names(self) -> list[str]:
        return [str(self._class_names[index]) for index in sorted(self._class_names)]

    @staticmethod
    def choose_target(
        detections: list[YoloDetection],
        previous_center: tuple[float, float] | None,
    ) -> YoloDetection | None:
        if not detections:
            return None
        if previous_center is None:
            return max(detections, key=lambda det: (det.area, det.confidence))
        px, py = previous_center
        return min(
            detections,
            key=lambda det: (det.center[0] - px) ** 2 + (det.center[1] - py) ** 2,
        )
