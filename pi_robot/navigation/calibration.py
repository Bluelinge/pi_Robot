from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone

import numpy as np

from pi_robot.navigation.map_store import GridMap


@dataclass(slots=True)
class CalibrationSession:
    samples: list[tuple[float, float]] = field(default_factory=list)
    boundary_points: list[list[float]] = field(default_factory=list)
    active: bool = False

    def start(self) -> None:
        self.samples.clear()
        self.boundary_points.clear()
        self.active = True

    def add_pose(self, x: float, y: float) -> None:
        if not self.active:
            return
        self.samples.append((x, y))
        self.boundary_points.append([x, y])

    def stop(self) -> None:
        self.active = False

    def build_map(self) -> dict:
        if not self.samples:
            raise ValueError("no calibration samples recorded")

        xs = [point[0] for point in self.samples]
        ys = [point[1] for point in self.samples]
        min_x, max_x = min(xs) - 0.1, max(xs) + 0.1
        min_y, max_y = min(ys) - 0.1, max(ys) + 0.1
        resolution = 0.01
        width = max(40, int((max_x - min_x) / resolution))
        height = max(40, int((max_y - min_y) / resolution))
        free_grid = np.zeros((height, width), dtype=np.uint8)
        blocked_grid = np.zeros((height, width), dtype=np.uint8)
        blocked_grid[[0, -1], :] = 1
        blocked_grid[:, [0, -1]] = 1
        inflated_grid = np.array(blocked_grid, copy=True)

        return {
            "resolution_m_per_cell": resolution,
            "origin_xy_m": [min_x, min_y],
            "width_cells": width,
            "height_cells": height,
            "free_grid": free_grid.tolist(),
            "blocked_grid": blocked_grid.tolist(),
            "inflated_grid": inflated_grid.tolist(),
            "boundary_points": self.boundary_points,
            "last_calibrated_at": datetime.now(timezone.utc).isoformat(),
        }


def load_grid_map(payload: dict) -> GridMap:
    return GridMap(
        resolution_m_per_cell=float(payload["resolution_m_per_cell"]),
        origin_xy_m=(float(payload["origin_xy_m"][0]), float(payload["origin_xy_m"][1])),
        free_grid=np.array(payload["free_grid"], dtype=np.uint8),
        blocked_grid=np.array(payload["blocked_grid"], dtype=np.uint8),
        inflated_grid=np.array(payload["inflated_grid"], dtype=np.uint8),
        boundary_points=[list(point) for point in payload.get("boundary_points", [])],
    )
