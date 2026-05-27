from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(slots=True)
class GridMap:
    resolution_m_per_cell: float
    origin_xy_m: tuple[float, float]
    free_grid: np.ndarray
    blocked_grid: np.ndarray
    inflated_grid: np.ndarray
    boundary_points: list[list[float]]

    @property
    def width_cells(self) -> int:
        return int(self.blocked_grid.shape[1])

    @property
    def height_cells(self) -> int:
        return int(self.blocked_grid.shape[0])

    def metric_to_grid(self, x: float, y: float) -> tuple[int, int]:
        gx = int((x - self.origin_xy_m[0]) / self.resolution_m_per_cell)
        gy = int((y - self.origin_xy_m[1]) / self.resolution_m_per_cell)
        return gx, gy

    def grid_to_metric(self, gx: int, gy: int) -> tuple[float, float]:
        x = self.origin_xy_m[0] + gx * self.resolution_m_per_cell
        y = self.origin_xy_m[1] + gy * self.resolution_m_per_cell
        return x, y
