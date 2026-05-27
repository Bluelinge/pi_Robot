import numpy as np

from pi_robot.navigation.map_store import GridMap
from pi_robot.navigation.planner import astar_plan


def test_astar_plan_returns_path() -> None:
    grid = GridMap(
        resolution_m_per_cell=0.1,
        origin_xy_m=(0.0, 0.0),
        free_grid=np.zeros((10, 10), dtype=np.uint8),
        blocked_grid=np.zeros((10, 10), dtype=np.uint8),
        inflated_grid=np.zeros((10, 10), dtype=np.uint8),
        boundary_points=[],
    )
    path = astar_plan(grid, (0.1, 0.1), (0.6, 0.6))
    assert path
