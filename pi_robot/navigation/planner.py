from __future__ import annotations

import heapq
from math import hypot

import numpy as np

from pi_robot.navigation.map_store import GridMap


def _neighbors(x: int, y: int) -> list[tuple[int, int]]:
    return [
        (x - 1, y),
        (x + 1, y),
        (x, y - 1),
        (x, y + 1),
        (x - 1, y - 1),
        (x + 1, y - 1),
        (x - 1, y + 1),
        (x + 1, y + 1),
    ]


def build_dynamic_cost(base_map: GridMap, near_obstacle: bool) -> np.ndarray:
    dynamic = np.array(base_map.inflated_grid, copy=True)
    if near_obstacle:
        dynamic = np.maximum(dynamic, base_map.blocked_grid)
    return dynamic


def astar_plan(
    grid_map: GridMap,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    dynamic_cost: np.ndarray | None = None,
) -> list[dict[str, float]]:
    occupancy = dynamic_cost if dynamic_cost is not None else grid_map.inflated_grid
    start = grid_map.metric_to_grid(*start_xy)
    goal = grid_map.metric_to_grid(*goal_xy)
    frontier: list[tuple[float, tuple[int, int]]] = [(0.0, start)]
    came_from: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
    cost_so_far: dict[tuple[int, int], float] = {start: 0.0}

    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal:
            break
        for nxt in _neighbors(*current):
            x, y = nxt
            if x < 0 or y < 0 or x >= grid_map.width_cells or y >= grid_map.height_cells:
                continue
            if occupancy[y, x] > 0:
                continue
            movement_cost = hypot(x - current[0], y - current[1])
            new_cost = cost_so_far[current] + movement_cost
            if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                cost_so_far[nxt] = new_cost
                priority = new_cost + hypot(goal[0] - x, goal[1] - y)
                heapq.heappush(frontier, (priority, nxt))
                came_from[nxt] = current

    if goal not in came_from:
        return []

    path: list[tuple[int, int]] = []
    current = goal
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return [
        {"x": grid_map.grid_to_metric(px, py)[0], "y": grid_map.grid_to_metric(px, py)[1]}
        for px, py in path
    ]
