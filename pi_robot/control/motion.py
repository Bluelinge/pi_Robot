from __future__ import annotations

from dataclasses import dataclass
from math import atan2, hypot

from pi_robot.models import NavigationGoal, Pose2D


@dataclass(slots=True)
class MotionLimits:
    max_linear_mps: float
    max_angular_rps: float


def normalize_motion(v: float, w: float, limits: MotionLimits) -> tuple[float, float]:
    linear = max(-limits.max_linear_mps, min(limits.max_linear_mps, v))
    angular = max(-limits.max_angular_rps, min(limits.max_angular_rps, w))
    return linear / limits.max_linear_mps, angular / limits.max_angular_rps


def pure_pursuit_command(
    pose: Pose2D,
    path: list[dict[str, float]],
    limits: MotionLimits,
    lookahead_distance: float = 0.08,
) -> tuple[float, float]:
    if not path:
        return 0.0, 0.0

    lookahead = path[-1]
    for point in path:
        if hypot(point["x"] - pose.x, point["y"] - pose.y) >= lookahead_distance:
            lookahead = point
            break

    dx = lookahead["x"] - pose.x
    dy = lookahead["y"] - pose.y
    heading = atan2(dy, dx)
    yaw_error = heading - pose.yaw
    while yaw_error > 3.14159:
        yaw_error -= 6.28318
    while yaw_error < -3.14159:
        yaw_error += 6.28318

    distance = hypot(dx, dy)
    linear = min(limits.max_linear_mps, max(0.0, distance))
    angular = max(-limits.max_angular_rps, min(limits.max_angular_rps, yaw_error * 1.5))
    return linear, angular


def goal_reached(pose: Pose2D, goal: NavigationGoal, tolerance_m: float = 0.08) -> bool:
    return hypot(goal.x - pose.x, goal.y - pose.y) <= tolerance_m
