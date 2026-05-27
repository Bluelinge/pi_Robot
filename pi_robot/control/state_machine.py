from __future__ import annotations

from pi_robot.models import RobotMode


ALLOWED_TRANSITIONS: dict[RobotMode, set[RobotMode]] = {
    RobotMode.IDLE: {RobotMode.MANUAL, RobotMode.AUTO, RobotMode.FOLLOW_TARGET, RobotMode.CALIBRATING, RobotMode.ESTOP},
    RobotMode.MANUAL: {RobotMode.IDLE, RobotMode.AUTO, RobotMode.FOLLOW_TARGET, RobotMode.CALIBRATING, RobotMode.ESTOP},
    RobotMode.AUTO: {RobotMode.IDLE, RobotMode.MANUAL, RobotMode.NAV_HOLD, RobotMode.ESTOP},
    RobotMode.FOLLOW_TARGET: {RobotMode.IDLE, RobotMode.MANUAL, RobotMode.NAV_HOLD, RobotMode.ESTOP},
    RobotMode.NAV_HOLD: {RobotMode.IDLE, RobotMode.MANUAL, RobotMode.AUTO, RobotMode.ESTOP},
    RobotMode.ESTOP: {RobotMode.IDLE, RobotMode.FAULT},
    RobotMode.FAULT: {RobotMode.IDLE, RobotMode.ESTOP},
    RobotMode.CALIBRATING: {RobotMode.IDLE, RobotMode.MANUAL, RobotMode.ESTOP},
}


def can_transition(current: RobotMode, next_mode: RobotMode) -> bool:
    return next_mode in ALLOWED_TRANSITIONS[current]
