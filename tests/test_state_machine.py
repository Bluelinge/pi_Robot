from pi_robot.control.state_machine import can_transition
from pi_robot.models import RobotMode


def test_valid_transition() -> None:
    assert can_transition(RobotMode.IDLE, RobotMode.MANUAL)


def test_invalid_transition() -> None:
    assert not can_transition(RobotMode.CALIBRATING, RobotMode.AUTO)
