from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any


class RobotMode(str, Enum):
    IDLE = "IDLE"
    MANUAL = "MANUAL"
    AUTO = "AUTO"
    FOLLOW_TARGET = "FOLLOW_TARGET"
    NAV_HOLD = "NAV_HOLD"
    ESTOP = "ESTOP"
    FAULT = "FAULT"
    CALIBRATING = "CALIBRATING"


@dataclass(slots=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    confidence: float = 0.0


@dataclass(slots=True)
class NavigationGoal:
    x: float
    y: float
    yaw: float | None = None
    label: str | None = None


@dataclass(slots=True)
class SafetyState:
    near_obstacle: bool = False
    edge_risk: bool = False
    tape_risk: bool = False
    camera_lost: bool = False
    vision_low_confidence: bool = False
    serial_timeout: bool = False
    firmware_fault: int = 0

    def any_risk(self) -> bool:
        return any(
            (
                self.near_obstacle,
                self.edge_risk,
                self.tape_risk,
                self.camera_lost,
                self.vision_low_confidence,
                self.serial_timeout,
                self.firmware_fault != 0,
            )
        )


@dataclass(slots=True)
class TelemetryState:
    left_pwm: int = 0
    right_pwm: int = 0
    servo_angle: int = 90
    estop: bool = False
    faults: int = 0
    last_cmd_age_ms: int = 0
    supply_state: str = "unknown"
    uptime_ms: int = 0
    serial_ok: bool = False


@dataclass(slots=True)
class Car0513State:
    connected: bool = False
    control_source: str = "none"
    speed_percent: int = 0
    motors: list[str] = field(default_factory=lambda: ["stop", "stop", "stop", "stop"])
    servo_angles: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    last_error: str = ""
    serial_port: str = ""


@dataclass(slots=True)
class TrackingState:
    active: bool = False
    locked: bool = False
    status: str = "idle"
    bbox: list[float] = field(default_factory=list)
    center: list[float] = field(default_factory=list)
    distance_m: float | None = None
    target_label: str = "custom"
    target_class: str = "person"
    confidence: float | None = None
    last_detected_at: float | None = None
    candidate_count: int = 0
    reacquire_attempts: int = 0
    blocked: bool = False
    initialized_at: float | None = None
    template_size: list[int] = field(default_factory=list)


@dataclass(slots=True)
class RobotState:
    mode: RobotMode = RobotMode.IDLE
    pose: Pose2D = field(default_factory=Pose2D)
    goal: NavigationGoal | None = None
    camera_ok: bool = False
    map_loaded: bool = False
    current_path: list[dict[str, float]] = field(default_factory=list)
    telemetry: TelemetryState = field(default_factory=TelemetryState)
    car0513: Car0513State = field(default_factory=Car0513State)
    tracking: TrackingState = field(default_factory=TrackingState)
    safety: SafetyState = field(default_factory=SafetyState)
    calibration_progress: float = 0.0
    last_error: str | None = None

    def as_dict(self) -> dict[str, Any]:
        return {
            "mode": self.mode.value,
            "pose": {
                "x": self.pose.x,
                "y": self.pose.y,
                "yaw": self.pose.yaw,
                "confidence": self.pose.confidence,
            },
            "goal": None
            if self.goal is None
            else {
                "x": self.goal.x,
                "y": self.goal.y,
                "yaw": self.goal.yaw,
                "label": self.goal.label,
            },
            "camera_ok": self.camera_ok,
            "serial_ok": self.telemetry.serial_ok,
            "safety": asdict(self.safety),
            "telemetry": asdict(self.telemetry),
            "car0513": asdict(self.car0513),
            "tracking": asdict(self.tracking),
            "map_loaded": self.map_loaded,
            "current_path": self.current_path,
            "calibration_progress": self.calibration_progress,
            "last_error": self.last_error,
        }
