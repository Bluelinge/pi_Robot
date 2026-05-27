from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path

from dotenv import load_dotenv


ROOT_DIR = Path(__file__).resolve().parents[1]
# 统一从项目根目录加载 .env，避免直接 `python -m` 启动时读不到配置。
load_dotenv(ROOT_DIR / ".env")


def _bool_env(name: str, default: bool) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _tuple_env(name: str, default: tuple[int, int, int]) -> tuple[int, int, int]:
    raw = os.getenv(name)
    if not raw:
        return default
    parts = [part.strip() for part in raw.split(",")]
    if len(parts) != 3:
        return default
    try:
        return tuple(int(part) for part in parts)  # type: ignore[return-value]
    except ValueError:
        return default


@dataclass(slots=True)
class Settings:
    # Web 服务监听地址与端口。
    host: str = os.getenv("PI_ROBOT_HOST", "0.0.0.0")
    port: int = int(os.getenv("PI_ROBOT_PORT", "8000"))
    # 当前主控默认走 UART 控制 ESP32；保留 HTTP 地址仅作为兼容配置。
    serial_port: str = os.getenv("PI_ROBOT_SERIAL_PORT", "/dev/serial0")
    serial_baud: int = int(os.getenv("PI_ROBOT_SERIAL_BAUD", "115200"))
    car0513_base_url: str = os.getenv("PI_ROBOT_CAR0513_BASE_URL", "http://192.168.26.101")
    # 预留给 YOLO 跟随能力的模型与阈值配置。
    yolo_model_path: str = os.getenv("PI_ROBOT_YOLO_MODEL_PATH", "./models/yolov8n.pt")
    yolo_target_class: str = os.getenv("PI_ROBOT_YOLO_TARGET_CLASS", "person")
    yolo_detect_interval_ms: int = int(os.getenv("PI_ROBOT_YOLO_DETECT_INTERVAL_MS", "180"))
    yolo_confidence_threshold: float = float(os.getenv("PI_ROBOT_YOLO_CONFIDENCE_THRESHOLD", "0.35"))
    # 数据目录、地图和电机参数文件都会相对项目根目录解析。
    data_dir: Path = (ROOT_DIR / os.getenv("PI_ROBOT_DATA_DIR", "data")).resolve()
    map_path: Path = (ROOT_DIR / os.getenv("PI_ROBOT_MAP_PATH", "data/desk_map.json")).resolve()
    motor_profile_path: Path = Path(
        (ROOT_DIR / os.getenv("PI_ROBOT_MOTOR_PROFILE_PATH", "data/motor_profile.json")).resolve()
    ).resolve()
    # D435i 默认采用已经验证过的 424x240 / 15fps 配置，优先保证稳定。
    color_width: int = int(os.getenv("PI_ROBOT_COLOR_WIDTH", "424"))
    color_height: int = int(os.getenv("PI_ROBOT_COLOR_HEIGHT", "240"))
    depth_width: int = int(os.getenv("PI_ROBOT_DEPTH_WIDTH", "424"))
    depth_height: int = int(os.getenv("PI_ROBOT_DEPTH_HEIGHT", "240"))
    camera_fps: int = int(os.getenv("PI_ROBOT_CAMERA_FPS", "15"))
    color_tape_low: tuple[int, int, int] = _tuple_env(
        "PI_ROBOT_COLOR_TAPE_LOW", (15, 100, 100)
    )
    color_tape_high: tuple[int, int, int] = _tuple_env(
        "PI_ROBOT_COLOR_TAPE_HIGH", (40, 255, 255)
    )
    require_imu: bool = _bool_env("PI_ROBOT_REALSENSE_REQUIRE_IMU", False)

    def ensure_dirs(self) -> None:
        # 首次启动时自动补齐运行目录，减少手工准备步骤。
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.map_path.parent.mkdir(parents=True, exist_ok=True)
        self.motor_profile_path.parent.mkdir(parents=True, exist_ok=True)


DEFAULT_MOTOR_PROFILE = {
    "max_linear_mps": 0.12,
    "max_angular_rps": 0.9,
    "max_pwm": 180,
    "ramp_step": 12,
    "left_min_pwm": 70,
    "right_min_pwm": 70,
    "left_gain": 1.0,
    "right_gain": 1.0,
    "track_width_m": 0.14,
    "default_servo_angle": 90,
}

DEFAULT_MAP = {
    "resolution_m_per_cell": 0.01,
    "origin_xy_m": [0.0, 0.0],
    "width_cells": 120,
    "height_cells": 80,
    "free_grid": [],
    "blocked_grid": [],
    "inflated_grid": [],
    "boundary_points": [],
    "tape_hsv_low": [15, 100, 100],
    "tape_hsv_high": [40, 255, 255],
    "last_calibrated_at": None,
}


def ensure_json_file(path: Path, payload: dict) -> None:
    if path.exists():
        return
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
