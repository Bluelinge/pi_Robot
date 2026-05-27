from __future__ import annotations

import asyncio
import json
import time
from contextlib import asynccontextmanager
from dataclasses import asdict, dataclass
from io import BytesIO
from pathlib import Path
from typing import Any

import numpy as np
from fastapi import FastAPI, HTTPException, Query, WebSocket
from fastapi.responses import HTMLResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from PIL import Image
from pydantic import BaseModel

from pi_robot.camera.realsense import RealSenseCamera
from pi_robot.config import DEFAULT_MAP, DEFAULT_MOTOR_PROFILE, Settings, ensure_json_file
from pi_robot.control.car0513_adapter import Car0513Adapter
from pi_robot.control.motion import MotionLimits, goal_reached, normalize_motion, pure_pursuit_command
from pi_robot.control.state_machine import can_transition
from pi_robot.models import NavigationGoal, RobotMode, RobotState
from pi_robot.navigation.calibration import CalibrationSession, load_grid_map
from pi_robot.navigation.map_store import GridMap
from pi_robot.navigation.planner import astar_plan, build_dynamic_cost
from pi_robot.perception.detectors import detect_perception
from pi_robot.perception.odometry import VisualOdometry
from pi_robot.perception.target_follow import (
    center_box,
    create_tracker,
    depth_median_m,
    extract_template,
    initialize_tracker,
    reacquire_template,
    sanitize_bbox,
    update_tracker,
)
from pi_robot.storage import load_json, save_json


class ModeRequest(BaseModel):
    mode: RobotMode


class ManualRequest(BaseModel):
    v: float
    w: float
    ttl_ms: int = 200


class GoalRequest(BaseModel):
    x: float
    y: float
    yaw: float | None = None
    label: str | None = None


class CalibrateRequest(BaseModel):
    action: str


class EstopRequest(BaseModel):
    active: bool


class DriveRequest(BaseModel):
    direction: str
    speed: int | None = None


class SpeedRequest(BaseModel):
    speed_percent: int


class ServoRequest(BaseModel):
    servo_id: int
    direction: str


class ServoGroupRequest(BaseModel):
    targets: list[int]
    direction: str


@dataclass(slots=True)
class ViewerConfig:
    color_width: int
    color_height: int
    depth_width: int
    depth_height: int
    fps: int


DEFAULT_VIEWER_OPTIONS = {
    "resolutions": [
        {"label": "424 x 240", "width": 424, "height": 240},
        {"label": "640 x 480", "width": 640, "height": 480},
        {"label": "848 x 480", "width": 848, "height": 480},
    ],
    "fps_values": [6, 15, 30],
}


class ViewerConfigRequest(BaseModel):
    width: int
    height: int
    fps: int


class FollowInitRequest(BaseModel):
    x: int
    y: int
    width: int
    height: int
    label: str | None = "custom"


FOLLOW_DISTANCE_M = 0.8
TOO_CLOSE_DISTANCE_M = 0.45
DISTANCE_DEADBAND_M = 0.12
TURN_HARD_THRESHOLD = 0.35
TURN_SOFT_THRESHOLD = 0.12
APPROACH_TURN_THRESHOLD = 0.18
FOLLOW_LOOP_INTERVAL_S = 0.08
FOLLOW_REACQUIRE_LIMIT = 10


class RobotRuntime:
    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self.state = RobotState()
        # 当前主链路是树莓派通过 UART 控制 ESP32，并从同一链路读取遥测状态。
        self.controller = Car0513Adapter(settings.serial_port, settings.serial_baud)
        self.camera = RealSenseCamera(
            settings.color_width,
            settings.color_height,
            settings.camera_fps,
            settings.require_imu,
        )
        self.odometry = VisualOdometry()
        self.calibration = CalibrationSession()
        self.map_payload = load_json(settings.map_path, DEFAULT_MAP)
        self.motor_profile = load_json(settings.motor_profile_path, DEFAULT_MOTOR_PROFILE)
        self.grid_map: GridMap | None = None
        self.last_frame_jpeg: bytes = b""
        self.last_depth_jpeg: bytes = b""
        self.last_depth_raw: bytes = b""
        self.depth_scale_m: float = 0.001
        self.websocket_clients: set[WebSocket] = set()
        self.background_tasks: list[asyncio.Task[Any]] = []
        self.stuck_started_at: float | None = None
        self.last_pose_for_stuck = (0.0, 0.0)
        self.nav_retry_count = 0
        self.viewer_config = ViewerConfig(
            color_width=settings.color_width,
            color_height=settings.color_height,
            depth_width=settings.depth_width,
            depth_height=settings.depth_height,
            fps=settings.camera_fps,
        )
        self.viewer_options = DEFAULT_VIEWER_OPTIONS
        self._viewer_lock = asyncio.Lock()
        self._follow_lock = asyncio.Lock()
        self._tracker: Any = None
        self._target_template: np.ndarray | None = None
        self._latest_color_bgr: np.ndarray | None = None
        self._latest_depth_mm: np.ndarray | None = None

        if self.map_payload.get("free_grid"):
            self.grid_map = load_grid_map(self.map_payload)
            self.state.map_loaded = True

    async def start(self) -> None:
        # 启动时顺手补齐运行目录和默认 JSON 文件，减少首次部署步骤。
        self.settings.ensure_dirs()
        ensure_json_file(self.settings.motor_profile_path, DEFAULT_MOTOR_PROFILE)
        ensure_json_file(self.settings.map_path, DEFAULT_MAP)
        await self.controller.connect()
        self.state.car0513.serial_port = self.settings.serial_port
        self.camera.start()
        self.state.camera_ok = True
        self.background_tasks = [
            asyncio.create_task(self._camera_loop()),
            asyncio.create_task(self._heartbeat_loop()),
            asyncio.create_task(self._follow_loop()),
        ]

    async def stop(self) -> None:
        for task in self.background_tasks:
            task.cancel()
        for task in self.background_tasks:
            try:
                await task
            except asyncio.CancelledError:
                pass
        await self.controller.disconnect()
        self.camera.stop()

    async def _heartbeat_loop(self) -> None:
        while True:
            await asyncio.sleep(0.1)
            try:
                # 轻量心跳用于保持串口链路活跃，并尽快发现 ESP32 失联。
                await self.controller.send({"type": "ping"})
            except Exception as exc:
                self.state.safety.serial_timeout = True
                self.state.last_error = str(exc)

    async def _camera_loop(self) -> None:
        while True:
            try:
                frame = await asyncio.to_thread(self.camera.get_frame)
            except Exception as exc:
                self.state.camera_ok = False
                self.state.safety.camera_lost = True
                self.state.last_error = str(exc)
                await self._stop_follow("camera_lost")
                await asyncio.sleep(0.1)
                continue

            self.state.camera_ok = True
            self.state.safety.camera_lost = False
            # 保留最新一帧彩色图与深度图，供网页推流、点击测距和跟随逻辑复用。
            self._latest_color_bgr = frame.color.copy()
            self._latest_depth_mm = frame.depth.copy()

            pose = self.odometry.update(frame.color, frame.depth)
            self.state.pose = pose
            self.state.safety.vision_low_confidence = pose.confidence < 0.15

            perception = detect_perception(
                frame.color,
                frame.depth,
                self.settings.color_tape_low,
                self.settings.color_tape_high,
            )
            self.state.safety.near_obstacle = perception.near_obstacle
            self.state.safety.edge_risk = perception.edge_risk
            self.state.safety.tape_risk = perception.tape_risk
            self.state.car0513.connected = (
                self.controller.telemetry.has_telemetry and not self.controller.is_timed_out()
            )
            self.state.car0513.control_source = self.controller.telemetry.control_source
            self.state.car0513.speed_percent = self.controller.telemetry.speed_percent
            self.state.car0513.motors = list(self.controller.telemetry.motors)
            self.state.car0513.servo_angles = list(self.controller.telemetry.servo_angles)
            self.state.car0513.last_error = self.controller.telemetry.last_error
            self.state.telemetry.serial_ok = not self.controller.is_timed_out()
            self.state.safety.serial_timeout = self.controller.is_timed_out()
            self.state.last_error = self.controller.telemetry.last_error or self.state.last_error

            if self.calibration.active:
                self.calibration.add_pose(pose.x, pose.y)
                self.state.calibration_progress = min(0.99, len(self.calibration.samples) / 500.0)

            await self._update_navigation()
            await self._broadcast_state()
            self.last_frame_jpeg = self._frame_to_jpeg(frame.color)
            self.last_depth_raw = frame.depth.tobytes()
            self.last_depth_jpeg = self._depth_to_jpeg(frame.depth)

    async def _update_navigation(self) -> None:
        if self.state.mode == RobotMode.FOLLOW_TARGET:
            return
        if self.state.safety.any_risk():
            self._force_nav_hold()
            return
        if self.state.mode not in {RobotMode.AUTO, RobotMode.MANUAL, RobotMode.CALIBRATING}:
            return
        if self.state.mode == RobotMode.MANUAL:
            return
        if self.state.mode == RobotMode.CALIBRATING:
            await self.controller.stop()
            return
        if self.state.goal is None or self.grid_map is None:
            await self.controller.stop()
            return
        if goal_reached(self.state.pose, self.state.goal):
            self.state.goal = None
            self.state.current_path = []
            self.state.mode = RobotMode.IDLE
            await self.controller.stop()
            return

        dynamic_cost = build_dynamic_cost(self.grid_map, self.state.safety.near_obstacle)
        path = astar_plan(
            self.grid_map,
            (self.state.pose.x, self.state.pose.y),
            (self.state.goal.x, self.state.goal.y),
            dynamic_cost=dynamic_cost,
        )
        if not path:
            self.nav_retry_count += 1
            if self.nav_retry_count > 1:
                self._force_nav_hold()
                return
            await self._recover_from_stuck()
            return

        self.state.current_path = path
        limits = MotionLimits(
            max_linear_mps=float(self.motor_profile["max_linear_mps"]),
            max_angular_rps=float(self.motor_profile["max_angular_rps"]),
        )
        linear, angular = pure_pursuit_command(self.state.pose, path, limits)
        normalized_v, normalized_w = normalize_motion(linear, angular, limits)
        direction = "forward"
        if normalized_v < -0.05:
            direction = "reverse"
        elif normalized_w > 0.1:
            direction = "left"
        elif normalized_w < -0.1:
            direction = "right"
        speed_percent = max(30, int(max(abs(normalized_v), abs(normalized_w)) * 100))
        await self.controller.drive(direction, speed_percent)
        self._update_stuck_watchdog(normalized_v)

    def _update_stuck_watchdog(self, normalized_v: float) -> None:
        current_pose = (self.state.pose.x, self.state.pose.y)
        distance = np.hypot(
            current_pose[0] - self.last_pose_for_stuck[0],
            current_pose[1] - self.last_pose_for_stuck[1],
        )
        if normalized_v > 0.1 and distance < 0.01:
            if self.stuck_started_at is None:
                self.stuck_started_at = time.monotonic()
            elif time.monotonic() - self.stuck_started_at > 2.0:
                self.nav_retry_count += 1
        else:
            self.stuck_started_at = None
            self.last_pose_for_stuck = current_pose

    async def _recover_from_stuck(self) -> None:
        await self.controller.drive("reverse", 35)
        await asyncio.sleep(0.2)
        await self.controller.drive("left", 30)
        await asyncio.sleep(0.2)
        if self.nav_retry_count > 1:
            self._force_nav_hold()

    def _force_nav_hold(self) -> None:
        if self.state.mode == RobotMode.ESTOP:
            return
        if self.state.safety.edge_risk or self.state.safety.tape_risk or self.state.safety.serial_timeout:
            self.state.mode = RobotMode.NAV_HOLD
        if self.state.safety.camera_lost:
            self.state.mode = RobotMode.FAULT

    async def _follow_loop(self) -> None:
        while True:
            await asyncio.sleep(FOLLOW_LOOP_INTERVAL_S)
            if self.state.mode != RobotMode.FOLLOW_TARGET:
                continue
            await self._run_follow_step()

    async def _run_follow_step(self) -> None:
        async with self._follow_lock:
            if self._latest_color_bgr is None or self._latest_depth_mm is None:
                return
            if self.state.safety.camera_lost:
                await self._stop_follow("camera_lost")
                return

            tracking = self.state.tracking
            color = self._latest_color_bgr
            depth = self._latest_depth_mm
            template = self._target_template
            if template is None:
                await self._stop_follow("missing_template")
                return

            result = None
            if tracking.locked and tracking.center and tracking.template_size:
                cx, cy = int(tracking.center[0]), int(tracking.center[1])
                tw, th = int(tracking.template_size[0]), int(tracking.template_size[1])
                search_region = center_box(cx, cy, tw * 3, th * 3, color.shape[1], color.shape[0])
                result = reacquire_template(color, template, search_region)

            if result is None or not result.found:
                tracking.status = "reacquiring"
                tracking.reacquire_attempts += 1
                result = reacquire_template(color, template, None)
                if not result.found:
                    await self.controller.stop()
                    if tracking.reacquire_attempts > FOLLOW_REACQUIRE_LIMIT:
                        await self._stop_follow("reacquire_failed")
                    return
                self._tracker = create_tracker()
                initialize_tracker(self._tracker, color, result.bbox)
                tracking.reacquire_attempts = 0

            if result.bbox is None or result.center is None:
                await self.controller.stop()
                return

            bbox = sanitize_bbox(*result.bbox, color.shape[1], color.shape[0])
            tracking.active = True
            tracking.locked = True
            tracking.status = "tracking"
            tracking.bbox = [float(v) for v in bbox]
            tracking.center = [float(result.center[0]), float(result.center[1])]
            tracking.template_size = [bbox[2], bbox[3]]
            tracking.distance_m = depth_median_m(depth, bbox)
            tracking.blocked = self.state.safety.near_obstacle

            if self.state.safety.near_obstacle:
                tracking.status = "blocked"
                await self.controller.stop()
                if result.center[0] < color.shape[1] * 0.45:
                    await self.controller.drive("left", 25)
                    await asyncio.sleep(0.15)
                elif result.center[0] > color.shape[1] * 0.55:
                    await self.controller.drive("right", 25)
                    await asyncio.sleep(0.15)
                return

            await self._drive_toward_target(color.shape[1], result.center[0], tracking.distance_m)

    async def _drive_toward_target(self, frame_width: int, center_x: int, distance_m: float | None) -> None:
        if distance_m is None:
            self.state.tracking.status = "no_depth"
            await self.controller.stop()
            return

        distance_error = distance_m - FOLLOW_DISTANCE_M
        center_error = (center_x - frame_width / 2.0) / (frame_width / 2.0)

        if distance_m < TOO_CLOSE_DISTANCE_M:
            self.state.tracking.status = "too_close"
            await self.controller.drive("reverse", 28)
            return

        if abs(center_error) > TURN_HARD_THRESHOLD:
            self.state.tracking.status = "turning"
            await self.controller.drive("left" if center_error < 0 else "right", 28)
            return

        if abs(distance_error) < DISTANCE_DEADBAND_M:
            if abs(center_error) > TURN_SOFT_THRESHOLD:
                self.state.tracking.status = "correcting"
                await self.controller.drive("left" if center_error < 0 else "right", 22)
            else:
                self.state.tracking.status = "holding"
                await self.controller.stop()
            return

        if distance_error > DISTANCE_DEADBAND_M:
            speed = 30 if distance_error < 0.35 else 40
            if abs(center_error) > APPROACH_TURN_THRESHOLD:
                self.state.tracking.status = "approach_turn"
                await self.controller.drive("left" if center_error < 0 else "right", 24)
            else:
                self.state.tracking.status = "approaching"
                await self.controller.drive("forward", speed)
            return

        self.state.tracking.status = "holding"
        await self.controller.stop()

    async def initialize_follow_target(
        self,
        x: int,
        y: int,
        width: int,
        height: int,
        label: str = "custom",
    ) -> dict[str, Any]:
        if self._latest_color_bgr is None or self._latest_depth_mm is None:
            raise ValueError("camera frame is unavailable")

        frame = self._latest_color_bgr
        bbox = sanitize_bbox(x, y, width, height, frame.shape[1], frame.shape[0])
        self._tracker = create_tracker()
        result = initialize_tracker(self._tracker, frame, bbox)
        if not result.found or result.bbox is None or result.center is None:
            raise ValueError("failed to initialize target")

        self._target_template = extract_template(frame, bbox)
        self.state.tracking.active = True
        self.state.tracking.locked = True
        self.state.tracking.status = "tracking"
        self.state.tracking.bbox = [float(v) for v in bbox]
        self.state.tracking.center = [float(result.center[0]), float(result.center[1])]
        self.state.tracking.distance_m = depth_median_m(self._latest_depth_mm, bbox)
        self.state.tracking.target_label = label
        self.state.tracking.reacquire_attempts = 0
        self.state.tracking.blocked = False
        self.state.tracking.initialized_at = time.time()
        self.state.tracking.template_size = [bbox[2], bbox[3]]
        self.state.mode = RobotMode.FOLLOW_TARGET
        return self.state_snapshot()

    async def stop_follow_target(self) -> dict[str, Any]:
        await self._stop_follow("stopped")
        return self.state_snapshot()

    async def _stop_follow(self, status: str) -> None:
        self._tracker = None
        self._target_template = None
        self.state.tracking.active = False
        self.state.tracking.locked = False
        self.state.tracking.status = status
        self.state.tracking.bbox = []
        self.state.tracking.center = []
        self.state.tracking.distance_m = None
        self.state.tracking.reacquire_attempts = 0
        self.state.tracking.blocked = False
        self.state.tracking.template_size = []
        if self.state.mode == RobotMode.FOLLOW_TARGET:
            self.state.mode = RobotMode.MANUAL
        await self.controller.stop()

    async def _broadcast_state(self) -> None:
        payload = json.dumps({"type": "state", "payload": self.state_snapshot()})
        stale: list[WebSocket] = []
        for socket in self.websocket_clients:
            try:
                await socket.send_text(payload)
            except Exception:
                stale.append(socket)
        for socket in stale:
            self.websocket_clients.discard(socket)

    async def set_mode(self, mode: RobotMode) -> None:
        if not can_transition(self.state.mode, mode):
            raise ValueError(f"illegal transition from {self.state.mode.value} to {mode.value}")
        if mode == RobotMode.ESTOP:
            await self.set_estop(True)
            return
        if self.state.mode == RobotMode.ESTOP and mode != RobotMode.IDLE:
            raise ValueError("must return to IDLE before leaving ESTOP")
        self.state.mode = mode
        if mode == RobotMode.IDLE:
            self.state.goal = None
            self.state.current_path = []
            await self.controller.stop()
        elif mode == RobotMode.CALIBRATING:
            self.calibration.start()
            self.state.calibration_progress = 0.0

    async def set_estop(self, active: bool) -> None:
        self.state.mode = RobotMode.ESTOP if active else RobotMode.IDLE
        self.state.telemetry.estop = active
        if active:
            await self.controller.stop()

    async def manual_drive(self, v: float, w: float, ttl_ms: int) -> None:
        if self.state.mode != RobotMode.MANUAL:
            raise ValueError("manual drive requires MANUAL mode")
        limits = MotionLimits(
            max_linear_mps=float(self.motor_profile["max_linear_mps"]),
            max_angular_rps=float(self.motor_profile["max_angular_rps"]),
        )
        normalized_v, normalized_w = normalize_motion(v, w, limits)
        direction = "stop"
        if normalized_v > 0.1:
            direction = "forward"
        elif normalized_v < -0.1:
            direction = "reverse"
        elif normalized_w > 0.1:
            direction = "left"
        elif normalized_w < -0.1:
            direction = "right"
        speed_percent = max(0, min(100, int(max(abs(normalized_v), abs(normalized_w)) * 100)))
        if direction == "stop":
            await self.controller.stop()
        else:
            await self.controller.drive(direction, speed_percent)

    async def set_goal(self, goal: NavigationGoal) -> None:
        if self.grid_map is None:
            raise ValueError("desk map is not loaded")
        self.state.goal = goal
        if self.state.mode == RobotMode.IDLE:
            self.state.mode = RobotMode.AUTO

    async def cancel_goal(self) -> None:
        self.state.goal = None
        self.state.current_path = []
        await self.controller.stop()
        self.state.mode = RobotMode.IDLE

    async def calibrate(self, action: str) -> None:
        if action == "start":
            await self.set_mode(RobotMode.CALIBRATING)
        elif action == "stop":
            self.calibration.stop()
            self.state.calibration_progress = 1.0 if self.calibration.samples else 0.0
        elif action == "save":
            payload = self.calibration.build_map()
            payload["tape_hsv_low"] = list(self.settings.color_tape_low)
            payload["tape_hsv_high"] = list(self.settings.color_tape_high)
            save_json(self.settings.map_path, payload)
            self.map_payload = payload
            self.grid_map = load_grid_map(payload)
            self.state.map_loaded = True
            self.state.mode = RobotMode.IDLE
        elif action == "discard":
            self.calibration = CalibrationSession()
            self.state.calibration_progress = 0.0
            self.state.mode = RobotMode.IDLE
        else:
            raise ValueError("invalid calibration action")

    async def drive_direction(self, direction: str, speed: int | None = None) -> dict[str, Any]:
        await self.controller.drive(direction, speed)
        return self.state_snapshot()

    async def stop_drive(self) -> dict[str, Any]:
        await self.controller.stop()
        return self.state_snapshot()

    async def set_speed_percent(self, speed_percent: int) -> dict[str, Any]:
        await self.controller.set_speed(speed_percent)
        return self.state_snapshot()

    async def set_servo_direction(self, servo_id: int, direction: str) -> dict[str, Any]:
        await self.controller.set_servo(servo_id, direction)
        return self.state_snapshot()

    async def set_servo_group_direction(self, targets: list[int], direction: str) -> dict[str, Any]:
        await self.controller.set_servo_group(targets, direction)
        return self.state_snapshot()

    async def reconfigure_viewer(self, width: int, height: int, fps: int) -> dict[str, Any]:
        async with self._viewer_lock:
            self.viewer_config = ViewerConfig(
                color_width=width,
                color_height=height,
                depth_width=width,
                depth_height=height,
                fps=fps,
            )
            self.camera.stop()
            self.camera = RealSenseCamera(width, height, fps, self.settings.require_imu)
            self.camera.start()
        return self.state_snapshot()

    def depth_at(self, x: int, y: int) -> dict[str, Any]:
        width = self.viewer_config.depth_width
        height = self.viewer_config.depth_height
        if not self.last_depth_raw:
            return {"valid": False, "distance_m": None, "reason": "depth frame unavailable"}
        if x < 0 or y < 0 or x >= width or y >= height:
            return {"valid": False, "distance_m": None, "reason": "pixel out of range"}
        index = (y * width + x) * 2
        if index + 1 >= len(self.last_depth_raw):
            return {"valid": False, "distance_m": None, "reason": "pixel out of buffer"}
        depth_units = int.from_bytes(self.last_depth_raw[index:index + 2], byteorder="little")
        distance_m = depth_units * self.depth_scale_m if depth_units > 0 else 0.0
        return {
            "valid": depth_units > 0,
            "distance_m": distance_m if depth_units > 0 else None,
            "depth_units": depth_units,
            "x": x,
            "y": y,
        }

    def state_snapshot(self) -> dict[str, Any]:
        snapshot = self.state.as_dict()
        # 额外把当前画面配置打包给前端，便于网页切换分辨率和帧率。
        snapshot["viewer"] = {
            "frame_width": self.viewer_config.color_width,
            "frame_height": self.viewer_config.color_height,
            "fps": self.viewer_config.fps,
            "depth_scale_m": self.depth_scale_m,
            "config": asdict(self.viewer_config),
            "options": self.viewer_options,
        }
        return snapshot

    def _frame_to_jpeg(self, color_bgr: Any) -> bytes:
        height, width = color_bgr.shape[:2]
        image = Image.frombuffer(
            "RGB",
            (width, height),
            bytes(color_bgr.data),
            "raw",
            "BGR",
            0,
            1,
        )
        buffer = BytesIO()
        image.save(buffer, format="JPEG", quality=80)
        return buffer.getvalue()

    def _depth_to_jpeg(self, depth_mm: Any) -> bytes:
        depth_clip = depth_mm.astype(np.float32)
        max_depth = float(np.max(depth_clip)) if np.max(depth_clip) > 0 else 1.0
        normalized = np.clip(depth_clip / max_depth, 0.0, 1.0)
        pseudo = np.stack(
            [
                (normalized * 255).astype(np.uint8),
                ((1.0 - normalized) * 255).astype(np.uint8),
                np.full_like(normalized, 96, dtype=np.uint8),
            ],
            axis=-1,
        )
        buffer = BytesIO()
        Image.fromarray(pseudo, mode="RGB").save(buffer, format="JPEG", quality=80)
        return buffer.getvalue()


settings = Settings()
runtime = RobotRuntime(settings)


@asynccontextmanager
async def lifespan(_: FastAPI):
    await runtime.start()
    yield
    await runtime.stop()


app = FastAPI(lifespan=lifespan)
web_dir = Path(__file__).resolve().parent / "web"
app.mount("/static", StaticFiles(directory=web_dir / "static"), name="static")


@app.get("/", response_class=HTMLResponse)
async def index() -> str:
    return (web_dir / "index.html").read_text(encoding="utf-8")


@app.get("/api/state")
async def get_state() -> dict[str, Any]:
    return runtime.state_snapshot()


@app.post("/api/mode")
async def post_mode(request: ModeRequest) -> dict[str, Any]:
    try:
        await runtime.set_mode(request.mode)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return runtime.state_snapshot()


@app.post("/api/manual")
async def post_manual(request: ManualRequest) -> dict[str, Any]:
    try:
        await runtime.manual_drive(request.v, request.w, request.ttl_ms)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return {"ok": True}


@app.post("/api/drive")
async def post_drive(request: DriveRequest) -> dict[str, Any]:
    return await runtime.drive_direction(request.direction, request.speed)


@app.post("/api/stop")
async def post_stop() -> dict[str, Any]:
    return await runtime.stop_drive()


@app.post("/api/speed")
async def post_speed(request: SpeedRequest) -> dict[str, Any]:
    return await runtime.set_speed_percent(request.speed_percent)


@app.post("/api/servo")
async def post_servo(request: ServoRequest) -> dict[str, Any]:
    return await runtime.set_servo_direction(request.servo_id, request.direction)


@app.post("/api/servo-group")
async def post_servo_group(request: ServoGroupRequest) -> dict[str, Any]:
    return await runtime.set_servo_group_direction(request.targets, request.direction)


@app.post("/api/viewer/config")
async def post_viewer_config(request: ViewerConfigRequest) -> dict[str, Any]:
    return await runtime.reconfigure_viewer(request.width, request.height, request.fps)


@app.post("/api/follow/init")
async def post_follow_init(request: FollowInitRequest) -> dict[str, Any]:
    try:
        return await runtime.initialize_follow_target(
            request.x,
            request.y,
            request.width,
            request.height,
            request.label or "custom",
        )
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@app.post("/api/follow/stop")
async def post_follow_stop() -> dict[str, Any]:
    return await runtime.stop_follow_target()


@app.get("/api/viewer/depth_at")
async def get_viewer_depth_at(
    x: int = Query(..., ge=0),
    y: int = Query(..., ge=0),
) -> dict[str, Any]:
    return runtime.depth_at(x, y)


@app.post("/api/nav/goal")
async def post_goal(request: GoalRequest) -> dict[str, Any]:
    try:
        await runtime.set_goal(NavigationGoal(**request.model_dump()))
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return runtime.state_snapshot()


@app.post("/api/nav/cancel")
async def post_cancel() -> dict[str, Any]:
    await runtime.cancel_goal()
    return runtime.state_snapshot()


@app.post("/api/calibrate/desk")
async def post_calibrate(request: CalibrateRequest) -> dict[str, Any]:
    try:
        await runtime.calibrate(request.action)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return runtime.state_snapshot()


@app.post("/api/estop")
async def post_estop(request: EstopRequest) -> dict[str, Any]:
    await runtime.set_estop(request.active)
    return runtime.state_snapshot()


@app.websocket("/ws")
async def websocket_state(socket: WebSocket) -> None:
    await socket.accept()
    runtime.websocket_clients.add(socket)
    await socket.send_text(json.dumps({"type": "state", "payload": runtime.state_snapshot()}))
    try:
        while True:
            await socket.receive_text()
    except Exception:
        runtime.websocket_clients.discard(socket)


@app.get("/stream.mjpg")
async def stream_mjpeg() -> StreamingResponse:
    async def generator():
        while True:
            if runtime.last_frame_jpeg:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + runtime.last_frame_jpeg
                    + b"\r\n"
                )
            await asyncio.sleep(0.1)

    return StreamingResponse(
        generator(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/stream_depth.mjpg")
async def stream_depth_mjpeg() -> StreamingResponse:
    async def generator():
        while True:
            if runtime.last_depth_jpeg:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + runtime.last_depth_jpeg
                    + b"\r\n"
                )
            await asyncio.sleep(0.1)

    return StreamingResponse(
        generator(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )
