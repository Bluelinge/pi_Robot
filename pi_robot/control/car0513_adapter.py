from __future__ import annotations

import asyncio
import json
import time
from dataclasses import dataclass, field
from typing import Any

import serial


@dataclass(slots=True)
class Car0513Telemetry:
    # 串口遥测会被树莓派状态页直接消费，因此这里尽量保持字段直观。
    control_source: str = "uart"
    speed_percent: int = 0
    motors: list[str] = field(default_factory=lambda: ["stop", "stop", "stop", "stop"])
    servo_angles: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    last_error: str = ""
    serial_ok: bool = False
    last_message_at: float = 0.0
    has_telemetry: bool = False


class Car0513Adapter:
    def __init__(self, port: str, baudrate: int) -> None:
        self._port = port
        self._baudrate = baudrate
        self._serial: serial.Serial | None = None
        self._read_task: asyncio.Task[None] | None = None
        self.telemetry = Car0513Telemetry()
        self._write_lock = asyncio.Lock()

    async def connect(self) -> None:
        self._serial = serial.Serial(self._port, self._baudrate, timeout=0.05)
        self.telemetry.serial_ok = True
        # ESP32 复位后需要一点时间重新拉起串口和网页逻辑。
        await asyncio.sleep(2.0)
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        self._read_task = asyncio.create_task(self._read_loop())
        await self.send({"type": "ping"})

    async def disconnect(self) -> None:
        self.telemetry.serial_ok = False
        if self._read_task:
            self._read_task.cancel()
            try:
                await self._read_task
            except asyncio.CancelledError:
                pass
        if self._serial:
            self._serial.close()
            self._serial = None

    async def send(self, payload: dict[str, Any]) -> None:
        if not self._serial:
            raise RuntimeError("uart adapter not connected")
        # 下位机协议统一采用 JSON Lines，方便树莓派和 ESP32 双向调试。
        line = json.dumps(payload, separators=(",", ":")) + "\n"
        async with self._write_lock:
            await asyncio.to_thread(self._serial.write, line.encode("utf-8"))

    async def drive(self, direction: str, speed: int | None = None) -> None:
        payload: dict[str, Any] = {"type": "drive", "dir": direction}
        if speed is not None:
            payload["speed"] = speed
        await self.send(payload)

    async def stop(self) -> None:
        await self.send({"type": "stop"})

    async def set_speed(self, speed_percent: int) -> None:
        await self.send({"type": "speed", "value": int(speed_percent)})

    async def set_servo(self, servo_id: int, direction: str) -> None:
        await self.send({"type": "servo", "id": int(servo_id), "dir": direction})

    async def set_servo_group(self, targets: list[int], direction: str) -> None:
        target_str = ",".join(str(item) for item in targets)
        await self.send({"type": "servo_group", "targets": target_str, "dir": direction})

    def is_timed_out(self, timeout_seconds: float = 1.0) -> bool:
        if not self.telemetry.serial_ok:
            return True
        if self.telemetry.last_message_at == 0.0:
            return True
        return (time.monotonic() - self.telemetry.last_message_at) > timeout_seconds

    async def _read_loop(self) -> None:
        assert self._serial is not None
        while self.telemetry.serial_ok:
            raw = await asyncio.to_thread(self._serial.readline)
            if not raw:
                continue
            self.telemetry.last_message_at = time.monotonic()
            try:
                message = json.loads(raw.decode("utf-8", errors="ignore").strip())
            except json.JSONDecodeError:
                # 串口偶发乱码时不直接抛异常，保留最后错误供状态页查看。
                self.telemetry.last_error = "invalid-json"
                continue

            msg_type = message.get("type")
            if msg_type == "telemetry":
                self.telemetry.control_source = str(message.get("control_source", "uart"))
                self.telemetry.speed_percent = int(message.get("speed_percent", 0))
                self.telemetry.motors = [str(item) for item in message.get("motors", ["stop"] * 4)]
                self.telemetry.servo_angles = [int(item) for item in message.get("servo_angles", [0, 0, 0, 0])]
                self.telemetry.last_error = str(message.get("last_error", ""))
                self.telemetry.serial_ok = True
                self.telemetry.has_telemetry = True
            elif msg_type == "error":
                self.telemetry.last_error = str(message.get("message", "unknown error"))
