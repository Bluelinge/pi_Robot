from __future__ import annotations

import asyncio
import json
import time
from dataclasses import dataclass
from typing import Any

import serial

from pi_robot.models import TelemetryState


@dataclass(slots=True)
class SerialStatus:
    connected: bool = False
    last_message_at: float = 0.0
    last_error: str | None = None


class SerialBridge:
    def __init__(self, port: str, baudrate: int) -> None:
        self._port = port
        self._baudrate = baudrate
        self._serial: serial.Serial | None = None
        self._read_task: asyncio.Task[None] | None = None
        self._telemetry = TelemetryState()
        self._status = SerialStatus()
        self._acks: asyncio.Queue[dict[str, Any]] = asyncio.Queue()

    @property
    def telemetry(self) -> TelemetryState:
        return self._telemetry

    @property
    def status(self) -> SerialStatus:
        return self._status

    async def connect(self) -> None:
        self._serial = serial.Serial(self._port, self._baudrate, timeout=0.05)
        self._status.connected = True
        self._read_task = asyncio.create_task(self._read_loop())
        await self.send({"type": "hello"})

    async def disconnect(self) -> None:
        self._status.connected = False
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
            raise RuntimeError("serial bridge not connected")
        line = json.dumps(payload, separators=(",", ":")) + "\n"
        await asyncio.to_thread(self._serial.write, line.encode("utf-8"))

    async def _read_loop(self) -> None:
        assert self._serial is not None
        while self._status.connected:
            raw = await asyncio.to_thread(self._serial.readline)
            if not raw:
                continue
            self._status.last_message_at = time.monotonic()
            try:
                message = json.loads(raw.decode("utf-8").strip())
            except json.JSONDecodeError:
                self._status.last_error = "invalid-json"
                continue

            msg_type = message.get("type")
            if msg_type == "telemetry":
                self._telemetry = TelemetryState(
                    left_pwm=int(message.get("left_pwm", 0)),
                    right_pwm=int(message.get("right_pwm", 0)),
                    servo_angle=int(message.get("servo_angle", 90)),
                    estop=bool(message.get("estop", False)),
                    faults=int(message.get("faults", 0)),
                    last_cmd_age_ms=int(message.get("last_cmd_age_ms", 0)),
                    supply_state=str(message.get("supply_state", "unknown")),
                    uptime_ms=int(message.get("uptime_ms", 0)),
                    serial_ok=True,
                )
            elif msg_type == "ack":
                await self._acks.put(message)

    def is_timed_out(self, timeout_seconds: float = 0.5) -> bool:
        if not self._status.connected:
            return True
        if self._status.last_message_at == 0.0:
            return False
        return (time.monotonic() - self._status.last_message_at) > timeout_seconds
