from __future__ import annotations

import asyncio
import json
from contextlib import asynccontextmanager
from dataclasses import asdict, dataclass
from io import BytesIO
import os
from pathlib import Path
from typing import Any

from fastapi import FastAPI, Query, WebSocket
from fastapi.responses import HTMLResponse, StreamingResponse
from PIL import Image
from pydantic import BaseModel

from pi_robot.config import Settings

try:
    import pyrealsense2 as rs
except ImportError:  # pragma: no cover - handled at runtime on target device
    rs = None  # type: ignore[assignment]


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


class CameraViewerRuntime:
    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self._pipeline = None
        self._colorizer = None
        self.last_frame_jpeg: bytes = b""
        self.last_depth_jpeg: bytes = b""
        self.last_depth_raw: bytes = b""
        self.depth_scale_m: float = 0.001
        self.last_error: str | None = None
        self.camera_ok = False
        self.frame_width = settings.color_width
        self.frame_height = settings.color_height
        self.websocket_clients: set[WebSocket] = set()
        self.capture_task: asyncio.Task[None] | None = None
        self._restart_lock = asyncio.Lock()
        self._config = ViewerConfig(
            color_width=settings.color_width,
            color_height=settings.color_height,
            depth_width=settings.depth_width,
            depth_height=settings.depth_height,
            fps=settings.camera_fps,
        )

    async def start(self) -> None:
        await self._start_pipeline()
        self.capture_task = asyncio.create_task(self._capture_loop())

    async def stop(self) -> None:
        if self.capture_task:
            self.capture_task.cancel()
            try:
                await self.capture_task
            except asyncio.CancelledError:
                pass
        await self._stop_pipeline()

    async def _capture_loop(self) -> None:
        while True:
            try:
                if self._pipeline is None:
                    raise RuntimeError("camera pipeline not started")
                frames = await asyncio.to_thread(self._pipeline.wait_for_frames)
                color = frames.get_color_frame()
                depth = frames.get_depth_frame()
                if not color or not depth:
                    raise RuntimeError("missing color or depth frame")

                self.frame_width = color.get_width()
                self.frame_height = color.get_height()
                self.last_frame_jpeg = self._frame_to_jpeg(color)

                depth_width = depth.get_width()
                depth_height = depth.get_height()
                self.last_depth_raw = bytes(depth.get_data())
                if self._colorizer is None:
                    raise RuntimeError("depth colorizer is not initialized")
                depth_color = self._colorizer.colorize(depth)
                self.last_depth_jpeg = self._frame_to_jpeg(depth_color)
                self.frame_width = depth_width
                self.frame_height = depth_height
                self.camera_ok = True
                self.last_error = None
            except Exception as exc:
                self.camera_ok = False
                self.last_error = str(exc)
            await self._broadcast_state()
            await asyncio.sleep(0.01)

    async def _start_pipeline(self) -> None:
        if rs is None:
            raise RuntimeError("pyrealsense2 is not installed")
        self._pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(
            rs.stream.color,
            self._config.color_width,
            self._config.color_height,
            rs.format.bgr8,
            self._config.fps,
        )
        config.enable_stream(
            rs.stream.depth,
            self._config.depth_width,
            self._config.depth_height,
            rs.format.z16,
            self._config.fps,
        )
        profile = self._pipeline.start(config)
        self._colorizer = rs.colorizer()
        self.depth_scale_m = profile.get_device().first_depth_sensor().get_depth_scale()
        self.camera_ok = True
        self.last_error = None

    async def _stop_pipeline(self) -> None:
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None
        self._colorizer = None

    async def reconfigure(self, width: int, height: int, fps: int) -> dict[str, Any]:
        async with self._restart_lock:
            self._config = ViewerConfig(
                color_width=width,
                color_height=height,
                depth_width=width,
                depth_height=height,
                fps=fps,
            )
            await self._stop_pipeline()
            await self._start_pipeline()
        return self.state()

    async def _broadcast_state(self) -> None:
        if not self.websocket_clients:
            return
        payload = json.dumps({"type": "state", "payload": self.state()})
        stale: list[WebSocket] = []
        for socket in self.websocket_clients:
            try:
                await socket.send_text(payload)
            except Exception:
                stale.append(socket)
        for socket in stale:
            self.websocket_clients.discard(socket)

    def state(self) -> dict[str, Any]:
        return {
            "camera_ok": self.camera_ok,
            "last_error": self.last_error,
            "frame_width": self.frame_width,
            "frame_height": self.frame_height,
            "fps": self._config.fps,
            "depth_scale_m": self.depth_scale_m,
            "config": asdict(self._config),
            "options": DEFAULT_VIEWER_OPTIONS,
        }

    def depth_at(self, x: int, y: int) -> dict[str, Any]:
        if not self.last_depth_raw:
            return {"valid": False, "distance_m": None, "reason": "depth frame unavailable"}
        if x < 0 or y < 0 or x >= self.frame_width or y >= self.frame_height:
            return {"valid": False, "distance_m": None, "reason": "pixel out of range"}

        index = (y * self.frame_width + x) * 2
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

    def _frame_to_jpeg(self, frame: Any) -> bytes:
        if rs is None:
            raise RuntimeError("pyrealsense2 is not installed")

        width = frame.get_width()
        height = frame.get_height()
        fmt = frame.get_profile().format()
        raw = bytes(frame.get_data())

        if fmt == rs.format.bgr8:
            image = Image.frombuffer("RGB", (width, height), raw, "raw", "BGR", 0, 1)
        elif fmt == rs.format.rgb8:
            image = Image.frombuffer("RGB", (width, height), raw, "raw", "RGB", 0, 1)
        elif fmt == rs.format.bgra8:
            image = Image.frombuffer("RGBA", (width, height), raw, "raw", "BGRA", 0, 1).convert("RGB")
        elif fmt == rs.format.rgba8:
            image = Image.frombuffer("RGBA", (width, height), raw, "raw", "RGBA", 0, 1).convert("RGB")
        else:
            raise RuntimeError(f"unsupported frame format: {fmt}")

        buffer = BytesIO()
        image.save(buffer, format="JPEG", quality=80)
        return buffer.getvalue()


settings = Settings()
runtime = CameraViewerRuntime(settings)


@asynccontextmanager
async def lifespan(_: FastAPI):
    await runtime.start()
    yield
    await runtime.stop()


app = FastAPI(title="D435i Camera Viewer", lifespan=lifespan)
viewer_page = Path(__file__).resolve().parent / "web" / "camera_viewer.html"


@app.get("/", response_class=HTMLResponse)
async def index() -> str:
    return viewer_page.read_text(encoding="utf-8")


@app.get("/api/state")
async def get_state() -> dict[str, Any]:
    return runtime.state()


@app.post("/api/config")
async def post_config(request: ViewerConfigRequest) -> dict[str, Any]:
    return await runtime.reconfigure(request.width, request.height, request.fps)


@app.get("/api/depth_at")
async def get_depth_at(
    x: int = Query(..., ge=0),
    y: int = Query(..., ge=0),
) -> dict[str, Any]:
    return runtime.depth_at(x, y)


@app.websocket("/ws")
async def websocket_state(socket: WebSocket) -> None:
    await socket.accept()
    runtime.websocket_clients.add(socket)
    await socket.send_text(json.dumps({"type": "state", "payload": runtime.state()}))
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
            await asyncio.sleep(0.05)

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
            await asyncio.sleep(0.05)

    return StreamingResponse(
        generator(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


def main() -> None:
    import uvicorn

    uvicorn.run(
        app,
        host=settings.host,
        port=int(os.getenv("PI_ROBOT_CAMERA_VIEWER_PORT", "8001")),
        loop="asyncio",
    )


if __name__ == "__main__":
    main()
