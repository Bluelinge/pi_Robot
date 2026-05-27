const state = { current: null };

const modeBadge = document.getElementById("modeBadge");
const cameraBadge = document.getElementById("cameraBadge");
const espBadge = document.getElementById("espBadge");
const cameraStatus = document.getElementById("cameraStatus");
const espStatus = document.getElementById("espStatus");
const serialPort = document.getElementById("serialPort");
const resolutionText = document.getElementById("resolutionText");
const fpsText = document.getElementById("fpsText");
const speedText = document.getElementById("speedText");
const controlSource = document.getElementById("controlSource");
const depthScale = document.getElementById("depthScale");
const depthAt = document.getElementById("depthAt");
const lastError = document.getElementById("lastError");
const trackingStatus = document.getElementById("trackingStatus");
const trackingDistance = document.getElementById("trackingDistance");
const telemetryBox = document.getElementById("telemetryBox");
const stream = document.getElementById("stream");
const colorStream = document.getElementById("colorStream");
const depthStream = document.getElementById("depthStream");
const singleView = document.getElementById("singleView");
const splitView = document.getElementById("splitView");
const resolutionSelect = document.getElementById("resolutionSelect");
const fpsSelect = document.getElementById("fpsSelect");
const applyConfigBtn = document.getElementById("applyConfigBtn");
const speedSlider = document.getElementById("speedSlider");
const speedSliderValue = document.getElementById("speedSliderValue");
const servoIdSelect = document.getElementById("servoIdSelect");
const overlayCanvas = document.getElementById("overlayCanvas");
const overlayCtx = overlayCanvas.getContext("2d");
const startFollowBtn = document.getElementById("startFollowBtn");
const stopFollowBtn = document.getElementById("stopFollowBtn");
const followHint = document.getElementById("followHint");

let currentView = "color";
let viewerConfigDirty = false;
let viewerApplyInFlight = false;
let driveHoldTimer = null;
let activeDriveDirection = "stop";
let activeSteerDirection = "stop";
let servoHoldTimer = null;
let servoGroupHoldTimer = null;
let activeServoDirection = "stop";
let activeServoGroupDirection = "stop";
let dragStart = null;
let pendingSelection = null;

async function postJson(path, body) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body ?? {}),
  });
  if (!response.ok) {
    throw new Error(await response.text());
  }
  return response.json().catch(() => ({}));
}

function setView(mode) {
  currentView = mode;
  const colorSrc = "/stream.mjpg";
  const depthSrc = "/stream_depth.mjpg";
  singleView.hidden = mode === "split";
  splitView.hidden = mode !== "split";
  stream.src = mode === "depth" ? depthSrc : colorSrc;
  colorStream.src = colorSrc;
  depthStream.src = depthSrc;
  document.querySelectorAll("[data-view]").forEach((button) => {
    button.classList.toggle("active", button.dataset.view === mode);
  });
}

function renderViewerSelectors(snapshot) {
  if (!snapshot.options) return;
  if (!resolutionSelect.options.length) {
    snapshot.options.resolutions.forEach((item) => {
      const option = document.createElement("option");
      option.value = `${item.width}x${item.height}`;
      option.textContent = item.label;
      resolutionSelect.appendChild(option);
    });
  }
  if (!fpsSelect.options.length) {
    snapshot.options.fps_values.forEach((value) => {
      const option = document.createElement("option");
      option.value = `${value}`;
      option.textContent = `${value} fps`;
      fpsSelect.appendChild(option);
    });
  }
  if (viewerConfigDirty || viewerApplyInFlight) return;
  resolutionSelect.value = `${snapshot.viewer.config.color_width}x${snapshot.viewer.config.color_height}`;
  fpsSelect.value = `${snapshot.viewer.config.fps}`;
}

function renderState(snapshot) {
  state.current = snapshot;
  modeBadge.textContent = snapshot.mode;

  const cameraOk = !!snapshot.camera_ok;
  cameraBadge.textContent = cameraOk ? "Camera: online" : "Camera: offline";
  cameraStatus.textContent = cameraOk ? "online" : "offline";

  const espOk = !!snapshot.car0513?.connected;
  espBadge.textContent = espOk ? "ESP32: online" : "ESP32: offline";
  espStatus.textContent = espOk ? "online" : "offline";

  serialPort.textContent = snapshot.car0513?.serial_port || "-";
  resolutionText.textContent = `${snapshot.viewer.frame_width} x ${snapshot.viewer.frame_height}`;
  fpsText.textContent = `${snapshot.viewer.fps} fps`;
  speedText.textContent = `${snapshot.car0513?.speed_percent ?? 0}%`;
  controlSource.textContent = snapshot.car0513?.control_source || "none";
  depthScale.textContent = `${snapshot.viewer.depth_scale_m?.toFixed?.(6) ?? snapshot.viewer.depth_scale_m} m/unit`;
  lastError.textContent = snapshot.car0513?.last_error || snapshot.last_error || "none";
  telemetryBox.textContent = JSON.stringify(snapshot.car0513, null, 2);
  trackingStatus.textContent = snapshot.tracking?.status || "idle";
  trackingDistance.textContent = snapshot.tracking?.distance_m != null
    ? `${snapshot.tracking.distance_m.toFixed(3)} m`
    : "-";

  speedSlider.value = `${snapshot.car0513?.speed_percent ?? 0}`;
  speedSliderValue.textContent = `${snapshot.car0513?.speed_percent ?? 0}%`;
  renderViewerSelectors(snapshot);
  drawTrackingOverlay(snapshot);
}

async function refreshState() {
  const response = await fetch("/api/state");
  renderState(await response.json());
}

async function applyViewerConfig() {
  viewerApplyInFlight = true;
  applyConfigBtn.disabled = true;
  applyConfigBtn.textContent = "应用中...";
  const [width, height] = resolutionSelect.value.split("x").map(Number);
  const fps = Number(fpsSelect.value);
  try {
    const snapshot = await postJson("/api/viewer/config", { width, height, fps });
    viewerConfigDirty = false;
    renderState(snapshot);
    setView(currentView);
  } finally {
    viewerApplyInFlight = false;
    applyConfigBtn.disabled = false;
    applyConfigBtn.textContent = "应用画面参数";
  }
}

async function queryDepthAt(event) {
  if (pendingSelection || dragStart) {
    return;
  }
  const target = event.currentTarget;
  const rect = target.getBoundingClientRect();
  const naturalWidth = target.naturalWidth || target.clientWidth;
  const naturalHeight = target.naturalHeight || target.clientHeight;
  if (!naturalWidth || !naturalHeight) return;
  const scaleX = naturalWidth / rect.width;
  const scaleY = naturalHeight / rect.height;
  const x = Math.max(0, Math.min(naturalWidth - 1, Math.floor((event.clientX - rect.left) * scaleX)));
  const y = Math.max(0, Math.min(naturalHeight - 1, Math.floor((event.clientY - rect.top) * scaleY)));
  const response = await fetch(`/api/viewer/depth_at?x=${x}&y=${y}`);
  const payload = await response.json();
  depthAt.textContent = payload.valid
    ? `${payload.distance_m.toFixed(3)} m @ (${payload.x}, ${payload.y})`
    : payload.reason || "invalid";
}

function resizeOverlay() {
  const rect = stream.getBoundingClientRect();
  overlayCanvas.width = Math.max(1, Math.floor(rect.width));
  overlayCanvas.height = Math.max(1, Math.floor(rect.height));
}

function drawTrackingOverlay(snapshot) {
  resizeOverlay();
  overlayCtx.clearRect(0, 0, overlayCanvas.width, overlayCanvas.height);

  const bbox = snapshot.tracking?.bbox || [];
  if (bbox.length === 4 && stream.naturalWidth && stream.naturalHeight) {
    const scaleX = overlayCanvas.width / stream.naturalWidth;
    const scaleY = overlayCanvas.height / stream.naturalHeight;
    const [x, y, width, height] = bbox;
    overlayCtx.strokeStyle = "#ffd166";
    overlayCtx.lineWidth = 3;
    overlayCtx.strokeRect(x * scaleX, y * scaleY, width * scaleX, height * scaleY);
    const center = snapshot.tracking?.center || [];
    if (center.length === 2) {
      const cx = center[0] * scaleX;
      const cy = center[1] * scaleY;
      overlayCtx.strokeStyle = "#06d6a0";
      overlayCtx.beginPath();
      overlayCtx.moveTo(cx - 10, cy);
      overlayCtx.lineTo(cx + 10, cy);
      overlayCtx.moveTo(cx, cy - 10);
      overlayCtx.lineTo(cx, cy + 10);
      overlayCtx.stroke();
    }
  }

  if (pendingSelection && stream.naturalWidth && stream.naturalHeight) {
    const scaleX = overlayCanvas.width / stream.naturalWidth;
    const scaleY = overlayCanvas.height / stream.naturalHeight;
    overlayCtx.strokeStyle = "#ef476f";
    overlayCtx.lineWidth = 2;
    overlayCtx.strokeRect(
      pendingSelection.x * scaleX,
      pendingSelection.y * scaleY,
      pendingSelection.width * scaleX,
      pendingSelection.height * scaleY,
    );
  }
}

function connectWebSocket() {
  const protocol = location.protocol === "https:" ? "wss" : "ws";
  const socket = new WebSocket(`${protocol}://${location.host}/ws`);
  socket.onmessage = (event) => {
    const message = JSON.parse(event.data);
    if (message.type === "state") {
      renderState(message.payload);
    }
  };
  socket.onclose = () => setTimeout(connectWebSocket, 1000);
}

document.querySelectorAll("[data-view]").forEach((button) => {
  button.addEventListener("click", () => setView(button.dataset.view));
});

document.querySelectorAll("[data-mode]").forEach((button) => {
  button.addEventListener("click", () => postJson("/api/mode", { mode: button.dataset.mode }).then(renderState));
});

async function sendDrive(direction) {
  if (direction === "stop") {
    return postJson("/api/stop", {}).then(renderState);
  }
  return postJson("/api/drive", {
    direction,
    speed: Number(speedSlider.value),
  }).then(renderState);
}

async function sendSteer(direction) {
  return postJson("/api/servo-group", {
    targets: [1, 2],
    direction,
  }).then(renderState);
}

function stopDriveHold() {
  if (driveHoldTimer) {
    clearInterval(driveHoldTimer);
    driveHoldTimer = null;
  }
  const shouldStopDrive = activeDriveDirection !== "stop";
  const shouldCenterSteer = activeSteerDirection !== "stop";
  activeDriveDirection = "stop";
  activeSteerDirection = "stop";
  if (shouldStopDrive) {
    postJson("/api/stop", {}).then(renderState).catch(console.error);
  }
  if (shouldCenterSteer) {
    sendSteer("center").catch(console.error);
  }
}

function startDriveHold(direction) {
  if (direction === "stop") {
    stopDriveHold();
    postJson("/api/stop", {}).then(renderState).catch(console.error);
    return;
  }
  const isSteeringDirection = direction === "left" || direction === "right";
  if (activeDriveDirection === direction) {
    return;
  }
  stopDriveHold();
  activeDriveDirection = direction;
  activeSteerDirection = isSteeringDirection ? direction : "stop";
  const tick = () => {
    if (isSteeringDirection) {
      sendDrive("forward").catch(console.error);
      sendSteer(direction).catch(console.error);
      return;
    }
    sendDrive(direction).catch(console.error);
  };
  tick();
  driveHoldTimer = setInterval(tick, 100);
}

function bindDriveHold(button, direction) {
  if (direction === "stop") {
    button.addEventListener("click", () => {
      stopDriveHold();
      postJson("/api/stop", {}).then(renderState).catch(console.error);
    });
    return;
  }
  button.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    startDriveHold(direction);
  });
  button.addEventListener("touchstart", (event) => {
    event.preventDefault();
    startDriveHold(direction);
  }, { passive: false });
  button.addEventListener("mousedown", (event) => {
    event.preventDefault();
    startDriveHold(direction);
  });
  ["pointerup", "pointercancel", "pointerleave", "touchend", "touchcancel", "mouseup", "mouseleave"].forEach((eventName) => {
    button.addEventListener(eventName, stopDriveHold);
  });
}

document.querySelectorAll("[data-drive]").forEach((button) => {
  bindDriveHold(button, button.dataset.drive);
});

async function sendServo(direction) {
  return postJson("/api/servo", {
    servo_id: Number(servoIdSelect.value),
    direction,
  }).then(renderState);
}

function stopServoHold() {
  if (servoHoldTimer) {
    clearInterval(servoHoldTimer);
    servoHoldTimer = null;
  }
  if (activeServoDirection !== "stop") {
    activeServoDirection = "stop";
    sendServo("stop").catch(console.error);
  }
}

function startServoHold(direction) {
  if (direction === "center") {
    stopServoHold();
    sendServo("center").catch(console.error);
    return;
  }
  if (activeServoDirection === direction) {
    return;
  }
  stopServoHold();
  activeServoDirection = direction;
  sendServo(direction).catch(console.error);
  servoHoldTimer = setInterval(() => {
    sendServo(direction).catch(console.error);
  }, 120);
}

function bindServoHold(button, direction) {
  if (direction === "center") {
    button.addEventListener("click", () => startServoHold(direction));
    return;
  }
  if (direction === "stop") {
    button.addEventListener("click", () => {
      stopServoHold();
      sendServo("stop").catch(console.error);
    });
    return;
  }
  button.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    startServoHold(direction);
  });
  button.addEventListener("touchstart", (event) => {
    event.preventDefault();
    startServoHold(direction);
  }, { passive: false });
  button.addEventListener("mousedown", (event) => {
    event.preventDefault();
    startServoHold(direction);
  });
  ["pointerup", "pointercancel", "pointerleave", "touchend", "touchcancel", "mouseup", "mouseleave"].forEach((eventName) => {
    button.addEventListener(eventName, stopServoHold);
  });
}

async function sendServoGroup(direction) {
  const targets = Array.from(document.querySelectorAll(".servo-target:checked")).map((item) => Number(item.value));
  return postJson("/api/servo-group", {
    targets,
    direction,
  }).then(renderState);
}

function stopServoGroupHold() {
  if (servoGroupHoldTimer) {
    clearInterval(servoGroupHoldTimer);
    servoGroupHoldTimer = null;
  }
  if (activeServoGroupDirection !== "stop") {
    activeServoGroupDirection = "stop";
    sendServoGroup("stop").catch(console.error);
  }
}

function startServoGroupHold(direction) {
  if (direction === "center") {
    stopServoGroupHold();
    sendServoGroup("center").catch(console.error);
    return;
  }
  if (activeServoGroupDirection === direction) {
    return;
  }
  stopServoGroupHold();
  activeServoGroupDirection = direction;
  sendServoGroup(direction).catch(console.error);
  servoGroupHoldTimer = setInterval(() => {
    sendServoGroup(direction).catch(console.error);
  }, 120);
}

function bindServoGroupHold(button, direction) {
  if (direction === "center") {
    button.addEventListener("click", () => startServoGroupHold(direction));
    return;
  }
  if (direction === "stop") {
    button.addEventListener("click", () => {
      stopServoGroupHold();
      sendServoGroup("stop").catch(console.error);
    });
    return;
  }
  button.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    startServoGroupHold(direction);
  });
  button.addEventListener("touchstart", (event) => {
    event.preventDefault();
    startServoGroupHold(direction);
  }, { passive: false });
  button.addEventListener("mousedown", (event) => {
    event.preventDefault();
    startServoGroupHold(direction);
  });
  ["pointerup", "pointercancel", "pointerleave", "touchend", "touchcancel", "mouseup", "mouseleave"].forEach((eventName) => {
    button.addEventListener(eventName, stopServoGroupHold);
  });
}

document.getElementById("stopBtn").addEventListener("click", () => postJson("/api/stop", {}).then(renderState));
document.getElementById("estopBtn").addEventListener("click", () => {
  const active = state.current?.mode !== "ESTOP";
  postJson("/api/estop", { active }).then(renderState);
});

document.getElementById("applySpeedBtn").addEventListener("click", () => {
  postJson("/api/speed", { speed_percent: Number(speedSlider.value) }).then(renderState);
});

document.querySelectorAll("[data-servo]").forEach((button) => {
  bindServoHold(button, button.dataset.servo);
});

document.querySelectorAll("[data-servo-group]").forEach((button) => {
  bindServoGroupHold(button, button.dataset.servoGroup);
});

resolutionSelect.addEventListener("change", () => { viewerConfigDirty = true; });
fpsSelect.addEventListener("change", () => { viewerConfigDirty = true; });
applyConfigBtn.addEventListener("click", () => applyViewerConfig().catch(console.error));
speedSlider.addEventListener("input", () => { speedSliderValue.textContent = `${speedSlider.value}%`; });

stream.addEventListener("click", (event) => queryDepthAt(event).catch(console.error));
colorStream.addEventListener("click", (event) => queryDepthAt(event).catch(console.error));
depthStream.addEventListener("click", (event) => queryDepthAt(event).catch(console.error));
window.addEventListener("blur", stopDriveHold);
window.addEventListener("pagehide", stopDriveHold);
document.addEventListener("visibilitychange", () => {
  if (document.hidden) {
    stopDriveHold();
    stopServoHold();
    stopServoGroupHold();
  }
});

function normalizeSelection(start, end) {
  const rect = stream.getBoundingClientRect();
  const scaleX = stream.naturalWidth / rect.width;
  const scaleY = stream.naturalHeight / rect.height;
  const x1 = Math.max(0, Math.min(stream.naturalWidth, Math.floor((start.x - rect.left) * scaleX)));
  const y1 = Math.max(0, Math.min(stream.naturalHeight, Math.floor((start.y - rect.top) * scaleY)));
  const x2 = Math.max(0, Math.min(stream.naturalWidth, Math.floor((end.x - rect.left) * scaleX)));
  const y2 = Math.max(0, Math.min(stream.naturalHeight, Math.floor((end.y - rect.top) * scaleY)));
  const x = Math.min(x1, x2);
  const y = Math.min(y1, y2);
  const width = Math.max(8, Math.abs(x2 - x1));
  const height = Math.max(8, Math.abs(y2 - y1));
  return { x, y, width, height };
}

overlayCanvas.addEventListener("pointerdown", (event) => {
  if (currentView !== "color" && currentView !== "split") return;
  dragStart = { x: event.clientX, y: event.clientY };
  pendingSelection = null;
});

overlayCanvas.addEventListener("pointermove", (event) => {
  if (!dragStart) return;
  pendingSelection = normalizeSelection(dragStart, { x: event.clientX, y: event.clientY });
  if (state.current) {
    drawTrackingOverlay(state.current);
  }
});

overlayCanvas.addEventListener("pointerup", (event) => {
  if (!dragStart) return;
  pendingSelection = normalizeSelection(dragStart, { x: event.clientX, y: event.clientY });
  dragStart = null;
  if (state.current) {
    drawTrackingOverlay(state.current);
  }
});

startFollowBtn.addEventListener("click", async () => {
  if (!pendingSelection) {
    followHint.textContent = "先在彩色画面上拖出一个目标框";
    return;
  }
  const snapshot = await postJson("/api/follow/init", pendingSelection);
  pendingSelection = null;
  followHint.textContent = "目标已锁定，自动跟随中";
  renderState(snapshot);
});

stopFollowBtn.addEventListener("click", async () => {
  const snapshot = await postJson("/api/follow/stop", {});
  pendingSelection = null;
  followHint.textContent = "已停止跟随，可重新框选目标";
  renderState(snapshot);
});

window.addEventListener("resize", () => {
  if (state.current) {
    drawTrackingOverlay(state.current);
  }
});

refreshState().catch(console.error);
connectWebSocket();
