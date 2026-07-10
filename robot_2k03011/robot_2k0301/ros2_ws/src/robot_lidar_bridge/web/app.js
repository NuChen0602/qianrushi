const canvas = document.getElementById("mapCanvas");
const context = canvas.getContext("2d");
const mapStage = document.getElementById("mapStage");
const mapEmpty = document.getElementById("mapEmpty");
const waypointList = document.getElementById("waypointList");
const eventLog = document.getElementById("eventLog");

const ui = {
  activeModeText: document.getElementById("activeModeText"),
  navStateText: document.getElementById("navStateText"),
  patrolState: document.getElementById("patrolState"),
  progressFill: document.getElementById("progressFill"),
  progressText: document.getElementById("progressText"),
  goalSummary: document.getElementById("goalSummary"),
  plannerDetail: document.getElementById("plannerDetail"),
  navigationDetail: document.getElementById("navigationDetail"),
  systemUptime: document.getElementById("systemUptime"),
  mapInfo: document.getElementById("mapInfo"),
  eventCount: document.getElementById("eventCount"),
};

let mapData = null;
let liveState = null;
let mapBitmap = null;
let mode = "goal";
let pointerStart = null;
let waypoints = JSON.parse(localStorage.getItem("patrolWaypoints") || "[]");
let viewport = { scale: 1, left: 0, top: 0 };
let localEvents = [];
let lastStates = { planner: "", navigation: "" };

const modeLabels = {
  goal: "目标点",
  initial: "初始位姿",
  waypoint: "巡检点",
};

const stateLabels = {
  starting: "启动中",
  waiting_map: "等待地图",
  ready: "就绪",
  planning: "规划中",
  path_ready: "路径就绪",
  waiting_for_path: "等待路径",
  following: "导航中",
  replanning: "重新规划",
  recovering: "恢复中",
  localization_paused: "定位暂停",
  obstacle_waiting: "避障等待",
  reached: "已到达",
  failed: "失败",
  cancelled: "已取消",
  emergency_stopped: "急停",
  idle: "空闲",
  completed: "已完成",
  waiting: "等待",
  navigating: "巡检中",
  initializing: "确认中",
  good: "良好",
  degraded: "下降",
  lost: "丢失",
  clear: "畅通",
  blocked: "受阻",
};

function stateText(state) {
  return stateLabels[state] || state || "--";
}

function setHealth(elementId, label, kind) {
  const element = document.getElementById(elementId);
  element.textContent = label;
  element.className = `health ${kind}`;
}

function statusKind(state) {
  if (["failed", "emergency_stopped", "lost"].includes(state)) return "error";
  if (["planning", "following", "replanning", "recovering", "localization_paused", "obstacle_waiting", "waiting_for_path", "navigating"].includes(state)) {
    return "busy";
  }
  if (["ready", "path_ready", "reached", "idle", "completed"].includes(state)) {
    return "ok";
  }
  return "idle";
}

function formatDuration(seconds) {
  if (!Number.isFinite(seconds)) return "00:00:00";
  const total = Math.max(0, Math.floor(seconds));
  const hours = String(Math.floor(total / 3600)).padStart(2, "0");
  const minutes = String(Math.floor((total % 3600) / 60)).padStart(2, "0");
  const rest = String(total % 60).padStart(2, "0");
  return `${hours}:${minutes}:${rest}`;
}

function formatEventTime(value) {
  const date = new Date((value || Date.now() / 1000) * 1000);
  return date.toLocaleTimeString([], { hour12: false });
}

function pushLocalEvent(text, level = "info", source = "app") {
  localEvents.unshift({
    id: `local-${Date.now()}-${Math.random()}`,
    time: Date.now() / 1000,
    level,
    source,
    text,
  });
  localEvents = localEvents.slice(0, 20);
  renderEventLog();
}

function renderEventLog() {
  const backendEvents = liveState?.events || [];
  const events = [...localEvents, ...backendEvents]
    .sort((a, b) => (b.time || 0) - (a.time || 0))
    .slice(0, 42);
  eventLog.replaceChildren();
  ui.eventCount.textContent = events.length;
  for (const item of events) {
    const row = document.createElement("div");
    row.className = `event-entry ${item.level || "info"}`;
    const time = document.createElement("time");
    time.textContent = formatEventTime(item.time);
    const source = document.createElement("span");
    source.className = "event-source";
    source.textContent = item.source || "system";
    const text = document.createElement("span");
    text.textContent = item.text || "";
    row.append(time, source, text);
    eventLog.append(row);
  }
}

function updateStatusUi() {
  if (!liveState) return;

  const localization = liveState.localization || {};
  const localizationState = localization.state || "initializing";
  const localizationOnline = liveState.robot_connected && localization.ok;
  setHealth(
    "robotHealth",
    `定位 ${stateText(localizationState)}`,
    localizationOnline ? (localizationState === "degraded" ? "busy" : "ok") : "error"
  );
  setHealth(
    "lidarHealth",
    liveState.lidar_connected ? "雷达在线" : "雷达离线",
    liveState.lidar_connected ? "ok" : "off"
  );

  const plannerState = liveState.planner?.state || "starting";
  const navigationState = liveState.navigation?.state || "starting";
  setHealth("plannerHealth", `规划 ${stateText(plannerState)}`, statusKind(plannerState));
  setHealth("navHealth", `导航 ${stateText(navigationState)}`, statusKind(navigationState));

  ui.navStateText.textContent = stateText(navigationState);
  ui.activeModeText.textContent = modeLabels[mode];
  ui.patrolState.textContent = stateText(liveState.patrol?.state || "idle");
  ui.systemUptime.textContent = formatDuration(liveState.system?.uptime_sec);

  if (mapData?.ready) {
    const width = (mapData.width * mapData.resolution).toFixed(2);
    const height = (mapData.height * mapData.resolution).toFixed(2);
    ui.mapInfo.textContent = `${width}m x ${height}m`;
  } else {
    ui.mapInfo.textContent = "等待地图";
  }

  if (plannerState !== lastStates.planner) {
    lastStates.planner = plannerState;
  }
  if (navigationState !== lastStates.navigation) {
    lastStates.navigation = navigationState;
  }

  const pose = liveState.robot;
  document.getElementById("poseX").textContent = pose ? `${pose.x.toFixed(3)} m` : "--";
  document.getElementById("poseY").textContent = pose ? `${pose.y.toFixed(3)} m` : "--";
  document.getElementById("poseYaw").textContent = pose
    ? `${(pose.yaw * 180 / Math.PI).toFixed(1)}°`
    : "--";

  const remaining = liveState.navigation?.remaining_m;
  const pathIndex = liveState.navigation?.path_index;
  const pathSize = liveState.navigation?.path_size;
  let progress = 0;
  if (Number.isFinite(pathIndex) && Number.isFinite(pathSize) && pathSize > 1) {
    progress = Math.max(0, Math.min(1, pathIndex / (pathSize - 1)));
  } else if (navigationState === "reached") {
    progress = 1;
  }
  ui.progressFill.style.width = `${Math.round(progress * 100)}%`;
  ui.progressText.textContent = Number.isFinite(progress) ? `${Math.round(progress * 100)}%` : "--";

  document.getElementById("remaining").textContent =
    Number.isFinite(remaining) ? `${remaining.toFixed(3)} m` : "--";
  document.getElementById("localizationQuality").textContent =
    Number.isFinite(localization.quality) ? `${localization.quality.toFixed(0)} / 100` : "--";
  document.getElementById("dynamicObstacles").textContent =
    Number(liveState.obstacle?.dynamic_points || 0);
  const drive = liveState.drive || {};
  document.getElementById("driveSpeed").textContent =
    Number.isFinite(drive.applied_speed_mps) ? `${drive.applied_speed_mps.toFixed(3)} m/s` : "--";
  document.getElementById("servoAngle").textContent =
    Number.isFinite(drive.servo_deg) ? `${drive.servo_deg.toFixed(1)}°` : "--";
  document.getElementById("wheelAngle").textContent =
    Number.isFinite(drive.front_wheel_deg) ? `${drive.front_wheel_deg.toFixed(1)}°` : "--";
  document.getElementById("turnRadius").textContent =
    Number.isFinite(drive.turning_radius_m) ? `${Math.abs(drive.turning_radius_m).toFixed(2)} m` : "直线";
  document.getElementById("pathCount").textContent = liveState.path?.length || 0;
  document.getElementById("scanCount").textContent = liveState.scan?.length || 0;

  if (liveState.goal) {
    ui.goalSummary.textContent =
      `目标 ${liveState.goal.x.toFixed(2)}, ${liveState.goal.y.toFixed(2)} · ` +
      `${(liveState.goal.yaw * 180 / Math.PI).toFixed(0)}°`;
  } else {
    ui.goalSummary.textContent = "未设置目标";
  }

  const planner = liveState.planner || {};
  if (planner.reason) {
    ui.plannerDetail.textContent = `规划：${stateText(plannerState)} · ${planner.reason}`;
  } else if (Number.isFinite(planner.poses)) {
    ui.plannerDetail.textContent =
      `规划：${planner.poses} 点 · ${Number(planner.planning_time_sec || 0).toFixed(2)}s`;
  } else {
    ui.plannerDetail.textContent = `规划：${stateText(plannerState)}`;
  }

  const nav = liveState.navigation || {};
  if (navigationState === "obstacle_waiting") {
    ui.navigationDetail.textContent =
      `避障：发现 ${Number(liveState.obstacle?.blocking_points || 0)} 个阻挡点，等待绕行`;
  } else if (navigationState === "localization_paused") {
    ui.navigationDetail.textContent =
      `定位：${stateText(localizationState)} · ${localization.reason || "等待恢复"}`;
  } else if (Number.isFinite(nav.cross_track_m)) {
    ui.navigationDetail.textContent =
      `跟踪：偏差 ${nav.cross_track_m.toFixed(3)}m · 舵量 ${Number(nav.steering || 0).toFixed(2)}`;
  } else {
    ui.navigationDetail.textContent = `跟踪：${stateText(navigationState)}`;
  }

  document.getElementById("emergencyStop").disabled = liveState.emergency_stop;
  document.getElementById("emergencyRelease").disabled = !liveState.emergency_stop;
  renderEventLog();
}

function buildMapBitmap() {
  if (!mapData?.ready) return;
  const offscreen = document.createElement("canvas");
  offscreen.width = mapData.width;
  offscreen.height = mapData.height;
  const offscreenContext = offscreen.getContext("2d");
  const image = offscreenContext.createImageData(mapData.width, mapData.height);
  for (let gy = 0; gy < mapData.height; gy += 1) {
    for (let gx = 0; gx < mapData.width; gx += 1) {
      const occupancy = mapData.data[gy * mapData.width + gx];
      const imageY = mapData.height - 1 - gy;
      const index = (imageY * mapData.width + gx) * 4;
      let color = [244, 248, 246];
      if (occupancy < 0) color = [163, 173, 176];
      if (occupancy >= 65) color = [36, 43, 47];
      image.data[index] = color[0];
      image.data[index + 1] = color[1];
      image.data[index + 2] = color[2];
      image.data[index + 3] = 255;
    }
  }
  offscreenContext.putImageData(image, 0, 0);
  mapBitmap = offscreen;
  mapEmpty.style.display = "none";
}

function resizeCanvas() {
  const rect = mapStage.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.round(rect.width * ratio));
  canvas.height = Math.max(1, Math.round(rect.height * ratio));
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  computeViewport();
  draw();
}

function computeViewport() {
  if (!mapData?.ready) return;
  const rect = mapStage.getBoundingClientRect();
  const widthMeters = mapData.width * mapData.resolution;
  const heightMeters = mapData.height * mapData.resolution;
  const padding = 44;
  viewport.scale = Math.max(
    1,
    Math.min(
      (rect.width - padding * 2) / widthMeters,
      (rect.height - padding * 2) / heightMeters
    )
  );
  viewport.left = (rect.width - widthMeters * viewport.scale) / 2;
  viewport.top = (rect.height - heightMeters * viewport.scale) / 2;
}

function worldToLocal(x, y) {
  const origin = mapData.origin;
  const dx = x - origin.x;
  const dy = y - origin.y;
  const cosine = Math.cos(origin.yaw);
  const sine = Math.sin(origin.yaw);
  return {
    x: cosine * dx + sine * dy,
    y: -sine * dx + cosine * dy,
  };
}

function localToWorld(x, y) {
  const origin = mapData.origin;
  const cosine = Math.cos(origin.yaw);
  const sine = Math.sin(origin.yaw);
  return {
    x: origin.x + cosine * x - sine * y,
    y: origin.y + sine * x + cosine * y,
  };
}

function worldToCanvas(x, y) {
  const local = worldToLocal(x, y);
  const heightMeters = mapData.height * mapData.resolution;
  return {
    x: viewport.left + local.x * viewport.scale,
    y: viewport.top + (heightMeters - local.y) * viewport.scale,
  };
}

function canvasToWorld(clientX, clientY) {
  const rect = canvas.getBoundingClientRect();
  const heightMeters = mapData.height * mapData.resolution;
  const localX = (clientX - rect.left - viewport.left) / viewport.scale;
  const localY = heightMeters - (clientY - rect.top - viewport.top) / viewport.scale;
  return localToWorld(localX, localY);
}

function drawGrid() {
  if (!mapData?.ready) return;
  const rect = mapStage.getBoundingClientRect();
  const stepMeters = 0.4;
  const origin = mapData.origin;
  const widthMeters = mapData.width * mapData.resolution;
  const heightMeters = mapData.height * mapData.resolution;
  context.save();
  context.strokeStyle = "rgba(255,255,255,0.10)";
  context.lineWidth = 1;
  context.beginPath();
  for (let x = 0; x <= widthMeters + 0.001; x += stepMeters) {
    const a = worldToCanvas(origin.x + x, origin.y);
    const b = worldToCanvas(origin.x + x, origin.y + heightMeters);
    context.moveTo(a.x, a.y);
    context.lineTo(b.x, b.y);
  }
  for (let y = 0; y <= heightMeters + 0.001; y += stepMeters) {
    const a = worldToCanvas(origin.x, origin.y + y);
    const b = worldToCanvas(origin.x + widthMeters, origin.y + y);
    context.moveTo(a.x, a.y);
    context.lineTo(b.x, b.y);
  }
  context.stroke();
  context.restore();
}

function drawPolyline(points, color, width) {
  if (!points || points.length < 2) return;
  context.save();
  context.beginPath();
  points.forEach((point, index) => {
    const canvasPoint = worldToCanvas(point[0], point[1]);
    if (index === 0) context.moveTo(canvasPoint.x, canvasPoint.y);
    else context.lineTo(canvasPoint.x, canvasPoint.y);
  });
  context.strokeStyle = color;
  context.lineWidth = width;
  context.lineCap = "round";
  context.lineJoin = "round";
  context.shadowColor = "rgba(15, 90, 42, 0.35)";
  context.shadowBlur = 8;
  context.stroke();
  context.restore();
}

function drawRobot(robot) {
  if (!robot) return;
  const center = worldToCanvas(robot.x, robot.y);
  const front = worldToCanvas(
    robot.x + 0.16 * Math.cos(robot.yaw),
    robot.y + 0.16 * Math.sin(robot.yaw)
  );
  const left = worldToCanvas(
    robot.x + 0.09 * Math.cos(robot.yaw + 2.45),
    robot.y + 0.09 * Math.sin(robot.yaw + 2.45)
  );
  const right = worldToCanvas(
    robot.x + 0.09 * Math.cos(robot.yaw - 2.45),
    robot.y + 0.09 * Math.sin(robot.yaw - 2.45)
  );
  context.save();
  context.beginPath();
  context.moveTo(front.x, front.y);
  context.lineTo(left.x, left.y);
  context.lineTo(right.x, right.y);
  context.closePath();
  context.fillStyle = "#dd7b20";
  context.strokeStyle = "#6f3608";
  context.lineWidth = 1.5;
  context.shadowColor = "rgba(0,0,0,0.25)";
  context.shadowBlur = 10;
  context.fill();
  context.stroke();
  context.beginPath();
  context.arc(center.x, center.y, 3, 0, Math.PI * 2);
  context.fillStyle = "#ffffff";
  context.fill();
  context.restore();
}

function drawGoal(goal) {
  if (!goal) return;
  const center = worldToCanvas(goal.x, goal.y);
  const end = worldToCanvas(
    goal.x + 0.18 * Math.cos(goal.yaw),
    goal.y + 0.18 * Math.sin(goal.yaw)
  );
  context.save();
  context.strokeStyle = "#2764b8";
  context.fillStyle = "#2764b8";
  context.lineWidth = 2;
  context.beginPath();
  context.arc(center.x, center.y, 6, 0, Math.PI * 2);
  context.stroke();
  context.beginPath();
  context.moveTo(center.x, center.y);
  context.lineTo(end.x, end.y);
  context.stroke();
  context.restore();
}

function drawWaypoints() {
  context.save();
  waypoints.forEach((waypoint, index) => {
    const point = worldToCanvas(waypoint.x, waypoint.y);
    context.beginPath();
    context.arc(point.x, point.y, 10, 0, Math.PI * 2);
    context.fillStyle = "#ffffff";
    context.fill();
    context.strokeStyle = "#1f7a4d";
    context.lineWidth = 2;
    context.stroke();
    context.fillStyle = "#1f7a4d";
    context.font = "800 10px sans-serif";
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillText(String(index + 1), point.x, point.y);
  });
  context.restore();
}

function drawPointerPreview() {
  if (!pointerStart?.current) return;
  const start = worldToCanvas(pointerStart.world.x, pointerStart.world.y);
  const current = worldToCanvas(pointerStart.current.x, pointerStart.current.y);
  context.save();
  context.beginPath();
  context.moveTo(start.x, start.y);
  context.lineTo(current.x, current.y);
  context.strokeStyle = mode === "initial" ? "#d97706" : "#2764b8";
  context.lineWidth = 2;
  context.setLineDash([6, 5]);
  context.stroke();
  context.restore();
}

function draw() {
  const rect = mapStage.getBoundingClientRect();
  context.clearRect(0, 0, rect.width, rect.height);
  context.fillStyle = "#252b2f";
  context.fillRect(0, 0, rect.width, rect.height);
  if (!mapData?.ready || !mapBitmap) return;

  context.imageSmoothingEnabled = false;
  context.drawImage(
    mapBitmap,
    viewport.left,
    viewport.top,
    mapData.width * mapData.resolution * viewport.scale,
    mapData.height * mapData.resolution * viewport.scale
  );
  drawGrid();
  drawPolyline(liveState?.path, "#1f9e58", 4);

  if (liveState?.scan?.length) {
    context.fillStyle = "#d53f8c";
    for (const point of liveState.scan) {
      const canvasPoint = worldToCanvas(point[0], point[1]);
      context.fillRect(canvasPoint.x - 1.4, canvasPoint.y - 1.4, 2.8, 2.8);
    }
  }
  drawWaypoints();
  drawGoal(liveState?.goal);
  drawRobot(liveState?.robot);
  drawPointerPreview();
}

async function requestJson(path, method = "POST", payload = null) {
  const options = { method, headers: {} };
  if (payload !== null) {
    options.headers["Content-Type"] = "application/json";
    options.body = JSON.stringify(payload);
  }
  const response = await fetch(path, options);
  const result = await response.json();
  if (!response.ok || !result.ok) {
    throw new Error(result.error || `HTTP ${response.status}`);
  }
  return result;
}

async function loadMap() {
  try {
    const response = await fetch("/api/map", { cache: "no-store" });
    const payload = await response.json();
    if (payload.ready) {
      mapData = payload;
      buildMapBitmap();
      computeViewport();
      draw();
    } else {
      setTimeout(loadMap, 1000);
    }
  } catch (error) {
    pushLocalEvent(`地图连接失败：${error.message}`, "error");
    setTimeout(loadMap, 1500);
  }
}

function connectEvents() {
  const source = new EventSource("/api/events");
  source.onmessage = (event) => {
    liveState = JSON.parse(event.data);
    if (mapData && liveState.map_version !== mapData.version) loadMap();
    updateStatusUi();
    draw();
  };
  source.onerror = () => {
    source.close();
    pushLocalEvent("上位机事件流断开，正在重连", "warn");
    setTimeout(connectEvents, 1000);
  };
}

function saveWaypoints() {
  localStorage.setItem("patrolWaypoints", JSON.stringify(waypoints));
  renderWaypoints();
  draw();
}

function renderWaypoints() {
  waypointList.replaceChildren();
  waypoints.forEach((waypoint, index) => {
    const row = document.createElement("li");
    row.className = "waypoint-item";
    row.innerHTML = `
      <span class="waypoint-number">${index + 1}</span>
      <div class="waypoint-detail">
        <strong>${waypoint.name}</strong>
        <span>${waypoint.x.toFixed(2)}, ${waypoint.y.toFixed(2)} · ${(waypoint.yaw * 180 / Math.PI).toFixed(0)}°</span>
      </div>
      <button type="button" class="waypoint-remove" aria-label="删除巡检点">×</button>
    `;
    row.querySelector("button").addEventListener("click", () => {
      waypoints.splice(index, 1);
      saveWaypoints();
    });
    waypointList.append(row);
  });
}

document.querySelectorAll(".mode").forEach((button) => {
  button.addEventListener("click", () => {
    mode = button.dataset.mode;
    document.querySelectorAll(".mode").forEach((item) => {
      item.classList.toggle("active", item === button);
    });
    ui.activeModeText.textContent = modeLabels[mode];
  });
});

canvas.addEventListener("pointerdown", (event) => {
  if (!mapData?.ready || event.button !== 0) return;
  canvas.setPointerCapture(event.pointerId);
  const world = canvasToWorld(event.clientX, event.clientY);
  pointerStart = { world, current: world, clientX: event.clientX, clientY: event.clientY };
  draw();
});

canvas.addEventListener("pointermove", (event) => {
  if (!mapData?.ready) return;
  const world = canvasToWorld(event.clientX, event.clientY);
  document.getElementById("cursorPosition").textContent =
    `x ${world.x.toFixed(2)} · y ${world.y.toFixed(2)}`;
  if (pointerStart) {
    pointerStart.current = world;
    draw();
  }
});

canvas.addEventListener("pointerup", async (event) => {
  if (!pointerStart || !mapData?.ready) return;
  const start = pointerStart;
  const end = canvasToWorld(event.clientX, event.clientY);
  const drag = Math.hypot(event.clientX - start.clientX, event.clientY - start.clientY);
  const yaw = drag > 8
    ? Math.atan2(end.y - start.world.y, end.x - start.world.x)
    : (liveState?.robot?.yaw || 0);
  pointerStart = null;
  draw();
  const pose = { x: start.world.x, y: start.world.y, yaw };
  try {
    if (mode === "goal") {
      await requestJson("/api/goal", "POST", pose);
      pushLocalEvent(`目标点：${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}`, "ok");
    } else if (mode === "initial") {
      await requestJson("/api/initial-pose", "POST", pose);
      pushLocalEvent(`初始位姿：${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}`, "ok");
    } else {
      waypoints.push({ ...pose, name: `巡检点 ${waypoints.length + 1}` });
      saveWaypoints();
      pushLocalEvent(`已加入巡检点 ${waypoints.length}`, "ok");
    }
  } catch (error) {
    pushLocalEvent(`操作失败：${error.message}`, "error");
  }
});

document.getElementById("fitMap").addEventListener("click", () => {
  computeViewport();
  draw();
});

document.getElementById("clearWaypoints").addEventListener("click", () => {
  waypoints = [];
  saveWaypoints();
  pushLocalEvent("巡检队列已清空", "warn");
});

document.getElementById("startPatrol").addEventListener("click", async () => {
  try {
    await requestJson("/api/patrol/start", "POST", {
      waypoints,
      repeat: document.getElementById("repeatPatrol").checked,
    });
    pushLocalEvent(`巡检开始，共 ${waypoints.length} 个点`, "ok");
  } catch (error) {
    pushLocalEvent(`巡检启动失败：${error.message}`, "error");
  }
});

document.getElementById("cancelPatrol").addEventListener("click", async () => {
  try {
    await requestJson("/api/patrol/cancel");
    pushLocalEvent("巡检已停止", "warn");
  } catch (error) {
    pushLocalEvent(`停止失败：${error.message}`, "error");
  }
});

document.getElementById("cancelNav").addEventListener("click", async () => {
  try {
    await requestJson("/api/cancel");
    pushLocalEvent("导航已取消", "warn");
  } catch (error) {
    pushLocalEvent(`取消失败：${error.message}`, "error");
  }
});

document.getElementById("emergencyStop").addEventListener("click", async () => {
  try {
    await requestJson("/api/emergency-stop");
    pushLocalEvent("急停已触发", "error");
  } catch (error) {
    pushLocalEvent(`急停失败：${error.message}`, "error");
  }
});

document.getElementById("emergencyRelease").addEventListener("click", async () => {
  try {
    await requestJson("/api/emergency-release");
    pushLocalEvent("急停已解除", "ok");
  } catch (error) {
    pushLocalEvent(`解除失败：${error.message}`, "error");
  }
});

new ResizeObserver(resizeCanvas).observe(mapStage);
renderWaypoints();
renderEventLog();
loadMap();
connectEvents();
resizeCanvas();
