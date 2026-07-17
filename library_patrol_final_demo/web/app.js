
// === final demo map point filter ===
// 默认隐藏额外业务点位，只保留三个书架和安全出口。
// 白色无字按钮可切换显示全部点位。
window.__hideExtraMapPoints = true;

const FINAL_DEMO_VISIBLE_MAP_POINTS = new Set([
  "LIT_SHELF_A3",
  "ENG_SHELF_B1",
  "SCI_SHELF_C1",
  "HAZARD_3_EXIT",
]);

function shouldDrawMapPoint(pointKey, pointObj) {
  if (!window.__hideExtraMapPoints) {
    return true;
  }

  const key = String(pointKey || "");
  if (FINAL_DEMO_VISIBLE_MAP_POINTS.has(key)) {
    return true;
  }

  const name = String((pointObj && pointObj.name) || "");
  return (
    name.includes("文学书架") ||
    name.includes("工科书架") ||
    name.includes("理科书架") ||
    name.includes("安全出口")
  );
}

function initMapPointFilterToggle() {
  if (document.getElementById("map-point-filter-toggle")) {
    return;
  }

  const panels = Array.from(document.querySelectorAll("section, .card, .panel, .map-panel, body > div"));
  const mapPanel = panels.find((el) => (el.textContent || "").includes("地图与导航"));

  if (!mapPanel) {
    return;
  }

  mapPanel.classList.add("map-panel-with-filter-toggle");

  const btn = document.createElement("button");
  btn.id = "map-point-filter-toggle";
  btn.type = "button";
  btn.className = "map-point-filter-toggle is-compact";
  btn.setAttribute("aria-label", "切换地图点位显示");

  btn.addEventListener("click", (event) => {
    event.preventDefault();
    event.stopPropagation();

    window.__hideExtraMapPoints = !window.__hideExtraMapPoints;
    btn.classList.toggle("is-compact", window.__hideExtraMapPoints);
  });

  mapPanel.appendChild(btn);
}

window.addEventListener("DOMContentLoaded", initMapPointFilterToggle);
setTimeout(initMapPointFilterToggle, 500);

let latestState = null;
let latestMap = null;
let latestLostVision = null;
let latestBookVision = null;
let lostVisionRequestPending = false;
let bookVisionRequestPending = false;

const $ = (id) => document.getElementById(id);

async function api(path, options = {}) {
  const { timeoutMs = 1800, ...fetchOptions } = options;
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), timeoutMs);
  try {
    const response = await fetch(path, {
      cache: "no-store",
      headers: { "Content-Type": "application/json" },
      ...fetchOptions,
      signal: controller.signal,
    });
    const data = await response.json();
    if (!response.ok || data.ok === false) {
      throw new Error(data.error || response.statusText);
    }
    return data;
  } finally {
    clearTimeout(timeout);
  }
}

function setText(id, value) {
  $(id).textContent = value || "-";
}

function mission(missionId) {
  api("/api/demo/mission", {
    method: "POST",
    body: JSON.stringify({ mission: missionId }),
  }).catch((error) => alert(error.message));
}

function simulateVoice(command) {
  api("/api/demo/simulate-voice", {
    method: "POST",
    body: JSON.stringify({ command }),
  }).catch((error) => alert(error.message));
}

function postEmpty(path) {
  api(path, { method: "POST" }).catch((error) => alert(error.message));
}

function worldToCanvas(map, x, y, width, height, scale, offsetX, offsetY) {
  const origin = map.origin || { x: 0, y: 0 };
  const resolution = map.resolution || 0.02;
  const mx = (x - origin.x) / resolution;
  const my = (y - origin.y) / resolution;
  return [
    offsetX + mx * scale,
    offsetY + (height - my) * scale,
  ];
}

function drawMap() {
  const canvas = $("map-canvas");
  const ctx = canvas.getContext("2d");
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

  const width = rect.width;
  const height = rect.height;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#f7f9fb";
  ctx.fillRect(0, 0, width, height);

  const state = latestState || {};
  const nav = state.navigation || {};
  const map = latestMap && latestMap.ready ? latestMap : null;

  let scale = 90;
  let offsetX = width / 2;
  let offsetY = height / 2;
  let mapWidth = 0;
  let mapHeight = 0;

  if (map && Array.isArray(map.data)) {
    mapWidth = map.width;
    mapHeight = map.height;
    scale = Math.min(width / Math.max(1, mapWidth), height / Math.max(1, mapHeight)) * 0.92;
    offsetX = (width - mapWidth * scale) / 2;
    offsetY = (height - mapHeight * scale) / 2;
    const image = ctx.createImageData(mapWidth, mapHeight);

    // ROS OccupancyGrid 的 data 原点在左下角；
    // Canvas ImageData 的原点在左上角。
    // 因此前端显示时必须把栅格图像按 Y 方向翻转，
    // 否则地图图像会和机器人/目标点/路径坐标不一致。
    for (let gy = 0; gy < mapHeight; gy += 1) {
      for (let gx = 0; gx < mapWidth; gx += 1) {
        const srcIndex = gy * mapWidth + gx;
        const dstY = mapHeight - 1 - gy;
        const dstIndex = dstY * mapWidth + gx;
        const value = map.data[srcIndex];

        let r = 242;
        let g = 246;
        let b = 250;
        let a = 255;

        if (value === 0) {
          // 空闲区域
          r = 252; g = 253; b = 255;
        } else if (value < 0) {
          // 未知区域，淡化显示，不当成障碍
          r = 232; g = 238; b = 245;
        } else if (value >= 65) {
          // 占用区域/墙体
          r = 44; g = 49; b = 55;
        } else {
          // 中间概率区域，浅灰显示
          const shade = 235 - Math.round(value * 1.2);
          r = shade; g = shade; b = shade;
        }

        image.data[dstIndex * 4] = r;
        image.data[dstIndex * 4 + 1] = g;
        image.data[dstIndex * 4 + 2] = b;
        image.data[dstIndex * 4 + 3] = a;
      }
    }

    const temp = document.createElement("canvas");
    temp.width = mapWidth;
    temp.height = mapHeight;
    temp.getContext("2d").putImageData(image, 0, 0);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(temp, offsetX, offsetY, mapWidth * scale, mapHeight * scale);
  } else {
    ctx.strokeStyle = "#d5dee8";
    for (let x = 0; x < width; x += 32) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, height);
      ctx.stroke();
    }
    for (let y = 0; y < height; y += 32) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(width, y);
      ctx.stroke();
    }
    ctx.fillStyle = "#667789";
    ctx.font = "16px sans-serif";
    ctx.fillText("导航地图未连接或未加载", 22, 32);
  }

  const convert = (x, y) => {
    if (map) return worldToCanvas(map, x, y, mapWidth, mapHeight, scale, offsetX, offsetY);
    return [offsetX + x * scale, offsetY - y * scale];
  };

  const path = nav.path || [];
  if (Array.isArray(path) && path.length > 1) {
    ctx.strokeStyle = "#d28b25";
    ctx.lineWidth = 3;
    ctx.beginPath();
    path.forEach((p, index) => {
      const point = Array.isArray(p) ? { x: p[0], y: p[1] } : p;
      const [cx, cy] = convert(point.x, point.y);
      if (index === 0) ctx.moveTo(cx, cy);
      else ctx.lineTo(cx, cy);
    });
    ctx.stroke();
  }

  // Navigation supplies laser returns in map/world coordinates.
  const scan = nav.scan || [];
  if (Array.isArray(scan) && scan.length) {
    ctx.save();
    ctx.fillStyle = "rgba(14, 165, 233, 0.82)";
    scan.forEach((point) => {
      const p = Array.isArray(point) ? { x: point[0], y: point[1] } : point;
      if (!Number.isFinite(p.x) || !Number.isFinite(p.y)) return;
      const [cx, cy] = convert(p.x, p.y);
      ctx.fillRect(cx - 1.4, cy - 1.4, 2.8, 2.8);
    });
    ctx.restore();
  }

  const points = state.points || {};
  Object.entries(points).forEach(([id, point]) => {
    if (!shouldDrawMapPoint(id, point)) return;
    const [cx, cy] = convert(point.x, point.y);
    ctx.fillStyle = "#2f8f5b";
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = "#1e2a36";
    ctx.font = "11px sans-serif";
    ctx.fillText(id, cx + 6, cy - 6);
  });

  const goal = nav.goal;
  if (goal && Number.isFinite(goal.x) && Number.isFinite(goal.y)) {
    const [cx, cy] = convert(goal.x, goal.y);
    ctx.fillStyle = "#c43a31";
    ctx.beginPath();
    ctx.arc(cx, cy, 7, 0, Math.PI * 2);
    ctx.fill();
  }

  const robot = nav.robot;
  if (robot && Number.isFinite(robot.x) && Number.isFinite(robot.y)) {
    const [cx, cy] = convert(robot.x, robot.y);
    const yaw = robot.yaw || 0;
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-yaw);
    ctx.fillStyle = "#1f7a8c";
    ctx.beginPath();
    ctx.moveTo(12, 0);
    ctx.lineTo(-9, -7);
    ctx.lineTo(-6, 0);
    ctx.lineTo(-9, 7);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }
}

function renderState(data) {
  latestState = data;
  const order = data.work_order || {};
  const nav = data.navigation || {};
  const orch = data.orchestrator || {};
  $("connection").textContent = nav.available
    ? "导航 dashboard 已连接"
    : `导航未连接：${nav.error || "等待启动"}`;
  $("home-status").textContent = data.home_source || "起点未记录";
  $("orchestrator-state").textContent = orch.state || "idle";
  const camera = data.camera || {};
  const cameraStatus = $("camera-stream-status");
  if (cameraStatus) {
    cameraStatus.textContent = camera.has_frame
      ? `实时画面 · ${camera.source_fps || 0} FPS · 帧 ${camera.frame_id || 0}`
      : "实时画面 · 等待摄像头";
  }
  setText("task", order.current_title || order.current_task);
  setText("stage", order.stage);
  setText("nav-status", order.navigation_status);
  setText("vision-status", order.vision_status);
  setText("voice-status", order.voice_status);
  setText("result", order.result);
  setText("error", order.error);

  const events = $("events");
  events.innerHTML = "";
  (order.events || []).slice(0, 18).forEach((event) => {
    const li = document.createElement("li");
    li.className = event.level || "";
    li.innerHTML = `<span class="time">${event.time}</span> [${event.source}] ${event.text}`;
    events.appendChild(li);
  });
  drawMap();
  drawCameraOverlay();
  updateAIVisionPanelFromState(data);
  updateSmokeStatus(data.smoke_sensor || {});
  updateEnvironmentStatus(data.environment || {});
}

function updateEnvironmentStatus(environment) {
  const tempEl = document.getElementById("env-temperature");
  const humEl = document.getElementById("env-humidity");
  const stateEl = document.getElementById("env-state");
  if (!tempEl || !humEl || !stateEl) return;
  if (!environment.ok) {
    tempEl.textContent = "--℃";
    humEl.textContent = "--%";
    const error = String(environment.error || "");
    stateEl.textContent = error.includes("未注册")
      ? "DHT11 驱动未注册"
      : "温湿度传感器离线";
    stateEl.className = "env-value";
    return;
  }
  tempEl.textContent = `${Number(environment.temperature_c).toFixed(1)}℃`;
  humEl.textContent = `${Number(environment.humidity_rh).toFixed(1)}%`;
  stateEl.textContent = environment.stale ? "数据陈旧" : "正常";
  stateEl.className = environment.stale ? "env-value" : "env-value env-ok";
}

function updateSmokeStatus(smoke) {
  const valueEl = document.getElementById("env-smoke");
  const stateEl = document.getElementById("env-state");
  if (!valueEl || !stateEl) return;
  if (!smoke.enabled) {
    valueEl.textContent = "MQ-2 已禁用";
    return;
  }
  if (!smoke.available) {
    valueEl.textContent = "MQ-2 / A3 未连接";
    stateEl.textContent = "传感器离线";
    stateEl.className = "env-value";
    return;
  }
  const volts = Number(smoke.voltage_mv || 0) / 1000;
  valueEl.textContent = `MQ-2 / A3 ${volts.toFixed(3)}V (raw ${smoke.raw})`;
  stateEl.textContent = smoke.alarm ? "烟雾告警" : "正常";
  stateEl.className = smoke.alarm ? "env-value" : "env-value env-ok";
}

async function refresh() {
  if (refresh.pending) return;
  refresh.pending = true;
  try {
    const data = await api("/api/demo/state", { timeoutMs: 1200 });
    renderState(data);
  } catch (error) {
    $("connection").textContent = `演示服务异常：${error.message}`;
  } finally {
    refresh.pending = false;
  }
}
refresh.pending = false;

async function refreshMap() {
  if (refreshMap.pending) return;
  refreshMap.pending = true;
  try {
    latestMap = await api("/api/demo/map", { timeoutMs: 1200 });
    drawMap();
  } catch (_error) {
    // Keep the last valid map visible during a transient request failure.
  } finally {
    refreshMap.pending = false;
  }
}
refreshMap.pending = false;

document.addEventListener("click", (event) => {
  const action = event.target.dataset.action;
  if (action === "cancel") postEmpty("/api/demo/cancel");
  if (action === "emergency-stop") postEmpty("/api/demo/emergency-stop");
  if (action === "emergency-release") postEmpty("/api/demo/emergency-release");
});

window.addEventListener("resize", () => {
  drawMap();
  drawCameraOverlay();
});
refresh();
refreshMap();
setInterval(refresh, 250);
setInterval(refreshMap, 1000);

// Firefox may leave an MJPEG <img> connection alive while no longer repainting it.
// Refresh the cached latest JPEG instead; the old image remains visible until the
// replacement has loaded, so this is both resilient and flicker-free.
const cameraFeed = document.getElementById("camera-feed");
let cameraRefreshPending = false;
let cameraObjectUrl = null;

async function refreshCameraFrame() {
  if (!cameraFeed || cameraRefreshPending || document.hidden) return;
  cameraRefreshPending = true;
  try {
    const response = await fetch(`/camera.jpg?t=${Date.now()}`, { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const blob = await response.blob();
    if (!blob.type.startsWith("image/")) throw new Error("返回内容不是图像");
    const nextUrl = URL.createObjectURL(blob);
    const previousUrl = cameraObjectUrl;
    cameraObjectUrl = nextUrl;
    cameraFeed.src = nextUrl;
    if (previousUrl) URL.revokeObjectURL(previousUrl);
  } catch (_error) {
    const status = document.getElementById("camera-stream-status");
    if (status) status.textContent = "摄像头画面断开，正在重连";
  } finally {
    cameraRefreshPending = false;
  }
}

if (cameraFeed) {
  cameraFeed.addEventListener("load", () => {
    const status = document.getElementById("camera-stream-status");
    if (status && status.textContent.includes("连接中")) status.textContent = "摄像头已连接";
    drawCameraOverlay();
  });
  setInterval(refreshCameraFrame, 220);
}


function isLostItemTask(state) {
  const wo = state && state.work_order ? state.work_order : {};
  const orch = state && state.orchestrator ? state.orchestrator : {};
  return (
    String(orch.active || "") === "LOST_ITEM_PATROL"
    || String(wo.current_task || "") === "LOST_ITEM_PATROL"
    || String(wo.current_title || "").includes("遗失物巡检")
    || String(wo.stage || "").toUpperCase().includes("LOST")
  );
}

function isBookVisionTask(state) {
  const wo = state && state.work_order ? state.work_order : {};
  const orch = state && state.orchestrator ? state.orchestrator : {};
  const title = String(wo.current_title || wo.title || "");
  const stage = String(wo.stage || "").toUpperCase();
  const running = !["completed", "failed", "idle", ""].includes(String(orch.state || ""));
  return running && (
    title.includes("寻书") || title.includes("书架") || stage.includes("SHELF") || stage.includes("BOOK")
  );
}

function drawDetectionBox(ctx, box, transform, color, label) {
  const x = transform.offsetX + Number(box.x || 0) * transform.scale;
  const y = transform.offsetY + Number(box.y || 0) * transform.scale;
  const width = Number(box.w || 0) * transform.scale;
  const height = Number(box.h || 0) * transform.scale;
  if (width <= 1 || height <= 1) return;

  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = 3;
  ctx.shadowColor = "rgba(15, 23, 42, 0.7)";
  ctx.shadowBlur = 3;
  ctx.strokeRect(x, y, width, height);
  ctx.shadowBlur = 0;

  ctx.font = "600 13px sans-serif";
  const textWidth = ctx.measureText(label).width;
  const labelHeight = 22;
  const labelY = y >= labelHeight + 4 ? y - labelHeight : y + 2;
  ctx.fillStyle = color;
  ctx.fillRect(x, labelY, textWidth + 12, labelHeight);
  ctx.fillStyle = color === "#facc15" ? "#422006" : "#ffffff";
  ctx.fillText(label, x + 6, labelY + 15);
  ctx.restore();
}

function drawCameraOverlay() {
  const canvas = document.getElementById("camera-overlay");
  const stage = document.getElementById("camera-stage");
  const imageElement = document.getElementById("camera-feed");
  if (!canvas || !stage || !imageElement) return;

  const rect = stage.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, rect.width, rect.height);

  const lostActive = isLostItemTask(latestState) && latestLostVision && latestLostVision.active;
  const bookTaskActive = isBookVisionTask(latestState);
  const bookVisible = latestBookVision && latestBookVision.ok;
  const payload = lostActive ? latestLostVision : (bookVisible ? latestBookVision : null);
  const imageSize = payload && payload.image_size ? payload.image_size : {};
  const sourceWidth = Number(imageSize.width || imageElement.naturalWidth || 640);
  const sourceHeight = Number(imageSize.height || imageElement.naturalHeight || 480);
  if (sourceWidth <= 0 || sourceHeight <= 0) return;

  // camera-feed uses object-fit: contain, so preserve its letterbox offsets.
  const scale = Math.min(rect.width / sourceWidth, rect.height / sourceHeight);
  const transform = {
    scale,
    offsetX: (rect.width - sourceWidth * scale) / 2,
    offsetY: (rect.height - sourceHeight * scale) / 2,
  };

  if (lostActive) {
    const ttlMs = Number(latestLostVision.track_ttl_sec || 1.5) * 1000;
    (latestLostVision.detections || []).forEach((detection) => {
      if (Number(detection.age_ms || 0) > ttlMs) return;
      const confirmed = detection.state === "confirmed";
      const score = Number(detection.score || 0);
      const label = confirmed
        ? `${detection.label || "遗失物"}${score > 0 ? ` ${(score * 100).toFixed(0)}%` : ""}`
        : `${detection.label || "疑似物品"} · 确认中`;
      drawDetectionBox(
        ctx,
        detection.bbox || {},
        transform,
        confirmed ? "#ef4444" : "#facc15",
        label,
      );
    });
  } else if (bookVisible) {
    (latestBookVision.books || []).forEach((book) => {
      const bbox = book.bbox || [];
      if (bbox.length !== 4) return;
      const target = bookTaskActive
        && Number(book.id) === Number(latestBookVision.expected_id);
      drawDetectionBox(ctx, {
        x: bbox[0],
        y: bbox[1],
        w: bbox[2] - bbox[0],
        h: bbox[3] - bbox[1],
      }, transform, target ? "#ef4444" : "#22c55e", `ID${book.id} R${book.rank}`);
    });
  }
}

async function refreshLostVision() {
  if (lostVisionRequestPending) return;
  lostVisionRequestPending = true;
  try {
    const response = await fetch("/api/demo/vision/lost-items", { cache: "no-store" });
    latestLostVision = await response.json();
  } catch (error) {
    latestLostVision = {
      ok: false,
      available: false,
      active: false,
      detections: [],
      status: "unavailable",
      error: error.message,
    };
  } finally {
    lostVisionRequestPending = false;
    drawCameraOverlay();
    if (latestState) updateAIVisionPanelFromState(latestState);
  }
}

async function refreshBookVision() {
  if (bookVisionRequestPending) return;
  bookVisionRequestPending = true;
  try {
    const response = await fetch("/api/demo/vision/books", { cache: "no-store" });
    latestBookVision = await response.json();
  } catch (_error) {
    latestBookVision = null;
  } finally {
    bookVisionRequestPending = false;
    drawCameraOverlay();
  }
}

refreshLostVision();
setInterval(refreshLostVision, 150);
setInterval(refreshBookVision, 500);


// === environment status refresh ===
function updateEnvironmentStatusPanel() {
  if (latestState && latestState.environment) updateEnvironmentStatus(latestState.environment);
}

setInterval(updateEnvironmentStatusPanel, 2000);
window.addEventListener("DOMContentLoaded", updateEnvironmentStatusPanel);


// === final demo AI vision model panel refresh ===
function updateAIVisionPanelFromState(state) {
  const badge = document.getElementById("ai-vision-status");
  const book = document.getElementById("ai-book-status");
  const lost = document.getElementById("ai-lost-status");
  const hazard = document.getElementById("ai-hazard-status");
  const result = document.getElementById("ai-vision-result");

  if (!badge || !book || !lost || !hazard || !result) {
    return;
  }

  const wo = state && state.work_order ? state.work_order : {};
  const title = String(wo.current_title || wo.title || "");
  const stage = String(wo.stage || "");
  const visionStatus = String(wo.vision_status || "");
  const detectResult = String(wo.result || "");

  badge.classList.remove("is-running", "is-alert");

  book.textContent = "本地 ArUco 已接入";
  lost.textContent = "Qwen-VL 待调用";
  hazard.textContent = "Qwen-VL 待调用";

  if (isBookVisionTask(state)) {
    badge.textContent = "分析中";
    badge.classList.add("is-running");
    book.textContent = "ArUco 图书定位 / 书脊标记识别";
    result.textContent = detectResult
      ? `最近输出：${detectResult}`
      : "最近输出：正在分析书架图像，定位目标图书与错放图书";
    return;
  }

  if (isLostItemTask(state)) {
    const vision = latestLostVision || {};
    const qwen = vision.qwen || {};
    const localModel = vision.local_model || {};
    const detections = Array.isArray(vision.detections) ? vision.detections : [];
    const confirmed = detections.filter((item) => item.state === "confirmed");
    const candidates = detections.filter((item) => item.state !== "confirmed");
    if (!vision.available) {
      badge.textContent = "服务未就绪";
      badge.classList.add("is-alert");
      lost.textContent = "等待遗失物视觉服务";
      result.textContent = `最近输出：${vision.error || "正在连接本地视觉服务"}`;
    } else if (confirmed.length) {
      badge.textContent = "已确认";
      badge.classList.add("is-alert");
      const confirmedLocally = confirmed.some((item) => item.confirmation_source === "local_hog_svm");
      const source = confirmedLocally
        ? "本地 HOG+SVM · 连续3帧"
        : `Qwen-VL${qwen.last_latency_ms ? ` · ${qwen.last_latency_ms}ms` : ""}`;
      lost.textContent = `红框 ${confirmed.length} 个 · ${source}`;
      result.textContent = `最近输出：${confirmed.map((item) => item.message || item.label).join("；")}`;
    } else if (candidates.length) {
      badge.textContent = "确认中";
      badge.classList.add("is-running");
      lost.textContent = localModel.available
        ? `黄色候选框 ${candidates.length} 个 · 本地模型连续帧确认`
        : `黄色候选框 ${candidates.length} 个 · Qwen 异步确认`;
      result.textContent = !localModel.available && qwen.last_error
        ? `最近输出：本地框持续跟踪；${qwen.last_error}`
        : "最近输出：候选框已锁定，正在等待连续3帧本地确认";
    } else {
      badge.textContent = "扫描中";
      badge.classList.add("is-running");
      lost.textContent = localModel.available ? "本地 HOG+SVM · 10 FPS" : "本地候选 + Qwen兜底";
      result.textContent = detectResult
        ? `最近输出：${detectResult}`
        : "最近输出：正在实时扫描地面遗失物";
    }
    return;
  }

  if (title.includes("高危") || stage.includes("HAZARD")) {
    badge.textContent = "告警";
    badge.classList.add("is-alert");
    hazard.textContent = "Qwen-VL 高危场景识别";
    result.textContent = detectResult
      ? `最近输出：${detectResult}`
      : "最近输出：正在识别插座乱接、线缆绊倒、出口阻塞等风险";
    return;
  }

  if (visionStatus && visionStatus !== "-") {
    badge.textContent = "就绪";
    result.textContent = `最近输出：${visionStatus}`;
  } else {
    badge.textContent = "就绪";
    result.textContent = "最近输出：等待巡检任务触发视觉分析";
  }
}
