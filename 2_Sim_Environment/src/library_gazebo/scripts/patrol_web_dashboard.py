#!/usr/bin/env python3
import json
import threading
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from patrol_knowledge import load_library_map


HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Library Patrol 中控 UI</title>
  <style>
    :root {
      color-scheme: light;
      --ink: #17202a;
      --muted: #667085;
      --line: #d7dde5;
      --paper: #f7f8fb;
      --panel: #ffffff;
      --accent: #1f7a8c;
      --warn: #c43d3d;
      --info: #2c5aa0;
      --ok: #317a45;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Noto Sans CJK SC", "Microsoft YaHei", Arial, sans-serif;
      color: var(--ink);
      background: var(--paper);
    }
    header {
      height: 56px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding: 0 20px;
      background: #20313f;
      color: white;
      border-bottom: 4px solid #d69c2f;
    }
    header h1 { margin: 0; font-size: 20px; font-weight: 700; letter-spacing: 0; }
    header .meta { font-size: 13px; opacity: .85; }
    main {
      display: grid;
      grid-template-columns: minmax(520px, 1fr) 360px;
      gap: 14px;
      padding: 14px;
      min-height: calc(100vh - 56px);
    }
    section, aside {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 6px;
      overflow: hidden;
    }
    .map-head, .side-head {
      display: flex;
      align-items: center;
      justify-content: space-between;
      min-height: 44px;
      padding: 10px 12px;
      border-bottom: 1px solid var(--line);
      background: #fbfcfe;
    }
    h2 { margin: 0; font-size: 15px; letter-spacing: 0; }
    #mapCanvas {
      width: 100%;
      height: calc(100vh - 128px);
      min-height: 520px;
      display: block;
      background: #eef2f6;
    }
    aside {
      display: grid;
      grid-template-rows: auto auto auto 1fr;
      min-height: calc(100vh - 84px);
    }
    .status {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
      padding: 12px;
      border-bottom: 1px solid var(--line);
    }
    .metric {
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 10px;
      min-height: 68px;
    }
    .metric span { display: block; color: var(--muted); font-size: 12px; margin-bottom: 6px; }
    .metric strong { display: block; font-size: 18px; letter-spacing: 0; }
    .controls {
      padding: 12px;
      border-bottom: 1px solid var(--line);
    }
    .controls h3, .events h3 { margin: 0 0 10px 0; font-size: 14px; }
    .area-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
    }
    button {
      min-height: 42px;
      border: 1px solid #b7c6d3;
      border-radius: 6px;
      background: #f6fbfd;
      color: var(--ink);
      font-weight: 700;
      cursor: pointer;
    }
    button:hover { border-color: var(--accent); color: var(--accent); }
    .legend {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 6px 10px;
      padding: 12px;
      border-bottom: 1px solid var(--line);
      color: var(--muted);
      font-size: 13px;
    }
    .dot { display: inline-block; width: 10px; height: 10px; border-radius: 50%; margin-right: 6px; }
    .events { padding: 12px; overflow: auto; }
    .event {
      border-left: 4px solid var(--accent);
      padding: 8px 8px 8px 10px;
      margin-bottom: 8px;
      background: #fbfcfe;
    }
    .event.warn { border-left-color: var(--warn); }
    .event.info { border-left-color: var(--info); }
    .event.ok { border-left-color: var(--ok); }
    .event .time { color: var(--muted); font-size: 12px; margin-bottom: 3px; }
    .event .msg { font-size: 13px; line-height: 1.4; }
    @media (max-width: 920px) {
      main { grid-template-columns: 1fr; }
      #mapCanvas { height: 560px; min-height: 420px; }
      aside { min-height: auto; }
    }
  </style>
</head>
<body>
  <header>
    <h1>Library Patrol 中控 UI</h1>
    <div class="meta" id="connection">等待 ROS 数据</div>
  </header>
  <main>
    <section>
      <div class="map-head">
        <h2>仿真图书馆业务地图</h2>
        <span id="mapName"></span>
      </div>
      <canvas id="mapCanvas"></canvas>
    </section>
    <aside>
      <div class="side-head"><h2>机器人状态</h2><span id="stamp">--:--:--</span></div>
      <div class="status">
        <div class="metric"><span>当前位置</span><strong id="pose">等待 /odom</strong></div>
        <div class="metric"><span>目标点</span><strong id="goal">等待目标</strong></div>
      </div>
      <div class="controls">
        <h3>区域导航</h3>
        <div class="area-grid" id="areaButtons"></div>
      </div>
      <div class="legend">
        <div><span class="dot" style="background:#1f7a8c"></span>机器人</div>
        <div><span class="dot" style="background:#d69c2f"></span>目标</div>
        <div><span class="dot" style="background:#c43d3d"></span>隐患</div>
        <div><span class="dot" style="background:#2c5aa0"></span>失物</div>
      </div>
      <div class="events">
        <h3>业务事件 / 工单</h3>
        <div id="events"></div>
      </div>
    </aside>
  </main>
  <script>
    const canvas = document.getElementById("mapCanvas");
    const ctx = canvas.getContext("2d");
    let state = null;

    function resizeCanvas() {
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.floor(rect.width * dpr);
      canvas.height = Math.floor(rect.height * dpr);
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      draw();
    }

    function worldToCanvas(x, y) {
      const b = state.map.layout.bounds;
      const w = canvas.getBoundingClientRect().width;
      const h = canvas.getBoundingClientRect().height;
      const pad = 36;
      const sx = (w - pad * 2) / (b.max_x - b.min_x);
      const sy = (h - pad * 2) / (b.max_y - b.min_y);
      const s = Math.min(sx, sy);
      const ox = (w - s * (b.max_x - b.min_x)) / 2;
      const oy = (h - s * (b.max_y - b.min_y)) / 2;
      return { x: ox + (x - b.min_x) * s, y: oy + (b.max_y - y) * s, s };
    }

    function rectObj(obj) {
      const c = worldToCanvas(obj.x, obj.y);
      ctx.fillRect(c.x - obj.width * c.s / 2, c.y - obj.height * c.s / 2, obj.width * c.s, obj.height * c.s);
      ctx.strokeRect(c.x - obj.width * c.s / 2, c.y - obj.height * c.s / 2, obj.width * c.s, obj.height * c.s);
    }

    function label(text, x, y) {
      ctx.font = "12px Arial";
      ctx.fillStyle = "#17202a";
      ctx.fillText(text, x + 7, y - 7);
    }

    function circle(x, y, r, color) {
      ctx.beginPath();
      ctx.arc(x, y, r, 0, Math.PI * 2);
      ctx.fillStyle = color;
      ctx.fill();
      ctx.lineWidth = 2;
      ctx.strokeStyle = "#fff";
      ctx.stroke();
    }

    function draw() {
      const rect = canvas.getBoundingClientRect();
      ctx.clearRect(0, 0, rect.width, rect.height);
      if (!state) return;
      const layout = state.map.layout;

      ctx.fillStyle = "#eef2f6";
      ctx.fillRect(0, 0, rect.width, rect.height);

      ctx.fillStyle = "#d8dee6";
      ctx.strokeStyle = "#aab4c0";
      layout.walls.forEach(rectObj);

      layout.obstacles.forEach(o => {
        ctx.fillStyle = o.kind === "bookshelf" ? "#9b6b43" : (o.kind === "desk" ? "#607d8b" : "#b18b5c");
        ctx.strokeStyle = "#594838";
        rectObj(o);
        const p = worldToCanvas(o.x, o.y);
        label(o.label, p.x, p.y);
      });

      ctx.strokeStyle = "#8596a8";
      ctx.setLineDash([6, 5]);
      ctx.beginPath();
      state.route.forEach((area, idx) => {
        const p = worldToCanvas(area.x, area.y);
        if (idx === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
      });
      ctx.stroke();
      ctx.setLineDash([]);

      state.map.areas.forEach(area => {
        const p = worldToCanvas(area.x, area.y);
        circle(p.x, p.y, 8, "#d69c2f");
        label(area.name, p.x, p.y);
      });

      layout.objects.forEach(o => {
        const p = worldToCanvas(o.x, o.y);
        circle(p.x, p.y, 7, o.kind === "hazard" ? "#c43d3d" : "#2c5aa0");
        label(o.label, p.x, p.y);
      });

      state.event_markers.forEach((e, idx) => {
        const p = worldToCanvas(e.x, e.y);
        const color = e.type === "safety_hazard" ? "#c43d3d" : (e.type === "lost_found_ticket" ? "#2c5aa0" : "#317a45");
        ctx.save();
        ctx.shadowColor = "rgba(0,0,0,.25)";
        ctx.shadowBlur = 8;
        circle(p.x, p.y, 11, color);
        ctx.restore();
        ctx.fillStyle = "#fff";
        ctx.font = "bold 11px Arial";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(String(state.event_markers.length - idx), p.x, p.y);
        ctx.textAlign = "start";
        ctx.textBaseline = "alphabetic";
        label(e.type === "safety_hazard" ? "隐患事件" : (e.type === "lost_found_ticket" ? "失物工单" : "业务事件"), p.x + 10, p.y);
      });

      if (state.goal) {
        const g = worldToCanvas(state.goal.x, state.goal.y);
        ctx.strokeStyle = "#d69c2f";
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(g.x - 10, g.y);
        ctx.lineTo(g.x + 10, g.y);
        ctx.moveTo(g.x, g.y - 10);
        ctx.lineTo(g.x, g.y + 10);
        ctx.stroke();
      }

      if (state.pose) {
        const p = worldToCanvas(state.pose.x, state.pose.y);
        circle(p.x, p.y, 9, "#1f7a8c");
        label("patrol_robot", p.x, p.y);
      }
    }

    function renderSide() {
      document.getElementById("connection").textContent = state ? "ROS 数据已连接" : "等待 ROS 数据";
      document.getElementById("mapName").textContent = state?.map?.layout?.name || "";
      document.getElementById("stamp").textContent = new Date().toLocaleTimeString();
      document.getElementById("pose").textContent = state?.pose ? `x=${state.pose.x.toFixed(2)}, y=${state.pose.y.toFixed(2)}` : "等待 /odom";
      document.getElementById("goal").textContent = state?.goal ? `x=${state.goal.x.toFixed(2)}, y=${state.goal.y.toFixed(2)}` : "等待目标";

      const buttons = document.getElementById("areaButtons");
      buttons.innerHTML = "";
      state.map.areas.forEach(area => {
        const btn = document.createElement("button");
        btn.textContent = area.name;
        btn.title = area.label;
        btn.onclick = () => sendGoal(area.id);
        buttons.appendChild(btn);
      });

      const events = document.getElementById("events");
      events.innerHTML = "";
      if (state.events.length === 0) {
        events.textContent = "暂无事件。";
        return;
      }
      state.events.forEach(e => {
        const div = document.createElement("div");
        div.className = "event " + (e.type === "safety_hazard" ? "warn" : (e.type === "lost_found_ticket" ? "info" : "ok"));
        const loc = Number.isFinite(e.x) && Number.isFinite(e.y) ? ` @ (${e.x.toFixed(2)}, ${e.y.toFixed(2)})` : "";
        div.innerHTML = `<div class="time">${e.received_at || ""} ${e.type || ""}${loc}</div><div class="msg">${e.message || ""}</div>`;
        events.appendChild(div);
      });
    }

    async function sendGoal(areaId) {
      await fetch("/api/goal", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({area_id: areaId})
      });
      await poll();
    }

    async function poll() {
      const res = await fetch("/api/state", {cache: "no-store"});
      state = await res.json();
      renderSide();
      draw();
    }

    window.addEventListener("resize", resizeCanvas);
    resizeCanvas();
    setInterval(poll, 1000);
    poll();
  </script>
</body>
</html>
"""


class PatrolWebDashboard(Node):
    def __init__(self):
        super().__init__("patrol_web_dashboard")
        self.declare_parameter("port", 8080)
        self.port = int(self.get_parameter("port").value)
        self.map_data = load_library_map()
        self.area_by_id = {area["id"]: area for area in self.map_data.get("areas", [])}
        self.route = [self.area_by_id[area_id] for area_id in self.map_data.get("patrol_route", []) if area_id in self.area_by_id]
        self.events = []
        self.event_markers = []
        self.pose = None
        self.goal = None
        self.lock = threading.Lock()
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.status_pub = self.create_publisher(String, "/patrol/mission_status", 10)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_subscription(String, "/patrol/events", self.event_callback, 10)
        self.create_subscription(String, "/patrol/mission_status", self.status_callback, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.server = self._start_server()
        self.get_logger().info(f"Web 中控 UI 已启动: http://localhost:{self.port}")

    def _start_server(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                path = urlparse(self.path).path
                if path == "/":
                    self._send(200, HTML.encode("utf-8"), "text/html; charset=utf-8")
                elif path == "/api/state":
                    self._send(200, json.dumps(node.snapshot(), ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")
                else:
                    self._send(404, b"not found", "text/plain")

            def do_POST(self):
                path = urlparse(self.path).path
                if path != "/api/goal":
                    self._send(404, b"not found", "text/plain")
                    return
                length = int(self.headers.get("Content-Length", "0"))
                payload = json.loads(self.rfile.read(length).decode("utf-8") or "{}")
                ok, message = node.handle_goal_request(payload)
                code = 200 if ok else 400
                self._send(code, json.dumps({"ok": ok, "message": message}, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")

            def _send(self, code, body, content_type):
                self.send_response(code)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, format, *args):
                return

        server = ThreadingHTTPServer(("0.0.0.0", self.port), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        return server

    def event_callback(self, msg):
        self._store_event(msg.data)

    def status_callback(self, msg):
        self._store_event(msg.data)

    def _store_event(self, data):
        try:
            event = json.loads(data)
        except json.JSONDecodeError:
            event = {"type": "raw", "message": data}
        event["received_at"] = datetime.now().strftime("%H:%M:%S")
        with self.lock:
            self.events.insert(0, event)
            self.events = self.events[:30]
            if self._is_marker_event(event):
                self.event_markers.insert(0, event)
                self.event_markers = self.event_markers[:20]

    def _is_marker_event(self, event):
        if event.get("type") not in {"safety_hazard", "lost_found_ticket"}:
            return False
        return isinstance(event.get("x"), (int, float)) and isinstance(event.get("y"), (int, float))

    def odom_callback(self, msg):
        with self.lock:
            self.pose = {"x": msg.pose.pose.position.x, "y": msg.pose.pose.position.y}

    def goal_callback(self, msg):
        with self.lock:
            self.goal = {"x": msg.pose.position.x, "y": msg.pose.position.y}

    def handle_goal_request(self, payload):
        area = self.area_by_id.get(payload.get("area_id"))
        if area is None:
            return False, "未知区域"
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = float(area["x"])
        goal_msg.pose.position.y = float(area["y"])
        goal_msg.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_msg)
        self.goal_callback(goal_msg)

        status = String()
        status.data = json.dumps({
            "type": "ui_goal",
            "message": f"中控 UI 下发导航目标：{area['name']} {area['label']}",
            "x": float(area["x"]),
            "y": float(area["y"]),
            "area_id": area["id"],
        }, ensure_ascii=False)
        self.status_pub.publish(status)

        if self.nav_client.server_is_ready():
            goal = NavigateToPose.Goal()
            goal.pose = goal_msg
            self.nav_client.send_goal_async(goal)
            return True, "目标已通过 Nav2 action 下发"
        return True, "目标已发布到 /goal_pose，Nav2 action server 暂未就绪"

    def snapshot(self):
        with self.lock:
            return {
                "map": self.map_data,
                "route": self.route,
                "pose": self.pose,
                "goal": self.goal,
                "events": list(self.events),
                "event_markers": list(self.event_markers),
            }

    def destroy_node(self):
        if hasattr(self, "server"):
            self.server.shutdown()
            self.server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PatrolWebDashboard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
