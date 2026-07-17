#!/usr/bin/env python3
import json
import mimetypes
import os
import sys
import threading
import time
import urllib.error
import urllib.request
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from app.camera_proxy import CameraProxy
from app.command_dispatcher import CommandDispatcher
from app.dht11_reader import Dht11Reader
from app.mission_orchestrator import MissionOrchestrator
from app.navigation_client import NavigationClient
from app.smoke_sensor import SmokeSensor
from app.temperature_risk_monitor import TemperatureRiskMonitor
from app.tts_hs_s77 import HsS77Tts
from app.utils import json_response, load_config, read_json_body
from app.voice_ci1302 import VoiceCi302Stub
from app.work_order import WorkOrderStore
from vision.aruco_book_detector import detect_books_in_jpeg, detect_books_in_jpeg_json


class DemoRuntime:
    def __init__(self):
        self.project_dir = PROJECT_DIR
        self.config_dir = self.project_dir / "config"
        self.paths = load_config(self.config_dir, "paths")
        nav_cfg = self.paths["navigation"]
        endpoints = {
            "map": nav_cfg["map_api"],
            "state": nav_cfg["state_api"],
            "goal": nav_cfg["goal_api"],
            "patrol": nav_cfg["patrol_api"],
            "cancel": nav_cfg["cancel_api"],
            "emergency_stop": nav_cfg["emergency_stop_api"],
            "emergency_release": nav_cfg["emergency_release_api"],
        }
        self.navigation_client = NavigationClient(
            nav_cfg["dashboard_url"],
            endpoints=endpoints,
        )
        self.work_order = WorkOrderStore()
        dht_cfg = dict(self.paths.get("dht11", {}))
        self.dht11 = Dht11Reader(
            dht_cfg.get("iio_root", "/sys/bus/iio/devices"),
            board_host=dht_cfg.get("board_host") or self.paths.get("board", {}).get("ip"),
            board_user=dht_cfg.get("board_user") or self.paths.get("board", {}).get("ssh_user", "root"),
            poll_interval=dht_cfg.get("poll_interval_seconds", 1.5),
        )
        self.voice = VoiceCi302Stub()
        self.tts = HsS77Tts(self.paths.get("tts", {}), self.paths.get("board", {}))
        try:
            self.tts.start()
            self.work_order.add_event("ok", "HS-S77 持久连接已建立", source="voice")
        except Exception as exc:
            raise RuntimeError(f"HS-S77 启动健康检查失败：{exc}") from exc
        smoke_cfg = dict(self.paths.get("smoke_sensor", {}))
        smoke_cfg.setdefault("board_host", self.paths.get("board", {}).get("ip", "192.168.43.192"))
        smoke_cfg.setdefault("board_user", self.paths.get("board", {}).get("ssh_user", "root"))
        self.smoke_sensor = SmokeSensor(smoke_cfg, self.work_order, self.tts)
        self.smoke_sensor.start()
        self.camera = CameraProxy(self.paths.get("camera", {}))
        vision_cfg = self.paths.get("vision", {})
        self.lost_item_api_url = str(
            vision_cfg.get("lost_item_api_url", "http://127.0.0.1:8091/api/vision/lost_items")
        )
        self.camera.start()
        self.orchestrator = MissionOrchestrator(
            self.config_dir,
            self.navigation_client,
            self.work_order,
            self.voice,
            camera=self.camera,
            tts=self.tts,
        )
        self.dispatcher = CommandDispatcher(self.orchestrator, self.work_order)
        self.temperature_monitor = TemperatureRiskMonitor.from_env(
            self.dht11,self.tts,self.work_order,self.dispatcher.snapshot)
        self.running = True
        self.tick_thread = threading.Thread(target=self._tick_loop, daemon=True)
        self.tick_thread.start()

    def _tick_loop(self):
        while self.running:
            self.orchestrator.tick()
            self.temperature_monitor.tick()
            time.sleep(0.3)

    def stop(self):
        self.running = False
        if self.tick_thread:
            self.tick_thread.join(timeout=2)
        self.temperature_monitor.close()
        self.dispatcher.close()
        self.tts.close()
        self.voice.stop()
        self.smoke_sensor.stop()
        self.dht11.close()
        self.camera.stop()

    def state_payload(self):
        orchestration = self.orchestrator.snapshot()
        # Navigation is polled by the background orchestrator tick. Reusing
        # that snapshot keeps browser/voice reads fast when navigation is
        # offline instead of blocking every HTTP request for 1.5 seconds.
        navigation = orchestration.get("navigation") or {
            "available": False,
            "error": self.navigation_client.last_error or "navigation state not ready",
        }
        return {
            "ok": True,
            "camera": self.camera.status(),
            "smoke_sensor": self.smoke_sensor.snapshot(),
            "environment": self.dht11.read(),
            "temperature_alert": self.temperature_monitor.snapshot(),
            "work_order": self.work_order.snapshot(),
            "navigation": navigation,
            "home": orchestration["home"],
            "home_source": orchestration["home_source"],
            "missions": orchestration["missions"],
            "points": orchestration["points"],
            "orchestrator": {
                "active": orchestration["active"],
                "state": orchestration["state"],
            },
            "dispatcher": self.dispatcher.snapshot(),
            "tts": self.tts.status(),
        }


class DemoRequestHandler(BaseHTTPRequestHandler):
    server_version = "LibraryPatrolFinalDemo/0.1"

    @property
    def runtime(self):
        return self.server.runtime

    def log_message(self, _format, *_args):
        return

    def require_control_access(self):
        if self.client_address[0] in ("127.0.0.1", "::1"):
            return
        expected = os.getenv("DEMO_CONTROL_TOKEN", "")
        supplied = self.headers.get("X-Control-Token", "")
        if not expected or supplied != expected:
            raise PermissionError("机器人控制接口仅允许本机或有效控制令牌")

    def send_text(self, text, status=HTTPStatus.OK, content_type="text/plain; charset=utf-8"):
        data = text.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def send_image(self, data, content_type, frame_id=None, frame_timestamp=None):
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        if frame_id is not None:
            self.send_header("X-Frame-ID", str(frame_id))
        if frame_timestamp is not None:
            self.send_header("X-Frame-Timestamp", f"{float(frame_timestamp):.6f}")
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def proxy_lost_item_state(self, query):
        url = self.runtime.lost_item_api_url
        if query:
            url = f"{url}?{query}"
        try:
            with urllib.request.urlopen(url, timeout=0.35) as response:
                payload = json.loads(response.read().decode("utf-8"))
            json_response(self, payload)
        except (OSError, ValueError, urllib.error.URLError) as exc:
            json_response(self, {
                "ok": False,
                "available": False,
                "active": False,
                "status": "unavailable",
                "detections": [],
                "error": f"遗失物视觉服务未就绪：{exc}",
            })

    def serve_static(self, request_path):
        relative = "index.html" if request_path == "/" else unquote(request_path).lstrip("/")
        candidate = (self.runtime.project_dir / "web" / relative).resolve()
        web_root = (self.runtime.project_dir / "web").resolve()
        if not str(candidate).startswith(str(web_root)) or not candidate.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        data = candidate.read_bytes()
        content_type = mimetypes.guess_type(candidate.name)[0] or "application/octet-stream"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        if path in ("/", "/app.js", "/styles.css"):
            self.serve_static(path)
        elif path == "/api/environment":
            json_response(self, self.runtime.dht11.read())
        elif path == "/api/demo/state":
            json_response(self, self.runtime.state_payload())
        elif path == "/api/demo/books":
            books = self.runtime.orchestrator.book_catalog.public_list()
            json_response(self, {"ok": True, "count": len(books), "books": books})
        elif path == "/api/demo/map":
            try:
                json_response(self, self.runtime.navigation_client.get_map())
            except RuntimeError as exc:
                json_response(self, {"ready": False, "error": str(exc)})
        elif path == "/api/demo/vision/lost-items":
            self.proxy_lost_item_state(parsed.query)
        elif path == "/camera_annotated.jpg":
            frame = self.runtime.camera.latest_frame()
            if frame:
                data, frame_id, frame_timestamp = frame
                try:
                    result = detect_books_in_jpeg(data, expected_id=self.runtime.orchestrator.current_expected_book_id())
                    out = result["annotated_jpeg"]
                    self.send_image(out, "image/jpeg", frame_id, frame_timestamp)
                except Exception:
                    self.send_image(data, "image/jpeg", frame_id, frame_timestamp)
            else:
                data = self.runtime.camera.placeholder_image()
                self.send_image(data, "image/svg+xml; charset=utf-8")

        elif path == "/api/demo/vision/books":
            frame = self.runtime.camera.latest_frame()
            if not frame:
                json_response(self, {"ok": False, "error": "camera frame not ready"})
            else:
                data, frame_id, frame_timestamp = frame
                try:
                    result = detect_books_in_jpeg_json(data, expected_id=self.runtime.orchestrator.current_expected_book_id())
                    result["ok"] = True
                    result["frame_id"] = frame_id
                    result["frame_timestamp"] = frame_timestamp
                    json_response(self, result)
                except Exception as exc:
                    json_response(self, {"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)

        elif path == "/camera.jpg":
            frame = self.runtime.camera.latest_frame()
            if frame:
                data, frame_id, frame_timestamp = frame
                self.send_image(data, "image/jpeg", frame_id, frame_timestamp)
            else:
                data = self.runtime.camera.placeholder_image()
                self.send_image(data, "image/svg+xml; charset=utf-8")
        elif path == "/camera.mjpg":
            if self.runtime.camera.enabled:
                self.runtime.camera.write_mjpeg_stream(self)
            else:
                data = self.runtime.camera.placeholder_image()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "image/svg+xml; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(data)
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self):
        path = urlparse(self.path).path
        try:
            self.require_control_access()
            payload = {} if path in (
                "/api/demo/cancel",
                "/api/demo/emergency-stop",
                "/api/demo/emergency-release",
            ) else read_json_body(self)
            if path == "/api/demo/mission":
                mission_id = str(payload.get("mission", ""))
                result=self.runtime.dispatcher.accept("compat_api",{"kind":"mission","mission":mission_id})
                json_response(self, {**result,"mission":mission_id})
            elif path == "/api/demo/find-book":
                book_id = int(payload.get("book_id"))
                book = self.runtime.orchestrator.book_catalog.get(book_id)
                result=self.runtime.dispatcher.accept("compat_api",{"kind":"find_book","book_id":book_id})
                json_response(self, {**result,"book":book})
            elif path == "/api/demo/introduce-book":
                book_id = payload.get("book_id")
                result=self.runtime.dispatcher.accept("compat_api",{"kind":"introduce_book","book_id":book_id})
                json_response(self, {**result,"book_id":book_id})
            elif path == "/api/demo/prompt":
                text=str(payload.get("text","")).strip()
                if not text or len(text)>100: raise ValueError("提示文本为空或过长")
                current=self.runtime.dispatcher.snapshot()
                ok=self.runtime.orchestrator.prompt(text,current["generation"],current["request_id"])
                json_response(self,{"ok":bool(ok),"completed":bool(ok),"text":text,**current})
            elif path == "/api/demo/speak":
                text = str(payload.get("text", "")).strip()
                if not text or len(text) > 300:
                    raise ValueError("播报文本为空或超过 300 字")
                result=self.runtime.dispatcher.execute_now("compat_api",{"kind":"speak","text":text})
                json_response(self,{**result,"text":text})
            elif path == "/api/demo/command/reserve":
                token=self.runtime.dispatcher.reserve(payload.get("source","api"),payload.get("command_type","input"),payload.get("request_id"))
                json_response(self,{"ok":True,"accepted":True,**token})
            elif path == "/api/demo/command":
                token={k:payload.get(k) for k in ("request_id","generation","source","received_at","command_type")}
                result=self.runtime.dispatcher.submit(token,payload.get("command") or {})
                json_response(self,result,status=HTTPStatus.OK if result.get("accepted") else HTTPStatus.CONFLICT)
            elif path == "/api/demo/input":
                token=self.runtime.dispatcher.reserve(payload.get("source","input"),payload.get("command_type","candidate"),payload.get("request_id"))
                json_response(self,{"ok":True,"accepted":True,**token})
            elif path == "/api/demo/cancel":
                json_response(self,self.runtime.dispatcher.accept("compat_api",{"kind":"cancel"}))
            elif path == "/api/demo/emergency-stop":
                json_response(self,self.runtime.dispatcher.emergency_stop("api"))
            elif path == "/api/demo/emergency-release":
                json_response(self,self.runtime.dispatcher.emergency_release("api"))
            elif path == "/api/demo/simulate-voice":
                command = str(payload.get("command", ""))
                self.runtime.voice.simulate_command(command)
                json_response(self, {"ok": True, "command": command})
            else:
                self.send_error(HTTPStatus.NOT_FOUND)
        except PermissionError as exc:
            json_response(self,{"ok":False,"error":str(exc)},status=HTTPStatus.FORBIDDEN)
        except (ValueError, KeyError, TypeError, RuntimeError) as exc:
            json_response(self, {"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)


class DemoHttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, handler_class, runtime):
        super().__init__(address, handler_class)
        self.runtime = runtime


def main():
    runtime = DemoRuntime()
    web_cfg = runtime.paths["demo_web"]
    address = (str(web_cfg.get("host", "0.0.0.0")), int(web_cfg.get("port", 8090)))
    server = DemoHttpServer(address, DemoRequestHandler, runtime)
    print(f"Final demo web: http://127.0.0.1:{address[1]}")
    print("This server does not start navigation, serial, the board camera process, or robot motion.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        runtime.stop()
        server.server_close()


if __name__ == "__main__":
    main()
