import threading

from app.utils import load_config, point_payload, simplify_navigation_state
from vision.aruco_book_detector import detect_books_stub, detect_books_in_jpeg_json
from vision.hazard_detector import detect_hazard_stub
from vision.lost_item_detector import detect_lost_item_stub


class MissionOrchestrator:
    def __init__(self, config_dir, navigation_client, work_order, voice, camera=None):
        self.config_dir = config_dir
        self.nav = navigation_client
        self.work_order = work_order
        self.voice = voice
        self.camera = camera
        self.paths = load_config(config_dir, "paths")
        self.points_config = load_config(config_dir, "points")
        self.missions_config = load_config(config_dir, "missions")
        self.voice_config = load_config(config_dir, "voice_frames")
        self.points = dict(self.points_config.get("points", {}))
        self.missions = dict(self.missions_config.get("missions", {}))
        self.commands = dict(self.voice_config.get("commands", {}))
        self.speeches = dict(self.voice_config.get("speeches", {}))
        self.lock = threading.RLock()
        self.active = None
        self.sequence_index = 0
        self.waiting_for_nav = False
        self.waiting_point_id = ""
        self.waiting_arrive_event = ""
        self.already_triggered_events = set()
        self.last_patrol_index = None
        self.last_nav_state = {}
        self.start_home = None
        self.home_source = "起点未记录"
        self.record_start_home_from_nav_state()
        self.voice.start(self.handle_voice_command)

    def record_start_home_from_nav_state(self):
        try:
            state = self.nav.get_state()
        except RuntimeError as exc:
            self.work_order.set_navigation_status(f"导航未连接：{exc}")
            return None
        robot = state.get("robot")
        if isinstance(robot, dict) and all(key in robot for key in ("x", "y")):
            self.start_home = {
                "name": "启动时起点",
                "x": float(robot["x"]),
                "y": float(robot["y"]),
                "yaw": float(robot.get("yaw", 0.0)),
            }
            self.home_source = "已从导航状态记录 START_HOME"
            self.work_order.add_event("ok", "已记录启动起点 START_HOME", source="navigation")
        else:
            self.home_source = "起点未记录"
            self.work_order.add_event("warn", "导航状态中暂无机器人位姿，起点未记录", source="navigation")
        return self.start_home

    def mission_list(self):
        return [
            {
                "id": mission_id,
                "title": mission.get("title", mission_id),
                "type": mission.get("type", "unknown"),
            }
            for mission_id, mission in self.missions.items()
        ]

    def point_by_id(self, point_id):
        if point_id == "START_HOME":
            if not self.start_home:
                raise ValueError("START_HOME has not been recorded from navigation state")
            return point_payload(point_id, self.start_home)
        point = self.points.get(point_id)
        if point is None:
            raise KeyError(f"unknown point: {point_id}")
        return point_payload(point_id, point)

    def route_points(self, route):
        return [self.point_by_id(point_id) for point_id in route]

    def speak(self, speech_id):
        speech = self.speeches.get(speech_id, {})
        text = speech.get("text", speech_id)
        frame = speech.get("frame", "")
        self.work_order.set_voice_status(f"播报占位：{text}")
        self.work_order.add_event(
            "info",
            f"已触发播报占位 {speech_id} {frame}".strip(),
            source="voice",
        )

    def handle_voice_command(self, command_id):
        command = self.commands.get(str(command_id))
        if not command:
            self.work_order.add_event("warn", f"未知模拟语音命令：{command_id}", source="voice")
            return
        self.work_order.set_voice_status(f"收到模拟语音：{command.get('name', command_id)}")
        self.work_order.add_event(
            "ok",
            f"模拟语音 {command_id} -> {command.get('mission')}",
            source="voice",
        )
        self.start_mission(command.get("mission"))

    def start_mission(self, mission_id):
        with self.lock:
            mission = self.missions.get(mission_id)
            if not mission:
                self.work_order.set_error(f"未知任务：{mission_id}")
                return False
            self.cancel(call_navigation=False)
            self.work_order.set_task(mission_id, mission.get("title", mission_id))
            self.work_order.clear_error()
            self.active = {
                "id": mission_id,
                "mission": mission,
                "state": "starting",
            }
            self.sequence_index = 0
            self.waiting_for_nav = False
            self.waiting_point_id = ""
            self.waiting_arrive_event = ""
            self.already_triggered_events = set()
            self.last_patrol_index = None
            self.work_order.set_stage("任务已启动")
            self._start_active_locked()
            return True

    def _start_active_locked(self):
        mission = self.active["mission"]
        mission_type = mission.get("type")
        if mission_type == "simple_speech":
            self.speak(mission.get("speech", ""))
            self.work_order.set_stage("播报任务完成")
            self.active["state"] = "completed"
        elif mission_type == "goto":
            self._start_goto_locked(mission.get("point"), "ARRIVED:" + str(mission.get("point")))
        elif mission_type == "cancel":
            self.cancel()
        elif mission_type == "sequence":
            self.active["state"] = "sequence"
            self._advance_sequence_locked()
        elif mission_type == "patrol":
            self._start_patrol_locked(mission)
        else:
            self.work_order.set_error(f"不支持的任务类型：{mission_type}")
            self.active["state"] = "failed"

    def _start_goto_locked(self, point_id, arrive_event=""):
        try:
            point = self.point_by_id(point_id)
            self.nav.send_goal(point)
        except (RuntimeError, KeyError, ValueError) as exc:
            self.work_order.set_error(f"下发目标失败：{exc}")
            if self.active:
                self.active["state"] = "failed"
            return
        self.waiting_for_nav = True
        self.waiting_point_id = point_id
        self.waiting_arrive_event = arrive_event
        self.work_order.set_stage(f"前往 {point.get('name', point_id)}")
        self.work_order.set_navigation_status(f"已下发目标点：{point_id}")
        if self.active:
            self.active["state"] = "waiting_goto"

    def _start_patrol_locked(self, mission):
        try:
            waypoints = self.route_points(mission.get("route", []))
            self.nav.start_patrol(waypoints)
        except (RuntimeError, KeyError, ValueError) as exc:
            self.work_order.set_error(f"启动巡检失败：{exc}")
            self.active["state"] = "failed"
            return
        if mission.get("start_speech"):
            self.speak(mission["start_speech"])
        self.active["state"] = "patrol"
        self.last_patrol_index = 0
        self.work_order.set_stage(f"巡检路线已启动，共 {len(waypoints)} 个点")
        self.work_order.set_navigation_status("巡检任务已下发")

    def _advance_sequence_locked(self):
        mission = self.active["mission"]
        steps = mission.get("steps", [])
        while self.sequence_index < len(steps):
            step = steps[self.sequence_index]
            action = step.get("action")
            self.sequence_index += 1
            if action == "speech":
                self.speak(step.get("speech", ""))
            elif action == "vision_stub":
                self._run_vision_stub(step)
            elif action == "goto":
                self._start_goto_locked(step.get("point"), step.get("arrive_event", ""))
                return
            else:
                self.work_order.add_event("warn", f"跳过未知步骤：{action}", source="mission")
        self.work_order.set_stage("序列任务完成")
        self.active["state"] = "completed"

    def _run_vision_stub(self, step_or_event):
        mode = step_or_event.get("mode") or step_or_event.get("vision_mode")
        if mode == "aruco_book":
            expected_id = int(step_or_event.get("expected_book_id", 203))
            result = None

            if self.camera is not None:
                frame = self.camera.latest_jpeg()
                if frame:
                    try:
                        result = detect_books_in_jpeg_json(frame, expected_id=expected_id)
                    except Exception as exc:
                        self.work_order.add_event("warn", f"真实图书识别失败，使用占位结果：{exc}", source="vision")

            if result is None:
                result = detect_books_stub()

            text = result.get("message", step_or_event.get("result_text", "图书识别完成"))
        elif mode == "lost":
            result = detect_lost_item_stub()
            text = result.get("speech", "遗失物检测占位完成")
        elif mode in ("socket", "fire", "exit"):
            result = detect_hazard_stub(mode)
            text = result.get("message", "高危点位检测占位完成")
        else:
            result = {"stub": True, "mode": mode}
            text = step_or_event.get("result_text", "视觉占位检测完成")
        self.work_order.set_vision_status(f"视觉占位完成：{mode}")
        self.work_order.set_result(text)
        return result

    def cancel(self, call_navigation=True):
        with self.lock:
            if call_navigation:
                try:
                    self.nav.cancel()
                except RuntimeError as exc:
                    self.work_order.add_event("warn", f"导航取消接口不可用：{exc}", source="navigation")
            self.active = None
            self.sequence_index = 0
            self.waiting_for_nav = False
            self.waiting_point_id = ""
            self.waiting_arrive_event = ""
            self.work_order.set_stage("任务已取消" if call_navigation else "空闲")
            if call_navigation:
                self.work_order.add_event("warn", "已请求取消当前任务", source="mission")

    def tick(self):
        with self.lock:
            try:
                state = self.nav.get_state()
                self.last_nav_state = simplify_navigation_state(state)
                nav_state = self.last_nav_state.get("navigation", {}).get("state", "unknown")
                patrol_state = self.last_nav_state.get("patrol", {}).get("state", "idle")
                self.work_order.set_navigation_status(
                    f"navigation={nav_state}, patrol={patrol_state}"
                )
                if self.start_home is None:
                    self._record_home_from_cached_state(state)
            except RuntimeError as exc:
                self.last_nav_state = {"available": False, "error": str(exc)}
                self.work_order.set_navigation_status(f"导航未连接：{exc}")
                return
            if not self.active:
                return
            active_state = self.active.get("state")
            # 任务已经完成后，只持续更新导航状态文本，不再把后续导航 failed
            # 写入当前工单错误，避免视频展示时残留“导航失败”。
            if active_state == "completed":
                return
            if active_state == "failed":
                return
            if active_state == "waiting_goto":
                self._tick_goto_locked()
            elif active_state == "patrol":
                self._tick_patrol_locked()

    def _record_home_from_cached_state(self, state):
        robot = state.get("robot")
        if isinstance(robot, dict) and all(key in robot for key in ("x", "y")):
            self.start_home = {
                "name": "启动时起点",
                "x": float(robot["x"]),
                "y": float(robot["y"]),
                "yaw": float(robot.get("yaw", 0.0)),
            }
            self.home_source = "已从导航状态记录 START_HOME"
            self.work_order.add_event("ok", "已自动补记 START_HOME", source="navigation")

    def _tick_goto_locked(self):
        nav_state = self.last_nav_state.get("navigation", {}).get("state")
        if nav_state == "reached":
            point_id = self.waiting_point_id
            event = self.waiting_arrive_event or f"ARRIVED:{point_id}"
            key = f"goto:{point_id}:{event}"
            if key not in self.already_triggered_events:
                self.already_triggered_events.add(key)
                self.work_order.add_event("ok", event, source="navigation")
                self.work_order.set_stage(f"已到达 {point_id}")
            self.waiting_for_nav = False
            self.waiting_point_id = ""
            if self.active and self.active["mission"].get("type") == "sequence":
                self._advance_sequence_locked()
            elif self.active:
                self.active["state"] = "completed"
                self.work_order.set_stage("目标点任务完成")
        elif nav_state == "failed":
            if self.active and self.active.get("state") != "completed":
                self.work_order.set_error("导航失败")
                self.active["state"] = "failed"

    def _tick_patrol_locked(self):
        patrol = self.last_nav_state.get("patrol", {})
        nav_state = self.last_nav_state.get("navigation", {}).get("state")
        route = self.active["mission"].get("route", [])
        index = patrol.get("index")
        if isinstance(index, int) and 0 <= index < len(route):
            self.last_patrol_index = index
            if nav_state == "reached":
                self._trigger_point_event_locked(route[index])
        if patrol.get("state") == "completed":
            self.work_order.set_stage("ROUTE_DONE:" + self.active["id"])
            self.work_order.add_event("ok", "路线完成", source="navigation")
            self.active["state"] = "completed"
        elif patrol.get("state") in ("failed", "cancelled", "emergency_stopped"):
            if self.active and self.active.get("state") != "completed":
                self.work_order.set_error(f"巡检异常：{patrol.get('state')}")
                self.active["state"] = "failed"

    def _trigger_point_event_locked(self, point_id):
        if point_id in self.already_triggered_events:
            return
        self.already_triggered_events.add(point_id)
        point_events = self.active["mission"].get("point_events", {})
        event = point_events.get(point_id)
        self.work_order.add_event("ok", f"ARRIVED:{point_id}", source="navigation")
        if not event:
            return
        if event.get("vision_mode"):
            self._run_vision_stub(event)
        if event.get("work_order"):
            self.work_order.set_result(event["work_order"])
        if event.get("speech"):
            self.speak(event["speech"])

    def snapshot(self):
        with self.lock:
            return {
                "active": self.active["id"] if self.active else None,
                "state": self.active.get("state") if self.active else "idle",
                "home": self.start_home,
                "home_source": self.home_source,
                "missions": self.mission_list(),
                "points": self.points,
                "navigation": self.last_nav_state,
            }
