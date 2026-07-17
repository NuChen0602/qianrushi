import threading
import time

from app.book_catalog import BookCatalog
from app.utils import load_config, point_payload, simplify_navigation_state
from vision.aruco_book_detector import (
    detect_books_in_jpeg_json,
    misplaced_books_for_shelf,
    misplaced_books_message,
    shelf_id_for_point,
)


class MissionOrchestrator:
    """Token-aware mission executor. No external I/O is performed under _lock."""

    def __init__(self, config_dir, navigation_client, work_order, voice, camera=None, tts=None):
        self.config_dir = config_dir; self.nav = navigation_client; self.work_order = work_order
        self.voice = voice; self.camera = camera; self.tts = tts
        self.book_catalog = BookCatalog(config_dir)
        self.paths = load_config(config_dir, "paths")
        self.points = dict(load_config(config_dir, "points").get("points", {}))
        self.missions = dict(load_config(config_dir, "missions").get("missions", {}))
        self.speeches = dict(load_config(config_dir, "voice_frames").get("speeches", {}))
        vision = self.paths.get("vision", {})
        self.vision_timeout = float(vision.get("book_timeout_seconds", 8.0))
        self.vision_confirm_frames = int(vision.get("book_confirm_frames", 3))
        self.shelf_vision_timeout = float(
            vision.get("shelf_timeout_seconds", 3.0))
        self.shelf_vision_confirm_frames = max(
            1, int(vision.get("shelf_confirm_frames", 2)))
        navigation = self.paths.get("navigation", {})
        self.goal_timeout = float(navigation.get("goal_timeout_seconds", 180))
        self.patrol_timeout = float(navigation.get("patrol_timeout_seconds", 900))
        self._lock = threading.RLock(); self.active = None; self.last_nav_state = {}
        self._preempt_generation = None
        self.last_book_context = None; self.last_requested_book_id = None; self.last_detected_book_id = None
        self.start_home = None; self.home_source = "起点未记录"; self.emergency_stopped = False
        self._token_checker = lambda _g, _r: True
        self._record_home_initial(); self.voice.start(self.handle_voice_command)
        self.work_order.add_event("ok", f"固定图书介绍已加载：{self.book_catalog.source_path}", source="books")

    def set_token_checker(self, checker): self._token_checker = checker
    def _current(self, generation, request_id): return bool(self._token_checker(int(generation), str(request_id)))

    def _record_home_initial(self):
        try: state = self.nav.get_state()
        except RuntimeError as exc:
            self.work_order.set_navigation_status(f"导航未连接：{exc}"); return
        robot = state.get("robot")
        if isinstance(robot, dict) and "x" in robot and "y" in robot:
            self.start_home = {"name":"启动时起点","x":float(robot["x"]),"y":float(robot["y"]),"yaw":float(robot.get("yaw",0))}
            self.home_source = "已从导航状态记录 START_HOME"

    def mission_list(self):
        return [{"id":key,"title":v.get("title",key),"type":v.get("type","unknown")} for key,v in self.missions.items()]

    def point_by_id(self, point_id):
        if point_id == "START_HOME":
            if not self.start_home: raise ValueError("START_HOME尚未记录")
            return point_payload(point_id, self.start_home)
        if point_id not in self.points: raise KeyError(f"未知导航点：{point_id}")
        return point_payload(point_id, self.points[point_id])

    def _set_book_context(self, book):
        if not book: return
        with self._lock:
            self.last_book_context = dict(book); self.last_requested_book_id = int(book["id"])

    def current_target_book(self):
        with self._lock:
            return dict(self.last_book_context) if self.last_book_context else None

    def _speak(self, text, generation=0, request_id="", source="voice"):
        if generation and not self._current(generation, request_id): return False
        try:
            played = bool(self.tts and self.tts.speak(text, request_id=request_id, generation=generation))
        except Exception as exc:
            if not generation or self._current(generation, request_id):
                self.work_order.set_voice_status(f"HS-S77错误：{exc}")
                self.work_order.add_event("error", f"HS-S77播报失败：{exc}", source=source)
            return False
        if played and (not generation or self._current(generation, request_id)):
            self.work_order.set_voice_status("HS-S77播放完成：" + str(text))
            self.work_order.add_event("info", "播报完成：" + str(text), source=source)
        return played

    def prompt(self, text, generation=0, request_id=""):
        """Non-preemptive dialogue prompt; never changes mission or book context."""
        return self._speak(text, generation, request_id, source="prompt")

    @staticmethod
    def _cancel_confirmed(response):
        if not isinstance(response, dict) or response.get("ok") is False:
            return False
        state=str(response.get("state","")).lower()
        return state in ("cancelled","canceled","idle","emergency_stopped") or response.get("cancelled") is True

    def _escalate_cancel_failure(self, reason, generation, request_id):
        try:
            response=self.nav.emergency_stop()
            if isinstance(response,dict) and response.get("ok") is False:
                raise RuntimeError(response.get("error") or response.get("reason") or "急停被拒绝")
        except RuntimeError as exc:
            self.work_order.set_error(f"取消无法确认且急停失败：{reason}；{exc}")
            raise RuntimeError(f"取消无法确认且急停失败：{reason}；{exc}") from exc
        with self._lock:
            self.active=None; self.emergency_stopped=True
        self.work_order.mark_idle("取消无法确认，已升级紧急停止")
        self.work_order.add_event("error",f"普通取消失败，已升级紧急停止：{reason}",source="safety",
                                  request_id=request_id,generation=generation)
        return False

    def _preempt(self, generation, request_id, announce=False):
        with self._lock:
            self.active = None
            emergency_stopped=self.emergency_stopped
            already_preempted = bool(generation) and self._preempt_generation == int(generation)
            if generation:
                self._preempt_generation = int(generation)
        if self.tts: self.tts.cancel_pending(generation)
        if emergency_stopped:
            self.work_order.mark_idle("紧急停止")
            return False
        if already_preempted:
            return True
        try:
            response=self.nav.cancel()
        except RuntimeError as exc:
            return self._escalate_cancel_failure(str(exc),generation,request_id)
        if not self._cancel_confirmed(response):
            return self._escalate_cancel_failure(f"取消响应无法确认停车：{response}",generation,request_id)
        if self._current(generation, request_id):
            self.work_order.mark_idle("任务已取消" if announce else "空闲")
        return True

    def cancel(self, generation=0, request_id="", announce=True):
        return self._preempt(generation, request_id, announce=announce)

    def invalidate(self, generation, request_id):
        """Invalidate older work, cancelling navigation only when we own motion.

        ASR reserves its latest-wins token before the utterance is classified.
        An idle dialogue reservation must therefore not require the navigation
        service to be running.  If this orchestrator owns a moving goal or
        patrol, retain the existing confirmed-cancel/emergency-stop path.
        """
        with self._lock:
            active_state = str((self.active or {}).get("state", ""))
        if active_state in ("waiting_goto", "patrol"):
            return self._preempt(generation, request_id, announce=False)
        with self._lock:
            self.active = None
            if generation:
                self._preempt_generation = int(generation)
            emergency_stopped = self.emergency_stopped
        if self.tts:
            self.tts.cancel_pending(generation)
        if emergency_stopped:
            self.work_order.mark_idle("紧急停止")
            return False
        if self._current(generation, request_id):
            self.work_order.mark_idle("空闲")
        return True

    def emergency_stop(self, generation, request_id):
        with self._lock: self.active=None; self.emergency_stopped=True
        self._preempt(generation, request_id, announce=False)
        errors=[]
        try: self.nav.emergency_stop()
        except RuntimeError as exc: errors.append(str(exc))
        self.work_order.mark_idle("紧急停止")
        self.work_order.add_event("error", "机器人已紧急停止", source="safety")
        if errors: raise RuntimeError("；".join(errors))
        return True

    def emergency_release(self, generation, request_id):
        self.nav.emergency_release()
        with self._lock: self.emergency_stopped=False; self.active=None
        self.work_order.mark_idle("急停已解除，保持空闲")
        return True

    def introduce_book(self, book_id, generation=0, request_id=""):
        book = self.book_catalog.get(book_id) if book_id is not None else self.current_target_book()
        if book: self._set_book_context(book)
        if not self._preempt(generation, request_id): return False
        if not self._current(generation, request_id): return False
        if not book:
            self._speak("请再说一下想介绍的书名。", generation, request_id); return False
        self.work_order.set_task("INTRO_BOOK", f"介绍《{book['name']}》")
        self.work_order.set_stage("播报图书介绍")
        if not self._speak("收到。", generation, request_id): return False
        if not self._speak(book["summary"], generation, request_id): return False
        if self._current(generation, request_id):
            self.work_order.set_result(f"图书介绍完成：《{book['name']}》")
            self.work_order.set_stage("介绍完成")
            with self._lock: self.active={"id":"INTRO_BOOK","state":"completed","generation":generation,"request_id":request_id,"target_book":dict(book)}
        return True

    def start_find_book(self, book_id, generation=0, request_id=""):
        book = self.book_catalog.get(book_id)
        if not book:
            if self._preempt(generation, request_id): self._speak("请再说一下想寻找的书名。", generation, request_id)
            return False
        self._set_book_context(book)
        if not self._preempt(generation, request_id): return False
        if not self._current(generation, request_id): return False
        self.work_order.set_task("FIND_BOOK", f"寻找《{book['name']}》")
        if not self._speak("收到。", generation, request_id): return False
        if not self._current(generation, request_id): return False
        point = self.point_by_id(book["shelf_point"])
        response = self.nav.send_goal(point)
        if not self._current(generation,request_id):
            try: self.nav.cancel()
            except RuntimeError as exc: self.work_order.add_event("warn",f"废弃旧目标时取消失败：{exc}",source="navigation")
            return False
        if not response.get("goal_id"):
            self.work_order.add_event("warn","导航后端未返回goal_id，使用状态过渡与下发时间防串任务",source="navigation")
        with self._lock:
            if not self._current(generation, request_id): return False
            self.active={"id":"FIND_BOOK","state":"waiting_goto","generation":generation,"request_id":request_id,
                         "target_book":dict(book),"point_id":book["shelf_point"],"goal_id":response.get("goal_id"),
                         "sent_at":time.monotonic(),"sent_wall_at":time.time(),
                         "observed_running":False,"vision_running":False}
        self.work_order.set_stage(f"前往{book['shelf_name']}")
        return True

    def start_mission(self, mission_id, generation=0, request_id=""):
        mission=self.missions.get(mission_id)
        if not mission: raise ValueError(f"未知任务：{mission_id}")
        if mission_id == "CANCEL": return self.cancel(generation, request_id)
        if not self._preempt(generation, request_id): return False
        if not self._current(generation, request_id): return False
        self.work_order.set_task(mission_id, mission.get("title",mission_id))
        if mission.get("type") == "simple_speech":
            speech=self.speeches.get(mission.get("speech"),{}).get("text",mission.get("title",mission_id))
            ok=self._speak(speech,generation,request_id); self.work_order.set_stage("播报完成" if ok else "播报失败"); return ok
        if mission.get("start_speech"):
            text=self.speeches.get(mission["start_speech"],{}).get("text",mission["start_speech"])
            if not self._speak(text,generation,request_id): return False
        if not self._current(generation,request_id): return False
        if mission.get("type") == "goto": response=self.nav.send_goal(self.point_by_id(mission["point"])); state="waiting_goto"
        elif mission.get("type") == "patrol":
            response=self.nav.start_patrol(
                [self.point_by_id(x) for x in mission.get("route",[])],
                pause_seconds=mission.get("patrol_pause_seconds"))
            state="patrol"
        else: raise ValueError(f"不支持的任务类型：{mission.get('type')}")
        if not self._current(generation,request_id):
            try: self.nav.cancel()
            except RuntimeError as exc: self.work_order.add_event("warn",f"废弃旧导航请求时取消失败：{exc}",source="navigation")
            return False
        with self._lock:
            self.active={"id":mission_id,"state":state,"generation":generation,"request_id":request_id,
                         "mission":mission,"goal_id":response.get("goal_id"),"patrol_id":response.get("patrol_id"),
                         "sent_at":time.monotonic(),"sent_wall_at":time.time(),
                         "observed_running":False,"handled_points":set()}
        if state=="patrol" and not response.get("patrol_id"):
            self.work_order.add_event("warn","导航后端未返回patrol_id，使用状态过渡与下发时间防串任务",source="navigation")
        return True

    def execute_command(self, command, generation, request_id):
        kind=command.get("kind")
        if kind=="find_book": return self.start_find_book(command.get("book_id"),generation,request_id)
        if kind in ("introduce_book","introduce_current"): return self.introduce_book(command.get("book_id"),generation,request_id)
        if kind=="mission": return self.start_mission(command.get("mission"),generation,request_id)
        if kind=="cancel": return self.cancel(generation,request_id,announce=bool(command.get("announce",True)))
        if kind=="speak":
            if not self._preempt(generation,request_id): return False
            return self._speak(command.get("text",""),generation,request_id)
        raise ValueError(f"未知命令类型：{kind}")

    def tick(self):
        try: raw=self.nav.get_state(); state=simplify_navigation_state(raw)
        except RuntimeError as exc: self.work_order.set_navigation_status(f"导航未连接：{exc}"); return
        with self._lock: self.last_nav_state=state; active=dict(self.active) if self.active else None
        nav=state.get("navigation",{}); patrol=state.get("patrol",{})
        self.work_order.set_navigation_status(f"navigation={nav.get('state','unknown')}, patrol={patrol.get('state','idle')}")
        if not active or not self._current(active["generation"],active["request_id"]): return
        if active["state"] == "waiting_goto":
            nav_state=nav.get("state"); returned_id=nav.get("goal_id")
            if returned_id and active.get("goal_id") and returned_id != active["goal_id"]: return
            # A goal that is already close can pass through waiting_for_path
            # and following between two 300 ms polls.  The dashboard keeps
            # timestamped transition events, so use only navigation events
            # emitted after this goal was sent as equivalent observation.
            active_states={
                "pending", "planning", "running", "moving", "waiting",
                "navigating", "waiting_for_path", "following",
                "replanning", "aligning_goal_yaw", "final_forward",
            }
            if not active.get("observed_running"):
                sent_wall=float(active.get("sent_wall_at", float("inf")))
                for event in raw.get("events", []):
                    if event.get("source") != "navigation": continue
                    try: event_time=float(event.get("time", 0.0))
                    except (TypeError, ValueError): continue
                    text=str(event.get("text", ""))
                    if event_time >= sent_wall and any(text.endswith(s) for s in active_states):
                        active["observed_running"]=True
                        with self._lock:
                            if self.active and self.active.get("request_id")==active["request_id"]:
                                self.active["observed_running"]=True
                        break
            if time.monotonic()-active.get("sent_at",time.monotonic()) > self.goal_timeout:
                try: self.nav.cancel()
                except RuntimeError as exc: self.work_order.add_event("warn",f"导航超时取消失败：{exc}",source="navigation")
                self.work_order.set_error("导航任务超时")
                with self._lock:
                    if self.active and self.active.get("request_id")==active["request_id"]: self.active["state"]="failed"
                return
            # The ROS navigation backend reports its active lifecycle as
            # waiting_for_path -> following, with replanning/alignment states
            # during recovery.  Treat those as proof that this newly-issued
            # goal ran; otherwise a later legitimate "reached" is mistaken
            # for stale state and book vision never starts.
            if nav_state in active_states:
                with self._lock:
                    if self.active and self.active.get("request_id")==active["request_id"]: self.active["observed_running"]=True
            elif nav_state in ("reached", "approached") and active.get("observed_running"):
                if active["id"]=="FIND_BOOK": self._start_book_vision(active)
                else:
                    with self._lock:
                        if self.active and self.active.get("request_id")==active["request_id"]: self.active["state"]="completed"
            elif nav_state in ("failed","cancelled","emergency_stopped"):
                self.work_order.set_error(f"导航任务结束：{nav_state}")
        elif active["state"]=="patrol":
            returned_id=patrol.get("patrol_id")
            if returned_id and active.get("patrol_id") and returned_id!=active["patrol_id"]: return
            patrol_state=patrol.get("state")
            if patrol_state in ("pending","running","moving","waiting","navigating"):
                with self._lock:
                    if self.active and self.active.get("request_id")==active["request_id"]: self.active["observed_running"]=True
            elif time.monotonic()-active.get("sent_at",time.monotonic()) > self.patrol_timeout:
                try: self.nav.cancel()
                except RuntimeError as exc: self.work_order.add_event("warn",f"巡检超时取消失败：{exc}",source="navigation")
                self.work_order.set_error("巡检任务超时")
                with self._lock:
                    if self.active and self.active.get("request_id")==active["request_id"]: self.active["state"]="failed"
            if patrol_state in ("waiting","completed") and active.get("observed_running"):
                self._handle_patrol_point(active,patrol.get("index"))
            if patrol_state in ("completed","failed","cancelled","emergency_stopped") and active.get("observed_running"):
                with self._lock:
                    if self.active and self.active.get("request_id")==active["request_id"]: self.active["state"]=patrol_state

    def _handle_patrol_point(self, active, index):
        if not isinstance(index,int): return
        route=active.get("mission",{}).get("route",[])
        if index<0 or index>=len(route): return
        with self._lock:
            current=self.active
            if not current or current.get("request_id")!=active["request_id"]: return
            handled=current.setdefault("handled_points",set())
            if index in handled: return
            handled.add(index)
        event=active.get("mission",{}).get("point_events",{}).get(route[index])
        if not event or not self._current(active["generation"],active["request_id"]): return
        if event.get("vision_mode") == "shelf_misplaced":
            self._start_shelf_check_vision(active, route[index])
            return
        if event.get("work_order"): self.work_order.set_result(event["work_order"])
        speech=self.speeches.get(event.get("speech"),{}).get("text") if event.get("speech") else None
        if speech: self._speak(speech,active["generation"],active["request_id"],source="patrol")

    def _start_shelf_check_vision(self, active, point_id):
        shelf_id = shelf_id_for_point(point_id)
        if not shelf_id:
            self.work_order.set_vision_status(f"巡检点{point_id}未配置书架视觉检查")
            return
        self.work_order.set_stage(f"视觉检查{self.point_by_id(point_id)['name']}")
        threading.Thread(
            target=self._shelf_check_vision_worker,
            args=(dict(active), point_id, shelf_id), daemon=True).start()

    def _shelf_check_vision_worker(self, active, point_id, shelf_id):
        generation, request_id = active["generation"], active["request_id"]
        deadline = time.monotonic() + self.shelf_vision_timeout
        last_frame = 0
        consecutive = {}
        confirmed = {}
        saw_catalog_book = False
        error = ""
        while time.monotonic() < deadline and self._current(generation, request_id):
            frame = self.camera.wait_for_frame(
                last_frame, 0.8, max_age_sec=1.5) if self.camera else None
            if not frame:
                error = "摄像头无新鲜画面"
                continue
            data, last_frame, _ = frame
            try:
                result = detect_books_in_jpeg_json(data, expected_id=None)
            except Exception as exc:
                error = str(exc)
                continue
            books = result.get("books", [])
            current = misplaced_books_for_shelf(books, shelf_id)
            catalog_ids = set(self.book_catalog.by_id)
            for book in books:
                try:
                    if int(book.get("id")) in catalog_ids:
                        saw_catalog_book = True
                except (AttributeError, TypeError, ValueError):
                    continue
            current_by_id = {item["id"]: item for item in current}
            for book_id in list(consecutive):
                consecutive[book_id] = (
                    consecutive[book_id] + 1
                    if book_id in current_by_id else 0)
            for book_id, item in current_by_id.items():
                consecutive[book_id] = consecutive.get(book_id, 0) + 1
                if consecutive[book_id] >= self.shelf_vision_confirm_frames:
                    confirmed[book_id] = item
            if confirmed:
                break

        if not self._current(generation, request_id):
            return
        if confirmed:
            message = misplaced_books_message(list(confirmed.values()))
            self.work_order.set_vision_status("真实ArUco连续检测到错放图书")
        elif saw_catalog_book:
            message = misplaced_books_message([])
            self.work_order.set_vision_status("真实ArUco检测完成，未发现错放")
        else:
            message = "当前书架未识别到有效图书标签，无法判断是否错放。"
            self.work_order.set_vision_status(
                "书架视觉检测无有效结果" + (f"：{error}" if error else ""))
        self.work_order.set_result(message)
        self._speak(message, generation, request_id, source="shelf_vision")

    def _start_book_vision(self, active):
        with self._lock:
            if not self.active or self.active.get("vision_running"): return
            self.active["vision_running"]=True; self.active["state"]="vision"
        threading.Thread(target=self._book_vision_worker,args=(active,),daemon=True).start()

    def _book_vision_worker(self, active):
        generation,request_id=active["generation"],active["request_id"]; book=active["target_book"]
        self.work_order.set_stage("连续识别目标图书")
        count=0; last_frame=0; deadline=time.monotonic()+self.vision_timeout; error=""
        while time.monotonic()<deadline and self._current(generation,request_id):
            frame=self.camera.wait_for_frame(last_frame,1.0,max_age_sec=1.5) if self.camera else None
            if not frame: error="摄像头无新鲜画面"; continue
            data,last_frame,_=frame
            try: result=detect_books_in_jpeg_json(data,expected_id=int(book["id"]))
            except Exception as exc: error=str(exc); count=0; continue
            count = count+1 if result.get("found") else 0
            if count>=self.vision_confirm_frames:
                if not self._current(generation,request_id): return
                with self._lock: self.last_detected_book_id=int(book["id"]); self.last_book_context=dict(book)
                message=result.get("message",f"已识别到《{book['name']}》")
                self.work_order.set_vision_status("真实ArUco连续检测成功"); self.work_order.set_result(message)
                self._speak(message,generation,request_id,source="vision")
                with self._lock:
                    if self.active and self.active.get("request_id")==request_id: self.active["state"]="completed"
                return
        if not self._current(generation,request_id): return
        message="暂未识别到目标图书。"
        self.work_order.set_vision_status("真实ArUco检测超时"+(f"：{error}" if error else "")); self.work_order.set_result(message)
        self._speak(message,generation,request_id,source="vision")
        with self._lock:
            if self.active and self.active.get("request_id")==request_id: self.active["state"]="failed"

    def handle_voice_command(self, command_id):
        self.work_order.add_event("warn", f"旧模拟语音入口已停用：{command_id}", source="voice")

    def snapshot(self):
        with self._lock:
            active=dict(self.active) if self.active else None
            return {"active":active.get("id") if active else None,"state":active.get("state") if active else "idle",
                    "home":self.start_home,"home_source":self.home_source,"missions":self.mission_list(),"points":self.points,
                    "navigation":dict(self.last_nav_state),"target_book":dict(self.last_book_context) if self.last_book_context else None,
                    "last_requested_book_id":self.last_requested_book_id,"last_detected_book_id":self.last_detected_book_id,
                    "emergency_stopped":self.emergency_stopped}

    def current_expected_book_id(self):
        with self._lock: return int(self.last_book_context["id"]) if self.last_book_context else None
