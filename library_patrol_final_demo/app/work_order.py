from app.utils import now_iso
import threading


class WorkOrderStore:
    def __init__(self, max_events=80):
        self.max_events = max_events
        self._lock = threading.RLock()
        self._event_id = 0
        self.request_id = ""
        self.generation = 0
        self.clear()

    def snapshot(self):
        with self._lock:
            return {
                "current_task": self.current_task,
                "current_title": self.current_title,
                "stage": self.stage,
                "navigation_status": self.navigation_status,
                "vision_status": self.vision_status,
                "voice_status": self.voice_status,
                "result": self.result,
                "error": self.error,
                "request_id": self.request_id,
                "generation": self.generation,
                "events": [dict(item) for item in self.events],
            }

    def set_command_context(self, request_id, generation):
        with self._lock:
            self.request_id = str(request_id or "")
            self.generation = int(generation or 0)

    def add_event(self, level, text, source="demo", request_id=None, generation=None):
        with self._lock:
            self._event_id += 1
            self.events.insert(
                0,
                {
                    "id": self._event_id,
                    "time": now_iso(),
                    "level": level,
                    "source": source,
                    "text": text,
                    "request_id": self.request_id if request_id is None else str(request_id),
                    "generation": self.generation if generation is None else int(generation),
                },
            )
            del self.events[self.max_events :]

    def set_task(self, task_id, title):
        with self._lock:
            self.current_task = task_id
            self.current_title = title
            self.error = ""
            self.result = ""
            self.vision_status = "空闲"
            self.voice_status = "空闲"
            self.add_event("info", f"任务切换：{title}", source="mission")

    def clear_error(self):
        with self._lock: self.error = ""

    def clear_result(self):
        with self._lock: self.result = ""

    def set_stage(self, text):
        with self._lock:
            self.stage = text
            self.add_event("info", text, source="stage")

    def set_navigation_status(self, text):
        with self._lock: self.navigation_status = text

    def set_vision_status(self, text):
        with self._lock: self.vision_status = text

    def set_voice_status(self, text):
        with self._lock: self.voice_status = text

    def set_result(self, text):
        with self._lock:
            self.result = text
            self.add_event("ok", text, source="result")

    def set_error(self, text):
        with self._lock:
            self.error = text
            self.add_event("error", text, source="error")

    def clear(self):
        with self._lock:
            self.current_task = ""
            self.current_title = ""
            self.stage = "空闲"
            self.navigation_status = "未连接"
            self.vision_status = "空闲"
            self.voice_status = "空闲"
            self.result = ""
            self.error = ""
            self.events = []
            self.add_event("info", "演示工单已初始化")

    def mark_idle(self, stage="空闲", preserve_context=True):
        with self._lock:
            self.current_task = ""
            self.current_title = ""
            self.stage = stage
            self.result = ""
            self.error = ""
            self.vision_status = "空闲"
            self.voice_status = "空闲"
            self.add_event("info", stage, source="stage")
