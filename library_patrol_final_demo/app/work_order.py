from app.utils import now_iso


class WorkOrderStore:
    def __init__(self, max_events=80):
        self.max_events = max_events
        self.clear()

    def snapshot(self):
        return {
            "current_task": self.current_task,
            "current_title": self.current_title,
            "stage": self.stage,
            "navigation_status": self.navigation_status,
            "vision_status": self.vision_status,
            "voice_status": self.voice_status,
            "result": self.result,
            "error": self.error,
            "events": list(self.events),
        }

    def add_event(self, level, text, source="demo"):
        self.events.insert(
            0,
            {
                "time": now_iso(),
                "level": level,
                "source": source,
                "text": text,
            },
        )
        del self.events[self.max_events :]

    def set_task(self, task_id, title):
        self.current_task = task_id
        self.current_title = title
        # 新任务开始时清理上一轮结果，避免视频画面残留“导航失败”等旧状态。
        self.error = ""
        self.result = ""
        self.vision_status = "视觉占位"
        self.voice_status = "语音占位"
        self.add_event("info", f"任务切换：{title}", source="mission")

    def clear_error(self):
        self.error = ""

    def clear_result(self):
        self.result = ""

    def set_stage(self, text):
        self.stage = text
        self.add_event("info", text, source="stage")

    def set_navigation_status(self, text):
        self.navigation_status = text

    def set_vision_status(self, text):
        self.vision_status = text

    def set_voice_status(self, text):
        self.voice_status = text

    def set_result(self, text):
        self.result = text
        self.add_event("ok", text, source="result")

    def set_error(self, text):
        self.error = text
        self.add_event("error", text, source="error")

    def clear(self):
        self.current_task = ""
        self.current_title = ""
        self.stage = "空闲"
        self.navigation_status = "未连接"
        self.vision_status = "视觉占位"
        self.voice_status = "语音占位"
        self.result = ""
        self.error = ""
        self.events = []
        self.add_event("info", "演示工单已初始化")
