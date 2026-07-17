import subprocess
import select
import threading
import time

SMOKE_ALERT_TEXT = "烟雾数据异常，请立即排查风险"

class SmokeSensor:
    def __init__(self, config, work_order, tts=None):
        self.config = dict(config or {})
        self.work_order = work_order
        self.tts = tts
        self.enabled = bool(self.config.get("enabled", True))
        self.board_host = str(self.config.get("board_host", "192.168.43.192"))
        self.board_user = str(self.config.get("board_user", "root"))
        self.raw_path = str(
            self.config.get(
                "raw_path",
                "/sys/bus/iio/devices/iio:device0/in_voltage3_raw",
            )
        )
        self.scale_path = str(
            self.config.get(
                "scale_path",
                "/sys/bus/iio/devices/iio:device0/in_voltage_scale",
            )
        )
        self.threshold_mv = float(self.config.get("threshold_mv", 1000.0))
        self.clear_threshold_mv = float(self.config.get("clear_threshold_mv", 900.0))
        self.interval = max(0.2, float(self.config.get("interval_seconds", 0.8)))
        self.confirm_samples = max(1, int(self.config.get("confirm_samples", 3)))
        self.lock = threading.RLock()
        self.running = False
        self.thread = None
        self.ssh_process = None
        self.state = {
            "enabled": self.enabled,
            "available": False,
            "channel": "A3",
            "raw": None,
            "scale_mv": None,
            "voltage_mv": None,
            "threshold_mv": self.threshold_mv,
            "alarm": False,
            "error": "等待首次采样" if self.enabled else "烟雾传感器已禁用",
            "updated_at": None,
        }
        self.high_count = 0
        self.low_count = 0
        self.alert_notified = False
        self.alert_thread = None

    def start(self):
        if not self.enabled or self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True, name="mq2-smoke-sensor")
        self.thread.start()

    def stop(self):
        self.running = False
        self._close_reader()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        if self.alert_thread and self.alert_thread.is_alive():
            self.alert_thread.join(timeout=0.5)

    def snapshot(self):
        with self.lock:
            return dict(self.state)

    def _start_reader(self):
        if self.ssh_process is not None and self.ssh_process.poll() is None:
            return self.ssh_process
        self._close_reader()
        command = (
            "while :; do "
            f"raw=$(cat {self.raw_path}) || exit 2; "
            f"scale=$(cat {self.scale_path}) || exit 2; "
            "printf '%s %s\\n' \"$raw\" \"$scale\"; "
            f"sleep {self.interval}; done"
        )
        self.ssh_process = subprocess.Popen(
            [
                "ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=15",
                "-o", "ServerAliveInterval=3", "-o", "ServerAliveCountMax=2",
                f"{self.board_user}@{self.board_host}", command,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
        )
        return self.ssh_process

    def _close_reader(self):
        process = self.ssh_process
        self.ssh_process = None
        if process is None:
            return
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()

    def _read(self):
        process = self._start_reader()
        readable, _, _ = select.select([process.stdout], [], [], 20.0)
        if not readable:
            self._close_reader()
            raise TimeoutError("MQ-2 板端持久连接读取超时")
        line = process.stdout.readline()
        if not line:
            self._close_reader()
            raise ConnectionError("MQ-2 板端持久连接已断开")
        values = line.split()
        if len(values) < 2:
            raise ValueError("板端 ADC 返回数据不完整")
        raw = int(values[0])
        scale_mv = float(values[1])
        return raw, scale_mv, raw * scale_mv

    def _update_alarm(self, voltage_mv):
        alarm = bool(self.state["alarm"])
        if voltage_mv >= self.threshold_mv:
            self.high_count += 1
            self.low_count = 0
        elif voltage_mv <= self.clear_threshold_mv:
            self.low_count += 1
            self.high_count = 0
        else:
            self.high_count = 0
            self.low_count = 0

        if not alarm and self.high_count >= self.confirm_samples:
            self.state["alarm"] = True
            if not self.alert_notified:
                self.alert_notified = True
                self.work_order.add_event(
                    "error",
                    f"MQ-2 烟雾告警：A3 {voltage_mv / 1000.0:.2f}V，连续{self.confirm_samples}次达到0.50V",
                    source="smoke",
                )
                self.alert_thread = threading.Thread(
                    target=self._speak_alert,
                    name="mq2-smoke-alert-tts",
                    daemon=True,
                )
                self.alert_thread.start()
        elif alarm and self.low_count >= self.confirm_samples:
            self.state["alarm"] = False
            self.work_order.add_event(
                "ok",
                f"MQ-2 烟雾告警解除：A3 {voltage_mv / 1000.0:.2f}V",
                source="smoke",
            )

    def _speak_alert(self):
        try:
            if self.tts is None:
                raise RuntimeError("HS-S77 TTS未配置")
            played = self.tts.speak(SMOKE_ALERT_TEXT)
            if not played:
                raise RuntimeError("HS-S77未完成烟雾报警播报")
            self.work_order.set_voice_status("HS-S77播放完成：" + SMOKE_ALERT_TEXT)
        except Exception as exc:
            self.work_order.add_event(
                "error",
                f"[smoke_alert] HS-S77播报失败：{exc}",
                source="smoke",
            )

    def _loop(self):
        while self.running:
            retry_delay = 0.0
            try:
                raw, scale_mv, voltage_mv = self._read()
                with self.lock:
                    self.state.update({
                        "available": True,
                        "raw": raw,
                        "scale_mv": scale_mv,
                        "voltage_mv": round(voltage_mv, 3),
                        "error": "",
                        "updated_at": time.time(),
                    })
                    self._update_alarm(voltage_mv)
            except (OSError, ValueError, subprocess.SubprocessError) as exc:
                with self.lock:
                    self.state["available"] = False
                    self.state["error"] = str(exc)
                    self.state["updated_at"] = time.time()
                self.high_count = 0
                self.low_count = 0
                retry_delay = 3.0
            if retry_delay:
                time.sleep(retry_delay)
