import math
import os
import statistics
import threading
import time
from collections import deque
from datetime import datetime, timezone


ALERT_TEXT = "温度数据异常，存在起火风险，请立即排查。"


def _env_bool(name, default=True):
    value=os.getenv(name)
    if value is None: return bool(default)
    return value.strip().lower() not in ("0", "false", "no", "off", "")


class TemperatureRiskMonitor:
    """Non-blocking temperature-rise monitor using the existing DHT11 cache."""

    def __init__(self, reader, tts, work_order, token_provider=None, *,
                 enabled=True, window_seconds=60.0, delta_c=1.0,
                 cooldown_seconds=300.0, rearm_delta_c=0.5,
                 sample_interval=2.0, clock=time.monotonic):
        self.reader=reader; self.tts=tts; self.work_order=work_order
        self.token_provider=token_provider or (lambda:{"generation":0,"request_id":""})
        self.enabled=bool(enabled); self.window_seconds=max(1.0,float(window_seconds))
        self.delta_threshold=float(delta_c); self.cooldown_seconds=max(0.0,float(cooldown_seconds))
        self.rearm_delta=float(rearm_delta_c); self.sample_interval=max(0.1,float(sample_interval))
        self.clock=clock; self._lock=threading.RLock(); self._samples=deque()
        self._next_sample_at=0.0; self._baseline=None; self._current=None; self._delta=None
        self._active=False; self.alert_latched=False; self.alert_inflight=False
        self._rearm_since=None; self._last_trigger_monotonic=None; self.last_triggered_at=None
        self._closed=False; self._worker_thread=None

    @classmethod
    def from_env(cls, reader, tts, work_order, token_provider=None):
        return cls(
            reader,tts,work_order,token_provider,
            enabled=_env_bool("TEMP_ALERT_ENABLED",True),
            window_seconds=float(os.getenv("TEMP_ALERT_WINDOW_SECONDS","60")),
            delta_c=float(os.getenv("TEMP_ALERT_DELTA_C","1.0")),
            cooldown_seconds=float(os.getenv("TEMP_ALERT_COOLDOWN_SECONDS","300")),
            rearm_delta_c=float(os.getenv("TEMP_ALERT_REARM_DELTA_C","0.5")),
        )

    def tick(self):
        if not self.enabled or self._closed: return False
        now=self.clock()
        with self._lock:
            if now < self._next_sample_at: return False
            self._next_sample_at=now+self.sample_interval
        try: payload=self.reader.read()
        except Exception: return False
        if (not isinstance(payload,dict) or not payload.get("ok")
                or payload.get("stale")):
            return False
        return self.add_sample(payload.get("temperature_c"),now)

    def add_sample(self, temperature_c, now=None):
        if not self.enabled or self._closed or isinstance(temperature_c,bool): return False
        try: temperature=float(temperature_c)
        except (TypeError,ValueError): return False
        if not math.isfinite(temperature): return False
        now=self.clock() if now is None else float(now)
        trigger=None
        with self._lock:
            self._samples.append((now,temperature))
            cutoff=now-self.window_seconds
            while self._samples and self._samples[0][0] < cutoff:
                self._samples.popleft()
            values=[value for _stamp,value in self._samples]
            if len(values)<3: return False
            edge_count=min(3,len(values))
            self._baseline=float(statistics.median(values[:edge_count]))
            self._current=float(statistics.median(values[-edge_count:]))
            self._delta=self._current-self._baseline
            self._active=self._delta>self.delta_threshold

            if self.alert_latched:
                if self._delta<=self.rearm_delta:
                    if self._rearm_since is None: self._rearm_since=now
                    elif now-self._rearm_since>=self.window_seconds and not self.alert_inflight:
                        self.alert_latched=False; self._rearm_since=None
                else:
                    self._rearm_since=None

            cooldown_ok=(self._last_trigger_monotonic is None or
                         now-self._last_trigger_monotonic>=self.cooldown_seconds)
            if self._active and not self.alert_latched and not self.alert_inflight and cooldown_ok:
                self.alert_latched=True; self.alert_inflight=True
                self._last_trigger_monotonic=now
                self.last_triggered_at=datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")
                trigger=(self._baseline,self._current,self._delta)
        if trigger:
            baseline,current,delta=trigger
            self.work_order.add_event(
                "warn",
                f"[temperature_alert] 基准温度={baseline:.1f}℃ 当前温度={current:.1f}℃ 变化={delta:.1f}℃",
                source="temperature_alert",
            )
            worker=threading.Thread(target=self._speak_alert,name="temperature-alert-tts",daemon=True)
            with self._lock: self._worker_thread=worker
            worker.start()
            return True
        return False

    def _speak_alert(self):
        try:
            token=dict(self.token_provider() or {})
            played=self.tts.speak(
                ALERT_TEXT,request_id=token.get("request_id",""),
                generation=int(token.get("generation",0) or 0),
            )
            if not played: raise RuntimeError("HS-S77未完成温度报警播报")
        except Exception as exc:
            self.work_order.add_event(
                "error",f"[temperature_alert] HS-S77播报失败：{exc}",
                source="temperature_alert",
            )
        finally:
            with self._lock: self.alert_inflight=False

    def wait_for_idle(self, timeout=1.0):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            with self._lock:
                if not self.alert_inflight: return True
            time.sleep(0.005)
        return False

    def snapshot(self):
        with self._lock:
            return {
                "enabled":self.enabled,"active":self._active,
                "baseline_c":self._baseline,"current_c":self._current,
                "delta_c":self._delta,"last_triggered_at":self.last_triggered_at,
                "latched":self.alert_latched,"inflight":self.alert_inflight,
            }

    def close(self):
        self._closed=True
        thread=self._worker_thread
        if thread and thread.is_alive(): thread.join(timeout=0.2)
