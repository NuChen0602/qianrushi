import math
import time
import unittest

from app.temperature_risk_monitor import ALERT_TEXT, TemperatureRiskMonitor
from app.work_order import WorkOrderStore


class FakeReader:
    def __init__(self, payload=None): self.payload=payload or {"ok":False}
    def read(self):
        if isinstance(self.payload,Exception): raise self.payload
        return dict(self.payload)


class FakeTts:
    def __init__(self, fail_calls=0): self.calls=[]; self.fail_calls=fail_calls
    def speak(self, text, **kwargs):
        self.calls.append((text,kwargs))
        if len(self.calls)<=self.fail_calls: raise RuntimeError("tts failed")
        return True


class TemperatureRiskMonitorTests(unittest.TestCase):
    def monitor(self, tts=None, **kwargs):
        return TemperatureRiskMonitor(
            FakeReader(),tts or FakeTts(),WorkOrderStore(),
            window_seconds=kwargs.pop("window_seconds",60),
            cooldown_seconds=kwargs.pop("cooldown_seconds",0),
            sample_interval=.1,**kwargs)

    @staticmethod
    def feed(monitor, values, start=0):
        triggered=[]
        for offset,value in enumerate(values):
            triggered.append(monitor.add_sample(value,start+offset))
        monitor.wait_for_idle()
        return triggered

    def test_sub_threshold_rise_does_not_alert(self):
        monitor=self.monitor(); self.feed(monitor,[26.0]*3+[26.8]*3)
        self.assertEqual(monitor.tts.calls,[])
        self.assertAlmostEqual(monitor.snapshot()["delta_c"],.8)

    def test_rise_over_one_degree_alerts_once_with_exact_text(self):
        monitor=self.monitor(); self.feed(monitor,[26.0]*3+[27.2]*3)
        self.assertEqual(len(monitor.tts.calls),1)
        self.assertEqual(monitor.tts.calls[0][0],ALERT_TEXT)
        self.assertEqual(ALERT_TEXT,"温度数据异常，存在起火风险，请立即排查。")
        event=next(e for e in monitor.work_order.snapshot()["events"]
                   if e["source"]=="temperature_alert" and e["level"]=="warn")
        self.assertIn("基准温度=26.0℃ 当前温度=27.2℃ 变化=1.2℃",event["text"])

    def test_single_spike_does_not_alert(self):
        monitor=self.monitor(); self.feed(monitor,[26.0,26.0,26.0,30.0,26.0,26.0])
        self.assertEqual(monitor.tts.calls,[])

    def test_sustained_anomaly_does_not_repeat(self):
        monitor=self.monitor(window_seconds=30)
        self.feed(monitor,[26.0]*3+[27.2]*12)
        self.assertEqual(len(monitor.tts.calls),1)
        self.assertTrue(monitor.snapshot()["latched"])

    def test_stable_recovery_rearms_then_can_alert_again(self):
        monitor=self.monitor(window_seconds=6,rearm_delta_c=.5)
        self.feed(monitor,[26.0]*3+[27.2]*3,start=0)
        self.feed(monitor,[26.0]*8,start=6)
        self.assertFalse(monitor.snapshot()["latched"])
        self.feed(monitor,[27.2]*3,start=14)
        self.assertEqual(len(monitor.tts.calls),2)

    def test_invalid_sensor_values_and_exceptions_are_ignored(self):
        monitor=self.monitor()
        for value in (None,float("nan"),float("inf"),"bad",True):
            self.assertFalse(monitor.add_sample(value,0))
        monitor.reader.payload=RuntimeError("sensor failed")
        self.assertFalse(monitor.tick())
        monitor.reader.payload={"ok":True,"stale":True,"temperature_c":30.0}
        monitor._next_sample_at=0
        self.assertFalse(monitor.tick())
        self.assertIsNone(monitor.snapshot()["baseline_c"])

    def test_tts_error_is_recorded_and_monitor_continues(self):
        tts=FakeTts(fail_calls=1); monitor=self.monitor(tts,window_seconds=6)
        self.feed(monitor,[26.0]*3+[27.2]*3)
        errors=[e for e in monitor.work_order.snapshot()["events"] if e["level"]=="error"]
        self.assertEqual(len(errors),1)
        self.feed(monitor,[26.0]*8,start=6)
        self.feed(monitor,[27.2]*3,start=14)
        self.assertEqual(len(tts.calls),2)

    def test_alert_does_not_change_dispatcher_or_call_navigation(self):
        class GuardNavigation:
            def __init__(self): self.calls=[]
            def cancel(self): self.calls.append("cancel")
            def send_goal(self,*_args): self.calls.append("send_goal")
            def emergency_stop(self): self.calls.append("emergency_stop")
        nav=GuardNavigation(); generation={"value":4}
        token_provider=lambda:{"generation":generation["value"],"request_id":"existing-task"}
        monitor=TemperatureRiskMonitor(
            FakeReader(),FakeTts(),WorkOrderStore(),token_provider,
            window_seconds=60,cooldown_seconds=0)
        self.feed(monitor,[26.0]*3+[27.2]*3)
        self.assertEqual(generation["value"],4)
        self.assertEqual(nav.calls,[])
        self.assertEqual(monitor.tts.calls[0][1]["generation"],4)


if __name__=="__main__": unittest.main()
