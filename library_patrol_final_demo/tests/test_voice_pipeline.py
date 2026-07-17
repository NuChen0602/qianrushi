import json
import io
import os
import re
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from app.book_catalog import BookCatalog
from app.command_dispatcher import CommandDispatcher
from app.ci302_commands import FIXED_COMMANDS
from app.demo_server import DemoRequestHandler, DemoRuntime
from app.intent_resolver import classify_local_intent, enforce_execution_safety
from app.mission_orchestrator import MissionOrchestrator
from app.navigation_client import NavigationClient
from app.tts_hs_s77 import HsS77Error, HsS77Tts, SshUartTransport
from app.utils import json_response
from app.work_order import WorkOrderStore
from vision.aruco_book_detector import (
    live_position_message,
    misplaced_books_for_shelf,
    misplaced_books_message,
)
from scripts.voice_q_record_transcribe import (
    AsrTokenReservation, Ci302WakeListener, build_standard_commands, report_dispatcher_result,
    resolve_command, TtsStillBusy, wait_tts_idle, wait_tts_idle_in_chunks,
)
import scripts.voice_q_record_transcribe as voice_script
from scripts.voice_trigger_ssh_bridge import VoiceBridge

ROOT=Path(__file__).resolve().parents[1]


class FakeVoice:
    def start(self,callback): self.callback=callback


class FakeNav:
    def __init__(self): self.calls=[]; self.state="idle"; self.events=[]
    def get_state(self): return {"robot":{"x":0,"y":0,"yaw":0},"navigation":{"state":self.state},"patrol":{"state":"idle"},"events":self.events}
    def cancel(self): self.calls.append("cancel"); return {"ok":True,"state":"cancelled"}
    def send_goal(self,point): self.calls.append(("goal",point["id"])); self.state="pending"; return {"ok":True,"goal_id":"g1"}
    def start_patrol(self,points,pause_seconds=None): self.calls.append(("patrol",len(points),pause_seconds)); return {"ok":True,"patrol_id":"p1"}
    def emergency_stop(self): self.calls.append("estop"); return {"ok":True}
    def emergency_release(self): self.calls.append("release"); return {"ok":True}


class FakeTts:
    def __init__(self): self.texts=[]
    def speak(self,text,**kwargs): self.texts.append(text); return True
    def cancel_pending(self,generation): pass


class IntentTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.catalog=BookCatalog(ROOT/"config")
    def test_all_books_introduction_and_find(self):
        for book in self.catalog.books:
            for prefix in ("介绍","介绍一下","讲讲","说说","了解"):
                self.assertEqual(classify_local_intent(prefix+book["name"],self.catalog).kind,"introduce_book")
            for prefix in ("寻找","找一下","带我去找","去书架找"):
                result=classify_local_intent(prefix+book["name"],self.catalog)
                self.assertEqual((result.kind,result.book_id),("find_book",book["id"]))
            self.assertNotEqual(classify_local_intent(book["name"],self.catalog).kind,"find_book")
            self.assertNotEqual(classify_local_intent("不要找"+book["name"],self.catalog).kind,"find_book")
    def test_api_find_cannot_override_introduction(self):
        safe=enforce_execution_safety("介绍百年孤独",{"action":"find_book","book_id":203},self.catalog)
        self.assertEqual(safe["action"],"introduce_book")
    def test_explicit_find_rejects_api_introduction(self):
        safe=enforce_execution_safety("寻找百年孤独",{"action":"introduce_book","book_id":203},self.catalog)
        self.assertEqual(safe["action"],"find_book")
    def test_short_alias_without_context_is_uncertain(self):
        self.assertIsNone(self.catalog.match_text("百年以后仍然孤独"))


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.nav=FakeNav(); self.tts=FakeTts(); self.work=WorkOrderStore()
        self.orch=MissionOrchestrator(ROOT/"config",self.nav,self.work,FakeVoice(),tts=self.tts)
        self.current=(1,"r1"); self.orch.set_token_checker(lambda g,r:(g,r)==self.current)
    def test_prompt_does_not_cancel(self):
        self.orch.prompt("请说。",1,"r1")
        self.assertNotIn("cancel",self.nav.calls)
    def test_introduction_never_sends_navigation_goal(self):
        self.assertTrue(self.orch.introduce_book(203,1,"r1"))
        self.assertFalse(any(isinstance(call,tuple) and call[0] in ("goal","patrol") for call in self.nav.calls))
    def test_find_book_only_speaks_non_triggering_acknowledgement(self):
        self.assertTrue(self.orch.start_find_book(203,1,"r1"))
        self.assertEqual(self.tts.texts,["收到。"])
        self.assertIn(("goal","LIT_SHELF_A3"),self.nav.calls)
    def test_context_survives_cancel_and_wake(self):
        self.orch.start_find_book(203,1,"r1")
        self.current=(2,"r2"); self.orch.cancel(2,"r2",False)
        self.current=(3,"r3"); self.assertTrue(self.orch.introduce_book(None,3,"r3"))
        self.assertIn("百年孤独",self.tts.texts[-1])
        self.assertEqual([x for x in self.nav.calls if isinstance(x,tuple) and x[0]=="goal"],[('goal','LIT_SHELF_A3')])
    def test_shelf_patrol_speaks_only_actual_aruco_misplacement(self):
        class Camera:
            def __init__(self): self.frame=0
            def wait_for_frame(self,*_args,**_kwargs):
                self.frame += 1
                return b"jpeg", self.frame, time.time()
        active={
            "id":"SHELF_CHECK", "state":"patrol", "generation":1,
            "request_id":"r1", "mission":self.orch.missions["SHELF_CHECK"],
        }
        self.orch.camera=Camera()
        with self.orch._lock: self.orch.active=active
        result={"books":[{"id":203,"name":"百年孤独"}]}
        with mock.patch(
                "app.mission_orchestrator.detect_books_in_jpeg_json",
                return_value=result):
            self.orch._shelf_check_vision_worker(active,"ENG_SHELF_B1","A1")
        self.assertEqual(len(self.tts.texts),1)
        self.assertIn("百年孤独",self.tts.texts[0])
        self.assertIn("文学历史书架",self.tts.texts[0])
        self.assertNotIn("工程控制论",self.tts.texts[0])
    def test_stale_reached_not_accepted(self):
        self.orch.start_find_book(203,1,"r1"); self.nav.state="reached"; self.orch.tick()
        self.assertEqual(self.orch.snapshot()["state"],"waiting_goto")
    def test_backend_following_then_reached_starts_book_vision(self):
        started=[]
        self.orch._start_book_vision=lambda active: started.append(active["request_id"])
        self.orch.start_find_book(203,1,"r1")
        self.nav.state="following"; self.orch.tick()
        self.nav.state="reached"; self.orch.tick()
        self.assertEqual(started,["r1"])
    def test_transient_following_event_then_reached_starts_book_vision(self):
        started=[]
        self.orch._start_book_vision=lambda active: started.append(active["request_id"])
        self.orch.start_find_book(203,1,"r1")
        sent=self.orch.active["sent_wall_at"]
        self.nav.events=[{"time":sent+.01,"source":"navigation","text":"导航状态：following"}]
        self.nav.state="reached"; self.orch.tick()
        self.assertEqual(started,["r1"])
    def test_same_generation_preempts_navigation_once(self):
        self.orch.cancel(1,"r1",False)
        self.orch.introduce_book(203,1,"r1")
        self.assertEqual(self.nav.calls.count("cancel"),1)
    def test_idle_token_invalidation_does_not_require_navigation(self):
        def unavailable_cancel():
            self.nav.calls.append("cancel"); raise RuntimeError("navigation offline")
        self.nav.cancel=unavailable_cancel
        self.assertTrue(self.orch.invalidate(1,"r1"))
        self.assertNotIn("cancel",self.nav.calls)
        self.assertTrue(self.orch.introduce_book(203,1,"r1"))
        self.assertNotIn("cancel",self.nav.calls)
        self.assertIn("百年孤独",self.tts.texts[-1])
    def test_moving_token_invalidation_still_confirms_cancel(self):
        with self.orch._lock:
            self.orch.active={"id":"FIND_BOOK","state":"waiting_goto",
                              "generation":1,"request_id":"r1"}
        self.assertTrue(self.orch.invalidate(1,"r1"))
        self.assertEqual(self.nav.calls.count("cancel"),1)
    def test_cancel_failure_escalates_and_old_task_cannot_resume(self):
        def failing_cancel():
            self.nav.calls.append("cancel"); raise RuntimeError("cancel timeout")
        self.nav.cancel=failing_cancel
        self.assertFalse(self.orch.cancel(1,"r1"))
        self.assertIn("estop",self.nav.calls)
        self.assertTrue(self.orch.snapshot()["emergency_stopped"])
        self.current=(2,"r2")
        self.assertFalse(self.orch.start_find_book(203,2,"r2"))
        self.assertFalse(any(isinstance(call,tuple) and call[0]=="goal" for call in self.nav.calls))
    def test_unconfirmed_cancel_response_escalates(self):
        def unconfirmed_cancel():
            self.nav.calls.append("cancel"); return {"ok":True}
        self.nav.cancel=unconfirmed_cancel
        self.assertFalse(self.orch.cancel(1,"r1"))
        self.assertIn("estop",self.nav.calls)
    def test_no_camera_frame_cannot_report_book_found(self):
        self.orch.vision_timeout=.01
        active={"generation":1,"request_id":"r1","target_book":self.orch.book_catalog.get(203)}
        with self.orch._lock: self.orch.active={**active,"id":"FIND_BOOK","state":"vision","vision_running":True}
        self.orch._book_vision_worker(active)
        self.assertEqual(self.orch.snapshot()["state"],"failed")
        self.assertIsNone(self.orch.snapshot()["last_detected_book_id"])
    def test_old_visual_result_cannot_write_after_new_generation(self):
        entered=threading.Event(); release=threading.Event()
        class Camera:
            def wait_for_frame(_self,*_args,**_kwargs): return b"jpeg",1,time.time()
        self.orch.camera=Camera(); active={"generation":1,"request_id":"r1","target_book":self.orch.book_catalog.get(203)}
        with self.orch._lock: self.orch.active={**active,"id":"FIND_BOOK","state":"vision","vision_running":True}
        def detect(*_args,**_kwargs): entered.set(); release.wait(1); return {"found":True,"message":"旧结果"}
        with mock.patch("app.mission_orchestrator.detect_books_in_jpeg_json",side_effect=detect):
            worker=threading.Thread(target=self.orch._book_vision_worker,args=(active,)); worker.start()
            self.assertTrue(entered.wait(1)); self.current=(2,"r2"); release.set(); worker.join(1)
        self.assertIsNone(self.orch.snapshot()["last_detected_book_id"])
        self.assertNotIn("旧结果",self.tts.texts)
    def test_three_consecutive_frames_required(self):
        class Camera:
            frame=0
            def wait_for_frame(self,*_args,**_kwargs):
                self.frame+=1; return b"jpeg",self.frame,time.time()
        self.orch.camera=Camera(); self.orch.vision_confirm_frames=3
        active={"generation":1,"request_id":"r1","target_book":self.orch.book_catalog.get(203)}
        with self.orch._lock: self.orch.active={**active,"id":"FIND_BOOK","state":"vision","vision_running":True}
        with mock.patch("app.mission_orchestrator.detect_books_in_jpeg_json",return_value={"found":True,"message":"连续确认"}) as detect:
            self.orch._book_vision_worker(active)
        self.assertEqual(detect.call_count,3)
        self.assertEqual(self.orch.snapshot()["last_detected_book_id"],203)


class FakeTransport:
    def __init__(self): self.calls=[]; self.release=threading.Event(); self.fail=False
    def transact(self,frame,accept,complete,state_callback=None):
        self.calls.append(frame)
        if state_callback: state_callback("accepted")
        if self.fail: raise HsS77Error("module_error_45")
        self.release.wait(2); return True
    def close(self): pass


class TtsTests(unittest.TestCase):
    def test_ssh_helper_keeps_channel_stdin_for_frames(self):
        transport=SshUartTransport("board","root","/dev/ttyS0",115200)
        process=mock.MagicMock(); process.poll.return_value=None
        with mock.patch("app.tts_hs_s77.subprocess.Popen",return_value=process) as popen, \
             mock.patch.object(transport,"_readline",return_value=b'{"ready":true}\n'):
            transport._start()
        command=popen.call_args.args[0][-1]
        self.assertIn("python3 -u -c",command)
        self.assertNotIn("| python3",command)
        transport.process=None

    def test_serial_queue_waits_for_completion(self):
        transport=FakeTransport(); tts=HsS77Tts({"enabled":True},transport=transport); tts.start()
        results=[]
        a=threading.Thread(target=lambda:results.append(tts.speak("第一条")))
        b=threading.Thread(target=lambda:results.append(tts.speak("第二条")))
        a.start(); time.sleep(.05); b.start(); time.sleep(.1)
        self.assertEqual(len(transport.calls),1)
        transport.release.set(); a.join(); b.join(); tts.close()
        self.assertEqual(len(transport.calls),2); self.assertEqual(results,[True,True])
    def test_module_error_propagates(self):
        transport=FakeTransport(); transport.fail=True
        tts=HsS77Tts({"enabled":True},transport=transport); tts.start()
        with self.assertRaises(HsS77Error): tts.speak("错误测试")
        tts.close()
    def test_completion_timeout_propagates(self):
        transport=FakeTransport()
        transport.transact=lambda *args,**kwargs: (_ for _ in ()).throw(HsS77Error("completion_timeout"))
        tts=HsS77Tts({"enabled":True},transport=transport); tts.start()
        with self.assertRaisesRegex(HsS77Error,"completion_timeout"): tts.speak("超时测试")
        tts.close()
    def test_ssh_reconnect_and_short_write(self):
        class ShortStream:
            def __init__(self): self.data=bytearray()
            def write(self,data):
                count=min(2,len(data)); self.data.extend(data[:count]); return count
            def flush(self): pass
        class Process:
            def __init__(self): self.stdin=ShortStream()
        class ReconnectingTransport(SshUartTransport):
            def __init__(self): self.process=None; self.stopping=False; self.starts=0; self.processes=[]
            def _start(self):
                if self.process is None:
                    self.starts+=1; self.process=Process(); self.processes.append(self.process)
            def _disconnect(self): self.process=None
            def _readline(self,_timeout):
                if self.starts==1: raise OSError("disconnected")
                return b'{"state":"accepted"}\n' if not hasattr(self,"accepted") else b'{"state":"completed"}\n'
            def close(self): self.process=None
        transport=ReconnectingTransport()
        states=[]
        original=transport._readline
        def read(timeout):
            line=original(timeout)
            if b'accepted' in line: transport.accepted=True
            return line
        transport._readline=read
        frame=b"1234567"
        self.assertTrue(transport.transact(frame,.1,.2,states.append))
        self.assertEqual(transport.starts,2)
        self.assertEqual(bytes(transport.processes[-1].stdin.data),len(frame).to_bytes(4,"big")+frame)
        self.assertEqual(states,["accepted","completed"])


class NavigationTests(unittest.TestCase):
    def test_http_200_ok_false_is_failure(self):
        response=mock.MagicMock(); response.__enter__.return_value=response; response.read.return_value=b'{"ok":false,"error":"bad"}'
        with mock.patch("urllib.request.urlopen",return_value=response):
            with self.assertRaises(RuntimeError): NavigationClient("http://x").cancel()

    def test_cancel_400_is_idempotent_only_after_confirmed_idle(self):
        client=NavigationClient("http://x")
        idle={"navigation":{"state":"idle"},"patrol":{"active":False,"state":"idle"}}
        with mock.patch.object(
                client,"_request_json",
                side_effect=[RuntimeError("navigation API unavailable: HTTP Error 400: Bad Request"),idle]) as request:
            result=client.cancel()
        self.assertTrue(result["ok"])
        self.assertEqual(result["state"],"idle")
        self.assertEqual(request.call_count,2)

    def test_cancel_400_remains_failure_while_navigation_active(self):
        client=NavigationClient("http://x")
        moving={"navigation":{"state":"following"},"patrol":{"active":False,"state":"idle"}}
        with mock.patch.object(
                client,"_request_json",
                side_effect=[RuntimeError("navigation API unavailable: HTTP Error 400: Bad Request"),moving]):
            with self.assertRaisesRegex(RuntimeError,"HTTP Error 400"):
                client.cancel()


class WorkOrderTests(unittest.TestCase):
    def test_concurrent_access(self):
        store=WorkOrderStore(max_events=1000)
        threads=[threading.Thread(target=lambda n=n:[store.add_event("info",str(i),source=str(n)) for i in range(100)]) for n in range(8)]
        for t in threads:t.start()
        for t in threads:t.join()
        ids=[x["id"] for x in store.snapshot()["events"]]
        self.assertEqual(len(ids),len(set(ids)))


class LiveBookPositionTests(unittest.TestCase):
    def test_complete_shelf_uses_live_left_to_right_rank(self):
        message=live_position_message({
            "name":"百年孤独", "rank":2, "visible_shelf_book_count":5,
            "shelf_position_complete":True,
        })
        self.assertEqual(message,"已识别到《百年孤独》，在书架上从左往右第2本")

    def test_partial_frame_never_claims_an_absolute_shelf_rank(self):
        message=live_position_message({
            "name":"百年孤独", "rank":1, "visible_shelf_book_count":2,
            "shelf_position_complete":False,
        })
        self.assertIn("当前画面已识别到的2本",message)
        self.assertIn("从左往右第1本",message)

    def test_misplacement_uses_detected_marker_shelf_not_fixed_event(self):
        misplaced=misplaced_books_for_shelf(
            [{"id":203,"name":"百年孤独"}],"A1")
        self.assertEqual(len(misplaced),1)
        self.assertEqual(misplaced[0]["expected_shelf_id"],"A3")
        message=misplaced_books_message(misplaced)
        self.assertIn("百年孤独",message)
        self.assertIn("工科书架",message)
        self.assertIn("文学历史书架",message)


class FakeDispatchOrchestrator:
    tts=None
    def __init__(self): self.executed=[]; self.checker=None
    def set_token_checker(self,checker): self.checker=checker
    def invalidate(self,generation,request_id): self.executed.append((generation,request_id,"invalidate"))
    def execute_command(self,command,generation,request_id): self.executed.append((generation,request_id,command["kind"]))
    def emergency_stop(self,generation,request_id): self.executed.append((generation,request_id,"emergency_stop"))
    def emergency_release(self,generation,request_id): self.executed.append((generation,request_id,"emergency_release"))


class DispatcherTests(unittest.TestCase):
    def test_server_rejects_stale_generation(self):
        orchestrator=FakeDispatchOrchestrator(); dispatcher=CommandDispatcher(orchestrator,WorkOrderStore())
        old=dispatcher.reserve("asr","input"); new=dispatcher.reserve("ci302","command")
        result=dispatcher.submit(old,{"kind":"find_book","book_id":203})
        self.assertFalse(result["accepted"]); self.assertEqual(result["reason"],"stale_generation")
        self.assertEqual(result["server_generation"],new["generation"])
        self.assertTrue(dispatcher.submit(new,{"kind":"cancel"})["accepted"])
        time.sleep(.1); dispatcher.close()
        self.assertEqual([x[2] for x in orchestrator.executed],["invalidate","invalidate","cancel","invalidate"])
    def test_emergency_invalidates_old_and_release_stays_idle(self):
        orchestrator=FakeDispatchOrchestrator(); dispatcher=CommandDispatcher(orchestrator,WorkOrderStore())
        old=dispatcher.reserve("web","find"); dispatcher.emergency_stop("api")
        self.assertFalse(dispatcher.submit(old,{"kind":"find_book","book_id":203})["accepted"])
        dispatcher.emergency_release("api"); dispatcher.close()
        self.assertEqual([x[2] for x in orchestrator.executed],
                         ["invalidate","invalidate","emergency_stop","invalidate","emergency_release","invalidate"])


class SafetyTests(unittest.TestCase):
    def test_recording_device_exit_is_retried(self):
        expected=Path("/tmp/recovered-recording.wav")
        with mock.patch.object(
                voice_script,"record_file",
                side_effect=[RuntimeError("录音设备提前退出"),expected]) as record:
            actual=voice_script.record_file_with_retry(
                SimpleNamespace(),1.0,"command_",threading.Event(),attempts=3,retry_delay=0)
        self.assertEqual(actual,expected)
        self.assertEqual(record.call_count,2)

    def test_recording_retries_do_not_hide_persistent_failure(self):
        with mock.patch.object(
                voice_script,"record_file",
                side_effect=RuntimeError("录音设备提前退出")) as record:
            with self.assertRaisesRegex(RuntimeError,"录音设备连续失败"):
                voice_script.record_file_with_retry(
                    SimpleNamespace(),1.0,"command_",threading.Event(),
                    attempts=3,retry_delay=0)
        self.assertEqual(record.call_count,3)

    def test_ci302_bridge_returns_on_ssh_eof(self):
        bridge=object.__new__(VoiceBridge)
        bridge.running=True
        bridge.handle_frame=mock.Mock()
        thread=threading.Thread(target=bridge.parse,args=(io.BytesIO(b""),),daemon=True)
        thread.start(); thread.join(timeout=.2)
        if thread.is_alive():
            bridge.running=False
            thread.join(timeout=.2)
        self.assertFalse(thread.is_alive())

    def test_ci302_reconnect_setup_reclaims_serial_owner(self):
        bridge=VoiceBridge("board","root","/dev/ttyS1",115200,"http://web","http://wake")
        setup=bridge.setup()
        self.assertIn("killall voice_motion_test",setup)
        self.assertIn("stty -F /dev/ttyS1 115200",setup)

    def test_find_book_bridge_reserves_and_submits_direct_navigation_command(self):
        bridge=VoiceBridge("board","root","/dev/null",115200,"http://web","http://wake")
        bridge.latest_command_sequence=1
        token={"generation":7,"request_id":"ci302-find"}
        bridge.reliable_post=mock.Mock(side_effect=[token,{"ok":True}])
        bridge.submit_after_ack=mock.Mock()
        command={"id":"FIND_203","kind":"find_book","book_id":203}
        with mock.patch("scripts.voice_trigger_ssh_bridge.threading.Thread") as thread:
            bridge.process_event({"event":"command","sequence":1,"frame":"AA 55 01 01 FB",
                                  "command":command})
        self.assertEqual(bridge.reliable_post.call_args_list[0].args,
                         ("http://web/api/demo/command/reserve",{"source":"ci302","command_type":"find_book"}))
        self.assertEqual(bridge.reliable_post.call_args_list[1].args,
                         ("http://wake",{"event":"command","source":"ci302","command_id":"FIND_203","generation":7}))
        self.assertEqual(thread.call_args.kwargs["args"],(token,command,1,1))
        thread.return_value.start.assert_called_once()


class RepeatedWakeTests(unittest.TestCase):
    def setUp(self):
        self.listener=Ci302WakeListener("127.0.0.1",0)

    def tearDown(self):
        self.listener.server.server_close()

    def test_second_wake_during_active_session_gets_new_prompt_generation(self):
        prompts=[]; last=0
        self.listener.handle_event("wake")
        first=self.listener.wait_for_wake(last); prompts.append(first); last=first
        self.assertTrue(self.listener.session_active)
        self.listener.handle_event("wake")
        second=self.listener.wait_for_wake(last); prompts.append(second)
        self.assertEqual(prompts,[1,2])
        self.assertFalse(self.listener.current_wake(first))
        self.assertTrue(self.listener.current_wake(second))

    def test_second_wake_cancels_old_recording_and_invalidates_old_result(self):
        self.listener.handle_event("wake")
        first=self.listener.wait_for_wake(0)
        self.listener.clear_change(first)
        cancel_event=self.listener.recording_cancel_event()
        old_api_executed=[]

        self.listener.handle_event("wake")
        self.assertTrue(cancel_event.is_set())
        if self.listener.current_wake(first):
            old_api_executed.append(True)
        self.assertEqual(old_api_executed,[])

        second=self.listener.wait_for_wake(first)
        self.listener.clear_change(second)
        self.assertFalse(cancel_event.is_set())
        self.assertTrue(self.listener.current_wake(second))

    def test_sleep_then_wake_starts_fresh_session(self):
        self.listener.handle_event("wake")
        first=self.listener.wait_for_wake(0)
        self.listener.handle_event("sleep")
        self.assertFalse(self.listener.current_wake(first))
        self.assertFalse(self.listener.session_active)
        self.listener.handle_event("wake")
        second=self.listener.wait_for_wake(first)
        self.assertEqual(second,2)
        self.assertTrue(self.listener.current_wake(second))

    def test_repeated_wake_does_not_cancel_navigation_or_clear_book_context(self):
        nav=FakeNav(); work=WorkOrderStore()
        orchestrator=MissionOrchestrator(ROOT/"config",nav,work,FakeVoice(),tts=FakeTts())
        orchestrator._set_book_context(orchestrator.book_catalog.get(203))
        before=orchestrator.current_target_book()
        self.listener.handle_event("wake")
        first=self.listener.wait_for_wake(0)
        self.listener.handle_event("wake")
        self.listener.wait_for_wake(first)
        self.assertNotIn("cancel",nav.calls)
        self.assertEqual(orchestrator.current_target_book(),before)


class FindCandidateFlowTests(unittest.TestCase):
    def setUp(self):
        self.listener=Ci302WakeListener("127.0.0.1",0)
        self.catalog=BookCatalog(ROOT/"config")
        self.standards=build_standard_commands(self.catalog)
        self.standard_by_id={item["id"]:item for item in self.standards}
        self.listener.handle_event("wake")
        self.wake_generation=self.listener.wait_for_wake(0)
        self.listener.clear_change(self.wake_generation)

    def tearDown(self):
        self.listener.server.server_close()

    def add_candidate(self, token=None):
        payload={"book_id":203,"command_id":"FIND_203",
                 "received_at":time.time(),"source":"ci302"}
        payload.update(token or {})
        self.listener.handle_event("find_candidate",payload)
        return payload

    def resolve(self, text, candidate):
        return resolve_command(text,self.catalog,self.standards,self.standard_by_id,
                               mock.MagicMock(),[],candidate)

    def test_candidate_during_recording_continues_then_find_is_submitted(self):
        nav=FakeNav(); work=WorkOrderStore()
        orchestrator=MissionOrchestrator(ROOT/"config",nav,work,FakeVoice(),tts=FakeTts())
        dispatcher=CommandDispatcher(orchestrator,work)
        token=dispatcher.reserve("asr","asr_utterance")
        self.add_candidate()
        self.assertFalse(self.listener.recording_cancel_event().is_set())
        self.assertTrue(self.listener.current_wake(self.wake_generation))
        candidate=self.listener.take_candidate()
        command,_=self.resolve("寻找百年孤独",candidate)
        result=dispatcher.submit(token,command)
        self.assertTrue(result["accepted"])
        time.sleep(.1); dispatcher.close()
        self.assertEqual((command["kind"],command["book_id"]),("find_book",203))
        self.assertIn(("goal","LIT_SHELF_A3"),nav.calls)

    def test_candidate_with_introduction_never_sends_navigation_goal(self):
        nav=FakeNav(); work=WorkOrderStore()
        orchestrator=MissionOrchestrator(ROOT/"config",nav,work,FakeVoice(),tts=FakeTts())
        dispatcher=CommandDispatcher(orchestrator,work)
        token=dispatcher.reserve("asr","asr_utterance")
        self.add_candidate()
        candidate=self.listener.take_candidate()
        command,_=self.resolve("介绍百年孤独",candidate)
        result=dispatcher.submit(token,command)
        self.assertTrue(result["accepted"])
        time.sleep(.1); dispatcher.close()
        self.assertEqual((command["kind"],command["book_id"]),("introduce_book",203))
        self.assertFalse(any(isinstance(call,tuple) and call[0] in ("goal","patrol") for call in nav.calls))

    def test_candidate_without_valid_asr_does_not_submit_find(self):
        orchestrator=FakeDispatchOrchestrator()
        dispatcher=CommandDispatcher(orchestrator,WorkOrderStore())
        before=dispatcher.snapshot()["generation"]
        self.add_candidate()
        self.listener.discard_candidate()  # Empty ASR path.
        self.assertEqual(dispatcher.snapshot()["generation"],before)
        dispatcher.close()
        self.assertNotIn("find_book",[item[2] for item in orchestrator.executed])

    def test_repeated_or_late_candidate_does_not_expire_asr_token(self):
        orchestrator=FakeDispatchOrchestrator(); dispatcher=CommandDispatcher(orchestrator,WorkOrderStore())
        token=dispatcher.reserve("asr","asr_utterance")
        generation=dispatcher.snapshot()["generation"]
        self.add_candidate(); self.add_candidate()
        self.assertEqual(dispatcher.snapshot()["generation"],generation)
        candidate=self.listener.take_candidate()
        command,_=self.resolve("寻找百年孤独",candidate)
        self.assertTrue(dispatcher.submit(token,command)["accepted"])
        dispatcher.close()

    def test_vad_reserves_once_and_new_wake_invalidates_token(self):
        reservation=AsrTokenReservation(
            SimpleNamespace(web_url="http://test"),self.listener,
            self.wake_generation,self.listener.current_task_generation())
        token={"generation":7,"request_id":"vad-7","source":"asr",
               "command_type":"asr_utterance","received_at":time.time()}
        with mock.patch.object(voice_script,"reserve_command",return_value=token) as reserve:
            reservation.reserve_once(); reservation.reserve_once()
            self.assertEqual(reservation.wait_or_reserve(),token)
        reserve.assert_called_once()
        self.listener.handle_event("wake")
        self.assertIsNone(reservation.wait_or_reserve())

    def test_fixed_command_cancels_current_recording(self):
        self.listener.handle_event("command",{"command_id":"INTRO_CURRENT"})
        self.assertTrue(self.listener.recording_cancel_event().is_set())
        self.assertEqual(self.listener.current_task_generation(),1)


class VoiceStateAndIntegrationTests(unittest.TestCase):
    def test_demo_state_has_top_level_tts(self):
        runtime=object.__new__(DemoRuntime)
        runtime.navigation_client=mock.Mock()
        runtime.navigation_client.get_state.return_value={
            "robot":{"x":0,"y":0,"yaw":0},"navigation":{"state":"idle"},
            "patrol":{"state":"idle"},
        }
        runtime.orchestrator=mock.Mock()
        runtime.orchestrator.snapshot.return_value={
            "home":None,"home_source":"test","missions":[],"points":{},
            "active":None,"state":"idle",
            "navigation":{"available":True,"navigation":{"state":"idle"}},
        }
        runtime.camera=mock.Mock(); runtime.camera.status.return_value={}
        runtime.smoke_sensor=mock.Mock(); runtime.smoke_sensor.snapshot.return_value={}
        runtime.dht11=mock.Mock(); runtime.dht11.read.return_value={}
        runtime.temperature_monitor=mock.Mock()
        runtime.temperature_monitor.snapshot.return_value={"enabled":True,"active":False}
        runtime.work_order=mock.Mock(); runtime.work_order.snapshot.return_value={}
        runtime.dispatcher=mock.Mock(); runtime.dispatcher.snapshot.return_value={}
        runtime.tts=mock.Mock(); runtime.tts.status.return_value={
            "current":"idle","queued":0,"last":"completed",
        }
        payload=DemoRuntime.state_payload(runtime)
        self.assertEqual(payload["tts"],runtime.tts.status.return_value)
        self.assertEqual(payload["temperature_alert"],{"enabled":True,"active":False})
        self.assertEqual(payload["navigation"]["navigation"]["state"],"idle")
        runtime.navigation_client.get_state.assert_not_called()

    def test_json_response_ignores_disconnected_poll_client(self):
        handler=mock.Mock()
        handler.wfile.write.side_effect=BrokenPipeError(32,"broken pipe")
        json_response(handler,{"ok":True})
        handler.send_response.assert_called_once_with(200)

    def test_missing_tts_state_fails_immediately(self):
        listener=Ci302WakeListener("127.0.0.1",0)
        try:
            listener.handle_event("wake"); generation=listener.wait_for_wake(0)
            response=mock.MagicMock(); response.__enter__.return_value=response
            response.read.return_value=b'{"ok":true}'
            started=time.monotonic()
            with mock.patch("scripts.voice_q_record_transcribe.urllib.request.urlopen",return_value=response):
                with self.assertRaisesRegex(RuntimeError,"缺少顶层 tts"):
                    wait_tts_idle(SimpleNamespace(web_url="http://test"),listener,
                                  generation,0,time.monotonic()+10)
            self.assertLess(time.monotonic()-started,.5)
        finally: listener.server.server_close()

    def test_new_wake_interrupts_tts_wait(self):
        listener=Ci302WakeListener("127.0.0.1",0)
        try:
            listener.handle_event("wake"); generation=listener.wait_for_wake(0)
            response=mock.MagicMock(); response.__enter__.return_value=response
            response.read.return_value=b'{"tts":{"current":"speaking","queued":0}}'
            def wake_during_request(*_args,**_kwargs):
                listener.handle_event("wake")
                return response
            with mock.patch("scripts.voice_q_record_transcribe.urllib.request.urlopen",
                            side_effect=wake_during_request):
                self.assertFalse(wait_tts_idle(SimpleNamespace(web_url="http://test"),listener,
                                               generation,0,time.monotonic()+10))
        finally: listener.server.server_close()

    def test_long_tts_playback_waits_in_chunks_instead_of_ending_session(self):
        listener=mock.Mock()
        listener.current_wake.return_value=True
        listener.current_task_generation.return_value=0
        output=io.StringIO()
        with mock.patch.object(
                voice_script,"wait_tts_idle",
                side_effect=[TtsStillBusy("busy"),TtsStillBusy("busy"),True]) as wait, \
             mock.patch("sys.stdout",output):
            result=wait_tts_idle_in_chunks(
                SimpleNamespace(),listener,1,0,time.monotonic()+10)
        self.assertTrue(result)
        self.assertEqual(wait.call_count,3)
        self.assertEqual(output.getvalue().count("tts_busy_wait_continue"),2)

    def test_dispatcher_rejection_logs_reason(self):
        output=io.StringIO()
        with mock.patch("sys.stdout",output):
            accepted=report_dispatcher_result(
                {"accepted":False,"reason":"stale_generation"},
                {"generation":3},{"kind":"find_book"},
            )
        self.assertFalse(accepted)
        self.assertIn("dispatcher_submit accepted=false generation=3",output.getvalue())
        self.assertIn("reason=stale_generation",output.getvalue())

    def test_prompt_completion_starts_recording_and_submits_asr_command(self):
        class FakeListener:
            def __init__(self):
                self.change_event=threading.Event(); self.stopped=False
                self.candidate={"book_id":203,"command_id":"FIND_203",
                                "received_at":time.time(),"source":"ci302"}
            def start(self): pass
            def wait_for_wake(self,_after): return 1
            def clear_change(self,*_args): self.change_event.clear()
            def current_wake(self,generation): return generation==1
            def current_task_generation(self): return 0
            def recording_cancel_event(self): return self.change_event
            def sleeping(self): return False
            def discard_candidate(self): pass
            def take_candidate(self):
                candidate=self.candidate; self.candidate=None; return candidate
            def stop(self): self.stopped=True

        args=SimpleNamespace(
            api_key="test",wake_host="127.0.0.1",wake_port=0,
            session_timeout=10.0,wake_response_delay=0.0,command_seconds=1.0,
            debug_transcripts=False,save_recordings=False,once=True,
            web_url="http://test",
        )
        fd,name=tempfile.mkstemp(suffix=".wav"); os.close(fd); wav=Path(name)
        calls=[]
        def fake_post(url,payload,**_kwargs):
            calls.append((url,payload))
            if url.endswith("/api/demo/prompt"): return {"completed":True}
            if url.endswith("/api/demo/command/reserve"):
                return {"request_id":"r1","generation":1,"source":"asr",
                        "received_at":time.time(),"command_type":"asr_utterance"}
            if url.endswith("/api/demo/command"): return {"accepted":True}
            raise AssertionError(url)
        def fake_record(*_args,**kwargs):
            kwargs["on_speech"]()
            return wav
        output=io.StringIO()
        with mock.patch.object(voice_script,"parse_args",return_value=args), \
             mock.patch.object(voice_script,"Ci302WakeListener",return_value=FakeListener()), \
             mock.patch.object(voice_script,"record_file",side_effect=fake_record) as recording, \
             mock.patch.object(voice_script,"audio_has_speech",return_value=True), \
             mock.patch.object(voice_script,"transcribe",return_value="寻找百年孤独"), \
             mock.patch.object(voice_script,"post_json",side_effect=fake_post), \
             mock.patch.object(voice_script,"wait_tts_idle",
                               side_effect=AssertionError("prompt后不应重复等待TTS")) as wait_idle, \
             mock.patch("sys.stdout",output):
            self.assertEqual(voice_script.main(),0)
        recording.assert_called_once()
        wait_idle.assert_not_called()
        self.assertIn("prompt_completed",output.getvalue())
        self.assertIn("recording_started wake_generation=1",output.getvalue())
        self.assertIn("speech_detected",output.getvalue())
        self.assertIn("asr_token_reserved generation=1 request_id=r1",output.getvalue())
        reserves=[payload for url,payload in calls if url.endswith("/api/demo/command/reserve")]
        self.assertEqual(len(reserves),1)
        self.assertEqual(reserves[0]["command_type"],"asr_utterance")
        submitted=[payload for url,payload in calls if url.endswith("/api/demo/command")]
        self.assertEqual(submitted[0]["command"]["kind"],"find_book")
        self.assertEqual(submitted[0]["command"]["book_id"],203)

    def test_full_patrol_mapping(self):
        command=next(item for item in FIXED_COMMANDS if item["id"]=="FULL_PATROL")
        self.assertEqual(command["mission"],"FULL_PATROL")
    def test_remote_emergency_release_requires_token(self):
        handler=object.__new__(DemoRequestHandler)
        handler.client_address=("192.168.43.88",1234); handler.headers={}
        with mock.patch.dict(os.environ,{"DEMO_CONTROL_TOKEN":"secret"}):
            with self.assertRaises(PermissionError): handler.require_control_access()
    def test_source_has_no_hardcoded_api_key(self):
        pattern=re.compile(r"\b[0-9a-fA-F]{32}\.[A-Za-z0-9_-]{8,}\b")
        paths=[ROOT/"app",ROOT/"scripts",ROOT/"config"]
        offenders=[]
        for base in paths:
            for path in base.rglob("*"):
                if path.suffix in (".py",".sh",".json",".yaml",".yml") and path.is_file():
                    if pattern.search(path.read_text(encoding="utf-8",errors="ignore")): offenders.append(str(path))
        self.assertEqual(offenders,[])

    def test_usb_microphone_defaults_keep_jieli_agc_enabled(self):
        source=(ROOT/"scripts/start_full_demo.sh").read_text(encoding="utf-8")
        self.assertIn('VOICE_MIC_GAIN="${VOICE_MIC_GAIN:-100%}"',source)
        self.assertIn('VOICE_AGC="${VOICE_AGC:-on}"',source)
        self.assertNotIn("set 'Auto Gain Control' off",source)


if __name__=="__main__": unittest.main()
