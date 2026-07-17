#!/usr/bin/env python3
"""“你好小亚”唤醒、GLM-ASR 转写、GLM 模糊对话和动态寻书。"""
from __future__ import annotations

import argparse
import array
import datetime as dt
import json
import math
import os
import signal
import select
import subprocess
import sys
import threading
import time
import urllib.request
import urllib.error
import uuid
import wave
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app.book_catalog import BookCatalog
from app.intent_resolver import classify_local_intent, enforce_execution_safety

RECORDINGS = ROOT / "data" / "voice" / "recordings"
TRANSCRIPTS = ROOT / "data" / "voice" / "transcripts"


def prune_private_files(directory, max_files=50, max_age_days=7):
    if not directory.exists(): return
    files=sorted((p for p in directory.iterdir() if p.is_file()),key=lambda p:p.stat().st_mtime,reverse=True)
    cutoff=time.time()-max_age_days*86400
    for index,path in enumerate(files):
        if index>=max_files or path.stat().st_mtime<cutoff: path.unlink(missing_ok=True)


def parse_args():
    parser = argparse.ArgumentParser(description="唤醒后通过自然语言寻找任意已登记图书")
    parser.add_argument("--device", default=os.getenv("VOICE_DEVICE", "default"))
    parser.add_argument("--rate", type=int, default=int(os.getenv("VOICE_RATE", "48000")))
    parser.add_argument("--channels", type=int, default=int(os.getenv("VOICE_CHANNELS", "1")))
    parser.add_argument("--command-seconds", type=float, default=7.0)
    parser.add_argument("--wake-response-delay", type=float,
                        default=float(os.getenv("CI302_ACK_FALLBACK_SECONDS",os.getenv("CI302_WAKE_RESPONSE_DELAY","1.0"))))
    parser.add_argument("--command-cooldown", type=float, default=1.5)
    parser.add_argument("--vad-rms-dbfs", type=float, default=-58.0)
    parser.add_argument("--vad-peak-dbfs", type=float, default=-42.0)
    parser.add_argument("--wake-host", default=os.getenv("VOICE_WAKE_HOST", "127.0.0.1"))
    parser.add_argument("--wake-port", type=int, default=int(os.getenv("VOICE_WAKE_PORT", "8092")))
    parser.add_argument("--api-key", default=os.getenv("ZHIPU_API_KEY"))
    parser.add_argument("--base-url", default=os.getenv("VOICE_BASE_URL", "https://open.bigmodel.cn/api/paas/v4"))
    parser.add_argument("--asr-model", default=os.getenv("VOICE_STT_MODEL", "glm-asr-2512"))
    parser.add_argument("--chat-model", default=os.getenv("VOICE_CHAT_MODEL", "glm-4.5-flash"))
    parser.add_argument("--web-url", default=os.getenv("DEMO_WEB_URL", "http://127.0.0.1:8090"))
    parser.add_argument("--once", action="store_true", help="完成一次对话后退出")
    parser.add_argument("--save-recordings", action="store_true", default=os.getenv("VOICE_SAVE_RECORDINGS","0")=="1")
    parser.add_argument("--debug-transcripts", action="store_true", default=os.getenv("VOICE_DEBUG_TRANSCRIPTS","0")=="1")
    parser.add_argument("--session-timeout", type=float, default=25.0)
    return parser.parse_args()


def record_file(args, seconds, prefix, cancel_event=None, on_speech=None):
    RECORDINGS.mkdir(parents=True, exist_ok=True)
    path = RECORDINGS / (prefix + dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".wav")
    command = ["arecord", "-D", args.device, "-c", str(args.channels), "-r", str(args.rate),
               "-f", "S16_LE", "-t", "raw"]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    started = time.monotonic()
    chunks=[]; heard=False; silent_since=None
    while time.monotonic() - started < seconds:
        if cancel_event and cancel_event.is_set(): break
        if process.poll() is not None: raise RuntimeError("录音设备提前退出")
        readable,_,_=select.select([process.stdout],[],[],0.05)
        if not readable: continue
        chunk=os.read(process.stdout.fileno(),max(4096,int(args.rate*args.channels*2*.1)))
        if not chunk: continue
        chunks.append(chunk)
        samples=array.array("h"); samples.frombytes(chunk[:len(chunk)//2*2])
        if not samples: continue
        peak=max(abs(x) for x in samples); rms=math.sqrt(sum(x*x for x in samples)/len(samples))
        audible=(20*math.log10(max(peak,1)/32768)>=args.vad_peak_dbfs and
                 20*math.log10(max(rms,1)/32768)>=args.vad_rms_dbfs)
        if audible:
            if not heard and on_speech:
                threading.Thread(target=on_speech,name="voice-input-reserve",daemon=True).start()
            heard=True; silent_since=None
        elif heard:
            silent_since=silent_since or time.monotonic()
            if time.monotonic()-silent_since>=0.9: break
    if process.poll() is None: process.send_signal(signal.SIGINT)
    try: process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.terminate()
        try: process.wait(timeout=1)
        except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=1)
    with wave.open(str(path),"wb") as output:
        output.setnchannels(args.channels); output.setsampwidth(2); output.setframerate(args.rate)
        output.writeframes(b"".join(chunks))
    os.chmod(path, 0o600)
    return path


def record_file_with_retry(args, seconds, prefix, cancel_event=None,
                           on_speech=None, attempts=3, retry_delay=0.25):
    """Retry short ALSA open/exit failures without killing the whole demo."""
    last_error = None
    for attempt in range(1, max(1, int(attempts)) + 1):
        if cancel_event and cancel_event.is_set():
            return None
        try:
            return record_file(
                args, seconds, prefix,
                cancel_event=cancel_event, on_speech=on_speech)
        except RuntimeError as exc:
            last_error = exc
            if str(exc) != "录音设备提前退出" or attempt >= attempts:
                break
            print(f"recording_retry attempt={attempt} reason=audio_device_exit", flush=True)
            if cancel_event:
                if cancel_event.wait(retry_delay):
                    return None
            else:
                time.sleep(retry_delay)
    raise RuntimeError(f"录音设备连续失败：{last_error}") from last_error


def transcribe(path, args, allow_empty=False):
    boundary="----xia-"+uuid.uuid4().hex
    body=[]
    def field(name,value):
        body.extend([f"--{boundary}\r\nContent-Disposition: form-data; name=\"{name}\"\r\n\r\n{value}\r\n".encode()])
    field("model",args.asr_model); field("stream","false")
    body.extend([f"--{boundary}\r\nContent-Disposition: form-data; name=\"file\"; filename=\"audio.wav\"\r\nContent-Type: audio/wav\r\n\r\n".encode(),path.read_bytes(),b"\r\n",f"--{boundary}--\r\n".encode()])
    req=urllib.request.Request(args.base_url.rstrip("/")+"/audio/transcriptions",b"".join(body),
        {"Authorization":"Bearer "+args.api_key,"Content-Type":"multipart/form-data; boundary="+boundary},method="POST")
    try:
        with urllib.request.urlopen(req,timeout=120) as response: payload=json.loads(response.read())
    except urllib.error.URLError as exc: raise RuntimeError(f"语音识别请求失败：{exc}") from exc
    text = str(payload.get("text", "")).strip()
    if text in {"#", "…", "..."}:
        text = ""
    if not text and not allow_empty:
        raise RuntimeError("语音识别响应缺少text")
    return text


def post_json(url, payload, api_key=None, timeout=15):
    headers = {"Content-Type": "application/json"}
    if api_key:
        headers["Authorization"] = "Bearer " + api_key
    request = urllib.request.Request(
        url, json.dumps(payload, ensure_ascii=False).encode("utf-8"), headers, method="POST"
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def normalize_command(text):
    normalized = str(text).strip()
    for punctuation in "，。！？,.!?；;：: \t\r\n《》":
        normalized = normalized.replace(punctuation, "")
    return normalized


def build_standard_commands(catalog):
    commands = [
        {"id": "RECOMMEND_BOOKS", "phrase": "推荐文学小说", "kind": "mission", "mission": "RECOMMEND_BOOKS"},
        {"id": "INTRO_CURRENT", "phrase": "介绍这本书", "kind": "introduce_current"},
        {"id": "SHELF_CHECK", "phrase": "检查当前书架", "kind": "mission", "mission": "SHELF_CHECK"},
        {"id": "LOST_ITEM_PATROL", "phrase": "扫描遗失物", "kind": "mission", "mission": "LOST_ITEM_PATROL"},
        {"id": "HAZARD_CHECK", "phrase": "检查高危点位", "kind": "mission", "mission": "HAZARD_CHECK"},
        {"id": "FULL_PATROL", "phrase": "开始全图巡检", "kind": "mission", "mission": "FULL_PATROL"},
        {"id": "RETURN_HOME", "phrase": "返回起点", "kind": "mission", "mission": "RETURN_HOME"},
        {"id": "CANCEL", "phrase": "停止当前任务", "kind": "cancel"},
    ]
    commands.extend({
        "id": f"FIND_{book['id']}", "phrase": f"寻找{book['name']}",
        "kind": "find_book", "book_id": int(book["id"]),
    } for book in catalog.books)
    commands.extend({
        "id": f"INTRO_{book['id']}", "phrase": f"介绍{book['name']}",
        "kind": "introduce_book", "book_id": int(book["id"]),
    } for book in catalog.books)
    return commands


def exact_standard_command(text, standard_commands):
    normalized = normalize_command(text)
    return next(
        (command for command in standard_commands if normalize_command(command["phrase"]) == normalized),
        None,
    )


def interpret_dialogue(text, catalog, standard_commands, args, history=None):
    books = [{"id": b["id"], "name": b["name"], "aliases": b.get("aliases", []),
              "category": b["category"]} for b in catalog.books]
    standards = [{"command_id": c["id"], "standard_phrase": c["phrase"]} for c in standard_commands]
    system = (
        "你是图书馆寻书机器人小亚。必须按优先级理解用户口语。"
        "必须严格区分介绍和寻找：介绍、简介、讲讲、说说某本书只允许使用introduce_book或对应INTRO命令，"
        "只做语音播报，绝对不能返回find_book或FIND命令；只有用户明确表达寻找、找书、带路或前往书架时才允许寻找。"
        "第一优先：判断用户是否与某条标准指令语义相同，允许同义说法、语序变化、口语、省略和ASR错字；"
        "若相同，route必须为standard并返回该command_id。"
        "只有确实不属于任何标准指令时，route才为fallback，并使用action。"
        "fallback的action只能是find_book、introduce_book、list_books、recommend、cancel、chat。"
        "涉及具体图书时book_id必须来自目录；无法确定时不要猜，action设为chat并用reply追问。"
        "reply必须是适合语音播报的简短中文，不使用Markdown。"
        "只输出JSON对象，格式为{\"route\":\"standard或fallback\",\"command_id\":\"...或null\","
        "\"action\":\"...或null\",\"book_id\":数字或null,\"reply\":\"...\"}。"
        "标准指令：" + json.dumps(standards, ensure_ascii=False) + "。"
        "图书目录：" + json.dumps(books, ensure_ascii=False)
    )
    messages = [{"role": "system", "content": system}]
    messages.extend(list(history or [])[-6:])
    messages.append({"role": "user", "content": text})
    payload = {
        "model": args.chat_model,
        "messages": messages,
        "temperature": 0.1,
        "response_format": {"type": "json_object"},
        "max_tokens": 800,
    }
    response = post_json(args.base_url.rstrip("/") + "/chat/completions", payload, args.api_key, 30)
    choice = response["choices"][0]
    content = choice["message"].get("content", "").strip()
    if not content and choice.get("finish_reason") == "length":
        payload["max_tokens"] = 1600
        response = post_json(args.base_url.rstrip("/") + "/chat/completions", payload, args.api_key, 45)
        content = response["choices"][0]["message"].get("content", "").strip()
    if content.startswith("```"):
        content = content.strip("`").removeprefix("json").strip()
    if not content:
        raise RuntimeError("对话 API 未返回可执行内容")
    decision = json.loads(content)
    if decision.get("route") == "standard":
        valid_ids = {command["id"] for command in standard_commands}
        if decision.get("command_id") not in valid_ids:
            raise ValueError("对话 API 返回未知标准指令")
        return decision
    decision["route"] = "fallback"
    action = decision.get("action")
    if action not in {"find_book", "introduce_book", "list_books", "recommend", "cancel", "chat"}:
        raise ValueError("对话 API 返回未知 action")
    if action in {"find_book", "introduce_book", "recommend"} and not catalog.get(decision.get("book_id")):
        decision = {"action": "chat", "book_id": None, "reply": "我还不能确定是哪本书，请再说一下书名或内容。"}
    return decision


def audio_has_speech(path, args):
    with wave.open(str(path), "rb") as stream:
        samples = array.array("h")
        samples.frombytes(stream.readframes(stream.getnframes()))
    if not samples:
        return False
    peak = max(abs(value) for value in samples)
    rms = math.sqrt(sum(value * value for value in samples) / len(samples))
    peak_dbfs = 20 * math.log10(max(peak, 1) / 32768.0)
    rms_dbfs = 20 * math.log10(max(rms, 1) / 32768.0)
    audible = rms_dbfs >= args.vad_rms_dbfs and peak_dbfs >= args.vad_peak_dbfs
    print(f"VAD: rms={rms_dbfs:.1f}dBFS peak={peak_dbfs:.1f}dBFS speech={audible}", flush=True)
    return audible


class Ci302WakeListener:
    def __init__(self, host, port):
        self.sleep_event = threading.Event()
        self.lock = threading.Lock()
        self.wake_condition = threading.Condition(self.lock)
        self.wake_generation = 0
        self.task_generation = 0
        self.pending_candidate = None
        self.change_event = threading.Event()
        self.session_active = False
        listener = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path != "/health": self.send_error(404); return
                body=b'{"ok":true,"state":"ready"}'
                self.send_response(200); self.send_header("Content-Type","application/json")
                self.send_header("Content-Length",str(len(body))); self.end_headers(); self.wfile.write(body)

            def do_POST(self):
                if self.path != "/wake":
                    self.send_error(404)
                    return
                length = int(self.headers.get("Content-Length", "0"))
                raw = self.rfile.read(length) if length else b"{}"
                try:
                    payload = json.loads(raw.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    payload = {}
                event = str(payload.get("event", "wake"))
                if not listener.handle_event(event, payload):
                    self.send_error(400, "unknown CI302 event")
                    return
                body = b'{"ok":true}'
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, _format, *_args):
                return

        self.server = ThreadingHTTPServer((host, port), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    def start(self):
        self.thread.start()

    def handle_event(self, event, payload=None):
        payload = payload or {}
        if event == "wake":
            with self.wake_condition:
                self.wake_generation += 1
                self.session_active = True
                self.pending_candidate = None
                self.sleep_event.clear()
                self.wake_condition.notify_all()
            self.change_event.set()
            return True
        if event == "sleep":
            with self.wake_condition:
                self.session_active = False
                self.pending_candidate = None
                self.wake_condition.notify_all()
            self.sleep_event.set()
            self.change_event.set()
            return True
        if event in ("command", "find_candidate"):
            with self.lock:
                if event == "find_candidate":
                    self.pending_candidate={k:payload.get(k) for k in
                        ("book_id","command_id","received_at","source")}
                    self.pending_candidate["candidate_received_at"]=time.time()
                else:
                    self.task_generation += 1
                    self.pending_candidate=None
            if event == "find_candidate":
                candidate=self.peek_candidate()
                print("find_candidate_received "
                      f"book_id={candidate.get('book_id')} "
                      "recording_continues=true no_dispatch_reserve=true",flush=True)
            if event == "command": self.change_event.set()
            return True
        return False

    def wait_for_wake(self, after_generation=0):
        with self.wake_condition:
            while self.wake_generation <= int(after_generation):
                self.wake_condition.wait()
            self.session_active = True
            return self.wake_generation

    def current_wake(self, generation):
        with self.lock:
            return (self.session_active and not self.sleep_event.is_set()
                    and self.wake_generation == int(generation))

    def latest_wake_generation(self):
        with self.lock: return self.wake_generation

    def current_task_generation(self):
        with self.lock: return self.task_generation

    def command_generation(self):
        """Compatibility name for callers/tests written before task separation."""
        return self.current_task_generation()

    def peek_candidate(self):
        with self.lock:
            return dict(self.pending_candidate) if self.pending_candidate else None

    def take_candidate(self):
        with self.lock:
            token=self.pending_candidate
            self.pending_candidate=None
            return token

    def discard_candidate(self):
        with self.lock: self.pending_candidate=None

    def recording_cancel_event(self):
        return self.change_event

    def clear_change(self, generation=None):
        with self.lock:
            if generation is None or self.wake_generation == int(generation):
                self.change_event.clear()

    def sleeping(self):
        return self.sleep_event.is_set()

    def stop(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)


def reserve_command(args, command_type="asr"):
    return post_json(args.web_url.rstrip("/")+"/api/demo/command/reserve",
                     {"source":"asr","command_type":command_type,"request_id":str(uuid.uuid4())},timeout=5)


class AsrTokenReservation:
    """One-shot task token owned by one recorded utterance."""
    def __init__(self, args, listener, wake_generation, task_generation):
        self.args=args; self.listener=listener
        self.wake_generation=int(wake_generation); self.task_generation=int(task_generation)
        self._condition=threading.Condition(); self._attempted=False; self._reserving=False
        self._invalid=False; self._token=None; self._error=None

    def _session_valid(self):
        return (self.listener.current_wake(self.wake_generation)
                and self.listener.current_task_generation()==self.task_generation)

    def invalidate(self):
        with self._condition:
            self._invalid=True; self._token=None; self._condition.notify_all()

    def reserve_once(self):
        with self._condition:
            if self._attempted:
                return self._token
            self._attempted=True; self._reserving=True
        token=None; error=None
        try: token=reserve_command(self.args,"asr_utterance")
        except Exception as exc: error=exc
        with self._condition:
            if not self._invalid and self._session_valid() and error is None:
                self._token=token
                print(f"asr_token_reserved generation={token['generation']} "
                      f"request_id={token['request_id']}",flush=True)
            self._error=error; self._reserving=False; self._condition.notify_all()
            return self._token

    def wait_or_reserve(self, timeout=0.75):
        with self._condition:
            if self._invalid or not self._session_valid():
                self._invalid=True; self._token=None; return None
            attempted=self._attempted
            if attempted and self._reserving:
                self._condition.wait_for(lambda:not self._reserving or self._invalid,timeout)
            should_reserve=not self._attempted and not self._invalid
        if should_reserve:
            self.reserve_once()
        with self._condition:
            # Never launch a second reserve while the VAD callback is still
            # running.  Wait for that one request instead.
            if self._reserving:
                self._condition.wait_for(lambda:not self._reserving or self._invalid,5.5)
            if self._invalid or not self._session_valid():
                self._invalid=True; self._token=None; return None
            if self._error: raise RuntimeError(f"ASR任务token创建失败：{self._error}")
            return dict(self._token) if self._token else None


def submit_command(args, token, command):
    payload={**token,"command":command}
    return post_json(args.web_url.rstrip("/")+"/api/demo/command",payload,timeout=5)


def decision_to_command(decision, standard_by_id, catalog):
    if decision.get("route") == "standard": return dict(standard_by_id[decision["command_id"]])
    action=decision.get("action")
    if action in ("find_book","introduce_book"): return {"kind":action,"book_id":int(decision["book_id"])}
    if action=="cancel": return {"kind":"cancel"}
    if action=="recommend" and catalog.get(decision.get("book_id")):
        book=catalog.get(decision["book_id"]); return {"kind":"speak","text":f"推荐《{book['name']}》。{book['summary']}"}
    if action=="list_books": return {"kind":"speak","text":"目前可以查找："+"、".join(x["name"] for x in catalog.books)+"。"}
    reply=str(decision.get("reply") or "请明确告诉我需要介绍还是寻找哪本书。")[:120]
    return {"kind":"speak","text":reply}


def resolve_command(text, catalog, standards, standard_by_id, args, history, candidate=None):
    exact=exact_standard_command(text,standards)
    if exact: return dict(exact),{"route":"local_standard","command_id":exact["id"]}
    local=classify_local_intent(text,catalog)
    candidate_book=catalog.get(candidate.get("book_id")) if candidate else None
    matched_book_id=local.book_id or (candidate_book["id"] if candidate_book else None)
    if local.kind in ("introduce_book","find_book") and matched_book_id:
        return {"kind":local.kind,"book_id":matched_book_id},{
            "route":"local_deterministic","action":local.kind,"book_id":matched_book_id,
            "book_source":"asr" if local.book_id else "ci302_candidate",
        }
    if local.kind=="clarify": return {"kind":"speak","text":"请明确说介绍这本书，或者寻找这本书。"},{"route":"local_clarify"}
    decision=interpret_dialogue(text,catalog,standards,args,history)
    if decision.get("route")=="standard":
        selected=standard_by_id[decision["command_id"]]
        decision={**decision,"action":selected.get("kind"),"book_id":selected.get("book_id")}
    safe=enforce_execution_safety(text,decision,catalog)
    return decision_to_command(safe,standard_by_id,catalog),safe


class TtsStillBusy(RuntimeError):
    pass


def wait_tts_idle(args, listener, wake_generation, task_generation,
                  session_deadline, timeout=3.0):
    deadline=min(float(session_deadline),time.monotonic()+min(float(timeout),3.0))
    while time.monotonic()<deadline:
        if (not listener.current_wake(wake_generation)
                or listener.current_task_generation()!=task_generation):
            return False
        try:
            # Demo state may wait up to 1.5 s for its navigation backend before
            # returning a degraded snapshot.  A shorter client timeout turns a
            # healthy TTS connection into a false voice-state failure.
            with urllib.request.urlopen(args.web_url.rstrip("/")+"/api/demo/state",timeout=2.5) as response:
                state=json.loads(response.read())
        except Exception as exc:
            raise RuntimeError(f"读取TTS状态失败：{exc}") from exc
        if "tts" not in state or not isinstance(state["tts"],dict):
            raise RuntimeError("/api/demo/state 缺少顶层 tts 状态")
        tts=state["tts"]
        current=tts.get("current"); queued=tts.get("queued")
        if current=="idle" and queued==0:
            return True
        if current not in ("idle","queued","speaking","accepted") or not isinstance(queued,int):
            raise RuntimeError(f"TTS状态异常：current={current!r}, queued={queued!r}")
        listener.change_event.wait(min(0.1,max(0.0,deadline-time.monotonic())))
    if not listener.current_wake(wake_generation):
        return False
    raise TtsStillBusy("TTS仍在播报（单次等待最多3秒）")


def wait_tts_idle_in_chunks(args, listener, wake_generation, task_generation,
                            session_deadline):
    """Wait through long valid playback without extending the session deadline."""
    while (listener.current_wake(wake_generation)
           and listener.current_task_generation()==task_generation
           and time.monotonic()<session_deadline):
        try:
            return wait_tts_idle(
                args,listener,wake_generation,task_generation,session_deadline)
        except TtsStillBusy:
            print("tts_busy_wait_continue",flush=True)
    return False


def report_dispatcher_result(result, token, command):
    accepted=bool(isinstance(result,dict) and result.get("accepted"))
    if accepted:
        print(f"dispatcher_submit accepted=true generation={token['generation']} "
              f"request_id={token['request_id']} kind={command['kind']}",flush=True)
        return True
    reason=result.get("reason") if isinstance(result,dict) else "invalid_response"
    server_generation=result.get("server_generation") if isinstance(result,dict) else None
    print(f"dispatcher_submit accepted=false generation={token.get('generation')} "
          f"request_id={token.get('request_id')} server_generation={server_generation} "
          f"reason={reason or 'unknown'}",flush=True)
    return False


def main():
    args=parse_args()
    if not args.api_key: print("缺少ZHIPU_API_KEY，语音服务拒绝启动。",file=sys.stderr); return 2
    catalog=BookCatalog(ROOT/"config"); standards=build_standard_commands(catalog); by_id={x["id"]:x for x in standards}
    history=[]; listener=Ci302WakeListener(args.wake_host,args.wake_port); listener.start()
    print(f"已从{catalog.source_path}载入{len(catalog.books)}本书。",flush=True)
    last_wake_generation=0
    try:
        while True:
            wake_generation=listener.wait_for_wake(last_wake_generation)
            last_wake_generation=wake_generation
            listener.clear_change(wake_generation)
            session_deadline=time.monotonic()+args.session_timeout
            print("收到CI302唤醒帧；CI302无播放完成帧，使用集中配置的确认音保守延时。",flush=True)
            # CI302固件没有提供播放完成帧，仅在此处集中降级；新 wake/sleep
            # 会中断等待并由外层循环只处理最新 wake generation。
            listener.change_event.wait(args.wake_response_delay)
            if not listener.current_wake(wake_generation):
                continue
            try:
                prompt=post_json(args.web_url.rstrip("/")+"/api/demo/prompt",{"text":"请说。"},timeout=130)
            except Exception as exc:
                print(f"prompt_failed error={exc}",file=sys.stderr,flush=True)
                continue
            if not listener.current_wake(wake_generation):
                continue
            if not prompt.get("completed"):
                print("prompt_failed error=HS-S77未完成‘请说’",file=sys.stderr,flush=True)
                continue
            print("prompt_completed",flush=True)
            handled_task_generation=listener.current_task_generation()
            prompt_just_completed=True
            while listener.current_wake(wake_generation) and time.monotonic()<session_deadline:
                if prompt_just_completed:
                    # /api/demo/prompt completed means HS-S77 already returned 0x4F.
                    prompt_just_completed=False
                else:
                    try:
                        idle=wait_tts_idle_in_chunks(
                            args,listener,wake_generation,
                            handled_task_generation,session_deadline)
                    except RuntimeError as exc:
                        print(f"voice_state_error {exc}",file=sys.stderr,flush=True)
                        break
                    if not idle: break
                if not listener.current_wake(wake_generation): break
                listener.clear_change(wake_generation)
                utterance_token=AsrTokenReservation(
                    args,listener,wake_generation,handled_task_generation)
                def on_speech(reservation=utterance_token):
                    print("speech_detected",flush=True)
                    reservation.reserve_once()
                print(f"recording_started wake_generation={wake_generation}",flush=True)
                try:
                    wav=record_file_with_retry(
                        args,args.command_seconds,"command_",
                        listener.recording_cancel_event(),on_speech=on_speech)
                except RuntimeError as exc:
                    print(f"recording_cancelled reason=audio_device_error error={exc}",
                          file=sys.stderr,flush=True)
                    utterance_token.invalidate()
                    listener.discard_candidate()
                    break
                if wav is None:
                    utterance_token.invalidate()
                    continue
                if not listener.current_wake(wake_generation):
                    reason="sleep" if listener.sleeping() else "new_wake"
                    print(f"recording_cancelled reason={reason}",flush=True)
                    utterance_token.invalidate()
                    wav.unlink(missing_ok=True); continue
                current_task_generation=listener.current_task_generation()
                if current_task_generation != handled_task_generation:
                    handled_task_generation=current_task_generation
                    print("recording_cancelled reason=fixed_command",flush=True)
                    utterance_token.invalidate()
                    wav.unlink(missing_ok=True); continue
                print(f"recording_completed wake_generation={wake_generation}",flush=True)
                has_speech=audio_has_speech(wav,args)
                print(f"vad_result speech={str(has_speech).lower()}",flush=True)
                if not has_speech:
                    utterance_token.invalidate()
                    listener.discard_candidate(); wav.unlink(missing_ok=True); continue
                try:
                    text=transcribe(wav,args,allow_empty=True)
                except Exception as exc:
                    wav.unlink(missing_ok=True)
                    utterance_token.invalidate()
                    listener.discard_candidate()
                    print(f"asr_failed error={exc}",file=sys.stderr,flush=True)
                    continue
                finally:
                    if not args.save_recordings: wav.unlink(missing_ok=True)
                    else: prune_private_files(RECORDINGS)
                if not listener.current_wake(wake_generation):
                    print("recording_cancelled reason=new_wake_or_sleep_after_asr",flush=True)
                    utterance_token.invalidate()
                    continue
                print(f"asr_text {text}",flush=True)
                if args.debug_transcripts and text:
                    TRANSCRIPTS.mkdir(parents=True,exist_ok=True); os.chmod(TRANSCRIPTS,0o700)
                    out=TRANSCRIPTS/(uuid.uuid4().hex+".txt"); out.write_text(text+"\n",encoding="utf-8"); os.chmod(out,0o600)
                    prune_private_files(TRANSCRIPTS)
                if not text:
                    utterance_token.invalidate()
                    listener.discard_candidate()
                    continue
                candidate=listener.take_candidate()
                try:
                    command,decision=resolve_command(
                        text,catalog,standards,by_id,args,history,candidate)
                except Exception as exc:
                    utterance_token.invalidate()
                    print(f"intent_failed error={exc}",file=sys.stderr,flush=True)
                    continue
                print("intent_result "+json.dumps(
                    {"kind":command.get("kind"),"book_id":command.get("book_id"),
                     "candidate_book_id":candidate.get("book_id") if candidate else None},
                    ensure_ascii=False),flush=True)
                if (not listener.current_wake(wake_generation)
                        or listener.current_task_generation()!=handled_task_generation):
                    utterance_token.invalidate()
                    continue
                try:
                    token=utterance_token.wait_or_reserve()
                except Exception as exc:
                    print(f"dispatcher_reserve_failed error={exc}",file=sys.stderr,flush=True)
                    continue
                if not token:
                    print("dispatcher_submit skipped reason=missing_asr_token",flush=True)
                    continue
                if (not listener.current_wake(wake_generation)
                        or listener.current_task_generation()!=handled_task_generation):
                    utterance_token.invalidate()
                    continue
                try: result=submit_command(args,token,command)
                except urllib.error.HTTPError as exc:
                    if exc.code==409: print("旧ASR结果已被服务端拒绝。",flush=True); continue
                    print(f"dispatcher_submit_failed error={exc}",file=sys.stderr,flush=True)
                    continue
                except Exception as exc:
                    print(f"dispatcher_submit_failed error={exc}",file=sys.stderr,flush=True)
                    continue
                if not report_dispatcher_result(result,token,command):
                    continue
                history=(history+[{"role":"user","content":text},
                                  {"role":"assistant","content":json.dumps(decision,ensure_ascii=False)}])[-6:]
                if args.once: return 0
            print("语音会话结束；已开始的机器人任务保持原状态。",flush=True)
    except KeyboardInterrupt: return 130
    except Exception as exc: print(f"语音会话失败：{exc}",file=sys.stderr); return 1
    finally: listener.stop()


if __name__ == "__main__":
    raise SystemExit(main())
