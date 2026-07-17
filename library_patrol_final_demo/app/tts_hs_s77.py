import json
import queue
import select
import shlex
import subprocess
import threading
import time
from dataclasses import dataclass, field


REMOTE_UART_HELPER = r'''
import json, os, select, struct, sys, termios, time
dev=sys.argv[1]; baud=int(sys.argv[2])
fd=os.open(dev, os.O_RDWR|os.O_NOCTTY)
a=termios.tcgetattr(fd); a[0]=0; a[1]=0; a[2]=termios.CLOCAL|termios.CREAD|termios.CS8; a[3]=0
a[2]&=~termios.PARENB; a[2]&=~termios.CSTOPB
if hasattr(termios,'CRTSCTS'): a[2]&=~termios.CRTSCTS
speed=getattr(termios,'B'+str(baud)); a[4]=speed; a[5]=speed; a[6][termios.VMIN]=0; a[6][termios.VTIME]=1
termios.tcsetattr(fd,termios.TCSANOW,a); termios.tcflush(fd,termios.TCIOFLUSH)
sys.stdout.write('{"ready":true}\n'); sys.stdout.flush()
def exact(n):
 d=b''
 while len(d)<n:
  c=sys.stdin.buffer.read(n-len(d))
  if not c: raise EOFError
  d+=c
 return d
def write_all(data):
 p=0
 while p<len(data):
  n=os.write(fd,data[p:])
  if n<=0: raise OSError('short write')
  p+=n
 termios.tcdrain(fd)
while True:
 try:
  size=struct.unpack('!I',exact(4))[0]; frame=exact(size); write_all(frame)
  accepted=False; started=time.monotonic(); final=time.monotonic()+120
  while time.monotonic()<final:
   ready,_,_=select.select([fd],[],[],0.2)
   if not ready: continue
   data=os.read(fd,256)
   for value in data:
    if value==0x45: raise RuntimeError('module_error_45')
    if value==0x41: accepted=True; sys.stdout.write('{"state":"accepted"}\n'); sys.stdout.flush()
    if value==0x4f and accepted:
     sys.stdout.write('{"state":"completed"}\n'); sys.stdout.flush(); raise StopIteration
   if not accepted and time.monotonic()-started>5: raise TimeoutError('accept_timeout')
  raise TimeoutError('completion_timeout')
 except StopIteration: continue
 except EOFError: break
 except Exception as e:
  sys.stdout.write(json.dumps({'state':'error','error':str(e)})+'\n'); sys.stdout.flush()
os.close(fd)
'''


class HsS77Error(RuntimeError):
    pass


class SshUartTransport:
    def __init__(self, host, user, port, baudrate, connect_timeout=12.0):
        self.host, self.user, self.port = host, user, port
        self.baudrate, self.connect_timeout = baudrate, connect_timeout
        self.process = None
        self.stopping = False

    def _disconnect(self):
        process, self.process = self.process, None
        if not process:
            return
        if process.stdin:
            try: process.stdin.close()
            except OSError: pass
        if process.poll() is None:
            process.terminate()
            try: process.wait(timeout=2)
            except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=2)

    def close(self):
        self.stopping = True
        self._disconnect()

    def _start(self):
        if self.stopping:
            raise OSError("HS-S77 transport is closing")
        if self.process is not None and self.process.poll() is None:
            return
        self._disconnect()
        # Keep the SSH channel as the helper's stdin.  Feeding the helper source
        # through `python3 -` consumes that channel and leaves no stdin for the
        # length-prefixed speech frames sent after startup.
        command = (
            f"python3 -u -c {shlex.quote(REMOTE_UART_HELPER)} "
            f"{shlex.quote(self.port)} {self.baudrate}"
        )
        self.process = subprocess.Popen(
            ["ssh", "-o", "BatchMode=yes", "-o", f"ConnectTimeout={int(self.connect_timeout)}",
             "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=2",
             f"{self.user}@{self.host}", command],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0,
        )
        line = self._readline(self.connect_timeout)
        if not line or not json.loads(line).get("ready"):
            self._disconnect(); raise HsS77Error("HS-S77 双向串口服务启动失败")

    def start(self): self.stopping=False; self._start()

    def _readline(self, timeout):
        readable, _, _ = select.select([self.process.stdout], [], [], timeout)
        line = self.process.stdout.readline() if readable else b""
        if not line and self.process.poll() is not None:
            raise OSError("HS-S77 SSH连接已断开")
        return line

    @staticmethod
    def _write_all(stream, data):
        offset=0
        while offset<len(data):
            written=stream.write(data[offset:])
            if not written: raise OSError("HS-S77 SSH管道短写")
            offset+=written
        stream.flush()

    def transact(self, frame, accept_timeout, completion_timeout, state_callback=None):
        for attempt in range(2):
            try:
                self._start()
                packet = len(frame).to_bytes(4, "big") + frame
                self._write_all(self.process.stdin,packet)
                accepted_deadline = time.monotonic() + accept_timeout
                completion_deadline = time.monotonic() + completion_timeout
                accepted = False
                while time.monotonic() < completion_deadline:
                    timeout = max(0.05, min(0.5, completion_deadline-time.monotonic()))
                    line = self._readline(timeout)
                    if not line:
                        if not accepted and time.monotonic() >= accepted_deadline:
                            self._disconnect()
                            raise HsS77Error("等待HS-S77接受状态0x41超时")
                        continue
                    payload = json.loads(line)
                    state = payload.get("state")
                    if state_callback: state_callback(state)
                    if state == "accepted": accepted = True
                    elif state == "completed": return True
                    elif state == "error": raise HsS77Error(payload.get("error", "HS-S77错误"))
                self._disconnect()
                raise HsS77Error("等待HS-S77播放结束状态0x4F超时")
            except (BrokenPipeError, OSError, ValueError, json.JSONDecodeError):
                self._disconnect()
                if attempt: raise HsS77Error("HS-S77 SSH连接失败")
        return False


@dataclass
class SpeechRequest:
    text: str
    request_id: str = ""
    generation: int = 0
    state: str = "queued"
    error: str = ""
    done: threading.Event = field(default_factory=threading.Event)


class HsS77Tts:
    def __init__(self, config, board_config=None, transport=None):
        self.config, self.board = dict(config or {}), dict(board_config or {})
        self.enabled = bool(self.config.get("enabled", False))
        self.accept_timeout = float(self.config.get("accept_timeout", 5.0))
        self.completion_timeout = float(self.config.get("completion_timeout", 120.0))
        host = self.config.get("board_host") or self.board.get("ip")
        user = self.config.get("board_user") or self.board.get("ssh_user", "root")
        self.transport = transport or SshUartTransport(
            host, user, str(self.config.get("serial_port", "/dev/ttyS0")),
            int(self.config.get("baudrate", 115200)), float(self.config.get("connect_timeout", 12)),
        )
        self._queue = queue.Queue(); self._running = False; self._thread = None
        self._lock = threading.RLock(); self._current = None; self._minimum_generation = 0
        self._last_state = "idle"

    @staticmethod
    def build_frame(text):
        try: encoded = str(text).encode("gbk")
        except UnicodeEncodeError as exc: raise ValueError(f"文本包含GBK无法编码字符：{exc}") from exc
        payload = b"\x01\x01" + encoded
        if not text or len(payload) > 0xFFFF: raise ValueError("HS-S77文本为空或过长")
        return b"\xfd" + len(payload).to_bytes(2, "big") + payload

    def start(self):
        if not self.enabled or self._running: return
        if hasattr(self.transport, "start"): self.transport.start()
        self._running = True
        self._thread = threading.Thread(target=self._worker, name="hs-s77-tts", daemon=True); self._thread.start()

    def cancel_pending(self, minimum_generation):
        with self._lock: self._minimum_generation = max(self._minimum_generation, int(minimum_generation))

    def status(self):
        with self._lock:
            return {"enabled":self.enabled,"queued":self._queue.qsize(),
                    "current":self._current.state if self._current else "idle",
                    "last":self._last_state}

    def speak(self, text, request_id="", generation=0, timeout=None):
        if not self.enabled: return False
        if not self._running: self.start()
        request = SpeechRequest(str(text).strip(), str(request_id), int(generation))
        self._queue.put(request)
        wait_timeout = timeout or (self.accept_timeout + self.completion_timeout + 2)
        if not request.done.wait(wait_timeout): raise HsS77Error("TTS队列等待超时")
        if request.state == "completed": return True
        if request.state == "cancelled": return False
        raise HsS77Error(request.error or "HS-S77播报失败")

    def _worker(self):
        while self._running:
            try: request = self._queue.get(timeout=0.2)
            except queue.Empty: continue
            with self._lock:
                self._current = request
                if request.generation < self._minimum_generation:
                    request.state="cancelled"; self._last_state=request.state
                    request.done.set(); self._current=None; continue
            try:
                request.state="speaking"
                def state(value):
                    if value == "accepted": request.state="accepted"
                self.transport.transact(self.build_frame(request.text), self.accept_timeout,
                                        self.completion_timeout, state)
                with self._lock:
                    request.state = "cancelled" if request.generation < self._minimum_generation else "completed"
            except Exception as exc:
                request.state="error"; request.error=str(exc)
            finally:
                with self._lock: self._last_state=request.state
                request.done.set()
                with self._lock: self._current=None

    def close(self):
        self._running=False
        self.transport.close()
        while True:
            try: request=self._queue.get_nowait()
            except queue.Empty: break
            request.state="cancelled"; request.done.set()
        if self._thread: self._thread.join(timeout=3)
