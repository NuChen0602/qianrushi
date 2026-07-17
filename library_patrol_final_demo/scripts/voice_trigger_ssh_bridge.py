#!/usr/bin/env python3
import argparse, json, queue, subprocess, sys, threading, time, urllib.request
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT))
from app.book_catalog import BookCatalog
from app.ci302_commands import WAKE_FRAME,SLEEP_FRAMES,commands_by_frame


def post(url,payload,timeout=3):
    req=urllib.request.Request(url,json.dumps(payload,ensure_ascii=False).encode(),{"Content-Type":"application/json"},method="POST")
    with urllib.request.urlopen(req,timeout=timeout) as response: return json.loads(response.read() or b"{}")


class VoiceBridge:
    """Serial reader only parses frames; HTTP is isolated from serial input."""
    def __init__(self,board,user,serial_port,baudrate,web_url,wake_url,debounce=1.0,ack_delay=1.0):
        self.target=f"{user}@{board}"; self.serial=serial_port; self.baudrate=baudrate
        self.web=web_url.rstrip("/"); self.wake_url=wake_url; self.commands=commands_by_frame(BookCatalog(ROOT/"config"))
        self.running=True; self.last={}; self.events=queue.Queue(); self.session_events=queue.Queue(); self.debounce=debounce
        self.ack_delay=max(0.0,float(ack_delay))
        self.epoch=0; self.epoch_lock=threading.Lock()
        self.sequence=0; self.latest_command_sequence=0; self.sequence_lock=threading.Lock()
        self.serial_process=None; self.threads=[]

    def setup(self):
        return ("/etc/init.d/S99_voice_motion_test stop 2>/dev/null || true; "
                "killall voice_motion_test 2>/dev/null || true; "
                f"stty -F {self.serial} {self.baudrate} cs8 -cstopb -parenb raw -echo")
    def enqueue(self,event):
        with self.sequence_lock:
            self.sequence+=1; event["sequence"]=self.sequence
            if event.get("event")=="command" or event.get("event")=="sleep":
                self.latest_command_sequence=self.sequence
        is_session=event.get("event") in ("wake","sleep") or event.get("command",{}).get("kind")=="sleep"
        target=self.session_events if is_session else self.events
        target.put_nowait(event)

    def submit_after_ack(self, token, command, epoch, sequence):
        """CI302 has no playback-complete frame; delay without blocking newer events."""
        if command.get("kind") != "cancel":
            time.sleep(self.ack_delay)
        with self.epoch_lock:
            if epoch != self.epoch:
                return
        with self.sequence_lock:
            if sequence != self.latest_command_sequence:
                return
        try:
            result=self.reliable_post(self.web+"/api/demo/command",{**token,"command":command})
            print(f"[ci302] {command['id']} accepted={result.get('accepted')} generation={token['generation']}",flush=True)
        except Exception as exc:
            print(f"[event] delayed command failed: {exc}",file=sys.stderr,flush=True)

    def handle_frame(self,frame):
        text=" ".join(f"{b:02X}" for b in frame); now=time.monotonic()
        if now-self.last.get(text,0)<self.debounce: return
        self.last[text]=now
        if text==WAKE_FRAME: self.enqueue({"event":"wake","frame":text}); return
        if text in SLEEP_FRAMES: self.enqueue({"event":"sleep","frame":text}); return
        command=self.commands.get(text)
        if command: self.enqueue({"event":"command","frame":text,"command":command})
        else: print(f"[serial] unknown frame {text}",flush=True)

    def parse(self,stream):
        buf=bytearray()
        while self.running:
            chunk=stream.read(1)
            # A blocking pipe returns b"" only after ssh/cat has exited.  Return
            # to serial_loop so it can reap the child and reconnect; otherwise a
            # board reboot leaves the bridge spinning forever on a dead pipe.
            if not chunk: return
            buf.extend(chunk)
            while len(buf)>=5:
                index=buf.find(b"\xaa\x55")
                if index<0: del buf[:-1]; break
                if index: del buf[:index]
                if len(buf)<5: break
                candidate=bytes(buf[:5])
                if candidate[4]==0xfb: del buf[:5]; self.handle_frame(candidate)
                else: del buf[0]

    def serial_loop(self):
        while self.running:
            proc=None
            try:
                proc=subprocess.Popen(["ssh","-o","BatchMode=yes","-o","ConnectTimeout=8",
                    "-o","ConnectionAttempts=2","-o","ServerAliveInterval=3",
                    "-o","ServerAliveCountMax=2",self.target,
                    self.setup()+f"; exec cat {self.serial}"],stdout=subprocess.PIPE,stderr=subprocess.DEVNULL)
                self.serial_process=proc
                self.parse(proc.stdout)
            except Exception as exc: print(f"[serial] {exc}",file=sys.stderr,flush=True)
            finally:
                if proc:
                    if proc.poll() is None: proc.terminate()
                    try: proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        proc.kill(); proc.wait(timeout=2)
                self.serial_process=None
            time.sleep(1)

    def reliable_post(self,url,payload):
        error=None
        for delay in (0,.15,.5):
            if delay: time.sleep(delay)
            try: return post(url,payload)
            except Exception as exc: error=exc
        raise error

    def process_event(self,event):
        kind=event["event"]
        if kind in ("wake","sleep"):
            if kind == "sleep":
                with self.epoch_lock: self.epoch += 1
            self.reliable_post(self.wake_url,{"event":kind,"source":"ci302","frame":event["frame"]})
            print(f"[ci302] {kind}",flush=True); return
        command=event["command"]
        with self.sequence_lock:
            if event["sequence"] != self.latest_command_sequence: return
        if command.get("kind")=="sleep":
            with self.epoch_lock: self.epoch += 1
            self.reliable_post(self.wake_url,{"event":"sleep","source":"ci302","frame":event["frame"]})
            return
        # Every CI302 command, including a recognised book-search phrase, is
        # authoritative.  A FIND_<book_id> frame therefore starts navigation
        # directly instead of waiting for a second ASR interpretation.
        token=self.reliable_post(self.web+"/api/demo/command/reserve",{"source":"ci302","command_type":command["kind"]})
        self.reliable_post(self.wake_url,{"event":"command","source":"ci302","command_id":command["id"],"generation":token["generation"]})
        with self.epoch_lock:
            self.epoch += 1; epoch=self.epoch
        threading.Thread(target=self.submit_after_ack,args=(token,command,epoch,event["sequence"]),daemon=True).start()

    def event_loop(self, event_queue):
        while self.running:
            try: event=event_queue.get(timeout=.2)
            except queue.Empty: continue
            try: self.process_event(event)
            except Exception as exc: print(f"[event] {exc}",file=sys.stderr,flush=True)

    def run(self):
        try:
            subprocess.run(["ssh","-o","BatchMode=yes","-o","ConnectTimeout=8",self.target,
                self.setup()],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=12)
        except (OSError,subprocess.SubprocessError) as exc:
            print(f"[serial] initial setup deferred: {exc}",file=sys.stderr,flush=True)
        self.threads=[threading.Thread(target=self.serial_loop,name="ci302-serial",daemon=True),
                      threading.Thread(target=self.event_loop,args=(self.events,),name="ci302-commands",daemon=True),
                      threading.Thread(target=self.event_loop,args=(self.session_events,),name="ci302-session",daemon=True)]
        for thread in self.threads: thread.start()
        try:
            while True: time.sleep(1)
        except KeyboardInterrupt: pass
        finally:
            self.running=False
            if self.serial_process and self.serial_process.poll() is None: self.serial_process.terminate()
            for thread in self.threads: thread.join(timeout=3)


def main():
    p=argparse.ArgumentParser(); p.add_argument("--board",default="192.168.43.192"); p.add_argument("--user",default="root")
    p.add_argument("--serial",default="/dev/ttyS1"); p.add_argument("--baudrate",default="115200")
    p.add_argument("--web",default="http://127.0.0.1:8090"); p.add_argument("--wake-url",default="http://127.0.0.1:8092/wake")
    p.add_argument("--input-only",action="store_true",help="deprecated; HS-S77 is always the output device")
    p.add_argument("--ack-delay",type=float,default=float(__import__("os").getenv("CI302_ACK_FALLBACK_SECONDS","1.0")))
    args=p.parse_args(); VoiceBridge(args.board,args.user,args.serial,args.baudrate,args.web,args.wake_url,ack_delay=args.ack_delay).run()
if __name__=="__main__": main()
