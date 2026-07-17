import queue
import threading
import time
import uuid


class CommandDispatcher:
    """Single authority for command generations and latest-wins execution."""

    def __init__(self, orchestrator, work_order):
        self.orchestrator=orchestrator; self.work_order=work_order
        self._lock=threading.RLock(); self._generation=0; self._request_id=""; self._emergency=False
        self._queue=queue.Queue(); self._running=True
        self.orchestrator.set_token_checker(self.is_current)
        self._thread=threading.Thread(target=self._worker,name="command-dispatcher",daemon=True); self._thread.start()

    def reserve(self, source, command_type="input", request_id=None):
        with self._lock:
            self._generation+=1; self._request_id=str(request_id or uuid.uuid4())
            token={"request_id":self._request_id,"generation":self._generation,"source":str(source),
                   "received_at":time.time(),"command_type":str(command_type)}
        self.work_order.set_command_context(token["request_id"],token["generation"])
        self.orchestrator.invalidate(token["generation"],token["request_id"])
        return token

    def is_current(self, generation, request_id):
        try: generation=int(generation)
        except (TypeError,ValueError): return False
        with self._lock: return generation==self._generation and str(request_id)==self._request_id

    def _emergency_active(self):
        with self._lock: local=self._emergency
        return local or bool(getattr(self.orchestrator,"emergency_stopped",False))

    def submit(self, token, command):
        if not self.is_current(token.get("generation"),token.get("request_id")):
            with self._lock:
                current_generation=self._generation; current_request_id=self._request_id
            return {"ok":False,"accepted":False,"reason":"stale_generation",**token,
                    "server_generation":current_generation,
                    "server_request_id":current_request_id}
        if self._emergency_active() and command.get("kind") not in ("cancel",):
            return {"ok":False,"accepted":False,"reason":"emergency_stopped",**token}
        self._queue.put((dict(token),dict(command)))
        return {"ok":True,"accepted":True,"reason":"queued",**token}

    def accept(self, source, command):
        token=self.reserve(source,command.get("kind","command")); return self.submit(token,command)

    def execute_now(self, source, command):
        """Execute a command synchronously while retaining generation checks."""
        token=self.reserve(source,command.get("kind","command"))
        if self._emergency_active() and command.get("kind") != "cancel":
            return {"ok":False,"accepted":False,"completed":False,"reason":"emergency_stopped",**token}
        completed=bool(self.orchestrator.execute_command(command,token["generation"],token["request_id"]))
        return {"ok":completed,"accepted":True,"completed":completed,
                "reason":"completed" if completed else "execution_failed",**token}

    def emergency_stop(self, source="api"):
        token=self.reserve(source,"emergency_stop")
        with self._lock: self._emergency=True
        self.orchestrator.emergency_stop(token["generation"],token["request_id"])
        return {"ok":True,"accepted":True,"reason":"emergency_stopped",**token}

    def emergency_release(self, source="api"):
        token=self.reserve(source,"emergency_release")
        self.orchestrator.emergency_release(token["generation"],token["request_id"])
        with self._lock: self._emergency=False
        return {"ok":True,"accepted":True,"reason":"idle",**token}

    def snapshot(self):
        with self._lock: snapshot={"generation":self._generation,"request_id":self._request_id}
        snapshot["emergency_stopped"]=self._emergency_active()
        return snapshot

    def _worker(self):
        while self._running:
            try: token,command=self._queue.get(timeout=0.2)
            except queue.Empty: continue
            if not self.is_current(token["generation"],token["request_id"]): continue
            try: self.orchestrator.execute_command(command,token["generation"],token["request_id"])
            except Exception as exc:
                if self.is_current(token["generation"],token["request_id"]): self.work_order.set_error(f"命令执行失败：{exc}")

    def close(self):
        if not self._running: return
        self.reserve("shutdown","shutdown")
        self._running=False; self._thread.join(timeout=3)
