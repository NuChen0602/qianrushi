#!/usr/bin/env python3
import argparse
import json
import re
import subprocess
import sys
import threading
import time
import urllib.request

# === final demo hard guard: hazard voice must be fire risk FF82 ===
def _force_fire_frame_if_hazard(frame):
    try:
        if isinstance(frame, (bytes, bytearray)):
            b = bytes(frame)
            if len(b) == 5 and b[0] == 0xAA and b[1] == 0x55 and b[2] == 0xFF and b[4] == 0xFB:
                # 7F/80/81/83/84 都是高危巡检相关播报，统一改成 FF82 起火风险
                if b[3] in (0x7F, 0x80, 0x81, 0x83, 0x84):
                    return bytes([0xAA, 0x55, 0xFF, 0x82, 0xFB])
        if isinstance(frame, str):
            compact = frame.replace(" ", "").upper()
            if compact in ("AA55FF7FFB", "AA55FF80FB", "AA55FF81FB", "AA55FF83FB", "AA55FF84FB"):
                return "AA 55 FF 82 FB"
    except Exception:
        pass
    return frame



COMMAND_FRAME_TO_MISSION = {
    "AA 55 00 A1 FB": "RECOMMEND_BOOKS",
    "AA 55 00 A2 FB": "FIND_BOOK",
    "AA 55 00 A3 FB": "INTRO_BOOK",
    "AA 55 00 A4 FB": "SHELF_CHECK",
    "AA 55 00 A5 FB": "LOST_ITEM_PATROL",
    "AA 55 00 A6 FB": "HAZARD_CHECK",
    "AA 55 00 A7 FB": "LOST_ITEM_PATROL",
    "AA 55 00 A8 FB": "RETURN_HOME",
    "AA 55 00 A9 FB": "CANCEL",
}


def frame_to_text(frame: bytes) -> str:
    return " ".join(f"{b:02X}" for b in frame)


def post_json(url: str, payload: dict, timeout: float = 3.0):
    data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        raw = resp.read().decode("utf-8", errors="replace")
        return json.loads(raw) if raw else {}


def get_json(url: str, timeout: float = 3.0):
    with urllib.request.urlopen(url, timeout=timeout) as resp:
        raw = resp.read().decode("utf-8", errors="replace")
        return json.loads(raw)


class VoiceBridge:
    def __init__(self, board, user, serial_port, baudrate, web_url, debounce_sec=1.5):
        self.board = board
        self.user = user
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.web_url = web_url.rstrip("/")
        self.debounce_sec = debounce_sec
        self.running = True
        self.last_trigger = {}
        self.seen_voice_events = set()

    @property
    def ssh_target(self):
        return f"{self.user}@{self.board}"

    def remote_serial_setup(self):
        return (
            f"stty -F {self.serial_port} {self.baudrate} "
            f"cs8 -cstopb -parenb raw -echo"
        )

    def stop_old_board_voice_process(self):
        cmd = (
            "/etc/init.d/S99_voice_motion_test stop 2>/dev/null; "
            "killall voice_motion_test 2>/dev/null; "
            f"{self.remote_serial_setup()}"
        )
        subprocess.run(
            ["ssh", self.ssh_target, cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=5,
        )

    def trigger_mission(self, mission):
        if mission == "CANCEL":
            url = f"{self.web_url}/api/demo/cancel"
            payload = {}
        else:
            url = f"{self.web_url}/api/demo/mission"
            payload = {"mission": mission}

        result = post_json(url, payload)
        print(f"[mission] {mission} -> {result}", flush=True)

    def handle_command_frame(self, frame: bytes):
        text = frame_to_text(frame)
        mission = COMMAND_FRAME_TO_MISSION.get(text)
        if not mission:
            print(f"[voice] unknown frame: {text}", flush=True)
            return

        now = time.time()
        last = self.last_trigger.get(text, 0)
        if now - last < self.debounce_sec:
            print(f"[voice] ignored duplicate: {text}", flush=True)
            return
        self.last_trigger[text] = now

        print(f"[voice] {text} -> {mission}", flush=True)
        try:
            self.trigger_mission(mission)
        except Exception as exc:
            print(f"[error] trigger mission failed: {exc}", flush=True)

    def parse_stream(self, stream):
        buf = bytearray()
        while self.running:
            chunk = stream.read(1)
            if not chunk:
                time.sleep(0.05)
                continue
            buf.extend(chunk)

            if len(buf) > 128:
                del buf[:-32]

            while True:
                idx = buf.find(b"\xAA\x55")
                if idx < 0:
                    if len(buf) > 2:
                        del buf[:-1]
                    break

                if idx > 0:
                    del buf[:idx]

                if len(buf) < 5:
                    break

                candidate = bytes(buf[:5])
                if candidate[4] == 0xFB:
                    del buf[:5]
                    self.handle_command_frame(candidate)
                else:
                    del buf[0]

    def read_serial_loop(self):
        while self.running:
            cmd = f"{self.remote_serial_setup()}; cat {self.serial_port}"
            print(f"[serial] ssh {self.ssh_target}: {cmd}", flush=True)
            proc = None
            try:
                proc = subprocess.Popen(
                    ["ssh", self.ssh_target, cmd],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                )
                self.parse_stream(proc.stdout)
            except Exception as exc:
                print(f"[serial] reader error: {exc}", flush=True)
                time.sleep(1.0)
            finally:
                if proc:
                    proc.terminate()
                    try:
                        proc.wait(timeout=1)
                    except subprocess.TimeoutExpired:
                        proc.kill()
            time.sleep(1.0)

    def send_frame_to_ci1302(self, frame_text: str):
        bs = bytes(int(x, 16) for x in frame_text.split())
        escaped = "".join(f"\\x{b:02X}" for b in bs)
        cmd = (
            f"{self.remote_serial_setup()}; "
            f"printf '{escaped}' > {self.serial_port}"
        )
        subprocess.run(
            ["ssh", self.ssh_target, cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=5,
        )
        print(f"[speak] sent {frame_text}", flush=True)

    def init_seen_voice_events(self):
        try:
            state = get_json(f"{self.web_url}/api/demo/state")
            events = state.get("work_order", {}).get("events", [])
            for ev in events:
                key = f'{ev.get("time")}|{ev.get("source")}|{ev.get("text")}'
                self.seen_voice_events.add(key)
        except Exception:
            pass

    def speech_event_loop(self):
        pattern = re.compile(r"AA 55 FF [0-9A-Fa-f]{2} FB")
        self.init_seen_voice_events()

        while self.running:
            try:
                state = get_json(f"{self.web_url}/api/demo/state", timeout=2.0)
                events = state.get("work_order", {}).get("events", [])

                for ev in reversed(events):
                    key = f'{ev.get("time")}|{ev.get("source")}|{ev.get("text")}'
                    if key in self.seen_voice_events:
                        continue
                    self.seen_voice_events.add(key)

                    if ev.get("source") != "voice":
                        continue

                    text = ev.get("text", "")
                    match = pattern.search(text)
                    if match:
                        frame_text = match.group(0).upper()
                        self.send_frame_to_ci1302(frame_text)

            except Exception as exc:
                print(f"[speech-poll] {exc}", flush=True)

            time.sleep(0.3)

    def run(self):
        print("[bridge] stopping old board voice process if any...", flush=True)
        self.stop_old_board_voice_process()

        print("[bridge] starting voice bridge", flush=True)
        print(f"[bridge] board={self.board}, serial={self.serial_port}, web={self.web_url}", flush=True)
        print("[bridge] command map:", flush=True)
        for frame, mission in COMMAND_FRAME_TO_MISSION.items():
            print(f"  {frame} -> {mission}", flush=True)

        t1 = threading.Thread(target=self.read_serial_loop, daemon=True)
        t2 = threading.Thread(target=self.speech_event_loop, daemon=True)
        t1.start()
        t2.start()

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            self.running = False
            print("\n[bridge] stopped", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", default="192.168.43.192")
    parser.add_argument("--user", default="root")
    parser.add_argument("--serial", default="/dev/ttyS1")
    parser.add_argument("--baudrate", default="115200")
    parser.add_argument("--web", default="http://127.0.0.1:8090")
    args = parser.parse_args()

    bridge = VoiceBridge(
        board=args.board,
        user=args.user,
        serial_port=args.serial,
        baudrate=args.baudrate,
        web_url=args.web,
    )
    bridge.run()


if __name__ == "__main__":
    main()
