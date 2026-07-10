#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import subprocess
import threading
import time
import urllib.request

CMD_HAZARD = bytes.fromhex("AA5500A6FB")
CMD_CANCEL = bytes.fromhex("AA5500A9FB")

# 只允许发送这个：起火风险
FRAME_FIRE = bytes.fromhex("AA55FF82FB")


def fmt(b):
    return " ".join(f"{x:02X}" for x in b)


def post_json(url, payload=None):
    data = None
    headers = {}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"

    req = urllib.request.Request(url, data=data, headers=headers, method="POST")
    with urllib.request.urlopen(req, timeout=2.0) as r:
        return r.read().decode("utf-8", "ignore")


def get_state(web):
    with urllib.request.urlopen(web + "/api/demo/state", timeout=1.0) as r:
        return json.loads(r.read().decode("utf-8", "ignore"))


def send_frame(board, serial, frame):
    octs = "".join(f"\\{b:03o}" for b in frame)

    # 简化版：先释放串口读进程，再用 timeout 写串口，避免阻塞
    remote = (
        f"pkill -f 'cat {serial}' 2>/dev/null; "
        f"sleep 0.3; "
        f"stty -F {serial} 115200 raw -echo 2>/dev/null; "
        f"timeout 1 sh -c 'printf \"{octs}\" > {serial}'; "
        f"echo SENT"
    )

    try:
        r = subprocess.run(
            ["ssh", "-o", "ConnectTimeout=2", f"root@{board}", remote],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=4,
        )
        print("[fire-bridge] ssh send rc:", r.returncode)
        print("[fire-bridge] ssh send stdout:", (r.stdout or "").strip())
        if r.stderr:
            print("[fire-bridge] ssh send stderr:", r.stderr.strip())
    except subprocess.TimeoutExpired:
        print("[fire-bridge] WARN: ssh send timeout ignored")
    except Exception as e:
        print("[fire-bridge] WARN: ssh send failed:", e)


def start_hazard(web):
    print("[fire-bridge] start HAZARD_CHECK")
    try:
        print(post_json(web + "/api/demo/mission", {"mission": "HAZARD_CHECK"}))
    except Exception as e:
        print("[fire-bridge] start mission failed:", e)


def cancel_task(web):
    print("[fire-bridge] cancel")
    for path in ["/api/demo/cancel", "/api/demo/emergency-release"]:
        try:
            post_json(web + path, {})
        except Exception:
            pass


def fire_sender_loop(args, state):
    while True:
        time.sleep(0.8)

        if not state["active"]:
            continue
        if state["sent_fire"]:
            continue

        should_send = False
        reason = ""

        try:
            st = get_state(args.web)
            text = json.dumps(st, ensure_ascii=False)

            if "起火风险" in text:
                should_send = True
                reason = "state contains 起火风险"
            elif time.time() - state["start_time"] > args.fallback_seconds:
                should_send = True
                reason = f"fallback {args.fallback_seconds}s"
        except Exception:
            if time.time() - state["start_time"] > args.fallback_seconds:
                should_send = True
                reason = "fallback after state read error"

        if should_send:
            print("[fire-bridge] send ONLY FIRE:", fmt(FRAME_FIRE), "reason=", reason)
            send_frame(args.board, args.serial, FRAME_FIRE)
            state["sent_fire"] = True


def read_serial_loop(args, state):
    remote_cmd = f"stty -F {args.serial} 115200 raw -echo 2>/dev/null; cat {args.serial}"

    while True:
        print(f"[fire-bridge] reading {args.board}:{args.serial}")

        p = subprocess.Popen(
            ["ssh", f"root@{args.board}", remote_cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )

        buf = b""

        try:
            while True:
                b = p.stdout.read(1)
                if not b:
                    break

                buf += b

                while len(buf) >= 5:
                    idx = buf.find(b"\xAA\x55")
                    if idx < 0:
                        buf = buf[-1:]
                        break

                    if idx > 0:
                        buf = buf[idx:]

                    if len(buf) < 5:
                        break

                    frame = buf[:5]
                    buf = buf[5:]

                    if frame[-1] != 0xFB:
                        continue

                    print("[fire-bridge] RX", fmt(frame))

                    if frame == CMD_HAZARD:
                        print("[fire-bridge] A6 -> HAZARD_CHECK")
                        state["active"] = True
                        state["sent_fire"] = False
                        state["start_time"] = time.time()
                        start_hazard(args.web)

                    elif frame == CMD_CANCEL:
                        print("[fire-bridge] A9 -> CANCEL")
                        state["active"] = False
                        state["sent_fire"] = False
                        cancel_task(args.web)

                    else:
                        print("[fire-bridge] ignored")
        except KeyboardInterrupt:
            raise
        except Exception as e:
            print("[fire-bridge] serial error:", e)

        try:
            p.kill()
        except Exception:
            pass

        print("[fire-bridge] reconnect in 2s")
        time.sleep(2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--board", default="192.168.43.192")
    ap.add_argument("--serial", default="/dev/ttyS1")
    ap.add_argument("--web", default="http://127.0.0.1:8090")
    ap.add_argument("--fallback-seconds", type=float, default=12.0)
    args = ap.parse_args()

    state = {
        "active": False,
        "sent_fire": False,
        "start_time": 0.0,
    }

    print("[fire-bridge] fire-only mode")
    print("[fire-bridge] will NEVER send FF7F or FF81")
    print("[fire-bridge] only broadcast:", fmt(FRAME_FIRE))

    threading.Thread(target=fire_sender_loop, args=(args, state), daemon=True).start()
    read_serial_loop(args, state)


if __name__ == "__main__":
    main()
