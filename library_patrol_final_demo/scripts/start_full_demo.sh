#!/usr/bin/env bash
set -euo pipefail

DEMO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_ROOT="$(cd "${DEMO_ROOT}/.." && pwd)"
NAV_ROOT="${PROJECT_ROOT}/robot_2k03011/robot_2k0301"

if [[ -f "${DEMO_ROOT}/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${DEMO_ROOT}/.env"
  set +a
fi

BOARD_IP="${BOARD_IP:-192.168.43.192}"
VOICE_SERIAL="${VOICE_SERIAL:-/dev/ttyS1}"
VOICE_BAUDRATE="${VOICE_BAUDRATE:-115200}"
VOICE_DEVICE="${VOICE_DEVICE:-plughw:CARD=Device,DEV=0}"
VOICE_RATE="${VOICE_RATE:-48000}"
VOICE_CHANNELS="${VOICE_CHANNELS:-1}"
VOICE_MIXER_CARD="${VOICE_MIXER_CARD:-Device}"
VOICE_MIC_GAIN="${VOICE_MIC_GAIN:-100%}"
VOICE_AGC="${VOICE_AGC:-on}"
CAMERA_DEVICE="${CAMERA_DEVICE:-/dev/video0}"
DASHBOARD_PORT=8080
WEB_PORT=8090
NAV_READY_TIMEOUT="${NAV_READY_TIMEOUT:-240}"
WEB_READY_TIMEOUT="${WEB_READY_TIMEOUT:-60}"

NAV_URL="http://127.0.0.1:${DASHBOARD_PORT}/api/state"
WEB_URL="http://127.0.0.1:${WEB_PORT}"

NAV_PID=""
WEB_PID=""
VOICE_PID=""
VOICE_DIALOG_PID=""
BOARD_CAMERA_STARTED=0
CLEANUP_DONE=0

usage() {
  cat <<EOF
用法：
  ./scripts/start_full_demo.sh

一次启动导航、板端视频流、演示网站、串口语音桥和自然语言寻书。按 Ctrl+C 统一停止。

可选环境变量：
  BOARD_IP=${BOARD_IP}
  VOICE_SERIAL=${VOICE_SERIAL}
  VOICE_BAUDRATE=${VOICE_BAUDRATE}
  VOICE_DEVICE=${VOICE_DEVICE}
  VOICE_MIXER_CARD=${VOICE_MIXER_CARD}
  VOICE_MIC_GAIN=${VOICE_MIC_GAIN}
  VOICE_AGC=${VOICE_AGC}
  CAMERA_DEVICE=${CAMERA_DEVICE}
  NAV_READY_TIMEOUT=${NAV_READY_TIMEOUT}
  WEB_READY_TIMEOUT=${WEB_READY_TIMEOUT}
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ $# -ne 0 ]]; then
  echo "未知参数：$*" >&2
  usage >&2
  exit 2
fi

require_path() {
  local path="$1"
  if [[ ! -e "${path}" ]]; then
    echo "缺少文件或目录：${path}" >&2
    exit 1
  fi
}

process_is_running() {
  local pid="$1"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null
}

wait_for_http() {
  local name="$1"
  local url="$2"
  local timeout="$3"
  local pid="$4"
  local start_time=${SECONDS}

  echo "[总启动] 等待${name}：${url}（最长 ${timeout}s）"
  while (( SECONDS - start_time < timeout )); do
    if ! process_is_running "${pid}"; then
      echo "[总启动] ${name}进程提前退出。" >&2
      return 1
    fi

    if python3 - "${url}" <<'PY' >/dev/null 2>&1
import sys
import urllib.request

with urllib.request.urlopen(sys.argv[1], timeout=1.0) as response:
    if response.status >= 400:
        raise RuntimeError(f"HTTP {response.status}")
    response.read(1)
PY
    then
      echo "[总启动] ${name}已就绪。"
      return 0
    fi
    sleep 1
  done

  echo "[总启动] 等待${name}超时：${url}" >&2
  return 1
}

post_web_control() {
  local path="$1"
  python3 - "${WEB_URL}${path}" <<'PY'
import json,sys,urllib.request
request=urllib.request.Request(
    sys.argv[1],b'{}',{'Content-Type':'application/json'},method='POST'
)
with urllib.request.urlopen(request,timeout=5) as response:
    payload=json.loads(response.read())
if not payload.get('ok'):
    raise RuntimeError(payload.get('error') or payload)
PY
}

stop_stale_local_services() {
  echo "[总启动] 清理上次遗留的网页和语音进程..."
  pkill -TERM -f '[s]cripts/voice_trigger_ssh_bridge.py' 2>/dev/null || true
  pkill -TERM -f '[s]cripts/voice_q_record_transcribe.py' 2>/dev/null || true
  "${DEMO_ROOT}/scripts/stop_demo_web.sh"
  for _ in $(seq 1 30); do
    if ! pgrep -f '[s]cripts/voice_trigger_ssh_bridge.py|[s]cripts/voice_q_record_transcribe.py|[a]pp/demo_server.py' >/dev/null; then
      return
    fi
    sleep 0.1
  done
  echo "[总启动] 遗留服务未及时退出。" >&2
  exit 1
}

terminate_group() {
  local name="$1"
  local pid="$2"
  if ! process_is_running "${pid}"; then
    return
  fi

  echo "[总启动] 停止${name}（PID ${pid}）..."
  kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
}

start_board_camera() {
  echo "[总启动] 停止 IPS200 摄像头显示，释放 ${CAMERA_DEVICE}..."
  echo "[总启动] 启动板端视频流：${BOARD_IP}:5000"
  ssh -o ConnectTimeout=5 "root@${BOARD_IP}" "
    set -e
    test -c '${CAMERA_DEVICE}'
    test -x '/home/root/board_stream_server'
    killall E05_01_ips200_display_demo 2>/dev/null || true
    killall board_stream_server 2>/dev/null || true
    nohup /home/root/board_stream_server \
      --camera '${CAMERA_DEVICE}' \
      --width 640 \
      --height 480 \
      --fps 15 \
      --jpeg-quality 85 \
      --rotate 180 \
      --framebuffer /dev/fb0 \
      --port 5000 \
      >/tmp/board_stream_server.log 2>&1 </dev/null &
    sleep 2
    if ! netstat -lnt 2>/dev/null | grep -q ':5000[[:space:]]'; then
      echo '板端视频流启动失败：' >&2
      tail -n 80 /tmp/board_stream_server.log >&2 || true
      exit 1
    fi
  "
  python3 - "${BOARD_IP}" <<'PY'
import socket,struct,sys
with socket.create_connection((sys.argv[1],5000),timeout=4) as sock:
    sock.settimeout(4)
    raw=sock.recv(4)
    if len(raw)!=4: raise RuntimeError("摄像头流没有帧长度头")
    size=struct.unpack("!I",raw)[0]
    if not 1000<=size<=5*1024*1024: raise RuntimeError(f"摄像头帧长度异常：{size}")
    data=b""
    while len(data)<size:
        chunk=sock.recv(size-len(data))
        if not chunk: raise RuntimeError("摄像头帧中断")
        data+=chunk
    if not data.startswith(b"\xff\xd8") or not data.endswith(b"\xff\xd9"):
        raise RuntimeError("摄像头未返回完整JPEG")
PY
  BOARD_CAMERA_STARTED=1
  echo "[总启动] 板端视频流已就绪。"
}

stop_board_camera() {
  if [[ "${BOARD_CAMERA_STARTED}" != "1" ]]; then
    return
  fi
  echo "[总启动] 停止板端视频流..."
  ssh -o ConnectTimeout=3 "root@${BOARD_IP}" \
    "killall board_stream_server 2>/dev/null || true" \
    >/dev/null 2>&1 || true
  BOARD_CAMERA_STARTED=0
}

cleanup() {
  local exit_code=$?
  if [[ "${CLEANUP_DONE}" == "1" ]]; then
    return "${exit_code}"
  fi
  CLEANUP_DONE=1

  trap - INT TERM
  echo "[总启动] 退出前取消任务并触发导航急停..."
  python3 - "${WEB_URL}" <<'PY' || echo "[总启动] 警告：无法通过 Web 接口确认取消/急停，仍将终止导航进程。" >&2
import json,sys,urllib.request
failures=[]
for path in ('/api/demo/cancel','/api/demo/emergency-stop'):
    try:
        req=urllib.request.Request(sys.argv[1]+path,b'{}',{'Content-Type':'application/json'},method='POST')
        with urllib.request.urlopen(req,timeout=2) as response:
            payload=json.loads(response.read())
        if not payload.get('ok'): raise RuntimeError(payload.get('error') or payload)
        print('[总启动] 已确认'+path)
    except Exception as exc:
        failures.append(f'{path}: {exc}')
if failures:
    raise RuntimeError('；'.join(failures))
PY
  terminate_group "语音桥" "${VOICE_PID}"
  terminate_group "自然语言寻书" "${VOICE_DIALOG_PID}"
  terminate_group "演示网站" "${WEB_PID}"
  terminate_group "导航" "${NAV_PID}"
  stop_board_camera

  local deadline=$((SECONDS + 8))
  while (( SECONDS < deadline )); do
    if ! process_is_running "${VOICE_PID}" && \
       ! process_is_running "${VOICE_DIALOG_PID}" && \
       ! process_is_running "${WEB_PID}" && \
       ! process_is_running "${NAV_PID}"; then
      break
    fi
    sleep 0.2
  done

  for pid in "${VOICE_PID}" "${VOICE_DIALOG_PID}" "${WEB_PID}" "${NAV_PID}"; do
    if process_is_running "${pid}"; then
      kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
    fi
  done

  for pid in "${VOICE_PID}" "${VOICE_DIALOG_PID}" "${WEB_PID}" "${NAV_PID}"; do
    if [[ -n "${pid}" ]]; then
      wait "${pid}" 2>/dev/null || true
    fi
  done

  echo "[总启动] 导航、板端视频流、网站和语音桥均已停止。"
  return "${exit_code}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

require_path "${NAV_ROOT}/scripts/start_navigation.sh"
require_path "${DEMO_ROOT}/scripts/start_demo_web.sh"
require_path "${DEMO_ROOT}/scripts/stop_demo_web.sh"
require_path "${DEMO_ROOT}/scripts/voice_trigger_ssh_bridge.py"
require_path "${DEMO_ROOT}/scripts/voice_q_record_transcribe.py"
require_path "${DEMO_ROOT}/scripts/validate_voice_config.py"
for command in python3 ssh arecord amixer setsid; do
  command -v "${command}" >/dev/null 2>&1 || { echo "缺少必要命令：${command}" >&2; exit 1; }
done
if [[ -z "${ZHIPU_API_KEY:-}" ]]; then
  echo "缺少 ZHIPU_API_KEY；请从受保护环境加载新密钥后再启动。" >&2
  exit 1
fi
python3 "${DEMO_ROOT}/scripts/validate_voice_config.py"
echo "[总启动] 检查 USB 麦克风：${VOICE_DEVICE}"
amixer -c "${VOICE_MIXER_CARD}" set Mic "${VOICE_MIC_GAIN}" cap >/dev/null 2>&1 || true
amixer -c "${VOICE_MIXER_CARD}" set 'Auto Gain Control' "${VOICE_AGC}" >/dev/null 2>&1 || true
arecord -D "${VOICE_DEVICE}" -c "${VOICE_CHANNELS}" -r "${VOICE_RATE}" -f S16_LE -d 1 -t raw /dev/null \
  >/dev/null 2>&1 || { echo "USB 麦克风健康检查失败：${VOICE_DEVICE}" >&2; exit 1; }

stop_stale_local_services

echo "[总启动] 板端地址：${BOARD_IP}"
echo "[总启动] 语音串口：${VOICE_SERIAL} @ ${VOICE_BAUDRATE}"

echo "[总启动] 1/5 启动导航..."
(
  cd "${NAV_ROOT}"
  exec setsid --wait env \
    BOARD_IP="${BOARD_IP}" \
    DASHBOARD_PORT="${DASHBOARD_PORT}" \
    ENABLE_IPS200_DISPLAY=false \
    ./scripts/start_navigation.sh
) &
NAV_PID=$!
wait_for_http "导航 API" "${NAV_URL}" "${NAV_READY_TIMEOUT}" "${NAV_PID}"

echo "[总启动] 2/5 启动板端摄像头视频流..."
start_board_camera

echo "[总启动] 3/5 启动演示网站与视觉服务..."
"${DEMO_ROOT}/scripts/stop_demo_web.sh"
(
  cd "${DEMO_ROOT}"
  exec setsid --wait env PYTHONUNBUFFERED=1 ./scripts/start_demo_web.sh
) &
WEB_PID=$!
wait_for_http "演示 Web" "${WEB_URL}/api/demo/state" "${WEB_READY_TIMEOUT}" "${WEB_PID}"

# A clean shutdown deliberately latches navigation emergency-stop.  At this
# point the replacement Web service is ready, no voice input is open, and no
# mission has been submitted, so it is safe to clear that persisted latch.
echo "[总启动] 清除上次安全退出遗留的导航急停..."
post_web_control "/api/demo/emergency-release"

echo "[总启动] 4/5 启动自然语言录音服务..."
(
  cd "${DEMO_ROOT}"
  exec setsid --wait env PYTHONUNBUFFERED=1 VOICE_DEVICE="${VOICE_DEVICE}" \
    python3 scripts/voice_q_record_transcribe.py --web-url "${WEB_URL}"
) &
VOICE_DIALOG_PID=$!

sleep 1
if ! process_is_running "${VOICE_DIALOG_PID}"; then
  echo "[总启动] 自然语言录音服务启动失败。" >&2
  exit 1
fi
wait_for_http "语音会话 API" "http://127.0.0.1:8092/health" 15 "${VOICE_DIALOG_PID}"

echo "[总启动] 5/5 启动 CI302 唤醒桥..."
(
  cd "${DEMO_ROOT}"
  exec setsid --wait env PYTHONUNBUFFERED=1 python3 scripts/voice_trigger_ssh_bridge.py \
    --board "${BOARD_IP}" \
    --serial "${VOICE_SERIAL}" \
    --baudrate "${VOICE_BAUDRATE}" \
    --web "${WEB_URL}" \
    --wake-url "http://127.0.0.1:8092/wake" \
    --input-only
) &
VOICE_PID=$!

sleep 1
if ! process_is_running "${VOICE_PID}"; then
  echo "[总启动] 语音桥启动失败。" >&2
  exit 1
fi

echo
echo "[总启动] 全部服务已启动：${WEB_URL}"
echo "[总启动] 保持本终端开启；按 Ctrl+C 统一停止。"

set +e
EXITED_PID=""
wait -n -p EXITED_PID "${NAV_PID}" "${WEB_PID}" "${VOICE_PID}" "${VOICE_DIALOG_PID}"
EXIT_CODE=$?
set -e

echo "[总启动] 子进程 ${EXITED_PID:-未知} 已退出（状态 ${EXIT_CODE}），正在停止其余服务。" >&2
if [[ "${EXIT_CODE}" == "0" ]]; then
  exit 1
fi
exit "${EXIT_CODE}"
