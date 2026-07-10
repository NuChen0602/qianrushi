#!/usr/bin/env bash
set -euo pipefail

DEMO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_ROOT="$(cd "${DEMO_ROOT}/.." && pwd)"
NAV_ROOT="${PROJECT_ROOT}/robot_2k03011/robot_2k0301"

BOARD_IP="${BOARD_IP:-192.168.43.192}"
VOICE_SERIAL="${VOICE_SERIAL:-/dev/ttyS1}"
VOICE_BAUDRATE="${VOICE_BAUDRATE:-115200}"
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
BOARD_CAMERA_STARTED=0
CLEANUP_DONE=0

usage() {
  cat <<EOF
用法：
  ./scripts/start_full_demo.sh

一次启动导航、板端视频流、演示网站和语音桥。按 Ctrl+C 统一停止。

可选环境变量：
  BOARD_IP=${BOARD_IP}
  VOICE_SERIAL=${VOICE_SERIAL}
  VOICE_BAUDRATE=${VOICE_BAUDRATE}
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
      --port 5000 \
      >/tmp/board_stream_server.log 2>&1 </dev/null &
    sleep 2
    if ! netstat -lnt 2>/dev/null | grep -q ':5000[[:space:]]'; then
      echo '板端视频流启动失败：' >&2
      tail -n 80 /tmp/board_stream_server.log >&2 || true
      exit 1
    fi
  "
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
  terminate_group "语音桥" "${VOICE_PID}"
  terminate_group "演示网站" "${WEB_PID}"
  terminate_group "导航" "${NAV_PID}"
  stop_board_camera

  local deadline=$((SECONDS + 8))
  while (( SECONDS < deadline )); do
    if ! process_is_running "${VOICE_PID}" && \
       ! process_is_running "${WEB_PID}" && \
       ! process_is_running "${NAV_PID}"; then
      break
    fi
    sleep 0.2
  done

  for pid in "${VOICE_PID}" "${WEB_PID}" "${NAV_PID}"; do
    if process_is_running "${pid}"; then
      kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
    fi
  done

  for pid in "${VOICE_PID}" "${WEB_PID}" "${NAV_PID}"; do
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
command -v setsid >/dev/null 2>&1 || {
  echo "缺少 setsid 命令，无法可靠管理全部子进程。" >&2
  exit 1
}

echo "[总启动] 板端地址：${BOARD_IP}"
echo "[总启动] 语音串口：${VOICE_SERIAL} @ ${VOICE_BAUDRATE}"

echo "[总启动] 1/4 启动导航..."
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

echo "[总启动] 2/4 启动板端摄像头视频流..."
start_board_camera

echo "[总启动] 3/4 启动演示网站与视觉服务..."
"${DEMO_ROOT}/scripts/stop_demo_web.sh"
(
  cd "${DEMO_ROOT}"
  exec setsid --wait env PYTHONUNBUFFERED=1 ./scripts/start_demo_web.sh
) &
WEB_PID=$!
wait_for_http "演示 Web" "${WEB_URL}/api/demo/state" "${WEB_READY_TIMEOUT}" "${WEB_PID}"

echo "[总启动] 4/4 启动语音桥..."
(
  cd "${DEMO_ROOT}"
  exec setsid --wait env PYTHONUNBUFFERED=1 python3 scripts/voice_trigger_ssh_bridge.py \
    --board "${BOARD_IP}" \
    --serial "${VOICE_SERIAL}" \
    --baudrate "${VOICE_BAUDRATE}" \
    --web "${WEB_URL}"
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
wait -n -p EXITED_PID "${NAV_PID}" "${WEB_PID}" "${VOICE_PID}"
EXIT_CODE=$?
set -e

echo "[总启动] 子进程 ${EXITED_PID:-未知} 已退出（状态 ${EXIT_CODE}），正在停止其余服务。" >&2
if [[ "${EXIT_CODE}" == "0" ]]; then
  exit 1
fi
exit "${EXIT_CODE}"
