#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

VISION_PID=""
cleanup() {
  if [[ -n "$VISION_PID" ]]; then
    kill "$VISION_PID" 2>/dev/null || true
    wait "$VISION_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[demo] 启动异步遗失物视觉服务: http://127.0.0.1:8091"
PYTHONUNBUFFERED=1 python3 scripts/lost_item_visual_api.py &
VISION_PID=$!

echo "[demo] 启动 Web 控制台: http://127.0.0.1:8090"
PYTHONUNBUFFERED=1 python3 app/demo_server.py
