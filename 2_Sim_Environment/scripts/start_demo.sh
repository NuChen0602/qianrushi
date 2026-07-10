#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-$HOME/Library_Patrol_Project/2_Sim_Environment}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/library_patrol_ros_logs}"
BRINGUP_LOG="${BRINGUP_LOG:-/tmp/library_patrol_bringup.log}"
VOICE_LOG="${VOICE_LOG:-/tmp/library_patrol_voice.log}"
AUTO_PATROL=false
SKIP_BUILD=false
OPEN_UI=true
WEB_PORT="${WEB_PORT:-8080}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --auto-patrol)
      AUTO_PATROL=true
      shift
      ;;
    --skip-build)
      SKIP_BUILD=true
      shift
      ;;
    --port)
      WEB_PORT="$2"
      shift 2
      ;;
    --no-open-ui)
      OPEN_UI=false
      shift
      ;;
    -h|--help)
      cat <<USAGE
Usage: scripts/start_demo.sh [--auto-patrol] [--skip-build] [--port 8080] [--no-open-ui]

Starts Gazebo, SLAM, Nav2, vision bridge, Web dashboard, and the voice navigator.
The ROS bringup runs in the background; voice interaction stays in this terminal.
USAGE
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

cleanup() {
  echo
  echo "[Library Patrol] 正在关闭后台节点..."
  if [[ -n "${MISSION_PID:-}" ]]; then kill "$MISSION_PID" 2>/dev/null || true; fi
  if [[ -n "${BRINGUP_PID:-}" ]]; then kill "$BRINGUP_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT INT TERM

if [[ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "$HOME/miniconda3/etc/profile.d/conda.sh"
  set -u
elif [[ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "$HOME/anaconda3/etc/profile.d/conda.sh"
  set -u
fi
conda deactivate 2>/dev/null || true

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
cd "$WORKSPACE"

if [[ -f "$WORKSPACE/.env" ]]; then
  echo "[Library Patrol] 读取环境变量文件: $WORKSPACE/.env"
  set -a
  # shellcheck disable=SC1091
  source "$WORKSPACE/.env"
  set +a
fi

if [[ -n "${DEEPSEEK_API_KEY:-}" ]]; then
  echo "[Library Patrol] DeepSeek API KEY 已设置，将启用云端 LLM。"
else
  echo "[Library Patrol] 未检测到 DEEPSEEK_API_KEY，语音节点将使用本地语义匹配兜底。"
  echo "[Library Patrol] 可在当前终端 export DEEPSEEK_API_KEY='你的key'，或写入 $WORKSPACE/.env"
fi

if [[ "$SKIP_BUILD" == false ]]; then
  echo "[Library Patrol] 构建 ROS 2 工作空间..."
  colcon build --symlink-install
fi

set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u
mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

echo "[Library Patrol] 启动 Gazebo + SLAM + Nav2 + 视觉桥接 + Web 中控..."
ros2 launch library_gazebo bringup.launch.py \
  start_web_dashboard:=true \
  web_dashboard_port:="$WEB_PORT" \
  >"$BRINGUP_LOG" 2>&1 &
BRINGUP_PID=$!

echo "[Library Patrol] 后台 bringup PID: $BRINGUP_PID"
echo "[Library Patrol] bringup 日志: $BRINGUP_LOG"
echo "[Library Patrol] 等待 Nav2/Gazebo 初始化..."

UI_URL="http://localhost:${WEB_PORT}"
echo "[Library Patrol] 检查 Web 中控 UI: ${UI_URL}"
UI_READY=false
for _ in {1..20}; do
  if python3 -c "import urllib.request; urllib.request.urlopen('${UI_URL}/api/state', timeout=0.5).read()" >/dev/null 2>&1; then
    UI_READY=true
    break
  fi
  sleep 0.5
done

if [[ "$UI_READY" == true ]]; then
  echo "[Library Patrol] Web 中控 UI 已就绪: ${UI_URL}"
  if [[ "$OPEN_UI" == true ]]; then
    if command -v xdg-open >/dev/null 2>&1; then
      xdg-open "$UI_URL" >/dev/null 2>&1 || echo "[Library Patrol] 浏览器未能自动打开，请手动访问: ${UI_URL}"
    else
      echo "[Library Patrol] 未找到 xdg-open，请手动访问: ${UI_URL}"
    fi
  fi
else
  echo "[Library Patrol] Web 中控 UI 暂未响应，请查看日志: $BRINGUP_LOG"
fi

echo "[Library Patrol] 等待 Nav2 /navigate_to_pose action 就绪..."
NAV2_READY=false
for _ in {1..90}; do
  if ros2 action list 2>/dev/null | grep -qx "/navigate_to_pose"; then
    NAV2_READY=true
    break
  fi
  if ! kill -0 "$BRINGUP_PID" 2>/dev/null; then
    echo "[Library Patrol] bringup 已退出，请查看日志: $BRINGUP_LOG"
    break
  fi
  sleep 1
done

if [[ "$NAV2_READY" == true ]]; then
  echo "[Library Patrol] Nav2 已就绪，可以语音巡检。"
else
  echo "[Library Patrol] Nav2 尚未就绪，语音节点仍会启动，但巡检需等导航栈 ready。"
  echo "[Library Patrol] 排查建议: grep -n \"Managed nodes are active\\|failed\\|Timed out\" $BRINGUP_LOG"
fi

if [[ "$AUTO_PATROL" == true ]]; then
  echo "[Library Patrol] 启动自动巡检路线..."
  ros2 run library_gazebo patrol_mission.py >/tmp/library_patrol_mission.log 2>&1 &
  MISSION_PID=$!
  echo "[Library Patrol] 自动巡检日志: /tmp/library_patrol_mission.log"
fi

echo
echo "[Library Patrol] Web 中控 UI: ${UI_URL}"
echo "[Library Patrol] 语音节点将在当前终端运行。唤醒词：机器人 / 小车 / 巡检机器人"
echo "[Library Patrol] 如果语音模块不可用，会自动进入键盘模式。"
echo

ros2 run library_gazebo llm_voice_navigator.py 2>&1 | tee "$VOICE_LOG"
