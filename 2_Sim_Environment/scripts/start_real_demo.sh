#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-$HOME/Library_Patrol_Project/2_Sim_Environment}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/library_patrol_real_ros_logs}"
BRINGUP_LOG="${BRINGUP_LOG:-/tmp/library_patrol_real_bringup.log}"
VOICE_LOG="${VOICE_LOG:-/tmp/library_patrol_real_voice.log}"
WEB_PORT="${WEB_PORT:-8080}"
BOARD_HOST="${BOARD_HOST:-192.168.2.77}"
BOARD_PORT="${BOARD_PORT:-15000}"
CAMERA_DEVICE="${CAMERA_DEVICE:-/dev/video0}"
SKIP_BUILD=false
OPEN_UI=true
START_VOICE=true

while [[ $# -gt 0 ]]; do
  case "$1" in
    --board-host)
      BOARD_HOST="$2"
      shift 2
      ;;
    --board-port)
      BOARD_PORT="$2"
      shift 2
      ;;
    --camera)
      CAMERA_DEVICE="$2"
      shift 2
      ;;
    --port)
      WEB_PORT="$2"
      shift 2
      ;;
    --skip-build)
      SKIP_BUILD=true
      shift
      ;;
    --no-open-ui)
      OPEN_UI=false
      shift
      ;;
    --no-voice)
      START_VOICE=false
      shift
      ;;
    -h|--help)
      cat <<USAGE
Usage: scripts/start_real_demo.sh [--board-host IP] [--board-port 15000] [--camera /dev/video0] [--port 8080] [--skip-build] [--no-voice]

Starts the real-robot ROS bridge, camera publisher, VisionCore bridge, Web dashboard,
and optionally the LLM voice navigator in real mode.
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
  echo "[Library Patrol Real] shutting down background launch..."
  if [[ -n "${BRINGUP_PID:-}" ]]; then kill "$BRINGUP_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT INT TERM

source "$HOME/miniconda3/etc/profile.d/conda.sh" 2>/dev/null || source "$HOME/anaconda3/etc/profile.d/conda.sh" 2>/dev/null || true
conda deactivate 2>/dev/null || true

set +u
source /opt/ros/humble/setup.bash
set -u
cd "$WORKSPACE"

if [[ -f "$WORKSPACE/.env" ]]; then
  set -a
  source "$WORKSPACE/.env"
  set +a
fi

if [[ "$SKIP_BUILD" == false ]]; then
  echo "[Library Patrol Real] building ROS workspace..."
  colcon build --symlink-install
fi

set +u
source install/setup.bash
set -u
mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

echo "[Library Patrol Real] board=${BOARD_HOST}:${BOARD_PORT}, camera=${CAMERA_DEVICE}"
ros2 launch library_gazebo real_bringup.launch.py \
  board_host:="$BOARD_HOST" \
  board_port:="$BOARD_PORT" \
  camera_device:="$CAMERA_DEVICE" \
  start_voice:=false \
  web_dashboard_port:="$WEB_PORT" \
  >"$BRINGUP_LOG" 2>&1 &
BRINGUP_PID=$!

UI_URL="http://localhost:${WEB_PORT}"
echo "[Library Patrol Real] bringup PID: $BRINGUP_PID"
echo "[Library Patrol Real] bringup log: $BRINGUP_LOG"
echo "[Library Patrol Real] Web UI: $UI_URL"

for _ in {1..20}; do
  if python3 -c "import urllib.request; urllib.request.urlopen('${UI_URL}/api/state', timeout=0.5).read()" >/dev/null 2>&1; then
    if [[ "$OPEN_UI" == true ]] && command -v xdg-open >/dev/null 2>&1; then
      xdg-open "$UI_URL" >/dev/null 2>&1 || true
    fi
    break
  fi
  sleep 0.5
done

if [[ "$START_VOICE" == true ]]; then
  echo "[Library Patrol Real] voice navigator runs in this terminal. Real mode does not call Nav2."
  ros2 run library_gazebo llm_voice_navigator.py --ros-args -p use_nav2:=false -p real_goal_wait_sec:=8.0 2>&1 | tee "$VOICE_LOG"
else
  echo "[Library Patrol Real] voice disabled. Press Ctrl+C to stop."
  wait "$BRINGUP_PID"
fi
