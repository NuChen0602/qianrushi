#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$ROOT_DIR/2_Sim_Environment"
BOARD_HOST="${BOARD_HOST:-192.168.43.192}"
BOARD_PORT="${BOARD_PORT:-15000}"
CAMERA_DEVICE="${CAMERA_DEVICE:-/dev/video0}"
WEB_PORT="${WEB_PORT:-8080}"
LOG_FILE="${LOG_FILE:-/tmp/library_patrol_real_bringup.log}"
PID_FILE="${PID_FILE:-/tmp/library_patrol_real_launch.pid}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/library_patrol_real_ros_logs}"

deactivate_conda() {
  source "$HOME/miniconda3/etc/profile.d/conda.sh" 2>/dev/null || source "$HOME/anaconda3/etc/profile.d/conda.sh" 2>/dev/null || true
  conda deactivate 2>/dev/null || true
}

source_ros() {
  deactivate_conda
  set +u
  source /opt/ros/humble/setup.bash
  cd "$SIM_DIR"
  [[ -f .env ]] && set -a && source .env && set +a
  source install/setup.bash
  set -u
}

usage() {
  cat <<USAGE
用法:
  ./run_real_robot.sh camera        检测电脑/板子摄像头和 ROS 图像话题
  ./run_real_robot.sh status        查看 Web、ROS、板子 UDP 连接状态
  ./run_real_robot.sh start-web     后台启动实机 Web/相机/视觉/运动桥接
  ./run_real_robot.sh start-ai      在当前终端启动 AI 对话导航
  ./run_real_robot.sh start-all     后台启动 Web 后，在当前终端启动 AI 对话导航
  ./run_real_robot.sh stop-car      给小车发送急停命令
  ./run_real_robot.sh stop          停止电脑端实机巡检服务

常用环境变量:
  BOARD_HOST=$BOARD_HOST
  BOARD_PORT=$BOARD_PORT
  CAMERA_DEVICE=$CAMERA_DEVICE
  WEB_PORT=$WEB_PORT
USAGE
}

udp_cmd() {
  local cmd="$1"
  python3 - "$BOARD_HOST" "$BOARD_PORT" "$cmd" <<'PY'
import socket
import sys

host, port, cmd = sys.argv[1], int(sys.argv[2]), sys.argv[3]
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(1.0)
s.sendto((cmd.strip() + "\n").encode(), (host, port))
try:
    print(s.recvfrom(2048)[0].decode(errors="replace").strip())
except TimeoutError:
    print("no reply")
except socket.timeout:
    print("no reply")
PY
}

cmd_camera() {
  echo "[PC] video devices:"
  ls -l /dev/video* 2>/dev/null || true
  if command -v v4l2-ctl >/dev/null 2>&1; then
    v4l2-ctl --list-devices || true
  else
    echo "[PC] v4l2-ctl 未安装，跳过详细摄像头枚举"
  fi

  echo
  echo "[Board] video devices:"
  ssh -o ConnectTimeout=3 -o StrictHostKeyChecking=no "root@$BOARD_HOST" \
    'ls -l /dev/video* 2>/dev/null || true' || true

  echo
  echo "[ROS] camera topics:"
  source_ros
  ros2 topic list | grep -E 'camera|image|low_angle' || true
  ros2 topic hz /low_angle_camera/image_raw --window 5 2>/dev/null &
  local hz_pid=$!
  sleep 4
  kill "$hz_pid" 2>/dev/null || true
}

cmd_status() {
  echo "[PC] Web UI: http://localhost:$WEB_PORT"
  python3 - "$WEB_PORT" <<'PY' || true
import sys, urllib.request
port = sys.argv[1]
try:
    data = urllib.request.urlopen(f"http://localhost:{port}/api/state", timeout=0.8).read(120)
    print("[PC] Web dashboard: OK")
except Exception as exc:
    print(f"[PC] Web dashboard: not reachable ({exc})")
PY
  echo "[PC] related processes:"
  pgrep -a -f 'real_bringup|patrol_web_dashboard|real_camera_publisher|vision_bridge|llm_voice_navigator|real_motion_bridge|real_goal_driver' || true
  echo
  echo "[Board] UDP agent:"
  udp_cmd "PING"
  udp_cmd "STATUS"
}

cmd_start_web() {
  mkdir -p "$ROS_LOG_DIR"
  if python3 - "$WEB_PORT" <<'PY' >/dev/null 2>&1
import sys, urllib.request
urllib.request.urlopen(f"http://localhost:{sys.argv[1]}/api/state", timeout=0.5).read()
PY
  then
    echo "[OK] Web 已经在运行: http://localhost:$WEB_PORT"
    return 0
  fi

  echo "[Start] 启动实机 Web/相机/视觉/运动桥接..."
  deactivate_conda
  cat > /tmp/library_patrol_real_launch.sh <<EOF
#!/usr/bin/env bash
set -eo pipefail
source "\$HOME/miniconda3/etc/profile.d/conda.sh" 2>/dev/null || source "\$HOME/anaconda3/etc/profile.d/conda.sh" 2>/dev/null || true
conda deactivate 2>/dev/null || true
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$SIM_DIR"
if [[ -f .env ]]; then set -a; source .env; set +a; fi
set +u
source install/setup.bash
set -u
export ROS_LOG_DIR="$ROS_LOG_DIR"
exec ros2 launch library_gazebo real_bringup.launch.py \\
  board_host:="$BOARD_HOST" \\
  board_port:="$BOARD_PORT" \\
  camera_device:="$CAMERA_DEVICE" \\
  start_voice:=false \\
  web_dashboard_port:="$WEB_PORT"
EOF
  chmod +x /tmp/library_patrol_real_launch.sh
  setsid nohup /tmp/library_patrol_real_launch.sh > "$LOG_FILE" 2>&1 < /dev/null &
  echo $! > "$PID_FILE"

  for _ in {1..30}; do
    if python3 - "$WEB_PORT" <<'PY' >/dev/null 2>&1
import sys, urllib.request
urllib.request.urlopen(f"http://localhost:{sys.argv[1]}/api/state", timeout=0.5).read()
PY
    then
      echo "[OK] Web UI: http://localhost:$WEB_PORT"
      echo "[Log] $LOG_FILE"
      return 0
    fi
    sleep 0.5
  done

  echo "[WARN] Web 还没响应，请看日志: $LOG_FILE"
}

cmd_start_ai() {
  echo "[Start] AI 对话导航会占用当前终端，按 Ctrl+C 退出。"
  source_ros
  ros2 run library_gazebo llm_voice_navigator.py --ros-args -p use_nav2:=false -p real_goal_wait_sec:=8.0
}

cmd_stop_car() {
  echo "[Board] STOP"
  udp_cmd "STOP"
}

cmd_stop() {
  echo "[Stop] 先给小车发送 STOP"
  udp_cmd "STOP" || true

  if [[ -f "$PID_FILE" ]]; then
    kill "$(cat "$PID_FILE")" 2>/dev/null || true
    rm -f "$PID_FILE"
  fi

  mapfile -t pids < <(pgrep -f '/opt/ros/humble/bin/ros2 launch library_gazebo real_bringup.launch.py|library_gazebo/real_motion_bridge.py|library_gazebo/real_goal_driver.py|library_gazebo/real_camera_publisher.py|sim_bridge_node/vision_bridge|library_gazebo/patrol_web_dashboard.py|library_gazebo/llm_voice_navigator.py' || true)
  if [[ "${#pids[@]}" -gt 0 ]]; then
    kill "${pids[@]}" 2>/dev/null || true
  fi
  echo "[OK] 已请求停止电脑端服务"
}

case "${1:-}" in
  camera) cmd_camera ;;
  status) cmd_status ;;
  start-web) cmd_start_web ;;
  start-ai) cmd_start_ai ;;
  start-all) cmd_start_web; cmd_start_ai ;;
  stop-car) cmd_stop_car ;;
  stop) cmd_stop ;;
  -h|--help|"") usage ;;
  *) echo "未知命令: $1"; usage; exit 2 ;;
esac
