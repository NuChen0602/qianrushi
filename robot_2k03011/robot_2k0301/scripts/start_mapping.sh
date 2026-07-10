#!/bin/bash
set -eo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="${PROJECT_ROOT}/ros2_ws"
BOARD_IP="${BOARD_IP:-192.168.123.70}"
BOARD_DIR="/home/root/robot_2k0301"
LIDAR_PORT="${LIDAR_PORT:-2368}"
ODOM_PORT="${ODOM_PORT:-2369}"
ODOM_MODE="${ODOM_MODE:-odom-stream}"
ENABLE_DRIVE="${ENABLE_DRIVE:-false}"
ENABLE_DRIVE_OBSTACLE_SAFETY="${ENABLE_DRIVE_OBSTACLE_SAFETY:-true}"
OBSTACLE_STOP_DISTANCE_M="${OBSTACLE_STOP_DISTANCE_M:-0.5}"
OBSTACLE_SLOW_DISTANCE_M="${OBSTACLE_SLOW_DISTANCE_M:-0.8}"
SKIP_MAPPING_PREFLIGHT="${SKIP_MAPPING_PREFLIGHT:-0}"
ENABLE_IPS200_DISPLAY="${ENABLE_IPS200_DISPLAY:-true}"
IPS200_DISPLAY_APP="${IPS200_DISPLAY_APP:-/home/root/E05_01_ips200_display_demo}"
IPS200_DISPLAY_BIN="$(basename "${IPS200_DISPLAY_APP}")"
IPS200_MAP_PORT="${IPS200_MAP_PORT:-2370}"

if [[ "${ODOM_MODE}" != "odom-stream" && "${ODOM_MODE}" != "mapping-drive" ]]; then
    echo "ODOM_MODE must be odom-stream or mapping-drive"
    exit 1
fi

set +u
source /opt/ros/humble/setup.bash
set -u

if ! ros2 pkg prefix slam_toolbox >/dev/null 2>&1; then
    echo "缺少 slam_toolbox，请先执行：sudo apt install ros-humble-slam-toolbox"
    exit 1
fi

if [[ "${ENABLE_IPS200_DISPLAY}" == "true" ]]; then
    echo "正在启动板端 IPS200 建图显示。"
    ssh -o ConnectTimeout=5 "root@${BOARD_IP}" "
        if [ -x '${IPS200_DISPLAY_APP}' ]; then
            killall '${IPS200_DISPLAY_BIN}' 2>/dev/null || true
            nohup '${IPS200_DISPLAY_APP}' >/tmp/ips200_map_display.log 2>&1 </dev/null &
        else
            echo '未找到 IPS200 显示程序：${IPS200_DISPLAY_APP}' >&2
        fi
    "
fi

echo "正在检查板端 ${BOARD_IP}，预检期间请保持小车静止。"
ssh -o ConnectTimeout=5 "root@${BOARD_IP}" "
    cd '${BOARD_DIR}'
    if ! netstat -lnt 2>/dev/null | grep -q ':${LIDAR_PORT} '; then
        nohup ./robot_board_app --test lidar-stream >/tmp/lidar_stream.log 2>&1 </dev/null &
    fi
    if ! netstat -lnt 2>/dev/null | grep -q ':${ODOM_PORT} '; then
        nohup ./robot_board_app --test '${ODOM_MODE}' >/tmp/odom_stream.log 2>&1 </dev/null &
    fi
    sleep 2
"

if [[ "${SKIP_MAPPING_PREFLIGHT}" != "1" ]]; then
    python3 "${PROJECT_ROOT}/scripts/check_mapping_inputs.py" \
        --host "${BOARD_IP}" \
        --lidar-port "${LIDAR_PORT}" \
        --odom-port "${ODOM_PORT}"
fi

cd "${ROS_WS}"
colcon build --packages-select robot_lidar_bridge --symlink-install
set +u
source install/setup.bash
set -u
exec ros2 launch robot_lidar_bridge mapping.launch.py \
    board_ip:="${BOARD_IP}" \
    lidar_port:="${LIDAR_PORT}" \
    odom_port:="${ODOM_PORT}" \
    enable_drive:="${ENABLE_DRIVE}" \
    enable_drive_obstacle_safety:="${ENABLE_DRIVE_OBSTACLE_SAFETY}" \
    obstacle_stop_distance_m:="${OBSTACLE_STOP_DISTANCE_M}" \
    obstacle_slow_distance_m:="${OBSTACLE_SLOW_DISTANCE_M}" \
    enable_ips200_map_display:="${ENABLE_IPS200_DISPLAY}" \
    ips200_map_port:="${IPS200_MAP_PORT}"
