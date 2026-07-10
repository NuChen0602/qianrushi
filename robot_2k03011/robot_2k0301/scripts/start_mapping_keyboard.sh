#!/bin/bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="${PROJECT_ROOT}/ros2_ws"
BOARD_IP="${BOARD_IP:-192.168.123.70}"
BOARD_DIR="/home/root/robot_2k0301"
MAPPING_LOG="${MAPPING_LOG:-/tmp/robot_mapping_keyboard.log}"
MAPPING_PID=""

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${ROS_WS}/install/setup.bash" ]]; then
    source "${ROS_WS}/install/setup.bash"
fi
set -u

cleanup() {
    set +e
    if [[ -n "${MAPPING_PID}" ]] && kill -0 "${MAPPING_PID}" 2>/dev/null; then
        kill -INT "${MAPPING_PID}" 2>/dev/null
        for _ in $(seq 1 30); do
            kill -0 "${MAPPING_PID}" 2>/dev/null || break
            sleep 0.1
        done
        kill -TERM "${MAPPING_PID}" 2>/dev/null
        wait "${MAPPING_PID}" 2>/dev/null
    fi
    ssh -o ConnectTimeout=5 "root@${BOARD_IP}" "killall robot_board_app 2>/dev/null || true" \
        >/dev/null 2>&1
}
trap cleanup EXIT INT TERM

echo "正在准备键盘建图，预检完成前请保持小车静止。"
ssh -o ConnectTimeout=5 "root@${BOARD_IP}" \
    "killall robot_board_app 2>/dev/null || true; sleep 1; cd '${BOARD_DIR}'"

ODOM_MODE=mapping-drive ENABLE_DRIVE=true ENABLE_DRIVE_OBSTACLE_SAFETY=false \
    BOARD_IP="${BOARD_IP}" \
    OBSTACLE_STOP_DISTANCE_M="${OBSTACLE_STOP_DISTANCE_M:-0.35}" \
    OBSTACLE_SLOW_DISTANCE_M="${OBSTACLE_SLOW_DISTANCE_M:-0.60}" \
    "${PROJECT_ROOT}/scripts/start_mapping.sh" >"${MAPPING_LOG}" 2>&1 &
MAPPING_PID=$!

ready=0
for _ in $(seq 1 60); do
    if ! kill -0 "${MAPPING_PID}" 2>/dev/null; then
        echo "键盘建图启动失败："
        tail -80 "${MAPPING_LOG}"
        exit 1
    fi
    if ros2 node list 2>/dev/null | grep -q '^/slam_toolbox$' &&
       ros2 node list 2>/dev/null | grep -q '^/robot_odometry_tcp_bridge$'; then
        ready=1
        break
    fi
    sleep 1
done

if [[ "${ready}" != "1" ]]; then
    echo "等待SLAM启动超时："
    tail -80 "${MAPPING_LOG}"
    exit 1
fi

set +u
source "${ROS_WS}/install/setup.bash"
set -u

echo "RViz与安全监控已就绪。按Q后将自动停车并保存地图。"
ros2 run robot_lidar_bridge keyboard_teleop

sleep 1
"${PROJECT_ROOT}/scripts/save_slam_map.sh"
echo "键盘建图完成，日志：${MAPPING_LOG}"
