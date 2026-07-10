#!/bin/bash
set -eo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="${PROJECT_ROOT}/ros2_ws"
BOARD_IP="${BOARD_IP:-192.168.123.70}"
BOARD_DIR="/home/root/robot_2k0301"
LIDAR_PORT="${LIDAR_PORT:-2368}"

ssh -o ConnectTimeout=5 "root@${BOARD_IP}" "
    if ! netstat -lnt 2>/dev/null | grep -q ':${LIDAR_PORT} '; then
        cd '${BOARD_DIR}'
        nohup ./robot_board_app --test lidar-stream >/tmp/lidar_stream.log 2>&1 </dev/null &
        sleep 1
    fi
"

set +u
source /opt/ros/humble/setup.bash
set -u
cd "${ROS_WS}"
colcon build --packages-select robot_lidar_bridge --symlink-install
set +u
source install/setup.bash
set -u
exec ros2 launch robot_lidar_bridge lidar_rviz.launch.py \
    board_ip:="${BOARD_IP}" \
    lidar_port:="${LIDAR_PORT}"
