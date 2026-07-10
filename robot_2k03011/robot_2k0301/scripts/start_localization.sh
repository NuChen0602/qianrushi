#!/bin/bash
set -eo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="${PROJECT_ROOT}/ros2_ws"
MAP_FILE="${1:-${PROJECT_ROOT}/maps/library}"
LOCAL_ROS_PREFIX="${PROJECT_ROOT}/.local_tools/ros-humble-local/opt/ros/humble"
BOARD_IP="${BOARD_IP:-192.168.123.70}"
BOARD_DIR="/home/root/robot_2k0301"
LIDAR_PORT="${LIDAR_PORT:-2368}"
ODOM_PORT="${ODOM_PORT:-2369}"
SKIP_MAPPING_PREFLIGHT="${SKIP_MAPPING_PREFLIGHT:-0}"
SKIP_MAP_QUALITY_CHECK="${SKIP_MAP_QUALITY_CHECK:-0}"

set +u
source /opt/ros/humble/setup.bash
set -u

if [[ -d "${LOCAL_ROS_PREFIX}/share/ament_index" ]]; then
    export AMENT_PREFIX_PATH="${LOCAL_ROS_PREFIX}:${AMENT_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="${LOCAL_ROS_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
    export PATH="${LOCAL_ROS_PREFIX}/bin:${PATH}"
fi

if ! ros2 pkg prefix nav2_amcl >/dev/null 2>&1; then
    echo "缺少 AMCL，请先执行：./scripts/install_localization_deps.sh"
    exit 1
fi
if ! ros2 pkg prefix nav2_map_server >/dev/null 2>&1; then
    echo "缺少 map_server，请先执行：./scripts/install_localization_deps.sh"
    exit 1
fi
if [[ ! -f "${MAP_FILE}.yaml" || ! -f "${MAP_FILE}.pgm" ]]; then
    echo "找不到固定地图：${MAP_FILE}.yaml / ${MAP_FILE}.pgm"
    exit 1
fi
if [[ "${SKIP_MAP_QUALITY_CHECK}" != "1" ]]; then
    python3 "${PROJECT_ROOT}/scripts/check_map_quality.py" \
        --map-yaml "${MAP_FILE}.yaml"
fi

echo "正在检查板端 ${BOARD_IP}，预检期间请保持小车静止。"
pkill -f '/robot_lidar_bridge/(tcp_laser_scan|tcp_odometry|goal_navigator|mapping_guard|occupancy_grid_cells)' 2>/dev/null || true
pkill -f '/(rviz2/rviz2|slam_toolbox/.*slam_toolbox|nav2_amcl/amcl|nav2_map_server/map_server)' 2>/dev/null || true
sleep 1
ssh -o ConnectTimeout=5 "root@${BOARD_IP}" "
    cd '${BOARD_DIR}'
    if ! netstat -lnt 2>/dev/null | grep -q ':${LIDAR_PORT} '; then
        nohup ./robot_board_app --test lidar-stream >/tmp/lidar_stream.log 2>&1 </dev/null &
    fi
    if ! netstat -lnt 2>/dev/null | grep -q ':${ODOM_PORT} '; then
        nohup ./robot_board_app --test odom-stream >/tmp/odom_stream.log 2>&1 </dev/null &
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
exec ros2 launch robot_lidar_bridge localization_amcl.launch.py \
    map_yaml:="${MAP_FILE}.yaml" \
    board_ip:="${BOARD_IP}" \
    lidar_port:="${LIDAR_PORT}" \
    odom_port:="${ODOM_PORT}"
