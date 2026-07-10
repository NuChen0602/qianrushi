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
DASHBOARD_PORT="${DASHBOARD_PORT:-8080}"
ENABLE_IPS200_DISPLAY="${ENABLE_IPS200_DISPLAY:-true}"
IPS200_DISPLAY_APP="${IPS200_DISPLAY_APP:-/home/root/E05_01_ips200_display_demo}"
IPS200_DISPLAY_BIN="$(basename "${IPS200_DISPLAY_APP}")"
SKIP_MAPPING_PREFLIGHT="${SKIP_MAPPING_PREFLIGHT:-0}"
SKIP_MAP_QUALITY_CHECK="${SKIP_MAP_QUALITY_CHECK:-0}"
NAV_LOCALIZER="${NAV_LOCALIZER:-slam}"
NAV_LOG_ROOT="${NAV_LOG_ROOT:-${PROJECT_ROOT}/log/navigation}"
NAV_LOG_DIR="${NAV_LOG_DIR:-${NAV_LOG_ROOT}/$(date +%Y%m%d_%H%M%S)}"
NAV_RECORD_BAG="${NAV_RECORD_BAG:-0}"
NAV_LAUNCH_LOG="${NAV_LOG_DIR}/navigation.launch.log"
NAV_TOPIC_LOG="${NAV_LOG_DIR}/topics.log"
NAV_BAG_LOG="${NAV_LOG_DIR}/rosbag.log"
NAV_BOARD_LOG="${NAV_LOG_DIR}/board_stream_tail.log"
DEBUG_RECORDER_PID=""
BAG_RECORDER_PID=""
CLEANUP_DONE=0

mkdir -p "${NAV_LOG_DIR}/ros"
export ROS_LOG_DIR="${NAV_LOG_DIR}/ros"
export ROS2CLI_DISABLE_DAEMON="${ROS2CLI_DISABLE_DAEMON:-1}"
exec > >(tee -a "${NAV_LAUNCH_LOG}") 2>&1

cleanup() {
    local exit_code=$?
    if [[ "${CLEANUP_DONE}" == "1" ]]; then
        return "${exit_code}"
    fi
    CLEANUP_DONE=1

    if [[ -n "${BAG_RECORDER_PID}" ]]; then
        kill "${BAG_RECORDER_PID}" 2>/dev/null || true
        wait "${BAG_RECORDER_PID}" 2>/dev/null || true
    fi
    if [[ -n "${DEBUG_RECORDER_PID}" ]]; then
        kill "${DEBUG_RECORDER_PID}" 2>/dev/null || true
        wait "${DEBUG_RECORDER_PID}" 2>/dev/null || true
    fi

    echo "正在保存板端传感器日志尾部..."
    ssh -o ConnectTimeout=3 "root@${BOARD_IP}" "
        echo '--- /tmp/lidar_stream.log ---'
        tail -n 400 /tmp/lidar_stream.log 2>/dev/null || true
        echo
        echo '--- /tmp/odom_stream.log ---'
        tail -n 400 /tmp/odom_stream.log 2>/dev/null || true
        echo
        echo '--- /tmp/ips200_camera_display.log ---'
        tail -n 120 /tmp/ips200_camera_display.log 2>/dev/null || true
    " > "${NAV_BOARD_LOG}" 2>&1 || true
    echo "导航日志已保存：${NAV_LOG_DIR}"
    return "${exit_code}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_debug_recorder() {
    (
        set +e
        echo "# navigation debug topics started at $(date --iso-8601=seconds)"
        sleep 4
        while true; do
            echo
            echo "===== $(date --iso-8601=seconds) ====="
            for topic in \
                /planner/status \
                /navigation/status \
                /navigation/localization_status \
                /navigation/obstacle_status \
                /navigation/localization_ok \
                /navigation/path_blocked \
                /drive/status \
                /cmd_vel \
                /goal_pose; do
                echo "--- ${topic} ---"
                timeout 2 ros2 topic echo --full-length --once \
                    "${topic}" 2>&1 || \
                    timeout 2 ros2 topic echo --once "${topic}" 2>&1 || true
            done
            echo "--- tf map -> base_link ---"
            timeout 2 ros2 run tf2_ros tf2_echo map base_link 2>&1 || true
            sleep 1
        done
    ) >> "${NAV_TOPIC_LOG}" 2>&1 &
    DEBUG_RECORDER_PID=$!
}

start_bag_recorder() {
    if [[ "${NAV_RECORD_BAG}" != "1" ]]; then
        return
    fi
    (
        set +e
        echo "# rosbag recorder requested at $(date --iso-8601=seconds)"
        if ! ros2 bag --help >/dev/null 2>&1; then
            echo "ros2 bag command is unavailable; skipping rosbag capture."
            exit 0
        fi
        for _ in $(seq 1 60); do
            ros2 topic list 2>/dev/null | grep -Eq \
                '^(/planner/status|/navigation/status|/odom)$' && break
            sleep 1
        done
        ros2 bag record \
            -o "${NAV_LOG_DIR}/rosbag" \
            /goal_pose \
            /navigation/replan \
            /planned_path \
            /planner/status \
            /navigation/status \
            /navigation/localization_status \
            /navigation/obstacle_status \
            /navigation/localization_ok \
            /navigation/path_blocked \
            /cmd_vel \
            /drive/status \
            /odom \
            /tf &
        record_pid=$!
        trap 'kill "${record_pid}" 2>/dev/null || true; wait "${record_pid}" 2>/dev/null || true' EXIT INT TERM
        wait "${record_pid}"
    ) >> "${NAV_BAG_LOG}" 2>&1 &
    BAG_RECORDER_PID=$!
}

set +u
source /opt/ros/humble/setup.bash
set -u

if [[ -d "${LOCAL_ROS_PREFIX}/share/ament_index" ]]; then
    export AMENT_PREFIX_PATH="${LOCAL_ROS_PREFIX}:${AMENT_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="${LOCAL_ROS_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
    export PATH="${LOCAL_ROS_PREFIX}/bin:${PATH}"
fi

if [[ "${NAV_LOCALIZER}" != "slam" && "${NAV_LOCALIZER}" != "amcl" ]]; then
    echo "NAV_LOCALIZER must be slam or amcl"
    exit 1
fi
if [[ "${NAV_LOCALIZER}" == "amcl" ]] && \
   ! ros2 pkg prefix nav2_amcl >/dev/null 2>&1; then
    echo "缺少 AMCL 定位依赖，请先执行：./scripts/install_localization_deps.sh"
    exit 1
fi
if ! ros2 pkg prefix nav2_map_server >/dev/null 2>&1; then
    echo "缺少 map_server 定位依赖，请先执行：./scripts/install_localization_deps.sh"
    exit 1
fi
if [[ "${NAV_LOCALIZER}" == "slam" ]] && \
   ! ros2 pkg prefix slam_toolbox >/dev/null 2>&1; then
    echo "缺少 slam_toolbox，请先安装 ROS2 slam_toolbox"
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

echo "正在启动目标点导航，地图：${MAP_FILE}"
echo "定位后端：${NAV_LOCALIZER}"
echo "预检期间请保持小车静止。"
echo "Web 上位机启动后访问：http://127.0.0.1:${DASHBOARD_PORT}"
echo "导航日志目录：${NAV_LOG_DIR}"
echo "完整 launch 输出：${NAV_LAUNCH_LOG}"
echo "关键 topic 采样：${NAV_TOPIC_LOG}"
if [[ "${NAV_RECORD_BAG}" == "1" ]]; then
    echo "rosbag 记录已开启：${NAV_LOG_DIR}/rosbag"
else
    echo "如需完整 topic rosbag，可用 NAV_RECORD_BAG=1 启动。"
fi
pkill -f '/robot_lidar_bridge/(tcp_laser_scan|tcp_odometry|ackermann_path_planner|goal_navigator|navigation_safety|navigation_dashboard|mapping_guard|occupancy_grid_cells)' 2>/dev/null || true
pkill -f '/(rviz2/rviz2|slam_toolbox/.*slam_toolbox|nav2_amcl/amcl|nav2_map_server/map_server)' 2>/dev/null || true
sleep 1
if [[ "${ENABLE_IPS200_DISPLAY}" == "true" ]]; then
    echo "正在启动板端 IPS200 导航摄像头显示。"
fi
ssh -o ConnectTimeout=5 "root@${BOARD_IP}" "
    if [ '${ENABLE_IPS200_DISPLAY}' = 'true' ]; then
        if [ -x '${IPS200_DISPLAY_APP}' ]; then
            killall '${IPS200_DISPLAY_BIN}' 2>/dev/null || true
            nohup '${IPS200_DISPLAY_APP}' --camera >/tmp/ips200_camera_display.log 2>&1 </dev/null &
        else
            echo '未找到 IPS200 显示程序：${IPS200_DISPLAY_APP}' >&2
        fi
    fi
    killall robot_board_app 2>/dev/null || true
    sleep 1
    cd '${BOARD_DIR}'
    nohup ./robot_board_app --test lidar-stream >/tmp/lidar_stream.log 2>&1 </dev/null &
    nohup ./robot_board_app --test mapping-drive >/tmp/odom_stream.log 2>&1 </dev/null &
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
start_debug_recorder
start_bag_recorder
ros2 launch robot_lidar_bridge navigation.launch.py \
    map_yaml:="${MAP_FILE}.yaml" \
    map_file:="${MAP_FILE}" \
    localizer:="${NAV_LOCALIZER}" \
    board_ip:="${BOARD_IP}" \
    lidar_port:="${LIDAR_PORT}" \
    odom_port:="${ODOM_PORT}" \
    dashboard_port:="${DASHBOARD_PORT}"
