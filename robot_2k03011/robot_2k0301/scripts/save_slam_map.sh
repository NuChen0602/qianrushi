#!/bin/bash
set -eo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_FILE="${1:-${PROJECT_ROOT}/maps/library}"
EXPECTED_MAP_WIDTH_M="${EXPECTED_MAP_WIDTH_M:-1.8}"
EXPECTED_MAP_HEIGHT_M="${EXPECTED_MAP_HEIGHT_M:-1.8}"
MAP_SIZE_TOLERANCE_M="${MAP_SIZE_TOLERANCE_M:-0.20}"
MAP_MAX_OUTER_EXCESS_M="${MAP_MAX_OUTER_EXCESS_M:-0.60}"
MAP_MAX_UNKNOWN_RATIO="${MAP_MAX_UNKNOWN_RATIO:-0.50}"
mkdir -p "$(dirname "${MAP_FILE}")"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${PROJECT_ROOT}/ros2_ws/install/setup.bash" ]]; then
    source "${PROJECT_ROOT}/ros2_ws/install/setup.bash"
fi
set -u

if ! timeout 10 ros2 service type /slam_toolbox/serialize_map >/dev/null 2>&1; then
    echo "SLAM 服务未就绪，地图未保存。请确认建图程序仍在运行。"
    exit 1
fi
if ! timeout 10 ros2 service type /slam_toolbox/save_map >/dev/null 2>&1; then
    echo "地图保存服务未就绪，地图未保存。"
    exit 1
fi

BACKUP_SUFFIX="$(date +%Y%m%d_%H%M%S)"
for suffix in posegraph data yaml pgm; do
    if [[ -s "${MAP_FILE}.${suffix}" ]]; then
        cp -p "${MAP_FILE}.${suffix}" "${MAP_FILE}.${suffix}.${BACKUP_SUFFIX}.bak"
    fi
done

restore_previous_map() {
    echo "地图质量检查失败，保留本次结果并恢复保存前地图。"
    for suffix in posegraph data yaml pgm; do
        if [[ -s "${MAP_FILE}.${suffix}" ]]; then
            mv "${MAP_FILE}.${suffix}" \
                "${MAP_FILE}.${suffix}.${BACKUP_SUFFIX}.rejected"
        fi
        if [[ -s "${MAP_FILE}.${suffix}.${BACKUP_SUFFIX}.bak" ]]; then
            cp -p "${MAP_FILE}.${suffix}.${BACKUP_SUFFIX}.bak" \
                "${MAP_FILE}.${suffix}"
        else
            rm -f "${MAP_FILE}.${suffix}"
        fi
    done
    echo "失败地图：${MAP_FILE}.*.${BACKUP_SUFFIX}.rejected"
}

if ! timeout 30 ros2 service call \
    /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '${MAP_FILE}'}"; then
    echo "位姿图保存服务调用失败。"
    restore_previous_map
    exit 1
fi
if ! timeout 30 ros2 service call \
    /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '${MAP_FILE}'}}"; then
    echo "栅格地图保存服务调用失败。"
    restore_previous_map
    exit 1
fi

if [[ ! -s "${MAP_FILE}.posegraph" ]]; then
    echo "位姿图文件未生成或为空：${MAP_FILE}.posegraph"
    restore_previous_map
    exit 1
fi
if [[ ! -s "${MAP_FILE}.yaml" || ! -s "${MAP_FILE}.pgm" ]]; then
    echo "栅格地图文件未完整生成：${MAP_FILE}.yaml / ${MAP_FILE}.pgm"
    restore_previous_map
    exit 1
fi

if ! python3 "${PROJECT_ROOT}/scripts/check_map_quality.py" \
    --map-yaml "${MAP_FILE}.yaml" \
    --expected-width "${EXPECTED_MAP_WIDTH_M}" \
    --expected-height "${EXPECTED_MAP_HEIGHT_M}" \
    --size-tolerance "${MAP_SIZE_TOLERANCE_M}" \
    --max-outer-excess "${MAP_MAX_OUTER_EXCESS_M}" \
    --max-unknown-ratio "${MAP_MAX_UNKNOWN_RATIO}"; then
    restore_previous_map
    exit 1
fi

echo "SLAM pose graph saved: ${MAP_FILE}.posegraph"
echo "Occupancy map saved: ${MAP_FILE}.yaml and ${MAP_FILE}.pgm"
