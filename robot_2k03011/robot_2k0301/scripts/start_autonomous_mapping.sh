#!/bin/bash
set -eo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ENABLE_DRIVE=true
export ENABLE_AUTONOMOUS_EXPLORATION=true
export EXPLORATION_RADIUS_M="${EXPLORATION_RADIUS_M:-2.0}"
export OBSTACLE_STOP_DISTANCE_M="${OBSTACLE_STOP_DISTANCE_M:-0.35}"
export OBSTACLE_SLOW_DISTANCE_M="${OBSTACLE_SLOW_DISTANCE_M:-0.55}"

echo "启动空白地图自主探索，边界半径 ${EXPLORATION_RADIUS_M}m。"
echo "障碍减速/停车距离：${OBSTACLE_SLOW_DISTANCE_M}m / ${OBSTACLE_STOP_DISTANCE_M}m。"
echo "请确认小车位于封闭环境、周围无人员，并确保实体急停可用。"
exec "${PROJECT_ROOT}/scripts/start_mapping.sh"
