#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build/host"

cmake -S "${PROJECT_ROOT}" -B "${BUILD_DIR}" -DROBOT_USE_LS2K0301_LIBRARY=OFF
cmake --build "${BUILD_DIR}"

echo "host build done: ${BUILD_DIR}/board_app/robot_board_app"

