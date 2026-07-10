#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build/ls2k0301"
TOOLCHAIN_FILE="${PROJECT_ROOT}/third_party/LS2K0301_Library/LS2K030x_Library/Seekfree_LS2K030x_Opensource_Library/project/user/cross.cmake"

if [ ! -f "${TOOLCHAIN_FILE}" ]; then
  echo "LS2K0301 official library is missing. Run ./scripts/fetch_ls2k0301_library.sh first."
  exit 1
fi

cmake -S "${PROJECT_ROOT}" -B "${BUILD_DIR}" \
  -DROBOT_USE_LS2K0301_LIBRARY=ON \
  -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}"

cmake --build "${BUILD_DIR}"

echo "LS2K0301 build done: ${BUILD_DIR}/board_app/robot_board_app"
