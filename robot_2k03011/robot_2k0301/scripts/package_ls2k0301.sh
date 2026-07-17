#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build/ls2k0301"
PACKAGE_DIR="${BUILD_DIR}/package/robot_2k0301"
ARCHIVE="${BUILD_DIR}/robot_2k0301-board.tar.gz"

"${SCRIPT_DIR}/build_ls2k0301.sh"
cmake -E remove_directory "${PACKAGE_DIR}"
install -d "${PACKAGE_DIR}/config" "${PACKAGE_DIR}/maps" "${PACKAGE_DIR}/scripts"
install -m 0755 \
  "${BUILD_DIR}/board_app/robot_board_app" \
  "${BUILD_DIR}/board_app/robot_board_navigation" \
  "${BUILD_DIR}/board_app/robot_board_services" \
  "${PACKAGE_DIR}/"
install -m 0644 "${PROJECT_ROOT}/config/robot.yaml" \
  "${PROJECT_ROOT}/config/robot_bringup.yaml" "${PACKAGE_DIR}/config/"
install -m 0644 "${PROJECT_ROOT}/maps/library.yaml" "${PROJECT_ROOT}/maps/library.pgm" "${PACKAGE_DIR}/maps/"
install -m 0755 "${SCRIPT_DIR}/board_preflight.sh" "${SCRIPT_DIR}/board_diagnostics.sh" \
  "${SCRIPT_DIR}/start_board_runtime.sh" "${SCRIPT_DIR}/send_board_command.sh" \
  "${SCRIPT_DIR}/board_network_diagnose.sh" "${SCRIPT_DIR}/install_board_autonetwork.sh" \
  "${PACKAGE_DIR}/scripts/"
tar -C "${BUILD_DIR}/package" -czf "${ARCHIVE}" robot_2k0301
echo "board package ready: ${ARCHIVE}"
