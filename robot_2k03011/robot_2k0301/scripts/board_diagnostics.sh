#!/bin/sh
set -eu

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG="${ROBOT_CONFIG:-config/robot_bringup.yaml}"
cd "${ROOT_DIR}"

echo "Running non-driving board diagnostics. Keep drive wheels raised."
./scripts/board_preflight.sh
./robot_board_services --self-test
./robot_board_navigation --self-test

echo "Reading raw encoders for 5 seconds; rotate each wheel by hand."
./robot_board_app --config "${CONFIG}" --test encoder-raw --seconds 5

echo "Reading lidar for 5 seconds."
./robot_board_app --config "${CONFIG}" --test lidar --seconds 5

echo "Non-driving diagnostics completed. No non-zero motor command was issued."
