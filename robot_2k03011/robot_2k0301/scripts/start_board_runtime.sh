#!/bin/sh
set -eu

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG="${ROBOT_CONFIG:-config/robot_bringup.yaml}"
ARMED=0
if [ "${1:-}" = "--arm" ]; then
  ARMED=1
  shift
fi

cd "${ROOT_DIR}"
./scripts/board_preflight.sh

set -- \
  --voice-port "${ROBOT_VOICE_PORT:-/dev/ttyS1}" \
  --adc-raw "${ROBOT_ADC_RAW:-/sys/bus/iio/devices/iio:device0/in_voltage3_raw}" \
  --adc-scale "${ROBOT_ADC_SCALE:-/sys/bus/iio/devices/iio:device0/in_voltage_scale}" \
  --board-app ./robot_board_app \
  --vision-app /bin/false \
  --robot-config "${CONFIG}" \
  --map maps/library.yaml \
  --camera /dev/null \
  --model /dev/null \
  --metadata /dev/null \
  --journal "${ROBOT_EVENT_LOG:-/root/robot_board_events.jsonl}" \
  --start-x "${ROBOT_START_X:-0}" \
  --start-y "${ROBOT_START_Y:-0}" \
  --start-yaw "${ROBOT_START_YAW:-0}" \
  "$@"

if [ -n "${ROBOT_COMMAND_TOKEN:-}" ]; then
  set -- "$@" --command-token "${ROBOT_COMMAND_TOKEN}"
fi

if [ "${ARMED}" -eq 1 ]; then
  echo "WARNING: board actions armed; keep wheels raised and physical emergency stop ready."
  set -- "$@" --execute-actions
  if [ -z "${ROBOT_COMMAND_TOKEN:-}" ]; then
    echo "Remote mission starts are disabled because ROBOT_COMMAND_TOKEN is empty."
    echo "CANCEL and E_STOP remain available without a token."
  fi
else
  echo "SAFE MODE: tasks are visible but navigation/vision child actions are not executed."
  echo "Use ./scripts/start_board_runtime.sh --arm only after individual hardware checks pass."
fi
exec ./robot_board_services "$@"
