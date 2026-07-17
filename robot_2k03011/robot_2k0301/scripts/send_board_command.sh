#!/bin/sh
set -eu

if [ "$#" -lt 2 ]; then
  echo "Usage: $0 BOARD_IP MISSION_ID|CANCEL|E_STOP"
  exit 2
fi
BOARD_IP="$1"
COMMAND="$2"
PORT="${ROBOT_COMMAND_PORT:-2381}"
if [ "${COMMAND}" != "CANCEL" ] && [ "${COMMAND}" != "E_STOP" ]; then
  COMMAND="MISSION ${COMMAND}"
  if [ -n "${ROBOT_COMMAND_TOKEN:-}" ]; then
    COMMAND="${ROBOT_COMMAND_TOKEN} ${COMMAND}"
  fi
fi
echo "${COMMAND}" | nc -w 2 "${BOARD_IP}" "${PORT}"
