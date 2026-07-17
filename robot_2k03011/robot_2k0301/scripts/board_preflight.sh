#!/bin/sh
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROBOT_CONFIG="${ROBOT_CONFIG:-config/robot_bringup.yaml}"
CONFIG_PATH="${ROOT_DIR}/${ROBOT_CONFIG}"
STRICT_HARDWARE=0
if [ "${1:-}" = "--strict-hardware" ]; then
  STRICT_HARDWARE=1
fi

failures=0
warnings=0
pass() { echo "[PASS] $*"; }
warn() { echo "[WARN] $*"; warnings=$((warnings + 1)); }
fail() { echo "[FAIL] $*"; failures=$((failures + 1)); }

check_file() {
  if [ -f "$1" ]; then pass "file $1"; else fail "missing file $1"; fi
}

check_executable() {
  if [ -x "$1" ]; then pass "executable $1"; else fail "missing executable $1"; fi
}

check_device() {
  local path="$1"
  local label="$2"
  if [ -e "${path}" ]; then
    pass "${label}: ${path}"
  elif [ "${STRICT_HARDWARE}" -eq 1 ]; then
    fail "${label} missing: ${path}"
  else
    warn "${label} missing: ${path}"
  fi
}

config_value() {
  local key="$1"
  awk -F: -v wanted="${key}" '
    $1 ~ "^[[:space:]]*" wanted "[[:space:]]*$" {
      sub(/^[[:space:]]+/, "", $2); sub(/[[:space:]]+$/, "", $2); print $2; exit
    }' "${CONFIG_PATH}"
}

echo "LS2K0301 board preflight (read-only)"
echo "root=${ROOT_DIR} arch=$(uname -m)"
if [ "$(uname -m)" = "loongarch64" ]; then
  pass "running on LoongArch64"
else
  warn "current architecture is $(uname -m), not the target board"
fi

for binary in robot_board_app robot_board_navigation robot_board_services; do
  check_executable "${ROOT_DIR}/${binary}"
done
check_file "${CONFIG_PATH}"
check_file "${ROOT_DIR}/maps/library.yaml"
check_file "${ROOT_DIR}/maps/library.pgm"

if [ -f "${CONFIG_PATH}" ]; then
  check_device "$(config_value left_motor_pwm)" "left motor PWM"
  check_device "$(config_value right_motor_pwm)" "right motor PWM"
  check_device "$(config_value left_motor_dir)" "left motor direction GPIO"
  check_device "$(config_value right_motor_dir)" "right motor direction GPIO"
  check_device "$(config_value left_encoder)" "left encoder"
  check_device "$(config_value right_encoder)" "right encoder"
  check_device "$(config_value steering_servo_pwm)" "steering servo PWM"
  check_device "$(config_value beep_gpio)" "beeper GPIO"
  check_device "$(config_value lidar_serial)" "lidar serial"
fi

check_device "${ROBOT_VOICE_PORT:-/dev/ttyS1}" "CI1302 serial"
check_device "${ROBOT_ADC_RAW:-/sys/bus/iio/devices/iio:device0/in_voltage3_raw}" "MQ-2 ADC raw"
check_device "${ROBOT_ADC_SCALE:-/sys/bus/iio/devices/iio:device0/in_voltage_scale}" "MQ-2 ADC scale"

for binary in robot_board_app robot_board_navigation robot_board_services; do
  if [ -x "${ROOT_DIR}/${binary}" ] && command -v ldd >/dev/null 2>&1; then
    missing="$(ldd "${ROOT_DIR}/${binary}" 2>/dev/null | awk '/not found/ {print $1}' | tr '\n' ' ')"
    if [ -n "${missing}" ]; then fail "${binary} missing shared libraries: ${missing}"; else pass "${binary} shared libraries"; fi
  fi
done

if command -v ss >/dev/null 2>&1; then
  for port in 2368 2369 2380 2381 5000; do
    if ss -ltn "sport = :${port}" 2>/dev/null | tail -n +2 | grep -q .; then
      warn "TCP port ${port} is already in use"
    else
      pass "TCP port ${port} available"
    fi
  done
fi

echo "preflight complete: failures=${failures} warnings=${warnings}"
exit "${failures}"
