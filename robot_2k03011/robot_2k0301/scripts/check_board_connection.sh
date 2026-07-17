#!/usr/bin/env bash
set -u

BOARD_IP="${1:-${BOARD_IP:-192.168.43.192}}"
echo "board connection check: ${BOARD_IP}"
echo "host virtualization: $(systemd-detect-virt 2>/dev/null || echo unknown)"
ip -brief address
echo "route: $(ip route get "${BOARD_IP}" 2>/dev/null | head -1)"

host_subnet="$(ip -4 -brief address | awk '$1 != "lo" {print $3; exit}')"
board_prefix="$(echo "${BOARD_IP}" | awk -F. '{print $1 "." $2 "." $3 "."}')"
case "${host_subnet}" in
  "${board_prefix}"*) ;;
  *) echo "[WARN] host is not directly on the board hotspot subnet ${board_prefix}0/24 (${host_subnet:-none})" ;;
esac

if ping -c 1 -W 1 "${BOARD_IP}" >/dev/null 2>&1; then
  echo "[PASS] ping"
else
  echo "[FAIL] ping"
fi
if nc -z -w 2 "${BOARD_IP}" 22 >/dev/null 2>&1; then
  echo "[PASS] SSH port 22"
else
  echo "[FAIL] SSH port 22"
fi
if [ -e /dev/ttyACM0 ]; then
  echo "[PASS] serial console /dev/ttyACM0 exists"
else
  echo "[WARN] serial console /dev/ttyACM0 is absent"
fi

if systemd-detect-virt 2>/dev/null | grep -q vmware &&
   ! ip -4 -brief address | grep -Fq "${board_prefix}"; then
  echo "[ACTION] VMware is using a different subnet. Use bridged networking to the adapter connected to the phone hotspot,"
  echo "         or verify that VMware NAT can route to the hotspot before changing the board."
fi
